#include "simulator/map_loader.hpp"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <stdexcept>
#include <cmath>

// ---------------------------------------------------------------------------
void MapData::worldToPixel(double wx, double wy, int& col, int& row) const {
    col = static_cast<int>((wx - origin_x) / resolution);
    row = height - 1 - static_cast<int>((wy - origin_y) / resolution);
}

void MapData::pixelToWorld(int col, int row, double& wx, double& wy) const {
    wx = col * resolution + origin_x;
    wy = (height - 1 - row) * resolution + origin_y;
}

bool MapData::isOccupied(double wx, double wy) const {
    if (!isValid()) return false;
    int col, row;
    worldToPixel(wx, wy, col, row);
    if (col < 0 || col >= width || row < 0 || row >= height) return true;
    return occupied[static_cast<size_t>(row) * static_cast<size_t>(width) + static_cast<size_t>(col)];
}

bool MapData::isCircleOccupied(double cx, double cy, double radius, int samples) const {
    if (!isValid()) return false;
    if (isOccupied(cx, cy)) return true;
    constexpr double pi2 = 2.0 * M_PI;
    for (int i = 0; i < samples; ++i) {
        double a = pi2 * i / samples;
        if (isOccupied(cx + radius * std::cos(a), cy + radius * std::sin(a))) return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// Ray-cast point-in-polygon (Jordan curve theorem)
static bool pointInPolygon(double px, double py,
                            const std::vector<std::pair<double,double>>& poly)
{
    bool inside = false;
    const int n = static_cast<int>(poly.size());
    for (int i = 0, j = n - 1; i < n; j = i++) {
        double xi = poly[i].first,  yi = poly[i].second;
        double xj = poly[j].first,  yj = poly[j].second;
        if (((yi > py) != (yj > py)) &&
            (px < (xj - xi) * (py - yi) / (yj - yi) + xi))
            inside = !inside;
    }
    return inside;
}

bool MapData::isPolygonOccupied(const std::vector<std::pair<double,double>>& poly) const {
    if (!isValid() || poly.size() < 3) return false;

    // World bounding box of polygon
    double xmin = poly[0].first,  xmax = xmin;
    double ymin = poly[0].second, ymax = ymin;
    for (const auto& [px, py] : poly) {
        xmin = std::min(xmin, px); xmax = std::max(xmax, px);
        ymin = std::min(ymin, py); ymax = std::max(ymax, py);
    }

    // Pixel bounding box (worldToPixel: row = height-1-(wy-oy)/res, so y-flip)
    int c0, r0, c1, r1;
    worldToPixel(xmin, ymin, c0, r0);
    worldToPixel(xmax, ymax, c1, r1);
    if (r0 > r1) std::swap(r0, r1);
    if (c0 > c1) std::swap(c0, c1);
    c0 = std::max(0, c0 - 1); c1 = std::min(width  - 1, c1 + 1);
    r0 = std::max(0, r0 - 1); r1 = std::min(height - 1, r1 + 1);

    // Check every occupied cell whose centre falls inside the polygon
    for (int r = r0; r <= r1; ++r) {
        for (int c = c0; c <= c1; ++c) {
            if (!occupied[static_cast<size_t>(r) * static_cast<size_t>(width) +
                          static_cast<size_t>(c)]) continue;
            double wx, wy;
            pixelToWorld(c, r, wx, wy);
            if (pointInPolygon(wx, wy, poly)) return true;
        }
    }

    // Also sample along each edge to catch thin walls the cell-centre check misses
    constexpr int kEdgeSamples = 8;
    for (size_t i = 0; i < poly.size(); ++i) {
        size_t j = (i + 1) % poly.size();
        for (int s = 0; s <= kEdgeSamples; ++s) {
            double t  = static_cast<double>(s) / kEdgeSamples;
            double wx = poly[i].first  + t * (poly[j].first  - poly[i].first);
            double wy = poly[i].second + t * (poly[j].second - poly[i].second);
            if (isOccupied(wx, wy)) return true;
        }
    }

    return false;
}

// ---------------------------------------------------------------------------
MapData loadMap(const std::string& yaml_path) {
    MapData data;

    YAML::Node cfg;
    try {
        cfg = YAML::LoadFile(yaml_path);
    } catch (const std::exception& e) {
        throw std::runtime_error(std::string("Failed to parse map YAML: ") + e.what());
    }

    data.resolution = cfg["resolution"].as<double>(0.05);

    auto origin_vec = cfg["origin"].as<std::vector<double>>(std::vector<double>{0.0, 0.0, 0.0});
    data.origin_x = origin_vec.size() > 0 ? origin_vec[0] : 0.0;
    data.origin_y = origin_vec.size() > 1 ? origin_vec[1] : 0.0;

    std::string image_file = cfg["image"].as<std::string>("");
    if (!std::filesystem::path(image_file).is_absolute()) {
        image_file = (std::filesystem::path(yaml_path).parent_path() / image_file).string();
    }

    // Load image via Qt (supports pgm, png, jpg …)
    data.image = QImage(QString::fromStdString(image_file));
    if (data.image.isNull()) {
        throw std::runtime_error("Failed to load map image: " + image_file);
    }

    // Build occupancy grid
    QImage gray = data.image.convertToFormat(QImage::Format_Grayscale8);
    data.width  = gray.width();
    data.height = gray.height();
    data.occupied.assign(static_cast<size_t>(data.width) * static_cast<size_t>(data.height), false);

    double occupied_thresh = cfg["occupied_thresh"].as<double>(0.65);
    bool   negate          = cfg["negate"].as<bool>(false);

    for (int row = 0; row < data.height; ++row) {
        const uchar* line = gray.constScanLine(row);
        for (int col = 0; col < data.width; ++col) {
            double pixel = line[col] / 255.0;           // 0 = black, 1 = white
            double p_occ = negate ? pixel : (1.0 - pixel);
            data.occupied[static_cast<size_t>(row) * static_cast<size_t>(data.width) +
                          static_cast<size_t>(col)] = (p_occ > occupied_thresh);
        }
    }

    return data;
}
