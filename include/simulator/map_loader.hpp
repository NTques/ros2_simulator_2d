#pragma once
#include <string>
#include <vector>
#include <QImage>

// ---------------------------------------------------------------------------
// MapData
//   World frame : x-right, y-up  (ROS convention)
//   Image/Scene : x-right, y-down (Qt / PGM convention)
//
//   world -> pixel:
//     col = (wx - origin_x) / resolution
//     row = height - 1 - (wy - origin_y) / resolution
// ---------------------------------------------------------------------------
struct MapData {
    double resolution = 0.05;   // metres per pixel
    double origin_x   = 0.0;
    double origin_y   = 0.0;
    int    width      = 0;      // pixels
    int    height     = 0;      // pixels
    std::vector<bool> occupied; // [row * width + col] true = occupied

    QImage image;               // for Qt display (original colours)

    bool isValid() const { return width > 0 && height > 0; }

    // World (m) → pixel
    void worldToPixel(double wx, double wy, int& col, int& row) const;
    // Pixel → world (m)
    void pixelToWorld(int col, int row, double& wx, double& wy) const;

    // Is world point occupied? (out-of-bounds → occupied)
    bool isOccupied(double wx, double wy) const;

    // Is a circle (robot footprint) overlapping any occupied cell?
    bool isCircleOccupied(double cx, double cy, double radius,
                          int samples = 16) const;

    // Is a polygon (world-frame, CCW) overlapping any occupied cell?
    // Uses bounding-box cell sweep + point-in-polygon + edge sampling.
    bool isPolygonOccupied(const std::vector<std::pair<double,double>>& poly) const;
};

MapData loadMap(const std::string& yaml_path);
