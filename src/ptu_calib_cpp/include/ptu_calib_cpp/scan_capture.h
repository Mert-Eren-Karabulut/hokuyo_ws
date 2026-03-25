#pragma once
// =========================================================================
//  ptu_calib_cpp/scan_capture.h
//
//  Data structures for captured laser scans + helper I/O (CSV).
//  Each capture stores:
//      - commanded & actual joint angles
//      - 2D inlier points (in the laser frame) after FOV extraction + RANSAC
//      - fitted line direction & mean
//
//  A capture set can be saved/loaded as a simple CSV so the solver can
//  run offline without ROS.
// =========================================================================

#include <Eigen/Core>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <stdexcept>
#include <cmath>

namespace ptu_calib {

struct ScanCapture {
    double pan_rad  = 0.0;   // actual pan  encoder reading at capture time
    double tilt_rad = 0.0;   // actual tilt encoder reading at capture time
    Eigen::Vector2d line_dir  = Eigen::Vector2d::Zero();  // RANSAC line direction (unit)
    Eigen::Vector2d line_mean = Eigen::Vector2d::Zero();  // RANSAC line centroid
    std::vector<Eigen::Vector2d> inlier_pts;              // inlier 2D points (laser frame)
    double timestamp = 0.0;
};

/// A collection of captures with metadata.
struct CaptureSet {
    std::vector<ScanCapture> captures;
    double fov_half_deg = 15.0;
    std::string label;  // e.g. "pan_sweep" or "tilt_sweep"
};

// ──────────────────────── CSV I/O ───────────────────────────────────

/// File format (one line per inlier point):
///   capture_idx, pan_rad, tilt_rad, ldir_x, ldir_y, lmean_x, lmean_y, pt_x, pt_y
///
/// The first line is a header.  Captures with the same index are grouped.

inline bool save_captures_csv(const CaptureSet& cs, const std::string& path)
{
    std::ofstream f(path);
    if (!f.is_open()) return false;
    f << std::fixed << std::setprecision(8);
    f << "# label=" << cs.label << "  fov_half_deg=" << cs.fov_half_deg << "\n";
    f << "capture_idx,pan_rad,tilt_rad,ldir_x,ldir_y,lmean_x,lmean_y,pt_x,pt_y\n";
    for (size_t ci = 0; ci < cs.captures.size(); ++ci) {
        const auto& cap = cs.captures[ci];
        for (const auto& pt : cap.inlier_pts) {
            f << ci << ","
              << cap.pan_rad  << "," << cap.tilt_rad << ","
              << cap.line_dir.x()  << "," << cap.line_dir.y()  << ","
              << cap.line_mean.x() << "," << cap.line_mean.y() << ","
              << pt.x() << "," << pt.y() << "\n";
        }
    }
    return true;
}

inline CaptureSet load_captures_csv(const std::string& path)
{
    CaptureSet cs;
    std::ifstream f(path);
    if (!f.is_open()) throw std::runtime_error("Cannot open " + path);

    std::string line;
    // Read comment line with metadata
    if (std::getline(f, line)) {
        if (line.size() > 2 && line[0] == '#') {
            auto pos_label = line.find("label=");
            auto pos_fov   = line.find("fov_half_deg=");
            if (pos_label != std::string::npos) {
                size_t start = pos_label + 6;
                size_t end = line.find("  ", start);
                cs.label = line.substr(start, (end == std::string::npos) ? std::string::npos : end - start);
            }
            if (pos_fov != std::string::npos) {
                cs.fov_half_deg = std::stod(line.substr(pos_fov + 13));
            }
        }
    }
    // Skip header
    std::getline(f, line);

    // Read data
    int prev_idx = -1;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ss(line);
        std::string token;
        auto next = [&]() -> double {
            std::getline(ss, token, ',');
            return std::stod(token);
        };
        int idx        = static_cast<int>(next());
        double pan     = next();
        double tilt    = next();
        double ldx     = next();
        double ldy     = next();
        double lmx     = next();
        double lmy     = next();
        double px      = next();
        double py      = next();

        if (idx != prev_idx) {
            cs.captures.emplace_back();
            cs.captures.back().pan_rad  = pan;
            cs.captures.back().tilt_rad = tilt;
            cs.captures.back().line_dir  = Eigen::Vector2d(ldx, ldy);
            cs.captures.back().line_mean = Eigen::Vector2d(lmx, lmy);
            prev_idx = idx;
        }
        cs.captures.back().inlier_pts.emplace_back(px, py);
    }
    return cs;
}

}  // namespace ptu_calib
