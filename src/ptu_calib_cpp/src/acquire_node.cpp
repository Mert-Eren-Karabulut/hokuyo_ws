// =========================================================================
//  ptu_calib_cpp  –  acquire_node.cpp
//
//  Automated grid-scan data acquisition for pan-tilt + planar-LiDAR
//  calibration.  Drives the PTU through user-specified 1-D sweeps:
//
//      mode = "pan"   → sweep pan with tilt SPREAD across steps (pan data)
//                       (tilt is varied so laser strips define a plane,
//                        not a degenerate line from horizontal-only scans)
//      mode = "tilt"  → sweep tilt with pan locked at 0  (tilt-only data)
//      mode = "both"  → sweep both (full grid, for validation)
//
//  For each pose the node:
//      1. Commands the PTU via /joint_command  (JointState)
//      2. Waits for convergence + settle time
//      3. Grabs /scan, extracts the forward-facing FOV
//      4. Fits a line via RANSAC
//      5. Stores the capture
//
//  Outputs a CSV file that the solver can read offline.
// =========================================================================

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>

#include <Eigen/Core>
#include <Eigen/SVD>

#include "ptu_calib_cpp/scan_capture.h"
#include "ptu_calib_cpp/kinematic_model.h"

#include <mutex>
#include <atomic>
#include <cmath>
#include <random>
#include <algorithm>
#include <numeric>
#include <limits>
#include <sys/stat.h>
#include <boost/make_shared.hpp>

using namespace ptu_calib;

// =====================================================================
//  AcquireNode
// =====================================================================
class AcquireNode {
public:
    AcquireNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh), pnh_(pnh)
    {
        // ---------- parameters ----------
        pnh_.param<std::string>("mode", mode_, "pan");  // "pan", "tilt", or "both"
        pnh_.param("pan_lower_deg",  pan_lower_deg_,  -25.0);
        pnh_.param("pan_upper_deg",  pan_upper_deg_,   25.0);
        pnh_.param("tilt_lower_deg", tilt_lower_deg_, -15.0);
        pnh_.param("tilt_upper_deg", tilt_upper_deg_,  15.0);
        pnh_.param("pan_steps",      pan_steps_,       7);
        pnh_.param("tilt_steps",     tilt_steps_,      7);
        pnh_.param("fov_half_deg",   fov_half_deg_,   15.0);
        pnh_.param("settle_time",    settle_time_,     1.5);
        pnh_.param("position_tolerance_deg", pos_tol_deg_, 1.0);
        pnh_.param("motion_timeout", motion_timeout_,  10.0);
        pnh_.param("ransac_iterations", ransac_iters_, 600);
        pnh_.param("ransac_inlier_thresh_m", ransac_thresh_, 0.015);
        pnh_.param("min_inliers",    min_inliers_,     30);
        pnh_.param("range_min",      range_min_,       0.10);
        pnh_.param("range_max",      range_max_,       4.00);
        pnh_.param("scan_avg_count", scan_avg_count_,  10);
        pnh_.param<std::string>("output_dir", output_dir_, ".");

        // ---------- ROS I/O ----------
        cmd_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_command", 10);
        viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/calibration/points", 5, true);
        scan_sub_ = nh_.subscribe("/scan", 5, &AcquireNode::scanCB, this);
        js_sub_   = nh_.subscribe("/joint_states", 50, &AcquireNode::jsCB, this);
    }

    /// Run the full acquisition and save to CSV. Returns true on success.
    bool run()
    {
        // Wait for data
        ROS_INFO("Waiting for /scan and /joint_states ...");
        ros::Rate wait_rate(10);
        ros::Time deadline = ros::Time::now() + ros::Duration(10.0);
        while (ros::ok()) {
            ros::spinOnce();
            if (have_scan_.load() && have_js_.load()) break;
            if (ros::Time::now() > deadline) {
                ROS_ERROR("Timed out waiting for /scan and /joint_states.");
                return false;
            }
            wait_rate.sleep();
        }

        // Build the grid
        auto grid = generateGrid();
        ROS_INFO("Mode=%s  Grid size=%zu", mode_.c_str(), grid.size());

        // Home first
        goToPose(0.0, 0.0);

        CaptureSet cs;
        cs.fov_half_deg = fov_half_deg_;
        cs.label = mode_;

        for (size_t i = 0; i < grid.size() && ros::ok(); ++i) {
            double pan_deg  = grid[i].first;
            double tilt_deg = grid[i].second;
            ROS_INFO("--- Pose %zu/%zu: pan=%+.1f deg  tilt=%+.1f deg ---",
                     i+1, grid.size(), pan_deg, tilt_deg);

            if (!goToPose(pan_deg, tilt_deg)) {
                ROS_WARN("  Motion timeout – skipping");
                continue;
            }

            ScanCapture cap;
            if (!captureAtCurrentPose(pan_deg, tilt_deg, cap)) {
                ROS_WARN("  Capture failed – skipping");
                continue;
            }
            cs.captures.push_back(std::move(cap));
            publishViz(cs);
            ROS_INFO("  OK: %zu inliers", cs.captures.back().inlier_pts.size());
        }

        // Home
        goToPose(0.0, 0.0);

        ROS_INFO("==============================");
        ROS_INFO("  Acquired %zu / %zu captures", cs.captures.size(), grid.size());
        ROS_INFO("==============================");

        if (cs.captures.size() < 4) {
            ROS_ERROR("Not enough captures (need >= 4).");
            return false;
        }

        // Save
        mkdir(output_dir_.c_str(), 0755);
        // timestamp in filename
        time_t now = time(nullptr);
        char buf[64]; strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", localtime(&now));
        std::string fname = output_dir_ + "/" + mode_ + "_data_" + buf + ".csv";
        if (save_captures_csv(cs, fname)) {
            ROS_INFO("Saved → %s", fname.c_str());
        } else {
            ROS_ERROR("Failed to write %s", fname.c_str());
            return false;
        }
        last_csv_path_ = fname;
        capture_set_ = std::move(cs);
        return true;
    }

    const CaptureSet& captureSet() const { return capture_set_; }
    const std::string& lastCsvPath() const { return last_csv_path_; }

private:
    ros::NodeHandle nh_, pnh_;
    ros::Publisher cmd_pub_, viz_pub_;
    ros::Subscriber scan_sub_, js_sub_;

    // Parameters
    std::string mode_;
    double pan_lower_deg_, pan_upper_deg_, tilt_lower_deg_, tilt_upper_deg_;
    int pan_steps_, tilt_steps_;
    double fov_half_deg_, settle_time_, pos_tol_deg_, motion_timeout_;
    int ransac_iters_, min_inliers_, scan_avg_count_;
    double ransac_thresh_, range_min_, range_max_;
    std::string output_dir_;

    // State
    std::mutex scan_mutex_, js_mutex_;
    sensor_msgs::LaserScan::ConstPtr last_scan_;
    sensor_msgs::JointState::ConstPtr last_js_;
    std::atomic<bool> have_scan_{false}, have_js_{false};
    CaptureSet capture_set_;
    std::string last_csv_path_;

    // ── callbacks ────────────────────────────────────────────────────
    void scanCB(const sensor_msgs::LaserScan::ConstPtr& msg) {
        std::lock_guard<std::mutex> lk(scan_mutex_);
        last_scan_ = msg;
        have_scan_.store(true);
    }
    void jsCB(const sensor_msgs::JointState::ConstPtr& msg) {
        std::lock_guard<std::mutex> lk(js_mutex_);
        last_js_ = msg;
        have_js_.store(true);
    }

    // ── joint reading ────────────────────────────────────────────────
    bool getCurrentJoints(double& tilt, double& pan) {
        std::lock_guard<std::mutex> lk(js_mutex_);
        if (!last_js_) return false;
        tilt = pan = 0.0;
        for (size_t i = 0; i < last_js_->name.size(); ++i) {
            if (last_js_->name[i] == "joint1") tilt = last_js_->position[i];
            if (last_js_->name[i] == "joint2") pan  = last_js_->position[i];
        }
        return true;
    }

    // ── PTU commanding ───────────────────────────────────────────────
    void commandPose(double tilt_rad, double pan_rad) {
        sensor_msgs::JointState msg;
        msg.header.stamp = ros::Time::now();
        msg.name = {"joint1", "joint2"};
        msg.position = {tilt_rad, pan_rad};
        cmd_pub_.publish(msg);
    }

    bool goToPose(double pan_deg, double tilt_deg) {
        double pan_rad  = pan_deg  * M_PI / 180.0;
        double tilt_rad = tilt_deg * M_PI / 180.0;
        double tol_rad  = pos_tol_deg_ * M_PI / 180.0;

        ros::Rate rate(50);
        ros::Time t0 = ros::Time::now();

        while (ros::ok()) {
            commandPose(tilt_rad, pan_rad);
            ros::spinOnce();

            double cur_tilt, cur_pan;
            if (getCurrentJoints(cur_tilt, cur_pan)) {
                if (std::abs(cur_tilt - tilt_rad) <= tol_rad &&
                    std::abs(cur_pan  - pan_rad)  <= tol_rad)
                {
                    // Settle
                    ros::Time settle_end = ros::Time::now() + ros::Duration(settle_time_);
                    while (ros::Time::now() < settle_end && ros::ok()) {
                        commandPose(tilt_rad, pan_rad);
                        ros::spinOnce();
                        rate.sleep();
                    }
                    return true;
                }
            }
            if ((ros::Time::now() - t0).toSec() > motion_timeout_) return false;
            rate.sleep();
        }
        return false;
    }

    // ── grid generation ──────────────────────────────────────────────
    std::vector<std::pair<double,double>> generateGrid() {
        std::vector<double> pans, tilts;

        if (mode_ == "pan" || mode_ == "both") {
            if (pan_steps_ <= 1) pans = {0.0};
            else {
                pans.resize(pan_steps_);
                for (int i = 0; i < pan_steps_; ++i)
                    pans[i] = pan_lower_deg_ + (pan_upper_deg_ - pan_lower_deg_)*i/(pan_steps_-1);
            }
        } else {
            pans = {0.0};  // locked
        }

        if (mode_ == "tilt" || mode_ == "both") {
            if (tilt_steps_ <= 1) tilts = {0.0};
            else {
                tilts.resize(tilt_steps_);
                for (int i = 0; i < tilt_steps_; ++i)
                    tilts[i] = tilt_lower_deg_ + (tilt_upper_deg_ - tilt_lower_deg_)*i/(tilt_steps_-1);
            }
        } else {
            tilts = {0.0};  // locked
        }

        std::vector<std::pair<double,double>> grid;

        if (mode_ == "pan") {
            // Pan mode: assign a different tilt to each pan step so that
            // the captured strips span a plane (not a single line).
            // Spread tilt over [tilt_lower, tilt_upper] across the pan steps.
            for (int i = 0; i < static_cast<int>(pans.size()); ++i) {
                double frac = (pans.size() <= 1) ? 0.0
                            : static_cast<double>(i) / (pans.size() - 1);
                double t = tilt_lower_deg_ + (tilt_upper_deg_ - tilt_lower_deg_) * frac;
                grid.emplace_back(pans[i], t);
            }
            ROS_INFO("Pan mode: tilt spread [%.1f, %.1f] deg across %zu pan steps",
                     tilt_lower_deg_, tilt_upper_deg_, pans.size());
        } else {
            for (double p : pans)
                for (double t : tilts)
                    grid.emplace_back(p, t);
        }
        return grid;
    }

    // ── FOV extraction ───────────────────────────────────────────────
    std::vector<Eigen::Vector2d> extractFOV(const sensor_msgs::LaserScan& scan) {
        double fov_rad = fov_half_deg_ * M_PI / 180.0;
        std::vector<Eigen::Vector2d> pts;
        for (size_t i = 0; i < scan.ranges.size(); ++i) {
            double r = scan.ranges[i];
            if (!std::isfinite(r) || r < range_min_ || r > range_max_) continue;
            double angle = scan.angle_min + i * scan.angle_increment;
            if (std::abs(angle) > fov_rad) continue;
            pts.emplace_back(r * std::cos(angle), r * std::sin(angle));
        }
        return pts;
    }

    // ── RANSAC line fit ──────────────────────────────────────────────
    struct LineResult {
        Eigen::Vector2d dir, mean;
        std::vector<Eigen::Vector2d> inliers;
        bool ok = false;
    };

    LineResult ransacLineFit(const std::vector<Eigen::Vector2d>& pts) {
        LineResult res;
        int N = static_cast<int>(pts.size());
        if (N < 2) return res;

        std::mt19937 rng(42);
        std::uniform_int_distribution<int> dist(0, N-1);

        int best_count = 0;
        Eigen::Vector2d best_p1, best_n;

        for (int iter = 0; iter < ransac_iters_; ++iter) {
            int a = dist(rng), b = dist(rng);
            if (a == b) continue;
            Eigen::Vector2d v = pts[b] - pts[a];
            double nv = v.norm();
            if (nv < 1e-6) continue;
            v /= nv;
            Eigen::Vector2d n(-v.y(), v.x());  // perpendicular

            int count = 0;
            for (int i = 0; i < N; ++i) {
                if (std::abs((pts[i] - pts[a]).dot(n)) <= ransac_thresh_)
                    ++count;
            }
            if (count > best_count) {
                best_count = count;
                best_p1 = pts[a];
                best_n  = n;
                if (best_count > 0.7 * N) break;
            }
        }
        if (best_count < 2) return res;

        // Collect inliers
        for (int i = 0; i < N; ++i) {
            if (std::abs((pts[i] - best_p1).dot(best_n)) <= ransac_thresh_)
                res.inliers.push_back(pts[i]);
        }
        if (static_cast<int>(res.inliers.size()) < min_inliers_) return res;

        // Refine direction via SVD
        Eigen::Vector2d mean = Eigen::Vector2d::Zero();
        for (auto& p : res.inliers) mean += p;
        mean /= res.inliers.size();

        Eigen::MatrixXd centered(res.inliers.size(), 2);
        for (size_t i = 0; i < res.inliers.size(); ++i)
            centered.row(i) = (res.inliers[i] - mean).transpose();

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(centered, Eigen::ComputeThinV);
        Eigen::Vector2d dir = svd.matrixV().col(0);
        if (dir.y() < 0) dir = -dir;

        res.dir  = dir;
        res.mean = mean;
        res.ok   = true;
        return res;
    }

    // ── collect N scans and average ranges per beam ─────────────────
    sensor_msgs::LaserScan::Ptr collectAveragedScan(int count)
    {
        // Gather `count` consecutive scans
        std::vector<sensor_msgs::LaserScan::ConstPtr> scans;
        scans.reserve(count);
        ros::Rate r(50);
        ros::Time dl = ros::Time::now() + ros::Duration(2.0 + count * 0.15);

        while (ros::ok() && (int)scans.size() < count && ros::Time::now() < dl) {
            have_scan_.store(false);
            while (ros::ok() && ros::Time::now() < dl) {
                ros::spinOnce();
                if (have_scan_.load()) {
                    std::lock_guard<std::mutex> lk(scan_mutex_);
                    scans.push_back(last_scan_);
                    break;
                }
                r.sleep();
            }
        }
        if (scans.empty()) return nullptr;

        // Build averaged scan (copy header/metadata from first)
        auto avg = boost::make_shared<sensor_msgs::LaserScan>(*scans[0]);
        size_t num_beams = avg->ranges.size();

        // Accumulate: for each beam, average only the finite readings
        std::vector<double> sum(num_beams, 0.0);
        std::vector<int>    cnt(num_beams, 0);
        for (auto& sc : scans) {
            for (size_t i = 0; i < std::min(num_beams, sc->ranges.size()); ++i) {
                float rv = sc->ranges[i];
                if (std::isfinite(rv) && rv >= range_min_ && rv <= range_max_) {
                    sum[i] += rv;
                    cnt[i]++;
                }
            }
        }
        for (size_t i = 0; i < num_beams; ++i) {
            avg->ranges[i] = (cnt[i] > 0) ? static_cast<float>(sum[i] / cnt[i])
                                           : std::numeric_limits<float>::quiet_NaN();
        }

        ROS_INFO("  Averaged %zu/%d scans", scans.size(), count);
        return avg;
    }

    // ── capture one pose ─────────────────────────────────────────────
    bool captureAtCurrentPose(double /*pan_deg*/, double /*tilt_deg*/,
                              ScanCapture& cap)
    {
        // Collect and average multiple scans to reduce lidar noise
        auto scan = collectAveragedScan(scan_avg_count_);
        if (!scan) { ROS_WARN("No scans collected"); return false; }

        double cur_tilt, cur_pan;
        if (!getCurrentJoints(cur_tilt, cur_pan)) {
            ROS_WARN("No joint_states"); return false;
        }

        auto pts = extractFOV(*scan);
        if (static_cast<int>(pts.size()) < min_inliers_) {
            ROS_WARN("Only %zu pts in FOV (need %d)", pts.size(), min_inliers_);
            return false;
        }

        auto lr = ransacLineFit(pts);
        if (!lr.ok) {
            ROS_WARN("RANSAC failed (%zu inliers)", lr.inliers.size());
            return false;
        }

        cap.pan_rad    = cur_pan;
        cap.tilt_rad   = cur_tilt;
        cap.line_dir   = lr.dir;
        cap.line_mean  = lr.mean;
        cap.inlier_pts = std::move(lr.inliers);
        cap.timestamp  = scan->header.stamp.toSec();
        return true;
    }

    // ── RViz visualization ───────────────────────────────────────────
    void publishViz(const CaptureSet& cs) {
        visualization_msgs::MarkerArray ma;

        // Accumulated wall points in base_link (using nominal FK)
        visualization_msgs::Marker mk;
        mk.header.frame_id = "base_link";
        mk.header.stamp = ros::Time::now();
        mk.ns = "wall_accumulated";
        mk.id = 0;
        mk.type = visualization_msgs::Marker::POINTS;
        mk.action = visualization_msgs::Marker::ADD;
        mk.pose.orientation.w = 1.0;
        mk.scale.x = mk.scale.y = 0.004;
        mk.lifetime = ros::Duration(0);

        for (size_t ci = 0; ci < cs.captures.size(); ++ci) {
            const auto& cap = cs.captures[ci];
            Eigen::Matrix4d T = forward_kinematics_nominal(cap.pan_rad, cap.tilt_rad);
            // Golden-ratio hue spread
            float hue = std::fmod(ci * 0.618033988f, 1.0f);
            float r, g, b;
            // HSV→RGB
            {
                float h = hue * 6.0f;
                int hi = static_cast<int>(h);
                float f = h - hi;
                float q = 1.0f - 0.9f * f;
                float t_val = 1.0f - 0.9f * (1.0f - f);
                switch (hi % 6) {
                    case 0: r=1; g=t_val; b=0.1f; break;
                    case 1: r=q; g=1; b=0.1f; break;
                    case 2: r=0.1f; g=1; b=t_val; break;
                    case 3: r=0.1f; g=q; b=1; break;
                    case 4: r=t_val; g=0.1f; b=1; break;
                    default: r=1; g=0.1f; b=q; break;
                }
            }
            std_msgs::ColorRGBA color;
            color.r = r; color.g = g; color.b = b; color.a = 1.0;

            for (const auto& pt2 : cap.inlier_pts) {
                Eigen::Vector4d ph(pt2.x(), pt2.y(), 0.0, 1.0);
                Eigen::Vector4d pb = T * ph;
                geometry_msgs::Point gp;
                gp.x = pb.x(); gp.y = pb.y(); gp.z = pb.z();
                mk.points.push_back(gp);
                mk.colors.push_back(color);
            }
        }
        ma.markers.push_back(mk);
        viz_pub_.publish(ma);
    }
};

// =====================================================================
//  main – standalone acquisition node
// =====================================================================
int main(int argc, char** argv)
{
    ros::init(argc, argv, "acquire_node");
    ros::NodeHandle nh, pnh("~");
    AcquireNode node(nh, pnh);
    if (node.run()) {
        ROS_INFO("Acquisition complete: %s", node.lastCsvPath().c_str());
        return 0;
    }
    return 1;
}
