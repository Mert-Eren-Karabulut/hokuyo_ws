#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>
#include <std_srvs/Trigger.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <deque>
#include <string>
#include <vector>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <random>

struct PoseSample
{
    ros::Time stamp;
    Eigen::Matrix3d Q_world_center;              // rotation from center->world at capture time
    std::vector<Eigen::Vector3d> plane_points_L; // inlier laser-frame points (z=0)
    Eigen::Vector3d line_dir_L;                  // unit direction (in laser frame)
};

class PTULidarCalibrator
{
public:
    PTULidarCalibrator(ros::NodeHandle &nh)
        : nh_(nh),
          tf_buffer_(ros::Duration(30.0)),
          tf_listener_(tf_buffer_)
    {
        nh_.param<std::string>("world_frame", world_frame_, "map");
        nh_.param<std::string>("center_frame", center_frame_, "tilt_link");
        nh_.param<std::string>("laser_frame", laser_frame_, "laser");

        nh_.param("capture_min_range", min_r_, 0.1);
        nh_.param("capture_max_range", max_r_, 5.0);
        nh_.param("line_inlier_thresh", inlier_thresh_, 0.02);
        nh_.param("min_inliers", min_inliers_, 40);
        nh_.param("publish_line", publish_line_, true);
        nh_.param("ransac_max_iters", ransac_max_iters_, 600);
        nh_.param("publish_inliers", publish_inliers_, true);
        nh_.param("robust_loss", robust_loss_, true);
        nh_.param<std::string>("result_yaml", result_yaml_path_, std::string("/home/isl9/dev/mert/hokuyo_ws/src/ptu_lidar_calib/results" 
                                                                                      " /calib_result_" + std::to_string(ros::Time::now().toSec()) + ".yaml"));

        scan_sub_ = nh_.subscribe("/scan", 10, &PTULidarCalibrator::scanCb, this);

        srv_cap_ = nh_.advertiseService("calib/capture", &PTULidarCalibrator::srvCapture, this);
        srv_solve_ = nh_.advertiseService("calib/solve", &PTULidarCalibrator::srvSolve, this);

        ROS_INFO("PTU LiDAR Calibrator ready. Call '/calib/capture' multiple times, then '/calib/solve'.");
    }

private:
    // ===== Data & ROS =====
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    ros::ServiceServer srv_cap_, srv_solve_;

    std::string world_frame_, center_frame_, laser_frame_;
    double min_r_, max_r_, inlier_thresh_;
    int ransac_max_iters_ = 600;
    int min_inliers_;
    bool robust_loss_, publish_line_, publish_inliers_;
    std::string result_yaml_path_;

    sensor_msgs::LaserScan last_scan_;
    bool have_scan_ = false;

    std::vector<PoseSample> samples_;

    // Get the current URDF transform as the initial guess
    bool getInitialGuess(Eigen::Vector3d &t_guess, Eigen::Vector3d &w_guess)
    {
        geometry_msgs::TransformStamped T_center_laser;
        try
        {
            // Use a recent time to get the static transform
            ros::Time time_now = ros::Time(0);
            T_center_laser = tf_buffer_.lookupTransform(center_frame_, laser_frame_, time_now, ros::Duration(1.0));
        }
        catch (const tf2::TransformException &ex)
        {
            ROS_ERROR("Could not get initial guess from TF: %s", ex.what());
            ROS_ERROR("Is robot_state_publisher running with your URDF?");
            return false;
        }

        const auto &tr = T_center_laser.transform.translation;
        t_guess << tr.x, tr.y, tr.z;

        const auto &rot = T_center_laser.transform.rotation;
        Eigen::Quaterniond q_guess(rot.w, rot.x, rot.y, rot.z);
        Eigen::AngleAxisd aa_guess(q_guess);

        w_guess = aa_guess.axis() * aa_guess.angle();

        ROS_INFO_STREAM("Using initial guess t = [" << t_guess.transpose() << "]");
        ROS_INFO_STREAM("Using initial guess w = [" << w_guess.transpose() << "]");
        return true;
    }

    // ===== Laser processing =====
    static void rangesToXY(const sensor_msgs::LaserScan &s,
                           double rmin, double rmax,
                           std::vector<Eigen::Vector2d> &pts)
    {
        pts.clear();
        pts.reserve(s.ranges.size());
        for (size_t i = 0; i < s.ranges.size(); ++i)
        {
            const float r = s.ranges[i];
            if (!std::isfinite(r) || r < rmin || r > rmax)
                continue;
            const double ang = s.angle_min + i * s.angle_increment;
            pts.emplace_back(r * std::cos(ang), r * std::sin(ang));
        }
    }

    static void publishLine(const Eigen::Vector2d &dir, const Eigen::Vector2d &mean)
    {
        static ros::Publisher line_pub = ros::NodeHandle().advertise<visualization_msgs::Marker>("calib/line", 1, true);
        visualization_msgs::Marker line_marker;
        line_marker.header.frame_id = "laser";
        line_marker.header.stamp = ros::Time::now();
        line_marker.ns = "calib";
        line_marker.id = 0;
        line_marker.type = visualization_msgs::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::Marker::ADD;
        line_marker.scale.x = 0.01; // line width
        line_marker.color.r = 0.0;
        line_marker.color.g = 0.0;
        line_marker.color.b = 1.0;
        line_marker.color.a = 1.0;

        geometry_msgs::Point p1, p2;
        double len = 10.0; // length of line segment
        p1.x = mean.x() - dir.x() * len;
        p1.y = mean.y() - dir.y() * len;
        p1.z = 0.0;
        p2.x = mean.x() + dir.x() * len;
        p2.y = mean.y() + dir.y() * len;
        p2.z = 0.0;

        line_marker.points.push_back(p1);
        line_marker.points.push_back(p2);

        line_pub.publish(line_marker);


        
    }

    static void inliersFromLine(const std::vector<Eigen::Vector2d> &pts,
                                const Eigen::Vector2d &line_dir_unit,
                                const Eigen::Vector2d &line_point,
                                double inlier_thresh,
                                std::vector<int> &out_idx)
    {
        out_idx.clear();
        Eigen::Vector2d n(-line_dir_unit.y(), line_dir_unit.x()); // unit normal
        const double th = std::max(1e-6, inlier_thresh);
        for (size_t i = 0; i < pts.size(); ++i)
        {
            const double d = std::abs(n.dot(pts[i] - line_point));
            if (d <= th)
                out_idx.push_back(static_cast<int>(i));
        }
    }

    // Find the dominant line via 2D RANSAC, then refine with SVD on its inliers.
    // Returns true if a line was found; outputs unit dir, mean point, and inlier indices.
    static bool dominantLineRANSAC(const std::vector<Eigen::Vector2d> &pts,
                                   double inlier_thresh, int max_iters,
                                   Eigen::Vector2d &out_dir, Eigen::Vector2d &out_mean,
                                   std::vector<int> &out_inlier_idx,
                                   bool publish_line = false)
    {
        out_inlier_idx.clear();
        out_dir.setZero();
        out_mean.setZero();

        const size_t N = pts.size();
        if (N < 2)
            return false;

        // Distance threshold (perpendicular to line)
        const double th = std::max(1e-6, inlier_thresh);

        // RANSAC
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<int> uni(0, static_cast<int>(N) - 1);

        size_t best_count = 0;
        Eigen::Vector2d best_p1, best_p2;

        for (int it = 0; it < max_iters; ++it)
        {
            // Sample two distinct points
            int i1 = uni(gen), i2 = uni(gen);
            if (i1 == i2)
                continue;
            const Eigen::Vector2d &p1 = pts[i1];
            const Eigen::Vector2d &p2 = pts[i2];

            Eigen::Vector2d v = p2 - p1;
            const double nv = v.norm();
            if (nv < 1e-6)
                continue; // degenerate pair
            v /= nv;

            // Unit normal to the candidate line
            const Eigen::Vector2d n(-v.y(), v.x());

            // Count inliers
            size_t cnt = 0;
            for (size_t k = 0; k < N; ++k)
            {
                const double d = std::abs(n.dot(pts[k] - p1));
                if (d <= th)
                    ++cnt;
            }

            if (cnt > best_count)
            {
                best_count = cnt;
                best_p1 = p1;
                best_p2 = p2;

                // Early exit if we already explain a big chunk (tweak 0.7 if you like)
                if (best_count > 0.7 * N)
                    break;
            }
        }

        if (best_count < 2)
            return false;

        // Collect inliers for best line model
        Eigen::Vector2d v = (best_p2 - best_p1);
        const double nv = v.norm();
        if (nv < 1e-6)
            return false;
        v /= nv;
        const Eigen::Vector2d n(-v.y(), v.x());

        out_inlier_idx.reserve(best_count);
        for (size_t k = 0; k < N; ++k)
        {
            const double d = std::abs(n.dot(pts[k] - best_p1));
            if (d <= th)
                out_inlier_idx.push_back(static_cast<int>(k));
        }

        // Refine: SVD on the inlier set to get a clean direction and mean
        if (out_inlier_idx.size() < 2)
            return false;

        Eigen::Vector2d mean(0, 0);
        for (int idx : out_inlier_idx)
            mean += pts[idx];
        mean /= static_cast<double>(out_inlier_idx.size());

        Eigen::MatrixXd M(2, out_inlier_idx.size());
        for (size_t j = 0; j < out_inlier_idx.size(); ++j)
            M.col(j) = pts[out_inlier_idx[j]] - mean;

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinU);
        Eigen::Vector2d dir = svd.matrixU().col(0);
        if (dir.y() < 0.0)
            dir = -dir; // fix sign for consistency

        out_dir = dir.normalized();
        out_mean = mean;

        if (publish_line)
        {
            publishLine(out_dir, out_mean);
        }
        return true;
    }

    void scanCb(const sensor_msgs::LaserScanConstPtr &msg)
    {
        last_scan_ = *msg;
        have_scan_ = true;
    }

    bool srvCapture(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &res)
    {
        if (!have_scan_)
        {
            res.success = false;
            res.message = "No /scan yet";
            return true;
        }

        // Grab TF for center->world at scan time
        geometry_msgs::TransformStamped Tcw_msg;
        try
        {
            Tcw_msg = tf_buffer_.lookupTransform(world_frame_, center_frame_, last_scan_.header.stamp, ros::Duration(0.2));
        }
        catch (const tf2::TransformException &ex)
        {
            res.success = false;
            res.message = std::string("TF error: ") + ex.what();
            return true;
        }
        Eigen::Quaterniond qcw;
        qcw.w() = Tcw_msg.transform.rotation.w;
        qcw.x() = Tcw_msg.transform.rotation.x;
        qcw.y() = Tcw_msg.transform.rotation.y;
        qcw.z() = Tcw_msg.transform.rotation.z;
        Eigen::Matrix3d Qcw = qcw.normalized().toRotationMatrix();

        // Build laser-frame points from the current scan
        std::vector<Eigen::Vector2d> pts2;
        rangesToXY(last_scan_, min_r_, max_r_, pts2);
        if (pts2.size() < size_t(min_inliers_ / 2))
        {
            res.success = false;
            res.message = "Not enough valid ranges in capture window. See wall clearly and within range. Points = " + std::to_string(pts2.size()) +
                          " < " + std::to_string(min_inliers_ / 2);
            return true;
        }

        // Dominant line via RANSAC (handles multiple walls in 240° FOV)
        Eigen::Vector2d dir2, mean2;
        std::vector<int> inlier_idx;
        if (!dominantLineRANSAC(pts2, inlier_thresh_, ransac_max_iters_, dir2, mean2, inlier_idx, publish_line_))
        {
            res.success = false;
            res.message = "RANSAC line fit failed (not enough structure in view)";
            return true;
        }

        // --- Recompute inliers against the *refined* line we publish ---
        std::vector<int> inlier_idx_refined;
        inliersFromLine(pts2, dir2, mean2, inlier_thresh_, inlier_idx_refined);
        // Build inlier set from refined indices (this is what we’ll publish & use)
        std::vector<Eigen::Vector3d> inlier_pts_L;
        inlier_pts_L.reserve(inlier_idx_refined.size());
        for (int idx : inlier_idx_refined)
        {
            inlier_pts_L.emplace_back(pts2[idx].x(), pts2[idx].y(), 0.0);
        }
        // Optionally publish inliers as marker
        if (publish_inliers_)
        {
            static ros::Publisher inlier_pub = ros::NodeHandle().advertise<visualization_msgs::Marker>("calib/inliers", 1, true);
            visualization_msgs::Marker inlier_marker;
            inlier_marker.header.frame_id = "laser";
            inlier_marker.header.stamp = ros::Time::now();
            inlier_marker.ns = "calib";
            inlier_marker.id = 1;
            inlier_marker.type = visualization_msgs::Marker::POINTS;
            inlier_marker.action = visualization_msgs::Marker::ADD;
            inlier_marker.scale.x = 0.01; // point width
            inlier_marker.scale.y = 0.01; // point height
            inlier_marker.color.r = 0.0;
            inlier_marker.color.g = 1.0;
            inlier_marker.color.b = 0.0;
            inlier_marker.color.a = 1.0;
            for (const auto &p : inlier_pts_L)
            {
                geometry_msgs::Point mp;
                mp.x = p.x();
                mp.y = p.y();
                mp.z = p.z();
                inlier_marker.points.push_back(mp);
            }
            inlier_pub.publish(inlier_marker);
        }
        if ((int)inlier_pts_L.size() < min_inliers_)
        {
            res.success = false;
            std::ostringstream oss;
            oss << "Too few inliers: " << inlier_pts_L.size() << " < " << min_inliers_
                << ". Move closer or pick a cleaner wall view.";
            res.message = oss.str();
            return true;
        }

        // Save sample
        PoseSample s;
        s.stamp = last_scan_.header.stamp;
        s.Q_world_center = Qcw;
        s.plane_points_L = std::move(inlier_pts_L);
        s.line_dir_L = Eigen::Vector3d(dir2.x(), dir2.y(), 0.0).normalized();
        samples_.push_back(std::move(s));

        res.success = true;
        std::ostringstream oss;
        oss << "Captured pose " << samples_.size()
            << " | inliers: " << samples_.back().plane_points_L.size();
        res.message = oss.str();
        ROS_INFO("%s", res.message.c_str());
        return true;
    }

    // ---- Ceres functors ----

    struct RotResidual
    {
        RotResidual(const Eigen::Matrix3d &Qwc, const Eigen::Vector3d &uL)
            : Qwc_(Qwc), uL_(uL) {}
        template <typename T>
        bool operator()(const T *const w, const T *const n_raw, T *r) const
        {
            // w: axis-angle (3), n_raw: 3-vector -> normalize to unit
            T n_norm = ceres::sqrt(n_raw[0] * n_raw[0] + n_raw[1] * n_raw[1] + n_raw[2] * n_raw[2] + T(1e-12));
            T n[3] = {n_raw[0] / n_norm, n_raw[1] / n_norm, n_raw[2] / n_norm};

            // R = Exp(w)
            T RwcL[9];
            T RwC[9]; // will fill with Qwc * R(w)
            // Build rotation from axis-angle
            T Rcl[9];
            ceres::AngleAxisToRotationMatrix(w, Rcl);

            // Map Qwc (double) to T
            T QwcT[9];
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    QwcT[3 * i + j] = T(Qwc_(i, j));

            // RwC = Qwc * Rcl
            // Rcl maps laser->center; Qwc maps center->world
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    RwC[3 * i + j] = T(0);
                    for (int k = 0; k < 3; k++)
                        RwC[3 * i + j] += QwcT[3 * i + k] * Rcl[3 * k + j];
                }
            }

            // d = RwC * uL
            T u[3] = {T(uL_(0)), T(uL_(1)), T(uL_(2))};
            T d[3] = {T(0), T(0), T(0)};
            for (int i = 0; i < 3; i++)
            {
                for (int k = 0; k < 3; k++)
                    d[i] += RwC[3 * i + k] * u[k];
            }

            // residual = d^T * n
            r[0] = d[0] * n[0] + d[1] * n[1] + d[2] * n[2];
            return true;
        }
        const Eigen::Matrix3d Qwc_;
        const Eigen::Vector3d uL_;
    };

    struct TransResidual
    {
        TransResidual(const Eigen::Matrix3d &Qwc,
                      const Eigen::Vector3d &pL,
                      const Eigen::Vector3d &n_world,
                      const Eigen::Vector3d &w_opt)
            : Qwc_(Qwc), pL_(pL), nW_(n_world), w_opt_(w_opt) {}

        template <typename T>
        bool operator()(const T *const t, const T *const d, T *r) const
        {
            // Fixed rotation from stage A (w_opt_)
            T w[3] = {T(w_opt_(0)), T(w_opt_(1)), T(w_opt_(2))};
            T Rcl[9];
            ceres::AngleAxisToRotationMatrix(w, Rcl);

            // Qwc as T
            T QwcT[9];
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    QwcT[3 * i + j] = T(Qwc_(i, j));

            // nW
            T n[3] = {T(nW_(0)), T(nW_(1)), T(nW_(2))};

            // map pL -> world: pW = Qwc * (Rcl * pL + t)
            T pL_t[3] = {T(pL_(0)), T(pL_(1)), T(pL_(2))};
            T q[3] = {T(0), T(0), T(0)};
            // Rcl * pL
            for (int i = 0; i < 3; i++)
                for (int k = 0; k < 3; k++)
                    q[i] += Rcl[3 * i + k] * pL_t[k];
            // + t
            q[0] += t[0];
            q[1] += t[1];
            q[2] += t[2];

            // multiply by Qwc
            T pW[3] = {T(0), T(0), T(0)};
            for (int i = 0; i < 3; i++)
                for (int k = 0; k < 3; k++)
                    pW[i] += QwcT[3 * i + k] * q[k];

            // residual = n^T pW - d
            r[0] = (n[0] * pW[0] + n[1] * pW[1] + n[2] * pW[2]) - d[0];
            return true;
        }
        const Eigen::Matrix3d Qwc_;
        const Eigen::Vector3d pL_;
        const Eigen::Vector3d nW_;
        const Eigen::Vector3d w_opt_;
    };

    bool srvSolve(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &res)
    {
        if (samples_.size() < 8)
        {
            res.success = false;
            res.message = "Need at least ~8 diverse captures (recommended 12–20).";
            return true;
        }

        // ==== Get Initial Guess from URDF ====
        Eigen::Vector3d t_guess_vec, w_guess_vec;
        if (!getInitialGuess(t_guess_vec, w_guess_vec))
        {
            res.success = false;
            res.message = "Failed to get initial guess from TF tree.";
            return true;
        }

        // ==== Stage A: solve rotation (w: axis-angle) and plane normal n ====
        // Use the guess from TF!
        double w[3] = {w_guess_vec.x(), w_guess_vec.y(), w_guess_vec.z()}; 

        // Get the plane normal guess from rosparam
        std::vector<double> n_guess_vec = {0.0, 1.0, 0.0}; // Default to Y-axis
        nh_.getParam("plane_normal_guess", n_guess_vec);
        if (n_guess_vec.size() != 3) {
            ROS_WARN("plane_normal_guess must be a 3-element array. Using default [0, 1, 0]");
            n_guess_vec = {0.0, 1.0, 0.0};
        }
        
        // Use the guess as the starting seed
        double n_raw[3] = {n_guess_vec[0], n_guess_vec[1], n_guess_vec[2]}; 
        ROS_INFO("Using initial plane normal guess: [%.2f, %.2f, %.2f]", n_raw[0], n_raw[1], n_raw[2]);

        
        ceres::Problem probA;
        for (const auto &s : samples_)
        {
            ceres::CostFunction *cf =
                new ceres::AutoDiffCostFunction<RotResidual, 1, 3, 3>(new RotResidual(s.Q_world_center, s.line_dir_L));
            if (robust_loss_)
                probA.AddResidualBlock(cf, new ceres::HuberLoss(0.05), w, n_raw);
            else
                probA.AddResidualBlock(cf, nullptr, w, n_raw);
        }
        // Small regularization to keep n non-zero
        // (not strictly needed, the normalization handles scale)
        ceres::Solver::Options opts;
        opts.max_num_iterations = 100;
        opts.num_threads = 2;
        opts.linear_solver_type = ceres::DENSE_QR;
        ceres::Solver::Summary sumA;
        ceres::Solve(opts, &probA, &sumA);
        ROS_INFO_STREAM("[Stage A] " << sumA.BriefReport());

        // Normalize plane normal
        Eigen::Vector3d nW(n_raw[0], n_raw[1], n_raw[2]);
        nW.normalize();
        Eigen::Vector3d w_opt(w[0], w[1], w[2]);

        // ==== Stage B: solve translation t and plane offset d ====
        double t[3] = {t_guess_vec.x(), t_guess_vec.y(), t_guess_vec.z()}; 
        double d_scalar = 0.0;
        ceres::Problem probB;

        size_t count_pts = 0;
        for (const auto &s : samples_)
        {
            for (const auto &pL : s.plane_points_L)
            {
                ceres::CostFunction *cf =
                    new ceres::AutoDiffCostFunction<TransResidual, 1, 3, 1>(
                        new TransResidual(s.Q_world_center, pL, nW, w_opt));
                if (robust_loss_)
                    probB.AddResidualBlock(cf, new ceres::HuberLoss(0.02), t, &d_scalar);
                else
                    probB.AddResidualBlock(cf, nullptr, t, &d_scalar);
                ++count_pts;
            }
        }

        ceres::Solver::Summary sumB;
        ceres::Solve(opts, &probB, &sumB);
        ROS_INFO_STREAM("[Stage B] " << sumB.BriefReport());
        ROS_INFO("Used %zu plane points.", count_pts);

        // Report & save
        Eigen::AngleAxisd aa(w_opt.norm(), (w_opt.norm() > 1e-12) ? w_opt.normalized() : Eigen::Vector3d(1, 0, 0));
        Eigen::Matrix3d Rcl = aa.toRotationMatrix();
        Eigen::Quaterniond qcl(Rcl);

        ROS_INFO("=== Calibration Result (center_frame -> laser) ===");
        ROS_INFO_STREAM("t = [" << std::fixed << std::setprecision(6)
                                << t[0] << ", " << t[1] << ", " << t[2] << "] m");
        ROS_INFO_STREAM("q (x,y,z,w) = [" << qcl.x() << ", " << qcl.y() << ", " << qcl.z() << ", " << qcl.w() << "]");
        ROS_INFO_STREAM("Plane normal (world): [" << nW.transpose() << "]");
        ROS_INFO_STREAM("Plane offset d (world): " << d_scalar);

        // Write YAML so you can load with static_transform_publisher or a small node
        std::ofstream f(result_yaml_path_);
        f << "calibrated_transform:\n";
        f << "  parent_frame: " << center_frame_ << "\n";
        f << "  child_frame: " << laser_frame_ << "\n";
        f << "  translation: [" << t[0] << ", " << t[1] << ", " << t[2] << "]\n";
        f << "  rotation_xyzw: [" << qcl.x() << ", " << qcl.y() << ", " << qcl.z() << ", " << qcl.w() << "]\n";
        f << "plane:\n";
        f << "  world_normal: [" << nW.x() << ", " << nW.y() << ", " << nW.z() << "]\n";
        f << "  world_offset_d: " << d_scalar << "\n";
        f.close();

        res.success = true;
        res.message = std::string("Solved. YAML saved to: ") + result_yaml_path_;
        return true;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ptu_lidar_calibrator");
    ros::NodeHandle nh("~");
    PTULidarCalibrator node(nh);
    ros::spin();
    return 0;
}
