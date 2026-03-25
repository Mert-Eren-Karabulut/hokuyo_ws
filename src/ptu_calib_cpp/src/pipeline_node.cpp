// =========================================================================
//  ptu_calib_cpp  –  pipeline_node.cpp
//
//  Full calibration pipeline in one shot:
//
//    1. Acquire tilt-sweep data  (pan locked at 0°, sweep tilt)
//    2. Acquire pan-sweep data   (tilt spread across steps, sweep pan)
//    3. Solve Stage 1 (tilt + laser)
//    4. Solve Stage 2 (pan, tilt+laser corrections frozen)
//    5. Optionally acquire validation data (full grid)
//    6. Validate
//    7. Emit calibrated URDF
//
//  Prerequisites (must already be running):
//    - urg_node       (publishing /scan)
//    - Arduino serial (publishing /joint_states, accepting /joint_command)
//    - robot_state_publisher with URDF loaded
//
//  Usage:
//    roslaunch ptu_calib_cpp pipeline.launch
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
#include <ceres/ceres.h>

#include "ptu_calib_cpp/kinematic_model.h"
#include "ptu_calib_cpp/scan_capture.h"

#include <mutex>
#include <atomic>
#include <cmath>
#include <random>
#include <string>
#include <fstream>
#include <sstream>
#include <ctime>
#include <limits>
#include <sys/stat.h>
#include <boost/make_shared.hpp>

using namespace ptu_calib;

// =====================================================================
//  Forward-declare solver functions (defined in solve_node.cpp, but we
//  inline the needed pieces here to keep the pipeline self-contained).
// =====================================================================

// We include solve_node's logic directly via the header-only solver
// approach.  Since the solver functions reference ROS_INFO etc., and
// this is a single-TU build, we embed the core solver code here.

// -- subsample --
static std::vector<std::vector<Eigen::Vector2d>>
subsample_pts(const CaptureSet& cs, int max_per_capture = 30)
{
    std::vector<std::vector<Eigen::Vector2d>> out;
    for (auto& cap : cs.captures) {
        std::vector<Eigen::Vector2d> sub;
        int N = static_cast<int>(cap.inlier_pts.size());
        if (N <= max_per_capture) {
            sub = cap.inlier_pts;
        } else {
            int step = std::max(1, N / max_per_capture);
            for (int i = 0; i < N && (int)sub.size() < max_per_capture; i += step)
                sub.push_back(cap.inlier_pts[i]);
        }
        out.push_back(std::move(sub));
    }
    return out;
}

// -- initial plane estimate --
static void estimate_plane(const CaptureSet& cs,
                           const std::vector<std::vector<Eigen::Vector2d>>& sub,
                           const CalibResult& partial_cal,
                           double n_out[3], double& d_out)
{
    std::vector<Eigen::Vector3d> all;
    for (size_t k = 0; k < cs.captures.size(); ++k) {
        Eigen::Matrix4d T = forward_kinematics(
            cs.captures[k].pan_rad, cs.captures[k].tilt_rad, partial_cal);
        for (auto& p2 : sub[k]) {
            Eigen::Vector4d ph(p2.x(), p2.y(), 0.0, 1.0);
            all.push_back((T * ph).head<3>());
        }
    }
    Eigen::Vector3d cen = Eigen::Vector3d::Zero();
    for (auto& p : all) cen += p;
    cen /= all.size();
    Eigen::MatrixXd M(all.size(), 3);
    for (size_t i = 0; i < all.size(); ++i)
        M.row(i) = (all[i] - cen).transpose();
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinV);
    Eigen::Vector3d n = svd.matrixV().col(2);
    double d = n.dot(cen);
    if (d < 0) { n = -n; d = -d; }
    n_out[0]=n.x(); n_out[1]=n.y(); n_out[2]=n.z();
    d_out = d;

    double sse = 0;
    for (auto& p : all) { double e=n.dot(p)-d; sse+=e*e; }
    ROS_INFO("  Plane: n=[%.4f,%.4f,%.4f] d=%.4f  RMSE=%.2fmm",
             n.x(),n.y(),n.z(),d, std::sqrt(sse/all.size())*1000);
}

// =====================================================================
//  Ceres cost functors (same as solve_node but self-contained here)
// =====================================================================

// Stage 1: tilt+laser cost (pan = identity, since pan≈0 in tilt data)
struct TiltLaserPlaneCostP {
    double theta_pan, theta_tilt, px, py;
    double d_norm;  // plane distance for range-normalisation (m)
    TiltLaserPlaneCostP(double tp, double tt, double x, double y, double dn)
        : theta_pan(tp), theta_tilt(tt), px(x), py(y), d_norm(dn) {}

    template<typename T>
    bool operator()(const T* tilt_params, const T* laser_params,
                    const T* plane, T* residual) const {
        T pan_p[7]={T(0),T(0),T(0),T(0),T(0),T(0),T(0)};
        auto Tf = forward_kinematics_t<T>(T(theta_pan),T(theta_tilt),
                                           pan_p,tilt_params,laser_params);
        Eigen::Matrix<T,4,1> ph; ph<<T(px),T(py),T(0),T(1);
        auto pb = Tf * ph;
        T nx=plane[0],ny=plane[1],nz=plane[2],d=plane[3];
        T nn=ceres::sqrt(nx*nx+ny*ny+nz*nz+T(1e-12));
        residual[0] = (nx*pb(0)+ny*pb(1)+nz*pb(2)-d)/nn / T(d_norm) * T(1000);
        return true;
    }
};

// Stage 2: pan cost (tilt+laser corrections frozen from Stage 1)
struct PanPlaneCostP {
    double theta_pan, theta_tilt, px, py;
    double frozen_tilt[7];
    double frozen_laser[6];
    double d_norm;  // plane distance for range-normalisation (m)
    PanPlaneCostP(double tp, double tt, double x, double y,
                  const double* ft, const double* fl, double dn)
        : theta_pan(tp), theta_tilt(tt), px(x), py(y), d_norm(dn)
    {
        for(int i=0;i<7;++i) frozen_tilt[i]=ft[i];
        for(int i=0;i<6;++i) frozen_laser[i]=fl[i];
    }

    template<typename T>
    bool operator()(const T* pan_params, const T* plane, T* residual) const {
        T tp[7]; for(int i=0;i<7;++i) tp[i]=T(frozen_tilt[i]);
        T lp[6]; for(int i=0;i<6;++i) lp[i]=T(frozen_laser[i]);
        auto Tf = forward_kinematics_t<T>(T(theta_pan),T(theta_tilt),
                                           pan_params,tp,lp);
        Eigen::Matrix<T,4,1> ph; ph<<T(px),T(py),T(0),T(1);
        auto pb = Tf * ph;
        T nx=plane[0],ny=plane[1],nz=plane[2],d=plane[3];
        T nn=ceres::sqrt(nx*nx+ny*ny+nz*nz+T(1e-12));
        residual[0] = (nx*pb(0)+ny*pb(1)+nz*pb(2)-d)/nn / T(d_norm) * T(1000);
        return true;
    }
};

struct RegCost1 {
    int idx; double w;
    RegCost1(int i, double ww) : idx(i), w(ww) {}
    template<typename T>
    bool operator()(const T* p, T* r) const { r[0]=T(w)*p[idx]; return true; }
};

// =====================================================================
//  Inline acquisition (same logic as acquire_node but embedded)
// =====================================================================
class AcquireHelper {
public:
    AcquireHelper(ros::NodeHandle& nh, double fov_half_deg, double settle_time,
                  double pos_tol_deg, double motion_timeout,
                  int ransac_iters, double ransac_thresh, int min_inliers,
                  double range_min, double range_max, int scan_avg_count = 10)
        : nh_(nh), fov_half_deg_(fov_half_deg), settle_time_(settle_time),
          pos_tol_deg_(pos_tol_deg), motion_timeout_(motion_timeout),
          ransac_iters_(ransac_iters), ransac_thresh_(ransac_thresh),
          min_inliers_(min_inliers), range_min_(range_min), range_max_(range_max),
          scan_avg_count_(scan_avg_count)
    {
        cmd_pub_  = nh_.advertise<sensor_msgs::JointState>("/joint_command", 10);
        viz_pub_  = nh_.advertise<visualization_msgs::MarkerArray>(
                        "/calibration/points", 5, true);
        scan_sub_ = nh_.subscribe("/scan", 5, &AcquireHelper::scanCB, this);
        js_sub_   = nh_.subscribe("/joint_states", 50, &AcquireHelper::jsCB, this);
    }

    ~AcquireHelper() {
        scan_sub_.shutdown();
        js_sub_.shutdown();
    }

    bool waitForData() {
        ros::Rate r(10);
        ros::Time dl = ros::Time::now() + ros::Duration(10.0);
        while (ros::ok()) {
            ros::spinOnce();
            if (have_scan_.load() && have_js_.load()) return true;
            if (ros::Time::now() > dl) return false;
            r.sleep();
        }
        return false;
    }

    CaptureSet acquire(const std::string& mode,
                       double pan_lower_deg, double pan_upper_deg,
                       double tilt_lower_deg, double tilt_upper_deg,
                       int pan_steps, int tilt_steps)
    {
        CaptureSet cs;
        cs.fov_half_deg = fov_half_deg_;
        cs.label = mode;

        auto grid = generateGrid(mode, pan_lower_deg, pan_upper_deg,
                                 tilt_lower_deg, tilt_upper_deg,
                                 pan_steps, tilt_steps);
        ROS_INFO("Acquiring %s data: %zu poses", mode.c_str(), grid.size());

        goToPose(0.0, 0.0);

        for (size_t i = 0; i < grid.size() && ros::ok(); ++i) {
            double pd = grid[i].first, td = grid[i].second;
            ROS_INFO("  Pose %zu/%zu: pan=%+.1f  tilt=%+.1f",
                     i+1, grid.size(), pd, td);
            if (!goToPose(pd, td)) { ROS_WARN("  timeout"); continue; }

            ScanCapture cap;
            if (!captureHere(cap)) { ROS_WARN("  capture failed"); continue; }
            cs.captures.push_back(std::move(cap));
            ROS_INFO("  OK: %zu inliers", cs.captures.back().inlier_pts.size());
        }
        goToPose(0.0, 0.0);
        ROS_INFO("  Acquired %zu/%zu", cs.captures.size(), grid.size());
        return cs;
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_pub_, viz_pub_;
    ros::Subscriber scan_sub_, js_sub_;
    double fov_half_deg_, settle_time_, pos_tol_deg_, motion_timeout_;
    int ransac_iters_, min_inliers_, scan_avg_count_;
    double ransac_thresh_, range_min_, range_max_;

    std::mutex scan_mx_, js_mx_;
    sensor_msgs::LaserScan::ConstPtr last_scan_;
    sensor_msgs::JointState::ConstPtr last_js_;
    std::atomic<bool> have_scan_{false}, have_js_{false};

    void scanCB(const sensor_msgs::LaserScan::ConstPtr& m) {
        std::lock_guard<std::mutex> l(scan_mx_); last_scan_=m; have_scan_=true;
    }
    void jsCB(const sensor_msgs::JointState::ConstPtr& m) {
        std::lock_guard<std::mutex> l(js_mx_); last_js_=m; have_js_=true;
    }

    bool getCurrentJoints(double& tilt, double& pan) {
        std::lock_guard<std::mutex> l(js_mx_);
        if (!last_js_) return false;
        tilt=pan=0;
        for (size_t i=0;i<last_js_->name.size();++i) {
            if (last_js_->name[i]=="joint1") tilt=last_js_->position[i];
            if (last_js_->name[i]=="joint2") pan=last_js_->position[i];
        }
        return true;
    }

    void commandPose(double tilt_rad, double pan_rad) {
        sensor_msgs::JointState m;
        m.header.stamp = ros::Time::now();
        m.name = {"joint1","joint2"};
        m.position = {tilt_rad, pan_rad};
        cmd_pub_.publish(m);
    }

    bool goToPose(double pd, double td) {
        double pr=pd*M_PI/180, tr=td*M_PI/180, tol=pos_tol_deg_*M_PI/180;
        ros::Rate rate(50);
        ros::Time t0=ros::Time::now();
        while (ros::ok()) {
            commandPose(tr,pr);
            ros::spinOnce();
            double ct,cp;
            if (getCurrentJoints(ct,cp) &&
                std::abs(ct-tr)<=tol && std::abs(cp-pr)<=tol)
            {
                ros::Time se=ros::Time::now()+ros::Duration(settle_time_);
                while (ros::Time::now()<se && ros::ok()) {
                    commandPose(tr,pr); ros::spinOnce(); rate.sleep();
                }
                return true;
            }
            if ((ros::Time::now()-t0).toSec()>motion_timeout_) return false;
            rate.sleep();
        }
        return false;
    }

    std::vector<std::pair<double,double>>
    generateGrid(const std::string& mode,
                 double pl_lo, double pl_hi,
                 double tl_lo, double tl_hi,
                 int ps, int ts)
    {
        std::vector<double> pans, tilts;
        if (mode=="pan"||mode=="both") {
            if (ps<=1) pans={0}; else {
                pans.resize(ps);
                for (int i=0;i<ps;++i) pans[i]=pl_lo+(pl_hi-pl_lo)*i/(ps-1);
            }
        } else pans={0};

        if (mode=="tilt"||mode=="both") {
            if (ts<=1) tilts={0}; else {
                tilts.resize(ts);
                for (int i=0;i<ts;++i) tilts[i]=tl_lo+(tl_hi-tl_lo)*i/(ts-1);
            }
        } else tilts={0};

        std::vector<std::pair<double,double>> g;
        if (mode=="pan") {
            // Spread tilt across pan steps so strips define a plane
            for (int i=0;i<(int)pans.size();++i) {
                double frac = (pans.size()<=1) ? 0.0 : (double)i/(pans.size()-1);
                double t = tl_lo + (tl_hi-tl_lo)*frac;
                g.emplace_back(pans[i], t);
            }
        } else {
            for (double p:pans) for (double t:tilts) g.emplace_back(p,t);
        }
        return g;
    }

    sensor_msgs::LaserScan::Ptr collectAveragedScan(int count) {
        std::vector<sensor_msgs::LaserScan::ConstPtr> scans;
        scans.reserve(count);
        ros::Rate r(50);
        ros::Time dl=ros::Time::now()+ros::Duration(2.0+count*0.15);
        while (ros::ok()&&(int)scans.size()<count&&ros::Time::now()<dl) {
            have_scan_=false;
            while (ros::ok()&&ros::Time::now()<dl) {
                ros::spinOnce();
                if (have_scan_.load()) {
                    std::lock_guard<std::mutex> l(scan_mx_);
                    scans.push_back(last_scan_);
                    break;
                }
                r.sleep();
            }
        }
        if (scans.empty()) return nullptr;
        auto avg=boost::make_shared<sensor_msgs::LaserScan>(*scans[0]);
        size_t nb=avg->ranges.size();
        std::vector<double> sum(nb,0); std::vector<int> cnt(nb,0);
        for (auto& sc:scans)
            for (size_t i=0;i<std::min(nb,sc->ranges.size());++i) {
                float rv=sc->ranges[i];
                if (std::isfinite(rv)&&rv>=range_min_&&rv<=range_max_) { sum[i]+=rv; cnt[i]++; }
            }
        for (size_t i=0;i<nb;++i)
            avg->ranges[i]=(cnt[i]>0)?static_cast<float>(sum[i]/cnt[i])
                                     :std::numeric_limits<float>::quiet_NaN();
        return avg;
    }

    bool captureHere(ScanCapture& cap) {
        auto sc=collectAveragedScan(scan_avg_count_);
        if (!sc) return false;

        double ct,cp;
        if (!getCurrentJoints(ct,cp)) return false;

        // FOV extract
        double fov_rad=fov_half_deg_*M_PI/180;
        std::vector<Eigen::Vector2d> pts;
        for (size_t i=0;i<sc->ranges.size();++i) {
            double rv=sc->ranges[i];
            if (!std::isfinite(rv)||rv<range_min_||rv>range_max_) continue;
            double a=sc->angle_min+i*sc->angle_increment;
            if (std::abs(a)>fov_rad) continue;
            pts.emplace_back(rv*std::cos(a), rv*std::sin(a));
        }
        if ((int)pts.size()<min_inliers_) return false;

        // RANSAC
        int N=(int)pts.size();
        std::mt19937 rng(42);
        std::uniform_int_distribution<int> dist(0,N-1);
        int best_cnt=0;
        Eigen::Vector2d best_p1, best_n;
        for (int it=0;it<ransac_iters_;++it) {
            int a=dist(rng), b=dist(rng);
            if (a==b) continue;
            Eigen::Vector2d v=pts[b]-pts[a];
            double nv=v.norm(); if(nv<1e-6) continue;
            v/=nv;
            Eigen::Vector2d nn(-v.y(),v.x());
            int cnt=0;
            for (int i=0;i<N;++i)
                if (std::abs((pts[i]-pts[a]).dot(nn))<=ransac_thresh_) ++cnt;
            if (cnt>best_cnt) {
                best_cnt=cnt; best_p1=pts[a]; best_n=nn;
                if (best_cnt>0.7*N) break;
            }
        }
        if (best_cnt<2) return false;

        std::vector<Eigen::Vector2d> inliers;
        for (int i=0;i<N;++i)
            if (std::abs((pts[i]-best_p1).dot(best_n))<=ransac_thresh_)
                inliers.push_back(pts[i]);
        if ((int)inliers.size()<min_inliers_) return false;

        Eigen::Vector2d mean=Eigen::Vector2d::Zero();
        for (auto& p:inliers) mean+=p;
        mean /= inliers.size();

        Eigen::MatrixXd C(inliers.size(),2);
        for (size_t i=0;i<inliers.size();++i) C.row(i)=(inliers[i]-mean).transpose();
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(C, Eigen::ComputeThinV);
        Eigen::Vector2d dir=svd.matrixV().col(0);
        if (dir.y()<0) dir=-dir;

        cap.pan_rad=cp; cap.tilt_rad=ct;
        cap.line_dir=dir; cap.line_mean=mean;
        cap.inlier_pts=std::move(inliers);
        cap.timestamp=sc->header.stamp.toSec();
        return true;
    }
};

// =====================================================================
//  Pipeline solve helpers
// =====================================================================

// Stage 1: tilt + laser (pan = identity)
static bool solve_tilt_stage(const CaptureSet& data,
                             double tilt_out[7], double laser_out[6],
                             double reg, double enc_b, double dt_b, double dr_b)
{
    auto sub = subsample_pts(data, 30);
    size_t nc = data.captures.size();
    ROS_INFO("=== STAGE 1: TILT+LASER  (%zu captures) ===", nc);
    if (nc < 3) { ROS_ERROR("Need >=3"); return false; }

    CalibResult zero;
    double plane[4]; estimate_plane(data, sub, zero, plane, plane[3]);
    double d_norm = std::max(std::abs(plane[3]), 0.1);
    ROS_INFO("  Range normalisation: d=%.3f m", d_norm);
    double tp[7]={}, lp[6]={};

    ceres::Problem prob;
    for (size_t k=0;k<nc;++k)
        for (auto& pt:sub[k])
            prob.AddResidualBlock(
                new ceres::AutoDiffCostFunction<TiltLaserPlaneCostP,1,7,6,4>(
                    new TiltLaserPlaneCostP(data.captures[k].pan_rad,
                                            data.captures[k].tilt_rad,
                                            pt.x(),pt.y(),d_norm)),
                nullptr, tp, lp, plane);

    double tr=reg*500000, rr=reg*100000, er=reg*100000;
    for (int i=0;i<3;++i) prob.AddResidualBlock(
        new ceres::AutoDiffCostFunction<RegCost1,1,7>(new RegCost1(i,tr)),nullptr,tp);
    for (int i=3;i<6;++i) prob.AddResidualBlock(
        new ceres::AutoDiffCostFunction<RegCost1,1,7>(new RegCost1(i,rr)),nullptr,tp);
    prob.AddResidualBlock(
        new ceres::AutoDiffCostFunction<RegCost1,1,7>(new RegCost1(6,er)),nullptr,tp);

    for (int i=0;i<3;++i) prob.AddResidualBlock(
        new ceres::AutoDiffCostFunction<RegCost1,1,6>(new RegCost1(i,tr)),nullptr,lp);
    for (int i=3;i<6;++i) prob.AddResidualBlock(
        new ceres::AutoDiffCostFunction<RegCost1,1,6>(new RegCost1(i,rr)),nullptr,lp);

    for (int i=0;i<3;++i) { prob.SetParameterLowerBound(tp,i,-dt_b); prob.SetParameterUpperBound(tp,i,dt_b); }
    for (int i=3;i<6;++i) { prob.SetParameterLowerBound(tp,i,-dr_b); prob.SetParameterUpperBound(tp,i,dr_b); }
    prob.SetParameterLowerBound(tp,6,-enc_b); prob.SetParameterUpperBound(tp,6,enc_b);

    for (int i=0;i<3;++i) { prob.SetParameterLowerBound(lp,i,-dt_b); prob.SetParameterUpperBound(lp,i,dt_b); }
    for (int i=3;i<6;++i) { prob.SetParameterLowerBound(lp,i,-dr_b); prob.SetParameterUpperBound(lp,i,dr_b); }

    for (int i=0;i<3;++i) { prob.SetParameterLowerBound(plane,i,-1.5); prob.SetParameterUpperBound(plane,i,1.5); }
    prob.SetParameterLowerBound(plane,3,-20); prob.SetParameterUpperBound(plane,3,20);

    ceres::Solver::Options o;
    o.linear_solver_type=ceres::DENSE_QR;
    o.max_num_iterations=500;
    o.function_tolerance=o.parameter_tolerance=o.gradient_tolerance=1e-12;
    o.minimizer_progress_to_stdout=true;
    ceres::Solver::Summary s;
    ceres::Solve(o,&prob,&s);
    ROS_INFO("\n%s",s.BriefReport().c_str());

    for (int i=0;i<7;++i) tilt_out[i]=tp[i];
    for (int i=0;i<6;++i) laser_out[i]=lp[i];
    ROS_INFO("  Tilt dt=[%+.3f,%+.3f,%+.3f]mm  dr=[%+.3f,%+.3f,%+.3f]deg  enc=%+.3fdeg",
             tp[0]*1e3,tp[1]*1e3,tp[2]*1e3,
             tp[3]*180/M_PI,tp[4]*180/M_PI,tp[5]*180/M_PI,tp[6]*180/M_PI);
    ROS_INFO("  Laser dt=[%+.3f,%+.3f,%+.3f]mm  dr=[%+.3f,%+.3f,%+.3f]deg",
             lp[0]*1e3,lp[1]*1e3,lp[2]*1e3,
             lp[3]*180/M_PI,lp[4]*180/M_PI,lp[5]*180/M_PI);
    return s.IsSolutionUsable();
}

// Stage 2: pan (tilt+laser corrections frozen from Stage 1)
static bool solve_pan_stage(const CaptureSet& data,
                            const double frozen_tilt[7], const double frozen_laser[6],
                            double pan_out[7],
                            double reg, double enc_b, double dt_b, double dr_b)
{
    auto sub = subsample_pts(data, 30);
    size_t nc = data.captures.size();
    ROS_INFO("=== STAGE 2: PAN  (%zu captures) ===", nc);
    if (nc < 3) { ROS_ERROR("Need >=3"); return false; }

    // Use tilt+laser corrections for initial plane estimate
    CalibResult partial;
    partial.tilt_corr.from_array(frozen_tilt);
    partial.tilt_encoder_zero = frozen_tilt[6];
    partial.laser_corr.from_array(frozen_laser);

    double plane[4]; estimate_plane(data, sub, partial, plane, plane[3]);
    double d_norm = std::max(std::abs(plane[3]), 0.1);
    ROS_INFO("  Range normalisation: d=%.3f m", d_norm);
    double pp[7] = {};

    ceres::Problem prob;
    for (size_t k=0;k<nc;++k)
        for (auto& pt:sub[k])
            prob.AddResidualBlock(
                new ceres::AutoDiffCostFunction<PanPlaneCostP,1,7,4>(
                    new PanPlaneCostP(data.captures[k].pan_rad,
                                     data.captures[k].tilt_rad,
                                     pt.x(),pt.y(),
                                     frozen_tilt, frozen_laser, d_norm)),
                nullptr, pp, plane);

    double tr=reg*500000, rr=reg*100000, er=reg*100000;
    for (int i=0;i<3;++i) prob.AddResidualBlock(
        new ceres::AutoDiffCostFunction<RegCost1,1,7>(new RegCost1(i,tr)),nullptr,pp);
    for (int i=3;i<6;++i) prob.AddResidualBlock(
        new ceres::AutoDiffCostFunction<RegCost1,1,7>(new RegCost1(i,rr)),nullptr,pp);
    prob.AddResidualBlock(
        new ceres::AutoDiffCostFunction<RegCost1,1,7>(new RegCost1(6,er)),nullptr,pp);

    for (int i=0;i<3;++i) { prob.SetParameterLowerBound(pp,i,-dt_b); prob.SetParameterUpperBound(pp,i,dt_b); }
    for (int i=3;i<6;++i) { prob.SetParameterLowerBound(pp,i,-dr_b); prob.SetParameterUpperBound(pp,i,dr_b); }
    prob.SetParameterLowerBound(pp,6,-enc_b); prob.SetParameterUpperBound(pp,6,enc_b);
    for (int i=0;i<3;++i) { prob.SetParameterLowerBound(plane,i,-1.5); prob.SetParameterUpperBound(plane,i,1.5); }
    prob.SetParameterLowerBound(plane,3,-20); prob.SetParameterUpperBound(plane,3,20);

    ceres::Solver::Options o;
    o.linear_solver_type=ceres::DENSE_QR;
    o.max_num_iterations=500;
    o.function_tolerance=o.parameter_tolerance=o.gradient_tolerance=1e-12;
    o.minimizer_progress_to_stdout=true;
    ceres::Solver::Summary s;
    ceres::Solve(o,&prob,&s);
    ROS_INFO("\n%s",s.BriefReport().c_str());

    for (int i=0;i<7;++i) pan_out[i]=pp[i];
    ROS_INFO("  Pan dt=[%+.3f,%+.3f,%+.3f]mm  dr=[%+.3f,%+.3f,%+.3f]deg  enc=%+.3fdeg",
             pp[0]*1e3,pp[1]*1e3,pp[2]*1e3,
             pp[3]*180/M_PI,pp[4]*180/M_PI,pp[5]*180/M_PI,pp[6]*180/M_PI);
    return s.IsSolutionUsable();
}

static double validate_set(const CaptureSet& data, const CalibResult& cal,
                           const std::string& label)
{
    auto sub = subsample_pts(data, 50);
    std::vector<Eigen::Vector3d> cal_pts, uncal_pts;

    CalibResult zero;
    for (size_t k=0;k<data.captures.size();++k) {
        Eigen::Matrix4d Tc = forward_kinematics(
            data.captures[k].pan_rad, data.captures[k].tilt_rad, cal);
        Eigen::Matrix4d Tu = forward_kinematics(
            data.captures[k].pan_rad, data.captures[k].tilt_rad, zero);
        for (auto& p2:sub[k]) {
            Eigen::Vector4d ph(p2.x(),p2.y(),0,1);
            cal_pts.push_back((Tc*ph).head<3>());
            uncal_pts.push_back((Tu*ph).head<3>());
        }
    }

    auto fit_rmse = [](const std::vector<Eigen::Vector3d>& pts) -> double {
        Eigen::Vector3d c=Eigen::Vector3d::Zero();
        for (auto& p:pts) c+=p; c/=pts.size();
        Eigen::MatrixXd M(pts.size(),3);
        for (size_t i=0;i<pts.size();++i) M.row(i)=(pts[i]-c).transpose();
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinV);
        Eigen::Vector3d n=svd.matrixV().col(2);
        double d=n.dot(c);
        double sse=0;
        for (auto& p:pts) { double e=n.dot(p)-d; sse+=e*e; }
        return std::sqrt(sse/pts.size())*1000;
    };

    double rmse_cal   = fit_rmse(cal_pts);
    double rmse_uncal = fit_rmse(uncal_pts);

    ROS_INFO("--- Validation [%s]: %zu caps, %zu pts ---",
             label.c_str(), data.captures.size(), cal_pts.size());
    ROS_INFO("  Uncalibrated RMSE: %.3f mm", rmse_uncal);
    ROS_INFO("  Calibrated   RMSE: %.3f mm", rmse_cal);
    ROS_INFO("  Improvement: %.1f%%", (1-rmse_cal/std::max(rmse_uncal,1e-12))*100);
    return rmse_cal;
}

// =====================================================================
//  main
// =====================================================================
int main(int argc, char** argv)
{
    ros::init(argc, argv, "ptu_calib_pipeline");
    ros::NodeHandle nh, pnh("~");

    // ── Parameters ──
    double pan_lower_deg, pan_upper_deg, tilt_lower_deg, tilt_upper_deg;
    int    pan_steps, tilt_steps;
    double fov_half_deg, settle_time, pos_tol_deg, motion_timeout;
    int    ransac_iters, min_inliers;
    double ransac_thresh, range_min, range_max;
    double reg_weight, enc_bound_deg, dt_bound_mm, dr_bound_deg;
    bool   do_validation;
    int    val_pan_steps, val_tilt_steps;
    double val_pan_limit, val_tilt_limit;
    std::string output_dir;

    pnh.param("pan_lower_deg",  pan_lower_deg, -25.0);
    pnh.param("pan_upper_deg",  pan_upper_deg,  25.0);
    pnh.param("tilt_lower_deg", tilt_lower_deg,-15.0);
    pnh.param("tilt_upper_deg", tilt_upper_deg, 15.0);
    pnh.param("pan_steps",      pan_steps,       7);
    pnh.param("tilt_steps",     tilt_steps,      7);
    pnh.param("fov_half_deg",   fov_half_deg,   15.0);
    pnh.param("settle_time",    settle_time,     1.5);
    pnh.param("position_tolerance_deg", pos_tol_deg, 1.0);
    pnh.param("motion_timeout", motion_timeout,  10.0);
    pnh.param("ransac_iterations", ransac_iters,  600);
    pnh.param("ransac_inlier_thresh_m", ransac_thresh, 0.015);
    pnh.param("min_inliers",    min_inliers,      30);
    pnh.param("range_min",      range_min,        0.10);
    pnh.param("range_max",      range_max,        4.00);
    pnh.param("reg_weight",     reg_weight,      0.01);
    pnh.param("enc_bound_deg",  enc_bound_deg,   8.0);
    pnh.param("dt_bound_mm",    dt_bound_mm,     2.0);
    pnh.param("dr_bound_deg",   dr_bound_deg,    2.0);
    pnh.param("do_validation",  do_validation,  true);
    pnh.param("val_pan_steps",  val_pan_steps,   5);
    pnh.param("val_tilt_steps", val_tilt_steps,  5);
    pnh.param("val_pan_limit",  val_pan_limit,  20.0);
    pnh.param("val_tilt_limit", val_tilt_limit, 12.0);
    pnh.param<std::string>("output_dir", output_dir, ".");

    mkdir(output_dir.c_str(), 0755);
    time_t now = time(nullptr);
    char ts[64]; strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", localtime(&now));

    double enc_b  = enc_bound_deg * M_PI/180;
    double dt_b   = dt_bound_mm / 1000.0;
    double dr_b   = dr_bound_deg * M_PI/180;

    ROS_INFO("========================================");
    ROS_INFO("  PTU CALIBRATION PIPELINE (C++)");
    ROS_INFO("========================================");
    ROS_INFO("  Pan  sweep: %d steps, [%.1f, %.1f] deg", pan_steps, pan_lower_deg, pan_upper_deg);
    ROS_INFO("  Tilt sweep: %d steps, [%.1f, %.1f] deg", tilt_steps, tilt_lower_deg, tilt_upper_deg);
    ROS_INFO("  FOV half: %.1f deg", fov_half_deg);
    ROS_INFO("  Output: %s", output_dir.c_str());

    // ── Acquisition helper ──
    int scan_avg_count;
    pnh.param("scan_avg_count", scan_avg_count, 10);

    AcquireHelper acq(nh, fov_half_deg, settle_time, pos_tol_deg,
                      motion_timeout, ransac_iters, ransac_thresh,
                      min_inliers, range_min, range_max, scan_avg_count);

    if (!acq.waitForData()) {
        ROS_ERROR("No /scan or /joint_states — is hardware running?");
        return 1;
    }

    // ── Phase 1: Tilt sweep (well-constrained, solve first) ──
    ROS_INFO("\n██ PHASE 1: TILT SWEEP (pan=0) ██");
    CaptureSet tilt_data = acq.acquire("tilt", pan_lower_deg, pan_upper_deg,
                                        tilt_lower_deg, tilt_upper_deg,
                                        pan_steps, tilt_steps);
    if (tilt_data.captures.size() < 3) {
        ROS_ERROR("Not enough tilt captures (%zu)", tilt_data.captures.size());
        return 1;
    }
    {
        std::string f = output_dir+"/tilt_data_"+ts+".csv";
        save_captures_csv(tilt_data, f);
        ROS_INFO("  Saved tilt data → %s", f.c_str());
    }

    // ── Phase 2: Pan sweep (tilt spread across steps) ──
    ROS_INFO("\n██ PHASE 2: PAN SWEEP (tilt spread) ██");
    CaptureSet pan_data = acq.acquire("pan", pan_lower_deg, pan_upper_deg,
                                       tilt_lower_deg, tilt_upper_deg,
                                       pan_steps, tilt_steps);
    if (pan_data.captures.size() < 3) {
        ROS_ERROR("Not enough pan captures (%zu)", pan_data.captures.size());
        return 1;
    }
    {
        std::string f = output_dir+"/pan_data_"+ts+".csv";
        save_captures_csv(pan_data, f);
        ROS_INFO("  Saved pan data → %s", f.c_str());
    }

    // ── Phase 3: Solve tilt + laser (Stage 1) ──
    ROS_INFO("\n██ PHASE 3: SOLVE TILT + LASER ██");
    double tilt_params[7] = {}, laser_params[6] = {};
    if (!solve_tilt_stage(tilt_data, tilt_params, laser_params,
                          reg_weight, enc_b, dt_b, dr_b)) {
        ROS_ERROR("Tilt+Laser solve failed!");
        return 1;
    }

    // ── Phase 4: Solve pan (Stage 2, frozen tilt+laser) ──
    ROS_INFO("\n██ PHASE 4: SOLVE PAN ██");
    double pan_params[7] = {};
    if (!solve_pan_stage(pan_data, tilt_params, laser_params, pan_params,
                         reg_weight, enc_b, dt_b, dr_b)) {
        ROS_ERROR("Pan solve failed!");
        return 1;
    }

    // ── Build CalibResult ──
    CalibResult result;
    result.pan_corr.from_array(pan_params);
    result.pan_encoder_zero  = pan_params[6];
    result.tilt_corr.from_array(tilt_params);
    result.tilt_encoder_zero = tilt_params[6];
    result.laser_corr.from_array(laser_params);

    // ── Phase 5: Validation (optional) ──
    if (do_validation) {
        ROS_INFO("\n██ PHASE 5: VALIDATION ██");
        CaptureSet val_data = acq.acquire("both",
                                           -val_pan_limit, val_pan_limit,
                                           -val_tilt_limit, val_tilt_limit,
                                           val_pan_steps, val_tilt_steps);
        if (val_data.captures.size() >= 4) {
            std::string f = output_dir+"/val_data_"+ts+".csv";
            save_captures_csv(val_data, f);
            validate_set(val_data, result, "validation");
        } else {
            ROS_WARN("Not enough validation captures, self-validating");
        }
    }

    // Cross-validate on training data
    validate_set(pan_data,  result, "pan_train");
    validate_set(tilt_data, result, "tilt_train");

    // ── Phase 6: Generate calibrated URDF ──
    ROS_INFO("\n██ PHASE 6: GENERATE CALIBRATED URDF ██");

    ROS_INFO("--- Combined result ---");
    ROS_INFO("  Pan  encoder zero: %+.4f deg", result.pan_encoder_zero*180/M_PI);
    ROS_INFO("  Tilt encoder zero: %+.4f deg", result.tilt_encoder_zero*180/M_PI);

    std::string urdf = generate_calibrated_urdf(result);
    std::string urdf_path = output_dir + "/calibrated_pantilt_" + ts + ".urdf";
    {
        std::ofstream f(urdf_path);
        if (f.is_open()) {
            f << urdf;
            ROS_INFO("Saved calibrated URDF → %s", urdf_path.c_str());
        } else {
            ROS_ERROR("Failed to write URDF");
        }
    }

    // Print URDF to console
    ROS_INFO("\n%s", urdf.c_str());

    ROS_INFO("\n========================================");
    ROS_INFO("  PIPELINE COMPLETE");
    ROS_INFO("========================================");

    return 0;
}
