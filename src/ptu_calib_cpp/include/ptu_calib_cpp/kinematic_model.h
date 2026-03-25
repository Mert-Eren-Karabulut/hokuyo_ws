#pragma once
// =========================================================================
//  ptu_calib_cpp/kinematic_model.h
//
//  Encodes the URDF kinematic chain of the real_pantilt robot:
//
//      map ─(fixed)─▸ base_link
//           ─(joint2, revolute Z⁻)─▸ pan_link
//           ─(joint1, revolute Z⁺, origin rpy="0 1.57 0")─▸ tilt_link
//           ─(hokuyo_base_joint, fixed)─▸ hokuyo_base
//           ─(hokuyo_joint, fixed)─▸ laser
//
//  All nominal values are read from the URDF that is already loaded on the
//  ROS parameter server (/robot_description).  The struct also holds the
//  calibration correction parameters (deltas) that the solver fills in.
//
//  Convention:
//      A "correction" dT is RIGHT-multiplied onto the nominal transform:
//          T_corrected = T_nominal * dT(dt, dr)
//      where dr = (roll, pitch, yaw) in fixed-axis XYZ order.
//
//  The 2D laser points live in the "laser" frame with z = 0.
// =========================================================================

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <array>
#include <string>
#include <sstream>
#include <iomanip>

namespace ptu_calib {

// ──────────────────────── helpers ────────────────────────────────────

/// Build a 4×4 homogeneous transform from xyz + rpy (URDF convention:
/// R = Rz(yaw) · Ry(pitch) · Rx(roll), i.e. extrinsic XYZ).
inline Eigen::Matrix4d make_tf(double x, double y, double z,
                                double roll, double pitch, double yaw)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) =
        (Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX())).toRotationMatrix();
    T(0,3) = x;  T(1,3) = y;  T(2,3) = z;
    return T;
}

/// Rotation about Z only (4×4).
inline Eigen::Matrix4d Rz4(double angle)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    double c = std::cos(angle), s = std::sin(angle);
    T(0,0) =  c;  T(0,1) = -s;
    T(1,0) =  s;  T(1,1) =  c;
    return T;
}

/// Apply a small (dt, dr) correction to a nominal transform.
/// T_corrected = T_nominal · dT(dt, dr)
inline Eigen::Matrix4d apply_correction(const Eigen::Matrix4d& T_nom,
                                         const Eigen::Vector3d& dt,
                                         const Eigen::Vector3d& dr)
{
    Eigen::Matrix4d dT = make_tf(dt.x(), dt.y(), dt.z(),
                                  dr.x(), dr.y(), dr.z());
    return T_nom * dT;
}

/// Extract (roll, pitch, yaw) from a 3×3 rotation in URDF extrinsic-XYZ
/// convention.  Uses atan2 so angles are in [-π, π] (no Eigen branch issue).
inline Eigen::Vector3d rotation_to_rpy(const Eigen::Matrix3d& R)
{
    double pitch = std::atan2(-R(2,0),
                              std::sqrt(R(0,0)*R(0,0) + R(1,0)*R(1,0)));
    double roll, yaw;
    double cp = std::cos(pitch);
    if (std::abs(cp) > 1e-6) {
        roll = std::atan2(R(2,1), R(2,2));
        yaw  = std::atan2(R(1,0), R(0,0));
    } else {
        // Gimbal lock
        roll = std::atan2(-R(1,2), R(1,1));
        yaw  = 0.0;
    }
    return Eigen::Vector3d(roll, pitch, yaw);
}

// ──────────────────────── nominal transforms ────────────────────────

// All values straight from hokuyo_go/urdf/real_pantilt.urdf:

// joint2: base_link → pan_link
//   origin xyz="0 -0.0285 0.092"  rpy="0 0 0"
//   axis   xyz="0 0 -1"
struct PanJointNominal {
    static constexpr double x = 0.0,    y = -0.0285, z = 0.092;
    static constexpr double roll = 0.0, pitch = 0.0, yaw = 0.0;
    // axis is (0 0 -1): rotation about –Z ⇒ we negate the angle in FK
};

// joint1: pan_link → tilt_link
//   origin xyz="0 -0.009 0.022"  rpy="0 1.57 0"
//   axis   xyz="0 0 1"
struct TiltJointNominal {
    static constexpr double x = 0.0,    y = -0.009,  z = 0.022;
    static constexpr double roll = 0.0, pitch = 1.57, yaw = 0.0;
    // axis is (0 0 1): positive rotation about Z
};

// hokuyo_base_joint: tilt_link → hokuyo_base
//   origin xyz="-0.045 0 0"  rpy="0 0 0"
struct HokuyoBaseJointNominal {
    static constexpr double x = -0.045, y = 0.0, z = 0.0;
    static constexpr double roll = 0.0, pitch = 0.0, yaw = 0.0;
};

// hokuyo_joint: hokuyo_base → laser
//   origin xyz="-0.061 0.014 0.040"  rpy="1.57 3.14 -1.57"
struct HokuyoJointNominal {
    static constexpr double x = -0.061, y = 0.014, z = 0.040;
    static constexpr double roll = 1.57, pitch = 3.14, yaw = -1.57;
};

// ──────────────────────── calibration parameters ────────────────────

/// Correction parameters for a single fixed transform (6 DOF).
struct TransformCorrection {
    Eigen::Vector3d dt = Eigen::Vector3d::Zero();  // translation (m)
    Eigen::Vector3d dr = Eigen::Vector3d::Zero();  // rotation (rad)

    double* data() { return dt.data(); }  // pointer to 6 contiguous doubles (dt then dr)

    /// Pack into a flat array [dt(3), dr(3)]
    std::array<double,6> to_array() const {
        return {dt.x(), dt.y(), dt.z(), dr.x(), dr.y(), dr.z()};
    }
    void from_array(const double* p) {
        dt = Eigen::Vector3d(p[0], p[1], p[2]);
        dr = Eigen::Vector3d(p[3], p[4], p[5]);
    }
};

/// All calibration outputs.
struct CalibResult {
    // Pan joint (joint2) correction: 6 DOF origin + 1 encoder zero
    TransformCorrection pan_corr;
    double              pan_encoder_zero = 0.0;   // rad

    // Tilt joint (joint1) correction: 6 DOF origin + 1 encoder zero
    TransformCorrection tilt_corr;
    double              tilt_encoder_zero = 0.0;  // rad

    // Laser mount correction (composite hokuyo_base+hokuyo_joint): 6 DOF
    TransformCorrection laser_corr;

    // Solver diagnostics
    double pan_cost_initial  = 0.0, pan_cost_final  = 0.0;
    double tilt_cost_initial = 0.0, tilt_cost_final = 0.0;
    int    pan_iterations = 0,  tilt_iterations = 0;
};

// ──────────────────────── forward kinematics ────────────────────────

/// Compute the full chain  base_link → laser  for given joint angles and
/// calibration corrections.
///
///   T = T_pan_corrected · Rz(-(θ_pan + enc_pan)) ·
///       T_tilt_corrected · Rz(+(θ_tilt + enc_tilt)) ·
///       T_laser_corrected
///
inline Eigen::Matrix4d forward_kinematics(
    double theta_pan,  double theta_tilt,
    const TransformCorrection& pan_corr,   double enc_pan,
    const TransformCorrection& tilt_corr,  double enc_tilt,
    const TransformCorrection& laser_corr)
{
    // Nominal static transforms
    static const Eigen::Matrix4d T_pan_nom  = make_tf(
        PanJointNominal::x, PanJointNominal::y, PanJointNominal::z,
        PanJointNominal::roll, PanJointNominal::pitch, PanJointNominal::yaw);

    static const Eigen::Matrix4d T_tilt_nom = make_tf(
        TiltJointNominal::x, TiltJointNominal::y, TiltJointNominal::z,
        TiltJointNominal::roll, TiltJointNominal::pitch, TiltJointNominal::yaw);

    // Composite nominal laser transform: hokuyo_base_joint · hokuyo_joint
    static const Eigen::Matrix4d T_laser_nom = [](){
        Eigen::Matrix4d Tb = make_tf(
            HokuyoBaseJointNominal::x, HokuyoBaseJointNominal::y, HokuyoBaseJointNominal::z,
            HokuyoBaseJointNominal::roll, HokuyoBaseJointNominal::pitch, HokuyoBaseJointNominal::yaw);
        Eigen::Matrix4d Tj = make_tf(
            HokuyoJointNominal::x, HokuyoJointNominal::y, HokuyoJointNominal::z,
            HokuyoJointNominal::roll, HokuyoJointNominal::pitch, HokuyoJointNominal::yaw);
        return Tb * Tj;
    }();

    // Apply corrections
    Eigen::Matrix4d T_pan   = apply_correction(T_pan_nom,   pan_corr.dt,   pan_corr.dr);
    Eigen::Matrix4d T_tilt  = apply_correction(T_tilt_nom,  tilt_corr.dt,  tilt_corr.dr);
    Eigen::Matrix4d T_laser = apply_correction(T_laser_nom, laser_corr.dt, laser_corr.dr);

    // Joint rotations (with encoder offsets)
    // Pan axis is (0 0 -1) ⇒ rotation by −(θ + enc)
    Eigen::Matrix4d R_pan  = Rz4(-(theta_pan  + enc_pan));
    // Tilt axis is (0 0 1) ⇒ rotation by +(θ + enc)
    Eigen::Matrix4d R_tilt = Rz4(  theta_tilt + enc_tilt);

    return T_pan * R_pan * T_tilt * R_tilt * T_laser;
}

/// Overload that takes a CalibResult.
inline Eigen::Matrix4d forward_kinematics(
    double theta_pan, double theta_tilt, const CalibResult& cal)
{
    return forward_kinematics(theta_pan, theta_tilt,
                              cal.pan_corr,   cal.pan_encoder_zero,
                              cal.tilt_corr,  cal.tilt_encoder_zero,
                              cal.laser_corr);
}

/// Nominal FK (zero corrections).
inline Eigen::Matrix4d forward_kinematics_nominal(
    double theta_pan, double theta_tilt)
{
    CalibResult zero;
    return forward_kinematics(theta_pan, theta_tilt, zero);
}

// ──────────────────────── Ceres-compatible templated FK ──────────────

/// Templated version for use inside Ceres cost functors.
/// Works with both double and ceres::Jet types.
template<typename T>
Eigen::Matrix<T,4,4> make_tf_t(T x, T y, T z, T roll, T pitch, T yaw)
{
    using std::cos; using std::sin;
    // Rz(yaw) * Ry(pitch) * Rx(roll)
    T cr = cos(roll),  sr = sin(roll);
    T cp = cos(pitch), sp = sin(pitch);
    T cy = cos(yaw),   sy = sin(yaw);

    Eigen::Matrix<T,4,4> M = Eigen::Matrix<T,4,4>::Identity();
    M(0,0) = cy*cp;   M(0,1) = cy*sp*sr - sy*cr;   M(0,2) = cy*sp*cr + sy*sr;
    M(1,0) = sy*cp;   M(1,1) = sy*sp*sr + cy*cr;   M(1,2) = sy*sp*cr - cy*sr;
    M(2,0) = -sp;     M(2,1) = cp*sr;               M(2,2) = cp*cr;
    M(0,3) = x;       M(1,3) = y;                   M(2,3) = z;
    return M;
}

template<typename T>
Eigen::Matrix<T,4,4> Rz4_t(T angle)
{
    using std::cos; using std::sin;
    Eigen::Matrix<T,4,4> M = Eigen::Matrix<T,4,4>::Identity();
    T c = cos(angle), s = sin(angle);
    M(0,0) =  c;  M(0,1) = -s;
    M(1,0) =  s;  M(1,1) =  c;
    return M;
}

/// Templated FK for Ceres auto-diff.
///
/// Parameter blocks layout:
///   pan_params[7]  = [dt_pan(3), dr_pan(3), enc_pan(1)]
///   tilt_params[7] = [dt_tilt(3), dr_tilt(3), enc_tilt(1)]
///   laser_params[6]= [dt_laser(3), dr_laser(3)]
///
template<typename T>
Eigen::Matrix<T,4,4> forward_kinematics_t(
    T theta_pan, T theta_tilt,
    const T* pan_params,    // [7]: dt(3) dr(3) enc(1)
    const T* tilt_params,   // [7]: dt(3) dr(3) enc(1)
    const T* laser_params)  // [6]: dt(3) dr(3)
{
    // Nominal values as T
    T pan_x  = T(PanJointNominal::x),   pan_y  = T(PanJointNominal::y),   pan_z  = T(PanJointNominal::z);
    T pan_r  = T(PanJointNominal::roll), pan_p  = T(PanJointNominal::pitch), pan_w = T(PanJointNominal::yaw);
    T tilt_x = T(TiltJointNominal::x),  tilt_y = T(TiltJointNominal::y),  tilt_z = T(TiltJointNominal::z);
    T tilt_r = T(TiltJointNominal::roll),tilt_p = T(TiltJointNominal::pitch),tilt_w= T(TiltJointNominal::yaw);

    // Composite nominal laser
    T hb_x = T(HokuyoBaseJointNominal::x), hb_y = T(HokuyoBaseJointNominal::y), hb_z = T(HokuyoBaseJointNominal::z);
    T hb_r = T(HokuyoBaseJointNominal::roll), hb_p = T(HokuyoBaseJointNominal::pitch), hb_w = T(HokuyoBaseJointNominal::yaw);
    T hj_x = T(HokuyoJointNominal::x), hj_y = T(HokuyoJointNominal::y), hj_z = T(HokuyoJointNominal::z);
    T hj_r = T(HokuyoJointNominal::roll), hj_p = T(HokuyoJointNominal::pitch), hj_w = T(HokuyoJointNominal::yaw);

    auto T_laser_nom = make_tf_t<T>(hb_x,hb_y,hb_z,hb_r,hb_p,hb_w) *
                       make_tf_t<T>(hj_x,hj_y,hj_z,hj_r,hj_p,hj_w);

    // Correction dTs
    auto dT_pan   = make_tf_t<T>(pan_params[0], pan_params[1], pan_params[2],
                                  pan_params[3], pan_params[4], pan_params[5]);
    auto dT_tilt  = make_tf_t<T>(tilt_params[0], tilt_params[1], tilt_params[2],
                                  tilt_params[3], tilt_params[4], tilt_params[5]);
    auto dT_laser = make_tf_t<T>(laser_params[0], laser_params[1], laser_params[2],
                                  laser_params[3], laser_params[4], laser_params[5]);

    // Corrected static transforms: T_nom * dT
    auto T_pan_c   = make_tf_t<T>(pan_x,pan_y,pan_z,pan_r,pan_p,pan_w) * dT_pan;
    auto T_tilt_c  = make_tf_t<T>(tilt_x,tilt_y,tilt_z,tilt_r,tilt_p,tilt_w) * dT_tilt;
    auto T_laser_c = T_laser_nom * dT_laser;

    // Joint rotations
    T enc_pan  = pan_params[6];
    T enc_tilt = tilt_params[6];
    auto R_pan  = Rz4_t<T>(-(theta_pan  + enc_pan));
    auto R_tilt = Rz4_t<T>(  theta_tilt + enc_tilt);

    return T_pan_c * R_pan * T_tilt_c * R_tilt * T_laser_c;
}

// ──────────────────────── URDF generation ───────────────────────────

/// Generate corrected URDF joint blocks as a string.
inline std::string generate_calibrated_urdf(const CalibResult& cal)
{
    // Compute corrected absolute transforms
    static const Eigen::Matrix4d T_pan_nom = make_tf(
        PanJointNominal::x, PanJointNominal::y, PanJointNominal::z,
        PanJointNominal::roll, PanJointNominal::pitch, PanJointNominal::yaw);
    static const Eigen::Matrix4d T_tilt_nom = make_tf(
        TiltJointNominal::x, TiltJointNominal::y, TiltJointNominal::z,
        TiltJointNominal::roll, TiltJointNominal::pitch, TiltJointNominal::yaw);
    static const Eigen::Matrix4d T_laser_nom = [](){
        auto Tb = make_tf(HokuyoBaseJointNominal::x, HokuyoBaseJointNominal::y, HokuyoBaseJointNominal::z,
                          HokuyoBaseJointNominal::roll, HokuyoBaseJointNominal::pitch, HokuyoBaseJointNominal::yaw);
        auto Tj = make_tf(HokuyoJointNominal::x, HokuyoJointNominal::y, HokuyoJointNominal::z,
                          HokuyoJointNominal::roll, HokuyoJointNominal::pitch, HokuyoJointNominal::yaw);
        return Tb * Tj;
    }();

    Eigen::Matrix4d T_pan_cal   = apply_correction(T_pan_nom,   cal.pan_corr.dt,   cal.pan_corr.dr);
    Eigen::Matrix4d T_tilt_cal  = apply_correction(T_tilt_nom,  cal.tilt_corr.dt,  cal.tilt_corr.dr);
    Eigen::Matrix4d T_laser_cal = apply_correction(T_laser_nom, cal.laser_corr.dt, cal.laser_corr.dr);

    auto xyz_rpy = [](const Eigen::Matrix4d& T) -> std::pair<Eigen::Vector3d, Eigen::Vector3d> {
        Eigen::Vector3d t = T.block<3,1>(0,3);
        Eigen::Vector3d rpy = rotation_to_rpy(T.block<3,3>(0,0));
        return {t, rpy};
    };

    auto [pan_t, pan_r]   = xyz_rpy(T_pan_cal);
    auto [tilt_t, tilt_r] = xyz_rpy(T_tilt_cal);
    auto [las_t, las_r]   = xyz_rpy(T_laser_cal);

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(6);

    ss << "<?xml version=\"1.0\"?>\n"
       << "<!--\n"
       << "    CALIBRATED real_pantilt URDF\n"
       << "    Generated by ptu_calib_cpp\n"
       << "    Pan  encoder zero offset: " << cal.pan_encoder_zero  << " rad ("
       << (cal.pan_encoder_zero * 180.0/M_PI) << " deg)\n"
       << "    Tilt encoder zero offset: " << cal.tilt_encoder_zero << " rad ("
       << (cal.tilt_encoder_zero * 180.0/M_PI) << " deg)\n"
       << "-->\n"
       << "<robot name=\"real_pantilt\" xmlns:xacro=\"http://ros.org/wiki/xacro\">\n\n";

    // map link
    ss << "    <link name=\"map\">\n"
       << "        <inertial>\n"
       << "            <mass value=\"0.001\" />\n"
       << "            <origin xyz=\"0 0 0\" rpy=\"0 0 0\" />\n"
       << "            <inertia ixx=\"0.0001\" ixy=\"0\" ixz=\"0\" iyy=\"0.0001\" iyz=\"0\" izz=\"0.0001\" />\n"
       << "        </inertial>\n"
       << "    </link>\n\n";

    // base_link
    ss << "    <link name=\"base_link\">\n"
       << "        <visual>\n"
       << "            <origin xyz=\"0.0 -0.05 0.035\" rpy=\"0 0 0\" />\n"
       << "            <geometry><box size=\"0.0625 0.095 0.07\" /></geometry>\n"
       << "            <material name=\"Cyan1\"><color rgba=\"0 0.9 0.9 1.0\" /></material>\n"
       << "        </visual>\n"
       << "        <inertial>\n"
       << "            <mass value=\"1\" />\n"
       << "            <origin xyz=\"0 0 0\" rpy=\"0 0 0\" />\n"
       << "            <inertia ixx=\"0.01\" ixy=\"0\" ixz=\"0\" iyy=\"0.01\" iyz=\"0\" izz=\"0.01\" />\n"
       << "        </inertial>\n"
       << "    </link>\n\n";

    // map → base_link
    ss << "    <joint name=\"map_to_base\" type=\"fixed\">\n"
       << "        <origin xyz=\"0 0 0\" rpy=\"0 0 0\" />\n"
       << "        <parent link=\"map\" />\n"
       << "        <child link=\"base_link\" />\n"
       << "    </joint>\n\n";

    // pan_link
    ss << "    <link name=\"pan_link\">\n"
       << "        <visual>\n"
       << "            <origin xyz=\"0 0 0\" rpy=\"0 0 0\" />\n"
       << "            <geometry><box size=\"0.035 0.038 0.044\" /></geometry>\n"
       << "            <material name=\"Yellow2\"><color rgba=\"0.8 0.8 0 1.0\" /></material>\n"
       << "        </visual>\n"
       << "        <inertial>\n"
       << "            <mass value=\"0.50\" />\n"
       << "            <origin xyz=\"0 0 0\" rpy=\"0 0 0\" />\n"
       << "            <inertia ixx=\"0.01\" ixy=\"0\" ixz=\"0\" iyy=\"0.01\" iyz=\"0\" izz=\"0.01\" />\n"
       << "        </inertial>\n"
       << "    </link>\n\n";

    // CALIBRATED joint2 (pan)
    ss << "    <!-- CALIBRATED joint2 (pan): base_link -> pan_link -->\n"
       << "    <joint name=\"joint2\" type=\"revolute\">\n"
       << "        <parent link=\"base_link\" />\n"
       << "        <child link=\"pan_link\" />\n"
       << "        <origin xyz=\"" << pan_t.x() << " " << pan_t.y() << " " << pan_t.z() << "\"\n"
       << "                rpy=\"" << pan_r.x() << " " << pan_r.y() << " " << pan_r.z() << "\" />\n"
       << "        <axis xyz=\"0 0 -1\" />\n"
       << "        <limit effort=\"10\" lower=\"-3.1416\" upper=\"3.1416\" velocity=\"3\" />\n"
       << "        <dynamics damping=\"1.0\" />\n"
       << "    </joint>\n\n";

    // tilt_link
    ss << "    <link name=\"tilt_link\">\n"
       << "        <visual>\n"
       << "            <origin xyz=\"-0.022 0 0\" rpy=\"0 -1.57 0\" />\n"
       << "            <geometry><box size=\"0.005 0.005 0.044\" /></geometry>\n"
       << "            <material name=\"Cyan1\"><color rgba=\"0 0.9 0.9 1.0\" /></material>\n"
       << "        </visual>\n"
       << "        <inertial>\n"
       << "            <mass value=\"0.20\" />\n"
       << "            <origin xyz=\"0 0 0\" rpy=\"0 0 0\" />\n"
       << "            <inertia ixx=\"0.01\" ixy=\"0\" ixz=\"0\" iyy=\"0.01\" iyz=\"0\" izz=\"0.01\" />\n"
       << "        </inertial>\n"
       << "    </link>\n\n";

    // CALIBRATED joint1 (tilt)
    ss << "    <!-- CALIBRATED joint1 (tilt): pan_link -> tilt_link -->\n"
       << "    <joint name=\"joint1\" type=\"revolute\">\n"
       << "        <parent link=\"pan_link\" />\n"
       << "        <child link=\"tilt_link\" />\n"
       << "        <origin xyz=\"" << tilt_t.x() << " " << tilt_t.y() << " " << tilt_t.z() << "\"\n"
       << "                rpy=\"" << tilt_r.x() << " " << tilt_r.y() << " " << tilt_r.z() << "\" />\n"
       << "        <axis xyz=\"0 0 1\" />\n"
       << "        <limit effort=\"10\" lower=\"-0.8727\" upper=\"0.8727\" velocity=\"3\" />\n"
       << "        <dynamics damping=\"1.0\" />\n"
       << "    </joint>\n\n";

    // laser link
    ss << "    <link name=\"laser\">\n"
       << "        <visual>\n"
       << "            <origin xyz=\"0.0 0.0 -0.059\" rpy=\"1.57 0 0\" />\n"
       << "            <geometry>\n"
       << "                <mesh filename=\"/home/isl9/dev/mert/hokuyo_ws/src/hokuyo_go/urdf/meshes/URG-04LX.dae\" scale=\"0.001 0.001 0.001\" />\n"
       << "            </geometry>\n"
       << "            <material name=\"Cyan1\"><color rgba=\"0 0.9 0.9 1.0\" /></material>\n"
       << "        </visual>\n"
       << "        <inertial>\n"
       << "            <mass value=\"0.16\" />\n"
       << "            <origin xyz=\"0 0 0\" rpy=\"0 0 0\" />\n"
       << "            <inertia ixx=\"0.001\" ixy=\"0\" ixz=\"0\" iyy=\"0.001\" iyz=\"0\" izz=\"0.001\" />\n"
       << "        </inertial>\n"
       << "    </link>\n\n";

    // CALIBRATED laser joint (collapsed hokuyo_base_joint + hokuyo_joint)
    ss << "    <!-- CALIBRATED laser mount: tilt_link -> laser (collapsed from hokuyo_base_joint + hokuyo_joint) -->\n"
       << "    <joint name=\"hokuyo_joint\" type=\"fixed\">\n"
       << "        <parent link=\"tilt_link\" />\n"
       << "        <child link=\"laser\" />\n"
       << "        <origin xyz=\"" << las_t.x() << " " << las_t.y() << " " << las_t.z() << "\"\n"
       << "                rpy=\"" << las_r.x() << " " << las_r.y() << " " << las_r.z() << "\" />\n"
       << "        <dynamics damping=\"1.0\" />\n"
       << "    </joint>\n\n";

    ss << "</robot>\n";
    return ss.str();
}

}  // namespace ptu_calib
