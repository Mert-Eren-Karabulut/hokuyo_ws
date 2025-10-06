//This code implements parametric scanning for the pan-tilt mechanism using constant velocity motion.
#include <ros/ros.h>
#include <math.h>
#include <algorithm>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>


#define PI 3.14159265359

// Current joint positions
double tiltangle = 0.0;
double panangle = 0.0;

// Target joint positions (calculated from parametric equations)
double tiltangle_goal = 0.0;
double panangle_goal = 0.0;

// Parametric scanning variables
double t_param = 0.0;             // Parameter time for scanning equations
double v_target = 1.0;            // Target velocity in rad/s
double delta_t_frame = 0.02;      // Frame time (50Hz)
double delta_1 = 60.0 * PI / 180.0;  // Yaw limit (60 degrees)
double delta_2 = 45.0 * PI / 180.0;  // Tilt limit (45 degrees)
double sqrt2_over_100 = sqrt(2.0) / 100.0;  // Irrational frequency component

// Parametric scanning functions from the paper
double f1(double t) {
    // f1(t) = sin(t) - Yaw function scaled to delta_1 limits (±60 degrees)
    return delta_1 * sin(t);
}

double f2(double t) {
    // f2(t) = (π/2) * (cos((3 + √2/100)t) + 1)/2 - π/4 - Tilt function from paper with non-repetitive component
    double freq = 3.0 + sqrt2_over_100;
    return (PI/2.0) * (cos(freq * t) + 1.0) / 2.0 - PI/4.0;
}

// Derivative functions for velocity calculation
double df1_dt(double t, double dt) {
    return (f1(t + dt) - f1(t)) / dt;
}

double df2_dt(double t, double dt) {
    return (f2(t + dt) - f2(t)) / dt;
}

// Calculate instantaneous speed for constant velocity implementation
double calculate_speed(double t, double dt) {
    double df1 = df1_dt(t, dt);
    double df2 = df2_dt(t, dt);
    return sqrt(df1 * df1 + df2 * df2);
}

// Update parameter time for constant velocity scanning
void update_parameter_time() {
    double dt_small = 0.001;  // Small dt for derivative calculation
    double current_speed = calculate_speed(t_param, dt_small);
    
    if (current_speed > 0.0) {
        double delta_s = v_target * delta_t_frame;  // Desired arc length
        double delta_t_param = delta_s / current_speed;  // Parameter time increment
        t_param += delta_t_param;
    } else {
        t_param += delta_t_frame;  // Fallback if speed is zero
    }
}

// Get target angles using parametric scanning
void get_parametric_targets(double& pan_target, double& tilt_target) {
    pan_target = f1(t_param);
    tilt_target = f2(t_param);
    
    // Clamp to physical limits
    // pan_target = std::max(-delta_1, std::min(delta_1, pan_target));
    // tilt_target = std::max(-delta_2, std::min(delta_2, tilt_target));
}

// Gets the current position and updates target positions using parametric scanning
// Gets the current position and updates target positions using parametric scanning
void encoCallback(const sensor_msgs::JointState::ConstPtr& msg){

    // Get current joint positions
    tiltangle = msg->position[1];
    panangle = msg->position[0];

    // Update parameter time for constant velocity
    update_parameter_time();
    
    // Get target angles from parametric equations
    get_parametric_targets(panangle_goal, tiltangle_goal);
    
    // Optional: Print current scanning parameters for debugging
    if (fmod(t_param, 1.0) < 0.02) {  // Print every ~1 radian of parameter time
        ROS_INFO("Parametric scan - t_param: %.2f, pan_goal: %.3f, tilt_goal: %.3f", 
                 t_param, panangle_goal, tiltangle_goal);
    }
}

int main(int argc, char** argv){
    
    ros::init(argc, argv, "vel_publisher");
    ros::NodeHandle nh;
    
    // Get scanning parameters
    nh.param("target_velocity", v_target, 1.0);
    nh.param("yaw_limit_deg", delta_1, 60.0);
    nh.param("tilt_limit_deg", delta_2, 45.0);
    
    // Convert degrees to radians
    delta_1 = delta_1 * PI / 180.0;
    delta_2 = delta_2 * PI / 180.0;
    
    ROS_INFO("Parametric scanning velocity publisher started");
    ROS_INFO("Target velocity: %.2f rad/s", v_target);
    ROS_INFO("Yaw limit: ±%.1f degrees", delta_1 * 180.0 / PI);
    ROS_INFO("Tilt limit: ±%.1f degrees", delta_2 * 180.0 / PI);
    
    // Subscribe to joint states
    ros::Subscriber sub = nh.subscribe("/joint_states", 300, encoCallback);
    
    // Publisher for joint trajectory commands
    ros::Publisher vel_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/gazebo_ros_control/command", 1000);
    
    trajectory_msgs::JointTrajectory joint_traj;
    trajectory_msgs::JointTrajectoryPoint traj_point;
    joint_traj.joint_names = {"pan_joint", "tilt_joint"};
    
    ros::Rate loop(50);  // 50Hz control loop
    
    while(ros::ok()){
        joint_traj.header.stamp = ros::Time::now();
        traj_point.time_from_start = ros::Duration(0.02); // 50Hz = 20ms
        traj_point.positions = {panangle_goal, tiltangle_goal};
        
        joint_traj.points.clear();
        joint_traj.points.push_back(traj_point);
        
        vel_pub.publish(joint_traj);
        
        ros::spinOnce();
        loop.sleep();
    }
    
    return 0;
}
