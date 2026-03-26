// =========================================================================
//  ptu_calib_cpp – sim_joint_controller.cpp
//
//  Minimal position controller for Gazebo simulation.
//  Bridges the real robot's interface (subscribe /joint_command,
//  publish /joint_states) to Gazebo joint effort commands.
//
//  Uses a simple PD controller to drive joint2 (pan) and joint1 (tilt)
//  to their commanded positions.
// =========================================================================

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/GetJointProperties.h>
#include <gazebo_msgs/ApplyJointEffort.h>
#include <gazebo_msgs/GetModelState.h>
#include <std_msgs/Float64.h>

#include <mutex>
#include <string>
#include <cmath>
#include <map>

class SimJointController {
public:
    SimJointController(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh)
    {
        // PD gains
        pnh.param("kp", kp_, 2.0);
        pnh.param("kd", kd_, 0.1);
        pnh.param("rate", rate_, 100.0);

        // Joint names (same as real robot)
        joints_ = {"joint1", "joint2"};  // tilt, pan

        // Commanded positions (start at 0)
        for (auto& j : joints_) cmd_pos_[j] = 0.0;

        // ROS I/O
        cmd_sub_ = nh_.subscribe("/joint_command", 10,
                                  &SimJointController::cmdCB, this);
        js_pub_  = nh_.advertise<sensor_msgs::JointState>("/joint_states", 50);

        // Gazebo services
        get_joint_client_ = nh_.serviceClient<gazebo_msgs::GetJointProperties>(
            "/gazebo/get_joint_properties");
        effort_client_ = nh_.serviceClient<gazebo_msgs::ApplyJointEffort>(
            "/gazebo/apply_joint_effort");

        ROS_INFO("SimJointController: waiting for Gazebo services...");
        get_joint_client_.waitForExistence(ros::Duration(30.0));
        effort_client_.waitForExistence(ros::Duration(30.0));
        ROS_INFO("SimJointController: Gazebo services available");
    }

    void spin()
    {
        ros::Rate r(rate_);
        ros::Duration effort_dur(1.0 / rate_);

        while (ros::ok()) {
            ros::spinOnce();

            sensor_msgs::JointState js;
            js.header.stamp = ros::Time::now();

            for (auto& jname : joints_) {
                // Read current joint state from Gazebo
                gazebo_msgs::GetJointProperties gjp;
                gjp.request.joint_name = jname;
                double pos = 0.0, vel = 0.0;
                if (get_joint_client_.call(gjp) && gjp.response.success) {
                    if (!gjp.response.position.empty()) pos = gjp.response.position[0];
                    if (!gjp.response.rate.empty())     vel = gjp.response.rate[0];
                }

                // PD control
                double target;
                {
                    std::lock_guard<std::mutex> lk(mutex_);
                    target = cmd_pos_[jname];
                }
                double error = target - pos;
                double effort = kp_ * error - kd_ * vel;

                // Clamp effort
                double max_eff = 5.0;
                effort = std::max(-max_eff, std::min(max_eff, effort));

                // Apply effort
                gazebo_msgs::ApplyJointEffort aje;
                aje.request.joint_name = jname;
                aje.request.effort = effort;
                aje.request.start_time = ros::Time(0);
                aje.request.duration = effort_dur;
                effort_client_.call(aje);

                // Publish
                js.name.push_back(jname);
                js.position.push_back(pos);
                js.velocity.push_back(vel);
                js.effort.push_back(effort);
            }

            js_pub_.publish(js);
            r.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber cmd_sub_;
    ros::Publisher  js_pub_;
    ros::ServiceClient get_joint_client_, effort_client_;

    std::vector<std::string> joints_;
    std::map<std::string, double> cmd_pos_;
    std::mutex mutex_;
    double kp_, kd_, rate_;

    void cmdCB(const sensor_msgs::JointState::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lk(mutex_);
        for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
            if (cmd_pos_.count(msg->name[i])) {
                cmd_pos_[msg->name[i]] = msg->position[i];
            }
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sim_joint_controller");
    ros::NodeHandle nh, pnh("~");
    SimJointController ctrl(nh, pnh);
    ctrl.spin();
    return 0;
}
