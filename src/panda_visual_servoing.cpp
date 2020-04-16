#include <ros/ros.h>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/QR>
#include <sophus/se3.hpp>
#include <math.h>
#include <mutex>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class SE3Pid {
public:
    SE3Pid():kp_(0.01), ki_(0.0), kd_(0.0), dt_(1), max_(0.1), integral_(Eigen::Matrix<double, 6, 1>::Zero()), pre_error_(Eigen::Matrix<double, 6, 1>::Zero()) {}
    ~SE3Pid() {}
    bool Init(const float& kp, const float& ki, const float& kd) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }
    bool Calc(const Eigen::Transform<double, 3, Eigen::Isometry>& pose_cur, const Eigen::Transform<double, 3, Eigen::Isometry>& pose_goal,
              Eigen::Transform<double, 3, Eigen::Isometry>& pose_delta)  {
        Sophus::SE3d se_goal = Sophus::SE3d(Eigen::Quaterniond(pose_goal.matrix().block<3, 3>(0, 0)), Eigen::Vector3d(pose_goal.matrix().block<3, 1>(0, 3)));
        Sophus::SE3d se_cur(Eigen::Quaterniond(pose_cur.matrix().block<3, 3>(0, 0)), Eigen::Vector3d(pose_cur.matrix().block<3, 1>(0, 3)));
        Eigen::Matrix<double, 6, 1> error = (se_cur.inverse() * se_goal).log();
        if (error.norm() < 1e-6) {
            return true;
        }
        // Proportional term
        Eigen::Matrix<double, 6, 1> p_out = kp_ * error;
        // Integral term
        integral_ += error * dt_;
        Eigen::Matrix<double, 6, 1> i_out = ki_ * integral_;
        // Derivative term
        Eigen::Matrix<double, 6, 1> derivative = (error - pre_error_) / dt_;
        Eigen::Matrix<double, 6, 1> d_out = kd_ * derivative;
        // Calculate total output
        Eigen::Matrix<double, 6, 1> diff = p_out + i_out + d_out;
        if (diff.norm() > max_) {
            ROS_INFO("pid is too large");
            diff = diff * max_/diff.norm();
        }
        Sophus::SE3d se_i = se_cur * Sophus::SE3d::exp(diff);
        pose_delta = se_i.matrix();
        pre_error_ = error;
        return false;
    }
    bool Calc(const Eigen::Transform<double, 3, Eigen::Isometry>& pose_cur, const Eigen::Transform<double, 3, Eigen::Isometry>& pose_goal,
              Eigen::Matrix<double, 6, 1>& cart_v)  {
        Sophus::SE3d se_goal = Sophus::SE3d(Eigen::Quaterniond(pose_goal.matrix().block<3, 3>(0, 0)), Eigen::Vector3d(pose_goal.matrix().block<3, 1>(0, 3)));
        Sophus::SE3d se_cur(Eigen::Quaterniond(pose_cur.matrix().block<3, 3>(0, 0)), Eigen::Vector3d(pose_cur.matrix().block<3, 1>(0, 3)));
        Eigen::Matrix<double, 6, 1> error = (se_cur.inverse() * se_goal).log();
        if (error.norm() < 1e-6) {
            return true;
        }
        // Proportional term
        Eigen::Matrix<double, 6, 1> p_out = kp_ * error;
        // Integral term
        integral_ += error * dt_;
        Eigen::Matrix<double, 6, 1> i_out = ki_ * integral_;
        // Derivative term
        Eigen::Matrix<double, 6, 1> derivative = (error - pre_error_) / dt_;
        Eigen::Matrix<double, 6, 1> d_out = kd_ * derivative;
        // Calculate total output
        Eigen::Matrix<double, 6, 1> diff = p_out + i_out + d_out;
        if (diff.norm() > max_) {
            ROS_INFO_STREAM("pid is too large: " << diff);
            diff = diff * max_/diff.norm();
        }
        cart_v = diff;
        pre_error_ = error;
        return false;
    }
private:
    float kp_;
    float ki_;
    float kd_;
    float dt_;
    float max_;
    Eigen::Matrix<double, 6, 1> pre_error_;
    Eigen::Matrix<double, 6, 1> integral_;
};

class Robot {
public:
    // Robot(ros::NodeHandle& nh):nh_(nh), planning_group_("panda_arm"), move_group_("panda_arm"), robot_model_loader_("robot_description") {
    Robot(ros::NodeHandle& nh):nh_(nh), planning_group_("panda_arm"), move_group_("panda_arm"), reference_point_position_(0.0, 0.0, 0.0) {
        // std::vector<double> joints = move_group_.getCurrentJointValues();
        std::vector<double> joints = {0, 0, 0, -M_PI/2, 0, M_PI/2, 0};
        move_group_.setStartStateToCurrentState();
        move_group_.setJointValueTarget(joints);
        move_group_.move();
        // kinematic_model_ = robot_model_loader_.getModel();
        // kinematic_state_.reset(new robot_state::RobotState(kinematic_model_));
        // kinematic_state_->getJacobian(kinematic_model_->getJointModelGroup("panda_arm"),
        //                        kinematic_state_->getLinkModel(kinematic_model_->getJointModelGroup("panda_arm")->getLinkModelNames().back()),
        //                        reference_point_position_, jacobian_);


        // kinematic_state_ = move_group_.getCurrentState();
        // kinematic_state_->getJacobian(kinematic_state_->getJointModelGroup("panda_arm"),
        //                        kinematic_state_->getLinkModel(kinematic_state_->getJointModelGroup("panda_arm")->getLinkModelNames().back()),
        //                        reference_point_position_, jacobian_);
        // ROS_INFO_STREAM("Jacobian: \n" << jacobian_ << "\n");
        // pose_msg_ = move_group_.getCurrentPose("panda_link8");
        // pose_eg_ = Eigen::Transform<double, 3, Eigen::Isometry>::Identity();
        // pose_eg_.rotate(Eigen::Quaterniond(pose_msg_.pose.orientation.w, pose_msg_.pose.orientation.x, pose_msg_.pose.orientation.y, pose_msg_.pose.orientation.z));
        // pose_eg_.pretranslate(Eigen::Vector3d(pose_msg_.pose.position.x, pose_msg_.pose.position.y, pose_msg_.pose.position.z));
        // ROS_INFO_STREAM("cur pose: " << pose_eg_.matrix());
        // Eigen::Matrix<double, 6, 1> cart_v;
        // cart_v << 0.01, 0, 0, 0, 0, 0;
        // Eigen::Matrix<double, 7, 1> jnt_v = jacobian_.transpose() * (jacobian_ * jacobian_.transpose()).inverse() * cart_v;
        // // Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cqr(jacobian);
        // // Eigen::Matrix<double, 7, 1> jnt_v = cqr.pseudoInverse() * cart_v;
        // ROS_INFO_STREAM("jnt_v: \n" << jnt_v << "\n");
        // std::vector<double> jnt = move_group_.getCurrentJointValues();
        // for (int i=0; i<7; i++) {
        //     jnt.at(i) += jnt_v(i, 0);
        // }
        // move_group_.setStartStateToCurrentState();
        // move_group_.setJointValueTarget(jnt);
        // move_group_.move();
        // pose_msg_ = move_group_.getCurrentPose("panda_link8");
        // pose_eg_ = Eigen::Transform<double, 3, Eigen::Isometry>::Identity();
        // pose_eg_.rotate(Eigen::Quaterniond(pose_msg_.pose.orientation.w, pose_msg_.pose.orientation.x, pose_msg_.pose.orientation.y, pose_msg_.pose.orientation.z));
        // pose_eg_.pretranslate(Eigen::Vector3d(pose_msg_.pose.position.x, pose_msg_.pose.position.y, pose_msg_.pose.position.z));
        // ROS_INFO_STREAM("cur pose: " << pose_eg_.matrix());
        // for (int i=0; i<10; i++) {
        //     kinematic_state_ = move_group_.getCurrentState();
        //     kinematic_state_->getJacobian(kinematic_state_->getJointModelGroup("panda_arm"),
        //                        kinematic_state_->getLinkModel(kinematic_state_->getJointModelGroup("panda_arm")->getLinkModelNames().back()),
        //                        reference_point_position_, jacobian_);
        //     jnt_v = jacobian_.transpose() * (jacobian_ * jacobian_.transpose()).inverse() * cart_v;
        //     jnt = move_group_.getCurrentJointValues();
        //     for (int i=0; i<7; i++) {
        //         jnt.at(i) += jnt_v(i, 0);
        //     }
        //     move_group_.setStartStateToCurrentState();
        //     move_group_.setJointValueTarget(jnt);
        //     move_group_.move();
        //     pose_msg_ = move_group_.getCurrentPose("panda_link8");
        //     pose_eg_ = Eigen::Transform<double, 3, Eigen::Isometry>::Identity();
        //     pose_eg_.rotate(Eigen::Quaterniond(pose_msg_.pose.orientation.w, pose_msg_.pose.orientation.x, pose_msg_.pose.orientation.y, pose_msg_.pose.orientation.z));
        //     pose_eg_.pretranslate(Eigen::Vector3d(pose_msg_.pose.position.x, pose_msg_.pose.position.y, pose_msg_.pose.position.z));
        //     ROS_INFO_STREAM("cur pose: " << pose_eg_.matrix());
        // }

    }
    ~Robot() {}
    Eigen::Transform<double, 3, Eigen::Isometry> bMe() {
        geometry_msgs::PoseStamped pose_msg = move_group_.getCurrentPose("panda_link8");
        Eigen::Transform<double, 3, Eigen::Isometry> pose = Eigen::Transform<double, 3, Eigen::Isometry>::Identity();
        pose.rotate(Eigen::Quaterniond(pose_msg.pose.orientation.w, pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z));
        pose.pretranslate(Eigen::Vector3d(pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z));
        return pose;
    }
    Eigen::MatrixXd jacobian() {
        kinematic_state_ = move_group_.getCurrentState();
        kinematic_state_->getJacobian(kinematic_state_->getJointModelGroup("panda_arm"),
                               kinematic_state_->getLinkModel(kinematic_state_->getJointModelGroup("panda_arm")->getLinkModelNames().back()),
                               reference_point_position_, jacobian_);
        return jacobian_;                       
    }
    bool Move(const Eigen::Matrix<double, 7, 1>& joint_v) {
        joint_position_ = move_group_.getCurrentJointValues();
        for (int i=0; i<7; i++) {
            joint_position_.at(i) += joint_v(i, 0);
        }
        move_group_.setStartStateToCurrentState();
        move_group_.setJointValueTarget(joint_position_);
        move_group_.move();
    }
private:
    ros::NodeHandle nh_;
    std::string planning_group_;
    moveit::planning_interface::MoveGroupInterface move_group_;
    // robot_model_loader::RobotModelLoader robot_model_loader_;
    // robot_model::RobotModelPtr kinematic_model_;
    robot_state::RobotStatePtr kinematic_state_;
    Eigen::Vector3d reference_point_position_;
    Eigen::MatrixXd jacobian_;
    std::vector<double> joint_position_;

    geometry_msgs::PoseStamped pose_msg_;
    Eigen::Transform<double, 3, Eigen::Isometry> pose_eg_;
};

class VisualServoing {
public:
    VisualServoing(ros::NodeHandle& nh):nh_(nh), robot_(nh) {

        cMo_sub_ = nh_.subscribe("board/pose", 1, &VisualServoing::CalibPoseCallback, this);
        eMc_ = Eigen::Transform<double, 3, Eigen::Isometry>::Identity();
        Eigen::Matrix<double, 3, 3> rot;
        rot << 0.0, 1.0, 0.0,
                -1.0, 0.0, 0.0,
                0.0, 0.0, 1.0;
        eMc_.rotate(Eigen::Quaterniond(rot));
        eMc_.pretranslate(Eigen::Vector3d(0.0, 0.0, 0.02));        
        cdMo_ = Eigen::Transform<double, 3, Eigen::Isometry>::Identity();
        rot << -1.0, 0.0, 0.0,
                0.0, -1.0, 0.0,
                0.0, 0.0, 1.0;
        cdMo_.rotate(Eigen::Quaterniond(rot));
        cdMo_.pretranslate(Eigen::Vector3d(0.1112315, 0.21367, 0.7107));

        cVe_ = Eigen::Matrix<double, 6, 6>::Zero();
        Eigen::Transform<double, 3, Eigen::Isometry> cMe = eMc_.inverse();
        Eigen::Matrix<double, 3, 3> t_skew;
        t_skew << 0, -cMe.matrix()(2, 3), cMe.matrix()(1, 3),
              cMe.matrix()(2, 3), 0, -cMe.matrix()(0, 3),
              -cMe.matrix()(1, 3), cMe.matrix()(0, 3), 0;
        Eigen::Matrix<double, 3, 3> r(cMe.matrix().block<3, 3>(0, 0));
        Eigen::Matrix<double, 3, 3> t_r = t_skew * r;
        for (unsigned int i = 0; i < 3; i++) {
            for (unsigned int j = 0; j < 3; j++) {
                cVe_(i, j) = r(i, j);
                cVe_(i+3, j+3) = r(i, j);
                cVe_(i, j+3) = t_r(i, j);
            }
        }
    }
    ~VisualServoing() {}
    void CalibPoseCallback(const geometry_msgs::PoseStamped& cMo_msg) {
        std::unique_lock<std::mutex> locker(mutex_);
        cMo_ = Eigen::Transform<double, 3, Eigen::Isometry>::Identity();
        cMo_.rotate(Eigen::Quaterniond(cMo_msg.pose.orientation.w, cMo_msg.pose.orientation.x, cMo_msg.pose.orientation.y, cMo_msg.pose.orientation.z));
        cMo_.pretranslate(Eigen::Vector3d(cMo_msg.pose.position.x, cMo_msg.pose.position.y, cMo_msg.pose.position.z));
        // ROS_INFO_STREAM("cMo_: " << cMo_.matrix());
        locker.unlock();
    }
    void Run() {
        bMe_ = robot_.bMe();
        bMc_ = bMe_ * eMc_;
        std::unique_lock<std::mutex> locker(mutex_);
        bMcd_ = bMc_ * cMo_ * cdMo_.inverse();
        locker.unlock();
        ROS_INFO_STREAM("bMc_:" << bMc_.matrix());
        ROS_INFO_STREAM("bMcd_:" << bMcd_.matrix());
        bool motion_finished = pid_.Calc(bMc_, bMcd_, cart_v_);
        if (motion_finished) {
            ROS_INFO_STREAM("motion_finished");
            return;
        }
        jacobian_ = robot_.jacobian();
        Eigen::Matrix<double, 6, 7> jv = cVe_ * jacobian_;
        Eigen::Matrix<double, 7, 1> q_dot = jv.transpose() * (jv * jv.transpose()).inverse() * cart_v_;
        // ROS_INFO_STREAM("q_dot: " << q_dot);
        robot_.Move(q_dot);
    }
private:
    ros::NodeHandle nh_;
    ros::Subscriber cMo_sub_;
    Robot robot_;
    SE3Pid pid_;
    Eigen::Transform<double, 3, Eigen::Isometry> bMe_;
    Eigen::Transform<double, 3, Eigen::Isometry> bMc_;
    Eigen::Transform<double, 3, Eigen::Isometry> eMc_;
    Eigen::Transform<double, 3, Eigen::Isometry> bMcd_;
    Eigen::Transform<double, 3, Eigen::Isometry> cdMo_;
    Eigen::Transform<double, 3, Eigen::Isometry> cMo_;
    Eigen::Matrix<double, 6, 1> cart_v_;
    Eigen::MatrixXd jacobian_;
    Eigen::Matrix<double, 6, 6> cVe_;
    std::mutex mutex_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "panda_visual_servoing");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ROS_INFO("hello panda_visual_servoing");

    VisualServoing vs(nh);
    while(1) {
        vs.Run();
    }

    ros::waitForShutdown();
    return 0;
}