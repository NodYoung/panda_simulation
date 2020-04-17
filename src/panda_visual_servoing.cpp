#include <ros/ros.h>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/QR>
#include <sophus/se3.hpp>
#include <math.h>
#include <mutex>
#include <atomic>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// visp
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/vs/vpServo.h>
#include <visp3/visual_features/vpFeatureThetaU.h>
#include <visp3/visual_features/vpFeatureTranslation.h>
#include <visp3/core/vpHomogeneousMatrix.h>

class PBVS {
public:
    PBVS():s_t_(vpFeatureTranslation::cMo), s_tu_(vpFeatureThetaU::cdRc),
                              s_star_t_(vpFeatureTranslation::cMo), s_star_tu_(vpFeatureThetaU::cdRc),
                              v_c_(6) {}
    ~PBVS() {task_.kill();}
    void Init() {
        cdMo_[0][0] = -1; cdMo_[0][1] = 0; cdMo_[0][2] =  0; cdMo_[0][3] = 0.1112315;
        cdMo_[1][0] = 0; cdMo_[1][1] = -1; cdMo_[1][2] =  0; cdMo_[1][3] = 0.21367;
        cdMo_[2][0] = 0; cdMo_[2][1] =  0; cdMo_[2][2] = 1; cdMo_[2][3] = 0.6;
        s_star_t_.buildFrom(cdMo_);
        task_.setServo(vpServo::EYEINHAND_CAMERA);    // Camera is monted on the robot end-effector and velocities are computed in the camera frame
        task_.setInteractionMatrixType(vpServo::CURRENT);    // Interaction matrix is computed with the current visual features s
        task_.setLambda(0.1);         // Set the contant 0.5
        task_.addFeature(s_t_, s_star_t_);          // Add current and desired translation feature
        task_.addFeature(s_tu_, s_star_tu_);           // Add current and desired ThetaU feature for the rotation
    }
    bool Calc(const Eigen::Transform<double, 3, Eigen::Isometry>& cMo, Eigen::Matrix<double, 6, 1>& cart_v) {
        cMo_[0][0] = cMo(0, 0); cMo_[0][1] = cMo(0, 1); cMo_[0][2] = cMo(0, 2); cMo_[0][3] = cMo(0, 3);
        cMo_[1][0] = cMo(1, 0); cMo_[1][1] = cMo(1, 1); cMo_[1][2] = cMo(1, 2); cMo_[1][3] = cMo(1, 3);
        cMo_[2][0] = cMo(2, 0); cMo_[2][1] = cMo(2, 1); cMo_[2][2] = cMo(2, 2); cMo_[2][3] = cMo(2, 3);
        cMo_[3][0] = cMo(3, 0); cMo_[3][1] = cMo(3, 1); cMo_[3][2] = cMo(3, 2); cMo_[3][3] = cMo(3, 3);
        cdMc_ = cdMo_ * cMo_.inverse();
        s_t_.buildFrom(cMo_);
        s_tu_.buildFrom(cdMc_);
        v_c_ = task_.computeControlLaw(); // Compute camera velocity skew
        cart_v << v_c_[0], v_c_[1], v_c_[2], v_c_[3], v_c_[4], v_c_[5];
        error_ = (task_.getError()).sumSquare(); // error = s^2 - s_star^2
        // ROS_INFO_STREAM("error_: " << error_);
        if(error_ < 0.00001) {
            return true;
        }
        return false;
        // ROS_INFO_STREAM("error_: " << error_);
    }
private:
    vpHomogeneousMatrix cdMc_;      //the displacement the camera has to achieve to move from the desired camera frame and the current one
    vpHomogeneousMatrix cMo_;
    vpHomogeneousMatrix oMo_;
    vpHomogeneousMatrix cdMo_;
    vpFeatureTranslation s_t_;     //the current visual feature s
    vpFeatureThetaU s_tu_;
    vpFeatureTranslation s_star_t_;      //the desired visual feature s*
    vpFeatureThetaU s_star_tu_;
    vpServo task_;         // the visual servo task
    vpColVector v_c_;      // Camera velocity
    double error_;         // Task error
};

class SE3Pid {
public:
    SE3Pid():kp_(0.01), ki_(0.0), kd_(0.0), dt_(1), max_(0.1), integral_(Eigen::Matrix<double, 6, 1>::Zero()), pre_error_(Eigen::Matrix<double, 6, 1>::Zero()) {}
    ~SE3Pid() {}
    bool Init() {
        cdMo_ = Eigen::Transform<double, 3, Eigen::Isometry>::Identity();
        Eigen::Matrix<double, 3, 3> rot;
        rot << -1.0, 0.0, 0.0,
                0.0, -1.0, 0.0,
                0.0, 0.0, 1.0;
        cdMo_.rotate(Eigen::Quaterniond(rot));
        cdMo_.pretranslate(Eigen::Vector3d(0.1112315, 0.21367, 0.6));
        cdMo_se3_ = Sophus::SE3d(Eigen::Quaterniond(cdMo_.matrix().block<3, 3>(0, 0)), Eigen::Vector3d(cdMo_.matrix().block<3, 1>(0, 3)));
        oMcd_se3_ = Sophus::SE3d(Eigen::Quaterniond(cdMo_.inverse().matrix().block<3, 3>(0, 0)), Eigen::Vector3d(cdMo_.inverse().matrix().block<3, 1>(0, 3)));
    }
    bool Calc(const Eigen::Transform<double, 3, Eigen::Isometry>& cMo, Eigen::Transform<double, 3, Eigen::Isometry>& pose)  {
        Sophus::SE3d cMo_se3(Eigen::Quaterniond(cMo.matrix().block<3, 3>(0, 0)), Eigen::Vector3d(cMo.matrix().block<3, 1>(0, 3)));
        Eigen::Matrix<double, 6, 1> error = (cMo_se3.inverse() * cdMo_se3_).log();
        Eigen::Matrix<double, 6, 1> diff = 0.1*error;
        Sophus::SE3d se_i = cMo_se3 * Sophus::SE3d::exp(diff);
        pose = se_i.matrix();
        return false;
    }
    bool Calc(const Eigen::Transform<double, 3, Eigen::Isometry>& cMo, Eigen::Matrix<double, 6, 1>& cart_v)  {
        Sophus::SE3d cMo_se3(Eigen::Quaterniond(cMo.matrix().block<3, 3>(0, 0)), Eigen::Vector3d(cMo.matrix().block<3, 1>(0, 3)));
        Eigen::Matrix<double, 6, 1> error = (cMo_se3.inverse() * cdMo_se3_).log();
        Eigen::Matrix<double, 6, 1> delta = 0.1*error;
        // ROS_INFO_STREAM("delta: " << delta);
        cart_v = delta;
        // if (error.norm() < 1e-6) {
        //     return true;
        // }
        // // Proportional term
        // Eigen::Matrix<double, 6, 1> p_out = kp_ * error;
        // // Integral term
        // integral_ += error * dt_;
        // Eigen::Matrix<double, 6, 1> i_out = ki_ * integral_;
        // // Derivative term
        // Eigen::Matrix<double, 6, 1> derivative = (error - pre_error_) / dt_;
        // Eigen::Matrix<double, 6, 1> d_out = kd_ * derivative;
        // // Calculate total output
        // Eigen::Matrix<double, 6, 1> diff = p_out + i_out + d_out;
        // if (diff.norm() > max_) {
        //     ROS_INFO_STREAM("pid is too large: " << diff);
        //     diff = diff * max_/diff.norm();
        // }
        // cart_v = diff;
        // pre_error_ = error;
        // return false;
    }
    bool Calc2o(const Eigen::Transform<double, 3, Eigen::Isometry>& cMo, Eigen::Matrix<double, 6, 1>& cart_v)  {
        Sophus::SE3d oMc_se3(Eigen::Quaterniond(cMo.inverse().matrix().block<3, 3>(0, 0)), Eigen::Vector3d(cMo.inverse().matrix().block<3, 1>(0, 3)));
        Eigen::Matrix<double, 6, 1> error = (oMc_se3.inverse() * oMcd_se3_).log();
        Eigen::Matrix<double, 6, 1> delta = 0.1*error;
        // ROS_INFO_STREAM("delta: " << delta);
        cart_v = delta;
    }
private:
    float kp_;
    float ki_;
    float kd_;
    float dt_;
    float max_;
    Eigen::Matrix<double, 6, 1> pre_error_;
    Eigen::Matrix<double, 6, 1> integral_;
    Eigen::Transform<double, 3, Eigen::Isometry> cdMo_;
    Sophus::SE3d cdMo_se3_;
    Sophus::SE3d oMcd_se3_;
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
    }
    ~Robot() {}
    Eigen::Transform<double, 3, Eigen::Isometry> bMe() {
        geometry_msgs::PoseStamped pose_msg = move_group_.getCurrentPose("panda_link8");
        Eigen::Transform<double, 3, Eigen::Isometry> pose = Eigen::Transform<double, 3, Eigen::Isometry>::Identity();
        pose.rotate(Eigen::Quaterniond(pose_msg.pose.orientation.w, pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z));
        pose.pretranslate(Eigen::Vector3d(pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z));
        return pose;
    }
    // Eigen::MatrixXd jacobian() {
    //     kinematic_state_ = move_group_.getCurrentState();
    //     kinematic_state_->getJacobian(kinematic_state_->getJointModelGroup("panda_arm"),
    //                            kinematic_state_->getLinkModel(kinematic_state_->getJointModelGroup("panda_arm")->getLinkModelNames().back()),
    //                            reference_point_position_, jacobian_);
    //     return jacobian_;
    // }
    Eigen::MatrixXd jacobian() {
        kinematic_state_ = move_group_.getCurrentState();
        // ROS_INFO_STREAM(kinematic_state_->getJointModelGroup("panda_arm")->getLinkModelNames().back());
        return kinematic_state_->getJacobian(kinematic_state_->getJointModelGroup("panda_arm"));
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
    bool Move(const Eigen::Transform<double, 3, Eigen::Isometry>& pose) {
        move_group_.setStartStateToCurrentState();
        move_group_.setPoseTarget(pose);
        move_group_.move();
    }
private:
    ros::NodeHandle nh_;
    std::string planning_group_;
    moveit::planning_interface::MoveGroupInterface move_group_;
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

        pbvs_.Init();
        pid_.Init();
        cMo_sub_ = nh_.subscribe("board/pose", 1, &VisualServoing::CalibPoseCallback, this);
    }
    ~VisualServoing() {}
    void CalibPoseCallback(const geometry_msgs::PoseStamped& cMo_msg) {
        // std::unique_lock<std::mutex> locker(mutex_);
        Eigen::Transform<double, 3, Eigen::Isometry> cMo = Eigen::Transform<double, 3, Eigen::Isometry>::Identity();
        cMo.rotate(Eigen::Quaterniond(cMo_msg.pose.orientation.w, cMo_msg.pose.orientation.x, cMo_msg.pose.orientation.y, cMo_msg.pose.orientation.z));
        cMo.pretranslate(Eigen::Vector3d(cMo_msg.pose.position.x, cMo_msg.pose.position.y, cMo_msg.pose.position.z));
        // ROS_INFO_STREAM("cMo: " << cMo.matrix());
        // locker.unlock();
        mutex_.lock();
        cMo_ = cMo;
        mutex_.unlock();
    }
    void RunVisp() {
        Eigen::Transform<double, 3, Eigen::Isometry> cMo;
        mutex_.lock();
        cMo = cMo_;
        mutex_.unlock();
        if (pbvs_.Calc(cMo, cart_v_)) {
            ROS_INFO_STREAM("motion_finished");
            return;
        }
        ROS_INFO_STREAM("pbvs_ cart_v_:" << cart_v_*10);
        jacobian_ = robot_.jacobian();
        Eigen::Matrix<double, 6, 1> cart_v;
        // cart_v << cart_v_(1, 0), cart_v_(0, 0), -cart_v_(2, 0), cart_v_(4, 0), cart_v_(3, 0), -cart_v_(5, 0);
        cart_v << cart_v_(1, 0), cart_v_(0, 0), -cart_v_(2, 0), cart_v_(4, 0), cart_v_(3, 0), -cart_v_(5, 0);
        Eigen::Matrix<double, 7, 1> q_dot = jacobian_.transpose() * (jacobian_ * jacobian_.transpose()).inverse() * cart_v;
        // robot_.Move(q_dot);
    }
    void RunPid() {
        Eigen::Transform<double, 3, Eigen::Isometry> cMo;
        mutex_.lock();
        cMo = cMo_;
        mutex_.unlock();
        pid_.Calc(cMo, cart_v_);
        ROS_INFO_STREAM("pid_ cart_v_:" << cart_v_*10);
        jacobian_ = robot_.jacobian();
        Eigen::Matrix<double, 6, 1> cart_v;
        cart_v << cart_v_(1, 0), cart_v_(0, 0), cart_v_(2, 0), cart_v_(4, 0), cart_v_(3, 0), cart_v_(5, 0);
        sleep(1);
        Eigen::Matrix<double, 7, 1> q_dot = jacobian_.transpose() * (jacobian_ * jacobian_.transpose()).inverse() * cart_v;
        // robot_.Move(q_dot);
    }
    void RunPid2o() {
        jacobian_ = robot_.jacobian();
        Eigen::Matrix<double, 3, 3> eRb = robot_.bMe().inverse().matrix().block<3, 3>(0, 0);
        Eigen::Matrix<double, 6, 6> eVb = Eigen::Matrix<double, 6, 6>::Zero();
        for(int i=0; i<3; i++) {
            for (int j = 0; j < 3; j++) {
                eVb(i, j) = eRb(i, j);
                eVb(i+3, j+3) = eRb(i, j);
            }
        }
        Eigen::MatrixXd eJe = eVb * jacobian_;
        Eigen::Transform<double, 3, Eigen::Isometry> cMo;
        mutex_.lock();
        cMo = cMo_;
        mutex_.unlock();
        pid_.Calc2o(cMo, cart_v_);
        ROS_INFO_STREAM("pid_ Calc2o cart_v_:" << cart_v_*10);
        // pid_.Calc(cMo, cart_v_);
        // ROS_INFO_STREAM("pid_ Calc cart_v_:" << cart_v_*10);
        // pbvs_.Calc(cMo, cart_v_);
        // ROS_INFO_STREAM("pbvs_ Calc cart_v_:" << cart_v_*10);
        Eigen::Matrix<double, 6, 1> cart_v;
        cart_v << cart_v_(1, 0), -cart_v_(0, 0), cart_v_(2, 0), cart_v_(4, 0), -cart_v_(3, 0), cart_v_(5, 0);
        // sleep(1);
        Eigen::Matrix<double, 7, 1> q_dot = eJe.transpose() * (eJe * eJe.transpose()).inverse() * cVe_.inverse() * cart_v_;
        robot_.Move(q_dot);
    }
    void RunPidPose() {
        Eigen::Transform<double, 3, Eigen::Isometry> cMo;
        mutex_.lock();
        cMo = cMo_;
        mutex_.unlock();
        Eigen::Transform<double, 3, Eigen::Isometry> cMo_next;
        pid_.Calc(cMo, cMo_next);
        Eigen::Transform<double, 3, Eigen::Isometry> bMo = robot_.bMe() * eMc_ * cMo;
        Eigen::Transform<double, 3, Eigen::Isometry> bMe_next = bMo * cMo_next.inverse()*eMc_.inverse();
        robot_.Move(bMe_next);
    }
    void Run() {
        jacobian_ = robot_.jacobian();
        Eigen::Matrix<double, 3, 3> eRb = robot_.bMe().inverse().matrix().block<3, 3>(0, 0);
        Eigen::Matrix<double, 6, 6> eVb = Eigen::Matrix<double, 6, 6>::Zero();
        for(int i=0; i<3; i++) {
            for (int j = 0; j < 3; j++) {
                eVb(i, j) = eRb(i, j);
                eVb(i+3, j+3) = eRb(i, j);
            }
        }
        Eigen::MatrixXd eJe = eVb * jacobian_;
        Eigen::Matrix<double, 6, 1> cart_v;
        cart_v << 0, 0, 0, 0.01, 0.01, 0;
        sleep(1);
        Eigen::Matrix<double, 7, 1> q_dot = eJe.transpose() * (eJe * eJe.transpose()).inverse() * cVe_.inverse() * cart_v;
        robot_.Move(q_dot);
    }
private:
    ros::NodeHandle nh_;
    ros::Subscriber cMo_sub_;
    Robot robot_;
    SE3Pid pid_;
    PBVS pbvs_;
    Eigen::Transform<double, 3, Eigen::Isometry> bMe_;
    Eigen::Transform<double, 3, Eigen::Isometry> bMc_;
    Eigen::Transform<double, 3, Eigen::Isometry> eMc_;
    Eigen::Transform<double, 3, Eigen::Isometry> bMcd_;
    Eigen::Transform<double, 3, Eigen::Isometry> cdMo_;
    // std::atomic<Eigen::Transform<double, 3, Eigen::Isometry>> cMo_;
    Eigen::Transform<double, 3, Eigen::Isometry> cMo_;
    Eigen::Matrix<double, 6, 1> cart_v_;
    Eigen::MatrixXd jacobian_;
    Eigen::Matrix<double, 6, 6> cVe_;
    std::mutex mutex_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "panda_visual_servoing");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ROS_INFO("hello panda_visual_servoing");

    VisualServoing vs(nh);
    while(1) {
        // vs.RunVisp();
        // vs.RunPid();
        vs.RunPid2o();
        // vs.RunPidPose();
        // vs.Run();
    }

    // ros::waitForShutdown();
    return 0;
}