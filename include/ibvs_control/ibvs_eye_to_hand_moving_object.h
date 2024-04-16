//
// Created by skywoodsz on 2022/8/24.
//

#ifndef SRC_IBVS_EYE_TO_HAND_MOVING_OBJECT_H
#define SRC_IBVS_EYE_TO_HAND_MOVING_OBJECT_H

#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <urdf_parser/urdf_parser.h>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/crba.hpp>

#include <trac_ik/trac_ik.hpp>

#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Vector3.h>
#include <control_msgs/JointControllerState.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/TransformStamped.h>
#include <dynamic_reconfigure/server.h>

#include "ibvs_control/ParamConfig.h"

#include <iostream>
#include <string>
#include <list>

class IBVS_EYE_TO_HAND{
public:
    IBVS_EYE_TO_HAND(const ros::NodeHandle &node_handle, const ros::NodeHandle &private_node_handle);
    bool Manipulator_init();
    void TransformInit();
    void BuildVisionError_(Eigen::Vector3d& error, Eigen::Matrix3d& Lt, Eigen::Vector3d& ho, Eigen::Vector3d& ht);
    void TargetCentroidSphericalProject(Eigen::Vector3d& ho);
    void DesiredCentroidSphericalProject(Eigen::Vector3d &ht, Eigen::Matrix3d& Lt);
    void GetCurrentEEPositionInOptical(Eigen::Vector3d& Qt, std::vector<Eigen::Vector3d>& Qti);
    void GetVirtualPlaneEEPositionInOptical(const Eigen::Vector3d& Qt, Eigen::Vector3d& Qt_virtual);
    void GetVirtualPlaneEEPositionInOpticalMore(const std::vector<Eigen::Vector3d> &Qti, std::vector<Eigen::Vector3d> &Qti_virtual);
    void ControlLaw(const Eigen::Vector3d& error, const Eigen::Matrix3d& Lt);
    void BuildJacobian(const Eigen::Matrix3d& Lt, Eigen::Matrix<double, 3, 6>& Jc_arm);
    void ManipulatorControl(const Eigen::Vector3d& error, const Eigen::Matrix<double, 3, 6>& Jc_arm);
    void QuadrupedVelocityControl(const Eigen::Vector3d& error, const Eigen::Matrix3d& Lt, bool control_flag);
    void VisualizationDesiredPoint(const Eigen::Vector3d desired_point, int id);
    void VisualizationPoint(const Eigen::Vector3d desired_point, int id, const std::string frame);
    void ObjectVelocityObserver(const Eigen::Vector3d& ho, const Eigen::Vector3d& ht, const Eigen::Matrix3d& Lt);
    void BuildObjectVelocityObserverError(const Eigen::Vector3d& ho, const Eigen::Vector3d& ho_observer, Eigen::Vector3d& error);
    void BuildObjectVelocityObserverYAndSoi(const Eigen::Vector3d& error, Eigen::Vector3d& y,
                                            const Eigen::Matrix3d& Lt, const Eigen::Vector3d& ho);

private:
    void ControLoop_(const ros::TimerEvent& event);
    void ManipulatorStateLoop_(const ros::TimerEvent& event);
    void StateLoop_(const ros::TimerEvent& event);

    void FindCirclePoint(const sensor_msgs::Image::ConstPtr& msg);

    void DogPoseCallBack(const nav_msgs::Odometry::ConstPtr& msg);

    void GTOdomCallBack(const nav_msgs::Odometry::ConstPtr& msg);

    void ArmJointState0CallBack(const control_msgs::JointControllerState::ConstPtr& msg);
    void ArmJointState1CallBack(const control_msgs::JointControllerState::ConstPtr& msg);
    void ArmJointState2CallBack(const control_msgs::JointControllerState::ConstPtr& msg);
    void ArmJointState3CallBack(const control_msgs::JointControllerState::ConstPtr& msg);
    void ArmJointState4CallBack(const control_msgs::JointControllerState::ConstPtr& msg);
    void ArmJointState5CallBack(const control_msgs::JointControllerState::ConstPtr& msg);

    void dynamicCallback(ibvs_control::ParamConfig& config, uint32_t /*level*/);

    // ros
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Timer state_timer_;
    ros::Timer odom_timer_;
    ros::Timer control_timer_;
    tf::TransformListener listener_;
    tf2_ros::TransformBroadcaster tf_br_;
    double DT_;

    // camera
    Eigen::Matrix3d camera_k_;
    std::vector<Eigen::Vector2d> circle_centers_;
    ros::Publisher image_debug_pub_;
    ros::Subscriber image_sub_;
    ros::Subscriber odom_sub_;

    // arm
    ros::Subscriber arm_joint_sub_[6];
    ros::Publisher arm_publisher_[6];
    int frameIndex_;
    Eigen::VectorXd q_;
    Eigen::VectorXd q_cmd_;
    std::string ee_name_;
    std::string arm_base_name_;
    Eigen::Vector3d t_E_T_;

    std::shared_ptr<urdf::ModelInterface> urdf_;
    std::shared_ptr<pinocchio::Model> pin_model_;
    std::shared_ptr<pinocchio::Data> pin_data_;

    bool work_range_;

    // debug
    ros::Publisher marker_pub_;
    ros::Publisher error_pub_;
    ros::Publisher quadruped_publisher_;

    ros::Publisher ho_debug_pub_, ht_debug_pub_;

    ros::Publisher error_observer_pub_;
    ros::Publisher vt_observer_pub_;
    ros::Publisher vcc_pub_;
    ros::Publisher ho_observer_pub_;

    ros::Publisher ee_pub_;

    // state
    Eigen::Isometry3d T_IC_; // camera optical to world
    Eigen::Isometry3d T_IB_; // dog base to world
    Eigen::Isometry3d T_B_MB_; // arm base to dog base
    Eigen::Isometry3d T_MB_EE_; // arm ee to arm base
    Eigen::Isometry3d T_EE_C_; // camera optical to arm ee
    Eigen::Isometry3d T_MBC_; // camera optical to arm base
    Eigen::Isometry3d T_IVirtual_C_; // camera optical to virtual plane
    Eigen::Isometry3d T_I_IVirtual_; // world to virtual plane

    // flag
    bool begin_control_flag_;
    int begin_control_count;

    // observer
    bool observer_begin_flag_;
    bool observer_init_flag_;
    int observer_begin_count;

    double p_, k1_, k2_, k3_, k4_;
    double k_error_z_;
    double k_error_y_;

    double error_t_z_;

    Eigen::Vector3d v_ib_, w_ib_;

    Eigen::Vector3d ho_observer_;
    Eigen::Vector3d object_vt_;

    std::shared_ptr<dynamic_reconfigure::Server<ibvs_control::ParamConfig>> dynamic_srv_{};

    ros::Subscriber GTOdomSub_;
};

#endif //SRC_IBVS_EYE_TO_HAND_MOVING_OBJECT_H
