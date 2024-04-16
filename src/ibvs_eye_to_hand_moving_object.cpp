//
// Created by skywoodsz on 2022/8/24.
//

#include <ibvs_control/ibvs_eye_to_hand_moving_object.h>
#include <ibvs_control/robot_math.h>

IBVS_EYE_TO_HAND::IBVS_EYE_TO_HAND(const ros::NodeHandle &node_handle, const ros::NodeHandle &private_node_handle) :
nh_(node_handle),
pnh_(private_node_handle),
DT_(1.0 / 30.0)
{
    // observer
    observer_begin_count = 0;
    observer_begin_flag_ = false;
    observer_init_flag_ = false;
    ho_observer_.setZero();

    ros::NodeHandle nh_param = ros::NodeHandle(node_handle, "observer_param");
    dynamic_srv_ = std::make_shared<dynamic_reconfigure::Server<ibvs_control::ParamConfig>>(nh_param);
    dynamic_reconfigure::Server<ibvs_control::ParamConfig>::CallbackType cb = [this](auto&& PH1, auto&& PH2) {
        dynamicCallback(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2));
    };
    dynamic_srv_->setCallback(cb);

    ee_name_ = "j2n6s300_end_effector";
    // ee_name_ = "j2n6s300_link_6";
    arm_base_name_ = "j2n6s300_link_base";
    t_E_T_ = Eigen::Vector3d(-0.06, 0.0, -0.2); // -0.1

    begin_control_flag_ = true;
    begin_control_count = 0;
    work_range_ = true;

    TransformInit();

    // image
    camera_k_<< 462.1379699707031, 0.0, 320.0, 0.0, 462.1379699707031, 240.0, 0.0, 0.0, 1.0;
    circle_centers_.resize(4);
    for (int j = 0; j < circle_centers_.size(); ++j) {
        circle_centers_[j].setZero();
    }

    image_debug_pub_ = nh_.advertise<sensor_msgs::Image>("/debug/image_find_result", 1);
    error_pub_ = nh_.advertise<geometry_msgs::Vector3>("/debug/error", 1);
    ho_debug_pub_ = nh_.advertise<geometry_msgs::Vector3>("/debug/ho", 1);
    ht_debug_pub_ = nh_.advertise<geometry_msgs::Vector3>("/debug/ht", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/visualization_marker", 1);
    quadruped_publisher_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    error_observer_pub_ = nh_.advertise<geometry_msgs::Vector3>("/debug/observer_error", 1);
    vt_observer_pub_ = nh_.advertise<geometry_msgs::Vector3>("/debug/vt", 1);
    vcc_pub_ = nh_.advertise<geometry_msgs::Vector3>("/debug/vcc", 1);
    ho_observer_pub_ = nh_.advertise<geometry_msgs::Vector3>("/debug/observer_ho", 1);
    ee_pub_ = nh_.advertise<geometry_msgs::Vector3>("/debug/ee_position", 1);

    // manipulator
    arm_publisher_[0] = nh_.advertise<std_msgs::Float64>("/joint_1_position_controller/command", 1);
    arm_publisher_[1] = nh_.advertise<std_msgs::Float64>("/joint_2_position_controller/command", 1);
    arm_publisher_[2] = nh_.advertise<std_msgs::Float64>("/joint_3_position_controller/command", 1);
    arm_publisher_[3] = nh_.advertise<std_msgs::Float64>("/joint_4_position_controller/command", 1);
    arm_publisher_[4] = nh_.advertise<std_msgs::Float64>("/joint_5_position_controller/command", 1);
    arm_publisher_[5] = nh_.advertise<std_msgs::Float64>("/joint_6_position_controller/command", 1);

    Manipulator_init();

    arm_joint_sub_[0] = nh_.subscribe("/joint_1_position_controller/state", 1, &IBVS_EYE_TO_HAND::ArmJointState0CallBack, this);
    arm_joint_sub_[1] = nh_.subscribe("/joint_2_position_controller/state", 1, &IBVS_EYE_TO_HAND::ArmJointState1CallBack, this);
    arm_joint_sub_[2] = nh_.subscribe("/joint_3_position_controller/state", 1, &IBVS_EYE_TO_HAND::ArmJointState2CallBack, this);
    arm_joint_sub_[3] = nh_.subscribe("/joint_4_position_controller/state", 1, &IBVS_EYE_TO_HAND::ArmJointState3CallBack, this);
    arm_joint_sub_[4] = nh_.subscribe("/joint_5_position_controller/state", 1, &IBVS_EYE_TO_HAND::ArmJointState4CallBack, this);
    arm_joint_sub_[5] = nh_.subscribe("/joint_6_position_controller/state", 1, &IBVS_EYE_TO_HAND::ArmJointState5CallBack, this);

    odom_sub_ = nh_.subscribe("/odom", 1, &IBVS_EYE_TO_HAND::DogPoseCallBack, this);

    image_sub_ = nh_.subscribe("/d435/color/image_raw", 1, &IBVS_EYE_TO_HAND::FindCirclePoint, this);

    GTOdomSub_ = nh_.subscribe("/ground_truth/state", 1, &IBVS_EYE_TO_HAND::GTOdomCallBack, this);

    state_timer_ = pnh_.createTimer(ros::Duration(DT_ / 5.0), &IBVS_EYE_TO_HAND::ManipulatorStateLoop_, this);
    odom_timer_ = pnh_.createTimer(ros::Duration(DT_ / 5.0), &IBVS_EYE_TO_HAND::StateLoop_, this);
    control_timer_ = pnh_.createTimer(ros::Duration(DT_), &IBVS_EYE_TO_HAND::ControLoop_, this);

}

void IBVS_EYE_TO_HAND::StateLoop_(const ros::TimerEvent &event) {
    T_IC_ = Eigen::Isometry3d::Identity();
    T_IC_ = T_IB_ * T_B_MB_ * T_MBC_;


    if (begin_control_flag_)
    {
        T_I_IVirtual_ = T_IC_;
    }

    T_IVirtual_C_ = T_I_IVirtual_.inverse() * T_IC_;

    Eigen::Vector3d euler_angules = T_IVirtual_C_.rotation().eulerAngles(2, 1, 0);
    euler_angules[1] = 0.;

    if(euler_angules[0] < -1.57)
    {
        euler_angules[0] += M_PI;
    }
    if(euler_angules[0] > 1.57)
    {
        euler_angules[0] -= M_PI;
    }
    if(euler_angules[2] < -1.57)
    {
        euler_angules[2] += M_PI;
    }
    if(euler_angules[2] > 1.57)
    {
        euler_angules[2] -= M_PI;
    }


    Eigen::Matrix3d rotation_matrix;
    rotation_matrix = Eigen::AngleAxisd(euler_angules[0], Eigen::Vector3d::UnitZ()) *
                      Eigen::AngleAxisd(euler_angules[1], Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(euler_angules[2], Eigen::Vector3d::UnitX());
    T_IVirtual_C_ = Eigen::Isometry3d::Identity();
    T_IVirtual_C_.rotate(rotation_matrix);

    begin_control_count++;
    if(begin_control_count > 300)
    {
        begin_control_flag_ = false;
    }
}

void IBVS_EYE_TO_HAND::ManipulatorStateLoop_(const ros::TimerEvent &event) {

    pinocchio::forwardKinematics(*pin_model_, *pin_data_, q_);
    pinocchio::computeJointJacobians(*pin_model_, *pin_data_);
    pinocchio::updateFramePlacements(*pin_model_, *pin_data_);

    Eigen::Vector3d t_mb_ee;
    Eigen::Matrix3d R_MB_EE;
    t_mb_ee = pin_data_->oMf[frameIndex_].translation();
    R_MB_EE =  pin_data_->oMf[frameIndex_].rotation();

    T_MB_EE_ = Eigen::Isometry3d::Identity();
    T_MB_EE_.rotate(R_MB_EE);
    T_MB_EE_.pretranslate(t_mb_ee);

    if(t_mb_ee[0] > 0.7) 
    {
        work_range_ = false;
    }
    else
    {
        work_range_ = true;
    }

    geometry_msgs::Vector3 ee_msg;
    ee_msg.x = t_mb_ee[0];
    ee_msg.y = t_mb_ee[1];
    ee_msg.z = t_mb_ee[2];
    ee_pub_.publish(ee_msg);
}

void IBVS_EYE_TO_HAND::ControLoop_(const ros::TimerEvent &event) {
    Eigen::Vector3d error;
    Eigen::Matrix3d Lt;
    Eigen::Vector3d ho, ht;

    if(!begin_control_flag_)
    {
        BuildVisionError_(error, Lt, ho, ht);
        ControlLaw(error, Lt);

        // observer
        observer_begin_count++;
        if(observer_begin_count > 300)
        {
            observer_begin_count = 301;
            observer_begin_flag_ = true;
        }
        if(observer_begin_flag_)
        {
            ObjectVelocityObserver(ho, ht, Lt);
        }
    }

}

void IBVS_EYE_TO_HAND::ObjectVelocityObserver(const Eigen::Vector3d& ho, const Eigen::Vector3d& ht, const Eigen::Matrix3d& Lt) {
    if(!observer_init_flag_)
    {
        ho_observer_ = ht; // observer init value set to arm
        object_vt_.setZero();
        ROS_INFO("\033[1;32m----> Observer Start!\033[0m");
        observer_init_flag_ = true;
    }

    Eigen::Vector3d error;

    BuildObjectVelocityObserverError(ho, ho_observer_, error);
    BuildObjectVelocityObserverYAndSoi(error, object_vt_, Lt, ho);
}

void IBVS_EYE_TO_HAND::BuildObjectVelocityObserverYAndSoi(const Eigen::Vector3d &error, Eigen::Vector3d &y,
                                                          const Eigen::Matrix3d& Lt, const Eigen::Vector3d& ho) {
    double k1, k2, k3, k4, p;
    double dt = DT_; // 1.0 / 30.0

    k1 = k1_; k2 = k2_; k3 = k3_; k4 = k4_; p = p_;

    double pha_one = k3 * pow(error.norm(), -p) + k4;
    double pha_two = (k3 * (1 - p) * pow(error.norm(), -p) + k4) * pha_one;
    Eigen::Vector3d y_dot = k2 * pha_two * error;
    y += y_dot * dt;

    Eigen::Vector3d w_cc, v_cc;
    Eigen::Matrix3d R_IC = T_IC_.rotation();
    w_cc = R_IC.transpose() * w_ib_;
    v_cc = R_IC.transpose() * v_ib_;

    Eigen::Vector3d soi_observer_dot;
    soi_observer_dot = -Vector2SkewSymmetric(w_cc) * ho - Lt * v_cc
            + k1 * pha_one * error + Lt * y;
    ho_observer_ += soi_observer_dot * dt;

    geometry_msgs::Vector3 vt;
    vt.x = y.x();
    vt.y = y.y();
    vt.z = y.z();
    vt_observer_pub_.publish(vt);

    geometry_msgs::Vector3 dog_v_cc;
    dog_v_cc.x = v_cc.x();
    dog_v_cc.y = v_cc.y();
    dog_v_cc.z = v_cc.z();
    vcc_pub_.publish(dog_v_cc);
    
    geometry_msgs::Vector3 ho_observer;
    ho_observer.x = ho_observer_.x();
    ho_observer.y = ho_observer_.y();
    ho_observer.z = ho_observer_.z();
    ho_observer_pub_.publish(ho_observer);
}

void IBVS_EYE_TO_HAND::BuildObjectVelocityObserverError(const Eigen::Vector3d &ho, const Eigen::Vector3d &ho_observer,
                                                        Eigen::Vector3d &error) {
    Eigen::Matrix3d k = Eigen::Matrix3d::Identity();
    k(1, 1) *= k_error_y_;
    k(2, 2) *= k_error_z_;

    error = k * (ho - ho_observer);

    geometry_msgs::Vector3 observer_error;
    observer_error.x = error.x();
    observer_error.y = error.y();
    observer_error.z = error.z();

    error_observer_pub_.publish(observer_error);
}


void IBVS_EYE_TO_HAND::ControlLaw(const Eigen::Vector3d& error, const Eigen::Matrix3d& Lt) {
    Eigen::Matrix<double, 3, 6> Jc_arm;

    BuildJacobian(Lt, Jc_arm);

    if((abs(error[0]) > 0.1) || (abs(error[2]) > error_t_z_)) 
    {
        QuadrupedVelocityControl(error, Lt, true);
    }
    else
    {
        QuadrupedVelocityControl(error, Lt, true);
        ManipulatorControl(error, Jc_arm);
    }
}

void IBVS_EYE_TO_HAND::QuadrupedVelocityControl(const Eigen::Vector3d &error, const Eigen::Matrix3d &Lt, bool control_flag) {
    Eigen::Vector3d Vc, Vdog, Vt;
    Eigen::Matrix3d R_IC = T_IC_.rotation();

    Vc = Lt.inverse() * error;
    // Vc = error;
    Vdog = R_IC * Vc;
    Vdog[0] += object_vt_.z();

    geometry_msgs::Twist twist;
    if(control_flag)
    {
        twist.linear.x = Vdog(0);
        twist.linear.y = Vdog(1);
        twist.linear.z = 0;
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = 0.0;
    }
    else
    {
        twist.linear.x = 0.;
        twist.linear.y = 0.;
        twist.linear.z = 0.;
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = 0.0;
    }

    quadruped_publisher_.publish(twist);
}

void IBVS_EYE_TO_HAND::ManipulatorControl(const Eigen::Vector3d &error, const Eigen::Matrix<double, 3, 6> &Jc_arm) {

    Eigen::Matrix<double, 6, 1> vel, vel_desired, qdot;
    vel.setZero(); vel_desired.setZero(); qdot.setZero();

    DMat<double> Jmatrix_temp, Jinv;
    Jmatrix_temp.resize(3, 6);
    Jmatrix_temp.block<3, 6>(0, 0) = Jc_arm.block<3, 6>(0, 0);

    pseudoInverse(Jmatrix_temp, 0.001, Jinv);
    Jinv.resize(6, 3);

    vel.head(3) = Jinv * error;
    vel_desired = vel;

    if(!work_range_)
    {
        vel_desired.setZero();
    }

    Eigen::Matrix<double, 6, 6> Be;
    Be.setZero();
    pinocchio::getFrameJacobian(*pin_model_, *pin_data_, frameIndex_,
                                pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, Be);

    qdot = Be.inverse() * vel_desired;
    qdot[5] = 0.;
    for (int j = 0; j < 6; ++j) {
        q_cmd_[j] += qdot[j] * DT_;
    }

    for (int j = 0; j < 6; ++j) {
        std_msgs::Float64 send_cmd;
        send_cmd.data = q_cmd_[j];
        arm_publisher_[j].publish(send_cmd);
    }
}

void IBVS_EYE_TO_HAND::BuildJacobian(const Eigen::Matrix3d &Lt, Eigen::Matrix<double, 3, 6> &Jc_arm) {

    Eigen::Matrix<double, 3, 6> Jt_temp, Jt;
    Eigen::Matrix3d I_3x3 = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d R_MB_E = T_MB_EE_.rotation();
    Eigen::Matrix3d R_C_MB = T_MBC_.rotation().transpose();
    Eigen::Matrix3d sk_matrix;

    sk_matrix = Vector2SkewSymmetric(R_MB_E * t_E_T_);
    Jt_temp.block<3, 3>(0, 0) = I_3x3;
    Jt_temp.block<3, 3>(0, 3) = -sk_matrix;
    Jt = R_C_MB * Jt_temp;

    Eigen::Matrix3d R_IVirtual_C  = T_IVirtual_C_.rotation();
    Jc_arm = R_IVirtual_C.transpose() * Jt;
    // Jc_arm = Lt * R_IVirtual_C.transpose() * Jt;
}


void IBVS_EYE_TO_HAND::BuildVisionError_(Eigen::Vector3d &error, Eigen::Matrix3d& Lt,
                                         Eigen::Vector3d& ho, Eigen::Vector3d& ht) {

    TargetCentroidSphericalProject(ho);
    DesiredCentroidSphericalProject(ht, Lt);

    error = ho - ht;

    geometry_msgs::Vector3 error_msg;
    error_msg.x = error(0);
    error_msg.y = error(1);
    error_msg.z = error(2);
    error_pub_.publish(error_msg);

    geometry_msgs::Vector3 ho_msg;
    ho_msg.x = ho(0);
    ho_msg.y = ho(1);
    ho_msg.z = ho(2);
    ho_debug_pub_.publish(ho_msg);

    geometry_msgs::Vector3 ht_msg;
    ht_msg.x = ht(0);
    ht_msg.y = ht(1);
    ht_msg.z = ht(2);
    ht_debug_pub_.publish(ht_msg);
}

void IBVS_EYE_TO_HAND::DesiredCentroidSphericalProject(Eigen::Vector3d &ht, Eigen::Matrix3d& Lt) {
    Eigen::Vector3d Qt;
    std::vector<Eigen::Vector3d> Qti, Qti_virtual, sti_virtual;
    Eigen::Matrix3d pi_s_virtual;
    Qti.resize(4); Qti_virtual.resize(4); sti_virtual.resize(4);

    Eigen::Matrix3d I_3x3 = Eigen::Matrix3d::Identity();

    GetCurrentEEPositionInOptical(Qt, Qti);
    GetVirtualPlaneEEPositionInOpticalMore(Qti, Qti_virtual);

    ht.setZero(); Lt.setZero();
    for (int j = 0; j < 4; ++j) {
        sti_virtual[j] = Qti_virtual[j] / Qti_virtual[j].norm();
        ht += sti_virtual[j] / 4.0;
        pi_s_virtual = I_3x3 - sti_virtual[j] * sti_virtual[j].transpose();
        Lt += pi_s_virtual / Qti_virtual[j].norm();
    }

    VisualizationDesiredPoint(ht, 1);
}

void IBVS_EYE_TO_HAND::GetVirtualPlaneEEPositionInOpticalMore(const std::vector<Eigen::Vector3d> &Qti,
                                                              std::vector<Eigen::Vector3d> &Qti_virtual) {
    Eigen::Matrix3d R_IVirtual_C  = T_IVirtual_C_.rotation();
    for (int j = 0; j < 4; ++j) {
        Qti_virtual[j] = R_IVirtual_C.transpose() * Qti[j];
    }

}

void IBVS_EYE_TO_HAND::GetVirtualPlaneEEPositionInOptical(const Eigen::Vector3d &Qt, Eigen::Vector3d &Qt_virtual) {
    Eigen::Matrix3d R_IVirtual_C  = T_IVirtual_C_.rotation();

    Qt_virtual = R_IVirtual_C.transpose() * Qt;
}

void IBVS_EYE_TO_HAND::GetCurrentEEPositionInOptical(Eigen::Vector3d &Qt, std::vector<Eigen::Vector3d>& Qti) {
    Eigen::Vector3d Qt_in_arm_base_link = T_MB_EE_.translation() + t_E_T_;
    std::vector<Eigen::Vector3d> Qti_in_arm_base_link;
    Qti_in_arm_base_link.resize(4);

    double delta_ryo = 0.075; 
    Eigen::Matrix<double, 3, 4> ryo;
    ryo <<          0,           0,          0,            0,
            delta_ryo,  -delta_ryo,   delta_ryo,  -delta_ryo,
            delta_ryo,   delta_ryo,  -delta_ryo,  -delta_ryo;

    for (int j = 0; j < 4; ++j) {
        Qti_in_arm_base_link[j] = Qt_in_arm_base_link + ryo.col(j);
        Qti[j] = T_MBC_.inverse() * Qti_in_arm_base_link[j];
    }

    Qt = T_MBC_.inverse() * Qt_in_arm_base_link;
}

void IBVS_EYE_TO_HAND::TargetCentroidSphericalProject(Eigen::Vector3d &ho) {
    std::vector<Eigen::Vector3d> soi, qoi;
    soi.resize(4); qoi.resize(4);

    ho.setZero();
    for (int i = 0; i < 4; ++i) {
        qoi[i] = camera_k_.inverse() * Eigen::Vector3d (circle_centers_[i].x(), circle_centers_[i].y(), 1);
        soi[i] = qoi[i] / qoi[i].norm();
        ho += soi[i] / 4;
    }

    VisualizationDesiredPoint(ho, 0);
}

void IBVS_EYE_TO_HAND::TransformInit() {
    T_B_MB_ = Eigen::Isometry3d::Identity();
    T_EE_C_ = Eigen::Isometry3d::Identity();
    T_IB_ = Eigen::Isometry3d::Identity();
    T_IVirtual_C_ = Eigen::Isometry3d::Identity();
    T_I_IVirtual_ = Eigen::Isometry3d::Identity();
    T_MBC_ = Eigen::Isometry3d::Identity();

    tf::StampedTransform transform, transform2, transform3;
    try
    {
        listener_.waitForTransform("/base_link", arm_base_name_,  ros::Time(0), ros::Duration(3.0));
        listener_.lookupTransform("/base_link", arm_base_name_, ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
    Eigen::Vector3d translation(transform.getOrigin().x(),
                                transform.getOrigin().y(),
                                transform.getOrigin().z());

    Eigen::Quaterniond roation_q(transform.getRotation().getW(),
                                 transform.getRotation().getX(),
                                 transform.getRotation().getY(),
                                 transform.getRotation().getZ());

    T_B_MB_.rotate(roation_q);
    T_B_MB_.pretranslate(translation);

    try
    {
        listener_.waitForTransform(arm_base_name_, "/d435_color_optical_frame",  ros::Time(0), ros::Duration(3.0));
        listener_.lookupTransform(arm_base_name_, "/d435_color_optical_frame", ros::Time(0), transform3);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
    Eigen::Vector3d translation3(transform3.getOrigin().x(),
                                 transform3.getOrigin().y(),
                                 transform3.getOrigin().z());

    Eigen::Quaterniond roation_q3(transform3.getRotation().getW(),
                                  transform3.getRotation().getX(),
                                  transform3.getRotation().getY(),
                                  transform3.getRotation().getZ());

    T_MBC_.rotate(roation_q3);
    T_MBC_.pretranslate(translation3);
}

bool IBVS_EYE_TO_HAND::Manipulator_init() {

    if (pin_model_ == nullptr && pin_data_ == nullptr)
    {
        // Get the URDF on param server, then build model and data
        std::string urdf_string;
        nh_.getParam("/manipulator_description", urdf_string); // decomposition control
        if (urdf_string.empty()) {
            ROS_ERROR("manipulator urdf load wrong!");
            return false;
        }
        pin_model_ = std::make_shared<pinocchio::Model>();
        urdf_ = urdf::parseURDF(urdf_string);
        pinocchio::urdf::buildModel(urdf_,  *pin_model_);
        pin_data_ = std::make_shared<pinocchio::Data>(*pin_model_);

    }

    frameIndex_ = pin_model_->getFrameId(ee_name_);
    q_.resize(6); q_.setZero();
    q_cmd_.resize(6); q_cmd_.setZero();

    return true;
}

void IBVS_EYE_TO_HAND::FindCirclePoint(const sensor_msgs::Image::ConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;
    // convert img to cv2
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // blob detect to get center uv
    cv::SimpleBlobDetector::Params params;
    params.filterByArea = true;
    params.maxArea = 50000;
    params.filterByInertia = true;
    params.minInertiaRatio = 0.5f;

    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
    std::vector<cv::KeyPoint> keypoints;

    detector->detect(cv_ptr->image, keypoints);


    cv::drawKeypoints(cv_ptr->image, keypoints, cv_ptr->image, cv::Scalar(255, 0, 0), cv::DrawMatchesFlags::DEFAULT);
//    cv::circle(cv_ptr->image, cv::Point(h_desired_pixel(0), h_desired_pixel(1)), 10, cv::Scalar(0, 0, 255), 2);

    if (keypoints.size() == 4)
    {
        for (int i = 0; i < keypoints.size(); ++i) {
            circle_centers_[i].x() = keypoints[i].pt.x;
            circle_centers_[i].y() = keypoints[i].pt.y;
        }
    }
    else
    {
        ROS_WARN("Find blob failed! The number is not 4!");
    }

    image_debug_pub_.publish(cv_ptr->toImageMsg());
}


void IBVS_EYE_TO_HAND::DogPoseCallBack(const nav_msgs::Odometry::ConstPtr &msg) {
    nav_msgs::Odometry body_pose = *msg;
    T_IB_ = Eigen::Isometry3d::Identity();

    Eigen::Vector3d translation(body_pose.pose.pose.position.x,
                                body_pose.pose.pose.position.y,
                                body_pose.pose.pose.position.z);

    Eigen::Quaterniond roation_q(body_pose.pose.pose.orientation.w,
                                 body_pose.pose.pose.orientation.x,
                                 body_pose.pose.pose.orientation.y,
                                 body_pose.pose.pose.orientation.z);

    T_IB_.rotate(roation_q);
    T_IB_.pretranslate(translation);
}

void IBVS_EYE_TO_HAND::ArmJointState0CallBack(const control_msgs::JointControllerState::ConstPtr &msg) {
    q_[0] = msg->process_value;
}

void IBVS_EYE_TO_HAND::ArmJointState1CallBack(const control_msgs::JointControllerState::ConstPtr &msg) {
    q_[1] = msg->process_value;
}

void IBVS_EYE_TO_HAND::ArmJointState2CallBack(const control_msgs::JointControllerState::ConstPtr &msg) {
    q_[2] = msg->process_value;
}

void IBVS_EYE_TO_HAND::ArmJointState3CallBack(const control_msgs::JointControllerState::ConstPtr &msg) {
    q_[3] = msg->process_value;
}

void IBVS_EYE_TO_HAND::ArmJointState4CallBack(const control_msgs::JointControllerState::ConstPtr &msg) {
    q_[4] = msg->process_value;
}

void IBVS_EYE_TO_HAND::ArmJointState5CallBack(const control_msgs::JointControllerState::ConstPtr &msg) {
    q_[5] = msg->process_value;
}

void IBVS_EYE_TO_HAND::VisualizationDesiredPoint(const Eigen::Vector3d desired_point, int id)
{

    visualization_msgs::Marker marker;
    marker.header.frame_id = "d435_color_optical_frame";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = id;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.points.resize(2);
    marker.points[0].x = 0.;
    marker.points[0].y = 0.;
    marker.points[0].z = 0.;
    marker.points[1].x = desired_point(0);
    marker.points[1].y = desired_point(1);
    marker.points[1].z = desired_point(2);

    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;

    if(!id)
    {
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
    }
    else
    {
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
    }


    marker_pub_.publish(marker);
}

void IBVS_EYE_TO_HAND::VisualizationPoint(const Eigen::Vector3d desired_point, int id, const std::string frame) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame;
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = id + 2;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = desired_point.x();
    marker.pose.position.y = desired_point.y();
    marker.pose.position.z = desired_point.z();
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;

    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker_pub_.publish(marker);
}

void IBVS_EYE_TO_HAND::dynamicCallback(ibvs_control::ParamConfig &config, uint32_t) {

    p_ = config.p;
    k1_ = config.k1;
    k2_ = config.k2;
    k3_ = config.k3;
    k4_ = config.k4;
    k_error_z_ = config.k_error_z;
    k_error_y_ = config.k_error_y;

    double t_E_T_x;
    t_E_T_x = config.t_E_T_x;
    t_E_T_[0] = t_E_T_x;

    error_t_z_ = config.error_t_z;

    ROS_INFO("\033[1;32m [IBVS_Observer]: Observer params update.\033[0m");
}

void IBVS_EYE_TO_HAND::GTOdomCallBack(const nav_msgs::Odometry::ConstPtr &msg) {
    nav_msgs::Odometry body_pose = *msg;

    v_ib_ = Eigen::Vector3d(body_pose.twist.twist.linear.x,
                            body_pose.twist.twist.linear.y,
                            body_pose.twist.twist.linear.z);

    w_ib_ = Eigen::Vector3d(body_pose.twist.twist.angular.x,
                            body_pose.twist.twist.angular.y,
                            body_pose.twist.twist.angular.z);
}