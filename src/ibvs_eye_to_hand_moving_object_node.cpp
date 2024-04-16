//
// Created by skywoodsz on 2022/8/24.
//

#include <ibvs_control/ibvs_eye_to_hand_moving_object.h>
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ibvs_eye_to_hand_moving_object_controller_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    IBVS_EYE_TO_HAND ibvs_controller(nh, nh_private);

    ros::spin();
    return 0;
}

