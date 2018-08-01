#include <ros/ros.h>
#include "calibrator.hpp"
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "imu_calib_node");
    ros::NodeHandle nh("~");
    ImuCalibration::Calibrator calib(nh);
    calib.calibrate();
    return 0;
}
