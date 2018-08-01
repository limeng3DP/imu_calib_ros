#ifndef CALIBRATOR_HPP
#define CALIBRATOR_HPP
#include <ros/ros.h>
#include "acc_calib_msb.hpp"
#include "gyro_calib_msb.hpp"
#include "imu_allanVar_calib.hpp"
namespace ImuCalibration {
class Calibrator
{
public:
    Calibrator(ros::NodeHandle &_nh):nh(_nh)
    {}
    void calibrate();
private:
    ros::NodeHandle &nh;
};

void Calibrator::calibrate()
{
    int calib_type = 0;
    nh.param<int>("calib_type", calib_type, 0);
    switch (calib_type)
    {
    case 0:
    {
        AccCalibMSB calibr0(nh);//acc_msb
        calibr0.calibrate();
        break;
    }
    case 1:
    {
        ImuAllanVarCalib calibr1(nh);//acc_noise
        calibr1.calibrate();
        break;
    }
    case 2:
    {
        GyroCalibMSB calibr2(nh);//gyro_msb
        calibr2.calibrate();
        break;
    }
    case 3:
    {
        ImuAllanVarCalib calibr3(nh);//gyro_msb
        calibr3.calibrate();
        break;
    }
    default:
        break;
    }
}


}//namespace ImuCalibration

#endif // CALIBRATOR_HPP
