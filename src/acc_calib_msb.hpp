#ifndef ACC_CALIB_MSB_HPP
#define ACC_CALIB_MSB_HPP
#include <iostream>
#include "IOstreamColor.h"
#include <fstream>

//ros
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <eigen3/Eigen/StdVector>
#include <ceres/ceres.h>
#include "ceres_cost_functor.hpp"
namespace ImuCalibration {

class AccCalibMSB
{
public:
    AccCalibMSB(ros::NodeHandle &_nh):nh(_nh)
    {
        std::string usage="Usage: put imu sensor still and then type 'c' to begin sampling,\
                           \n repeate this process until collecting enough data,type 'q' to quit data collection.";
        std::cout<<GREEN<<usage<<NOCOLOR<<std::endl;
        nh.param<int>("sample_num_per_pose",sample_num_per_pose,300);
        nh.param<double>("gravity_norm",gravity_norm,9.80665);
        imu_msgs_buffer.reserve(sample_num_per_pose);
        if(acc_parameters != NULL)
            acc_parameters = NULL;
        acc_parameters = new double[12];

        //scale parameters
        nh.param<double>("initial/scale_x",acc_parameters[6],1.0);
        nh.param<double>("initial/scale_y",acc_parameters[7],1.0);
        nh.param<double>("initial/scale_z",acc_parameters[8],1.0);

        //bias parameters
        nh.param<double>("initial/bias_x",acc_parameters[9],0.0);
        nh.param<double>("initial/bias_y",acc_parameters[10],0.0);
        nh.param<double>("initial/bias_z",acc_parameters[11],0.0);
        //misalign parameters
        acc_parameters[0] = 0.0; acc_parameters[1] = 0.0; acc_parameters[2] = 0.0;
        acc_parameters[3] = 0.0; acc_parameters[4] = 0.0; acc_parameters[5] = 0.0;
        vAccMeas.reserve(30);

        nh.param<std::string>("out_file",out_file_name,"");
    }
    ~AccCalibMSB()
    {
        delete[] acc_parameters;
    }
    void calibrate();
private:
    void imuCallback(const sensor_msgs::ImuConstPtr& msg);
    void computeAverageSample(Eigen::Vector3d &average_acc);
    void ceresSolverAccCalib();
    void saveCalibResult();
private:
    ros::NodeHandle &nh;
    ros::Subscriber imu_sub;
    bool sample_ready_per_pose = false;
    int sample_num_per_pose = 300;
    double gravity_norm = 9.80665;
    double *acc_parameters = NULL;
    std::string out_file_name;
    std::vector<sensor_msgs::ImuConstPtr> imu_msgs_buffer;
    std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > vAccMeas;
};

void AccCalibMSB::calibrate()
{
    while(true)
    {
        char key;
        std::cout<<RED<<"Type 'c' or 'q': "<<NOCOLOR;
        std::cin >> key;
        if(key == 'c')
        {
            imu_sub = nh.subscribe("imu",1, &AccCalibMSB::imuCallback, this);
            ros::Rate rate(500);
            while(ros::ok())
            {
                if(sample_ready_per_pose)
                    break;
                ros::spinOnce();
                rate.sleep();
            }
            imu_sub.shutdown();
            Eigen::Vector3d average_acc_meas(0.0,0.0,0.0);
            computeAverageSample(average_acc_meas);
            vAccMeas.push_back(average_acc_meas);
            sample_ready_per_pose = false;
            imu_msgs_buffer.clear();
        }
        else if(key == 'q')
            break;
        else
            std::cout<<RED<<"Unknown key!"<<NOCOLOR<<std::endl;
    }
    ceresSolverAccCalib();
}

void AccCalibMSB::imuCallback(const sensor_msgs::ImuConstPtr& msg)
{

    if(!sample_ready_per_pose)
    {
        Eigen::Vector3d acc_meas(msg->linear_acceleration.x,
                                 msg->linear_acceleration.y,
                                 msg->linear_acceleration.z);
        if(fabs(acc_meas.norm() - gravity_norm) < 0.6)
            imu_msgs_buffer.push_back(msg);
        else
            std::cout<<RED<<"Warning: imu sensor is not kept still! \
                             Calibration result may be wrong!"<<NOCOLOR<<std::endl;
    }

    if(imu_msgs_buffer.size() == sample_num_per_pose)
        sample_ready_per_pose = true;
}


void AccCalibMSB::computeAverageSample(Eigen::Vector3d &average_acc)
{
    for(int i = 0;i < imu_msgs_buffer.size();++i)
    {
        sensor_msgs::ImuConstPtr& imu = imu_msgs_buffer[i];
        average_acc += Eigen::Vector3d(imu->linear_acceleration.x,
                                      imu->linear_acceleration.y,
                                      imu->linear_acceleration.z);
    }
    average_acc /= imu_msgs_buffer.size();
}

void AccCalibMSB::ceresSolverAccCalib()
{
    ceres::Problem problem;

    for(int i = 0;i < vAccMeas.size();++i)
    {
        Eigen::Vector3d &acc_meas = vAccMeas[i];
        ceres::CostFunction *costFunc =
        new ceres::AutoDiffCostFunction<AccGravityCostFunctor,1,12>(new AccGravityCostFunctor(acc_meas));
        ceres::LossFunction *loss_function = new ceres::HuberLoss(1.0);
        problem.AddResidualBlock(costFunc,loss_function,acc_parameters);
    }
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options,&problem,&summary);
    if(summary.termination_type == ceres::CONVERGENCE )
    {
        std::cout<<GREEN<<"ceres optimization report for accelerometer msb calibration: "<<NOCOLOR<<std::endl;
        std::cout << summary.BriefReport() << "\n";
        saveCalibResult();
    }
    else
        std::cout<<RED<<"ceres optimization for acc msb params: not convergence!"<<NOCOLOR<<std::endl;
}

void AccCalibMSB::saveCalibResult()
{

    if(out_file_name.empty())
        out_file_name = std::string("./acc-msb-calibration-result.txt");

    std::ofstream out_file;
    out_file.open(out_file_name.c_str(),std::ios::out);
    if(!out_file.is_open())
    {
        std::cout<<RED<<"Can not open output file "<<out_file_name<<NOCOLOR<<std::endl;
        return;
    }
    Eigen::Matrix<double,3,3> misalign_mat;
    misalign_mat << 1.0, acc_parameters[0], acc_parameters[1],
                    acc_parameters[2], 1.0, acc_parameters[3],
                    acc_parameters[4],acc_parameters[5], 1.0;
    Eigen::Matrix<double,3,3> scale_mat;
    scale_mat << acc_parameters[6], 0.0, 0.0,
                 0.0,acc_parameters[7], 0.0,
                 0.0, 0.0,acc_parameters[8];
    Eigen::Matrix<double,3,1> offset_mat;
    offset_mat << acc_parameters[9],acc_parameters[10],acc_parameters[11];

    out_file <<"Misalignment Matrix(M):\n"<<misalign_mat
             <<"\nAxis Scale Matrix(S):\n"<<scale_mat
             <<"\nBias Vector(B): \n"<<offset_mat.transpose()<<std::endl;

    out_file <<"Rectify Matrix(M * S): \n"<<misalign_mat * scale_mat<<std::endl;


    std::cout <<"Misalignment Matrix(M):\n"<<misalign_mat
              <<"\nAxis Scale Matrix(S):\n"<<scale_mat
              <<"\nBias Vector(B): \n"<<offset_mat.transpose()<<std::endl;

    std::cout <<"Rectify Matrix(M * S): \n"<<misalign_mat * scale_mat<<std::endl;

    out_file.close();
}
}//namespace ImuCalibration

#endif // ACC_CALIB_MSB_HPP
