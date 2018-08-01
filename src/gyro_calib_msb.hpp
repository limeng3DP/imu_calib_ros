#ifndef GYRO_CALIB_MSB_HPP
#define GYRO_CALIB_MSB_HPP
#include <iostream>
#include "IOstreamColor.h"
#include <fstream>
//ros
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <ceres/ceres.h>
#include "ceres_cost_functor.hpp"
#include <eigen3/Eigen/StdVector>

namespace ImuCalibration {

//gyro calibration: misalignment, scale and bias
class GyroCalibMSB
{
public:
    GyroCalibMSB(ros::NodeHandle &_nh):nh(_nh)
    {
        std::string usage="Type 'c' to start gathering samples \
                           \n and rotating imu sensor without linear acceleration.\n";
        std::cout<<GREEN<<usage<<NOCOLOR<<std::endl;

        std::string acc_calib_file_name;
        nh.param<std::string>("acc_calib_file",acc_calib_file_name,"");
        bool read_acc_param_done = true;
        if(!acc_calib_file_name.empty())
        {
            std::ifstream acc_param_file;
            acc_param_file.open(acc_calib_file_name,std::ios::in);
            if(!acc_param_file.is_open())
            {
                std::cout<<RED<<"No accelerometer parameters file is found!"<<NOCOLOR<<std::endl;
                read_acc_param_done = false;
            }
            else
            {
                std::string line;
                std::getline(acc_param_file,line);
                std::cout<<line<<std::endl;
                if(line != "Misalignment Matrix(M):")
                {
                    std::cout<<"Not correct acc parameter file format!"<<std::endl;
                    //read_acc_param_done = false;
                }
                if(read_acc_param_done)
                {
                    for(int i = 0; i < 3; ++i)
                    {
                        std::getline(acc_param_file,line);
                        std::stringstream ss(line);
                        ss >> misalignment_mat(i,0)
                           >> misalignment_mat(i,1)
                           >> misalignment_mat(i,2);
                    }
                    std::getline(acc_param_file,line);
                    for(int i = 0; i < 3; ++i)
                    {
                        std::getline(acc_param_file,line);
                        std::stringstream ss(line);
                        ss >> scale_mat(i,0)
                           >> scale_mat(i,1)
                           >> scale_mat(i,2);
                    }
                    std::getline(acc_param_file,line);
                    std::getline(acc_param_file,line);
                    std::stringstream ss(line);
                    ss >> offset_mat(0,0)
                       >> offset_mat(1,0)
                       >> offset_mat(2,0);
                }
                std::cout<<misalignment_mat<<std::endl;
                std::cout<<scale_mat<<std::endl;
                std::cout<<offset_mat<<std::endl;

            }
        }
        else
            read_acc_param_done = false;

        if(!read_acc_param_done)
        {
            misalignment_mat = Eigen::Matrix<double,3,3>::Identity();
            scale_mat = Eigen::Matrix<double,3,3>::Identity();
            offset_mat = Eigen::Matrix<double,3,1>::Zero();
        }

        gyro_parameters = new double[12];
        //scale parameters
        nh.param<double>("initial/scale_x",gyro_parameters[6],1.0);
        nh.param<double>("initial/scale_y",gyro_parameters[7],1.0);
        nh.param<double>("initial/scale_z",gyro_parameters[8],1.0);
        //bias parameters
        nh.param<double>("initial/bias_x",gyro_parameters[9],0.0);
        nh.param<double>("initial/bias_y",gyro_parameters[10],0.0);
        nh.param<double>("initial/bias_z",gyro_parameters[11],0.0);
        //misalig`n parameters
        gyro_parameters[0] = 0.0; gyro_parameters[1] = 0.0; gyro_parameters[2] = 0.0;
        gyro_parameters[3] = 0.0; gyro_parameters[4] = 0.0; gyro_parameters[5] = 0.0;

        nh.param<int>("total_samples_num",total_samples_num,1000);
        nh.param<int>("sub_interval",sub_interval,10);

        nh.param<std::string>("out_file",out_file_name,"");
        imu_msgs_buffer.reserve(total_samples_num);
        nh.param<double>("gravity_norm",gravity_norm,9.80665);

    }
    ~GyroCalibMSB()
    {
        delete[] gyro_parameters;
    }
    void calibrate();
private:
    void imuCallback(const sensor_msgs::ImuConstPtr &msg);
    bool ceresSolverGyroCalib();
    void saveCalibResult();
private:
    ros::NodeHandle &nh;
    ros::Subscriber imu_sub;
    double *gyro_parameters;
    int total_samples_num = 1000;
    int sub_interval = 100;
    std::string out_file_name;
    std::vector<sensor_msgs::Imu> imu_msgs_buffer;
    double gravity_norm = 9.80665;
    Eigen::Matrix<double,3,3> misalignment_mat;
    Eigen::Matrix<double,3,3> scale_mat;
    Eigen::Matrix<double,3,1> offset_mat;
};

void GyroCalibMSB::calibrate()
{
    char key;
    std::cout<<RED<<"Type 'c' to start gather samples: "<<NOCOLOR;
    std::cin >> key;
    if(key == 'c')
    {
        imu_sub = nh.subscribe("imu", 1,&GyroCalibMSB::imuCallback, this);
        ros::Rate rate(500);
        while(ros::ok())
        {
            if(imu_msgs_buffer.size() == total_samples_num)
                break;
            ros::spinOnce();
            rate.sleep();
        }
        imu_sub.shutdown();
    }
    else
    {
        std::cout<<"unknown key!"<<std::endl;
        return;
    }
    ceresSolverGyroCalib();
}

void GyroCalibMSB::imuCallback(const sensor_msgs::ImuConstPtr& msg)
{
    Eigen::Vector3d acc_meas(msg->linear_acceleration.x,
                             msg->linear_acceleration.y,
                             msg->linear_acceleration.z);

    Eigen::Vector3d rec_acc_meas = misalignment_mat * scale_mat *(acc_meas - offset_mat);
    if(fabs(rec_acc_meas.norm() - gravity_norm) > 0.5 )//no linear acceleration!
        std::cout<<YELLOW<<"Warning: Not pure rotation motion! gravity_norm: "
                 <<rec_acc_meas.norm()<<NOCOLOR<<std::endl;
    std::cout<<"gravity norm rec: "<<rec_acc_meas.norm()<<std::endl;

    //rectified imu with acc misalign scale and bias params
    sensor_msgs::Imu rec_imu;
    rec_imu.linear_acceleration.x = rec_acc_meas(0);
    rec_imu.linear_acceleration.y = rec_acc_meas(1);
    rec_imu.linear_acceleration.z = rec_acc_meas(2);

    rec_imu.angular_velocity = msg->angular_velocity;

    rec_imu.header.stamp = msg->header.stamp;
    imu_msgs_buffer.push_back(rec_imu);
}


bool GyroCalibMSB::ceresSolverGyroCalib()
{
    ceres::Problem problem;
    std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > vAccMeas;
    std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > vGyroMeas;
    std::vector<double> vTimeStamps;
    vAccMeas.reserve(sub_interval);
    vGyroMeas.reserve(sub_interval);
    vTimeStamps.reserve(sub_interval);
    for(int i = 0; i < imu_msgs_buffer.size();++i)
    {
        sensor_msgs::Imu& msg = imu_msgs_buffer[i];
        if(vAccMeas.size() < sub_interval)
        {
            vAccMeas.push_back(Eigen::Vector3d(msg.linear_acceleration.x,
                                               msg.linear_acceleration.y,
                                               msg.linear_acceleration.z));
            vGyroMeas.push_back(Eigen::Vector3d(msg.angular_velocity.x,
                                                msg.angular_velocity.y,
                                                msg.angular_velocity.z));
            vTimeStamps.push_back(msg.header.stamp.toSec());
        }
        else
        {
            Eigen::Vector3d &ref_acc_meas = vAccMeas[0];
            Eigen::Vector3d &cur_acc_meas = vAccMeas[vAccMeas.size() - 1];
            if(fabs(ref_acc_meas.norm() - gravity_norm) > 1.0 ||
                    fabs(cur_acc_meas.norm() - gravity_norm) > 1.0 ||
                    fabs(ref_acc_meas.norm() - cur_acc_meas.norm()) > 0.5)
            {
                std::cout<<RED<<"Not static gravity vector,abort this sample!"<<NOCOLOR<<std::endl;
                vAccMeas.clear();
                vGyroMeas.clear();
                vTimeStamps.clear();
                continue;
            }
            ceres::CostFunction *costFunc =
            new ceres::AutoDiffCostFunction<GyroRotationCostFunctor,1,12>(new GyroRotationCostFunctor(vAccMeas,vGyroMeas,vTimeStamps));
            //ceres::LossFunction *loss_function = new ceres::HuberLoss(2.0);
            problem.AddResidualBlock(costFunc,NULL,gyro_parameters);
            vAccMeas.clear();
            vGyroMeas.clear();
            vTimeStamps.clear();
        }
    }
    ceres::Solver::Options options;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 300;
    ceres::Solver::Summary summary;
    ceres::Solve(options,&problem,&summary);
    std::cout<<GREEN<<"ceres optimization report for gyro msb calibration: "<<NOCOLOR<<std::endl;
    std::cout<< summary.BriefReport() << "\n";
    if(summary.termination_type == ceres::CONVERGENCE )
    {
        saveCalibResult();
    }
    else
        std::cout<<RED<<"ceres optimization for gyro msb params: not convergence!"<<NOCOLOR<<std::endl;
}
void GyroCalibMSB::saveCalibResult()
{
    if(out_file_name.empty())
        out_file_name = std::string("./gyro-msb-calibration-result.txt");

    std::ofstream out_file;
    out_file.open(out_file_name.c_str(),std::ios::out);
    if(!out_file.is_open())
    {
        std::cout<<RED<<"Can not open output file "<<out_file_name<<NOCOLOR<<std::endl;
        return;
    }
    Eigen::Matrix<double,3,3> misalign_mat;
    misalign_mat << 1.0, gyro_parameters[0], gyro_parameters[1],
                    gyro_parameters[2], 1.0, gyro_parameters[3],
                    gyro_parameters[4],gyro_parameters[5], 1.0;
    Eigen::Matrix<double,3,3> scale_mat;
    scale_mat << gyro_parameters[6], 0.0, 0.0,
                 0.0,gyro_parameters[7], 0.0,
                 0.0, 0.0,gyro_parameters[8];
    Eigen::Matrix<double,3,1> offset_mat;
    offset_mat << gyro_parameters[9],gyro_parameters[10],gyro_parameters[11];

    out_file << "Misalignment Matrix(M):\n"<<misalign_mat
             <<"\nAxis Scale Matrix(S):\n"<<scale_mat
             <<"\nBias Vector(B): \n"<<offset_mat.transpose()<<std::endl;

    out_file <<"Rectify Matrix(M * S): \n"<<misalign_mat * scale_mat<<std::endl;


    std::cout << "Misalignment Matrix(M):\n"<<misalign_mat
             <<"\nAxis Scale Matrix(S):\n"<<scale_mat
             <<"\nBias Vector(B): \n"<<offset_mat.transpose()<<std::endl;
    std::cout <<"Rectify Matrix(M * S): \n"<<misalign_mat * scale_mat<<std::endl;

    out_file.close();
}

}//namespace ImuCalibration

#endif // GYRO_CALIB_MSB_HPP
