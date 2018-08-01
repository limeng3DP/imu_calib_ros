#ifndef IMU_ALLANVAR_CALIB_HPP
#define IMU_ALLANVAR_CALIB_HPP
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <set>
#include <cmath>
#include <ros/ros.h>
#include <ceres/ceres.h>
#include "gnu_plot.hpp"


#include "IOstreamColor.h"
#include "ceres_cost_functor.hpp"

namespace ImuCalibration {

class ImuAllanVarCalib
{
public:
    ImuAllanVarCalib(ros::NodeHandle &nh)
    {
        int calib_type = 0;
        nh.param<int>("calib_type", calib_type, 0);
        if(calib_type == 0 || calib_type == 1)
            acc_sensor = true;
        else if(calib_type == 2 || calib_type == 3)
            acc_sensor = false;

        if(acc_sensor)
        {
            nh.param<std::string>("acc_raw_data_file", raw_data_file,"");
        }
        else
        {
            nh.param<std::string>("gyro_raw_data_file", raw_data_file,"");
        }
        nh.param<bool>("plot_allan_curve", plot_allan_curve,true);
        nh.param<int>("allan_sample_pts_num", allan_sample_pts_num,30);
        nh.param<double>("sample_frequency", sample_frequency,200.0);
        double max_sample_duration;
        nh.param<double>("max_sample_duration",max_sample_duration,3.0);
        max_sample_num =std::ceil(max_sample_duration * 3600 * sample_frequency);

        if(noise_params != NULL)
            noise_params = NULL;
        noise_params= new double[5];

        nh.param<double>("initial/Q",initial_Q,0.00001);
        nh.param<double>("initial/N",initial_N,0.00001);
        nh.param<double>("initial/B",initial_B,0.00001);
        nh.param<double>("initial/K",initial_K,0.00001);
        nh.param<double>("initial/R",initial_R,0.00001);
        nh.param<double>("ceres_iter_num",ceres_iter_num,100);

        std::string out_file_name;

        nh.param<std::string>("out_file_name",out_file_name,"");

        if(out_file_name.empty())
        {
            if(acc_sensor)
                out_file_name = std::string("./acc-noise-calibration-result.txt");
            else
                out_file_name = std::string("./gyro-noise-calibration-result.txt");
        }
        out_file.open(out_file_name.c_str(),std::ios::out);
        if(!out_file.is_open())
        {
            std::cout<<RED<<"Can not open output file "<<out_file_name<<NOCOLOR<<std::endl;
            return;
        }
    }
    void calibrate();
    ~ImuAllanVarCalib()
    {
        out_file.close();
    }
private:
    bool readRawDataFromFile(std::string fullFileName,
                             std::vector<std::vector<double> > &raw_data);
    void overlapAllanVar(std::vector<double> &data, int log_space_pts_num,
                           std::vector<std::pair<double, double> > &allan_vars_result);
    bool optimiseRandomNoiseParams(std::vector<std::pair<double, double> > &allan_vars,int axis);
    void saveCalibResult(int axis);
    void computeAllanDev(std::vector<std::pair<double, double> > &allan_vars,
                         std::vector<std::pair<double, double> > &allan_devs,
                         bool acc_sensor_type);

private:
    std::string raw_data_file;
    bool acc_sensor = true;
    bool plot_allan_curve = true;
    int allan_sample_pts_num;
    long int max_sample_num;
    double sample_frequency;
    double *noise_params = NULL;
    double ceres_iter_num = 100;
    std::ofstream out_file;

    double initial_Q,initial_N,initial_B,initial_K,initial_R;
};

void ImuAllanVarCalib::calibrate()
{
    std::vector<std::vector<double> > raw_data;
    if(raw_data_file.empty())
        return;
    if(!readRawDataFromFile(raw_data_file,raw_data))
    {
        std::cout<<RED<<"Loading imu raw data failed! "
                      <<"check the raw data file!"<<NOCOLOR<<std::endl;
        return;
    }
    PlotUtils plotutil;
    if(plot_allan_curve)
    {
        plotutil.open();
        plotutil.command("set terminal x11");
        plotutil.command("set lmargin 16");
        plotutil.command("set size 10,10");
        plotutil.command("set origin 0,0");
        plotutil.command("set multiplot layout 3,1");
        plotutil.command("set xlabel 'Cluster time(sec)'");
        plotutil.command("set logscale xy 10");
        plotutil.command("set xrange [0.001:1000000]");
        plotutil.command("set grid");

        if(acc_sensor)
            plotutil.command("set ylabel 'AllanDeviation (m/s/s)'");
        else
            plotutil.command("set ylabel 'AllanDeviation (deg/s)'");

    }
    int N = raw_data.size();
    for(int i = 0; i < N; ++i)
    {
        std::vector<std::pair<double,double> > allan_vars_result;
        overlapAllanVar(raw_data[i],allan_sample_pts_num,allan_vars_result);
        optimiseRandomNoiseParams(allan_vars_result,i);
        if(plot_allan_curve)
        {
            //compute allan deviation
            std::vector<std::pair<double,double> > allan_devs;
            computeAllanDev(allan_vars_result,allan_devs,acc_sensor);

            plotutil.command("$DATA << EOD");
            for(int k = 0; k < allan_devs.size(); ++k)
            {
                std::ostringstream ss;
                ss << allan_devs[k].first << " " <<allan_devs[k].second<<"\n";
                plotutil.command(ss.str());
            }
            plotutil.command("EOD");
            std::ostringstream cmd_ss;
            cmd_ss << "set title '"<<i<<"-Axis Allan Deviation Curve'";
            plotutil.command(cmd_ss.str());
            plotutil.command("plot $DATA using 1:2");
        }
    }
    if(plot_allan_curve)
    {
        plotutil.command("pause mouse");
        plotutil.command("exit");
        plotutil.close();
    }

}
bool ImuAllanVarCalib::readRawDataFromFile(std::string fullFileName,
                                           std::vector<std::vector<double> > &raw_data)
{
    raw_data.resize(3);//3-axis sensor
    for(int i = 0;i < 3;++i)
        raw_data[i].reserve(max_sample_num);
    std::ifstream raw_data_file;
    raw_data_file.open(fullFileName,std::ios::in);
    if(!raw_data_file.is_open())
    {
        std::cerr<<"Failed to open raw data file!"<<std::endl;
        return false;
    }
    std::cout<<GREEN<<"Loading raw data from file..."<<NOCOLOR<<std::endl;
    std::string line;
    while(std::getline(raw_data_file,line))
    {
        double data0,data1,data2;
        std::sscanf(line.c_str(),"%lf\t%lf\t%lf",&data0,&data1,&data2);
        raw_data[0].push_back(data0);
        raw_data[1].push_back(data1);
        raw_data[2].push_back(data2);
    }
    //remove mean value from raw data
    #ifdef USE_OPENMP
    #pragma message("remove mean value from accel raw data : openmp activated!")
    const unsigned int nb_max_thread = omp_get_max_threads();
    omp_set_num_threads(nb_max_thread);
    #pragma omp parallel for schedule(dynamic) if (nb_max_thread > 0)
    #endif
    for(int i = 0;i < raw_data.size(); ++i)
    {
        std::vector<double>& single_axis_raw_data = raw_data[i];
        long double sum = std::accumulate(single_axis_raw_data.begin(),single_axis_raw_data.end(),0.0l);
        double mean_value = sum / single_axis_raw_data.size();
        for(int j = 0; j < single_axis_raw_data.size();++j)
            single_axis_raw_data[j] -= mean_value;
    }
    std::cout<<GREEN<<"Loading raw data finished!"<<NOCOLOR<<std::endl;
    return true;
}
void ImuAllanVarCalib::overlapAllanVar(std::vector<double> &data, int log_space_pts_num,
                                         std::vector<std::pair<double,double> > &allan_vars_result)
{
    double t0 = 1.0 / sample_frequency; //sample interval

    //figure out how big the output data set is
    long int N = data.size();
    long int maxN = pow(2.0,std::max(0.0, floor(log2(N / 2.0))));
    std::vector<double> theta(data.size());
    theta[0] = data[0] * t0;
    for(long int i = 1; i < N; ++i)
    {
        theta[i] = theta[i - 1] + data[i] * t0;
    }

    double log_space_end = log10(maxN);
    double log_space_start = 0;
    double log_space_inc = (log_space_end - log_space_start) / (log_space_pts_num - 1);

    std::set<long int,std::less<int> > log_space_samples;
    for(int i = 0; i < log_space_pts_num; ++i)
    {
        double log_space_sample = log_space_start + i * log_space_inc;
        //insert std::pari<average_factor,allan_var>
        log_space_samples.insert(static_cast<long int>(ceil(pow(10,log_space_sample))));
    }
    std::vector<long int> vlog_space_sampels;
    vlog_space_sampels.reserve(log_space_samples.size());
    for(auto it = log_space_samples.begin(),it_end = log_space_samples.end();
        it != it_end; ++it)
        vlog_space_sampels.push_back(*it);

    allan_vars_result.resize(log_space_samples.size());
    #ifdef USE_OPENMP
    #pragma message("compute allan variance : openmp activated!")
    const unsigned int nb_max_thread = omp_get_max_threads();
    omp_set_num_threads(nb_max_thread);
    #pragma omp parallel for schedule(dynamic) if (nb_max_thread > 0)
    #endif
    for(int i =0; i < vlog_space_sampels.size(); ++i)
    {
        long int m_i = vlog_space_sampels[i];
        double sigma2 = 0;
        for(long int k = 1 ; k <= N - 2 * m_i; ++k)
        {
            double sum = theta[k + 2 * m_i] - 2 * theta[ k + m_i] + theta[k];
            sigma2 += sum * sum;
        }
        double tau = m_i * t0;
        sigma2 /= 2 * tau * tau * (N - 2 * m_i);
        allan_vars_result[i].first = tau;
        allan_vars_result[i].second = sigma2;
    }
}

bool ImuAllanVarCalib::optimiseRandomNoiseParams(std::vector<std::pair<double, double> > &allan_vars, int axis)
{
    ceres::Problem problem;
    noise_params[0] = initial_Q;noise_params[1] = initial_N;
    noise_params[2] = initial_B;noise_params[3] = initial_K;
    noise_params[4] = initial_R;
    for(int i = 0; i < allan_vars.size();++i)
    {
        std::pair<double,double>& allan_var = allan_vars[i];
        double tau = allan_var.first;
        double sigma2 = allan_var.second;
        ceres::CostFunction *costFunc =
        new ceres::AutoDiffCostFunction<AllanSigmaConstFunctor,1,5>(new AllanSigmaConstFunctor(tau,sigma2));
        ceres::LossFunction *loss_function = new ceres::HuberLoss(1.0);
        problem.AddResidualBlock(costFunc,loss_function,noise_params);
    }
    ceres::Solver::Options options;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = ceres_iter_num;
    ceres::Solver::Summary summary;
    ceres::Solve(options,&problem,&summary);
    if(summary.termination_type == ceres::CONVERGENCE || summary.final_cost < 1.0)
    {
        saveCalibResult(axis);
    }
    else
    {
        std::cout<<RED<<std::endl<<summary.BriefReport()<<NOCOLOR<<std::endl;
        std::cout<<RED<<axis<<"-axis ceres optimization is not convergent,"
                      <<"adjust launch parameters and try again!"<<NOCOLOR<<std::endl;
        return false;
    }
    return true;
}

void ImuAllanVarCalib::computeAllanDev(std::vector<std::pair<double, double> > &allan_vars,
                                       std::vector<std::pair<double, double> > &allan_devs,
                                       bool acc_sensor_type)
{
    if(acc_sensor_type)
        for(auto& var: allan_vars)
            allan_devs.push_back(std::pair<double,double>(var.first, sqrt(var.second)) );
    else
        for(auto& var: allan_vars)
            allan_devs.push_back(std::pair<double,double>(var.first, sqrt(var.second) * 180.0 / 3.14159265) );
}

void ImuAllanVarCalib::saveCalibResult(int axis)
{
    if(acc_sensor)
    {
        std::stringstream ss;
        ss << axis <<"-axis accelerometer noise parameters";
        out_file <<"*********************"<<ss.str()<<"**************************\n";
        out_file <<"Quantisation noise: "<<noise_params[0]<<" m/s\n";
        out_file <<"noise density (velocity random walk): "<<noise_params[1]<<" m/s^2/sqrt(Hz)\n";
        out_file <<"bias instability: "<<noise_params[2]<<" m/s^2\n";
        out_file <<"rate random wakl: "<<noise_params[3]<<" (m/s^2) * sqrt(Hz)\n";
        out_file <<"rate ramp: "<<noise_params[4]<<" m/s^2/s"<<std::endl;

        std::cout <<"*********************"<<ss.str()<<"**************************\n";
        std::cout <<"Quantisation noise: "<<noise_params[0]<<" m/s\n";
        std::cout <<"noise density (velocity random walk): "<<noise_params[1]<<" m/s^2/sqrt(Hz)\n";
        std::cout <<"bias instability: "<<noise_params[2]<<" m/s^2\n";
        std::cout <<"rate random wakl: "<<noise_params[3]<<" (m/s^2) * sqrt(Hz)";
        std::cout <<"rate ramp: "<<noise_params[4]<<" m/s^2/s"<<std::endl<<std::endl;
    }
    else
    {
        std::stringstream ss;
        ss << axis <<"-axis gyrometer noise parameters";
        out_file <<"*********************"<<ss.str()<<"**************************\n";
        out_file <<"Quantisation noise: "<<noise_params[0]<<" rad\n";
        out_file <<"noise density (velocity random walk): "<<noise_params[1]<<" rad/sqrt(s)\n";
        out_file <<"bias instability: "<<noise_params[2]<<" rad/s\n";
        out_file <<"rate random wakl: "<<noise_params[3]<<" rad/s/sqrt(s)\n";
        out_file <<"rate ramp: "<<noise_params[4]<<" rad/s^2"<<std::endl;

        std::cout <<"*********************"<<ss.str()<<"**************************\n";
        std::cout <<"Quantisation noise: "<<noise_params[0]<<" rad\n";
        std::cout <<"noise density (velocity random walk): "<<noise_params[1]<<" rad/sqrt(s)\n";
        std::cout <<"bias instability: "<<noise_params[2]<<" rad/s\n";
        std::cout <<"rate random wakl: "<<noise_params[3]<<" rad/s/sqrt(s)\n";
        std::cout <<"rate ramp: "<<noise_params[4]<<" rad/s^2"<<std::endl<<std::endl;
    }
}
}//namespace ImuCalibration


#endif // IMU_ALLANVAR_CALIB_HPP
