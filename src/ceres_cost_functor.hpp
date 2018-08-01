#ifndef CERES_COST_FUNCTOR_HPP
#define CERES_COST_FUNCTOR_HPP
#include <eigen3/Eigen/Eigen>

#include "ceres/ceres.h"
#define GRAVITY 9.80665

namespace ImuCalibration {

struct AccGravityCostFunctor{
AccGravityCostFunctor(Eigen::Vector3d& _acc_meas):
    acc_meas(_acc_meas)
{}

template <typename T>
bool operator()(const T* const param, T* residual) const
{
    Eigen::Matrix<T,3,3> orthogonal_matrix;
    orthogonal_matrix << T(1.0), param[0], param[1],
                         param[2], T(1.0), param[3],
                         param[4],param[5], T(1.0);

    Eigen::Matrix<T,3,3> scale_matrix;
    scale_matrix << param[6], T(0.0) , T(0.0),
                    T(0.0),param[7], T(0.0),
                    T(0.0), T(0.0),param[8];

    Eigen::Matrix<T,3,1> offset_matrix;
    offset_matrix << param[9],param[10],param[11];
    Eigen::Matrix<T,3,1> acc_meas_T_type;
    acc_meas_T_type << T(acc_meas[0]),T(acc_meas[1]),T(acc_meas[2]);
    //Misalign * scale * (acc_meas - bias)
    residual[0] = (orthogonal_matrix * scale_matrix*( acc_meas_T_type - offset_matrix ) ).norm() - T(GRAVITY);
    return true;

}
private:
    Eigen::Vector3d acc_meas;
};

struct GyroRotationCostFunctor{
GyroRotationCostFunctor(std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> >&_vAccMeas,
                        std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> >&_vGyroMeas,
                        std::vector<double> &_vTimeStamps):
    vAccMeas(_vAccMeas),vGyroMeas(_vGyroMeas),vTimeStamps(_vTimeStamps)
{

}

template <typename T>
bool operator()(const T* const param, T* residual) const
{
    Eigen::Matrix<T,3,3> orthogonal_matrix;
    orthogonal_matrix << T(1.0), param[0], param[1],
                         param[2], T(1.0), param[3],
                         param[4],param[5], T(1.0);

    Eigen::Matrix<T,3,3> scale_matrix;
    scale_matrix << param[6], T(0.0) , T(0.0),
                    T(0.0),param[7], T(0.0),
                    T(0.0), T(0.0),param[8];

    Eigen::Matrix<T,3,1> offset_matrix;
    offset_matrix << param[9],param[10],param[11];
    Eigen::Matrix<T,3,1> delt_theta;
    delt_theta << T(0.0),T(0.0),T(0.0);
    for(int i = 1; i < vGyroMeas.size();++i )
    {
        Eigen::Vector3d &gyro_meas_prev = vGyroMeas[i - 1];
        Eigen::Matrix<T,3,1> gyro_meas_prev_T_type;
        gyro_meas_prev_T_type << T(gyro_meas_prev[0]),T(gyro_meas_prev[1]),T(gyro_meas_prev[2]);
        T dt =T(vTimeStamps[i] - vTimeStamps[i - 1]) ;
        delt_theta += orthogonal_matrix * scale_matrix * ( gyro_meas_prev_T_type - offset_matrix) * dt;
    }
    //这个旋转并不一定是小角度，所以最好不用小角度近似
    Eigen::AngleAxis<T> aa_r_c = Eigen::AngleAxis<T>(delt_theta.norm(),
                                                 delt_theta.normalized());
    Eigen::Matrix<T,3,3> rot_r_c = aa_r_c.toRotationMatrix().transpose();

    Eigen::Vector3d &acc_meas_ref = vAccMeas[0];
    Eigen::Vector3d &acc_meas_cur = vAccMeas[vAccMeas.size() - 1];

    Eigen::Matrix<T,3,1> gravity_ref;
    gravity_ref << T(acc_meas_ref[0]),T(acc_meas_ref[1]),T(acc_meas_ref[2]);

    Eigen::Matrix<T,3,1> gravity_cur;
    gravity_cur << T(acc_meas_cur[0]),T(acc_meas_cur[1]),T(acc_meas_cur[2]);

    //rotation from ref to cur
    //    Eigen::Quaternion<T> q_r_c_ref = Eigen::Quaternion<T>::FromTwoVectors(gravity_ref, gravity_cur);
    //    Eigen::Quaternion<T> q_r_c_obs(rot_r_c);
    //    T residual_0 = q_r_c_ref.w() - q_r_c_obs.w();

    //    T residual_1 = q_r_c_ref.x() - q_r_c_obs.x();
    //    T residual_2 = q_r_c_ref.y() - q_r_c_obs.y();
    //    T residual_3 = q_r_c_ref.z() - q_r_c_obs.z();

    //std::cout<<"q_r_c_ref: "<<q_r_c_ref.w()<<"\t"<<q_r_c_ref.x()<<"\t"<<q_r_c_ref.y()<<"\t"<<q_r_c_ref.z()<<std::endl;
    //std::cout<<"q_r_c_obs: "<<q_r_c_obs.w()<<"\t"<<q_r_c_obs.x()<<"\t"<<q_r_c_obs.y()<<"\t"<<q_r_c_obs.z()<<std::endl;


//    residual[0] = ceres::sqrt(residual_0 * residual_0
//                              + residual_1 * residual_1
//                              + residual_2 * residual_2
//                              + residual_3 * residual_3);

    residual[0] = (gravity_cur - rot_r_c * gravity_ref).norm();
    return true;
}
private:
    std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> >& vAccMeas;
    std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> >& vGyroMeas;
    std::vector<double>& vTimeStamps;

};

struct AllanSigmaConstFunctor
{
    AllanSigmaConstFunctor(double _tau,double _sigma2):tau(_tau),sigma2(_sigma2)
    {}
    template <typename T>
    bool operator()(const T* const param, T* residual) const
    {

        T tau_2 = T(tau) * T(tau);
        const T& Q = param[0];
        const T& N = param[1];
        const T& B = param[2];
        const T& K = param[3];
        const T& R = param[4];
        T sigma2_eval = T(3.0) * Q * Q / tau_2  + N * N / tau + (B / T(0.6648))*(B / T(0.6648)) + K * K * tau / T(3.0) + R * R * tau_2 / T(2.0);
        residual[0] = ceres::log( (sigma2_eval)/T(sigma2)) / ceres::log(T(10.0));
        return true;
    }
private:
    double tau;
    double sigma2;
};

}//namespace imuCalib


#endif // CERES_COST_FUNCTOR_HPP
