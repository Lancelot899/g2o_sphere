#include <stdio.h>
#include <iostream>

#include <Eigen/Sparse>


#include <ceres/ceres.h>

#include "Sphere.h"

class PoseGraphError : public ceres::SizedCostFunction<6, 6, 6> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PoseGraphError(int i, Sophus::SE3d &T_ij, Eigen::Matrix<double, 6, 6> &information) {
        T_ij_ = T_ij;
        Eigen::LLT<Eigen::Matrix<double, 6, 6>> llt(information);
        sqrt_information_ = llt.matrixL();
        id = i;
    }


    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const {
        //printf("param %d:[%lf, %lf, %lf, %lf, %lf, %lf]\n", id, (*parameters)[0], (*parameters)[1], (*parameters)[2], (*parameters)[3], (*parameters)[4], (*parameters)[5]);
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> lie_i(*parameters);
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> lie_j(*(parameters + 1));

//        Eigen::Matrix<double, 6, 1> lie_i;
//        Eigen::Matrix<double, 6, 1> lie_j;
//
//        for(int i = 0; i < 6; ++i) {
//            lie_i(i, 0) = parameters[0][i];
//            lie_j(i, 0) = parameters[0][i];
//        }
//
        Eigen::Matrix<double, 6, 6> Jac_i;
        Eigen::Matrix<double, 6, 6> Jac_j;
        //printf("get lie\n");
        Sophus::SE3d T_i = Sophus::SE3d::exp(lie_i);
        //std::cout << T_i.matrix3x4() << std::endl;
        Sophus::SE3d T_j = Sophus::SE3d::exp(lie_j);
        Sophus::SE3d Tij_estimate = T_i * T_j.inverse();
        Sophus::SE3d err = Tij_estimate * T_ij_.inverse();
        //std::cout << err.log() << std::endl;
        Eigen::Matrix<double, 6, 6> Jl;
        Jl.block(3, 3, 3, 3) = Jl.block(0, 0, 3, 3) = Sophus::SO3d::hat(err.so3().log());
        Jl.block(0, 3, 3, 3) = Sophus::SO3d::hat(err.translation());
        Jl.block(3, 0, 3, 3) = Eigen::Matrix3d::Zero();
        Eigen::Matrix<double, 6, 6> I = Eigen::Matrix<double, 6, 6>::Identity();
        Jl.noalias() = sqrt_information_ * (I - 0.5 * Jl);

        Jac_i = Jl;

        Eigen::Matrix<double, 6, 1> err_ = sqrt_information_ * err.log();

        const Eigen::Matrix<double, 3, 3>& R = Tij_estimate.rotationMatrix();
        Eigen::Matrix<double, 6, 6> adj;
        adj.block<3, 3>(3, 3) = adj.block<3, 3>(0, 0) = R;
        adj.block<3, 3>(0, 3) = Sophus::SO3d::hat(Tij_estimate.translation()) * R;
        adj.block<3, 3>(3, 0) = Eigen::Matrix<double, 3, 3>::Zero(3, 3);

       // printf("compute adj ok!\n");

        Jac_j = -Jac_i * adj;
        int k = 0;
        for(int i = 0; i < 6; i++) {
            residuals[i] = err_(i);
            if(jacobians) {
                for (int j = 0; j < 6; ++j) {
                    if (jacobians[0])
                        jacobians[0][k] = Jac_i(i, j);
                    if (jacobians[1])
                        jacobians[1][k] = Jac_j(i, j);
                    k++;
                }
            }
        }

        return true;
    }

private:
    int id;
    Sophus::SE3d T_ij_;
    Eigen::Matrix<double, 6, 6> sqrt_information_;
};




Sphere::Sphere()
{
}


class CERES_EXPORT SE3Parameterization : public ceres::LocalParameterization {
public:
    virtual ~SE3Parameterization() {}
    virtual bool Plus(const double* x,
                      const double* delta,
                      double* x_plus_delta) const;
    virtual bool ComputeJacobian(const double* x,
                                 double* jacobian) const;
    virtual int GlobalSize() const { return 6; }
    virtual int LocalSize() const { return 6; }
};

bool SE3Parameterization::ComputeJacobian(const double *x, double *jacobian) const {
    ceres::MatrixRef(jacobian, 6, 6) = ceres::Matrix::Identity(6, 6);
    return true;
}

bool SE3Parameterization::Plus(const double* x,
                               const double* delta,
                               double* x_plus_delta) const {
    Eigen::Map<const Eigen::Matrix<double, 6, 1>> lie(x);
    Eigen::Map<const Eigen::Matrix<double, 6, 1>> delta_lie(delta);

    Sophus::SE3d T = Sophus::SE3d::exp(lie);
    Sophus::SE3d delta_T = Sophus::SE3d::exp(delta_lie);
    Eigen::Matrix<double, 6, 1> x_plus_delta_lie = (delta_T * T).log();

    for(int i = 0; i < 6; ++i) x_plus_delta[i] = x_plus_delta_lie(i, 0);

    return true;

}


bool Sphere::optimize(int iter_) {
    if(vertexes.empty() == true || edges.empty() == true)
        return false;

    ceres::Problem problem;
    for(size_t i = 0; i < edges.size(); ++i) {
        ceres::CostFunction* costFun = new PoseGraphError(i, edges[i].pose, edges[i].infomation);
        problem.AddResidualBlock(costFun, new ceres::HuberLoss(1.5), vertexes[edges[i].i].pose.data(),
                                 vertexes[edges[i].j].pose.data());
    }

    for(size_t i = 0; i < vertexes.size(); ++i) {
        problem.SetParameterization(vertexes[i].pose.data(), new SE3Parameterization());
    }


    //printf("optimization start!\n");

    ceres::Solver::Options options;
    options.dynamic_sparsity = true;
    options.max_num_iterations = iter_;
    options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
    options.minimizer_type = ceres::TRUST_REGION;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.minimizer_progress_to_stdout = true;
    options.dogleg_type = ceres::SUBSPACE_DOGLEG;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";

    return true;
}




