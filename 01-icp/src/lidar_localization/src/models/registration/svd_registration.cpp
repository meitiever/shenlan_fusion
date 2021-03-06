/*
 * @Description: ICP 匹配模块
 * @Author: Ge Yao
 * @Date: 2020-10-24 21:46:45
 */
#include "lidar_localization/models/registration/svd_registration.hpp"

#include "glog/logging.h"

using namespace std;
using namespace Eigen;

namespace lidar_localization {

    SVDRegistration::SVDRegistration(const YAML::Node &node)
        : icp_ptr_(new pcl::IterativeClosestPoint<CloudData::POINT, CloudData::POINT>()) {

        float max_corr_dist = node["max_corr_dist"].as<float>();
        float trans_eps = node["trans_eps"].as<float>();
        float euc_fitness_eps = node["euc_fitness_eps"].as<float>();
        int max_iter = node["max_iter"].as<int>();

        SetRegistrationParam(max_corr_dist, trans_eps, euc_fitness_eps, max_iter);
    }

    SVDRegistration::SVDRegistration(
                    float max_corr_dist,
                    float trans_eps,
                    float euc_fitness_eps,
                    int max_iter
    ) : icp_ptr_(new pcl::IterativeClosestPoint<CloudData::POINT, CloudData::POINT>()) {

        SetRegistrationParam(max_corr_dist, trans_eps, euc_fitness_eps, max_iter);
    }

    bool SVDRegistration::SetRegistrationParam(
                    float max_corr_dist,
                    float trans_eps,
                    float euc_fitness_eps,
                    int max_iter
    ) {
        icp_ptr_->setMaxCorrespondenceDistance(max_corr_dist);
        icp_ptr_->setTransformationEpsilon(trans_eps);
        icp_ptr_->setEuclideanFitnessEpsilon(euc_fitness_eps);
        icp_ptr_->setMaximumIterations(max_iter);

        LOG(INFO) << "ICP params:" << std::endl
                  << "max_corr_dist: " << max_corr_dist << ", "
                  << "trans_eps: " << trans_eps << ", "
                  << "euc_fitness_eps: " << euc_fitness_eps << ", "
                  << "max_iter: " << max_iter
                  << std::endl << std::endl;

        return true;
    }

    bool SVDRegistration::SetInputTarget(const CloudData::CLOUD_PTR &input_target) {
        double sum_x, sum_y, sum_z;

        for(auto p : input_target->points) {
            sum_x += p.x;
            sum_y += p.y;
            sum_z += p.z;
        }

        ux << sum_x / input_target->points.size(), sum_y / input_target->points.size(), sum_z / input_target->points.size();
        return true;
    }

    bool SVDRegistration::ScanMatch(const CloudData::CLOUD_PTR &input_source,
                                    const Eigen::Matrix4f &predict_pose,
                                    CloudData::CLOUD_PTR &result_cloud_ptr,
                                    Eigen::Matrix4f &result_pose) {
        double sum_x, sum_y, sum_z;

        for(auto p : input_source->points) {
            sum_x += p.x;
            sum_y += p.y;
            sum_z += p.z;
        }

        uy << sum_x / input_source->points.size(), sum_y / input_source->points.size(), sum_z / input_source->points.size();

        // LOG(INFO) << "Here is the matrix H:" << std::endl << H << std::endl;
        // JacobiSVD<MatrixXf> svd(H, ComputeThinU | ComputeThinV);
        // LOG(INFO) << "Its singular values are:" << std::endl << svd.singularValues() << std::endl;
        // LOG(INFO) << "Its left singular vectors are the columns of the thin U matrix:" << std::endl << svd.matrixU() << std::endl;
        // LOG(INFO) << "Its right singular vectors are the columns of the thin V matrix:" << std::endl << svd.matrixV() << std::endl;
        // Eigen::Matrix3d rot = svd.matrixV() * svd.matrixU().transpose();
        // LOG(INFO) << "Now consider this rhs vector:" << std::endl << rot << std::endl;
        // Eigen::Vector3d t = ux - rot * uy;
        return true;
    }

}