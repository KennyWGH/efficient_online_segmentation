/*
 * Copyright 2021 WANG Guanhua
 */

#ifndef ADAPTIVE_GROUND_PLANE_ESTIMATOR_H_
#define ADAPTIVE_GROUND_PLANE_ESTIMATOR_H_

#include <cmath>

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "segmentation_utility.h"

/**
 * @brief <Future TODO> This class is designed to estimate ground plane 
 * parameters with no prior information about how lidar is mounted at all.
 * This class would be helpfull if you find no way to know the transform 
 * between lidar and robot/car.
 */
class AdaptiveGroundPlaneEstimator {
  public:
    AdaptiveGroundPlaneEstimator();
    ~AdaptiveGroundPlaneEstimator();

    AdaptiveGroundPlaneEstimator(const AdaptiveGroundPlaneEstimator&) = delete;
    AdaptiveGroundPlaneEstimator& operator= (const AdaptiveGroundPlaneEstimator&) = delete; 

    void AddPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudIn);

    // The return type contains roll/pitch/height respectively.
    Eigen::Vector3f EstimateGroundPlaneParams();

  private:
    //
};




#endif // ADAPTIVE_GROUND_PLANE_ESTIMATOR_H_