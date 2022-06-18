/*
 * Copyright 2021 WANG Guanhua
 */

#include "adaptive_ground_plane_estimator.h"

AdaptiveGroundPlaneEstimator::AdaptiveGroundPlaneEstimator() {}

AdaptiveGroundPlaneEstimator::~AdaptiveGroundPlaneEstimator() {}

void AdaptiveGroundPlaneEstimator::AddPointCloud(
    pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudIn) {}

Eigen::Vector3f EstimateGroundPlaneParams() 
{return Eigen::Vector3f::Zero();}