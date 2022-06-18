/*
 * Copyright 2021 Guanhua WANG
 */

#ifndef EFFICIENT_ONLINE_SEGMENTATION_H_
#define EFFICIENT_ONLINE_SEGMENTATION_H_

#include <vector>
#include <array>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/core/core.hpp> // only for visualization.

#include "segmentation_utility.h"
#include "sector.h"

class EfficientOnlineSegmentation {
  public:
    EfficientOnlineSegmentation(const SegmentationParams& params = SegmentationParams());
    ~EfficientOnlineSegmentation();

    EfficientOnlineSegmentation(const EfficientOnlineSegmentation&) = delete;
    EfficientOnlineSegmentation& operator=(const EfficientOnlineSegmentation&) = delete;

    void ResetParameters(const SegmentationParams& params);

    void Segment(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_data, 
                std::vector<int>* labels_out, 
                bool use_intensity=false);

    void Segment(pcl::PointCloud<PointXYZIRT>::Ptr& cloud_data, 
                std::vector<int>* labels_out, 
                bool use_intensity=false);

    pcl::PointCloud<pcl::PointXYZI>::Ptr
    GetTransformedCloud();
    pcl::PointCloud<PointXYZIRT>::Ptr
    GetTransformedCustomCloud();

    cv::Mat GetRangeImage();
    std::vector<BasicLine>& GetExtractedLines();

  private:
    SegmentationParams params_;
    std::vector<SmartSector> sectors_;
    Eigen::Affine3f sensor_pose_in_base_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in_base_;
    pcl::PointCloud<PointXYZIRT>::Ptr custom_cloud_in_base_;
    bool ordinary_cloud_used = false;
    bool custom_cloud_used = false;
    cv::Mat range_image_;
    std::vector<BasicLine> extracted_lines_;

    std::uint64_t num_received_msgs = 0;
    double accumulated_run_time = 0;
    float kGroundIntensity = 100;
    float kWallIntensity = 10;
    float kOtherIntensity = 180;
    bool kPrintLog = true;

};

#endif // EFFICIENT_ONLINE_SEGMENTATION_H_
