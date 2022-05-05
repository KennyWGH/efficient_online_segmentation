/*
 * Copyright 2021 WANG Guanhua
 */

#ifndef ADVANCED_SEGMENTATION_ADVANCED_SEGMENTATION_H_
#define ADVANCED_SEGMENTATION_ADVANCED_SEGMENTATION_H_

#include <vector>
#include <array>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/core/core.hpp> // only for visualization.

#include "sgmtt_utility.h"
#include "sector.h"

class AdvancedSegmentation {
  public:
    AdvancedSegmentation(const AdvancedSegmentationParams& paramsIn = AdvancedSegmentationParams());

    AdvancedSegmentation(const AdvancedSegmentation&) = delete;
    AdvancedSegmentation& operator=(const AdvancedSegmentation&) = delete;

    void UpdateParameters(const AdvancedSegmentationParams& paramsIn);

    void Segment(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudIn, std::vector<int>* labelsOut, bool useIntensity=false);

    cv::Mat GetRangeImage();

    std::vector<BasicLine>& GetExtractedLines();

  private:
    AdvancedSegmentationParams params_;
    std::vector<SmartSector> sectors_;

    std::uint64_t num_received_msgs = 0;
    double life_ling_run_time = 0;
    cv::Mat range_image_;
    std::vector<BasicLine> extracted_lines_;

    

};

#endif // ADVANCED_SEGMENTATION_ADVANCED_SEGMENTATION_H_
