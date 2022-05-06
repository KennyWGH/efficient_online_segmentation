/*
 * Copyright 2021 WANG Guanhua
 */

#include "efficient_online_segmentation.h"

#include <chrono>
#include <cmath>
#include <ctime>
#include <iomanip>


AdvancedSegmentation::AdvancedSegmentation(const SegmentationParams& paramsIn)
{
    UpdateParameters(paramsIn);
}

void AdvancedSegmentation::UpdateParameters(const SegmentationParams& paramsIn)
{
    params_ = paramsIn;
    sectors_.clear();
    sectors_.resize(params_.kNumSectors);
    for (int i=0; i<sectors_.size(); i++) {
        sectors_[i] = SmartSector(params_.kLidarRows,
                                params_.kSensorHeight,
                                i,
                                params_.kGroundSameLineTolerance,
                                params_.kGroundSlopeTolerance,
                                params_.kGroundYInterceptTolerance,
                                params_.kGroundPointLineDistThres,
                                params_.kWallSameLineTolerance,
                                params_.kWallSlopeTolerance,
                                params_.kWallLineMinBinNum,
                                params_.kWallPointLineDistThres);
    }
}

void AdvancedSegmentation::Segment(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudIn, 
                                    std::vector<int>* labelsOut, bool useIntensity) 
{
    num_received_msgs++;
    std::cout << "Segmenting cloud with " << cloudIn->size() << " points...  [msg #" << num_received_msgs << "] \n";
    clock_t time_start = clock();

    // step01 add each and every point into corresponding sector!
    for (auto& sector : sectors_) {sector.Reset();}
    range_image_ = cv::Mat::zeros(params_.kLidarRows, params_.kLidarCols, CV_32FC1);
    for (int i=0; i<cloudIn->size(); ++i)
    {
        if (useIntensity) {cloudIn->points[i].intensity=10 /*light red*/;}
        float ang_azimuth = std::atan2(cloudIn->points[i].y, cloudIn->points[i].x);
        if (ang_azimuth < 0) { ang_azimuth += 2*M_PI; } /* from [-pi,+pi] to [0, 2pi] */
        float xy_range = std::sqrt( cloudIn->points[i].x*cloudIn->points[i].x
                                    +cloudIn->points[i].y*cloudIn->points[i].y );
        float ang_elevation = std::atan2(cloudIn->points[i].z, xy_range); /* naturally [-pi,+pi] */
        if (ang_elevation<params_.kLidarAngVertMin || ang_elevation>params_.kLidarAngVertMax) {
            continue;
        }

        int rowIdx=0, colIdx=0;
        rowIdx = round((ang_elevation+params_.kLidarAngOffsetVert)*params_.kLidarAngResVInverse);
        colIdx = floor(ang_azimuth*params_.kLidarAngResHInverse);
        if (rowIdx < 0 || rowIdx >= params_.kLidarRows || colIdx < 0 || colIdx >= params_.kLidarCols) {
            continue;
        }
        int sectorIdx = floor(colIdx/params_.kColsPerSector/*5*/);
        if (sectorIdx<0 || sectorIdx>=params_.kNumSectors) {
            continue;
        }

        float* rangeImgRowPtr = range_image_.ptr<float>(rowIdx);
        rangeImgRowPtr[colIdx] = xy_range;

        pcl::PointXYZI point3D = cloudIn->points[i];
        point3D.intensity = xy_range;
        sectors_[sectorIdx].AddPoint(point3D, i, rowIdx);
    }
    clock_t time_stop01 = clock();

    // step02 extract line within each sector.
    extracted_lines_.clear();
    std::vector<int> ground_indices;
    std::vector<int> wall_indices;
    for (auto& sector_ : sectors_) {
        sector_.RunLineExtraction();
        auto& result1 = sector_.GetGroundPointIndices(); 
        ground_indices.insert(ground_indices.end(), result1.begin(), result1.end());
        auto& result2 = sector_.GetWallPointIndices(); 
        wall_indices.insert(wall_indices.end(), result2.begin(), result2.end());
        auto& geometry_lines = sector_.GetExtractedLines();
        extracted_lines_.insert(extracted_lines_.end(), geometry_lines.begin(), geometry_lines.end());
    }
    clock_t time_stop02 = clock();

    // step03 project labels to original `cloudIn`.
    labelsOut->clear();
    labelsOut->resize(cloudIn->size());
    std::fill(labelsOut->begin(), labelsOut->end(), 0); /* 0(unknown), 1(ground), 2(others) */
    for (const auto& index : ground_indices) {
        (*labelsOut)[index] = 1;
        if (useIntensity) {cloudIn->points[index].intensity=255;}
    }
    for (const auto& index : wall_indices) {
        (*labelsOut)[index] = 2;
        if (useIntensity) {cloudIn->points[index].intensity=100;}
    }
    if (useIntensity) { /* make sure 0&255 both exist. */
        cloudIn->points[0].intensity=0;
        cloudIn->points[1].intensity=255;
    } 

    clock_t time_end = clock();
    float timePhase1 = (float(time_stop01-time_start))/CLOCKS_PER_SEC;
    float timePhase2 = (float(time_stop02-time_stop01))/CLOCKS_PER_SEC;
    float timePhase3 = (float(time_end-time_stop02))/CLOCKS_PER_SEC;
    float timeTotal = (float(time_end-time_start))/CLOCKS_PER_SEC;
    life_ling_run_time += timeTotal;
    std::cout << std::fixed << std::setprecision(3) << "Done! Took [" << timePhase1 << "," << timePhase2 
    << "," << timePhase3 << "]=" << timeTotal << "s, avg[" << life_ling_run_time/num_received_msgs << "]s\n";
}

cv::Mat AdvancedSegmentation::GetRangeImage()
{
    return range_image_;
}

std::vector<BasicLine>& AdvancedSegmentation::GetExtractedLines()
{
    return extracted_lines_;
}


