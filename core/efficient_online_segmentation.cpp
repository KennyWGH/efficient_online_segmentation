/*
 * Copyright 2021 Guanhua WANG
 */

#include "efficient_online_segmentation.h"

#include <chrono>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <pcl/common/transforms.h>


EfficientOnlineSegmentation::EfficientOnlineSegmentation(const SegmentationParams& params)
{
    ResetParameters(params);
    cloud_in_base_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    custom_cloud_in_base_.reset(new pcl::PointCloud<PointXYZIRT>());
}

EfficientOnlineSegmentation::~EfficientOnlineSegmentation() {}

void EfficientOnlineSegmentation::ResetParameters(const SegmentationParams& params)
{
    params_ = params;
    sectors_.clear();
    sectors_.resize(params_.kNumSectors);
    for (int i=0; i<sectors_.size(); i++) {
        sectors_[i] = SmartSector(i/*sector id*/,
                                params_.kLidarRows,
                                0, // params_.kSensorHeight,
                                params_.kGroundSameLineTolerance,
                                params_.kGroundSlopeTolerance,
                                params_.kGroundYInterceptTolerance,
                                params_.kGroundPointLineDistThres,
                                params_.kWallSameLineTolerance,
                                params_.kWallSlopeTolerance,
                                params_.kWallLineMinBinNum,
                                params_.kWallPointLineDistThres);
    }
    sensor_pose_in_base_ = params_.kBaseToSensor;
    std::cout << "sensor_pose_in_base_: " << std::endl 
        << sensor_pose_in_base_.matrix() << std::endl 
        << " " << std::endl;
}

// implementation 1.
void EfficientOnlineSegmentation::Segment(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_data, 
                                    std::vector<int>* labels_out, bool use_intensity) 
{
    ordinary_cloud_used = true;
    custom_cloud_used = false;
    num_received_msgs++;
    if (kPrintLog) {
        std::cout << "Segmenting cloud with " << cloud_data->size() << " points... -------------- "
                  << "[msg #" << num_received_msgs << "] " << std::endl;
    }

    clock_t time_start = clock();

    // step01 register every and each point into corresponding sector!
    for (auto& sector : sectors_) {sector.Reset();}
    pcl::transformPointCloud(*cloud_data, *cloud_in_base_, sensor_pose_in_base_);
    range_image_ = cv::Mat::zeros(params_.kLidarRows, params_.kLidarCols, CV_32FC1);
    for (std::size_t i=0; i<cloud_data->size(); ++i)
    {
        // if (cloud_in_base_->points[i].z > 1.0) continue;
        if (use_intensity) {cloud_data->points[i].intensity=kOtherIntensity;}
        // if (cloud_in_base_->points[i].z > 1.0) continue;
        float ang_azimuth = std::atan2(cloud_data->points[i].y, cloud_data->points[i].x);
        while (ang_azimuth < 0) { ang_azimuth += 2*M_PI; } /* from [-pi,+pi] to [0, 2pi] */
        float xy_range = std::sqrt( cloud_data->points[i].x*cloud_data->points[i].x
                                    +cloud_data->points[i].y*cloud_data->points[i].y );
        float ang_elevation = std::atan2(cloud_data->points[i].z, xy_range); /* naturally [-pi,+pi] */
        if (ang_elevation<params_.kLidarVertAngMin || ang_elevation>params_.kLidarVertAngMax) {
            continue;
        }

        int rowIdx=0, colIdx=0;
        rowIdx = round((ang_elevation - params_.kLidarVertFovMin) * params_.kLidarVertResInv);
        colIdx = floor(ang_azimuth*params_.kLidarHorizResInv);
        if (rowIdx < 0 || rowIdx >= params_.kLidarRows 
            || colIdx < 0 || colIdx >= params_.kLidarCols) {
            continue;
        }
        int sectorIdx = floor(colIdx/params_.kColsPerSector/*5*/);
        if (sectorIdx<0 || sectorIdx>=params_.kNumSectors) {
            continue;
        }

        float* rangeImgRowPtr = range_image_.ptr<float>(rowIdx);
        rangeImgRowPtr[colIdx] = xy_range;

        // Note that here we must pass in point in base (or ground) frame!
        // Remember that the `intensity` represents x-y range!!!!
        pcl::PointXYZI point3D = cloud_in_base_->points[i];
        point3D.intensity = std::sqrt(point3D.x*point3D.x
                                     +point3D.y*point3D.y);
        sectors_[sectorIdx].AddPoint(point3D, i/*original index*/, rowIdx/*bin id*/);
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

    // step03 project labels to original `cloud_data`.
    labels_out->clear();
    labels_out->resize(cloud_data->size());
    std::fill(labels_out->begin(), labels_out->end(), 0); /* 0(unknown), 1(ground), 2(others) */
    for (const auto& index : ground_indices) {
        (*labels_out)[index] = 1;
        if (use_intensity) {cloud_data->points[index].intensity=kGroundIntensity;}
    }
    for (const auto& index : wall_indices) {
        (*labels_out)[index] = 2;
        if (use_intensity) {cloud_data->points[index].intensity=kWallIntensity;}
    }
    if (use_intensity) { /* expand intensity interval to [0, 255]. */
        cloud_data->points[0].intensity=0;
        cloud_data->points[1].intensity=255;
    } 

    clock_t time_end = clock();
    float timePhase1 = (float(time_stop01-time_start))/CLOCKS_PER_SEC;
    float timePhase2 = (float(time_stop02-time_stop01))/CLOCKS_PER_SEC;
    float timePhase3 = (float(time_end-time_stop02))/CLOCKS_PER_SEC;
    float timeTotal = (float(time_end-time_start))/CLOCKS_PER_SEC;
    accumulated_run_time += timeTotal;
    if (kPrintLog) {
        float ground_percentage = ground_indices.size() * 100.f / cloud_data->size();
        float wall_percentage = wall_indices.size() * 100.f / cloud_data->size();
        std::cout << std::fixed << std::setprecision(3) 
                  << "Ground points " << ground_indices.size() << "(" << ground_percentage 
                  << "%), Wall points " << wall_indices.size() << "(" << wall_percentage 
                  << "%). " << std::endl 
                  << "Took [" << timePhase1 << "," << timePhase2 << "," 
                  << timePhase3 << "]=" << timeTotal << "s, [avg" 
                  << accumulated_run_time/num_received_msgs << "s]." 
                  << std::endl << std::endl;
    }

}

// implementation 2.
void EfficientOnlineSegmentation::Segment(pcl::PointCloud<PointXYZIRT>::Ptr& cloud_data, 
                                    std::vector<int>* labels_out, bool use_intensity) 
{
    ordinary_cloud_used = false;
    custom_cloud_used = true;
    num_received_msgs++;
    if (kPrintLog) {
        std::cout << "Segmenting cloud with " << cloud_data->size() << " points... -------------- "
                  << "[msg #" << num_received_msgs << "] " << std::endl;
    }

    clock_t time_start = clock();

    // step01 register every and each point into corresponding sector!
    for (auto& sector : sectors_) {sector.Reset();}
    pcl::transformPointCloud(*cloud_data, *custom_cloud_in_base_, sensor_pose_in_base_);
    range_image_ = cv::Mat::zeros(params_.kLidarRows, params_.kLidarCols, CV_32FC1);
    for (std::size_t i=0; i<cloud_data->size(); ++i)
    {
        // if (custom_cloud_in_base_->points[i].z > 1.0) continue;
        if (use_intensity) {cloud_data->points[i].intensity=kOtherIntensity;}
        // if (custom_cloud_in_base_->points[i].z > 1.0) continue;
        float ang_azimuth = std::atan2(cloud_data->points[i].y, cloud_data->points[i].x);
        while (ang_azimuth < 0) { ang_azimuth += 2*M_PI; } /* from [-pi,+pi] to [0, 2pi] */
        float xy_range = std::sqrt( cloud_data->points[i].x*cloud_data->points[i].x
                                    +cloud_data->points[i].y*cloud_data->points[i].y );

        int rowIdx=0, colIdx=0;
        rowIdx = cloud_data->points[i].ring;
        colIdx = floor(ang_azimuth*params_.kLidarHorizResInv);
        if (rowIdx < 0 || rowIdx >= params_.kLidarRows 
            || colIdx < 0 || colIdx >= params_.kLidarCols) {
            continue;
        }
        int sectorIdx = floor(colIdx/params_.kColsPerSector/*5*/);
        if (sectorIdx<0 || sectorIdx>=params_.kNumSectors) {
            continue;
        }

        float* rangeImgRowPtr = range_image_.ptr<float>(rowIdx);
        rangeImgRowPtr[colIdx] = xy_range;

        // Note that here we must pass in point in base (or ground) frame!
        // Remember that the `intensity` represents x-y range!!!!
        pcl::PointXYZI point3D;
        point3D.x = custom_cloud_in_base_->points[i].x;
        point3D.y = custom_cloud_in_base_->points[i].y;
        point3D.z = custom_cloud_in_base_->points[i].z;
        point3D.intensity = std::sqrt(point3D.x*point3D.x
                                     +point3D.y*point3D.y);
        sectors_[sectorIdx].AddPoint(point3D, i/*original index*/, rowIdx/*bin id*/);
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

    // step03 project labels to original `cloud_data`.
    labels_out->clear();
    labels_out->resize(cloud_data->size());
    std::fill(labels_out->begin(), labels_out->end(), 0); /* 0(unknown), 1(ground), 2(others) */
    for (const auto& index : ground_indices) {
        (*labels_out)[index] = 1;
        if (use_intensity) {cloud_data->points[index].intensity=kGroundIntensity;}
    }
    for (const auto& index : wall_indices) {
        (*labels_out)[index] = 2;
        if (use_intensity) {cloud_data->points[index].intensity=kWallIntensity;}
    }
    if (use_intensity) { /* expand intensity interval to [0, 255]. */
        cloud_data->points[0].intensity=0;
        cloud_data->points[1].intensity=255;
    } 

    clock_t time_end = clock();
    float timePhase1 = (float(time_stop01-time_start))/CLOCKS_PER_SEC;
    float timePhase2 = (float(time_stop02-time_stop01))/CLOCKS_PER_SEC;
    float timePhase3 = (float(time_end-time_stop02))/CLOCKS_PER_SEC;
    float timeTotal = (float(time_end-time_start))/CLOCKS_PER_SEC;
    accumulated_run_time += timeTotal;
    if (kPrintLog) {
        float ground_percentage = ground_indices.size() * 100.f / cloud_data->size();
        float wall_percentage = wall_indices.size() * 100.f / cloud_data->size();
        std::cout << std::fixed << std::setprecision(3) 
                  << "Ground points " << ground_indices.size() << "(" << ground_percentage 
                  << "%), Wall points " << wall_indices.size() << "(" << wall_percentage 
                  << "%). " << std::endl 
                  << "Took [" << timePhase1 << "," << timePhase2 << "," 
                  << timePhase3 << "]=" << timeTotal << "s, [avg" 
                  << accumulated_run_time/num_received_msgs << "s]." 
                  << std::endl << std::endl;
    }

}

pcl::PointCloud<pcl::PointXYZI>::Ptr
EfficientOnlineSegmentation::GetTransformedCloud()
{
    if (custom_cloud_used) {
        std::cout << "ERROR! wrong cloud type used." << std::endl;
    }
    return cloud_in_base_;
}

pcl::PointCloud<PointXYZIRT>::Ptr
EfficientOnlineSegmentation::GetTransformedCustomCloud()
{
    if (ordinary_cloud_used) {
        std::cout << "ERROR! wrong cloud type used." << std::endl;
    }
    return custom_cloud_in_base_;
}

cv::Mat EfficientOnlineSegmentation::GetRangeImage()
{
    return range_image_;
}

std::vector<BasicLine>& EfficientOnlineSegmentation::GetExtractedLines()
{
    return extracted_lines_;
}


