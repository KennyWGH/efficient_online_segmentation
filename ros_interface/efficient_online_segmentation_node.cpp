/*
 * Copyright 2021 WANG Guanhua
 */

#include <string>
#include <iostream>

#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>

#include "core/efficient_online_segmentation.h"

// ###########################################################################

ros::Subscriber subPointCloud;
ros::Publisher pubSegmtedCloud;
ros::Publisher pubRangeImage;
ros::Publisher pubExtractedLines;

// System variables.
AdvancedSegmentation advanced_sgmtt_;
pcl::PointCloud<pcl::PointXYZI>::Ptr original_cloud_;
std::vector<int> labels_;

// Callback.
void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    clock_t time_start = clock();
    pcl::fromROSMsg(*msg, *original_cloud_); 

    advanced_sgmtt_.Segment(original_cloud_, &labels_, true);

    // Visualize results.
    if (pubSegmtedCloud.getNumSubscribers() != 0){
        sensor_msgs::PointCloud2 cloudMsgTemp;
        pcl::toROSMsg(*original_cloud_, cloudMsgTemp);
        cloudMsgTemp.header = msg->header;
        pubSegmtedCloud.publish(cloudMsgTemp);
    }

    if (pubRangeImage.getNumSubscribers() != 0){
        cv::Mat imgTmp1, imgTmp2;
        imgTmp1 = advanced_sgmtt_.GetRangeImage();
        cv::normalize(imgTmp1,imgTmp2, 255, 0, cv::NORM_MINMAX);
        imgTmp2.convertTo(imgTmp1, CV_8UC1); // from CV_32FC1 to CV_8UC1
        cv::flip(imgTmp1, imgTmp2, 0);
        cv::applyColorMap(imgTmp2, imgTmp1, cv::COLORMAP_RAINBOW);
        cv::resize(imgTmp1, imgTmp2, cv::Size(600,16));
        sensor_msgs::ImagePtr imgMsgTemp = 
            cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgTmp2).toImageMsg();
        pubRangeImage.publish(*imgMsgTemp);
    }

    if (pubExtractedLines.getNumSubscribers() != 0) {
        visualization_msgs::MarkerArray lines_array;
        const auto& geometry_lines = advanced_sgmtt_.GetExtractedLines();
        if (!geometry_lines.empty()) {
            float kEdgeScale = 0.05;
            visualization_msgs::Marker edge;
            edge.header = msg->header;
            edge.action = visualization_msgs::Marker::ADD;
            edge.ns = "AS_lines";
            edge.id = 0;
            edge.type = visualization_msgs::Marker::LINE_STRIP;
            edge.scale.x = kEdgeScale;
            edge.scale.y = kEdgeScale;
            edge.scale.z = kEdgeScale;
            edge.color.r = 0.0;
            edge.color.g = 1.0;
            edge.color.b = 1.0;
            edge.color.a = 1.0;
            geometry_msgs::Point pStart;
            geometry_msgs::Point pEnd;
            int id=0;
            for(const auto& curr_line : geometry_lines)
            {
                //遍历每个边 将位置赋值
                edge.points.clear();
                edge.id = id;
                pStart.x = curr_line.start_point.x;
                pStart.y = curr_line.start_point.y;
                pStart.z = curr_line.start_point.z;
                edge.points.push_back(pStart);
                pEnd.x = curr_line.end_point.x;
                pEnd.y = curr_line.end_point.y;
                pEnd.z = curr_line.end_point.z;
                edge.points.push_back(pEnd);
                if (curr_line.label==LineLabel::GROUND) {
                    edge.scale.x = kEdgeScale;
                    edge.scale.y = kEdgeScale;
                    edge.scale.z = kEdgeScale;
                    edge.color.r = 1.0;
                    edge.color.g = 0.8;
                    edge.color.b = 0.0;
                }
                else if (curr_line.label==LineLabel::WALL) {
                    edge.scale.x = 2*kEdgeScale;
                    edge.scale.y = 2*kEdgeScale;
                    edge.scale.z = 2*kEdgeScale;
                    edge.color.r = 0.0;
                    edge.color.g = 1.0;
                    edge.color.b = 0.0;
                }
                lines_array.markers.push_back(visualization_msgs::Marker(edge));
                id++;
            }
        }
        pubExtractedLines.publish(lines_array);
    }

    return;
}

int main(int argc, char** argv) {
    std::string node_name = "advanced_segmentation_node";
    ::ros::init(argc, argv, node_name);
    ::ros::start();

    ::ros::NodeHandle node_handle("~");

    // initialize system.
    {
        std::string sub_cloud_topic, pub_cloud_topic, pub_rangeimage_topic, pub_extractedlines_topic;
        node_handle.param<std::string>("sub_cloud_topic", sub_cloud_topic, "/velodyne_points");
        node_handle.param<std::string>("pub_cloud_topic", pub_cloud_topic, "/AS_segmted_cloud"); // AS: Advanced Segmentation.
        node_handle.param<std::string>("pub_rangeimage_topic", pub_rangeimage_topic, "/AS_range_image");
        node_handle.param<std::string>("pub_extractedlines_topic", pub_extractedlines_topic, "/AS_extracted_lines");

        subPointCloud = node_handle.subscribe<sensor_msgs::PointCloud2>
            (sub_cloud_topic, 1, &pointCloudCallback); // velodyne_points points_raw
        pubSegmtedCloud = node_handle.advertise<sensor_msgs::PointCloud2>
            (pub_cloud_topic, 1);
        pubRangeImage = node_handle.advertise<sensor_msgs::Image>
            (pub_rangeimage_topic, 1);
        pubExtractedLines = node_handle.advertise<visualization_msgs::MarkerArray>
            (pub_extractedlines_topic,1);

        SegmentationParams params;
        node_handle.param("kNumSectors", params.kNumSectors, params.kNumSectors);
        node_handle.param("kColsPerSector", params.kColsPerSector, params.kColsPerSector);
        node_handle.param("kSensorHeight", params.kSensorHeight, params.kSensorHeight);
        node_handle.param("kGroundSameLineTolerance",   params.kGroundSameLineTolerance, 
                                                        params.kGroundSameLineTolerance);
        node_handle.param("kGroundSlopeTolerance",  params.kGroundSlopeTolerance, 
                                                    params.kGroundSlopeTolerance);
        node_handle.param("kGroundYInterceptTolerance", params.kGroundYInterceptTolerance, 
                                                        params.kGroundYInterceptTolerance);
        node_handle.param("kGroundPointLineDistThres",  params.kGroundPointLineDistThres, 
                                                        params.kGroundPointLineDistThres);
        node_handle.param("kWallSameLineTolerance", params.kWallSameLineTolerance, 
                                                    params.kWallSameLineTolerance);
        node_handle.param("kWallSlopeTolerance",    params.kWallSlopeTolerance, 
                                                    params.kWallSlopeTolerance);
        node_handle.param("kWallLineMinBinNum", params.kWallLineMinBinNum, 
                                                params.kWallLineMinBinNum);
        node_handle.param("kWallPointLineDistThres",    params.kWallPointLineDistThres, 
                                                        params.kWallPointLineDistThres);
        advanced_sgmtt_.UpdateParameters(params);

        original_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    }

    ::ros::spin();
    ::ros::shutdown();
}
