/*
 * Copyright 2021 Guanhua WANG
 */

#ifndef EFFICIENT_ONLINE_SEGMENTATION_SGMTT_UTILITY_H_
#define EFFICIENT_ONLINE_SEGMENTATION_SGMTT_UTILITY_H_

#include <cmath>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// ********************************************************* //

struct PointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)

// ********************************************************* //

struct SegmentationParams {

    // LiDAR parameters. (velodyne VLP-16 / velodyne HDL-32)
    int kLidarRows = 16;
    int kLidarCols = 1800;
    float kLidarHorizRes = 0.2/180.*M_PI;       // radius.
    float kLidarVertRes =  2.0/180.*M_PI;       // radius.
    float kLidarVertFovMax =  15.0/180.*M_PI;   // radius.
    float kLidarVertFovMin = -15.0/180.*M_PI;   // radius.
    float kLidarProjectionError = 0.5/180.*M_PI; // Maximum permissible error of row projection.

    // LiDAR related deduced parameters, which should 
    // help algorithm calculation.
    float kLidarHorizResInv = 1/kLidarHorizRes;                         // require re-calculation ***.
    float kLidarVertResInv = 1/kLidarVertRes;                           // require re-calculation ***.
    float kLidarVertAngMin = kLidarVertFovMin - kLidarProjectionError;  // require re-calculation ***.
    float kLidarVertAngMax = kLidarVertFovMax + kLidarProjectionError;  // require re-calculation ***.

    // Basic segmentation parameters.
    int kNumSectors = 360;
    int kColsPerSector = kLidarCols/kNumSectors;                        // require re-calculation ***.
    float kSensorHeight = 1.0;
    float kSensorRoll = 0.0;                                            // radius.
    float kSensorPitch = 0.0;                                           // radius.
    Eigen::Affine3f kBaseToSensor;                                     // require re-calculation ***.

    // Identify ground.
    float kGroundSameLineTolerance = 0.035; // radius. // 2 degree(tan[2]=0.035, around 0.1m/3m)
    float kGroundSlopeTolerance = 0.176;    // radius. // 10 degrees(tan[10]=0.176)
    float kGroundYInterceptTolerance = 0.2; // the intercept(b) of straight line.
    float kGroundPointLineDistThres = 0.1;  // Maximum point-line distance for a point to be labelled as ground.

    // Identify wall.
    float kWallSameLineTolerance = 0.0524;  // radius. // 6 degree(tan[6]=0.1051), 3 degree(0.0524), 2 degree(0.035)
    float kWallSlopeTolerance = 3.73;       // radius. // 85 degrees(tan[85]=11.43), 80 degrees(5.67), 75 degrees(3.73)
    int kWallLineMinBinNum = 3;             // a wall line should cover at least 3 rings.
    float kWallPointLineDistThres = 0.1;    // Maximum point-line distance for a point to be labelled as wall.

    SegmentationParams();
    SegmentationParams& operator=(const SegmentationParams& other);
    void UpdateInternalParams();
    void PrintAllParams(const std::string& title);
};





#endif // EFFICIENT_ONLINE_SEGMENTATION_SGMTT_UTILITY_H_
