/*
 * Copyright 2021 WANG Guanhua
 */

#ifndef ADVANCED_SEGMENTATION_SGMTT_UTILITY_H_
#define ADVANCED_SEGMENTATION_SGMTT_UTILITY_H_

#include<cmath>

struct AdvancedSegmentationParams {

    // LiDAR parameters. (VLP16)
    int kLidarRows = 16;
    int kLidarCols = 1800;
    float kLidarAngResHoriz = 0.2/180*M_PI;
    float kLidarAngResVert = 2./180*M_PI;
    float kLidarAngResHInverse = 1/kLidarAngResHoriz; //900./M_PI;
    float kLidarAngResVInverse = 1/kLidarAngResVert;  //90./M_PI;
    float kLidarAngOffsetVert = 15./180*M_PI;
    float kLidarAngVertMin = -15.33333*M_PI/180.; //  +-0.3333 error.
    float kLidarAngVertMax = 15.33333*M_PI/180.;

    int kNumSectors = 360;                  // Number of angular segments.
    int kColsPerSector = kLidarCols/kNumSectors;
    float kSensorHeight = 1.3;

    // Identify ground.
    float kGroundSameLineTolerance = 0.035;       // equal to 2 degree, around 0.1m/3m
    float kGroundSlopeTolerance = 0.182;    // 10 degrees
    float kGroundYInterceptTolerance = 0.2; // the intercept(b) of straight line.
    float kGroundPointLineDistThres = 0.1;  // Maximum distance to a ground line to be classified as ground.

    // Identify wall.
    float kWallSameLineTolerance = 0.1051; // 6 degree(0.1051), 2 degree(0.035)
    float kWallSlopeTolerance = 3.73;  // 85 degrees(11.43), 80 degrees(5.67), 75 degrees(3.73)
    float kWallLineMinBinNum = 3;       // consider using this to replace MinLength.
    float kWallPointLineDistThres = 0.1;

    AdvancedSegmentationParams();
};





#endif // ADVANCED_SEGMENTATION_SGMTT_UTILITY_H_
