/*
 * Copyright 2021 Guanhua WANG
 */

#include "segmentation_utility.h"
#include <iostream>


SegmentationParams::SegmentationParams() {}

SegmentationParams& SegmentationParams::operator=(const SegmentationParams& other)
{
    kLidarRows = other.kLidarRows;
    kLidarCols = other.kLidarCols;
    kLidarHorizRes = other.kLidarHorizRes;
    kLidarVertRes = other.kLidarVertRes;
    kLidarVertFovMax = other.kLidarVertFovMax;
    kLidarVertFovMin = other.kLidarVertFovMin;
    kLidarProjectionError = other.kLidarProjectionError;

    kLidarHorizResInv = other.kLidarHorizResInv;
    kLidarVertResInv = other.kLidarVertResInv;
    kLidarVertAngMin = other.kLidarVertAngMin;
    kLidarVertAngMax = other.kLidarVertAngMax;

    kNumSectors = other.kNumSectors;
    kColsPerSector = other.kColsPerSector;
    kSensorHeight = other.kSensorHeight;
    kSensorRoll = other.kSensorRoll;
    kSensorPitch = other.kSensorPitch;
    kBaseToSensor = other.kBaseToSensor;

    kGroundSameLineTolerance = other.kGroundSameLineTolerance;
    kGroundSlopeTolerance = other.kGroundSlopeTolerance;
    kGroundYInterceptTolerance = other.kGroundYInterceptTolerance;
    kGroundPointLineDistThres = other.kGroundPointLineDistThres;

    kWallSameLineTolerance = other.kWallSameLineTolerance;
    kWallSlopeTolerance = other.kWallSlopeTolerance;
    kWallLineMinBinNum = other.kWallLineMinBinNum;
    kWallPointLineDistThres = other.kWallPointLineDistThres;

    // PrintAllParams("Copied parameters");
}

void SegmentationParams::UpdateInternalParams()
{
    kLidarHorizResInv = 1/kLidarHorizRes; 
    kLidarVertResInv = 1/kLidarVertRes; 
    kLidarVertAngMin = kLidarVertFovMin - kLidarProjectionError;
    kLidarVertAngMax = kLidarVertFovMax + kLidarProjectionError; 

    kColsPerSector = kLidarCols/kNumSectors;

    Eigen::AngleAxisf rot1 (kSensorRoll, Eigen::Vector3f(1,0,0).normalized());
    Eigen::AngleAxisf rot2 (kSensorPitch, Eigen::Vector3f(0,1,0).normalized());
    Eigen::Translation3f translation (0, 0, kSensorHeight);
    Eigen::Matrix4f transform = (translation * rot1 * rot2).matrix();
    // Eigen::Matrix4f transform = (rot1 * rot2 * translation).matrix();
    kBaseToSensor = transform;

    PrintAllParams("Updated parameters");
}

void SegmentationParams::PrintAllParams(const std::string& title)
{
    std::cout << "########## " << title << ": " << std::endl
            << "kLidarRows: " << kLidarRows << std::endl
            << "kLidarCols: " << kLidarCols << std::endl
            << "kLidarHorizRes: " << kLidarHorizRes << std::endl
            << "kLidarVertRes: " << kLidarVertRes << std::endl
            << "kLidarVertFovMax: " << kLidarVertFovMax << std::endl
            << "kLidarVertFovMin: " << kLidarVertFovMin << std::endl
            << "kLidarProjectionError: " << kLidarProjectionError << std::endl
            << " " << std::endl
            << "kLidarHorizResInv: " << kLidarHorizResInv << std::endl
            << "kLidarVertResInv: " << kLidarVertResInv << std::endl
            << "kLidarVertAngMin: " << kLidarVertAngMin << std::endl
            << "kLidarVertAngMax: " << kLidarVertAngMax << std::endl
            << " " << std::endl
            << "kNumSectors: " << kNumSectors << std::endl
            << "kColsPerSector: " << kColsPerSector << std::endl
            << "kSensorHeight: " << kSensorHeight << std::endl
            << "kSensorRoll: " << kSensorRoll << std::endl
            << "kSensorPitch: " << kSensorPitch << std::endl
            << "kBaseToSensor: " << std::endl 
            << kBaseToSensor.matrix() << std::endl 
            << " " << std::endl
            << "kGroundSameLineTolerance: " << kGroundSameLineTolerance << std::endl 
            << "kGroundSlopeTolerance: " << kGroundSlopeTolerance << std::endl 
            << "kGroundYInterceptTolerance: " << kGroundYInterceptTolerance << std::endl 
            << "kGroundPointLineDistThres: " << kGroundPointLineDistThres << std::endl 
            << " " << std::endl
            << "kWallSameLineTolerance: " << kWallSameLineTolerance << std::endl 
            << "kWallSlopeTolerance: " << kWallSlopeTolerance << std::endl 
            << "kWallLineMinBinNum: " << kWallLineMinBinNum << std::endl 
            << "kWallPointLineDistThres: " << kWallPointLineDistThres << std::endl 
            << std::endl;
}


