/*
 * Copyright 2021 Guanhua WANG
 */

#ifndef EFFICIENT_ONLINE_SEGMENTATION_SECTOR_H_
#define EFFICIENT_ONLINE_SEGMENTATION_SECTOR_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <set>

// ******************************************************* //

struct IndexedPoint {
    pcl::PointXYZI point;
    int bin_index = -1;
    int original_index = -1;

    IndexedPoint();
    IndexedPoint(const pcl::PointXYZI& pointIn, const int& binIdxIn, const int& origIdxIn);

    bool operator==(const IndexedPoint& other) const;
    bool operator!=(const IndexedPoint& other) const;
    bool operator<(const IndexedPoint& other) const;
};

enum LineLabel { NAL /*not a label*/, GROUND, WALL };

struct BasicLine {
    LineLabel label;
    pcl::PointXYZI start_point;
    pcl::PointXYZI end_point;

    BasicLine();
    BasicLine(const LineLabel& labelIn,
                const pcl::PointXYZI startPoint, 
                const pcl::PointXYZI endPoint );
};

struct SmartLine {
    int start_bin = -1;
    int end_bin = -1;
    std::vector<int> bins_;
    std::vector<float> slopes_;

    // used for calculating slope in `local` frame.
    float locOriginX = 0;
    float locOriginY = 0;
    float locCosTheta = 1;
    float locSinTheta = 0;
    float locTanTheta = 0; // first two points slope (or tan).
    float last_point_x = 0;
    float last_point_y = 0;

    // only for calculating line parameters.
    bool isCalculated = false;
    pcl::PointXYZI start_point;
    pcl::PointXYZI end_point;
    float length_ = 0;
    float k_ = 0;
    float b_ = 0;

    // params. 
    float kWallSlopeThres; /* if abs(TanTheta)>3.7321, i.e., 105deg>Theta>75deg, a possible wall line.*/
    float kMaxDistBtwnRings;  /* 6.0; interval btwn rings should be less than X meters.*/

    SmartLine(const float& wall_slope, const float& max_ring_dist=6);
    void Reset();
    bool TryAddNewBin(const int& newBin, const pcl::PointXYZI& binPoint, 
          const float& groundSameLineTolerance, const float& wallSameLineTolerance);
    BasicLine GetLine(const std::vector<pcl::PointXYZI>& binVec, 
                        const LineLabel& labelID);

};

// ******************************************************* //

/**
 * @brief The SmartSector class is an advanced implementation for line-extraction,
 * a line can be a ground line, a wall line, or a line attached on whatever object.
 * Note that inside the SmartSector class, PointXYZI::intensity is used for xy-range!
 */
class SmartSector {
  public:

    SmartSector();
    SmartSector(const int& this_sector_id,
            const unsigned int& n_bins,
            const float& sensor_height,
            const float& ground_same_line_tolerance = 0.035,
            const float& ground_slope_tolerance = 0.182,
            const float& ground_intercept_tolerance = 0.2,
            const float& ground_pointline_dist = 0.1,
            const float& wall_same_line_tolerance = 0.1051,
            const float& wall_slope_tolerance = 3.73,
            const float& wall_min_num_bins = 3,
            const float& wall_pointline_dist = 0.1);

    // Essential pipeline.
    void Reset();
    void AddPoint(const pcl::PointXYZI& pointIn, const int& pointIdx, const int& binIdx);
    void RunLineExtraction();

    // Get results.
    std::vector<int>& GetGroundPointIndices();
    std::vector<int>& GetWallPointIndices();
    std::vector<BasicLine>& GetExtractedLines();

    // Query from outside. (1-ground, 2-wall, 0-neither)
    int IdentifyExternalPoint(const pcl::PointXYZI& pointIn, const int& binIdx);

  public:

   LineLabel JudgeLineLAbel(SmartLine& smartLine, const std::vector<pcl::PointXYZI>& binVec);

    // General params.
    int kSectorId = -1;
    int kNumBins;           // = 16 or 32;
    float kSensorHeight;    // = 0.0;

    // Identify ground.
    float kGroundSameLineTolerance;     // = 0.035;   // 2 degree(0.035, around 0.1m/3m) 
    float kGroundSlopeTolerance;        // = 0.1763;  // 10 degrees(0.1763), 2 degree(0.035), 1 degree(0.0175)
    float kGroundYInterceptTolerance;   // = 0.2;
    float kGroundPointLineDistThres;    // = 0.1;

    // Identify wall.
    float kWallSameLineTolerance;       // = 0.1051; // 6 degree(0.1051), 2 degree(0.035)
    float kWallSlopeTolerance;          // = 3.73;  // 85 degrees(11.43), 80 degrees(5.67), 75 degrees(3.73)
    float kWallLineMinBinNum;           // = 3;       // consider using this to replace MinLength.
    float kWallPointLineDistThres;      // = 0.1;

    pcl::PointXYZI nanPoint;

    // all the following variables/containers need to be reset before next round.
    std::set<IndexedPoint> src_points_;
    std::vector<pcl::PointXYZI> sector_; 
    std::vector<int> sectorCounts_; 

    std::vector<BasicLine> extracted_lines_;

    std::vector<int> ground_bins_;
    std::vector<int> ground_points_indices_;
    std::vector<int> wall_bins_;
    std::vector<int> wall_points_indices_;

};

#endif // EFFICIENT_ONLINE_SEGMENTATION_SECTOR_H_
