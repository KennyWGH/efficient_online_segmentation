/*
 * Copyright 2021 WANG Guanhua
 */

#include "sector.h"
#include <cmath>
#include <climits>

// ******************************************************* //

IndexedPoint::IndexedPoint(){}

IndexedPoint::IndexedPoint(const pcl::PointXYZI& pointIn, const int& binIdxIn, const int& origIdxIn)
    :point(pointIn), bin_index(binIdxIn), original_index(origIdxIn) {}

bool IndexedPoint::operator==(const IndexedPoint& other) const{
    return std::forward_as_tuple(bin_index, original_index) ==
        std::forward_as_tuple(other.bin_index, other.original_index);
}

bool IndexedPoint::operator!=(const IndexedPoint& other) const{
    return std::forward_as_tuple(bin_index, original_index) !=
        std::forward_as_tuple(other.bin_index, other.original_index);
}

bool IndexedPoint::operator<(const IndexedPoint& other) const{
    return std::forward_as_tuple(bin_index, original_index) <
        std::forward_as_tuple(other.bin_index, other.original_index);
}

BasicLine::BasicLine() :label(LineLabel::NAL)
{
    start_point.x=0; start_point.y=0; start_point.z=0; start_point.intensity=0;
    end_point.x=0; end_point.y=0; end_point.z=0; end_point.intensity=0;
}

BasicLine::BasicLine(const LineLabel& labelIn,
                    const pcl::PointXYZI startPoint, 
                    const pcl::PointXYZI endPoint )
    :label(labelIn),start_point(startPoint),end_point(endPoint)
{}

SmartLine::SmartLine(){}

void SmartLine::Reset() 
{
    start_bin = -1;
    end_bin = -1;
    bins_.clear();
    slopes_.clear();
    locOriginX = 0;
    locOriginY = 0;
    locCosTheta = 1;
    locSinTheta = 0;
    last_point_x = 0;
    last_point_y = 0;
    isCalculated = false;
}

bool SmartLine::TryAddNewBin(const int& newBin, const pcl::PointXYZI& binPoint, 
                            const float& groundTolerance, const float& wallTolerance)
{
    /** @brief Note that we use the first two points define a New Local Coordinate System,
     * all slopes btwn two adjacent points are calculated under this coordinate!
     * By doing so, we avoid singularity when tangent angle is near 90 degrees.
     * 
     * the coordinate of a point in Local is calculated by:
     * x' = (x-xOffset)*cos(theta) + (y-yOffset)*sin(theta);
     * y' = (y-yOffset)*cos(theta) - (x-xOffset)*sin(theta);
    **/

    if (bins_.size()==0) {
        start_bin = newBin;
        end_bin = newBin;
        bins_.push_back(newBin);
        locOriginX = binPoint.intensity;
        locOriginY = binPoint.z;
        return true;
    }
    else if (bins_.size()==1) {
        float deltaX = binPoint.intensity - locOriginX;
        float deltaY = binPoint.z - locOriginY;
        float dist = std::sqrt(deltaX*deltaX + deltaY*deltaY);
        if (dist>kMaxDistBtwnRings) return false;
        end_bin = newBin;
        bins_.push_back(newBin);
        slopes_.push_back(0);
        locCosTheta = deltaX/dist;
        locSinTheta = deltaY/dist;
        last_point_x = dist;
        last_point_y = 0;
        return true;

    }
    else /*accumulated_bins>=2*/ {
        float curr_point_x = (binPoint.intensity-locOriginX)*locCosTheta 
                            +(binPoint.z-locOriginY)*locSinTheta;
        float curr_point_y = (binPoint.z-locOriginY)*locCosTheta 
                            -(binPoint.intensity-locOriginX)*locSinTheta;
        float curr_k = (curr_point_y-last_point_y)/(curr_point_x-last_point_x);
        if (locCosTheta<kWallLineSlopeThres) /*a possible wall line.*/ {
            for (const auto& existed_slope : slopes_) {
                if (std::abs(existed_slope-curr_k)>wallTolerance) return false;
            }
        }
        else /*other line.*/ {
            for (const auto& existed_slope : slopes_) {
                if (std::abs(existed_slope-curr_k)>groundTolerance
                    || curr_point_x-last_point_x>kMaxDistBtwnRings ) return false;
            }
        }

        end_bin = newBin;
        bins_.push_back(newBin);
        slopes_.push_back(curr_k);
        last_point_x = curr_point_x;
        last_point_y = curr_point_y;
        return true;
    }

}

BasicLine SmartLine::GetLine(const std::vector<pcl::PointXYZI>& binVec, const LineLabel& labelID)
{
    if (bins_.size()<2 || binVec.size()<2) return BasicLine();
    if (start_bin<0 || (end_bin<=start_bin)) return BasicLine();

    if (!isCalculated) {
        start_point = binVec[start_bin];
        end_point = binVec[end_bin];
        if (!pcl::isFinite(start_point) || !pcl::isFinite(end_point)) return BasicLine();
    }

    return BasicLine(labelID,start_point,end_point);
}

// ******************************************************* //

SmartSector::SmartSector(){}

SmartSector::SmartSector(const unsigned int& n_bins,
                const float& sensor_height,
                const int& this_sector_id,
                const float& ground_same_line_tolerance,
                const float& ground_slope_tolerance,
                const float& ground_intercept_tolerance,
                const float& ground_pointline_dist,
                const float& wall_same_line_tolerance,
                const float& wall_slope_tolerance,
                const float& wall_min_num_bins,
                const float& wall_pointline_dist) 
                : kNumBins(n_bins),
                kSensorHeight(sensor_height),
                kSectorId(this_sector_id),
                kGroundSameLineTolerance(ground_same_line_tolerance),
                kGroundSlopeTolerance(ground_slope_tolerance),
                kGroundYInterceptTolerance(ground_intercept_tolerance),
                kGroundPointLineDistThres(ground_pointline_dist),
                kWallSameLineTolerance(wall_same_line_tolerance),
                kWallSlopeTolerance(wall_slope_tolerance),
                kWallLineMinBinNum(wall_min_num_bins),
                kWallPointLineDistThres(wall_pointline_dist)
                
{
    nanPoint.x = std::numeric_limits<float>::quiet_NaN();
    nanPoint.y = std::numeric_limits<float>::quiet_NaN();
    nanPoint.z = std::numeric_limits<float>::quiet_NaN();
    nanPoint.intensity = -1;
    sector_.resize(kNumBins);
    sectorCounts_.resize(kNumBins);
    Reset();

    // check initialization.
    int num_NanPoint = 0;
    for (auto& binPoint : sector_) {
        if (!pcl::isFinite(binPoint)) num_NanPoint++;
    }
    std::cout << "SmartSector[id=" << kSectorId 
                << "] initialized with [" << num_NanPoint 
                << " out of " << sector_.size() << "] nan points. \n";
}

void SmartSector::Reset()
{
    src_points_.clear();
    std::fill(sector_.begin(), sector_.end(), nanPoint);
    std::fill(sectorCounts_.begin(), sectorCounts_.end(), 0);

    extracted_lines_.clear();

    ground_bins_.clear();
    ground_points_indices_.clear();
    wall_bins_.clear();
    wall_points_indices_.clear();
}

void SmartSector::AddPoint(const pcl::PointXYZI& pointIn, const int& pointIdx, const int& binIdx)
{
    if (binIdx<0 || binIdx>=kNumBins) {
        // TODO: warn.
        return;
    }

    src_points_.emplace(pointIn,binIdx,pointIdx);

    // // 'lowest as indicator' version.
    // if (!pcl::isFinite(sector_[binIdx])) {
    //     sector_[binIdx] = pointIn;
    //     return;
    // }
    // if (pointIn.z < sector_[binIdx].z) sector_[binIdx] = pointIn;

    // 'mean as indicator' version
    if (!pcl::isFinite(sector_[binIdx])) {
        sector_[binIdx] = pointIn;
        sectorCounts_[binIdx]++;
        return;
    }
    sector_[binIdx].x += pointIn.x;
    sector_[binIdx].y += pointIn.y;
    sector_[binIdx].z += pointIn.z;
    sector_[binIdx].intensity += pointIn.intensity;
    sectorCounts_[binIdx]++;

}

void SmartSector::RunLineExtraction()
{
    // debug
    int NaNBins_num = 0;

    // 'mean as indicator' version
    for (std::size_t i=0; i<sector_.size(); ++i) {
        if (!pcl::isFinite(sector_[i]) || sectorCounts_[i]==0) continue;
        sector_[i].x = sector_[i].x/sectorCounts_[i];
        sector_[i].y = sector_[i].y/sectorCounts_[i];
        sector_[i].z = sector_[i].z/sectorCounts_[i];
        sector_[i].intensity = sector_[i].intensity/sectorCounts_[i];
    }

    // extract line.
    SmartLine smart_line;
    for (int i=0; i<sector_.size(); ++i) {
        if ( !pcl::isFinite(sector_[i]) ) { NaNBins_num++; continue; }

        if ( smart_line.TryAddNewBin(i,sector_[i],kGroundSameLineTolerance,kWallSameLineTolerance) ) { /*do nothing*/ }
        else { /* current line ended. */
            LineLabel line_label = JudgeLineLAbel(smart_line, sector_);
            if (line_label!=LineLabel::NAL) {
                extracted_lines_.push_back(smart_line.GetLine(sector_, line_label));
            }
            if (line_label==LineLabel::GROUND) {
                ground_bins_.insert(ground_bins_.end(), smart_line.bins_.begin(), smart_line.bins_.end());
            }
            else if (line_label==LineLabel::WALL) {
                wall_bins_.insert(wall_bins_.end(), smart_line.bins_.begin(), smart_line.bins_.end());
            }
            smart_line.Reset();
            /* start new line after reset. */
            smart_line.TryAddNewBin(i,sector_[i],kGroundSameLineTolerance,kWallSameLineTolerance);
        }

        if (i==sector_.size()-1) {
            LineLabel line_label = JudgeLineLAbel(smart_line, sector_);
            if (line_label!=LineLabel::NAL) {
                extracted_lines_.push_back(smart_line.GetLine(sector_, line_label));
            }
            if (line_label==LineLabel::GROUND) {
                ground_bins_.insert(ground_bins_.end(), smart_line.bins_.begin(), smart_line.bins_.end());
            }
            else if (line_label==LineLabel::WALL) {
                wall_bins_.insert(wall_bins_.end(), smart_line.bins_.begin(), smart_line.bins_.end());
            }
            smart_line.Reset();
        }
    }

    // if (NaNBins_num>kNumBins-3) {
    //     std::cout << "WARNING! SmartSector[id=" << kSectorId << "] has too many NaN bins!" << "\n";
    // }
    // std::cout << "SmartSector[id=" << kSectorId 
    //             << "]: ground bins [" << ground_bins_.size() 
    //             << "], wll bins [" << wall_bins_.size() << "], [" 
    //             << line_check_times << "," << ground_times << "," << wall_times << "], [" << NaNBins_num << "] NaN bins. \n";

    // label GROUND point, collect their indices.
    std::size_t binPointer = 0;
    int unlabeled_bin = -1;
    ground_points_indices_.clear();
    if (!ground_bins_.empty()) {
        for (const auto& point : src_points_) { 
            if (point.bin_index==unlabeled_bin) continue;
            /* search direction: point --> bin. */
            if (point.bin_index!=ground_bins_[binPointer])
            {
                while (point.bin_index!=ground_bins_[binPointer]) {
                    binPointer++;
                    if (binPointer>=ground_bins_.size()) {
                        binPointer = 0;
                        unlabeled_bin = point.bin_index;
                        break;
                    }
                }
            }
            if (point.bin_index!=ground_bins_[binPointer]) continue;
            /* now, judge whether a point belongs to ground (according to height diff). */
            if (point.point.z-sector_[point.bin_index].z<kGroundPointLineDistThres) {
                ground_points_indices_.push_back(point.original_index);
            }
        }
    }

    // label WALL point, collect their indices. 
    binPointer = 0;
    unlabeled_bin = -1;
    wall_points_indices_.clear();
    if (!wall_bins_.empty()) {
        for (const auto& point : src_points_) { 
            if (point.bin_index==unlabeled_bin) continue;
            /* search direction: point --> bin. */
            if (point.bin_index!=wall_bins_[binPointer])
            {
                while (point.bin_index!=wall_bins_[binPointer]) {
                    binPointer++;
                    if (binPointer>=wall_bins_.size()) {
                        binPointer = 0;
                        unlabeled_bin = point.bin_index;
                        break;
                    }
                }
            }
            if (point.bin_index!=wall_bins_[binPointer]) continue;
            /* now, judge whether a point belongs to wall (according to xy-range diff). */
            if ( std::abs(point.point.intensity-sector_[point.bin_index].intensity) <kWallPointLineDistThres) {
                wall_points_indices_.push_back(point.original_index);
            }
        }
    }

}

std::vector<int>& SmartSector::GetGroundPointIndices()
{
    // ground_points_indices_.clear();
    // if (ground_bins_.empty()) {return ground_points_indices_;}
    // std::size_t binPointer = 0;
    // float pointerBinHeight = sector_[ground_bins_[binPointer]].z;
    // int unlabeled_bin = -1;
    // ground_points_indices_.clear();

    // for (const auto& point : src_points_) { 

    //     if (point.bin_index==unlabeled_bin) continue;

    //     // try to find corresponding bin.
    //     if (point.bin_index!=ground_bins_[binPointer])
    //     {
    //         while (point.bin_index!=ground_bins_[binPointer]) {
    //             binPointer++;
    //             if (binPointer>=ground_bins_.size()) {
    //                 binPointer = 0;
    //                 pointerBinHeight = sector_[ground_bins_[binPointer]].z;
    //                 unlabeled_bin = point.bin_index;
    //                 break;
    //             }
    //             pointerBinHeight = sector_[ground_bins_[binPointer]].z;
    //         }
    //     }
    //     if (point.bin_index!=ground_bins_[binPointer]) continue;

    //     // now, judge whether a point belongs to ground.
    //     if (point.point.z-sector_[point.bin_index].z<kGroundPointLineDistThres) {
    //         ground_points_indices_.push_back(point.original_index);
    //     }
    // }

    return ground_points_indices_;
}

std::vector<int>& SmartSector::GetWallPointIndices()
{
    return wall_points_indices_;
}

std::vector<BasicLine>& SmartSector::GetExtractedLines()
{
    return extracted_lines_;
}

int SmartSector::IdentifyExternalPoint(const pcl::PointXYZI& pointIn, const int& binIdx)
{
    if (binIdx<0 || binIdx>=kNumBins) {
        // TODO: warn.
        return -1;
    }

    for (const auto& bin : ground_bins_) {
        if (binIdx==bin) {
            if (std::abs(pointIn.z-sector_[bin].z)<kGroundPointLineDistThres) {
                return 0;
            }
        }
    }

    for (const auto& bin : wall_bins_) {
        if (binIdx==bin) {
            if (std::abs(pointIn.intensity-sector_[bin].intensity)<kWallPointLineDistThres) {
                return 1;
            }
        }
    }

    return -1;
}

LineLabel SmartSector::JudgeLineLAbel(SmartLine& smartLine, const std::vector<pcl::PointXYZI>& binVec)
{
    if (smartLine.bins_.size()<2 || binVec.size()<2) return LineLabel::NAL;
    if (smartLine.start_bin<0 || (smartLine.end_bin<=smartLine.start_bin)) return LineLabel::NAL;

    if (!smartLine.isCalculated) {
        smartLine.start_point = binVec[smartLine.start_bin];
        smartLine.end_point = binVec[smartLine.end_bin];
        if (!pcl::isFinite(smartLine.start_point) || !pcl::isFinite(smartLine.end_point)) return LineLabel::NAL;
        float deltaX = smartLine.end_point.intensity-smartLine.start_point.intensity;
        float deltaY = smartLine.end_point.z-smartLine.start_point.z;
        smartLine.k_ = deltaY/deltaX;
        smartLine.b_ = smartLine.start_point.z - smartLine.k_ * smartLine.start_point.intensity;
        smartLine.length_ = std::sqrt(deltaY*deltaY+deltaX*deltaX);
        smartLine.isCalculated = true;
    }

    // check whether this line is a ground line.
    if ((smartLine.bins_.size()>2
         &&std::abs(smartLine.k_)<kGroundSlopeTolerance
         && std::abs(smartLine.b_+kSensorHeight)<kGroundYInterceptTolerance)
        || 
        (smartLine.bins_.size()==2
         &&std::abs(smartLine.k_)<0.01745 /*1 or 2 degree*/
         && std::abs(smartLine.b_+kSensorHeight)<0.5*kGroundYInterceptTolerance) )
    {
        return LineLabel::GROUND;
    }
    // check whether this line is a wall line.
    else if (std::abs(smartLine.k_)>kWallSlopeTolerance
            && smartLine.bins_.size()>=kWallLineMinBinNum ) {
        return LineLabel::WALL;
    }

    return LineLabel::NAL;
}







