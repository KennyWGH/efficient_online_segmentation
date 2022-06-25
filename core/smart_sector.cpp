/*
 * Copyright 2021 Guanhua WANG
 */

#include "smart_sector.h"
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

SmartLine::SmartLine(const float& wall_slope_t, 
                    const float& wall_same_line_slope_t, 
                    const float& ground_slope_t, 
                    const float& ground_same_line_slope_t, 
                    const float& ground_max_ring_dist)
    :kWallSlopeThres(wall_slope_t), 
    kWallSameLineSlopeThres(wall_same_line_slope_t),
    kGroundSlopeThres(ground_slope_t),
    kGroundSameLineSlopeThres(ground_same_line_slope_t),
    kGroundMaxDistBtwnRings(ground_max_ring_dist) 
{}

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
    locTanTheta = 0;
    last_point_x = 0;
    last_point_y = 0;
    isCalculated = false;
}

bool SmartLine::TryAddNewBin(const int& newBin, const pcl::PointXYZI& binPoint)
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
        locOriginX = binPoint.intensity; /*x-y range*/
        locOriginY = binPoint.z;
        return true;
    }
    else if (bins_.size()==1) {
        float deltaX = binPoint.intensity - locOriginX;
        float deltaY = binPoint.z - locOriginY;
        float dist = std::sqrt(deltaX*deltaX + deltaY*deltaY);
        if (dist > kGroundMaxDistBtwnRings) return false;
        end_bin = newBin;
        bins_.push_back(newBin);
        slopes_.push_back(0);
        locCosTheta = deltaX/dist;
        locSinTheta = deltaY/dist;
        locTanTheta = deltaY/deltaX;
        last_point_x = dist;
        last_point_y = 0;
        return true;
    }
    else /*accumulated_bins>=2*/ {
        // calculate point coordinate in local frame.
        float curr_point_x = (binPoint.intensity-locOriginX)*locCosTheta 
                            +(binPoint.z-locOriginY)*locSinTheta;
        float curr_point_y = (binPoint.z-locOriginY)*locCosTheta 
                            -(binPoint.intensity-locOriginX)*locSinTheta;
        float curr_k = (curr_point_y - last_point_y) / (curr_point_x - last_point_x);
        if (std::abs(locTanTheta) > kWallSlopeThres) /*a possible wall line.*/ {
            for (const auto& existed_slope : slopes_) {
                if (std::abs(existed_slope - curr_k) > kWallSameLineSlopeThres) return false;
            }
        }
        else if (std::abs(locTanTheta) < kGroundSlopeThres) /*a possible ground line.*/ {
            for (const auto& existed_slope : slopes_) {
                if (std::abs(existed_slope - curr_k) > kGroundSameLineSlopeThres
                    || curr_point_x - last_point_x > kGroundMaxDistBtwnRings ) return false;
            }
        }
        else /*other line.*/ { return false; }

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

SmartSector::SmartSector(const int& this_sector_id,
                const unsigned int& n_bins,
                const float& sensor_height,
                const float& ground_same_line_tolerance,
                const float& ground_slope_tolerance,
                const float& ground_intercept_tolerance,
                const float& ground_pointline_dist,
                const float& wall_same_line_tolerance,
                const float& wall_slope_tolerance,
                const float& wall_min_num_bins,
                const float& wall_pointline_dist) 
                : kSectorId(this_sector_id),
                kNumBins(n_bins),
                kSensorHeight(sensor_height),
                kGroundSameLineTolerance(ground_same_line_tolerance),
                kGroundSlopeTolerance(ground_slope_tolerance),
                kGroundYInterceptTolerance(ground_intercept_tolerance),
                kGroundPointLineDistThres(ground_pointline_dist),
                kWallSameLineTolerance(wall_same_line_tolerance),
                kWallSlopeTolerance(wall_slope_tolerance),
                kWallLineMinBinNum(wall_min_num_bins),
                kWallPointLineDistThres(wall_pointline_dist)
{
    NanPoint.x = std::numeric_limits<float>::quiet_NaN();
    NanPoint.y = std::numeric_limits<float>::quiet_NaN();
    NanPoint.z = std::numeric_limits<float>::quiet_NaN();
    NanPoint.intensity = -1;
    sector_data_ptr_.reset(new std::vector<pcl::PointXYZI>);
    sector_data_ptr_->resize(kNumBins);
    sector_counts_.resize(kNumBins);
    Reset();

    // Initialize sector data.
    int num_nan_bins = 0;
    for (auto& binPoint : (*sector_data_ptr_)) {
        if (!pcl::isFinite(binPoint)) num_nan_bins++;
    }
    if (kDebugInfo) {
        std::cout << "SmartSector[id=" << kSectorId 
                << "] initialized with [" << num_nan_bins 
                << " out of " << sector_data_ptr_->size() << "] nan bins." << std::endl;
    }

}

void SmartSector::Reset()
{
    src_points_.clear();
    std::fill(sector_data_ptr_->begin(), sector_data_ptr_->end(), NanPoint);
    std::fill(sector_counts_.begin(), sector_counts_.end(), 0);

    ground_bins_.clear();
    ground_points_indices_.clear();
    wall_bins_.clear();
    wall_points_indices_.clear();

    extracted_lines_.clear();
}

std::shared_ptr<const std::vector<pcl::PointXYZI>> SmartSector::GetSectorDataPtr()
{
    return sector_data_ptr_;
}

void SmartSector::SetReferenceSector(const std::shared_ptr<const std::vector<pcl::PointXYZI>>& sect_ptr_in)
{
    other_sector_ptr_ = sect_ptr_in;
}

void SmartSector::AddPoint(const pcl::PointXYZI& pointIn, const int& pointIdx, const int& binIdx)
{
    if (binIdx<0 || binIdx>=kNumBins) {
        // TODO: warn.
        return;
    }

    src_points_.emplace(pointIn,binIdx,pointIdx);

    // 'lowest as indicator' version.
    if (!pcl::isFinite((*sector_data_ptr_)[binIdx])) {
        (*sector_data_ptr_)[binIdx] = pointIn;
        return;
    }
    if (pointIn.z < (*sector_data_ptr_)[binIdx].z) (*sector_data_ptr_)[binIdx] = pointIn;

    // // 'mean as bin indicator' version
    // if (!pcl::isFinite((*sector_data_ptr_)[binIdx])) {
    //     (*sector_data_ptr_)[binIdx] = pointIn;
    //     sector_counts_[binIdx]++;
    //     return;
    // }
    // (*sector_data_ptr_)[binIdx].x += pointIn.x;
    // (*sector_data_ptr_)[binIdx].y += pointIn.y;
    // (*sector_data_ptr_)[binIdx].z += pointIn.z;
    // (*sector_data_ptr_)[binIdx].intensity += pointIn.intensity;
    // sector_counts_[binIdx]++;

}

void SmartSector::RunLineExtraction()
{
    // debug
    int num_nan_bins = 0;

    // // 'mean as bin indicator' version
    // for (std::size_t i=0; i<sector_data_ptr_->size(); ++i) {
    //     if (!pcl::isFinite((*sector_data_ptr_)[i]) || sector_counts_[i]==0) continue;
    //     (*sector_data_ptr_)[i].x = (*sector_data_ptr_)[i].x/sector_counts_[i];
    //     (*sector_data_ptr_)[i].y = (*sector_data_ptr_)[i].y/sector_counts_[i];
    //     (*sector_data_ptr_)[i].z = (*sector_data_ptr_)[i].z/sector_counts_[i];
    //     (*sector_data_ptr_)[i].intensity = (*sector_data_ptr_)[i].intensity/sector_counts_[i];
    // }

    // extract line.
    SmartLine smart_line(kWallSlopeTolerance, 
                        kWallSameLineTolerance,
                        kGroundSlopeTolerance,
                        kGroundSameLineTolerance);
    for (int i=0; i<sector_data_ptr_->size(); ++i) {
        if ( !pcl::isFinite((*sector_data_ptr_)[i]) ) { num_nan_bins++; continue; }

        if (smart_line.TryAddNewBin(i,(*sector_data_ptr_)[i])) { /*need to do nothing*/ }
        else { /* current line ended. */
            LineLabel line_label = JudgeLineLAbel(smart_line, (*sector_data_ptr_));
            if (line_label!=LineLabel::NAL) {
                extracted_lines_.push_back(smart_line.GetLine((*sector_data_ptr_), line_label));
            }
            if (line_label==LineLabel::GROUND) {
                ground_bins_.insert(ground_bins_.end(), smart_line.bins_.begin(), smart_line.bins_.end());
            }
            else if (line_label==LineLabel::WALL) {
                wall_bins_.insert(wall_bins_.end(), smart_line.bins_.begin(), smart_line.bins_.end());
            }
            smart_line.Reset();
            /* start new line after reset. */
            smart_line.TryAddNewBin(i,(*sector_data_ptr_)[i]);
        }

        if (i==sector_data_ptr_->size()-1) {
            LineLabel line_label = JudgeLineLAbel(smart_line, (*sector_data_ptr_));
            if (line_label!=LineLabel::NAL) {
                extracted_lines_.push_back(smart_line.GetLine((*sector_data_ptr_), line_label));
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

    if (kDebugInfo) {
        std::cout << "SmartSector[id=" << kSectorId 
                << "]: ground bins [" << ground_bins_.size() 
                << "], wll bins [" << wall_bins_.size()
                << "], nan bins [" << num_nan_bins 
                << "], out of [" << kNumBins << "]." << std::endl;
    }

    // label GROUND point, collect their indices.
    std::size_t binPointer = 0;
    int non_class_bin = -1;
    ground_points_indices_.clear();
    if (!ground_bins_.empty()) {
        /* all points of src_points_ have been sorted in an bin-index ascending order. */
        for (const auto& point : src_points_) { 
            if (point.bin_index==non_class_bin) continue;
            /* try to find corresponding bin index for a point. */
            if (point.bin_index!=ground_bins_[binPointer])
            {
                while (point.bin_index!=ground_bins_[binPointer]) {
                    binPointer++;
                    if (binPointer>=ground_bins_.size()) {
                        binPointer = 0;
                        non_class_bin = point.bin_index;
                        break;
                    }
                }
            }
            if (point.bin_index!=ground_bins_[binPointer]) continue;
            /* now, judge whether a point belongs to ground (according to height diff). */
            if (std::abs(point.point.z-(*sector_data_ptr_)[point.bin_index].z)<kGroundPointLineDistThres) {
                ground_points_indices_.push_back(point.original_index);
            }
        }
    }

    // label WALL point, collect their indices. 
    binPointer = 0;
    non_class_bin = -1;
    wall_points_indices_.clear();
    if (!wall_bins_.empty()) {
        for (const auto& point : src_points_) { 
            if (point.bin_index==non_class_bin) continue;
            /* try to find corresponding bin index for a point. */
            if (point.bin_index!=wall_bins_[binPointer])
            {
                while (point.bin_index!=wall_bins_[binPointer]) {
                    binPointer++;
                    if (binPointer>=wall_bins_.size()) {
                        binPointer = 0;
                        non_class_bin = point.bin_index;
                        break;
                    }
                }
            }
            if (point.bin_index!=wall_bins_[binPointer]) continue;
            /* now, judge whether a point belongs to wall (according to xy-range diff). */
            float point_plane_distance = 
                std::abs(point.point.intensity-(*sector_data_ptr_)[point.bin_index].intensity);
            /* calculate point-to-plane distance with z=0, since xy distance is enough. */
            if (other_sector_ptr_ != nullptr && pcl::isFinite((*other_sector_ptr_)[point.bin_index])) {
                float x1 = point.point.x - (*sector_data_ptr_)[point.bin_index].x;
                float y1 = point.point.y - (*sector_data_ptr_)[point.bin_index].y;
                float x2 = (*other_sector_ptr_)[point.bin_index].x - (*sector_data_ptr_)[point.bin_index].x;
                float y2 = (*other_sector_ptr_)[point.bin_index].y - (*sector_data_ptr_)[point.bin_index].y;
                point_plane_distance = std::abs((x1*y2-x2*y1)/std::sqrt(x2*x2+y2*y2));
            }
            if ( point_plane_distance < kWallPointLineDistThres) {
                wall_points_indices_.push_back(point.original_index);
            }
        }
    }

}

std::vector<int>& SmartSector::GetGroundPointIndices()
{
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
        return 0;
    }

    for (const auto& bin : ground_bins_) {
        if (binIdx==bin) {
            if (std::abs(pointIn.z-(*sector_data_ptr_)[bin].z)<kGroundPointLineDistThres) {
                return 1;
            }
        }
    }

    for (const auto& bin : wall_bins_) {
        if (binIdx==bin) {
            float point_plane_distance = std::abs(pointIn.intensity-(*sector_data_ptr_)[bin].intensity);
            // calculate point-to-plane distance with z=0, since xy distance is enough.
            if (other_sector_ptr_ != nullptr && pcl::isFinite((*other_sector_ptr_)[bin])) {
                float x1 = pointIn.x - (*sector_data_ptr_)[bin].x;
                float y1 = pointIn.y - (*sector_data_ptr_)[bin].y;
                float x2 = (*other_sector_ptr_)[bin].x - (*sector_data_ptr_)[bin].x;
                float y2 = (*other_sector_ptr_)[bin].y - (*sector_data_ptr_)[bin].y;
                point_plane_distance = std::abs((x1*y2-x2*y1)/std::sqrt(x2*x2+y2*y2));
            }
            if (point_plane_distance < kWallPointLineDistThres) {
                return 2;
            }
        }
    }

    return 0;
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
        // smartLine.k_ = deltaY/deltaX;           // strategy A. 
        smartLine.k_ = smartLine.locTanTheta;   // strategy B. 
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
         &&std::abs(smartLine.k_)<0.01745 /*1 degree*/
         && std::abs(smartLine.start_point.z)<kGroundYInterceptTolerance
         && std::abs(smartLine.end_point.z)<kGroundYInterceptTolerance
         && std::abs(smartLine.b_+kSensorHeight)<0.5*kGroundYInterceptTolerance) )
    {
        return LineLabel::GROUND;
    }
    // check whether this line is a wall line.
    else if (std::abs(smartLine.k_)>kWallSlopeTolerance
            && smartLine.bins_.size()>=kWallLineMinBinNum ) 
    {
        return LineLabel::WALL;
    }

    return LineLabel::NAL;
}







