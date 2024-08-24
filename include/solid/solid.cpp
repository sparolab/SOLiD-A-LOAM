#include <memory>
#include "solid/solid_module.h"

////////////////////////////// solid module //////////////////////////////
float xy2theta(float &x, float &y)
{
    if ((x >= 0) && (y >= 0))
        return (180/M_PI) * atan(y/x);
    if ((x < 0) && (y > 0))
        return 180 - ((180/M_PI) ) * atan(y/(-x));
    if ((x < 0) && (y < 0))
        return 180 + ((180/M_PI) ) * atan(y/x);
    if ((x >= 0) && (y < 0))
        return 360 - ((180/M_PI) * atan((-y)/x));
}

std::vector<float> eig2stdvec(const Eigen::VectorXd& _eigvec) {
    std::vector<float> vec(_eigvec.size());
    // Eigen::VectorXd를 std::vector<float>로 변환
    for(int i = 0; i < _eigvec.size(); ++i) {
        vec[i] = static_cast<float>(_eigvec[i]);
    }
    return vec;
}

RAH SOLiDModule::pt2rah(PointType & point, float gap_angle, float gap_range, float gap_height)
{
    RAH rah;
    float point_x = point.x;
    float point_y = point.y;
    float point_z = point.z;

    if(point_x == 0.0)
        point_x = 0.001;
    if(point_y == 0.0)
        point_y = 0.001;

    float theta = xy2theta(point_x, point_y);
    float dist_xy = sqrt(point_x*point_x + point_y*point_y);
    float phi = rad2deg(atan2(point_z, dist_xy));
    rah.idx_range = std::min(static_cast<int>(dist_xy / gap_range), NUM_RANGE - 1);
    rah.idx_angle = std::min(static_cast<int>(theta / gap_angle), NUM_ANGLE - 1);
    rah.idx_height = std::min(static_cast<int>((phi - FOV_d)/gap_height), NUM_HEIGHT - 1);
    // std::cout << rah.idx_height << std::endl;
    return rah;
}

void SOLiDModule::setParams(double FOV_u_, double FOV_d_, int NUM_ANGLE_, int NUM_RANGE_, int NUM_HEIGHT_, int MIN_DISTANCE_, int MAX_DISTANCE_, double VOXEL_SIZE_, int NUM_EXCLUDE_RECENT_, int NUM_CANDIDATES_FROM_TREE_, double R_SOLiD_THRES_)
{
    SOLiDModule::FOV_u = FOV_u_;
    SOLiDModule::FOV_d = FOV_d_;
    SOLiDModule::NUM_ANGLE = NUM_ANGLE_;
    SOLiDModule::NUM_RANGE = NUM_RANGE_;
    SOLiDModule::NUM_HEIGHT = NUM_HEIGHT_;
    SOLiDModule::MIN_DISTANCE = MIN_DISTANCE_;
    SOLiDModule::MAX_DISTANCE = MAX_DISTANCE_;
    SOLiDModule::VOXEL_SIZE = VOXEL_SIZE_;
    SOLiDModule::NUM_EXCLUDE_RECENT = NUM_EXCLUDE_RECENT_;
    SOLiDModule::NUM_CANDIDATES_FROM_TREE = NUM_CANDIDATES_FROM_TREE_;
    SOLiDModule::R_SOLiD_THRES = R_SOLiD_THRES_;
}
void SOLiDModule::saveSolid(Eigen::VectorXd solid) 
{
    solids_.push_back(solid);
}

const Eigen::VectorXd& SOLiDModule::getConstRefRecentSOLiD(void)
{
    return solids_.back();
}

void SOLiDModule::makeAndSaveSolid(pcl::PointCloud<PointType> &scan_down) 
{
    Eigen::VectorXd solid(NUM_RANGE  + NUM_ANGLE);
    Eigen::MatrixXd range_matrix(NUM_RANGE, NUM_HEIGHT);
    Eigen::MatrixXd angle_matrix(NUM_ANGLE, NUM_HEIGHT);
    range_matrix.setZero();
    angle_matrix.setZero();
    solid.setZero();

    float gap_angle = 360/NUM_ANGLE;
    float gap_range = static_cast<float>(MAX_DISTANCE)/NUM_RANGE;
    float gap_height = (FOV_u - FOV_d) / NUM_HEIGHT;
    // std::cout << NUM_RANGE << std::endl;
    for(int i = 0; i < scan_down.points.size(); i++)
    {
        RAH rah = pt2rah(scan_down.points[i], gap_angle, gap_range, gap_height);
        if(rah.idx_height < 0)
            continue;
        range_matrix(rah.idx_range, rah.idx_height) +=1;
        angle_matrix(rah.idx_angle, rah.idx_height) +=1;
    }

    Eigen::VectorXd number_vector(NUM_HEIGHT);
    number_vector.setZero();
    for(int col_idx=0; col_idx<range_matrix.cols(); col_idx++)
    {
        number_vector(col_idx) = range_matrix.col(col_idx).sum();
    }

    // min max nomalization
    double min_val = number_vector.minCoeff();
    double max_val = number_vector.maxCoeff();
    number_vector = (number_vector.array() - min_val) / (max_val - min_val);
    
    Eigen::VectorXd range_solid = range_matrix * number_vector;
    Eigen::VectorXd angle_solid = angle_matrix * number_vector;
    solid.head(NUM_RANGE) = range_solid;
    solid.tail(NUM_ANGLE) = angle_solid;

    std::vector<float> solid_vec = eig2stdvec(solid);

    solids_.push_back(solid);
    solids_mat_.push_back(solid_vec);
}

std::pair<double, int> SOLiDModule::distanceBtnSOLiD(const Eigen::VectorXd &query, const Eigen::VectorXd &candidate)
{
    // R-SOLiD for loop detection
    Eigen::VectorXd r_solid_q = getRSolid(query);
    Eigen::VectorXd r_solid_c = getRSolid(candidate);
    // A-SOLiD for calculating initial angle
    Eigen::VectorXd a_solid_q = getASolid(query);
    Eigen::VectorXd a_solid_c = getASolid(candidate);
    int argmin_shift = 0;
    double minL1normDist = std::numeric_limits<double>::max();

    // calculate min cosine distance of R-SOLiD
    double rsolid_dist = (r_solid_q.dot(r_solid_c))/(r_solid_q.norm() * r_solid_c.norm());
    
    // calculate initial rotation angle with A-SOLiD
    for(int shiftIndex = 0; shiftIndex < NUM_ANGLE; ++shiftIndex)
    {
        Eigen::VectorXd shiftedQuery = Eigen::VectorXd::Zero(NUM_ANGLE);
        for (int i = 0; i < NUM_ANGLE; ++i)
        {
            shiftedQuery((i+shiftIndex) % NUM_ANGLE) = a_solid_q(i);
        }
        double L1NormDist = (a_solid_c - shiftedQuery).cwiseAbs().sum();
        if(L1NormDist < minL1normDist)
        {
            minL1normDist = L1NormDist;
            argmin_shift = shiftIndex;
        }
    }
    return std::make_pair(rsolid_dist, argmin_shift);
}

Eigen::VectorXd SOLiDModule::getRSolid(const Eigen::VectorXd &solid)
{
    return solid.segment(0, NUM_RANGE);
}

Eigen::VectorXd SOLiDModule::getASolid(const Eigen::VectorXd &solid)
{
    return solid.segment(NUM_RANGE, NUM_ANGLE);
}

std::pair<int, float> SOLiDModule::detectLoopClosureID( void )
{
    int loop_id { -1 };
    if(solids_.empty() && solids_mat_.empty())
    {
        return std::make_pair(-1, 0.0f);
    }

    auto curr_solid = solids_.back();
    auto curr_solid_vec = solids_mat_.back();
    if(solids_.size() < NUM_EXCLUDE_RECENT + 1)
    {
        return std::make_pair(-1, 0.0f);
    }

    double min_dist = std::numeric_limits<double>::min();
    int nn_align = 0;
    int nn_idx = -1;

    for(size_t i = 0; i < solids_.size() - NUM_EXCLUDE_RECENT; ++i)
    {
        auto solid_candidate = solids_[i];
        auto dist_result = distanceBtnSOLiD(curr_solid, solid_candidate);

        double candidate_dist = dist_result.first;
        if(candidate_dist > min_dist)
        {
            min_dist = candidate_dist;
            nn_align = dist_result.second;
            nn_idx = i;
        }
    }
    std::cout << min_dist << std::endl;
    if(min_dist > R_SOLiD_THRES)
    {
        std::cout << "\033[1;32m[Loop found] Nearest distance: " << min_dist << " between " << solids_.size() - 1 << " and " << nn_idx << "\033[0m" << std::endl;
        loop_id = nn_idx;
    }
    else
    {
        std::cout << "\033[1;31m[Not loop] Nearest distance: " << min_dist << " between " << solids_.size() - 1 << " and " << nn_idx << "\033[0m" << std::endl;
    }

    float yaw_diff_rad = deg2rad((nn_align + 1) * (360.0 / NUM_ANGLE));
    return std::make_pair(loop_id, yaw_diff_rad);
}

//////////////////////////////////////////////////////////////////////////

////////////////////////////// point module //////////////////////////////
void SOLiDModule::remove_far_points(pcl::PointCloud<PointType> & scan_raw, pcl::PointCloud<PointType>::Ptr scan_out)
{
    for(int i = 0; i < scan_raw.points.size(); i++)
    {
        float dist = calc_dist(scan_raw.points[i].x, scan_raw.points[i].y, scan_raw.points[i].z);
        if(dist < MAX_DISTANCE)
        {
            scan_out->points.push_back(scan_raw.points[i]);
        }
    }
}

void SOLiDModule::remove_closest_points(pcl::PointCloud<PointType> & scan_raw, pcl::PointCloud<PointType>::Ptr scan_out)
{
    for(int i = 0; i < scan_raw.size(); i++)
    {
        float dist = calc_dist(scan_raw.points[i].x, scan_raw.points[i].y, scan_raw.points[i].z);
        if(dist > MIN_DISTANCE)
        {
            scan_out->points.push_back(scan_raw.points[i]);
        }
    }
}

void SOLiDModule::down_sampling(pcl::PointCloud<PointType> & scan_raw, pcl::PointCloud<PointType>::Ptr scan_down)
{
    pcl::PointCloud<PointType> Data_for_Voxel;
    pcl::VoxelGrid<PointType> downSizeFilterSolid;

    copyPointCloud(scan_raw, Data_for_Voxel);
    downSizeFilterSolid.setInputCloud(Data_for_Voxel.makeShared());
    downSizeFilterSolid.setLeafSize(VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE);
    downSizeFilterSolid.filter(*scan_down);
}
//////////////////////////////////////////////////////////////////////////