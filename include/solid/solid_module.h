#ifndef SOLiD_H
#define SOLiD_H

#include <ctime>
#include <cassert>
#include <cmath>
#include <utility>
#include <vector>
#include <algorithm> 
#include <cstdlib>
#include <memory>
#include <iostream>

#include <opencv2/opencv.hpp>
// #include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

#include "solid/nanoflann.hpp"
#include "solid/KDTreeVectorOfVectorsAdaptor.h"
#include "aloam_velodyne/tic_toc.h"

typedef pcl::PointXYZI PointType;

using namespace Eigen;
using namespace nanoflann;
using KeyMat = std::vector<std::vector<float> >;
using solidKeyTree = KDTreeVectorOfVectorsAdaptor< KeyMat, float >;

inline float calc_dist(float &x, float &y, float &z){return sqrt(x*x + y*y + z*z);}
inline float rad2deg(float rad){return rad * 180.0 / M_PI;}
inline float deg2rad(float deg){return deg * M_PI / 180.0;}
struct RAH 
{
    int idx_range = 0;
    int idx_angle = 0;
    int idx_height = 0;
};

class SOLiDModule 
{
public:
    double FOV_u;
    double FOV_d;
    int NUM_ANGLE;
    int NUM_RANGE;
    int NUM_HEIGHT;
    int MIN_DISTANCE;
    int MAX_DISTANCE;
    double VOXEL_SIZE;
    int NUM_EXCLUDE_RECENT;
    int NUM_CANDIDATES_FROM_TREE;
    double R_SOLiD_THRES;

    const int TREE_MAKING_PERIOD_ = 30;
    int       tree_making_period_conter = 0;

    double rsolid_timestamp_;
    std::vector<Eigen::VectorXd> solids_;

    KeyMat solids_mat_;
    KeyMat solids_to_search_;
    std::unique_ptr<solidKeyTree> solid_tree_;

    bool is_tree_batch_made = false;
    std::unique_ptr<solidKeyTree> solid_tree_batch_;
public:
    // Constructor
    SOLiDModule( ) = default;
    void setParams(double FOV_u_, double FOV_d_, int NUM_ANGLE_, int NUM_RANGE_, int NUM_HEIGHT_, int MIN_DISTANCE_, int MAX_DISTANCE_, double VOXEL_SIZE_, int NUM_EXCLUDE_RECENT_, int NUM_CANDIDATES_FROM_TREE_, double R_SOLiD_THRES_);
    Eigen::VectorXd makeSolid(pcl::PointCloud<PointType> scan_down);
    Eigen::VectorXd getRSolid(const Eigen::VectorXd &solid);
    Eigen::VectorXd getASolid(const Eigen::VectorXd &solid);
    const Eigen::VectorXd& getConstRefRecentSOLiD(void);
    RAH pt2rah(PointType & scan_down, float gap_angle, float gap_range, float gap_elevation);

    double calc_cosDist(const Eigen::VectorXd &query, const Eigen::VectorXd &candidate);
    double pose_estimation(const Eigen::VectorXd &query_a, const Eigen::VectorXd &candidate_a);
    std::pair<double, int> distanceBtnSOLiD(const Eigen::VectorXd &query, const Eigen::VectorXd &candidate);
    void saveSolid(Eigen::VectorXd solid);
    void makeAndSaveSolid(pcl::PointCloud<PointType> &scan_down);
    
    std::pair<int, float> detectLoopClosureID(void);

    ////// point module //////
    void remove_far_points(pcl::PointCloud<PointType> & scan_raw, pcl::PointCloud<PointType>::Ptr scan_out);
    void remove_closest_points(pcl::PointCloud<PointType> & scan_raw, pcl::PointCloud<PointType>::Ptr scan_out);
    void down_sampling(pcl::PointCloud<PointType> & scan_raw, pcl::PointCloud<PointType>::Ptr scan_down);


};

#endif /* SOLiD_H */
