#pragma once

#include <Eigen/Dense>

using namespace Eigen;

class SOLiDManager
{
public:
    SOLiDManager() = default;
    Eigen::MatrixXd makeSOLiD(pcl::PointCloud<SCPointType> & _scan_down);

};