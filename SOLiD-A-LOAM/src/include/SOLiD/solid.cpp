#include "SOLiD/solid.h"

float xy2angle(const float & _x, const float & _y)
{
    if ( (_x >= 0) & (_y >= 0)) 
        return (180/M_PI) * atan(_y / _x);

    if ( (_x < 0) & (_y >= 0)) 
        return 180 - ( (180/M_PI) * atan(_y / (-_x)) );

    if ( (_x < 0) & (_y < 0)) 
        return 180 + ( (180/M_PI) * atan(_y / _x) );

    if ( (_x >= 0) & (_y < 0))
        return 360 - ( (180/M_PI) * atan((-_y) / _x) );
}

float rz2elevation(const float & _r, const float & _z)
{
    return (180/M_PI) * atan2(_z / _r)
}


MatrixXd SOLiDManager::makeSOLiD(pcl::PointCloud<SCPointType> & _scan_down)
{
    SOLiDPointType pt;
    int num_pts_scan_down = _scan_down.points.size();

    for (int pt_idx=0; pt_idx < num_pts_scan_down; pt_idx++)
    {
        pt.x = _scan_down.points[pt_idx].x; 
        pt.y = _scan_down.points[pt_idx].y;
        pt.z = _scan_down.points[pt_idx].z;

        range = sqrt(pt.x * pt.x + pt.y * pt.y);
        angle = xy2angle(pt.x, pt.y);
        elevation = rz2elevation(range, pt.z);
    }

}