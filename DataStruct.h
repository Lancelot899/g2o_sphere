#ifndef DATASTRUCT_H
#define DATASTRUCT_H

#include <Eigen/Dense>
#include <sophus/se3.hpp>

struct Vertex
{
    int                         index;
    Eigen::Matrix<double, 6, 1> pose;
};

struct Edge
{
    int                          i;
    int                          j;
    Sophus::SE3d                 pose;
    Eigen::Matrix<double, 6, 6>  infomation;
};


#endif // DATASTRUCT_H
