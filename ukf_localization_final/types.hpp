#ifndef TYPES_HPP
#define TYPES_HPP

#include <Eigen/Dense>

typedef Eigen::ArrayXi Mask;

typedef Eigen::VectorXd Vector;
typedef Eigen::ArrayXd Array;

typedef Eigen::Vector3d Pose;
typedef Eigen::Array3d PoseArray;
typedef Eigen::Matrix3d PoseCov;
typedef Eigen::Matrix3Xd Poses;

typedef Eigen::Quaterniond Quaternion;

typedef Eigen::Vector2d Point;
typedef Eigen::Matrix2Xd Points;

typedef Eigen::Vector4d SensorData;

// Added for UKF implementation
typedef Eigen::MatrixXd Matrix;

struct EnemyData
{
    EnemyData()
        : time(0.0)
        , position(0., 0.)
    {
    }

    EnemyData(double recvTime, const Point& position)
        : time(recvTime)
        , position(position)
    {
    }

    double time;
    Point position;
};

#endif
