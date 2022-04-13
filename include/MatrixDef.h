#include "../Eigen/Dense"
#pragma once 
typedef Eigen::Matrix<double,3,3> MAT_3X3; // Redefines the type to MAT for easy use. Most matrices used are 3x3
typedef Eigen::Matrix<double,3,1> VEC_3X1; // Redefines the type to VEC for easy use. Most vectors used are 3x1
typedef Eigen::Matrix<double,12,1> VEC_12X1; //State vector for dynamics is 12 x 1
