#include<iostream>
#include "../../Eigen/Dense"
#include "../LiftCoefficient.h"

int main() {
    Eigen::Matrix<float, 20,1> AoA;
    AoA << -0.8727,
    -0.7808,
    -0.6889,
   -0.5971,
   -0.5052,
   -0.4134,
   -0.3215,
   -0.2296,
   -0.1378,
   -0.0459,
    0.0459,
    0.1378,
    0.2296,
    0.3215,
    0.4134,
    0.5052,
    0.5971,
    0.6889,
    0.7808,
    0.8727;

    Eigen::Matrix<float, 20,1> SideSlip;
    SideSlip << -0.8727,
    -0.7808,
    -0.6889,
   -0.5971,
   -0.5052,
   -0.4134,
   -0.3215,
   -0.2296,
   -0.1378,
   -0.0459,
    0.0459,
    0.1378,
    0.2296,
    0.3215,
    0.4134,
    0.5052,
    0.5971,
    0.6889,
    0.7808,
    0.8727;

    for (int i = 0; i < AoA.rows(); i++) {
        for (int j = 0; j < SideSlip.rows(); j++) {
            std::cout << " | " << "AoA: " << AoA(i,0) << " | " << "SideSlip: " << SideSlip(j,0) << " | " << LiftCoefficient::Value(AoA(i,0),SideSlip(j,0),0) << " | " << std::endl;
        }
    }

   
}