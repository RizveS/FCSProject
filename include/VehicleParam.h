#include "./MatrixDef.h"
#pragma once
struct VehicleParamStruct {
    float m; // Mass of vehicle
    MAT_3X3 I; // Moment of inertia of vehicle

    float WingSpan;
    float WingArea;
    float RearRotorBladeLen;


    VEC_3X1 RightRotorPos;
    VEC_3X1 LeftRotorPos;
    VEC_3X1 RearRotorPos;
    VEC_3X1 RudderPos;
    VEC_3X1 ElevatorPos;

};

