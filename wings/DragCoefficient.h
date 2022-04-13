#include "../include/MatrixDef.h"
#include "./WingProperties.h"
#include "./LiftCoefficient.h"

#ifndef PI
#define PI 3.14159
#endif
#pragma once

namespace DragCoefficient {
    float CD_0 = 0.05; //Parasitic drag coefficient;
    float e = 0.7; //Wing efficiency factor

    float Value(float AoA, float SideSlip, float MachNumber) {
        float CL = LiftCoefficient::Value(AoA,SideSlip,MachNumber);
        return CD_0 + (std::pow(CL,2)/(PI*e*WingProps::AR));
    };

}