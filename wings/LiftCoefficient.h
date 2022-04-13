#include "../include/MatrixDef.h"
#include "./WingProperties.h"

#ifndef PI
#define PI 3.14159
#endif


#pragma once
namespace LiftCoefficient {
    float CL_alpha = 0.25; //Slope of lift coefficient versus AoA. Expressed in deg^-1
    float CL_max = 2.5; //Maximum lift coefficient before stall
    float CL_0 = 0.0;
    float CL_stall = 2.0; //Assume lift coefficient drops after stall and remains constant
    float Stall_angle = 15;

    float Value(float AoA, float SideSlip, float MachNumber) {
        //Assumes that all angles are expressed in radians 
        //Mach number independence is assumed
        float AoADeg = AoA*(180.0/PI);
        if (std::abs(AoADeg) > Stall_angle) {
            return CL_stall;
        }
        return (AoADeg*CL_alpha + CL_0);
    };
}