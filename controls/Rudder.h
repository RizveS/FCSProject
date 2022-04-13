#pragma once

namespace Rudder {
    float RudderSideforceSensitivity = 0.25;
    float RudderDragSensitivity = 0.005;
    float RudderPlanformArea = 2.0;

 float SideForceValue(float RudderAngle) {
        //Assumes that all angles are expressed in radians 
        //Mach number independence is assumed
        float RudderAngleDeg = RudderAngle*(180.0/PI);
        return (RudderSideforceSensitivity*RudderAngleDeg);
    	};

    float DragValue(float RudderAngle) {
        float RudderAngleDeg = RudderAngle*(180.0/PI);
        return (RudderDragSensitivity*RudderAngleDeg);
    };
}
