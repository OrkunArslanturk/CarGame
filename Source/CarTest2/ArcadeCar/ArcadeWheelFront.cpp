#include "ArcadeWheelFront.h"

UArcadeWheelFront::UArcadeWheelFront()
{
    // Dimensions
    WheelRadius = 35.f;
    WheelWidth = 25.f;

    // === NFS-STYLE FRONT WHEEL ===
    // High friction to maintain steering control during drifts
    FrictionForceMultiplier = 4.0f; 
    
    // High cornering stiffness for instant steering response
    CorneringStiffness = 1000.f;
    
    // Lower thresholds for responsive feel
    SlipThreshold = 15.f;
    SkidThreshold = 15.f;

    // Suspension - slightly stiffer for responsive handling
    SuspensionMaxRaise = 10.f;
    SuspensionMaxDrop = 10.f;
    SpringRate = 180.f;
    SpringPreload = 60.f;
    WheelLoadRatio = 0.5f;

    // Steering - wider angle for deep drifts
    bAffectedBySteering = true;
    MaxSteerAngle = 50.f;

    // Brakes - strong front brakes
    MaxBrakeTorque = 4500.f;
    MaxHandBrakeTorque = 0.f;
    bAffectedByEngine = false;
    bAffectedByHandbrake = false;
    SweepShape = ESweepShape::Raycast;
}
