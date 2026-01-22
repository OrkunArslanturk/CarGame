#include "ArcadeWheelFront.h"

UArcadeWheelFront::UArcadeWheelFront()
{
    // Dimensions
    WheelRadius = 35.f;
    WheelWidth = 25.f;

    // === CONTROL TUNING ===
    // Friction: HIGH to maintain steering while drifting
    FrictionForceMultiplier = 3.5f; 
    
    // Cornering Stiffness: High to bite into the turn instantly
    CorneringStiffness = 900.f; //1200
    
    SlipThreshold = 20.f;
    SkidThreshold = 20.f;

    // Suspension
    SuspensionMaxRaise = 12.f;
    SuspensionMaxDrop = 12.f;
    SpringRate = 160.f;
    SpringPreload = 50.f;
    WheelLoadRatio = 0.5f;

    // Steering
    bAffectedBySteering = true;
    MaxSteerAngle = 45.f; // Slightly more angle for deep drifts

    // Brakes/Drive
    MaxBrakeTorque = 4000.f;
    MaxHandBrakeTorque = 0.f;
    bAffectedByEngine = false;
    bAffectedByHandbrake = false;
    SweepShape = ESweepShape::Raycast;
}
