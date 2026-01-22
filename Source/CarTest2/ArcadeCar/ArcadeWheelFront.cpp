#include "ArcadeWheelFront.h"

UArcadeWheelFront::UArcadeWheelFront()
{
    // Dimensions
    WheelRadius = 35.f;
    WheelWidth = 25.f;

    // Friction
    FrictionForceMultiplier = 3.f;
    CorneringStiffness = 1000.f;
    SlipThreshold = 20.f;
    SkidThreshold = 20.f;

    // Suspension
    SuspensionMaxRaise = 12.f;
    SuspensionMaxDrop = 12.f;
    SpringRate = 150.f;
    SpringPreload = 50.f;
    WheelLoadRatio = 0.5f;

    // Steering - Front wheels CAN steer
    bAffectedBySteering = true;
    MaxSteerAngle = 40.f;

    // Brakes
    MaxBrakeTorque = 4000.f;
    MaxHandBrakeTorque = 0.f;  // No handbrake on front

    // Drive - Not driven (RWD car)
    bAffectedByEngine = false;
    bAffectedByHandbrake = false;

    // Physics
    SweepShape = ESweepShape::Raycast;
}
