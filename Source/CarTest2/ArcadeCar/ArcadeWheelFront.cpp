#include "ArcadeWheelFront.h"

UArcadeWheelFront::UArcadeWheelFront()
{
    WheelRadius = 35.f;
    WheelWidth = 25.f;

    FrictionForceMultiplier = 4.0f;
    CorneringStiffness = 1000.f;
    SlipThreshold = 15.f;
    SkidThreshold = 15.f;

    SuspensionMaxRaise = 10.f;
    SuspensionMaxDrop = 10.f;
    SpringRate = 180.f;
    SpringPreload = 60.f;
    WheelLoadRatio = 0.5f;

    bAffectedBySteering = true;
    MaxSteerAngle = 50.f;

    MaxBrakeTorque = 4500.f;
    MaxHandBrakeTorque = 0.f;
    bAffectedByEngine = false;
    bAffectedByHandbrake = false;
    SweepShape = ESweepShape::Raycast;
}
