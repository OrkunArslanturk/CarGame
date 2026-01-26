#include "ArcadeWheelRear.h"

UArcadeWheelRear::UArcadeWheelRear()
{
    WheelRadius = 35.f;
    WheelWidth = 28.f;

    FrictionForceMultiplier = 1.8f;
    CorneringStiffness = 400.f;
    SlipThreshold = 12.f;
    SkidThreshold = 12.f;

    SuspensionMaxRaise = 30.f;
    SuspensionMaxDrop = 30.f;
    SpringRate = 70.f;
    SpringPreload = 25.f;
    SuspensionDampingRatio = 1.3f;
    WheelLoadRatio = 0.5f;

    bAffectedBySteering = false;
    MaxSteerAngle = 0.f;
    MaxBrakeTorque = 2500.f;

    MaxHandBrakeTorque = 8000.f;
    bAffectedByEngine = true;
    bAffectedByHandbrake = true;
    SweepShape = ESweepShape::Raycast;
}
