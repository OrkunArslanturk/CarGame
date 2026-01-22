#include "ArcadeWheelRear.h"

UArcadeWheelRear::UArcadeWheelRear()
{
    // Dimensions
    WheelRadius = 35.f;
    WheelWidth = 28.f;

    // Friction (slightly less for easier drifting)
    FrictionForceMultiplier = 2.5f;
    CorneringStiffness = 800.f;
    SlipThreshold = 20.f;
    SkidThreshold = 20.f;

    // Suspension
    SuspensionMaxRaise = 12.f;
    SuspensionMaxDrop = 12.f;
    SpringRate = 150.f;
    SpringPreload = 50.f;
    WheelLoadRatio = 0.5f;

    // Steering - Rear wheels DON'T steer
    bAffectedBySteering = false;
    MaxSteerAngle = 0.f;

    // Brakes
    MaxBrakeTorque = 3000.f;
    MaxHandBrakeTorque = 6000.f;  // Handbrake works on rear!

    // Drive - Rear wheels ARE driven (RWD)
    bAffectedByEngine = true;
    bAffectedByHandbrake = true;

    // Physics
    SweepShape = ESweepShape::Raycast;
}
