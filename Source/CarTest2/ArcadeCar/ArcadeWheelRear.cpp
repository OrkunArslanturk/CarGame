#include "ArcadeWheelRear.h"

UArcadeWheelRear::UArcadeWheelRear()
{
    WheelRadius = 35.f;
    WheelWidth = 28.f;

    // --- Drift Tuning ---
    FrictionForceMultiplier = 2.0f; 
    
    // Stiffness: 400 was too loose, 550 is a good balance for drift vs control
    CorneringStiffness = 550.f; 
    
    SlipThreshold = 20.f;
    SkidThreshold = 20.f;

    // --- Suspension ---
    SuspensionMaxRaise = 12.f;
    SuspensionMaxDrop = 12.f;
    SpringRate = 160.f;
    SpringPreload = 50.f;
    WheelLoadRatio = 0.5f;

    // --- Drive ---
    bAffectedBySteering = false;
    MaxSteerAngle = 0.f;
    MaxBrakeTorque = 3000.f;
    MaxHandBrakeTorque = 6000.f;
    bAffectedByEngine = true;
    bAffectedByHandbrake = true;
    SweepShape = ESweepShape::Raycast;
}