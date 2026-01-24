#include "ArcadeWheelRear.h"

UArcadeWheelRear::UArcadeWheelRear()
{
    WheelRadius = 35.f;
    WheelWidth = 28.f;

    // === NFS-STYLE REAR WHEEL ===
    // Lower friction to slide easily when handbrake is pulled
    FrictionForceMultiplier = 1.8f; 
    
    // Lower cornering stiffness - rear should break loose easily
    CorneringStiffness = 400.f; 
    
    // Lower thresholds for earlier sliding
    SlipThreshold = 12.f;
    SkidThreshold = 12.f;

    // Suspension - softer for weight transfer during drifts
    SuspensionMaxRaise = 12.f;
    SuspensionMaxDrop = 12.f;
    SpringRate = 140.f;
    SpringPreload = 45.f;
    WheelLoadRatio = 0.5f;

    // Drive - rear wheel drive
    bAffectedBySteering = false;
    MaxSteerAngle = 0.f;
    MaxBrakeTorque = 2500.f;
    
    // Strong handbrake for initiating drifts
    MaxHandBrakeTorque = 8000.f;
    bAffectedByEngine = true;
    bAffectedByHandbrake = true;
    SweepShape = ESweepShape::Raycast;
}