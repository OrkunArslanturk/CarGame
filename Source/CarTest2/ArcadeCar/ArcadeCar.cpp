#include "ArcadeCar.h"
#include "ArcadeWheelFront.h"
#include "ArcadeWheelRear.h"
#include "ChaosWheeledVehicleMovementComponent.h"
#include "ChaosVehicleWheel.h"
#include "GameFramework/SpringArmComponent.h"
#include "Camera/CameraComponent.h"
#include "EnhancedInputComponent.h"
#include "EnhancedInputSubsystems.h"

AArcadeCar::AArcadeCar()
{
    // Body mesh (attach car model here)
    BodyMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("BodyMesh"));
    BodyMesh->SetupAttachment(GetMesh());
    BodyMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);

    // Wheel meshes (visual only, physics handled by Chaos)
    Wheel_FL = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Wheel_FL"));
    Wheel_FL->SetupAttachment(GetMesh());
    Wheel_FL->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Wheel_FL->SetRelativeLocation(WheelPos_FL);

    Wheel_FR = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Wheel_FR"));
    Wheel_FR->SetupAttachment(GetMesh());
    Wheel_FR->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Wheel_FR->SetRelativeLocation(WheelPos_FR);
    Wheel_FR->SetRelativeScale3D(FVector(1.f, -1.f, 1.f)); // Mirror for right side

    Wheel_RL = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Wheel_RL"));
    Wheel_RL->SetupAttachment(GetMesh());
    Wheel_RL->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Wheel_RL->SetRelativeLocation(WheelPos_RL);

    Wheel_RR = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Wheel_RR"));
    Wheel_RR->SetupAttachment(GetMesh());
    Wheel_RR->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Wheel_RR->SetRelativeLocation(WheelPos_RR);
    Wheel_RR->SetRelativeScale3D(FVector(1.f, -1.f, 1.f));

    // Camera setup with lag for smooth follow
    CameraArm = CreateDefaultSubobject<USpringArmComponent>(TEXT("CameraArm"));
    CameraArm->SetupAttachment(RootComponent);
    CameraArm->TargetArmLength = CameraDistance;
    CameraArm->SetRelativeLocation(FVector(0.f, 0.f, CameraHeight));
    CameraArm->SetRelativeRotation(FRotator(-15.f, 0.f, 0.f));
    CameraArm->bEnableCameraLag = true;
    CameraArm->bEnableCameraRotationLag = true;
    CameraArm->CameraLagSpeed = CameraLag;
    CameraArm->CameraRotationLagSpeed = CameraLag;
    CameraArm->bInheritPitch = false;
    CameraArm->bInheritRoll = false;

    Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));
    Camera->SetupAttachment(CameraArm);
    Camera->SetFieldOfView(BaseFOV);

    // Chaos Vehicle setup
    UChaosWheeledVehicleMovementComponent* Vehicle = 
        CastChecked<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent());

    // Chassis
    Vehicle->Mass = 1500.f;
    Vehicle->ChassisHeight = 140.f;
    Vehicle->DragCoefficient = 0.4f;
    Vehicle->CenterOfMassOverride = FVector(0.f, 0.f, -45.f); // Lower for stability
    Vehicle->bEnableCenterOfMassOverride = true;

    // Wheel configuration
    Vehicle->bLegacyWheelFrictionPosition = false;
    Vehicle->WheelSetups.SetNum(4);

    Vehicle->WheelSetups[0].WheelClass = UArcadeWheelFront::StaticClass();
    Vehicle->WheelSetups[0].AdditionalOffset = WheelPos_FL;

    Vehicle->WheelSetups[1].WheelClass = UArcadeWheelFront::StaticClass();
    Vehicle->WheelSetups[1].AdditionalOffset = WheelPos_FR;

    Vehicle->WheelSetups[2].WheelClass = UArcadeWheelRear::StaticClass();
    Vehicle->WheelSetups[2].AdditionalOffset = WheelPos_RL;

    Vehicle->WheelSetups[3].WheelClass = UArcadeWheelRear::StaticClass();
    Vehicle->WheelSetups[3].AdditionalOffset = WheelPos_RR;

    // Engine
    Vehicle->EngineSetup.MaxTorque = EnginePower;
    Vehicle->EngineSetup.MaxRPM = MaxRPM;
    Vehicle->EngineSetup.EngineIdleRPM = 1000.f;
    Vehicle->EngineSetup.EngineBrakeEffect = 0.1f;
    Vehicle->EngineSetup.EngineRevUpMOI = 2.f;
    Vehicle->EngineSetup.EngineRevDownRate = 1000.f;

    // Transmission
    Vehicle->TransmissionSetup.bUseAutomaticGears = true;
    Vehicle->TransmissionSetup.bUseAutoReverse = true;
    Vehicle->TransmissionSetup.FinalRatio = 3.5f;
    Vehicle->TransmissionSetup.ChangeUpRPM = 5500.f;
    Vehicle->TransmissionSetup.ChangeDownRPM = 2000.f;
    Vehicle->TransmissionSetup.GearChangeTime = 0.2f;

    // RWD setup
    Vehicle->DifferentialSetup.DifferentialType = EVehicleDifferential::RearWheelDrive;
    Vehicle->SteeringSetup.SteeringType = ESteeringType::AngleRatio;
    Vehicle->SteeringSetup.AngleRatio = 1.f;

    BaseEngineTorque = EnginePower;
}

void AArcadeCar::BeginPlay()
{
    Super::BeginPlay();
    RefreshSettings();

    // Initialize nitro
    CurrentNitro = FMath::Clamp(StartingNitroAmount, 0.f, MaxNitroAmount);
    CurrentFOV = BaseFOV;

    if (UChaosWheeledVehicleMovementComponent* Vehicle = 
        Cast<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent()))
    {
        BaseEngineTorque = Vehicle->EngineSetup.MaxTorque;
    }

    // Setup input mapping context
    if (APlayerController* PC = Cast<APlayerController>(GetController()))
    {
        if (UEnhancedInputLocalPlayerSubsystem* Subsystem =
            ULocalPlayer::GetSubsystem<UEnhancedInputLocalPlayerSubsystem>(PC->GetLocalPlayer()))
        {
            if (InputMapping)
            {
                Subsystem->AddMappingContext(InputMapping, 0);
            }
        }
    }
}

void AArcadeCar::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    UChaosWheeledVehicleMovementComponent* Vehicle = 
        Cast<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent());

    if (Vehicle)
    {
        // Update state
        SpeedKMH = Vehicle->GetForwardSpeed() * 0.036f; // cm/s to km/h
        CurrentRPM = Vehicle->GetEngineRotationSpeed();
        CurrentGear = Vehicle->GetCurrentGear();

        CheckGroundContact();
        UpdateDynamicGrip();

        // Drift mode: active when handbraking at speed
        if (bIsHandbraking && FMath::Abs(SpeedKMH) > 15.f)
        {
            if (bEnableDriftAssist)
            {
                ApplyDriftPhysics(DeltaTime);
            }
        }
        // Normal driving: stability control
        else if (bEnableStabilityControl)
        {
            ApplyStabilityControl(DeltaTime);
            TimeDrifting = 0.f;
            DriftAngleMomentum = FMath::FInterpTo(DriftAngleMomentum, 0.f, DeltaTime, 3.f);
        }

        if (bEnableArcadePhysics)
        {
            ApplyArcadePhysics(DeltaTime);
        }

        if (bEnableNitro)
        {
            UpdateNitro(DeltaTime);
        }
    }

    UpdateWheelVisuals();
    ShowDebugInfo();
}

// =============================================================================
// ARCADE PHYSICS
// =============================================================================

void AArcadeCar::ApplyArcadePhysics(float DeltaTime)
{
    SmoothedThrottleInput = FMath::FInterpTo(SmoothedThrottleInput, ThrottleInput, DeltaTime, ThrottleResponseRate);

    ApplySpeedSensitiveSteering();
    ApplyDownforce(DeltaTime);

    if (bIsAirborne)
    {
        ApplyAirControl(DeltaTime);
    }

    // Launch boost: extra acceleration from standstill
    if (FMath::Abs(SpeedKMH) < LaunchBoostMaxSpeed && ThrottleInput > 0.5f && !bIsAirborne)
    {
        UPrimitiveComponent* PhysicsRoot = Cast<UPrimitiveComponent>(GetMesh());
        if (PhysicsRoot)
        {
            float SpeedRatio = FMath::Abs(SpeedKMH) / LaunchBoostMaxSpeed;
            float BoostFactor = FMath::Lerp(LaunchBoostMultiplier, 1.f, SpeedRatio);
            FVector BoostForce = GetActorForwardVector() * (BoostFactor - 1.f) * 15000.f * ThrottleInput;
            PhysicsRoot->AddForce(BoostForce);
        }
    }
}

void AArcadeCar::ApplySpeedSensitiveSteering()
{
    if (SpeedSteeringFactor <= 0.f) return;

    float AbsSpeed = FMath::Abs(SpeedKMH);
    float SteeringMultiplier = 1.f;
    
    // Reduce steering at high speed for stability
    if (AbsSpeed > SteeringReductionStartSpeed)
    {
        float SpeedFactor = (AbsSpeed - SteeringReductionStartSpeed) / 100.f;
        SpeedFactor = FMath::Clamp(SpeedFactor, 0.f, 1.f);
        SteeringMultiplier = FMath::Lerp(1.f, MinSpeedSteeringMultiplier, SpeedFactor * SpeedSteeringFactor);
    }

    UChaosWheeledVehicleMovementComponent* Vehicle = 
        Cast<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent());
    
    if (Vehicle)
    {
        float ModifiedSteer = SteerInput * SteeringMultiplier;
        if (bIsDrifting)
        {
            ModifiedSteer *= DriftSteeringMultiplier;
        }
        Vehicle->SetSteeringInput(FMath::Clamp(ModifiedSteer, -1.f, 1.f));
    }
}

void AArcadeCar::ApplyDownforce(float DeltaTime)
{
    if (DownforceCoefficient <= 0.f || bIsAirborne) return;

    float AbsSpeed = FMath::Abs(SpeedKMH);
    if (AbsSpeed < DownforceStartSpeed) return;

    UPrimitiveComponent* PhysicsRoot = Cast<UPrimitiveComponent>(GetMesh());
    if (!PhysicsRoot) return;

    // Quadratic downforce for realistic feel
    float SpeedOver = AbsSpeed - DownforceStartSpeed;
    float DownforceAmount = DownforceCoefficient * SpeedOver * SpeedOver * 0.5f;
    PhysicsRoot->AddForce(FVector(0.f, 0.f, -DownforceAmount));
}

void AArcadeCar::ApplyAirControl(float DeltaTime)
{
    if (AirControlStrength <= 0.f) return;

    UPrimitiveComponent* PhysicsRoot = Cast<UPrimitiveComponent>(GetMesh());
    if (!PhysicsRoot) return;

    FVector AngularVelocity = PhysicsRoot->GetPhysicsAngularVelocityInDegrees();
    
    // Yaw with steering, pitch with throttle
    AngularVelocity.Z += SteerInput * AirControlStrength * 180.f * DeltaTime;
    AngularVelocity.Y -= SmoothedThrottleInput * AirControlStrength * 60.f * DeltaTime;
    
    PhysicsRoot->SetPhysicsAngularVelocityInDegrees(AngularVelocity);
}

void AArcadeCar::CheckGroundContact()
{
    UChaosWheeledVehicleMovementComponent* Vehicle = 
        Cast<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent());
    
    if (!Vehicle) return;

    WheelsOnGround = 0;
    for (int32 i = 0; i < Vehicle->Wheels.Num(); i++)
    {
        if (UChaosVehicleWheel* Wheel = Vehicle->Wheels[i])
        {
            if (!Wheel->IsInAir())
            {
                WheelsOnGround++;
            }
        }
    }
    
    // Airborne if fewer than 2 wheels on ground
    bIsAirborne = WheelsOnGround < 2;
}

// =============================================================================
// NITRO SYSTEM
// =============================================================================

void AArcadeCar::UpdateNitro(float DeltaTime)
{
    bool bWantsNitro = bNitroInputHeld && CanActivateNitro();
    
    if (bWantsNitro && !bIsNitroActive)
    {
        bIsNitroActive = true;
    }
    else if (!bWantsNitro || CurrentNitro <= 0.f)
    {
        bIsNitroActive = false;
    }

    if (bIsNitroActive)
    {
        // Drain nitro
        CurrentNitro -= NitroDrainRate * DeltaTime;
        CurrentNitro = FMath::Max(CurrentNitro, 0.f);
        ApplyNitroBoost(DeltaTime);
        NitroIntensity = FMath::FInterpTo(NitroIntensity, 1.f, DeltaTime, 10.f);
    }
    else
    {
        // Regenerate nitro (bonus while drifting)
        float RegenAmount = NitroRegenRate;
        if (bIsDrifting && CurrentSlipAngle > 15.f)
        {
            RegenAmount += NitroDriftRegenBonus;
        }
        
        CurrentNitro += RegenAmount * DeltaTime;
        CurrentNitro = FMath::Min(CurrentNitro, MaxNitroAmount);
        NitroIntensity = FMath::FInterpTo(NitroIntensity, 0.f, DeltaTime, 6.f);
        
        // Reset torque when not boosting
        UChaosWheeledVehicleMovementComponent* Vehicle = 
            Cast<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent());
        if (Vehicle)
        {
            Vehicle->EngineSetup.MaxTorque = BaseEngineTorque;
        }
    }

    NitroPercent = (MaxNitroAmount > 0.f) ? (CurrentNitro / MaxNitroAmount) : 0.f;
    UpdateNitroVisuals(DeltaTime);
}

void AArcadeCar::ApplyNitroBoost(float DeltaTime)
{
    UChaosWheeledVehicleMovementComponent* Vehicle = 
        Cast<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent());
    
    if (!Vehicle) return;

    // Boost engine torque
    Vehicle->EngineSetup.MaxTorque = BaseEngineTorque * NitroTorqueMultiplier;

    // Apply direct forward force
    if (NitroBoostForce > 0.f && !bIsAirborne)
    {
        UPrimitiveComponent* PhysicsRoot = Cast<UPrimitiveComponent>(GetMesh());
        if (PhysicsRoot)
        {
            // Reduce force near speed cap
            float AbsSpeed = FMath::Abs(SpeedKMH);
            float SpeedCapFactor = 1.f;
            
            if (NitroMaxSpeed > 0.f && AbsSpeed > NitroMaxSpeed * 0.9f)
            {
                SpeedCapFactor = FMath::Max(0.f, 1.f - ((AbsSpeed - NitroMaxSpeed * 0.9f) / (NitroMaxSpeed * 0.1f)));
            }
            
            FVector BoostForce = GetActorForwardVector() * NitroBoostForce * SpeedCapFactor * DeltaTime;
            PhysicsRoot->AddForce(BoostForce, NAME_None, true);
        }
    }
}

void AArcadeCar::UpdateNitroVisuals(float DeltaTime)
{
    // FOV increase during boost
    float TargetFOV = bIsNitroActive ? (BaseFOV + NitroFOVIncrease) : BaseFOV;
    CurrentFOV = FMath::FInterpTo(CurrentFOV, TargetFOV, DeltaTime, NitroFOVLerpSpeed);
    
    if (Camera)
    {
        Camera->SetFieldOfView(CurrentFOV);
    }
}

bool AArcadeCar::CanActivateNitro() const
{
    return CurrentNitro >= NitroMinToActivate && ThrottleInput > 0.1f;
}

void AArcadeCar::AddNitro(float Amount)
{
    CurrentNitro = FMath::Clamp(CurrentNitro + Amount, 0.f, MaxNitroAmount);
}

void AArcadeCar::RefillNitro()
{
    CurrentNitro = MaxNitroAmount;
}

// =============================================================================
// GRIP SYSTEM
// =============================================================================

void AArcadeCar::UpdateDynamicGrip()
{
    UChaosWheeledVehicleMovementComponent* Vehicle = 
        Cast<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent());
    
    if (!Vehicle) return;

    // Rear wheels: low grip for drift, high grip for stability
    for (int32 i = 2; i < Vehicle->Wheels.Num(); i++)
    {
        if (UChaosVehicleWheel* Wheel = Vehicle->Wheels[i])
        {
            if (bIsHandbraking)
            {
                Wheel->FrictionForceMultiplier = TireGrip * DriftRearGripMultiplier;
            }
            else if (bEnableStabilityControl)
            {
                Wheel->FrictionForceMultiplier = TireGrip * StabilityGripMultiplier;
            }
            else
            {
                Wheel->FrictionForceMultiplier = TireGrip;
            }
        }
    }
    
    // Front wheels: high grip for steering control
    for (int32 i = 0; i < 2 && i < Vehicle->Wheels.Num(); i++)
    {
        if (UChaosVehicleWheel* Wheel = Vehicle->Wheels[i])
        {
            if (bIsHandbraking)
            {
                Wheel->FrictionForceMultiplier = TireGrip * DriftFrontGripMultiplier;
            }
            else if (bEnableStabilityControl)
            {
                Wheel->FrictionForceMultiplier = TireGrip * StabilityGripMultiplier * 1.2f;
            }
            else
            {
                Wheel->FrictionForceMultiplier = TireGrip;
            }
        }
    }
}

// =============================================================================
// STABILITY CONTROL (Normal Driving)
// =============================================================================

void AArcadeCar::ApplyStabilityControl(float DeltaTime)
{
    float AbsSpeed = FMath::Abs(SpeedKMH);
    
    FVector VelocityDir, ForwardDir;
    float SlipAngle = CalculateSlipAngle(VelocityDir, ForwardDir);
    float AbsSlipAngle = FMath::Abs(SlipAngle);
    
    CurrentSlipAngle = AbsSlipAngle;
    DriftDirection = FMath::Sign(SlipAngle);
    bIsDrifting = false;
    DriftIntensity = 0.f;
    
    // Reduce stability at low speed for maneuverability
    float SpeedFactor = FMath::Clamp((AbsSpeed - 5.f) / (StabilityMinSpeed - 5.f), 0.f, 1.f);
    if (SpeedFactor <= 0.f || bIsAirborne) return;

    UPrimitiveComponent* PhysicsRoot = Cast<UPrimitiveComponent>(GetMesh());
    if (!PhysicsRoot) return;

    FVector AngularVelocity = PhysicsRoot->GetPhysicsAngularVelocityInDegrees();
    float YawRate = AngularVelocity.Z;
    float AbsYawRate = FMath::Abs(YawRate);

    // Limit yaw rate to prevent spin-outs
    if (AbsYawRate > MaxYawRateWithoutHandbrake)
    {
        float ClampedYaw = FMath::Sign(YawRate) * MaxYawRateWithoutHandbrake;
        AngularVelocity.Z = FMath::Lerp(YawRate, ClampedYaw, YawDampingStrength * SpeedFactor);
        PhysicsRoot->SetPhysicsAngularVelocityInDegrees(AngularVelocity);
    }
    
    // Dampen yaw when not steering
    if (FMath::Abs(SteerInput) < 0.3f && AbsYawRate > 20.f)
    {
        AngularVelocity.Z = FMath::FInterpTo(YawRate, 0.f, DeltaTime, YawDampingStrength * SpeedFactor * 10.f);
        PhysicsRoot->SetPhysicsAngularVelocityInDegrees(AngularVelocity);
    }

    // Align velocity with forward direction
    if (AbsSlipAngle > StabilitySlipThreshold)
    {
        FVector CurrentVelocity = GetVelocity();
        float Speed = CurrentVelocity.Size();
        
        if (Speed > 100.f)
        {
            FVector TargetDir = (SpeedKMH < 0.f) ? -ForwardDir : ForwardDir;
            float OverThreshold = FMath::Clamp((AbsSlipAngle - StabilitySlipThreshold) / 20.f, 0.f, 1.f);
            float BlendAmount = StabilityStrength * SpeedFactor * OverThreshold * DeltaTime * 15.f;
            
            FVector NewVelocityDir = FMath::Lerp(VelocityDir, TargetDir, BlendAmount).GetSafeNormal();
            FVector NewVelocity = NewVelocityDir * Speed;
            NewVelocity.Z = CurrentVelocity.Z;
            PhysicsRoot->SetPhysicsLinearVelocity(NewVelocity);
        }
    }

    // Counter-rotate when spinning away from velocity
    bool bRotatingAway = (SlipAngle > 2.f && YawRate > 5.f) || (SlipAngle < -2.f && YawRate < -5.f);
    if (bRotatingAway)
    {
        float CounterTorque = -FMath::Sign(YawRate) * StabilityStrength * SpeedFactor * 100.f * DeltaTime;
        AngularVelocity.Z += CounterTorque;
        PhysicsRoot->SetPhysicsAngularVelocityInDegrees(AngularVelocity);
    }
}

// =============================================================================
// DRIFT PHYSICS
// =============================================================================

void AArcadeCar::ApplyDriftPhysics(float DeltaTime)
{
    float AbsSpeed = FMath::Abs(SpeedKMH);
    
    FVector VelocityDir, ForwardDir;
    float SlipAngle = CalculateSlipAngle(VelocityDir, ForwardDir);
    float AbsSlipAngle = FMath::Abs(SlipAngle);
    
    CurrentSlipAngle = AbsSlipAngle;
    DriftDirection = FMath::Sign(SlipAngle);
    
    bIsDrifting = AbsSlipAngle > 10.f;
    DriftIntensity = FMath::Clamp(AbsSlipAngle / OptimalDriftAngle, 0.f, 1.5f);
    
    if (bIsDrifting)
    {
        TimeDrifting += DeltaTime;
    }

    UPrimitiveComponent* PhysicsRoot = Cast<UPrimitiveComponent>(GetMesh());
    if (!PhysicsRoot) return;

    FVector AngularVelocity = PhysicsRoot->GetPhysicsAngularVelocityInDegrees();
    float YawRate = AngularVelocity.Z;

    // Throttle controls drift angle: gas = wider, brake = tighter
    float TargetDriftAngle = OptimalDriftAngle;
    if (ThrottleInput > 0.1f)
    {
        float ThrottleEffect = ThrottleInput * ThrottleDriftControl;
        TargetDriftAngle = FMath::Lerp(OptimalDriftAngle * 0.7f, MaxDriftAngle * 0.8f, ThrottleEffect);
    }
    else if (ThrottleInput < -0.1f)
    {
        TargetDriftAngle = OptimalDriftAngle * 0.5f;
    }

    // Steering adjusts drift angle
    float SteerEffect = SteerInput * DriftDirection;
    if (SteerEffect > 0.1f)
    {
        TargetDriftAngle *= (1.f + SteerEffect * 0.3f);
    }
    else if (SteerEffect < -0.1f)
    {
        TargetDriftAngle *= (1.f + SteerEffect * 0.5f);
    }

    TargetDriftAngle = FMath::Clamp(TargetDriftAngle, 15.f, MaxDriftAngle);

    // Smooth drift angle changes
    float AngleDiff = TargetDriftAngle - AbsSlipAngle;
    DriftAngleMomentum = FMath::FInterpTo(DriftAngleMomentum, AngleDiff, DeltaTime, DriftResponsiveness);

    float DesiredYawChange = DriftAngleMomentum * DriftDirection * DeltaTime * 60.f;
    
    // Counter-steer assistance
    bool bCounterSteering = (DriftDirection > 0.f && SteerInput > 0.2f) || 
                            (DriftDirection < 0.f && SteerInput < -0.2f);
    
    if (bCounterSteering)
    {
        float CounterStrength = FMath::Abs(SteerInput) * 40.f * DeltaTime;
        DesiredYawChange -= DriftDirection * CounterStrength;
    }

    // Anti-spin protection
    float MaxYawRate = 120.f;
    if (AbsSlipAngle > MaxDriftAngle * 0.9f)
    {
        MaxYawRate = 60.f;
        
        // Emergency correction when over limit
        if (AbsSlipAngle > MaxDriftAngle)
        {
            float EmergencyDamp = FMath::Clamp((AbsSlipAngle - MaxDriftAngle) / 20.f, 0.f, 1.f);
            DesiredYawChange *= (1.f - EmergencyDamp * 0.8f);
            
            // Pull velocity toward forward
            FVector CurrentVelocity = GetVelocity();
            float Speed = CurrentVelocity.Size();
            if (Speed > 100.f)
            {
                float BlendAmount = EmergencyDamp * DeltaTime * 10.f;
                FVector NewVelocityDir = FMath::Lerp(VelocityDir, ForwardDir, BlendAmount).GetSafeNormal();
                FVector NewVelocity = NewVelocityDir * Speed;
                NewVelocity.Z = CurrentVelocity.Z;
                PhysicsRoot->SetPhysicsLinearVelocity(NewVelocity);
            }
        }
    }

    float NewYawRate = FMath::Clamp(YawRate + DesiredYawChange, -MaxYawRate, MaxYawRate);
    
    // Dampen if spinning makes drift worse
    bool bSpinningWorse = (SlipAngle > 0.f && YawRate > 10.f) || (SlipAngle < 0.f && YawRate < -10.f);
    if (bSpinningWorse && AbsSlipAngle > OptimalDriftAngle)
    {
        float DampFactor = FMath::Clamp((AbsSlipAngle - OptimalDriftAngle) / 30.f, 0.f, 0.8f);
        NewYawRate = FMath::Lerp(NewYawRate, 0.f, DampFactor * DeltaTime * 5.f);
    }

    AngularVelocity.Z = NewYawRate;
    PhysicsRoot->SetPhysicsAngularVelocityInDegrees(AngularVelocity);

    // Drift momentum: maintain forward speed while sliding
    FVector CurrentVelocity = GetVelocity();
    float Speed = CurrentVelocity.Size();
    
    if (Speed > 100.f && ThrottleInput > 0.f)
    {
        float MomentumFactor = DriftMomentumRetention * ThrottleInput;
        FVector TargetDir = FMath::Lerp(VelocityDir, ForwardDir, MomentumFactor * DeltaTime * 3.f).GetSafeNormal();
        FVector NewVelocity = TargetDir * Speed;
        NewVelocity.Z = CurrentVelocity.Z;
        PhysicsRoot->SetPhysicsLinearVelocity(NewVelocity);
    }

    // Drift boost: reward for maintaining optimal angle
    if (bIsDrifting && DriftBoostForce > 0.f && ThrottleInput > 0.3f)
    {
        float DriftQuality = 1.f - FMath::Abs(AbsSlipAngle - OptimalDriftAngle) / OptimalDriftAngle;
        DriftQuality = FMath::Clamp(DriftQuality, 0.f, 1.f);
        
        float TimeBonus = FMath::Clamp(TimeDrifting / 2.f, 0.f, 1.f);
        float BoostAmount = DriftBoostForce * DriftQuality * TimeBonus * ThrottleInput * DeltaTime;
        
        PhysicsRoot->AddForce(ForwardDir * BoostAmount);
    }
}

float AArcadeCar::CalculateSlipAngle(FVector& OutVelocityDir, FVector& OutForwardDir) const
{
    FVector Velocity = GetVelocity();
    FVector Forward = GetActorForwardVector();
    
    // Flatten to 2D
    Velocity.Z = 0.f;
    Forward.Z = 0.f;
    
    float Speed = Velocity.Size();
    if (Speed < 1.f)
    {
        OutVelocityDir = Forward;
        OutForwardDir = Forward;
        return 0.f;
    }
    
    OutVelocityDir = Velocity / Speed;
    OutForwardDir = Forward.GetSafeNormal();
    
    float Dot = FMath::Clamp(FVector::DotProduct(OutForwardDir, OutVelocityDir), -1.f, 1.f);
    float AngleDeg = FMath::RadiansToDegrees(FMath::Acos(Dot));
    
    // Sign: positive = drifting right, negative = left
    FVector Cross = FVector::CrossProduct(OutForwardDir, OutVelocityDir);
    return (Cross.Z < 0.f) ? -AngleDeg : AngleDeg;
}

// =============================================================================
// INPUT HANDLING
// =============================================================================

void AArcadeCar::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
    Super::SetupPlayerInputComponent(PlayerInputComponent);

    if (UEnhancedInputComponent* Input = Cast<UEnhancedInputComponent>(PlayerInputComponent))
    {
        if (Input_Throttle)
        {
            Input->BindAction(Input_Throttle, ETriggerEvent::Triggered, this, &AArcadeCar::OnThrottle);
            Input->BindAction(Input_Throttle, ETriggerEvent::Completed, this, &AArcadeCar::OnThrottle);
        }
        if (Input_Brake)
        {
            Input->BindAction(Input_Brake, ETriggerEvent::Triggered, this, &AArcadeCar::OnBrake);
            Input->BindAction(Input_Brake, ETriggerEvent::Completed, this, &AArcadeCar::OnBrake);
        }
        if (Input_Steer)
        {
            Input->BindAction(Input_Steer, ETriggerEvent::Triggered, this, &AArcadeCar::OnSteer);
            Input->BindAction(Input_Steer, ETriggerEvent::Completed, this, &AArcadeCar::OnSteer);
        }
        if (Input_Handbrake)
        {
            Input->BindAction(Input_Handbrake, ETriggerEvent::Started, this, &AArcadeCar::OnHandbrakeStart);
            Input->BindAction(Input_Handbrake, ETriggerEvent::Completed, this, &AArcadeCar::OnHandbrakeEnd);
        }
        if (Input_Nitro)
        {
            Input->BindAction(Input_Nitro, ETriggerEvent::Started, this, &AArcadeCar::OnNitroStart);
            Input->BindAction(Input_Nitro, ETriggerEvent::Completed, this, &AArcadeCar::OnNitroEnd);
        }
    }
}

void AArcadeCar::OnThrottle(const FInputActionValue& Value)
{
    ThrottleInput = Value.Get<float>();
    
    UChaosWheeledVehicleMovementComponent* Vehicle = 
        Cast<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent());
    
    if (!Vehicle) return;

    // W key: forward acceleration only
    if (ThrottleInput > 0.f)
    {
        bWantsToReverse = false;
        Vehicle->SetThrottleInput(ThrottleInput);
        
        // Only clear brake if not pressing brake key
        if (BrakeInput <= 0.f)
        {
            Vehicle->SetBrakeInput(0.f);
        }
    }
    else
    {
        // No throttle input
        if (BrakeInput <= 0.f)
        {
            Vehicle->SetThrottleInput(0.f);
        }
    }
}

void AArcadeCar::OnBrake(const FInputActionValue& Value)
{
    BrakeInput = Value.Get<float>();
    
    UChaosWheeledVehicleMovementComponent* Vehicle = 
        Cast<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent());
    
    if (!Vehicle) return;

    // S key: brake when moving forward, reverse when stopped
    if (BrakeInput > 0.f)
    {
        // Going forward: brake
        if (SpeedKMH > ReverseThresholdSpeed)
        {
            bWantsToReverse = false;
            Vehicle->SetThrottleInput(0.f);
            Vehicle->SetBrakeInput(BrakeInput);
        }
        // Stopped or nearly stopped: engage reverse
        else if (SpeedKMH < ReverseThresholdSpeed && SpeedKMH > -ReverseThresholdSpeed)
        {
            bWantsToReverse = true;
            Vehicle->SetBrakeInput(0.f);
            Vehicle->SetThrottleInput(-BrakeInput);
        }
        // Already reversing: continue reverse
        else
        {
            bWantsToReverse = true;
            Vehicle->SetBrakeInput(0.f);
            Vehicle->SetThrottleInput(-BrakeInput);
        }
    }
    else
    {
        // No brake input
        bWantsToReverse = false;
        if (ThrottleInput <= 0.f)
        {
            Vehicle->SetBrakeInput(0.f);
            Vehicle->SetThrottleInput(0.f);
        }
    }
}

void AArcadeCar::OnSteer(const FInputActionValue& Value)
{
    SteerInput = Value.Get<float>();
    
    // Direct steering when arcade physics disabled
    if (!bEnableArcadePhysics)
    {
        if (UChaosWheeledVehicleMovementComponent* Vehicle = 
            Cast<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent()))
        {
            Vehicle->SetSteeringInput(SteerInput);
        }
    }
}

void AArcadeCar::OnHandbrakeStart(const FInputActionValue& Value)
{
    bIsHandbraking = true;
    if (UChaosWheeledVehicleMovementComponent* Vehicle = 
        Cast<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent()))
    {
        Vehicle->SetHandbrakeInput(true);
    }
}

void AArcadeCar::OnHandbrakeEnd(const FInputActionValue& Value)
{
    bIsHandbraking = false;
    if (UChaosWheeledVehicleMovementComponent* Vehicle = 
        Cast<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent()))
    {
        Vehicle->SetHandbrakeInput(false);
    }
}

void AArcadeCar::OnNitroStart(const FInputActionValue& Value)
{
    bNitroInputHeld = true;
}

void AArcadeCar::OnNitroEnd(const FInputActionValue& Value)
{
    bNitroInputHeld = false;
}

// =============================================================================
// CONFIGURATION
// =============================================================================

#if WITH_EDITOR
void AArcadeCar::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
    Super::PostEditChangeProperty(PropertyChangedEvent);
    RefreshSettings();
}
#endif

void AArcadeCar::RefreshSettings()
{
    UpdateWheelPositions();

    if (CameraArm)
    {
        CameraArm->TargetArmLength = CameraDistance;
        CameraArm->SetRelativeLocation(FVector(0.f, 0.f, CameraHeight));
        CameraArm->CameraLagSpeed = CameraLag;
        CameraArm->CameraRotationLagSpeed = CameraLag;
    }
    
    if (Camera)
    {
        Camera->SetFieldOfView(BaseFOV);
        CurrentFOV = BaseFOV;
    }

    BaseEngineTorque = EnginePower;
}

void AArcadeCar::UpdateWheelPositions()
{
    // Update visual positions
    if (Wheel_FL) Wheel_FL->SetRelativeLocation(WheelPos_FL);
    if (Wheel_FR) Wheel_FR->SetRelativeLocation(WheelPos_FR);
    if (Wheel_RL) Wheel_RL->SetRelativeLocation(WheelPos_RL);
    if (Wheel_RR) Wheel_RR->SetRelativeLocation(WheelPos_RR);

    UChaosWheeledVehicleMovementComponent* Vehicle = 
        Cast<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent());
    
    if (!Vehicle) return;

    // Update physics wheel offsets
    if (Vehicle->WheelSetups.Num() >= 4)
    {
        Vehicle->WheelSetups[0].AdditionalOffset = WheelPos_FL;
        Vehicle->WheelSetups[1].AdditionalOffset = WheelPos_FR;
        Vehicle->WheelSetups[2].AdditionalOffset = WheelPos_RL;
        Vehicle->WheelSetups[3].AdditionalOffset = WheelPos_RR;
    }

    Vehicle->EngineSetup.MaxTorque = EnginePower;
    Vehicle->EngineSetup.MaxRPM = MaxRPM;

    // Apply tuning to all wheels
    for (int32 i = 0; i < Vehicle->Wheels.Num(); i++)
    {
        UChaosVehicleWheel* Wheel = Vehicle->Wheels[i];
        if (!Wheel) continue;

        Wheel->WheelRadius = WheelRadius;
        Wheel->WheelWidth = WheelWidth;
        Wheel->FrictionForceMultiplier = TireGrip;
        Wheel->SuspensionMaxRaise = SuspensionTravel;
        Wheel->SuspensionMaxDrop = SuspensionTravel;
        Wheel->SpringRate = SuspensionStiffness;

        // Front wheels steer
        if (i < 2) Wheel->MaxSteerAngle = MaxSteerAngle;
    }
}

// =============================================================================
// VISUALS
// =============================================================================

void AArcadeCar::UpdateWheelVisuals()
{
    UChaosWheeledVehicleMovementComponent* Vehicle = 
        Cast<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent());

    if (!Vehicle) return;

    TArray<UStaticMeshComponent*> WheelMeshes = { Wheel_FL, Wheel_FR, Wheel_RL, Wheel_RR };
    TArray<FVector> BasePositions = { WheelPos_FL, WheelPos_FR, WheelPos_RL, WheelPos_RR };

    for (int32 i = 0; i < Vehicle->Wheels.Num() && i < WheelMeshes.Num(); i++)
    {
        if (UChaosVehicleWheel* PhysWheel = Vehicle->Wheels[i])
        {
            // Position with suspension offset
            FVector NewPos = BasePositions[i];
            NewPos.Z += PhysWheel->GetSuspensionOffset();
            WheelMeshes[i]->SetRelativeLocation(NewPos);

            // Rotation (spin + steer for front wheels)
            FRotator NewRot = FRotator(PhysWheel->GetRotationAngle(), i < 2 ? PhysWheel->GetSteerAngle() : 0.f, 0.f);
            WheelMeshes[i]->SetRelativeRotation(NewRot);
        }
    }
}

void AArcadeCar::ShowDebugInfo()
{
    if (!bShowDebug || !GEngine) return;

    FString GearStr = (CurrentGear == -1) ? TEXT("R") : (CurrentGear == 0) ? TEXT("N") : FString::FromInt(CurrentGear);
    
    // Color based on drift quality
    FColor DriftColor = FColor::Green;
    if (bIsDrifting)
    {
        float AngleRatio = CurrentSlipAngle / OptimalDriftAngle;
        if (AngleRatio > 1.3f) DriftColor = FColor::Red;
        else if (AngleRatio > 0.7f) DriftColor = FColor::Orange;
        else DriftColor = FColor::Yellow;
    }
    
    FString ModeStr = bIsHandbraking ? TEXT("DRIFT") : TEXT("GRIP");
    FColor ModeColor = bIsHandbraking ? FColor::Orange : FColor::Green;
    FString DriftDirStr = (DriftDirection > 0.f) ? TEXT("R") : (DriftDirection < 0.f) ? TEXT("L") : TEXT("-");

    // Nitro bar
    int32 NitroBarLen = 20;
    int32 Filled = FMath::RoundToInt(NitroPercent * NitroBarLen);
    FString NitroBar = TEXT("[");
    for (int32 i = 0; i < NitroBarLen; i++) NitroBar += (i < Filled) ? TEXT("|") : TEXT(" ");
    NitroBar += TEXT("]");
    FColor NitroColor = bIsNitroActive ? FColor::Orange : (NitroPercent > 0.3f ? FColor::Yellow : FColor::Red);

    GEngine->AddOnScreenDebugMessage(0, 0.f, FColor::White, TEXT("=== ARCADE CAR ==="));
    GEngine->AddOnScreenDebugMessage(1, 0.f, FColor::Green, FString::Printf(TEXT("Speed: %.0f km/h | Gear: %s"), FMath::Abs(SpeedKMH), *GearStr));
    GEngine->AddOnScreenDebugMessage(2, 0.f, FColor::Cyan, FString::Printf(TEXT("Throttle: %.2f | Brake: %.2f | Steer: %.2f"), ThrottleInput, BrakeInput, SteerInput));
    GEngine->AddOnScreenDebugMessage(3, 0.f, ModeColor, FString::Printf(TEXT("Mode: %s | Reverse: %s | Handbrake: %s"), *ModeStr, bWantsToReverse ? TEXT("Y") : TEXT("N"), bIsHandbraking ? TEXT("ON") : TEXT("OFF")));
    GEngine->AddOnScreenDebugMessage(4, 0.f, DriftColor, FString::Printf(TEXT("Slip: %.1f° [%s] | Optimal: %.0f°"), CurrentSlipAngle, *DriftDirStr, OptimalDriftAngle));
    GEngine->AddOnScreenDebugMessage(5, 0.f, bIsDrifting ? FColor::Yellow : FColor::White, FString::Printf(TEXT("Drifting: %s | Time: %.1fs"), bIsDrifting ? TEXT("YES") : TEXT("NO"), TimeDrifting));
    GEngine->AddOnScreenDebugMessage(6, 0.f, NitroColor, FString::Printf(TEXT("NITRO %s %.0f%% %s"), *NitroBar, NitroPercent * 100.f, bIsNitroActive ? TEXT("BOOST!") : TEXT("")));
}
