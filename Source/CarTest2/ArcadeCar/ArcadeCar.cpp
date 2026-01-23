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
    // --- Body Mesh ---
    BodyMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("BodyMesh"));
    BodyMesh->SetupAttachment(GetMesh());
    BodyMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);

    // --- Wheel Meshes ---
    Wheel_FL = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Wheel_FL"));
    Wheel_FL->SetupAttachment(GetMesh());
    Wheel_FL->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Wheel_FL->SetRelativeLocation(WheelPos_FL);

    Wheel_FR = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Wheel_FR"));
    Wheel_FR->SetupAttachment(GetMesh());
    Wheel_FR->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Wheel_FR->SetRelativeLocation(WheelPos_FR);
    Wheel_FR->SetRelativeScale3D(FVector(1.f, -1.f, 1.f));

    Wheel_RL = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Wheel_RL"));
    Wheel_RL->SetupAttachment(GetMesh());
    Wheel_RL->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Wheel_RL->SetRelativeLocation(WheelPos_RL);

    Wheel_RR = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Wheel_RR"));
    Wheel_RR->SetupAttachment(GetMesh());
    Wheel_RR->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Wheel_RR->SetRelativeLocation(WheelPos_RR);
    Wheel_RR->SetRelativeScale3D(FVector(1.f, -1.f, 1.f));

    // --- Camera ---
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

    // --- Vehicle Physics ---
    UChaosWheeledVehicleMovementComponent* Vehicle = 
        CastChecked<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent());

    Vehicle->Mass = 1500.f;
    Vehicle->ChassisHeight = 140.f;
    Vehicle->DragCoefficient = 0.4f;
    Vehicle->CenterOfMassOverride = FVector(0.f, 0.f, -45.f); 
    Vehicle->bEnableCenterOfMassOverride = true;

    // Wheel Setup
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

    // Engine & Transmission
    Vehicle->EngineSetup.MaxTorque = EnginePower;
    Vehicle->EngineSetup.MaxRPM = MaxRPM;
    Vehicle->EngineSetup.EngineIdleRPM = 1000.f;
    Vehicle->EngineSetup.EngineBrakeEffect = 0.1f;
    Vehicle->EngineSetup.EngineRevUpMOI = 2.f;
    Vehicle->EngineSetup.EngineRevDownRate = 1000.f;

    Vehicle->TransmissionSetup.bUseAutomaticGears = true;
    Vehicle->TransmissionSetup.bUseAutoReverse = true;
    Vehicle->TransmissionSetup.FinalRatio = 3.5f;
    Vehicle->TransmissionSetup.ChangeUpRPM = 5500.f;
    Vehicle->TransmissionSetup.ChangeDownRPM = 2000.f;
    Vehicle->TransmissionSetup.GearChangeTime = 0.2f;

    Vehicle->DifferentialSetup.DifferentialType = EVehicleDifferential::RearWheelDrive;
    Vehicle->SteeringSetup.SteeringType = ESteeringType::AngleRatio;
    Vehicle->SteeringSetup.AngleRatio = 1.f;

    // Store base engine torque for nitro calculations
    BaseEngineTorque = EnginePower;
}

void AArcadeCar::BeginPlay()
{
    Super::BeginPlay();
    RefreshSettings();

    // Initialize nitro
    CurrentNitro = FMath::Clamp(StartingNitroAmount, 0.f, MaxNitroAmount);
    CurrentFOV = BaseFOV;

    // Store base engine torque
    if (UChaosWheeledVehicleMovementComponent* Vehicle = 
        Cast<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent()))
    {
        BaseEngineTorque = Vehicle->EngineSetup.MaxTorque;
    }

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
        SpeedKMH = Vehicle->GetForwardSpeed() * 0.036f;
        CurrentRPM = Vehicle->GetEngineRotationSpeed();
        CurrentGear = Vehicle->GetCurrentGear();

        // Check ground contact for air control
        CheckGroundContact();

        // Apply arcade physics enhancements
        if (bEnableArcadePhysics)
        {
            ApplyArcadePhysics(DeltaTime);
        }

        // Apply assist systems
        if (bEnableDriveAssist)
        {
            ApplyDriveAssist(DeltaTime);
            ApplyDriftAssist(DeltaTime);
        }

        // Update nitro system
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
    // Smooth throttle input for better feel
    SmoothedThrottleInput = FMath::FInterpTo(SmoothedThrottleInput, ThrottleInput, DeltaTime, ThrottleResponseRate);

    // Apply different systems
    ApplySpeedSensitiveSteering();
    ApplyDownforce(DeltaTime);

    if (bIsAirborne)
    {
        ApplyAirControl(DeltaTime);
    }

    // Apply launch boost for faster acceleration from standstill
    if (FMath::Abs(SpeedKMH) < LaunchBoostMaxSpeed && ThrottleInput > 0.5f && !bIsAirborne)
    {
        UChaosWheeledVehicleMovementComponent* Vehicle = 
            Cast<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent());
        
        if (Vehicle)
        {
            float SpeedRatio = FMath::Abs(SpeedKMH) / LaunchBoostMaxSpeed;
            float BoostFactor = FMath::Lerp(LaunchBoostMultiplier, 1.f, SpeedRatio);
            
            // Apply extra forward force
            UPrimitiveComponent* PhysicsRoot = Cast<UPrimitiveComponent>(GetMesh());
            if (PhysicsRoot)
            {
                FVector BoostForce = GetActorForwardVector() * (BoostFactor - 1.f) * 15000.f * ThrottleInput;
                PhysicsRoot->AddForce(BoostForce);
            }
        }
    }
}

void AArcadeCar::ApplySpeedSensitiveSteering()
{
    if (SpeedSteeringFactor <= 0.f) return;

    float AbsSpeed = FMath::Abs(SpeedKMH);
    
    // Calculate steering multiplier based on speed
    float SteeringMultiplier = 1.f;
    if (AbsSpeed > SteeringReductionStartSpeed)
    {
        // Gradually reduce steering at higher speeds
        float SpeedFactor = (AbsSpeed - SteeringReductionStartSpeed) / 100.f;
        SpeedFactor = FMath::Clamp(SpeedFactor, 0.f, 1.f);
        SteeringMultiplier = FMath::Lerp(1.f, MinSpeedSteeringMultiplier, SpeedFactor * SpeedSteeringFactor);
    }

    // Apply modified steering
    UChaosWheeledVehicleMovementComponent* Vehicle = 
        Cast<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent());
    
    if (Vehicle)
    {
        float ModifiedSteer = SteerInput * SteeringMultiplier;
        Vehicle->SetSteeringInput(ModifiedSteer);
    }
}

void AArcadeCar::ApplyDownforce(float DeltaTime)
{
    if (DownforceCoefficient <= 0.f || bIsAirborne) return;

    float AbsSpeed = FMath::Abs(SpeedKMH);
    if (AbsSpeed < DownforceStartSpeed) return;

    UPrimitiveComponent* PhysicsRoot = Cast<UPrimitiveComponent>(GetMesh());
    if (!PhysicsRoot) return;

    // Quadratic downforce based on speed
    float SpeedOver = AbsSpeed - DownforceStartSpeed;
    float DownforceAmount = DownforceCoefficient * SpeedOver * SpeedOver * 0.5f;
    
    // Apply downward force
    FVector DownforceVector = FVector(0.f, 0.f, -DownforceAmount);
    PhysicsRoot->AddForce(DownforceVector);
}

void AArcadeCar::ApplyAirControl(float DeltaTime)
{
    if (AirControlStrength <= 0.f) return;

    UPrimitiveComponent* PhysicsRoot = Cast<UPrimitiveComponent>(GetMesh());
    if (!PhysicsRoot) return;

    FVector AngularVelocity = PhysicsRoot->GetPhysicsAngularVelocityInDegrees();
    
    // Allow yaw control with steering input
    float YawControl = SteerInput * AirControlStrength * 180.f * DeltaTime;
    AngularVelocity.Z += YawControl;
    
    // Allow pitch control with throttle input
    float PitchControl = SmoothedThrottleInput * AirControlStrength * 60.f * DeltaTime;
    AngularVelocity.Y -= PitchControl;
    
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
            if (Wheel->IsInAir() == false)
            {
                WheelsOnGround++;
            }
        }
    }

    // Consider airborne if fewer than 2 wheels touching ground
    bIsAirborne = WheelsOnGround < 2;
}

// =============================================================================
// NITRO SYSTEM
// =============================================================================

void AArcadeCar::UpdateNitro(float DeltaTime)
{
    // Determine if nitro should be active
    bool bWantsNitro = bNitroInputHeld && CanActivateNitro();
    
    // Update active state with small hysteresis to prevent flickering
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
        
        // Apply boost
        ApplyNitroBoost(DeltaTime);
        
        // Smooth intensity up
        NitroIntensity = FMath::FInterpTo(NitroIntensity, 1.f, DeltaTime, 10.f);
    }
    else
    {
        // Regenerate nitro when not boosting
        float RegenAmount = NitroRegenRate;
        
        // Bonus regen while drifting
        if (CurrentSlipAngle > 15.f && !bIsAirborne)
        {
            RegenAmount += NitroDriftRegenBonus;
        }
        
        CurrentNitro += RegenAmount * DeltaTime;
        CurrentNitro = FMath::Min(CurrentNitro, MaxNitroAmount);
        
        // Smooth intensity down
        NitroIntensity = FMath::FInterpTo(NitroIntensity, 0.f, DeltaTime, 6.f);
        
        // Reset engine torque when not boosting
        UChaosWheeledVehicleMovementComponent* Vehicle = 
            Cast<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent());
        if (Vehicle)
        {
            Vehicle->EngineSetup.MaxTorque = BaseEngineTorque;
        }
    }

    // Update normalized percent for UI
    NitroPercent = (MaxNitroAmount > 0.f) ? (CurrentNitro / MaxNitroAmount) : 0.f;

    // Update visual effects
    UpdateNitroVisuals(DeltaTime);
}

void AArcadeCar::ApplyNitroBoost(float DeltaTime)
{
    UChaosWheeledVehicleMovementComponent* Vehicle = 
        Cast<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent());
    
    if (!Vehicle) return;

    // Increase engine torque
    Vehicle->EngineSetup.MaxTorque = BaseEngineTorque * NitroTorqueMultiplier;

    // Apply direct forward force
    if (NitroBoostForce > 0.f && !bIsAirborne)
    {
        UPrimitiveComponent* PhysicsRoot = Cast<UPrimitiveComponent>(GetMesh());
        if (PhysicsRoot)
        {
            // Check speed cap
            float AbsSpeed = FMath::Abs(SpeedKMH);
            float SpeedCapFactor = 1.f;
            
            if (NitroMaxSpeed > 0.f && AbsSpeed > NitroMaxSpeed * 0.9f)
            {
                // Reduce force as we approach speed cap
                SpeedCapFactor = FMath::Max(0.f, 1.f - ((AbsSpeed - NitroMaxSpeed * 0.9f) / (NitroMaxSpeed * 0.1f)));
            }
            
            FVector BoostForce = GetActorForwardVector() * NitroBoostForce * SpeedCapFactor * DeltaTime;
            PhysicsRoot->AddForce(BoostForce, NAME_None, true);
        }
    }
}

void AArcadeCar::UpdateNitroVisuals(float DeltaTime)
{
    // Update FOV based on nitro state
    float TargetFOV = BaseFOV;
    if (bIsNitroActive)
    {
        TargetFOV = BaseFOV + NitroFOVIncrease;
    }
    
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
// DRIVE ASSIST SYSTEMS
// =============================================================================

float AArcadeCar::CalculateSlipAngle(FVector& OutVelocityDir, FVector& OutForwardDir) const
{
    FVector Velocity = GetVelocity();
    FVector Forward = GetActorForwardVector();
    
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
    
    float Dot = FVector::DotProduct(OutForwardDir, OutVelocityDir);
    Dot = FMath::Clamp(Dot, -1.f, 1.f);
    float AngleRad = FMath::Acos(Dot);
    float AngleDeg = FMath::RadiansToDegrees(AngleRad);
    
    FVector Cross = FVector::CrossProduct(OutForwardDir, OutVelocityDir);
    if (Cross.Z < 0.f)
    {
        AngleDeg = -AngleDeg;
    }
    
    return AngleDeg;
}

void AArcadeCar::ApplyDriveAssist(float DeltaTime)
{
    float AbsSpeed = FMath::Abs(SpeedKMH);
    if (AbsSpeed < AssistMinSpeed)
    {
        return;
    }
    
    FVector VelocityDir, ForwardDir;
    float SlipAngle = CalculateSlipAngle(VelocityDir, ForwardDir);
    float AbsSlipAngle = FMath::Abs(SlipAngle);
    
    if (AbsSlipAngle < 5.f)
    {
        return;
    }
    
    // Only apply light straightening when not in a heavy drift
    if (AbsSlipAngle < DriftAssistStartAngle && !bIsHandbraking)
    {
        UPrimitiveComponent* PhysicsRoot = Cast<UPrimitiveComponent>(GetMesh());
        if (!PhysicsRoot) return;
        
        FVector CurrentVelocity = GetVelocity();
        float Speed = CurrentVelocity.Size();
        
        FVector TargetDir = ForwardDir;
        if (SpeedKMH < 0.f)
        {
            TargetDir = -ForwardDir;
        }
        
        float SpeedFactor = FMath::Clamp((AbsSpeed - AssistMinSpeed) / 30.f, 0.f, 1.f);
        float AngleFactor = AbsSlipAngle / DriftAssistStartAngle;
        float BlendAmount = VelocityStraightenStrength * SpeedFactor * AngleFactor * DeltaTime * 5.f;
        
        FVector NewVelocityDir = FMath::Lerp(VelocityDir, TargetDir, BlendAmount).GetSafeNormal();
        FVector NewVelocity = NewVelocityDir * Speed;
        NewVelocity.Z = CurrentVelocity.Z;
        
        PhysicsRoot->SetPhysicsLinearVelocity(NewVelocity);
    }
}

void AArcadeCar::ApplyDriftAssist(float DeltaTime)
{
    float AbsSpeed = FMath::Abs(SpeedKMH);
    if (AbsSpeed < AssistMinSpeed)
    {
        CurrentSlipAngle = 0.f;
        DriftDirection = 0.f;
        return;
    }
    
    UPrimitiveComponent* PhysicsRoot = Cast<UPrimitiveComponent>(GetMesh());
    if (!PhysicsRoot) return;
    
    FVector VelocityDir, ForwardDir;
    float SlipAngle = CalculateSlipAngle(VelocityDir, ForwardDir);
    float AbsSlipAngle = FMath::Abs(SlipAngle);
    
    CurrentSlipAngle = AbsSlipAngle;
    DriftDirection = FMath::Sign(SlipAngle);
    
    if (AbsSlipAngle < DriftAssistStartAngle)
    {
        return;
    }
    
    FVector AngularVelocity = PhysicsRoot->GetPhysicsAngularVelocityInDegrees();
    float YawRate = AngularVelocity.Z;
    
    bool bSpinningWorse = false;
    if (SlipAngle > 0.f && YawRate > 0.f)
    {
        bSpinningWorse = true;
    }
    else if (SlipAngle < 0.f && YawRate < 0.f)
    {
        bSpinningWorse = true;
    }
    
    float OverThreshold = (AbsSlipAngle - DriftAssistStartAngle) / (MaxDriftAngle - DriftAssistStartAngle);
    OverThreshold = FMath::Clamp(OverThreshold, 0.f, 1.f);
    
    bool bEmergency = AbsSlipAngle > MaxDriftAngle;
    
    // Angular Velocity Damping
    if (bSpinningWorse || bEmergency)
    {
        float DampingStrength = AngularVelocityDamping;
        
        if (bEmergency)
        {
            DampingStrength *= EmergencyDampingMultiplier;
        }
        else
        {
            DampingStrength *= (0.5f + OverThreshold * 0.5f);
        }
        
        float DampedYaw = YawRate;
        if (FMath::Abs(YawRate) > 1.f)
        {
            float Reduction = DampingStrength * DeltaTime * 60.f;
            DampedYaw = FMath::FInterpTo(YawRate, 0.f, DeltaTime, Reduction);
        }
        
        AngularVelocity.Z = DampedYaw;
        PhysicsRoot->SetPhysicsAngularVelocityInDegrees(AngularVelocity);
    }
    
    // Counter-Steer Assist
    bool bPlayerCounterSteering = false;
    if (SlipAngle > 10.f && SteerInput > 0.1f)
    {
        bPlayerCounterSteering = true;
    }
    else if (SlipAngle < -10.f && SteerInput < -0.1f)
    {
        bPlayerCounterSteering = true;
    }
    
    if (bPlayerCounterSteering && CounterSteerAssist > 0.f)
    {
        float CounterTorque = -FMath::Sign(SlipAngle) * CounterSteerAssist * OverThreshold * 50.f;
        AngularVelocity.Z += CounterTorque * DeltaTime;
        PhysicsRoot->SetPhysicsAngularVelocityInDegrees(AngularVelocity);
    }
    
    // Emergency Velocity Correction
    if (bEmergency && OverAngleVelocityCorrection > 0.f)
    {
        FVector CurrentVelocity = GetVelocity();
        float Speed = CurrentVelocity.Size();
        
        if (Speed > 100.f)
        {
            float EmergencyFactor = (AbsSlipAngle - MaxDriftAngle) / 30.f;
            EmergencyFactor = FMath::Clamp(EmergencyFactor, 0.f, 1.f);
            
            FVector TargetDir = ForwardDir;
            if (SpeedKMH < 0.f)
            {
                TargetDir = -ForwardDir;
            }
            
            float BlendAmount = OverAngleVelocityCorrection * EmergencyFactor * DeltaTime * 8.f;
            FVector NewVelocityDir = FMath::Lerp(VelocityDir, TargetDir, BlendAmount).GetSafeNormal();
            FVector NewVelocity = NewVelocityDir * Speed;
            NewVelocity.Z = CurrentVelocity.Z;
            
            PhysicsRoot->SetPhysicsLinearVelocity(NewVelocity);
        }
    }
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
    UChaosWheeledVehicleMovementComponent* Vehicle = Cast<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent());
    if (Vehicle)
    {
        if (ThrottleInput >= 0.f)
        {
            Vehicle->SetThrottleInput(ThrottleInput);
            Vehicle->SetBrakeInput(0.f);
        }
        else
        {
            Vehicle->SetThrottleInput(0.f);
            Vehicle->SetBrakeInput(FMath::Abs(ThrottleInput));
        }
    }
}

void AArcadeCar::OnSteer(const FInputActionValue& Value)
{
    SteerInput = Value.Get<float>();
    
    // If arcade physics is disabled, apply steering directly
    if (!bEnableArcadePhysics)
    {
        if (UChaosWheeledVehicleMovementComponent* Vehicle = Cast<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent()))
        {
            Vehicle->SetSteeringInput(SteerInput);
        }
    }
    // Otherwise, ApplySpeedSensitiveSteering() handles it in Tick
}

void AArcadeCar::OnHandbrakeStart(const FInputActionValue& Value)
{
    bIsHandbraking = true;
    if (UChaosWheeledVehicleMovementComponent* Vehicle = Cast<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent()))
    {
        Vehicle->SetHandbrakeInput(true);
    }
}

void AArcadeCar::OnHandbrakeEnd(const FInputActionValue& Value)
{
    bIsHandbraking = false;
    if (UChaosWheeledVehicleMovementComponent* Vehicle = Cast<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent()))
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
// CONFIGURATION & VISUALS
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
    
    if (DriftAssistStartAngle >= MaxDriftAngle)
    {
        DriftAssistStartAngle = MaxDriftAngle - 10.f;
    }

    // Update base engine torque reference
    BaseEngineTorque = EnginePower;
}

void AArcadeCar::UpdateWheelPositions()
{
    if (Wheel_FL) Wheel_FL->SetRelativeLocation(WheelPos_FL);
    if (Wheel_FR) Wheel_FR->SetRelativeLocation(WheelPos_FR);
    if (Wheel_RL) Wheel_RL->SetRelativeLocation(WheelPos_RL);
    if (Wheel_RR) Wheel_RR->SetRelativeLocation(WheelPos_RR);

    UChaosWheeledVehicleMovementComponent* Vehicle = 
        Cast<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent());
    
    if (!Vehicle) return;

    if (Vehicle->WheelSetups.Num() >= 4)
    {
        Vehicle->WheelSetups[0].AdditionalOffset = WheelPos_FL;
        Vehicle->WheelSetups[1].AdditionalOffset = WheelPos_FR;
        Vehicle->WheelSetups[2].AdditionalOffset = WheelPos_RL;
        Vehicle->WheelSetups[3].AdditionalOffset = WheelPos_RR;
    }

    Vehicle->EngineSetup.MaxTorque = EnginePower;
    Vehicle->EngineSetup.MaxRPM = MaxRPM;

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

        if (i < 2) Wheel->MaxSteerAngle = MaxSteerAngle;
    }
}

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
            FVector NewPos = BasePositions[i];
            NewPos.Z += PhysWheel->GetSuspensionOffset();
            WheelMeshes[i]->SetRelativeLocation(NewPos);

            FRotator NewRot = FRotator(PhysWheel->GetRotationAngle(), i < 2 ? PhysWheel->GetSteerAngle() : 0.f, 0.f);
            WheelMeshes[i]->SetRelativeRotation(NewRot);
        }
    }
}

void AArcadeCar::ShowDebugInfo()
{
    if (!bShowDebug || !GEngine) return;

    FString GearStr = (CurrentGear == -1) ? TEXT("R") : (CurrentGear == 0) ? TEXT("N") : FString::FromInt(CurrentGear);
    FColor StatusColor = bIsHandbraking ? FColor::Red : FColor::White;
    
    FColor SlipColor = FColor::Green;
    if (CurrentSlipAngle > MaxDriftAngle)
    {
        SlipColor = FColor::Red;
    }
    else if (CurrentSlipAngle > DriftAssistStartAngle)
    {
        SlipColor = FColor::Yellow;
    }
    
    FString DriftDirStr = (DriftDirection > 0.f) ? TEXT("RIGHT") : (DriftDirection < 0.f) ? TEXT("LEFT") : TEXT("STRAIGHT");
    FString AirborneStr = bIsAirborne ? TEXT("AIRBORNE") : TEXT("GROUNDED");
    FColor AirColor = bIsAirborne ? FColor::Cyan : FColor::Green;

    // Nitro bar visualization
    int32 NitroBarLength = 20;
    int32 FilledSegments = FMath::RoundToInt(NitroPercent * NitroBarLength);
    FString NitroBar = TEXT("[");
    for (int32 i = 0; i < NitroBarLength; i++)
    {
        NitroBar += (i < FilledSegments) ? TEXT("|") : TEXT(" ");
    }
    NitroBar += TEXT("]");
    FColor NitroColor = bIsNitroActive ? FColor::Orange : (NitroPercent > 0.3f ? FColor::Yellow : FColor::Red);

    GEngine->AddOnScreenDebugMessage(0, 0.f, FColor::White, TEXT("=== ARCADE CAR ==="));
    GEngine->AddOnScreenDebugMessage(1, 0.f, FColor::Green, FString::Printf(TEXT("Speed: %.0f km/h"), FMath::Abs(SpeedKMH)));
    GEngine->AddOnScreenDebugMessage(2, 0.f, FColor::Yellow, FString::Printf(TEXT("RPM: %.0f | Gear: %s"), CurrentRPM, *GearStr));
    GEngine->AddOnScreenDebugMessage(3, 0.f, FColor::Cyan, FString::Printf(TEXT("Throttle: %.2f | Steer: %.2f"), ThrottleInput, SteerInput));
    GEngine->AddOnScreenDebugMessage(4, 0.f, StatusColor, FString::Printf(TEXT("Handbrake: %s"), bIsHandbraking ? TEXT("ON") : TEXT("OFF")));
    GEngine->AddOnScreenDebugMessage(5, 0.f, SlipColor, FString::Printf(TEXT("Slip Angle: %.1f° | Drift: %s"), CurrentSlipAngle, *DriftDirStr));
    GEngine->AddOnScreenDebugMessage(6, 0.f, AirColor, FString::Printf(TEXT("Ground: %d wheels | %s"), WheelsOnGround, *AirborneStr));
    GEngine->AddOnScreenDebugMessage(7, 0.f, NitroColor, FString::Printf(TEXT("NITRO: %s %.0f%% %s"), *NitroBar, NitroPercent * 100.f, bIsNitroActive ? TEXT(">>> BOOST! <<<") : TEXT("")));
    GEngine->AddOnScreenDebugMessage(8, 0.f, FColor::White, FString::Printf(TEXT("Assist: %s | Arcade: %s"), 
        bEnableDriveAssist ? TEXT("ON") : TEXT("OFF"), bEnableArcadePhysics ? TEXT("ON") : TEXT("OFF")));
}
