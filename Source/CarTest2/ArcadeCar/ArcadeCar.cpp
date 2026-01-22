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
}

void AArcadeCar::BeginPlay()
{
    Super::BeginPlay();
    RefreshSettings();

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

        // Apply assist systems
        if (bEnableDriveAssist)
        {
            ApplyDriveAssist(DeltaTime);
            ApplyDriftAssist(DeltaTime);
        }
    }

    UpdateWheelVisuals();
    ShowDebugInfo();
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
    
    // Calculate signed angle
    float Dot = FVector::DotProduct(OutForwardDir, OutVelocityDir);
    Dot = FMath::Clamp(Dot, -1.f, 1.f);
    float AngleRad = FMath::Acos(Dot);
    float AngleDeg = FMath::RadiansToDegrees(AngleRad);
    
    // Determine sign (positive = drifting right, negative = drifting left)
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
    
    // Skip if already pretty straight
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
        
        // Determine target direction based on movement (forward or backward)
        FVector TargetDir = ForwardDir;
        if (SpeedKMH < 0.f)
        {
            TargetDir = -ForwardDir;
        }
        
        // Blend velocity toward target direction
        float SpeedFactor = FMath::Clamp((AbsSpeed - AssistMinSpeed) / 30.f, 0.f, 1.f);
        float AngleFactor = AbsSlipAngle / DriftAssistStartAngle;
        float BlendAmount = VelocityStraightenStrength * SpeedFactor * AngleFactor * DeltaTime * 5.f;
        
        FVector NewVelocityDir = FMath::Lerp(VelocityDir, TargetDir, BlendAmount).GetSafeNormal();
        FVector NewVelocity = NewVelocityDir * Speed;
        NewVelocity.Z = CurrentVelocity.Z; // Preserve vertical velocity
        
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
    
    // Update debug state
    CurrentSlipAngle = AbsSlipAngle;
    DriftDirection = FMath::Sign(SlipAngle);
    
    // No assist needed if angle is small
    if (AbsSlipAngle < DriftAssistStartAngle)
    {
        return;
    }
    
    // Get current angular velocity (yaw rate)
    FVector AngularVelocity = PhysicsRoot->GetPhysicsAngularVelocityInDegrees();
    float YawRate = AngularVelocity.Z;
    
    // Determine if the car is spinning further away from the velocity direction
    // (i.e., the spin is making the drift worse)
    bool bSpinningWorse = false;
    if (SlipAngle > 0.f && YawRate > 0.f) // Drifting right, spinning right = worse
    {
        bSpinningWorse = true;
    }
    else if (SlipAngle < 0.f && YawRate < 0.f) // Drifting left, spinning left = worse
    {
        bSpinningWorse = true;
    }
    
    // Calculate assist intensity based on how far over the threshold we are
    float OverThreshold = (AbsSlipAngle - DriftAssistStartAngle) / (MaxDriftAngle - DriftAssistStartAngle);
    OverThreshold = FMath::Clamp(OverThreshold, 0.f, 1.f);
    
    // Check if we're over the hard limit (emergency mode)
    bool bEmergency = AbsSlipAngle > MaxDriftAngle;
    
    // --- 1. Angular Velocity Damping ---
    // Always damp if spinning the wrong way, or if over angle threshold
    if (bSpinningWorse || bEmergency)
    {
        float DampingStrength = AngularVelocityDamping;
        
        if (bEmergency)
        {
            // Emergency: much stronger damping
            DampingStrength *= EmergencyDampingMultiplier;
        }
        else
        {
            // Scale damping by how far over threshold
            DampingStrength *= (0.5f + OverThreshold * 0.5f);
        }
        
        // Apply damping to yaw
        float DampedYaw = YawRate;
        if (FMath::Abs(YawRate) > 1.f)
        {
            float Reduction = DampingStrength * DeltaTime * 60.f; // Scale to ~60fps equivalent
            DampedYaw = FMath::FInterpTo(YawRate, 0.f, DeltaTime, Reduction);
        }
        
        AngularVelocity.Z = DampedYaw;
        PhysicsRoot->SetPhysicsAngularVelocityInDegrees(AngularVelocity);
    }
    
    // --- 2. Counter-Steer Assist ---
    // If player is steering into the drift (trying to recover), help them
    bool bPlayerCounterSteering = false;
    if (SlipAngle > 10.f && SteerInput > 0.1f)  // Drifting right, steering right
    {
        bPlayerCounterSteering = true;
    }
    else if (SlipAngle < -10.f && SteerInput < -0.1f)  // Drifting left, steering left
    {
        bPlayerCounterSteering = true;
    }
    
    if (bPlayerCounterSteering && CounterSteerAssist > 0.f)
    {
        // Add extra counter-rotation torque
        float CounterTorque = -FMath::Sign(SlipAngle) * CounterSteerAssist * OverThreshold * 50.f;
        AngularVelocity.Z += CounterTorque * DeltaTime;
        PhysicsRoot->SetPhysicsAngularVelocityInDegrees(AngularVelocity);
    }
    
    // --- 3. Emergency Velocity Correction ---
    // When over max angle, actively pull velocity toward forward direction
    if (bEmergency && OverAngleVelocityCorrection > 0.f)
    {
        FVector CurrentVelocity = GetVelocity();
        float Speed = CurrentVelocity.Size();
        
        if (Speed > 100.f) // Only if moving with reasonable speed
        {
            // Calculate how much over the limit we are
            float EmergencyFactor = (AbsSlipAngle - MaxDriftAngle) / 30.f;
            EmergencyFactor = FMath::Clamp(EmergencyFactor, 0.f, 1.f);
            
            // Determine target direction
            FVector TargetDir = ForwardDir;
            if (SpeedKMH < 0.f)
            {
                TargetDir = -ForwardDir;
            }
            
            // Aggressively blend toward forward direction
            float BlendAmount = OverAngleVelocityCorrection * EmergencyFactor * DeltaTime * 8.f;
            FVector NewVelocityDir = FMath::Lerp(VelocityDir, TargetDir, BlendAmount).GetSafeNormal();
            FVector NewVelocity = NewVelocityDir * Speed;
            NewVelocity.Z = CurrentVelocity.Z;
            
            PhysicsRoot->SetPhysicsLinearVelocity(NewVelocity);
        }
    }
}

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
    
    // Ensure DriftAssistStartAngle < MaxDriftAngle
    if (DriftAssistStartAngle >= MaxDriftAngle)
    {
        DriftAssistStartAngle = MaxDriftAngle - 10.f;
    }
}

void AArcadeCar::UpdateWheelPositions()
{
    // Update Visuals
    if (Wheel_FL) Wheel_FL->SetRelativeLocation(WheelPos_FL);
    if (Wheel_FR) Wheel_FR->SetRelativeLocation(WheelPos_FR);
    if (Wheel_RL) Wheel_RL->SetRelativeLocation(WheelPos_RL);
    if (Wheel_RR) Wheel_RR->SetRelativeLocation(WheelPos_RR);

    // Update Physics
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
    if (UChaosWheeledVehicleMovementComponent* Vehicle = Cast<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent()))
    {
        Vehicle->SetSteeringInput(SteerInput);
    }
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

void AArcadeCar::ShowDebugInfo()
{
    if (!bShowDebug || !GEngine) return;

    FString GearStr = (CurrentGear == -1) ? TEXT("R") : (CurrentGear == 0) ? TEXT("N") : FString::FromInt(CurrentGear);
    FColor StatusColor = bIsHandbraking ? FColor::Red : FColor::White;
    
    // Slip angle color coding
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

    GEngine->AddOnScreenDebugMessage(0, 0.f, FColor::White, TEXT("=== ARCADE CAR ==="));
    GEngine->AddOnScreenDebugMessage(1, 0.f, FColor::Green, FString::Printf(TEXT("Speed: %.0f km/h"), FMath::Abs(SpeedKMH)));
    GEngine->AddOnScreenDebugMessage(2, 0.f, FColor::Yellow, FString::Printf(TEXT("RPM: %.0f | Gear: %s"), CurrentRPM, *GearStr));
    GEngine->AddOnScreenDebugMessage(3, 0.f, FColor::Cyan, FString::Printf(TEXT("Throttle: %.2f | Steer: %.2f"), ThrottleInput, SteerInput));
    GEngine->AddOnScreenDebugMessage(4, 0.f, StatusColor, FString::Printf(TEXT("Handbrake: %s"), bIsHandbraking ? TEXT("ON") : TEXT("OFF")));
    GEngine->AddOnScreenDebugMessage(5, 0.f, SlipColor, FString::Printf(TEXT("Slip Angle: %.1f° | Drift: %s"), CurrentSlipAngle, *DriftDirStr));
    GEngine->AddOnScreenDebugMessage(6, 0.f, FColor::White, FString::Printf(TEXT("Assist: %s | Limits: %.0f°-%.0f°"), 
        bEnableDriveAssist ? TEXT("ON") : TEXT("OFF"), DriftAssistStartAngle, MaxDriftAngle));
}
