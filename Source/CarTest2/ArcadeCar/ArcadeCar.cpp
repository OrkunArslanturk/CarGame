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
    BodyMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("BodyMesh"));
    BodyMesh->SetupAttachment(GetMesh());
    BodyMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);

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

    UChaosWheeledVehicleMovementComponent* Vehicle =
        CastChecked<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent());

    Vehicle->Mass = 2000.f;
    Vehicle->InertiaTensorScale = FVector(2.5f, 2.5f, 2.5f);
    Vehicle->DragCoefficient = 0.35f;
    Vehicle->ChassisHeight = 140.f;
    Vehicle->CenterOfMassOverride = FVector(0.f, 0.f, -65.f);
    Vehicle->bEnableCenterOfMassOverride = true;
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

    Vehicle->EngineSetup.MaxTorque = EnginePower;
    Vehicle->EngineSetup.MaxRPM = MaxRPM;
    Vehicle->EngineSetup.EngineIdleRPM = 1000.f;
    Vehicle->EngineSetup.EngineBrakeEffect = 0.5f;
    Vehicle->EngineSetup.EngineRevUpMOI = 2.f;
    Vehicle->EngineSetup.EngineRevDownRate = 1000.f;

    Vehicle->TransmissionSetup.bUseAutomaticGears = true;
    Vehicle->TransmissionSetup.bUseAutoReverse = true;
    Vehicle->TransmissionSetup.FinalRatio = 2.8f;
    Vehicle->TransmissionSetup.ChangeUpRPM = 7500.f;
    Vehicle->TransmissionSetup.ChangeDownRPM = 2500.f;
    Vehicle->TransmissionSetup.GearChangeTime = 0.15f;

    Vehicle->DifferentialSetup.DifferentialType = EVehicleDifferential::RearWheelDrive;
    Vehicle->SteeringSetup.SteeringType = ESteeringType::AngleRatio;
    Vehicle->SteeringSetup.AngleRatio = 1.f;

    BaseEngineTorque = EnginePower;
    DriftSideFriction = 5.0f;
    NormalSideFriction = 40.0f;
}

void AArcadeCar::BeginPlay()
{
    Super::BeginPlay();
    RefreshSettings();

    CurrentNitro = FMath::Clamp(StartingNitroAmount, 0.f, MaxNitroAmount);
    CurrentFOV = BaseFOV;

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

        CheckGroundContact();
        UpdateDynamicGrip();

        if (bEnableArcadePhysics && !bIsAirborne)
        {
            ApplyImphenziaPhysics(DeltaTime);
            ApplyArcadePhysics(DeltaTime);
        }

        if (bEnableNitro)
        {
            UpdateNitro(DeltaTime);
        }
    }

    UpdateWheelVisuals();
    UpdateCameraEffects(DeltaTime);
    ShowDebugInfo();
}

void AArcadeCar::ApplyImphenziaPhysics(float DeltaTime)
{
    UPrimitiveComponent* PhysicsRoot = Cast<UPrimitiveComponent>(GetMesh());
    if (!PhysicsRoot) return;

    FVector Velocity = PhysicsRoot->GetPhysicsLinearVelocity();
    FVector RightVec = GetActorRightVector();
    FVector ForwardVec = GetActorForwardVector();

    FVector FlatRightVec = FVector(RightVec.X, RightVec.Y, 0.f).GetSafeNormal();
    FVector FlatForwardVec = FVector(ForwardVec.X, ForwardVec.Y, 0.f).GetSafeNormal();
    if (FlatRightVec.IsNearlyZero()) FlatRightVec = RightVec;
    if (FlatForwardVec.IsNearlyZero()) FlatForwardVec = ForwardVec;

    float SideSpeed = FVector::DotProduct(Velocity, FlatRightVec);
    float ForwardSpeed = FVector::DotProduct(Velocity, FlatForwardVec);
    float AbsSpeed = FMath::Abs(SpeedKMH);

    FVector VelocityDir, ForwardDir;
    CurrentSlipAngle = CalculateSlipAngle(VelocityDir, ForwardDir);
    float AbsSlip = FMath::Abs(CurrentSlipAngle);

    bool bWasDrifting = bIsDrifting;

    if (!bIsDrifting)
    {
        if ((bIsHandbraking && AbsSpeed > 30.f) || AbsSlip > 25.f)
        {
            bIsDrifting = true;
            DriftDirection = FMath::Sign(CurrentSlipAngle);
        }
    }
    else
    {
        if (AbsSlip < 8.f && !bIsHandbraking)
        {
            bIsDrifting = false;
        }
        if (AbsSpeed < 15.f)
        {
            bIsDrifting = false;
        }
    }

    float FrictionAmount;

    if (bIsHandbraking)
    {
        FrictionAmount = DriftSideFriction * 0.3f;
    }
    else if (bIsDrifting)
    {
        float ThrottleDriftBonus = FMath::Lerp(1.0f, 0.4f, SmoothedThrottleInput);
        FrictionAmount = DriftSideFriction * ThrottleDriftBonus;

        bool bCounterSteering = (DriftDirection > 0.f && SteerInput < -0.3f) ||
                                (DriftDirection < 0.f && SteerInput > 0.3f);
        if (bCounterSteering)
        {
            FrictionAmount *= 1.5f;
        }
    }
    else
    {
        FrictionAmount = NormalSideFriction;
    }

    float CounterAccel = -SideSpeed * FrictionAmount;
    float MaxGripAccel = 980.f * 4.0f;
    CounterAccel = FMath::Clamp(CounterAccel, -MaxGripAccel, MaxGripAccel);

    PhysicsRoot->AddForce(FlatRightVec * CounterAccel, NAME_None, true);

    if (bIsDrifting && ThrottleInput > 0.f && ForwardSpeed > 0.f)
    {
        float SpeedRetention = 0.3f;
        FVector MomentumBoost = FlatForwardVec * SpeedRetention * 5000.f * SmoothedThrottleInput;
        PhysicsRoot->AddForce(MomentumBoost, NAME_None, true);
    }

    FVector AngularVel = PhysicsRoot->GetPhysicsAngularVelocityInDegrees();
    float CurrentYawRate = AngularVel.Z;
    float TargetYawRate = SteerInput * MaxYawRotationSpeed;

    float SpeedSteeringScale = FMath::GetMappedRangeValueClamped(
        FVector2D(50.f, 200.f),
        FVector2D(1.f, 0.5f),
        AbsSpeed
    );

    if (!bIsDrifting)
    {
        TargetYawRate *= SpeedSteeringScale;
    }
    else
    {
        TargetYawRate *= 1.4f;
        float DriftMomentum = DriftDirection * AbsSlip * 0.5f;
        TargetYawRate += DriftMomentum;
    }

    float SteeringResponse = bIsDrifting ? 12.f : 15.f;
    float NextYawRate = FMath::FInterpTo(CurrentYawRate, TargetYawRate, DeltaTime, SteeringResponse);

    if (FMath::IsNearlyZero(SteerInput) && !bIsDrifting)
    {
        NextYawRate = FMath::FInterpTo(NextYawRate, 0.f, DeltaTime, AngularDamping);
    }

    PhysicsRoot->SetPhysicsAngularVelocityInDegrees(FVector(AngularVel.X, AngularVel.Y, NextYawRate));

    if (bIsDrifting)
    {
        DriftTime += DeltaTime;
        float ScoreRate = FMath::GetMappedRangeValueClamped(
            FVector2D(10.f, 90.f),
            FVector2D(0.f, 1.f),
            AbsSlip
        );
        DriftScore = FMath::Clamp(DriftScore + (ScoreRate * DeltaTime * 0.3f), 0.f, 1.f);
        DriftIntensity = FMath::GetMappedRangeValueClamped(
            FVector2D(15.f, 60.f),
            FVector2D(0.2f, 1.f),
            AbsSlip
        );
    }
    else
    {
        DriftTime = 0.f;
        DriftScore = FMath::FInterpTo(DriftScore, 0.f, DeltaTime, 1.f);
        DriftIntensity = FMath::FInterpTo(DriftIntensity, 0.f, DeltaTime, 3.f);
    }

    DriftDirection = FMath::Sign(CurrentSlipAngle);
}

void AArcadeCar::ApplyArcadePhysics(float DeltaTime)
{
    SmoothedThrottleInput = FMath::FInterpTo(SmoothedThrottleInput, ThrottleInput, DeltaTime, ThrottleResponseRate);

    ApplyDownforce(DeltaTime);

    if (bIsAirborne)
    {
        UPrimitiveComponent* PhysicsRoot = Cast<UPrimitiveComponent>(GetMesh());
        if (PhysicsRoot)
        {
            FVector ExtraGravity = FVector(0, 0, -980.f * 2.0f) * PhysicsRoot->GetMass();
            PhysicsRoot->AddForce(ExtraGravity);
            ApplyAirControl(DeltaTime);
        }
    }

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
}

void AArcadeCar::ApplyDownforce(float DeltaTime)
{
    if (DownforceCoefficient <= 0.f || bIsAirborne) return;

    float AbsSpeed = FMath::Abs(SpeedKMH);
    if (AbsSpeed < DownforceStartSpeed) return;

    UPrimitiveComponent* PhysicsRoot = Cast<UPrimitiveComponent>(GetMesh());
    if (!PhysicsRoot) return;

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
    bIsAirborne = WheelsOnGround < 2;
}

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
        CurrentNitro -= NitroDrainRate * DeltaTime;
        CurrentNitro = FMath::Max(CurrentNitro, 0.f);
        ApplyNitroBoost(DeltaTime);
        NitroIntensity = FMath::FInterpTo(NitroIntensity, 1.f, DeltaTime, 10.f);
    }
    else
    {
        float RegenAmount = NitroRegenRate;
        if (bIsDrifting && FMath::Abs(CurrentSlipAngle) > 15.f)
        {
            RegenAmount += NitroDriftRegenBonus * DriftScore;
        }

        CurrentNitro += RegenAmount * DeltaTime;
        CurrentNitro = FMath::Min(CurrentNitro, MaxNitroAmount);
        NitroIntensity = FMath::FInterpTo(NitroIntensity, 0.f, DeltaTime, 6.f);

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

    Vehicle->EngineSetup.MaxTorque = BaseEngineTorque * NitroTorqueMultiplier;

    if (NitroBoostForce > 0.f && !bIsAirborne)
    {
        UPrimitiveComponent* PhysicsRoot = Cast<UPrimitiveComponent>(GetMesh());
        if (PhysicsRoot)
        {
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
    float TargetFOV = bIsNitroActive ? (BaseFOV + NitroFOVIncrease) : BaseFOV;
    CurrentFOV = FMath::FInterpTo(CurrentFOV, TargetFOV, DeltaTime, NitroFOVLerpSpeed);

    if (Camera)
    {
        Camera->SetFieldOfView(CurrentFOV);
    }
}

void AArcadeCar::UpdateCameraEffects(float DeltaTime)
{
    if (!CameraArm) return;

    float TargetYawOffset = 0.f;
    if (bIsDrifting)
    {
        float SlipRatio = FMath::Clamp(CurrentSlipAngle / 45.f, -1.f, 1.f);
        TargetYawOffset = SlipRatio * CameraDriftSwingAngle;
    }

    FRotator CurrentRot = CameraArm->GetRelativeRotation();
    float NewYaw = FMath::FInterpTo(CurrentRot.Yaw, TargetYawOffset, DeltaTime, CameraSwingSpeed);
    CameraArm->SetRelativeRotation(FRotator(-15.f, NewYaw, 0.f));
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

void AArcadeCar::UpdateDynamicGrip()
{
    UChaosWheeledVehicleMovementComponent* Vehicle =
        Cast<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent());

    if (!Vehicle) return;

    float FrontGrip = 3.0f;
    float RearGrip = 2.5f;

    if (bIsHandbraking)
    {
        FrontGrip = 2.5f;
        RearGrip = 0.1f;
    }
    else if (bIsDrifting)
    {
        FrontGrip = 2.0f;
        RearGrip = 0.5f;
    }

    for (int32 i = 0; i < Vehicle->Wheels.Num(); i++)
    {
        if (UChaosVehicleWheel* Wheel = Vehicle->Wheels[i])
        {
            Wheel->FrictionForceMultiplier = (i < 2) ? FrontGrip : RearGrip;
        }
    }
}

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

    float Dot = FMath::Clamp(FVector::DotProduct(OutForwardDir, OutVelocityDir), -1.f, 1.f);
    float AngleDeg = FMath::RadiansToDegrees(FMath::Acos(Dot));

    FVector Cross = FVector::CrossProduct(OutForwardDir, OutVelocityDir);
    return (Cross.Z < 0.f) ? -AngleDeg : AngleDeg;
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

    if (ThrottleInput > 0.f)
    {
        bWantsToReverse = false;
        Vehicle->SetThrottleInput(ThrottleInput);

        if (BrakeInput <= 0.f)
        {
            Vehicle->SetBrakeInput(0.f);
        }
    }
    else
    {
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

    if (BrakeInput > 0.f)
    {
        if (SpeedKMH > ReverseThresholdSpeed)
        {
            bWantsToReverse = false;
            Vehicle->SetThrottleInput(0.f);
            Vehicle->SetBrakeInput(BrakeInput);
        }
        else if (SpeedKMH < ReverseThresholdSpeed && SpeedKMH > -ReverseThresholdSpeed)
        {
            bWantsToReverse = true;
            Vehicle->SetBrakeInput(0.f);
            Vehicle->SetThrottleInput(-BrakeInput);
        }
        else
        {
            bWantsToReverse = true;
            Vehicle->SetBrakeInput(0.f);
            Vehicle->SetThrottleInput(-BrakeInput);
        }
    }
    else
    {
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

    if (UChaosWheeledVehicleMovementComponent* Vehicle =
        Cast<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent()))
    {
        Vehicle->SetSteeringInput(SteerInput);
    }
}

void AArcadeCar::OnHandbrakeStart(const FInputActionValue& Value)
{
    bIsHandbraking = true;

    UChaosWheeledVehicleMovementComponent* Vehicle =
        Cast<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent());

    if (Vehicle)
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

            float SteerAngle = i < 2 ? PhysWheel->GetSteerAngle() : 0.f;

            if (i < 2 && bIsDrifting)
            {
                float VisualCounterSteer = CurrentSlipAngle * 1.5f;
                SteerAngle -= VisualCounterSteer;
                SteerAngle = FMath::Clamp(SteerAngle, -MaxSteerAngle, MaxSteerAngle);
            }

            FRotator NewRot = FRotator(PhysWheel->GetRotationAngle(), SteerAngle, 0.f);
            WheelMeshes[i]->SetRelativeRotation(NewRot);
        }
    }
}

void AArcadeCar::ShowDebugInfo()
{
    if (!bShowDebug || !GEngine) return;

    FString GearStr = (CurrentGear == -1) ? TEXT("R") : (CurrentGear == 0) ? TEXT("N") : FString::FromInt(CurrentGear);

    FColor DriftColor = FColor::White;
    if (bIsDrifting)
    {
        if (DriftScore > 0.7f) DriftColor = FColor::Green;
        else if (DriftScore > 0.4f) DriftColor = FColor::Yellow;
        else if (DriftScore > 0.2f) DriftColor = FColor::Orange;
        else DriftColor = FColor::Red;
    }

    FString ModeStr = bIsHandbraking ? TEXT("DRIFT") : bIsDrifting ? TEXT("SLIDE") : TEXT("GRIP");
    FColor ModeColor = bIsHandbraking ? FColor::Orange : bIsDrifting ? FColor::Yellow : FColor::Green;
    FString DriftDirStr = (DriftDirection > 0.f) ? TEXT("R") : (DriftDirection < 0.f) ? TEXT("L") : TEXT("-");

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
    GEngine->AddOnScreenDebugMessage(4, 0.f, DriftColor, FString::Printf(TEXT("Slip: %.1f° [%s]"), CurrentSlipAngle, *DriftDirStr));
    GEngine->AddOnScreenDebugMessage(5, 0.f, bIsDrifting ? FColor::Green : FColor::White, FString::Printf(TEXT("IS DRIFTING: %s"), bIsDrifting ? TEXT("YES") : TEXT("NO")));
    GEngine->AddOnScreenDebugMessage(6, 0.f, NitroColor, FString::Printf(TEXT("NITRO %s %.0f%% %s"), *NitroBar, NitroPercent * 100.f, bIsNitroActive ? TEXT("BOOST!") : TEXT("")));
}
