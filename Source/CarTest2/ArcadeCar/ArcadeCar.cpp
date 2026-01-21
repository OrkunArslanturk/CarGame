// ArcadeCar.cpp
// Arcade-style car with Enhanced Input System
// For AGP Racing Game Assignment - Futuregames 2026

#include "ArcadeCar.h"
#include "EnhancedInputComponent.h"
#include "EnhancedInputSubsystems.h"
#include "InputAction.h"
#include "InputMappingContext.h"
#include "DrawDebugHelpers.h"
#include "Engine/World.h"

// ============================================================================
// CONSTRUCTOR
// ============================================================================

AArcadeCar::AArcadeCar()
{
    PrimaryActorTick.bCanEverTick = true;

    // ========== CREATE CAR BODY ==========
    CarBody = CreateDefaultSubobject<UBoxComponent>(TEXT("CarBody"));
    CarBody->SetBoxExtent(BodyExtent);
    CarBody->SetSimulatePhysics(true);
    CarBody->SetCollisionProfileName(TEXT("PhysicsActor"));
    CarBody->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
    CarBody->SetNotifyRigidBodyCollision(true);
    CarBody->BodyInstance.bUseCCD = true;
    CarBody->SetMassOverrideInKg(NAME_None, CarMass);
    CarBody->SetLinearDamping(0.1f);
    CarBody->SetAngularDamping(0.8f);
    CarBody->SetCenterOfMass(FVector(0.f, 0.f, -20.f));
    RootComponent = CarBody;

    // ========== CREATE BODY MESH ==========
    BodyMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("BodyMesh"));
    BodyMesh->SetupAttachment(CarBody);
    BodyMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    BodyMesh->SetRelativeLocation(FVector::ZeroVector);

    // ========== CREATE WHEEL SOCKETS ==========
    WheelSocket_FrontLeft = CreateDefaultSubobject<USceneComponent>(TEXT("WheelSocket_FrontLeft"));
    WheelSocket_FrontLeft->SetupAttachment(CarBody);
    WheelSocket_FrontLeft->SetRelativeLocation(FVector(120.f, -80.f, -20.f));

    WheelSocket_FrontRight = CreateDefaultSubobject<USceneComponent>(TEXT("WheelSocket_FrontRight"));
    WheelSocket_FrontRight->SetupAttachment(CarBody);
    WheelSocket_FrontRight->SetRelativeLocation(FVector(120.f, 80.f, -20.f));

    WheelSocket_RearLeft = CreateDefaultSubobject<USceneComponent>(TEXT("WheelSocket_RearLeft"));
    WheelSocket_RearLeft->SetupAttachment(CarBody);
    WheelSocket_RearLeft->SetRelativeLocation(FVector(-120.f, -80.f, -20.f));

    WheelSocket_RearRight = CreateDefaultSubobject<USceneComponent>(TEXT("WheelSocket_RearRight"));
    WheelSocket_RearRight->SetupAttachment(CarBody);
    WheelSocket_RearRight->SetRelativeLocation(FVector(-120.f, 80.f, -20.f));

    // ========== CREATE WHEEL MESHES ==========
    WheelMesh_FrontLeft = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("WheelMesh_FrontLeft"));
    WheelMesh_FrontLeft->SetupAttachment(WheelSocket_FrontLeft);
    WheelMesh_FrontLeft->SetCollisionEnabled(ECollisionEnabled::NoCollision);

    WheelMesh_FrontRight = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("WheelMesh_FrontRight"));
    WheelMesh_FrontRight->SetupAttachment(WheelSocket_FrontRight);
    WheelMesh_FrontRight->SetCollisionEnabled(ECollisionEnabled::NoCollision);

    WheelMesh_RearLeft = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("WheelMesh_RearLeft"));
    WheelMesh_RearLeft->SetupAttachment(WheelSocket_RearLeft);
    WheelMesh_RearLeft->SetCollisionEnabled(ECollisionEnabled::NoCollision);

    WheelMesh_RearRight = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("WheelMesh_RearRight"));
    WheelMesh_RearRight->SetupAttachment(WheelSocket_RearRight);
    WheelMesh_RearRight->SetCollisionEnabled(ECollisionEnabled::NoCollision);

    // ========== CREATE CAMERA ==========
    CameraArm = CreateDefaultSubobject<USpringArmComponent>(TEXT("CameraArm"));
    CameraArm->SetupAttachment(CarBody);
    CameraArm->TargetArmLength = CameraDistance;
    CameraArm->SetRelativeRotation(FRotator(CameraPitch, 0.f, 0.f));
    CameraArm->bDoCollisionTest = true;
    CameraArm->bEnableCameraLag = true;
    CameraArm->bEnableCameraRotationLag = true;
    CameraArm->CameraLagSpeed = CameraLagSpeed;
    CameraArm->CameraRotationLagSpeed = 5.f;

    Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));
    Camera->SetupAttachment(CameraArm);

    // ========== DEFAULT GEAR RATIOS ==========
    GearRatios.Empty();
    GearRatios.Add(-3.0f);  // Reverse
    GearRatios.Add(0.0f);   // Neutral
    GearRatios.Add(3.5f);   // 1st
    GearRatios.Add(2.5f);   // 2nd
    GearRatios.Add(1.8f);   // 3rd
    GearRatios.Add(1.3f);   // 4th
    GearRatios.Add(1.0f);   // 5th

    WheelDataArray.SetNum(4);
}

// ============================================================================
// BEGIN PLAY
// ============================================================================

void AArcadeCar::BeginPlay()
{
    Super::BeginPlay();

    // Apply physics settings
    CarBody->SetBoxExtent(BodyExtent);
    CarBody->SetMassOverrideInKg(NAME_None, CarMass);

    // Apply camera settings
    CameraArm->TargetArmLength = CameraDistance;
    CameraArm->SetRelativeRotation(FRotator(CameraPitch, 0.f, 0.f));
    CameraArm->CameraLagSpeed = CameraLagSpeed;

    // Cache wheel references
    CacheWheelReferences();

    // Initialize wheel data
    WheelDataArray.SetNum(4);
    for (int32 i = 0; i < 4; i++)
    {
        WheelDataArray[i].SpringLength = SuspensionRestLength;
    }

    // ========== ADD INPUT MAPPING CONTEXT ==========
    if (APlayerController* PlayerController = Cast<APlayerController>(GetController()))
    {
        if (UEnhancedInputLocalPlayerSubsystem* Subsystem = 
            ULocalPlayer::GetSubsystem<UEnhancedInputLocalPlayerSubsystem>(PlayerController->GetLocalPlayer()))
        {
            if (CarMappingContext)
            {
                Subsystem->AddMappingContext(CarMappingContext, 0);
                UE_LOG(LogTemp, Log, TEXT("Added Car Input Mapping Context"));
            }
            else
            {
                UE_LOG(LogTemp, Warning, TEXT("CarMappingContext is not set! Assign it in Blueprint."));
            }
        }
    }

    UE_LOG(LogTemp, Log, TEXT("ArcadeCar initialized with Enhanced Input"));
}

// ============================================================================
// WHEEL CACHING
// ============================================================================

void AArcadeCar::CacheWheelReferences()
{
    WheelSocketArray.Empty();
    WheelSocketArray.Add(WheelSocket_FrontLeft);
    WheelSocketArray.Add(WheelSocket_FrontRight);
    WheelSocketArray.Add(WheelSocket_RearLeft);
    WheelSocketArray.Add(WheelSocket_RearRight);

    WheelMeshArray.Empty();
    WheelMeshArray.Add(WheelMesh_FrontLeft);
    WheelMeshArray.Add(WheelMesh_FrontRight);
    WheelMeshArray.Add(WheelMesh_RearLeft);
    WheelMeshArray.Add(WheelMesh_RearRight);

    RefreshWheelPositions();
}

void AArcadeCar::RefreshWheelPositions()
{
    WheelOffsets.Empty();

    if (WheelSocket_FrontLeft)
        WheelOffsets.Add(WheelSocket_FrontLeft->GetRelativeLocation());
    else
        WheelOffsets.Add(FVector(120.f, -80.f, -20.f));

    if (WheelSocket_FrontRight)
        WheelOffsets.Add(WheelSocket_FrontRight->GetRelativeLocation());
    else
        WheelOffsets.Add(FVector(120.f, 80.f, -20.f));

    if (WheelSocket_RearLeft)
        WheelOffsets.Add(WheelSocket_RearLeft->GetRelativeLocation());
    else
        WheelOffsets.Add(FVector(-120.f, -80.f, -20.f));

    if (WheelSocket_RearRight)
        WheelOffsets.Add(WheelSocket_RearRight->GetRelativeLocation());
    else
        WheelOffsets.Add(FVector(-120.f, 80.f, -20.f));
}

// ============================================================================
// EDITOR SUPPORT
// ============================================================================

#if WITH_EDITOR
void AArcadeCar::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
    Super::PostEditChangeProperty(PropertyChangedEvent);
    RefreshWheelPositions();

    if (CarBody)
    {
        CarBody->SetBoxExtent(BodyExtent);
        CarBody->SetMassOverrideInKg(NAME_None, CarMass);
    }

    if (CameraArm)
    {
        CameraArm->TargetArmLength = CameraDistance;
        CameraArm->SetRelativeRotation(FRotator(CameraPitch, 0.f, 0.f));
        CameraArm->CameraLagSpeed = CameraLagSpeed;
    }
}

void AArcadeCar::PostEditMove(bool bFinished)
{
    Super::PostEditMove(bFinished);
    RefreshWheelPositions();
}
#endif

// ============================================================================
// ENHANCED INPUT SETUP
// ============================================================================

void AArcadeCar::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
    Super::SetupPlayerInputComponent(PlayerInputComponent);

    // Cast to Enhanced Input Component
    if (UEnhancedInputComponent* EnhancedInput = Cast<UEnhancedInputComponent>(PlayerInputComponent))
    {
        // Bind Throttle (Axis1D)
        if (ThrottleAction)
        {
            EnhancedInput->BindAction(ThrottleAction, ETriggerEvent::Triggered, this, &AArcadeCar::HandleThrottleInput);
            EnhancedInput->BindAction(ThrottleAction, ETriggerEvent::Completed, this, &AArcadeCar::HandleThrottleInput);
        }

        // Bind Steer (Axis1D)
        if (SteerAction)
        {
            EnhancedInput->BindAction(SteerAction, ETriggerEvent::Triggered, this, &AArcadeCar::HandleSteerInput);
            EnhancedInput->BindAction(SteerAction, ETriggerEvent::Completed, this, &AArcadeCar::HandleSteerInput);
        }

        // Bind Handbrake (Boolean)
        if (HandbrakeAction)
        {
            EnhancedInput->BindAction(HandbrakeAction, ETriggerEvent::Started, this, &AArcadeCar::HandleHandbrakeStarted);
            EnhancedInput->BindAction(HandbrakeAction, ETriggerEvent::Completed, this, &AArcadeCar::HandleHandbrakeCompleted);
        }

        // Bind Shift Up (Boolean)
        if (ShiftUpAction)
        {
            EnhancedInput->BindAction(ShiftUpAction, ETriggerEvent::Started, this, &AArcadeCar::HandleShiftUp);
        }

        // Bind Shift Down (Boolean)
        if (ShiftDownAction)
        {
            EnhancedInput->BindAction(ShiftDownAction, ETriggerEvent::Started, this, &AArcadeCar::HandleShiftDown);
        }

        UE_LOG(LogTemp, Log, TEXT("Enhanced Input bindings complete"));
    }
    else
    {
        UE_LOG(LogTemp, Error, TEXT("Failed to cast to EnhancedInputComponent! Make sure Enhanced Input is enabled."));
    }
}

// ============================================================================
// ENHANCED INPUT HANDLERS
// ============================================================================

void AArcadeCar::HandleThrottleInput(const FInputActionValue& Value)
{
    // Get Axis1D value (-1 to 1)
    ThrottleInput = Value.Get<float>();
}

void AArcadeCar::HandleSteerInput(const FInputActionValue& Value)
{
    // Get Axis1D value (-1 to 1)
    SteerInput = Value.Get<float>();
}

void AArcadeCar::HandleHandbrakeStarted(const FInputActionValue& Value)
{
    bHandbrakePressed = true;
}

void AArcadeCar::HandleHandbrakeCompleted(const FInputActionValue& Value)
{
    bHandbrakePressed = false;
}

void AArcadeCar::HandleShiftUp(const FInputActionValue& Value)
{
    ShiftUp();
}

void AArcadeCar::HandleShiftDown(const FInputActionValue& Value)
{
    ShiftDown();
}

// ============================================================================
// GEAR FUNCTIONS
// ============================================================================

void AArcadeCar::ShiftUp()
{
    if (CurrentGear < GearRatios.Num() - 1)
    {
        CurrentGear++;
        UE_LOG(LogTemp, Log, TEXT("Shifted UP to: %s"), *GetGearDisplayString());
    }
}

void AArcadeCar::ShiftDown()
{
    if (CurrentGear > 0)
    {
        CurrentGear--;
        UE_LOG(LogTemp, Log, TEXT("Shifted DOWN to: %s"), *GetGearDisplayString());
    }
}

void AArcadeCar::SetGear(int32 NewGear)
{
    if (NewGear >= 0 && NewGear < GearRatios.Num())
    {
        CurrentGear = NewGear;
    }
}

FString AArcadeCar::GetGearDisplayString() const
{
    if (CurrentGear == 0) return TEXT("R");
    if (CurrentGear == 1) return TEXT("N");
    return FString::FromInt(CurrentGear - 1);
}

// ============================================================================
// MAIN TICK
// ============================================================================

void AArcadeCar::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    if (WheelOffsets.Num() < 4)
    {
        CacheWheelReferences();
    }

    FVector Velocity = CarBody->GetPhysicsLinearVelocity();
    FVector Forward = GetActorForwardVector();
    FVector Right = GetActorRightVector();

    float ForwardSpeed = FVector::DotProduct(Velocity, Forward);
    CurrentSpeedMS = ForwardSpeed / 100.f;
    CurrentSpeedKMH = CurrentSpeedMS * 3.6f;

    float LateralSpeed = FVector::DotProduct(Velocity, Right);
    if (FMath::Abs(ForwardSpeed) > 100.f)
    {
        CurrentSlipAngle = FMath::RadiansToDegrees(
            FMath::Atan2(FMath::Abs(LateralSpeed), FMath::Abs(ForwardSpeed))
        );
    }
    else
    {
        CurrentSlipAngle = 0.f;
    }

    UpdateSuspension(DeltaTime);
    UpdateSteering(DeltaTime);
    UpdateDrift(DeltaTime);
    UpdateAcceleration(DeltaTime);

    if (bAutoGearShift)
    {
        UpdateAutoGearShift();
    }

    UpdateWheelVisuals(DeltaTime);

    if (bShowDebugText)
    {
        GEngine->AddOnScreenDebugMessage(0, 0.f, FColor::White,
            FString::Printf(TEXT("Speed: %.1f km/h"), CurrentSpeedKMH));
        GEngine->AddOnScreenDebugMessage(1, 0.f, FColor::White,
            FString::Printf(TEXT("RPM: %.0f"), CurrentRPM));
        GEngine->AddOnScreenDebugMessage(2, 0.f, FColor::White,
            FString::Printf(TEXT("Gear: %s"), *GetGearDisplayString()));
        GEngine->AddOnScreenDebugMessage(3, 0.f, FColor::Yellow,
            FString::Printf(TEXT("Drift: %s | Slip: %.1f deg"),
                bIsDrifting ? TEXT("YES") : TEXT("NO"), CurrentSlipAngle));
    }
}

// ============================================================================
// SUSPENSION SYSTEM
// ============================================================================

void AArcadeCar::UpdateSuspension(float DeltaTime)
{
    UWorld* World = GetWorld();
    if (!World) return;

    FVector CarUp = GetActorUpVector();
    FVector CarLocation = GetActorLocation();
    FRotator CarRotation = GetActorRotation();

    for (int32 i = 0; i < 4; i++)
    {
        if (!WheelOffsets.IsValidIndex(i)) continue;

        FVector LocalWheelPos = WheelOffsets[i];
        FVector WorldWheelPos = CarLocation + CarRotation.RotateVector(LocalWheelPos);

        FVector RayStart = WorldWheelPos;
        FVector RayEnd = WorldWheelPos - CarUp * (SuspensionMaxLength + WheelRadius);

        FHitResult Hit;
        FCollisionQueryParams QueryParams;
        QueryParams.AddIgnoredActor(this);

        bool bHit = World->LineTraceSingleByChannel(
            Hit, RayStart, RayEnd, ECC_Visibility, QueryParams
        );

        if (bShowDebugLines)
        {
            DrawDebugLine(World, RayStart, bHit ? Hit.ImpactPoint : RayEnd,
                bHit ? FColor::Green : FColor::Red, false, -1.f, 0, 2.f);

            if (bHit)
            {
                DrawDebugSphere(World, Hit.ImpactPoint, WheelRadius * 0.3f, 8,
                    FColor::Yellow, false, -1.f);
            }
        }

        if (bHit)
        {
            WheelDataArray[i].bIsGrounded = true;
            WheelDataArray[i].ContactPoint = Hit.ImpactPoint;
            WheelDataArray[i].ContactNormal = Hit.ImpactNormal;

            float CurrentLength = Hit.Distance - WheelRadius;
            CurrentLength = FMath::Max(CurrentLength, 0.f);

            float Compression = SuspensionRestLength - CurrentLength;
            float SpringForce = SpringStrength * Compression;

            float PreviousLength = WheelDataArray[i].SpringLength;
            float SpringVelocity = (CurrentLength - PreviousLength) / DeltaTime;
            float DamperForce = -DamperStrength * SpringVelocity;

            WheelDataArray[i].SpringLength = CurrentLength;
            WheelDataArray[i].SpringVelocity = SpringVelocity;

            float TotalForce = FMath::Max(SpringForce + DamperForce, 0.f);
            FVector SuspensionForce = CarUp * TotalForce;
            CarBody->AddForceAtLocation(SuspensionForce, WorldWheelPos);

            if (bShowDebugLines)
            {
                float ForceScale = TotalForce / 50000.f;
                DrawDebugLine(World, WorldWheelPos, WorldWheelPos + CarUp * ForceScale * 100.f,
                    FColor::Blue, false, -1.f, 0, 3.f);
            }
        }
        else
        {
            WheelDataArray[i].bIsGrounded = false;
            WheelDataArray[i].SpringLength = SuspensionMaxLength;
            WheelDataArray[i].SpringVelocity = 0.f;
        }
    }
}

// ============================================================================
// STEERING SYSTEM
// ============================================================================

void AArcadeCar::UpdateSteering(float DeltaTime)
{
    float TargetSteerAngle = SteerInput * MaxSteerAngle;
    CurrentSteerAngle = FMath::FInterpTo(CurrentSteerAngle, TargetSteerAngle, DeltaTime, SteerSpeed);

    FVector Velocity = CarBody->GetPhysicsLinearVelocity();
    FVector Right = GetActorRightVector();
    FVector Forward = GetActorForwardVector();

    float LateralSpeed = FVector::DotProduct(Velocity, Right);
    float ForwardSpeed = FVector::DotProduct(Velocity, Forward);

    for (int32 i = 0; i < 4; i++)
    {
        if (!WheelDataArray[i].bIsGrounded) continue;
        if (!WheelOffsets.IsValidIndex(i)) continue;

        FVector WorldWheelPos = GetActorLocation() +
            GetActorRotation().RotateVector(WheelOffsets[i]);

        float WheelGrip = TireGrip;
        bool bIsFrontWheel = (i < 2);

        if (bIsFrontWheel)
        {
            WheelGrip *= FrontGripMultiplier;
        }

        if (!bIsFrontWheel && (bIsDrifting || bHandbrakePressed))
        {
            WheelGrip *= DriftGripMultiplier;
        }

        float GripForce = -LateralSpeed * WheelGrip * 100.f;
        GripForce = FMath::Clamp(GripForce, -60000.f, 60000.f);

        FVector LateralForce = Right * GripForce;
        CarBody->AddForceAtLocation(LateralForce, WorldWheelPos);
    }

    bool bFrontWheelsGrounded = WheelDataArray[0].bIsGrounded || WheelDataArray[1].bIsGrounded;

    if (bFrontWheelsGrounded && FMath::Abs(CurrentSpeedMS) > 0.5f)
    {
        float SteerTorque = CurrentSteerAngle * FMath::Sign(ForwardSpeed);

        if (bIsDrifting)
        {
            bool bCounterSteering = FMath::Sign(SteerInput) != FMath::Sign(LateralSpeed);
            if (bCounterSteering)
            {
                SteerTorque *= DriftSteerBoost;
            }
        }

        float SpeedFactor = FMath::Clamp(FMath::Abs(CurrentSpeedMS) / 20.f, 0.5f, 1.5f);
        SteerTorque *= SpeedFactor * 60000.f;

        FVector TorqueVector = FVector(0.f, 0.f, SteerTorque);
        FVector WorldTorque = GetActorRotation().RotateVector(TorqueVector);
        CarBody->AddTorqueInRadians(WorldTorque);
    }
}

// ============================================================================
// DRIFT SYSTEM
// ============================================================================

void AArcadeCar::UpdateDrift(float DeltaTime)
{
    FVector Velocity = CarBody->GetPhysicsLinearVelocity();
    FVector Right = GetActorRightVector();

    float LateralSpeed = FVector::DotProduct(Velocity, Right);

    bool bHighSpeed = FMath::Abs(CurrentSpeedMS) > MinDriftSpeed;
    bool bAggressiveSteering = FMath::Abs(SteerInput) > 0.7f;
    bool bHighSlipAngle = CurrentSlipAngle > DriftEntryAngle;

    bool bShouldStartDrift = bHandbrakePressed ||
        (bHighSpeed && bAggressiveSteering && bHighSlipAngle);

    bool bLowSpeed = FMath::Abs(CurrentSpeedMS) < MinDriftSpeed * 0.5f;
    bool bLowSlipAngle = CurrentSlipAngle < DriftExitAngle;

    bool bShouldStopDrift = bLowSpeed || (bLowSlipAngle && !bHandbrakePressed);

    if (!bIsDrifting && bShouldStartDrift)
    {
        bIsDrifting = true;

        if (DriftKickImpulse > 0.f)
        {
            FVector KickDirection = Right * FMath::Sign(SteerInput);
            CarBody->AddImpulse(KickDirection * DriftKickImpulse);
        }

        UE_LOG(LogTemp, Log, TEXT("DRIFT START - Slip: %.1f deg"), CurrentSlipAngle);
    }
    else if (bIsDrifting && bShouldStopDrift)
    {
        bIsDrifting = false;
        UE_LOG(LogTemp, Log, TEXT("DRIFT END"));
    }
}

// ============================================================================
// ACCELERATION SYSTEM
// ============================================================================

void AArcadeCar::UpdateAcceleration(float DeltaTime)
{
    bool bRearWheelsGrounded = WheelDataArray[2].bIsGrounded || WheelDataArray[3].bIsGrounded;

    if (!bRearWheelsGrounded)
    {
        CurrentRPM = FMath::FInterpTo(CurrentRPM, IdleRPM, DeltaTime, 5.f);
        return;
    }

    float WheelTorque = CalculateEngineTorque();

    FVector Forward = GetActorForwardVector();

    for (int32 i = 2; i < 4; i++)
    {
        if (!WheelDataArray[i].bIsGrounded) continue;
        if (!WheelOffsets.IsValidIndex(i)) continue;

        FVector WorldWheelPos = GetActorLocation() +
            GetActorRotation().RotateVector(WheelOffsets[i]);

        float WheelForce = WheelTorque * ThrottleInput * 0.5f;

        if (bIsDrifting || bHandbrakePressed)
        {
            WheelForce *= 0.6f;
        }

        FVector TractionForce = Forward * WheelForce;
        CarBody->AddForceAtLocation(TractionForce, WorldWheelPos);

        WheelDataArray[i].WheelAngularVelocity = CurrentSpeedMS / (WheelRadius / 100.f);
    }

    for (int32 i = 0; i < 2; i++)
    {
        WheelDataArray[i].WheelAngularVelocity = CurrentSpeedMS / (WheelRadius / 100.f);
    }
}

float AArcadeCar::CalculateEngineTorque()
{
    if (CurrentGear == 1 || GearRatios.Num() == 0)
    {
        CurrentRPM = IdleRPM;
        return 0.f;
    }

    float WheelRadiusMeters = WheelRadius / 100.f;
    float WheelRPM = FMath::Abs(CurrentSpeedMS) * 60.f / (2.f * PI * WheelRadiusMeters);

    float GearRatio = GearRatios[CurrentGear];

    CurrentRPM = WheelRPM * FMath::Abs(GearRatio) * FinalDriveRatio;
    CurrentRPM = FMath::Clamp(CurrentRPM, IdleRPM, MaxRPM);

    float NormalizedRPM = CurrentRPM / MaxRPM;
    float TorqueCurveMultiplier = CalculateTorqueCurve(NormalizedRPM);

    float EngineTorque = MaxEngineTorque * TorqueCurveMultiplier;
    float WheelTorque = EngineTorque * GearRatio * FinalDriveRatio;

    return WheelTorque;
}

float AArcadeCar::CalculateTorqueCurve(float NormalizedRPM)
{
    float Curve = FMath::Sin(NormalizedRPM * PI * 0.8f + 0.2f);
    Curve = FMath::Max(Curve, 0.3f);
    return Curve;
}

// ============================================================================
// AUTO GEAR SHIFT
// ============================================================================

void AArcadeCar::UpdateAutoGearShift()
{
    if (CurrentGear <= 1) return;
    if (bIsDrifting) return;

    if (CurrentRPM > MaxRPM * ShiftUpRPM)
    {
        if (CurrentGear < GearRatios.Num() - 1)
        {
            ShiftUp();
        }
    }
    else if (CurrentRPM < MaxRPM * ShiftDownRPM)
    {
        if (CurrentGear > 2)
        {
            ShiftDown();
        }
    }
}

// ============================================================================
// WHEEL VISUALS
// ============================================================================

void AArcadeCar::UpdateWheelVisuals(float DeltaTime)
{
    TArray<UStaticMeshComponent*> Meshes = {
        WheelMesh_FrontLeft,
        WheelMesh_FrontRight,
        WheelMesh_RearLeft,
        WheelMesh_RearRight
    };

    for (int32 i = 0; i < 4; i++)
    {
        if (!Meshes.IsValidIndex(i) || !Meshes[i]) continue;

        float SuspensionDrop = SuspensionRestLength - WheelDataArray[i].SpringLength;

        WheelDataArray[i].WheelRotation += WheelDataArray[i].WheelAngularVelocity * DeltaTime;

        if (WheelDataArray[i].WheelRotation > 2.f * PI)
            WheelDataArray[i].WheelRotation -= 2.f * PI;
        else if (WheelDataArray[i].WheelRotation < -2.f * PI)
            WheelDataArray[i].WheelRotation += 2.f * PI;

        FRotator WheelRot = FRotator::ZeroRotator;
        WheelRot.Pitch = FMath::RadiansToDegrees(WheelDataArray[i].WheelRotation);

        if (i < 2)
        {
            WheelRot.Yaw = CurrentSteerAngle;
        }

        if (i == 1 || i == 3)
        {
            WheelRot.Roll = 180.f;
        }

        Meshes[i]->SetRelativeLocation(FVector(0.f, 0.f, -SuspensionDrop));
        Meshes[i]->SetRelativeRotation(WheelRot);
    }
}