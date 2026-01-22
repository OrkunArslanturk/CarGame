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
    // ========================================================================
    // CREATE BODY MESH
    // ========================================================================
    BodyMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("BodyMesh"));
    BodyMesh->SetupAttachment(GetMesh());
    BodyMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);

    // ========================================================================
    // CREATE WHEEL MESHES (attached to root, we position them manually)
    // ========================================================================
    Wheel_FL = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Wheel_FL"));
    Wheel_FL->SetupAttachment(GetMesh());
    Wheel_FL->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Wheel_FL->SetRelativeLocation(WheelPos_FL);

    Wheel_FR = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Wheel_FR"));
    Wheel_FR->SetupAttachment(GetMesh());
    Wheel_FR->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Wheel_FR->SetRelativeLocation(WheelPos_FR);
    Wheel_FR->SetRelativeScale3D(FVector(1.f, -1.f, 1.f));  // Mirror right side

    Wheel_RL = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Wheel_RL"));
    Wheel_RL->SetupAttachment(GetMesh());
    Wheel_RL->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Wheel_RL->SetRelativeLocation(WheelPos_RL);

    Wheel_RR = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Wheel_RR"));
    Wheel_RR->SetupAttachment(GetMesh());
    Wheel_RR->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Wheel_RR->SetRelativeLocation(WheelPos_RR);
    Wheel_RR->SetRelativeScale3D(FVector(1.f, -1.f, 1.f));  // Mirror right side

    // ========================================================================
    // CREATE CAMERA
    // ========================================================================
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
    CameraArm->bDoCollisionTest = true;

    Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));
    Camera->SetupAttachment(CameraArm);

    // ========================================================================
    // SETUP VEHICLE PHYSICS
    // ========================================================================
    UChaosWheeledVehicleMovementComponent* Vehicle = 
        CastChecked<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent());

    // Physics body settings
    Vehicle->Mass = 1500.f;
    Vehicle->ChassisHeight = 140.f;
    Vehicle->DragCoefficient = 0.3f;
    Vehicle->CenterOfMassOverride = FVector(0.f, 0.f, -20.f);
    Vehicle->bEnableCenterOfMassOverride = true;

    // Setup 4 wheels (positions set from our editable variables)
    Vehicle->bLegacyWheelFrictionPosition = false;
    Vehicle->WheelSetups.SetNum(4);

    // FL - Front Left
    Vehicle->WheelSetups[0].WheelClass = UArcadeWheelFront::StaticClass();
    Vehicle->WheelSetups[0].BoneName = NAME_None;
    Vehicle->WheelSetups[0].AdditionalOffset = WheelPos_FL;

    // FR - Front Right
    Vehicle->WheelSetups[1].WheelClass = UArcadeWheelFront::StaticClass();
    Vehicle->WheelSetups[1].BoneName = NAME_None;
    Vehicle->WheelSetups[1].AdditionalOffset = WheelPos_FR;

    // RL - Rear Left
    Vehicle->WheelSetups[2].WheelClass = UArcadeWheelRear::StaticClass();
    Vehicle->WheelSetups[2].BoneName = NAME_None;
    Vehicle->WheelSetups[2].AdditionalOffset = WheelPos_RL;

    // RR - Rear Right
    Vehicle->WheelSetups[3].WheelClass = UArcadeWheelRear::StaticClass();
    Vehicle->WheelSetups[3].BoneName = NAME_None;
    Vehicle->WheelSetups[3].AdditionalOffset = WheelPos_RR;

    // Engine
    Vehicle->EngineSetup.MaxTorque = EnginePower;
    Vehicle->EngineSetup.MaxRPM = MaxRPM;
    Vehicle->EngineSetup.EngineIdleRPM = 1000.f;
    Vehicle->EngineSetup.EngineBrakeEffect = 0.1f;
    Vehicle->EngineSetup.EngineRevUpMOI = 5.f;
    Vehicle->EngineSetup.EngineRevDownRate = 600.f;

    // Transmission (Automatic)
    Vehicle->TransmissionSetup.bUseAutomaticGears = true;
    Vehicle->TransmissionSetup.bUseAutoReverse = true;
    Vehicle->TransmissionSetup.FinalRatio = 3.5f;
    Vehicle->TransmissionSetup.ChangeUpRPM = 5500.f;
    Vehicle->TransmissionSetup.ChangeDownRPM = 2000.f;
    Vehicle->TransmissionSetup.GearChangeTime = 0.2f;

    // Differential (Rear Wheel Drive)
    Vehicle->DifferentialSetup.DifferentialType = EVehicleDifferential::RearWheelDrive;

    // Steering
    Vehicle->SteeringSetup.SteeringType = ESteeringType::AngleRatio;
    Vehicle->SteeringSetup.AngleRatio = 1.f;
}

// ============================================================================
// EDITOR: Update wheel positions when you change values in Details panel
// ============================================================================
#if WITH_EDITOR
void AArcadeCar::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
    Super::PostEditChangeProperty(PropertyChangedEvent);

    FName PropertyName = PropertyChangedEvent.Property ? 
        PropertyChangedEvent.Property->GetFName() : NAME_None;

    // If any wheel position or tuning setting changed, refresh everything
    if (PropertyName == GET_MEMBER_NAME_CHECKED(AArcadeCar, WheelPos_FL) ||
        PropertyName == GET_MEMBER_NAME_CHECKED(AArcadeCar, WheelPos_FR) ||
        PropertyName == GET_MEMBER_NAME_CHECKED(AArcadeCar, WheelPos_RL) ||
        PropertyName == GET_MEMBER_NAME_CHECKED(AArcadeCar, WheelPos_RR) ||
        PropertyName == GET_MEMBER_NAME_CHECKED(AArcadeCar, WheelRadius) ||
        PropertyName == GET_MEMBER_NAME_CHECKED(AArcadeCar, TireGrip) ||
        PropertyName == GET_MEMBER_NAME_CHECKED(AArcadeCar, SuspensionTravel) ||
        PropertyName == GET_MEMBER_NAME_CHECKED(AArcadeCar, SuspensionStiffness) ||
        PropertyName == GET_MEMBER_NAME_CHECKED(AArcadeCar, MaxSteerAngle) ||
        PropertyName == GET_MEMBER_NAME_CHECKED(AArcadeCar, EnginePower) ||
        PropertyName == GET_MEMBER_NAME_CHECKED(AArcadeCar, MaxRPM))
    {
        UpdateWheelPositions();
    }

    // Camera settings
    if (PropertyName == GET_MEMBER_NAME_CHECKED(AArcadeCar, CameraDistance) ||
        PropertyName == GET_MEMBER_NAME_CHECKED(AArcadeCar, CameraHeight) ||
        PropertyName == GET_MEMBER_NAME_CHECKED(AArcadeCar, CameraLag))
    {
        if (CameraArm)
        {
            CameraArm->TargetArmLength = CameraDistance;
            CameraArm->SetRelativeLocation(FVector(0.f, 0.f, CameraHeight));
            CameraArm->CameraLagSpeed = CameraLag;
            CameraArm->CameraRotationLagSpeed = CameraLag;
        }
    }
}
#endif

// ============================================================================
// BEGIN PLAY
// ============================================================================
void AArcadeCar::BeginPlay()
{
    Super::BeginPlay();

    // Apply all settings
    RefreshSettings();

    // Setup input
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

    UE_LOG(LogTemp, Log, TEXT("ArcadeCar: Ready to drive!"));
}

// ============================================================================
// REFRESH ALL SETTINGS (Call this if you change settings at runtime)
// ============================================================================
void AArcadeCar::RefreshSettings()
{
    UpdateWheelPositions();

    // Camera
    if (CameraArm)
    {
        CameraArm->TargetArmLength = CameraDistance;
        CameraArm->SetRelativeLocation(FVector(0.f, 0.f, CameraHeight));
        CameraArm->CameraLagSpeed = CameraLag;
        CameraArm->CameraRotationLagSpeed = CameraLag;
    }
}

// ============================================================================
// UPDATE WHEEL POSITIONS (Physics + Visual)
// ============================================================================
void AArcadeCar::UpdateWheelPositions()
{
    // Update visual wheel positions
    if (Wheel_FL) Wheel_FL->SetRelativeLocation(WheelPos_FL);
    if (Wheel_FR) Wheel_FR->SetRelativeLocation(WheelPos_FR);
    if (Wheel_RL) Wheel_RL->SetRelativeLocation(WheelPos_RL);
    if (Wheel_RR) Wheel_RR->SetRelativeLocation(WheelPos_RR);

    // Update physics wheel positions
    UChaosWheeledVehicleMovementComponent* Vehicle = 
        Cast<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent());
    
    if (!Vehicle) return;

    // Update wheel setup offsets
    if (Vehicle->WheelSetups.Num() >= 4)
    {
        Vehicle->WheelSetups[0].AdditionalOffset = WheelPos_FL;
        Vehicle->WheelSetups[1].AdditionalOffset = WheelPos_FR;
        Vehicle->WheelSetups[2].AdditionalOffset = WheelPos_RL;
        Vehicle->WheelSetups[3].AdditionalOffset = WheelPos_RR;
    }

    // Update engine settings
    Vehicle->EngineSetup.MaxTorque = EnginePower;
    Vehicle->EngineSetup.MaxRPM = MaxRPM;

    // Update individual wheel properties (if wheels exist)
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
        if (i < 2)
        {
            Wheel->MaxSteerAngle = MaxSteerAngle;
        }
    }
}

// ============================================================================
// TICK
// ============================================================================
void AArcadeCar::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    // Get vehicle state
    UChaosWheeledVehicleMovementComponent* Vehicle = 
        Cast<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent());

    if (Vehicle)
    {
        SpeedKMH = Vehicle->GetForwardSpeed() * 0.036f;  // cm/s to km/h
        CurrentRPM = Vehicle->GetEngineRotationSpeed();
        CurrentGear = Vehicle->GetCurrentGear();
    }

    // Update wheel visuals (rotation, suspension)
    UpdateWheelVisuals();

    // Show debug info
    ShowDebugInfo();
}

// ============================================================================
// UPDATE WHEEL VISUALS (Rotation + Suspension)
// ============================================================================
void AArcadeCar::UpdateWheelVisuals()
{
    UChaosWheeledVehicleMovementComponent* Vehicle = 
        Cast<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent());

    if (!Vehicle) return;

    TArray<UStaticMeshComponent*> WheelMeshes = { Wheel_FL, Wheel_FR, Wheel_RL, Wheel_RR };
    TArray<FVector> BasePositions = { WheelPos_FL, WheelPos_FR, WheelPos_RL, WheelPos_RR };

    for (int32 i = 0; i < Vehicle->Wheels.Num() && i < WheelMeshes.Num(); i++)
    {
        UChaosVehicleWheel* PhysWheel = Vehicle->Wheels[i];
        UStaticMeshComponent* VisualWheel = WheelMeshes[i];

        if (!PhysWheel || !VisualWheel) continue;

        // Get wheel physics state
        float SuspensionOffset = PhysWheel->GetSuspensionOffset();
        float SteerAngle = PhysWheel->GetSteerAngle();
        float SpinAngle = PhysWheel->GetRotationAngle();

        // Update position (base position + suspension offset)
        FVector NewPos = BasePositions[i];
        NewPos.Z += SuspensionOffset;
        VisualWheel->SetRelativeLocation(NewPos);

        // Update rotation
        FRotator NewRot = FRotator::ZeroRotator;
        NewRot.Pitch = SpinAngle;  // Wheel spin

        // Front wheels: add steering
        if (i < 2)
        {
            NewRot.Yaw = SteerAngle;
        }

        VisualWheel->SetRelativeRotation(NewRot);
    }
}

// ============================================================================
// INPUT SETUP
// ============================================================================
void AArcadeCar::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
    Super::SetupPlayerInputComponent(PlayerInputComponent);

    UEnhancedInputComponent* Input = Cast<UEnhancedInputComponent>(PlayerInputComponent);
    if (!Input) return;

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

void AArcadeCar::OnThrottle(const FInputActionValue& Value)
{
    ThrottleInput = Value.Get<float>();

    UChaosWheeledVehicleMovementComponent* Vehicle = 
        Cast<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent());

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

    UChaosWheeledVehicleMovementComponent* Vehicle = 
        Cast<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent());

    if (Vehicle)
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

    UChaosWheeledVehicleMovementComponent* Vehicle = 
        Cast<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent());

    if (Vehicle)
    {
        Vehicle->SetHandbrakeInput(false);
    }
}

// ============================================================================
// DEBUG HUD
// ============================================================================
void AArcadeCar::ShowDebugInfo()
{
    if (!bShowDebug || !GEngine) return;

    FString GearStr = (CurrentGear == -1) ? TEXT("R") : 
                      (CurrentGear == 0) ? TEXT("N") : 
                      FString::FromInt(CurrentGear);

    GEngine->AddOnScreenDebugMessage(0, 0.f, FColor::White, TEXT("=== ARCADE CAR ==="));
    GEngine->AddOnScreenDebugMessage(1, 0.f, FColor::Green,
        FString::Printf(TEXT("Speed: %.0f km/h"), FMath::Abs(SpeedKMH)));
    GEngine->AddOnScreenDebugMessage(2, 0.f, FColor::Yellow,
        FString::Printf(TEXT("RPM: %.0f | Gear: %s"), CurrentRPM, *GearStr));
    GEngine->AddOnScreenDebugMessage(3, 0.f, FColor::Cyan,
        FString::Printf(TEXT("Throttle: %.2f | Steer: %.2f"), ThrottleInput, SteerInput));
    GEngine->AddOnScreenDebugMessage(4, 0.f, bIsHandbraking ? FColor::Red : FColor::White,
        FString::Printf(TEXT("Handbrake: %s"), bIsHandbraking ? TEXT("ON") : TEXT("OFF")));
}
