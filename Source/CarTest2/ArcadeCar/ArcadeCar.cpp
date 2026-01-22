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
    // Base stability
    BodyMesh->SetAngularDamping(BaseAngularDamping);

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

        // --- DRIFT STABILITY LOGIC (Prevents 180 spins) ---
        if (FMath::Abs(SpeedKMH) > 10.f)
        {
            FVector Velocity = GetVelocity();
            FVector Forward = GetActorForwardVector();
            
            // Ignore vertical movement
            Velocity.Z = 0.f;
            Forward.Z = 0.f;
            Velocity.Normalize();
            Forward.Normalize();

            // Calculate Angle
            float Dot = FVector::DotProduct(Forward, Velocity);
            float SlipAngle = FMath::Acos(Dot) * (180.f / UE_DOUBLE_PI);

            // Apply dynamic damping
            float TargetDamping = BaseAngularDamping;

            if (SlipAngle > MaxDriftAngle)
            {
                // Ramp up damping to stop the spin
                float OverLimitFactor = (SlipAngle - MaxDriftAngle) / 20.f; 
                TargetDamping = BaseAngularDamping + (StabilityCorrection * OverLimitFactor);
            }

            BodyMesh->SetAngularDamping(TargetDamping);
        }
        else
        {
            BodyMesh->SetAngularDamping(BaseAngularDamping);
        }
    }

    UpdateWheelVisuals();
    ShowDebugInfo();
}

#if WITH_EDITOR
void AArcadeCar::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
    Super::PostEditChangeProperty(PropertyChangedEvent);
    // Simple refresh for any property change
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
    
    // Update Damping Base
    if (BodyMesh)
    {
        BodyMesh->SetAngularDamping(BaseAngularDamping);
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

    GEngine->AddOnScreenDebugMessage(0, 0.f, FColor::White, TEXT("=== ARCADE CAR ==="));
    GEngine->AddOnScreenDebugMessage(1, 0.f, FColor::Green, FString::Printf(TEXT("Speed: %.0f km/h"), FMath::Abs(SpeedKMH)));
    GEngine->AddOnScreenDebugMessage(2, 0.f, FColor::Yellow, FString::Printf(TEXT("RPM: %.0f | Gear: %s"), CurrentRPM, *GearStr));
    GEngine->AddOnScreenDebugMessage(3, 0.f, FColor::Cyan, FString::Printf(TEXT("Throttle: %.2f | Steer: %.2f"), ThrottleInput, SteerInput));
    GEngine->AddOnScreenDebugMessage(4, 0.f, StatusColor, FString::Printf(TEXT("Handbrake: %s"), bIsHandbraking ? TEXT("ON") : TEXT("OFF")));
}