#pragma once

#include "CoreMinimal.h"
#include "WheeledVehiclePawn.h"
#include "InputActionValue.h"
#include "ArcadeCar.generated.h"

class UChaosWheeledVehicleMovementComponent;
class UInputAction;
class UInputMappingContext;
class USpringArmComponent;
class UCameraComponent;

UCLASS()
class CARTEST2_API AArcadeCar : public AWheeledVehiclePawn
{
    GENERATED_BODY()

public:
    AArcadeCar();

    virtual void BeginPlay() override;
    virtual void Tick(float DeltaTime) override;
    virtual void SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) override;

#if WITH_EDITOR
    virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
#endif

    // Visual Components
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components")
    UStaticMeshComponent* BodyMesh;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components")
    UStaticMeshComponent* Wheel_FL;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components")
    UStaticMeshComponent* Wheel_FR;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components")
    UStaticMeshComponent* Wheel_RL;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components")
    UStaticMeshComponent* Wheel_RR;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components")
    USpringArmComponent* CameraArm;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components")
    UCameraComponent* Camera;

    // Wheel Positions
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Positions")
    FVector WheelPos_FL = FVector(140.f, -85.f, 0.f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Positions")
    FVector WheelPos_FR = FVector(140.f, 85.f, 0.f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Positions")
    FVector WheelPos_RL = FVector(-140.f, -85.f, 0.f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Positions")
    FVector WheelPos_RR = FVector(-140.f, 85.f, 0.f);

    // Vehicle Tuning
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Tuning")
    float WheelRadius = 35.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Tuning")
    float WheelWidth = 25.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Tuning")
    float TireGrip = 3.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Tuning")
    float EnginePower = 1200.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Tuning")
    float MaxRPM = 8500.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Tuning")
    float SuspensionTravel = 25.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Tuning")
    float SuspensionStiffness = 75.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Tuning")
    float MaxSteerAngle = 40.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Tuning", meta = (ClampMin = "1.0", ClampMax = "20.0"))
    float ReverseThresholdSpeed = 5.f;

    // Arcade Physics
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Arcade")
    bool bEnableArcadePhysics = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Drift|Torque")
    float MaxYawRotationSpeed = 120.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Drift|Friction")
    float DriftSideFriction = 5.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Drift|Friction")
    float NormalSideFriction = 20.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Drift|Torque")
    float AngularDamping = 25.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Arcade", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float SpeedSteeringFactor = 0.7f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Arcade", meta = (ClampMin = "0.3", ClampMax = "1.0"))
    float MinSpeedSteeringMultiplier = 0.5f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Arcade", meta = (ClampMin = "20.0"))
    float SteeringReductionStartSpeed = 60.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Arcade", meta = (ClampMin = "0.0"))
    float DownforceCoefficient = 6.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Arcade", meta = (ClampMin = "50.0"))
    float DownforceStartSpeed = 40.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Arcade", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float AirControlStrength = 0.4f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Arcade", meta = (ClampMin = "1.0", ClampMax = "20.0"))
    float ThrottleResponseRate = 8.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Arcade", meta = (ClampMin = "1.0", ClampMax = "3.0"))
    float LaunchBoostMultiplier = 2.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Arcade", meta = (ClampMin = "10.0"))
    float LaunchBoostMaxSpeed = 45.f;

    // Nitro System
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Nitro")
    bool bEnableNitro = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Nitro", meta = (ClampMin = "1.0", ClampMax = "30.0"))
    float MaxNitroAmount = 10.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Nitro", meta = (ClampMin = "0.0"))
    float StartingNitroAmount = 5.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Nitro", meta = (ClampMin = "0.5"))
    float NitroDrainRate = 2.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Nitro", meta = (ClampMin = "0.0"))
    float NitroRegenRate = 0.3f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Nitro", meta = (ClampMin = "0.0"))
    float NitroDriftRegenBonus = 0.8f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Nitro", meta = (ClampMin = "1.0", ClampMax = "3.0"))
    float NitroTorqueMultiplier = 2.5f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Nitro", meta = (ClampMin = "0.0"))
    float NitroBoostForce = 80000.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Nitro", meta = (ClampMin = "0.0"))
    float NitroMaxSpeed = 350.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Nitro", meta = (ClampMin = "0.0", ClampMax = "2.0"))
    float NitroMinToActivate = 0.5f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Nitro", meta = (ClampMin = "0.0", ClampMax = "30.0"))
    float NitroFOVIncrease = 15.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Nitro", meta = (ClampMin = "1.0"))
    float NitroFOVLerpSpeed = 8.f;

    // Camera
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Camera")
    float CameraDistance = 500.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Camera")
    float CameraHeight = 150.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Camera")
    float CameraLag = 8.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Camera")
    float BaseFOV = 90.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Camera|Juice")
    float CameraDriftSwingAngle = 15.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Camera|Juice")
    float CameraSwingSpeed = 5.f;

    // Input Actions
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Input")
    UInputMappingContext* InputMapping;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Input")
    UInputAction* Input_Throttle;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Input")
    UInputAction* Input_Steer;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Input")
    UInputAction* Input_Brake;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Input")
    UInputAction* Input_Handbrake;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Input")
    UInputAction* Input_Nitro;

    // Debug
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug")
    bool bShowDebug = true;

    // Runtime State
    UPROPERTY(BlueprintReadOnly, Category = "State")
    float SpeedKMH = 0.f;

    UPROPERTY(BlueprintReadOnly, Category = "State")
    float CurrentRPM = 0.f;

    UPROPERTY(BlueprintReadOnly, Category = "State")
    int32 CurrentGear = 0;

    UPROPERTY(BlueprintReadOnly, Category = "State")
    bool bIsHandbraking = false;

    UPROPERTY(BlueprintReadOnly, Category = "State")
    float CurrentSlipAngle = 0.f;

    UPROPERTY(BlueprintReadOnly, Category = "State")
    float DriftDirection = 0.f;

    UPROPERTY(BlueprintReadOnly, Category = "State")
    bool bIsAirborne = false;

    UPROPERTY(BlueprintReadOnly, Category = "State")
    bool bIsDrifting = false;

    UPROPERTY(BlueprintReadOnly, Category = "State")
    float DriftIntensity = 0.f;

    UPROPERTY(BlueprintReadOnly, Category = "State")
    float DriftScore = 0.f;

    UPROPERTY(BlueprintReadOnly, Category = "State")
    float DriftTime = 0.f;

    UPROPERTY(BlueprintReadOnly, Category = "State")
    bool bWantsToReverse = false;

    UPROPERTY(BlueprintReadOnly, Category = "State|Nitro")
    float CurrentNitro = 0.f;

    UPROPERTY(BlueprintReadOnly, Category = "State|Nitro")
    bool bIsNitroActive = false;

    UPROPERTY(BlueprintReadOnly, Category = "State|Nitro")
    float NitroPercent = 0.f;

    UPROPERTY(BlueprintReadOnly, Category = "State|Nitro")
    float NitroIntensity = 0.f;

    // Public Functions
    UFUNCTION(BlueprintCallable, Category = "Vehicle")
    void RefreshSettings();

    UFUNCTION(BlueprintCallable, Category = "Vehicle|Nitro")
    void AddNitro(float Amount);

    UFUNCTION(BlueprintCallable, Category = "Vehicle|Nitro")
    void RefillNitro();

    UFUNCTION(BlueprintPure, Category = "Vehicle|Nitro")
    bool CanActivateNitro() const;

protected:
    void OnThrottle(const FInputActionValue& Value);
    void OnBrake(const FInputActionValue& Value);
    void OnSteer(const FInputActionValue& Value);
    void OnHandbrakeStart(const FInputActionValue& Value);
    void OnHandbrakeEnd(const FInputActionValue& Value);
    void OnNitroStart(const FInputActionValue& Value);
    void OnNitroEnd(const FInputActionValue& Value);

    void UpdateWheelPositions();
    void UpdateWheelVisuals();
    void ShowDebugInfo();

    void ApplyCustomPhysics(float DeltaTime);
    void UpdateDynamicGrip();
    float CalculateSlipAngle(FVector& OutVelocityDir, FVector& OutForwardDir) const;

    void ApplyArcadePhysics(float DeltaTime);
    void ApplySpeedSensitiveSteering();
    void ApplyDownforce(float DeltaTime);
    void ApplyAirControl(float DeltaTime);
    void CheckGroundContact();

    void UpdateNitro(float DeltaTime);
    void ApplyNitroBoost(float DeltaTime);
    void UpdateNitroVisuals(float DeltaTime);

    void UpdateCameraEffects(float DeltaTime);

private:
    float ThrottleInput = 0.f;
    float BrakeInput = 0.f;
    float SteerInput = 0.f;
    float SmoothedThrottleInput = 0.f;
    bool bNitroInputHeld = false;
    float CurrentFOV = 90.f;
    float BaseEngineTorque = 0.f;
    int32 WheelsOnGround = 0;
    float PrevWheelSpin[4] = { 0.f, 0.f, 0.f, 0.f };
    FVector WheelBasePositions[4];
    FVector WheelBaseScales[4];
};
