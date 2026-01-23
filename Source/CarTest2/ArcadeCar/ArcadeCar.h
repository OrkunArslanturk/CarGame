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

    // --- Visual Components ---
    
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

    // --- Wheel Positions (relative to root) ---

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Positions")
    FVector WheelPos_FL = FVector(140.f, -85.f, 0.f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Positions")
    FVector WheelPos_FR = FVector(140.f, 85.f, 0.f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Positions")
    FVector WheelPos_RL = FVector(-140.f, -85.f, 0.f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Positions")
    FVector WheelPos_RR = FVector(-140.f, 85.f, 0.f);

    // --- Vehicle Tuning ---

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Tuning")
    float WheelRadius = 35.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Tuning")
    float WheelWidth = 25.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Tuning")
    float TireGrip = 3.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Tuning")
    float EnginePower = 500.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Tuning")
    float MaxRPM = 6500.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Tuning")
    float SuspensionTravel = 12.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Tuning")
    float SuspensionStiffness = 150.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Tuning")
    float MaxSteerAngle = 40.f;

    // Speed threshold for brake-to-reverse transition (km/h)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Tuning", meta = (ClampMin = "1.0", ClampMax = "20.0"))
    float ReverseThresholdSpeed = 5.f;

    // --- Arcade Physics ---

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Arcade")
    bool bEnableArcadePhysics = true;

    // Reduces steering sensitivity at high speed
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Arcade", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float SpeedSteeringFactor = 0.7f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Arcade", meta = (ClampMin = "0.3", ClampMax = "1.0"))
    float MinSpeedSteeringMultiplier = 0.5f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Arcade", meta = (ClampMin = "20.0"))
    float SteeringReductionStartSpeed = 60.f;

    // Adds downward force at high speed for grip
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Arcade", meta = (ClampMin = "0.0"))
    float DownforceCoefficient = 0.5f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Arcade", meta = (ClampMin = "50.0"))
    float DownforceStartSpeed = 80.f;

    // Allows rotation control while airborne
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Arcade", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float AirControlStrength = 0.4f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Arcade", meta = (ClampMin = "1.0", ClampMax = "20.0"))
    float ThrottleResponseRate = 8.f;

    // Extra acceleration from standstill
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Arcade", meta = (ClampMin = "1.0", ClampMax = "3.0"))
    float LaunchBoostMultiplier = 1.5f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Arcade", meta = (ClampMin = "10.0"))
    float LaunchBoostMaxSpeed = 30.f;

    // --- Nitro System ---

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

    // Extra nitro gained while drifting
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Nitro", meta = (ClampMin = "0.0"))
    float NitroDriftRegenBonus = 0.8f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Nitro", meta = (ClampMin = "1.0", ClampMax = "3.0"))
    float NitroTorqueMultiplier = 1.8f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Nitro", meta = (ClampMin = "0.0"))
    float NitroBoostForce = 50000.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Nitro", meta = (ClampMin = "0.0"))
    float NitroMaxSpeed = 250.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Nitro", meta = (ClampMin = "0.0", ClampMax = "2.0"))
    float NitroMinToActivate = 0.5f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Nitro", meta = (ClampMin = "0.0", ClampMax = "30.0"))
    float NitroFOVIncrease = 15.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Nitro", meta = (ClampMin = "1.0"))
    float NitroFOVLerpSpeed = 8.f;

    // --- Stability Control (normal driving) ---

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Stability")
    bool bEnableStabilityControl = true;

    // How strongly to correct unwanted slip
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Stability", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float StabilityStrength = 0.85f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Stability", meta = (ClampMin = "1.0", ClampMax = "15.0"))
    float StabilitySlipThreshold = 5.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Stability", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float YawDampingStrength = 0.7f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Stability", meta = (ClampMin = "30.0", ClampMax = "180.0"))
    float MaxYawRateWithoutHandbrake = 80.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Stability", meta = (ClampMin = "1.0", ClampMax = "3.0"))
    float StabilityGripMultiplier = 1.5f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Stability", meta = (ClampMin = "5.0"))
    float StabilityMinSpeed = 20.f;

    // --- Drift System ---

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Drift")
    bool bEnableDriftAssist = true;

    // Target angle for best drift boost
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Drift", meta = (ClampMin = "15.0", ClampMax = "60.0"))
    float OptimalDriftAngle = 35.f;

    // Hard limit before emergency correction
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Drift", meta = (ClampMin = "40.0", ClampMax = "90.0"))
    float MaxDriftAngle = 65.f;

    // How much throttle affects drift angle
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Drift", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float ThrottleDriftControl = 0.6f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Drift", meta = (ClampMin = "0.5", ClampMax = "2.0"))
    float DriftSteeringMultiplier = 1.3f;

    // Forward momentum kept while drifting
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Drift", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float DriftMomentumRetention = 0.7f;

    // Speed boost for maintaining good drift
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Drift", meta = (ClampMin = "0.0"))
    float DriftBoostForce = 15000.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Drift", meta = (ClampMin = "1.0", ClampMax = "10.0"))
    float DriftResponsiveness = 4.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Drift", meta = (ClampMin = "0.1", ClampMax = "1.0"))
    float DriftRearGripMultiplier = 0.4f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Drift", meta = (ClampMin = "0.5", ClampMax = "2.0"))
    float DriftFrontGripMultiplier = 1.4f;

    // --- Camera ---

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Camera")
    float CameraDistance = 500.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Camera")
    float CameraHeight = 150.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Camera")
    float CameraLag = 8.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Camera")
    float BaseFOV = 90.f;

    // --- Input Actions ---

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

    // --- Debug ---

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug")
    bool bShowDebug = true;

    // --- Runtime State (read-only) ---

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
    bool bWantsToReverse = false;

    UPROPERTY(BlueprintReadOnly, Category = "State|Nitro")
    float CurrentNitro = 0.f;

    UPROPERTY(BlueprintReadOnly, Category = "State|Nitro")
    bool bIsNitroActive = false;

    UPROPERTY(BlueprintReadOnly, Category = "State|Nitro")
    float NitroPercent = 0.f;

    UPROPERTY(BlueprintReadOnly, Category = "State|Nitro")
    float NitroIntensity = 0.f;

    // --- Blueprint Callable Functions ---

    UFUNCTION(BlueprintCallable, Category = "Vehicle")
    void RefreshSettings();

    UFUNCTION(BlueprintCallable, Category = "Vehicle|Nitro")
    void AddNitro(float Amount);

    UFUNCTION(BlueprintCallable, Category = "Vehicle|Nitro")
    void RefillNitro();

    UFUNCTION(BlueprintPure, Category = "Vehicle|Nitro")
    bool CanActivateNitro() const;

protected:
    // Input handlers
    void OnThrottle(const FInputActionValue& Value);
    void OnBrake(const FInputActionValue& Value);
    void OnSteer(const FInputActionValue& Value);
    void OnHandbrakeStart(const FInputActionValue& Value);
    void OnHandbrakeEnd(const FInputActionValue& Value);
    void OnNitroStart(const FInputActionValue& Value);
    void OnNitroEnd(const FInputActionValue& Value);

    // Visual updates
    void UpdateWheelPositions();
    void UpdateWheelVisuals();
    void ShowDebugInfo();

    // Physics systems
    void ApplyStabilityControl(float DeltaTime);
    void ApplyDriftPhysics(float DeltaTime);
    void UpdateDynamicGrip();
    float CalculateSlipAngle(FVector& OutVelocityDir, FVector& OutForwardDir) const;

    // Arcade physics
    void ApplyArcadePhysics(float DeltaTime);
    void ApplySpeedSensitiveSteering();
    void ApplyDownforce(float DeltaTime);
    void ApplyAirControl(float DeltaTime);
    void CheckGroundContact();

    // Nitro system
    void UpdateNitro(float DeltaTime);
    void ApplyNitroBoost(float DeltaTime);
    void UpdateNitroVisuals(float DeltaTime);

private:
    float ThrottleInput = 0.f;
    float BrakeInput = 0.f;
    float SteerInput = 0.f;
    float SmoothedThrottleInput = 0.f;
    bool bNitroInputHeld = false;
    float CurrentFOV = 90.f;
    float BaseEngineTorque = 0.f;
    int32 WheelsOnGround = 0;
    float DriftAngleMomentum = 0.f;
    float TimeDrifting = 0.f;
};
