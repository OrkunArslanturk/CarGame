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

    // --- Components ---

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

    // --- Configuration: Wheel Positions ---

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Positions")
    FVector WheelPos_FL = FVector(140.f, -85.f, 0.f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Positions")
    FVector WheelPos_FR = FVector(140.f, 85.f, 0.f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Positions")
    FVector WheelPos_RL = FVector(-140.f, -85.f, 0.f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Positions")
    FVector WheelPos_RR = FVector(-140.f, 85.f, 0.f);

    // --- Configuration: Tuning ---

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

    // --- Configuration: Arcade Feel ---

    // Enable all arcade physics enhancements
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Arcade Feel")
    bool bEnableArcadePhysics = true;

    // Steering becomes tighter at low speeds, more stable at high speeds
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Arcade Feel", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float SpeedSteeringFactor = 0.7f;

    // Minimum steering multiplier at max speed (prevents over-twitchiness)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Arcade Feel", meta = (ClampMin = "0.3", ClampMax = "1.0"))
    float MinSpeedSteeringMultiplier = 0.5f;

    // Speed at which steering starts reducing (km/h)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Arcade Feel", meta = (ClampMin = "20.0"))
    float SteeringReductionStartSpeed = 60.f;

    // Downforce coefficient - adds grip at high speed
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Arcade Feel", meta = (ClampMin = "0.0"))
    float DownforceCoefficient = 0.5f;

    // Speed at which downforce starts applying (km/h)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Arcade Feel", meta = (ClampMin = "50.0"))
    float DownforceStartSpeed = 80.f;

    // Air control strength (0-1) - allows rotation while airborne
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Arcade Feel", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float AirControlStrength = 0.4f;

    // How quickly throttle input reaches full value (higher = snappier)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Arcade Feel", meta = (ClampMin = "1.0", ClampMax = "20.0"))
    float ThrottleResponseRate = 8.f;

    // How quickly car accelerates from standstill (launch boost)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Arcade Feel", meta = (ClampMin = "1.0", ClampMax = "3.0"))
    float LaunchBoostMultiplier = 1.5f;

    // Speed below which launch boost applies (km/h)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Arcade Feel", meta = (ClampMin = "10.0"))
    float LaunchBoostMaxSpeed = 30.f;

    // --- Configuration: Nitro/Boost ---

    // Enable the nitro system
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Nitro")
    bool bEnableNitro = true;

    // Maximum nitro amount (seconds of boost at full drain)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Nitro", meta = (ClampMin = "1.0", ClampMax = "30.0"))
    float MaxNitroAmount = 10.f;

    // Starting nitro amount
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Nitro", meta = (ClampMin = "0.0"))
    float StartingNitroAmount = 5.f;

    // How fast nitro drains when boosting (units per second)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Nitro", meta = (ClampMin = "0.5"))
    float NitroDrainRate = 2.f;

    // Passive nitro regeneration rate (units per second, 0 = no regen)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Nitro", meta = (ClampMin = "0.0"))
    float NitroRegenRate = 0.3f;

    // Extra regen from drifting (units per second while drifting)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Nitro", meta = (ClampMin = "0.0"))
    float NitroDriftRegenBonus = 0.8f;

    // Engine torque multiplier while boosting
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Nitro", meta = (ClampMin = "1.0", ClampMax = "3.0"))
    float NitroTorqueMultiplier = 1.8f;

    // Direct forward force applied during boost (Newtons)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Nitro", meta = (ClampMin = "0.0"))
    float NitroBoostForce = 50000.f;

    // Speed cap while boosting (km/h, 0 = no cap)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Nitro", meta = (ClampMin = "0.0"))
    float NitroMaxSpeed = 250.f;

    // Minimum nitro required to activate boost
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Nitro", meta = (ClampMin = "0.0", ClampMax = "2.0"))
    float NitroMinToActivate = 0.5f;

    // FOV increase during boost (added to base FOV)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Nitro|Visuals", meta = (ClampMin = "0.0", ClampMax = "30.0"))
    float NitroFOVIncrease = 15.f;

    // How fast FOV transitions in/out (lerp speed)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Nitro|Visuals", meta = (ClampMin = "1.0"))
    float NitroFOVLerpSpeed = 8.f;

    // --- Configuration: Drive Assist ---

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Drive Assist")
    bool bEnableDriveAssist = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Drive Assist", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float VelocityStraightenStrength = 0.6f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Drive Assist", meta = (ClampMin = "5.0"))
    float AssistMinSpeed = 15.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Drive Assist", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float CounterSteerAssist = 0.5f;

    // --- Configuration: Drift Assist (Anti-Spin) ---

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Drift Assist", meta = (ClampMin = "10.0", ClampMax = "20.0"))
    float MaxDriftAngle = 15.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Drift Assist", meta = (ClampMin = "15.0", ClampMax = "60.0"))
    float DriftAssistStartAngle = 35.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Drift Assist", meta = (ClampMin = "0.0"))
    float AngularVelocityDamping = 5.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Drift Assist", meta = (ClampMin = "1.0"))
    float EmergencyDampingMultiplier = 4.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Drift Assist", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float OverAngleVelocityCorrection = 0.8f;

    // --- Configuration: Camera ---

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Camera")
    float CameraDistance = 500.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Camera")
    float CameraHeight = 150.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Camera")
    float CameraLag = 8.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Camera")
    float BaseFOV = 90.f;

    // --- Input ---

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Input")
    UInputMappingContext* InputMapping;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Input")
    UInputAction* Input_Throttle;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Input")
    UInputAction* Input_Steer;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Input")
    UInputAction* Input_Handbrake;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Input")
    UInputAction* Input_Nitro;

    // --- Debug & State ---

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug")
    bool bShowDebug = true;

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

    // Is the car currently airborne
    UPROPERTY(BlueprintReadOnly, Category = "State")
    bool bIsAirborne = false;

    // Current nitro amount
    UPROPERTY(BlueprintReadOnly, Category = "State|Nitro")
    float CurrentNitro = 0.f;

    // Is nitro currently being used
    UPROPERTY(BlueprintReadOnly, Category = "State|Nitro")
    bool bIsNitroActive = false;

    // Normalized nitro amount (0-1) for UI
    UPROPERTY(BlueprintReadOnly, Category = "State|Nitro")
    float NitroPercent = 0.f;

    // Current boost intensity (0-1) for VFX/audio
    UPROPERTY(BlueprintReadOnly, Category = "State|Nitro")
    float NitroIntensity = 0.f;

    // --- Functions ---

    UFUNCTION(BlueprintCallable, Category = "Vehicle")
    void RefreshSettings();

    // Add nitro to the tank
    UFUNCTION(BlueprintCallable, Category = "Vehicle|Nitro")
    void AddNitro(float Amount);

    // Set nitro to full
    UFUNCTION(BlueprintCallable, Category = "Vehicle|Nitro")
    void RefillNitro();

    // Check if nitro can be activated
    UFUNCTION(BlueprintPure, Category = "Vehicle|Nitro")
    bool CanActivateNitro() const;

protected:
    void OnThrottle(const FInputActionValue& Value);
    void OnSteer(const FInputActionValue& Value);
    void OnHandbrakeStart(const FInputActionValue& Value);
    void OnHandbrakeEnd(const FInputActionValue& Value);
    void OnNitroStart(const FInputActionValue& Value);
    void OnNitroEnd(const FInputActionValue& Value);

    void UpdateWheelPositions();
    void UpdateWheelVisuals();
    void ShowDebugInfo();

    // Assist systems
    void ApplyDriveAssist(float DeltaTime);
    void ApplyDriftAssist(float DeltaTime);
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
    float SteerInput = 0.f;
    float SmoothedThrottleInput = 0.f;
    bool bNitroInputHeld = false;
    float CurrentFOV = 90.f;
    float BaseEngineTorque = 0.f;
    int32 WheelsOnGround = 0;
};
