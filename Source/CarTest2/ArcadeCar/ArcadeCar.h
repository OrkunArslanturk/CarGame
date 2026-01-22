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

    // --- Configuration: Drive Assist ---

    // Master toggle for all assist systems
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Drive Assist")
    bool bEnableDriveAssist = true;

    // How strongly to straighten velocity toward forward direction (0-1)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Drive Assist", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float VelocityStraightenStrength = 0.6f;

    // Speed at which drive assist fully kicks in (km/h)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Drive Assist", meta = (ClampMin = "5.0"))
    float AssistMinSpeed = 15.f;

    // Counter-steer assist strength when car starts rotating wrong way
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Drive Assist", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float CounterSteerAssist = 0.5f;

    // --- Configuration: Drift Assist (Anti-Spin) ---

    // Maximum allowed drift angle before hard correction kicks in
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Drift Assist", meta = (ClampMin = "30.0", ClampMax = "90.0"))
    float MaxDriftAngle = 25.f;

    // Soft limit - assist starts ramping up here
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Drift Assist", meta = (ClampMin = "15.0", ClampMax = "60.0"))
    float DriftAssistStartAngle = 35.f;

    // How aggressively to kill angular velocity when drifting (deg/s reduction per frame)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Drift Assist", meta = (ClampMin = "0.0"))
    float AngularVelocityDamping = 5.0f;

    // Extra damping multiplier when exceeding MaxDriftAngle (emergency stop)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Drift Assist", meta = (ClampMin = "1.0"))
    float EmergencyDampingMultiplier = 4.0f;

    // Velocity correction strength when over max angle (pulls car straight)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Drift Assist", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float OverAngleVelocityCorrection = 0.8f;

    // --- Configuration: Camera ---

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Camera")
    float CameraDistance = 500.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Camera")
    float CameraHeight = 150.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Camera")
    float CameraLag = 8.f;

    // --- Input ---

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Input")
    UInputMappingContext* InputMapping;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Input")
    UInputAction* Input_Throttle;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Input")
    UInputAction* Input_Steer;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Input")
    UInputAction* Input_Handbrake;

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

    // Debug: Current slip angle in degrees
    UPROPERTY(BlueprintReadOnly, Category = "State")
    float CurrentSlipAngle = 0.f;

    // Debug: Which direction we're drifting (-1 = left, 0 = straight, 1 = right)
    UPROPERTY(BlueprintReadOnly, Category = "State")
    float DriftDirection = 0.f;

    UFUNCTION(BlueprintCallable, Category = "Vehicle")
    void RefreshSettings();

protected:
    void OnThrottle(const FInputActionValue& Value);
    void OnSteer(const FInputActionValue& Value);
    void OnHandbrakeStart(const FInputActionValue& Value);
    void OnHandbrakeEnd(const FInputActionValue& Value);

    void UpdateWheelPositions();
    void UpdateWheelVisuals();
    void ShowDebugInfo();

    // New assist functions
    void ApplyDriveAssist(float DeltaTime);
    void ApplyDriftAssist(float DeltaTime);
    float CalculateSlipAngle(FVector& OutVelocityDir, FVector& OutForwardDir) const;

private:
    float ThrottleInput = 0.f;
    float SteerInput = 0.f;
};
