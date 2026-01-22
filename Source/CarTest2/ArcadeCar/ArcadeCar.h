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

    // --- Configuration: Stability (Anti-Spin) ---

    // Angle in degrees where stability control kicks in (stops 180 spins)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Stability", meta = (ClampMin = "45.0", ClampMax = "120.0"))
    float MaxDriftAngle = 70.f;

    // Normal resistance to rotation
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Stability")
    float BaseAngularDamping = 2.0f;

    // Extra resistance applied when exceeding MaxDriftAngle
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config|Stability")
    float StabilityCorrection = 10.0f;

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

private:
    float ThrottleInput = 0.f;
    float SteerInput = 0.f;
};