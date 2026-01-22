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

    // ████████████████████████████████████████████████████████████████████████
    // STEP 1: ASSIGN YOUR MESHES (Just drag & drop in Blueprint!)
    // ████████████████████████████████████████████████████████████████████████

    /** Your car body mesh - assign in Blueprint! */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "1. Meshes|Body")
    UStaticMeshComponent* BodyMesh;

    /** Front Left wheel mesh */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "1. Meshes|Wheels")
    UStaticMeshComponent* Wheel_FL;

    /** Front Right wheel mesh */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "1. Meshes|Wheels")
    UStaticMeshComponent* Wheel_FR;

    /** Rear Left wheel mesh */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "1. Meshes|Wheels")
    UStaticMeshComponent* Wheel_RL;

    /** Rear Right wheel mesh */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "1. Meshes|Wheels")
    UStaticMeshComponent* Wheel_RR;

    // ████████████████████████████████████████████████████████████████████████
    // STEP 2: POSITION YOUR WHEELS (Edit these values and see changes live!)
    // ████████████████████████████████████████████████████████████████████████

    /** Front Left wheel position (X=forward, Y=right, Z=up) */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "2. Wheel Positions", meta = (DisplayName = "Front Left (FL)"))
    FVector WheelPos_FL = FVector(140.f, -85.f, 0.f);

    /** Front Right wheel position */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "2. Wheel Positions", meta = (DisplayName = "Front Right (FR)"))
    FVector WheelPos_FR = FVector(140.f, 85.f, 0.f);

    /** Rear Left wheel position */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "2. Wheel Positions", meta = (DisplayName = "Rear Left (RL)"))
    FVector WheelPos_RL = FVector(-140.f, -85.f, 0.f);

    /** Rear Right wheel position */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "2. Wheel Positions", meta = (DisplayName = "Rear Right (RR)"))
    FVector WheelPos_RR = FVector(-140.f, 85.f, 0.f);

    // ████████████████████████████████████████████████████████████████████████
    // STEP 3: TUNE YOUR CAR (All the important settings in one place!)
    // ████████████████████████████████████████████████████████████████████████

    /** Wheel radius in cm - measure your wheel mesh! */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "3. Tuning|Wheels", meta = (ClampMin = "10.0", ClampMax = "100.0"))
    float WheelRadius = 35.f;

    /** Wheel width in cm */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "3. Tuning|Wheels", meta = (ClampMin = "10.0", ClampMax = "60.0"))
    float WheelWidth = 25.f;

    /** Tire grip - higher = more grip, lower = more sliding */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "3. Tuning|Wheels", meta = (ClampMin = "0.5", ClampMax = "10.0"))
    float TireGrip = 3.f;

    /** Engine power (torque) */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "3. Tuning|Engine", meta = (ClampMin = "100.0", ClampMax = "2000.0"))
    float EnginePower = 500.f;

    /** Maximum engine RPM */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "3. Tuning|Engine", meta = (ClampMin = "4000.0", ClampMax = "15000.0"))
    float MaxRPM = 6500.f;

    /** Suspension travel distance (up/down) */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "3. Tuning|Suspension", meta = (ClampMin = "5.0", ClampMax = "30.0"))
    float SuspensionTravel = 12.f;

    /** Suspension stiffness - higher = stiffer ride */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "3. Tuning|Suspension", meta = (ClampMin = "50.0", ClampMax = "500.0"))
    float SuspensionStiffness = 150.f;

    /** Maximum steering angle in degrees */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "3. Tuning|Steering", meta = (ClampMin = "20.0", ClampMax = "60.0"))
    float MaxSteerAngle = 40.f;

    // ████████████████████████████████████████████████████████████████████████
    // CAMERA SETTINGS
    // ████████████████████████████████████████████████████████████████████████

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "4. Camera")
    USpringArmComponent* CameraArm;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "4. Camera")
    UCameraComponent* Camera;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "4. Camera")
    float CameraDistance = 500.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "4. Camera")
    float CameraHeight = 150.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "4. Camera")
    float CameraLag = 8.f;

    // ████████████████████████████████████████████████████████████████████████
    // INPUT (Assign your Input Actions here!)
    // ████████████████████████████████████████████████████████████████████████

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "5. Input")
    UInputMappingContext* InputMapping;

    /** W/S or Triggers - Axis1D float */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "5. Input")
    UInputAction* Input_Throttle;

    /** A/D or Left Stick X - Axis1D float */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "5. Input")
    UInputAction* Input_Steer;

    /** Space or A Button - Bool */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "5. Input")
    UInputAction* Input_Handbrake;

    // ████████████████████████████████████████████████████████████████████████
    // DEBUG
    // ████████████████████████████████████████████████████████████████████████

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "6. Debug")
    bool bShowDebug = true;

    // ████████████████████████████████████████████████████████████████████████
    // RUNTIME INFO (Read-only, shown in HUD)
    // ████████████████████████████████████████████████████████████████████████

    UPROPERTY(BlueprintReadOnly, Category = "Runtime")
    float SpeedKMH = 0.f;

    UPROPERTY(BlueprintReadOnly, Category = "Runtime")
    float CurrentRPM = 0.f;

    UPROPERTY(BlueprintReadOnly, Category = "Runtime")
    int32 CurrentGear = 0;

    UPROPERTY(BlueprintReadOnly, Category = "Runtime")
    bool bIsHandbraking = false;

    // ████████████████████████████████████████████████████████████████████████
    // FUNCTIONS
    // ████████████████████████████████████████████████████████████████████████

    /** Call this after changing any settings at runtime */
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

    // Wheel rotation tracking
    float WheelSpinAngle = 0.f;
};
