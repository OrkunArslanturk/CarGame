// ArcadeCar.h
// Arcade-style car with Enhanced Input System
// For AGP Racing Game Assignment - Futuregames 2026

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "Components/BoxComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Components/SceneComponent.h"
#include "GameFramework/SpringArmComponent.h"
#include "Camera/CameraComponent.h"
#include "InputActionValue.h"
#include "ArcadeCar.generated.h"

// Forward declarations for Enhanced Input
class UInputAction;
class UInputMappingContext;

// Stores runtime data for each wheel
USTRUCT(BlueprintType)
struct FWheelData
{
    GENERATED_BODY()

    UPROPERTY(BlueprintReadOnly, Category = "Wheel")
    bool bIsGrounded = false;

    UPROPERTY(BlueprintReadOnly, Category = "Wheel")
    float SpringLength = 0.f;

    UPROPERTY(BlueprintReadOnly, Category = "Wheel")
    float SpringVelocity = 0.f;

    UPROPERTY(BlueprintReadOnly, Category = "Wheel")
    FVector ContactPoint = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category = "Wheel")
    FVector ContactNormal = FVector::UpVector;

    UPROPERTY(BlueprintReadOnly, Category = "Wheel")
    float WheelRotation = 0.f;

    UPROPERTY(BlueprintReadOnly, Category = "Wheel")
    float WheelAngularVelocity = 0.f;
};

UCLASS()
class CARTEST2_API AArcadeCar : public APawn
{
    GENERATED_BODY()

public:
    AArcadeCar();

    virtual void BeginPlay() override;
    virtual void Tick(float DeltaTime) override;
    virtual void SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) override;

#if WITH_EDITOR
    virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
    virtual void PostEditMove(bool bFinished) override;
#endif

    // ==================== ENHANCED INPUT ====================

    // Input Mapping Context - assign in Blueprint!
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Input", meta = (DisplayName = "Car Input Mapping Context"))
    UInputMappingContext* CarMappingContext;

    // Input Actions - assign these in Blueprint!
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Input", meta = (DisplayName = "IA Throttle"))
    UInputAction* ThrottleAction;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Input", meta = (DisplayName = "IA Steer"))
    UInputAction* SteerAction;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Input", meta = (DisplayName = "IA Handbrake"))
    UInputAction* HandbrakeAction;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Input", meta = (DisplayName = "IA Shift Up"))
    UInputAction* ShiftUpAction;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Input", meta = (DisplayName = "IA Shift Down"))
    UInputAction* ShiftDownAction;

    // ==================== COMPONENTS ====================

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components")
    UBoxComponent* CarBody;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components")
    UStaticMeshComponent* BodyMesh;

    // ==================== WHEEL SOCKETS ====================

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Wheel Sockets", meta = (DisplayName = "Socket: Front Left"))
    USceneComponent* WheelSocket_FrontLeft;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Wheel Sockets", meta = (DisplayName = "Socket: Front Right"))
    USceneComponent* WheelSocket_FrontRight;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Wheel Sockets", meta = (DisplayName = "Socket: Rear Left"))
    USceneComponent* WheelSocket_RearLeft;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Wheel Sockets", meta = (DisplayName = "Socket: Rear Right"))
    USceneComponent* WheelSocket_RearRight;

    // ==================== WHEEL MESHES ====================

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Wheel Meshes")
    UStaticMeshComponent* WheelMesh_FrontLeft;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Wheel Meshes")
    UStaticMeshComponent* WheelMesh_FrontRight;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Wheel Meshes")
    UStaticMeshComponent* WheelMesh_RearLeft;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Wheel Meshes")
    UStaticMeshComponent* WheelMesh_RearRight;

    // ==================== CAMERA ====================

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Camera")
    USpringArmComponent* CameraArm;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Camera")
    UCameraComponent* Camera;

    // ==================== CAR DIMENSIONS ====================

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Car Dimensions")
    FVector BodyExtent = FVector(150.f, 80.f, 40.f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Car Dimensions")
    float CarMass = 1200.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Car Dimensions")
    float WheelRadius = 35.f;

    // ==================== SUSPENSION SETTINGS ====================

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Suspension", meta = (ClampMin = "10.0", ClampMax = "100.0"))
    float SuspensionRestLength = 50.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Suspension", meta = (ClampMin = "20.0", ClampMax = "150.0"))
    float SuspensionMaxLength = 70.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Suspension", meta = (ClampMin = "10000.0", ClampMax = "200000.0"))
    float SpringStrength = 55000.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Suspension", meta = (ClampMin = "1000.0", ClampMax = "20000.0"))
    float DamperStrength = 4500.f;

    // ==================== ENGINE SETTINGS ====================

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Engine", meta = (ClampMin = "1000.0", ClampMax = "50000.0"))
    float MaxEngineTorque = 8000.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Engine")
    float MaxRPM = 7000.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Engine")
    float IdleRPM = 1000.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Engine")
    float FinalDriveRatio = 3.5f;

    // ==================== GEAR RATIOS ====================

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Gears")
    TArray<float> GearRatios;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Gears", meta = (ClampMin = "0.5", ClampMax = "1.0"))
    float ShiftUpRPM = 0.85f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Gears", meta = (ClampMin = "0.1", ClampMax = "0.5"))
    float ShiftDownRPM = 0.25f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Gears")
    bool bAutoGearShift = true;

    // ==================== HANDLING SETTINGS ====================

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Handling", meta = (ClampMin = "0.5", ClampMax = "5.0"))
    float TireGrip = 2.5f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Handling", meta = (ClampMin = "1.0", ClampMax = "2.0"))
    float FrontGripMultiplier = 1.2f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Handling", meta = (ClampMin = "15.0", ClampMax = "60.0"))
    float MaxSteerAngle = 35.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Handling", meta = (ClampMin = "1.0", ClampMax = "15.0"))
    float SteerSpeed = 5.f;

    // ==================== DRIFT SETTINGS ====================

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drift", meta = (ClampMin = "0.1", ClampMax = "1.0"))
    float DriftGripMultiplier = 0.4f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drift", meta = (ClampMin = "1.0", ClampMax = "3.0"))
    float DriftSteerBoost = 1.5f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drift", meta = (ClampMin = "5.0", ClampMax = "50.0"))
    float MinDriftSpeed = 25.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drift", meta = (ClampMin = "5.0", ClampMax = "30.0"))
    float DriftEntryAngle = 12.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drift", meta = (ClampMin = "1.0", ClampMax = "15.0"))
    float DriftExitAngle = 5.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drift", meta = (ClampMin = "0.0", ClampMax = "200000.0"))
    float DriftKickImpulse = 80000.f;

    // ==================== CAMERA SETTINGS ====================

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera Settings")
    float CameraDistance = 600.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera Settings")
    float CameraPitch = -20.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera Settings")
    float CameraLagSpeed = 5.f;

    // ==================== DEBUG SETTINGS ====================

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug")
    bool bShowDebugLines = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug")
    bool bShowDebugText = false;

    // ==================== RUNTIME STATE ====================

    UPROPERTY(BlueprintReadOnly, Category = "Runtime State")
    float CurrentRPM = 0.f;

    UPROPERTY(BlueprintReadOnly, Category = "Runtime State")
    int32 CurrentGear = 2;

    UPROPERTY(BlueprintReadOnly, Category = "Runtime State")
    bool bIsDrifting = false;

    UPROPERTY(BlueprintReadOnly, Category = "Runtime State")
    float CurrentSpeedMS = 0.f;

    UPROPERTY(BlueprintReadOnly, Category = "Runtime State")
    float CurrentSpeedKMH = 0.f;

    UPROPERTY(BlueprintReadOnly, Category = "Runtime State")
    float CurrentSlipAngle = 0.f;

    UPROPERTY(BlueprintReadOnly, Category = "Runtime State")
    TArray<FWheelData> WheelDataArray;

    // ==================== INPUT STATE ====================

    UPROPERTY(BlueprintReadOnly, Category = "Input State")
    float ThrottleInput = 0.f;

    UPROPERTY(BlueprintReadOnly, Category = "Input State")
    float SteerInput = 0.f;

    UPROPERTY(BlueprintReadOnly, Category = "Input State")
    float CurrentSteerAngle = 0.f;

    UPROPERTY(BlueprintReadOnly, Category = "Input State")
    bool bHandbrakePressed = false;

    // ==================== PUBLIC FUNCTIONS ====================

    UFUNCTION(BlueprintCallable, Category = "Car Control")
    void ShiftUp();

    UFUNCTION(BlueprintCallable, Category = "Car Control")
    void ShiftDown();

    UFUNCTION(BlueprintCallable, Category = "Car Control")
    void SetGear(int32 NewGear);

    UFUNCTION(BlueprintCallable, Category = "Car Control")
    FString GetGearDisplayString() const;

    UFUNCTION(BlueprintCallable, Category = "Car Setup")
    void RefreshWheelPositions();

protected:
    // ==================== ENHANCED INPUT HANDLERS ====================

    void HandleThrottleInput(const FInputActionValue& Value);
    void HandleSteerInput(const FInputActionValue& Value);
    void HandleHandbrakeStarted(const FInputActionValue& Value);
    void HandleHandbrakeCompleted(const FInputActionValue& Value);
    void HandleShiftUp(const FInputActionValue& Value);
    void HandleShiftDown(const FInputActionValue& Value);

    // ==================== PHYSICS SYSTEMS ====================

    void UpdateSuspension(float DeltaTime);
    void UpdateSteering(float DeltaTime);
    void UpdateAcceleration(float DeltaTime);
    void UpdateDrift(float DeltaTime);
    void UpdateAutoGearShift();
    void UpdateWheelVisuals(float DeltaTime);

    // ==================== HELPER FUNCTIONS ====================

    float CalculateEngineTorque();
    float CalculateTorqueCurve(float NormalizedRPM);
    void CacheWheelReferences();

private:
    TArray<FVector> WheelOffsets;
    TArray<USceneComponent*> WheelSocketArray;
    TArray<UStaticMeshComponent*> WheelMeshArray;
};