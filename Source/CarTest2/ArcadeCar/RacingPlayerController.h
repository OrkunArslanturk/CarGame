// RacingPlayerController.h
// Player controller with Enhanced Input System
// For AGP Racing Game Assignment - Futuregames 2026

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/PlayerController.h"
#include "InputActionValue.h"
#include "RacingPlayerController.generated.h"

class AArcadeCar;
class UInputAction;
class UInputMappingContext;

UCLASS()
class CARTEST2_API ARacingPlayerController : public APlayerController
{
    GENERATED_BODY()

public:
    ARacingPlayerController();

    virtual void BeginPlay() override;
    virtual void SetupInputComponent() override;
    virtual void OnPossess(APawn* InPawn) override;

    // ==================== ENHANCED INPUT ====================

    // Controller's Input Mapping Context
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Input", meta = (DisplayName = "Controller Input Mapping Context"))
    UInputMappingContext* ControllerMappingContext;

    // Input Actions
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Input", meta = (DisplayName = "IA Pause"))
    UInputAction* PauseAction;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Input", meta = (DisplayName = "IA Cycle Camera"))
    UInputAction* CycleCameraAction;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Input", meta = (DisplayName = "IA Respawn"))
    UInputAction* RespawnAction;

    // ==================== PAUSE ====================

    UFUNCTION(BlueprintCallable, Category = "Game")
    void TogglePause();

    UPROPERTY(BlueprintReadOnly, Category = "Game")
    bool bIsGamePaused = false;

    // ==================== CAMERA ====================

    UFUNCTION(BlueprintCallable, Category = "Camera")
    void CycleCamera();

    UPROPERTY(BlueprintReadOnly, Category = "Camera")
    int32 CurrentCameraMode = 0;

    // ==================== RESPAWN ====================

    UFUNCTION(BlueprintCallable, Category = "Game")
    void RespawnCar();

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Respawn")
    FTransform RespawnTransform;

    // ==================== REFERENCE ====================

    UPROPERTY(BlueprintReadOnly, Category = "Reference")
    AArcadeCar* ControlledCar;

protected:
    // Enhanced Input Handlers
    void HandlePause(const FInputActionValue& Value);
    void HandleCycleCamera(const FInputActionValue& Value);
    void HandleRespawn(const FInputActionValue& Value);
};