// RacingPlayerController.cpp
// Player controller with Enhanced Input System
// For AGP Racing Game Assignment - Futuregames 2026

#include "RacingPlayerController.h"
#include "ArcadeCar.h"
#include "EnhancedInputComponent.h"
#include "EnhancedInputSubsystems.h"
#include "InputAction.h"
#include "InputMappingContext.h"
#include "Kismet/GameplayStatics.h"

ARacingPlayerController::ARacingPlayerController()
{
    bShowMouseCursor = false;
}

void ARacingPlayerController::BeginPlay()
{
    Super::BeginPlay();

    // Set input mode to game only
    FInputModeGameOnly InputMode;
    SetInputMode(InputMode);

    // Add controller's mapping context
    if (UEnhancedInputLocalPlayerSubsystem* Subsystem = 
        ULocalPlayer::GetSubsystem<UEnhancedInputLocalPlayerSubsystem>(GetLocalPlayer()))
    {
        if (ControllerMappingContext)
        {
            // Add with priority 1 (higher than car's 0)
            Subsystem->AddMappingContext(ControllerMappingContext, 1);
            UE_LOG(LogTemp, Log, TEXT("Added Controller Input Mapping Context"));
        }
        else
        {
            UE_LOG(LogTemp, Warning, TEXT("ControllerMappingContext is not set! Assign it in Blueprint."));
        }
    }

    // Store initial spawn as respawn point
    if (GetPawn())
    {
        RespawnTransform = GetPawn()->GetActorTransform();
    }
}

void ARacingPlayerController::OnPossess(APawn* InPawn)
{
    Super::OnPossess(InPawn);

    ControlledCar = Cast<AArcadeCar>(InPawn);

    if (ControlledCar)
    {
        UE_LOG(LogTemp, Log, TEXT("RacingPlayerController possessed ArcadeCar"));
        RespawnTransform = ControlledCar->GetActorTransform();
    }
}

void ARacingPlayerController::SetupInputComponent()
{
    Super::SetupInputComponent();

    // Cast to Enhanced Input Component
    if (UEnhancedInputComponent* EnhancedInput = Cast<UEnhancedInputComponent>(InputComponent))
    {
        // Bind Pause
        if (PauseAction)
        {
            EnhancedInput->BindAction(PauseAction, ETriggerEvent::Started, this, &ARacingPlayerController::HandlePause);
        }

        // Bind Cycle Camera
        if (CycleCameraAction)
        {
            EnhancedInput->BindAction(CycleCameraAction, ETriggerEvent::Started, this, &ARacingPlayerController::HandleCycleCamera);
        }

        // Bind Respawn
        if (RespawnAction)
        {
            EnhancedInput->BindAction(RespawnAction, ETriggerEvent::Started, this, &ARacingPlayerController::HandleRespawn);
        }

        UE_LOG(LogTemp, Log, TEXT("Controller Enhanced Input bindings complete"));
    }
}

// ============================================================================
// ENHANCED INPUT HANDLERS
// ============================================================================

void ARacingPlayerController::HandlePause(const FInputActionValue& Value)
{
    TogglePause();
}

void ARacingPlayerController::HandleCycleCamera(const FInputActionValue& Value)
{
    CycleCamera();
}

void ARacingPlayerController::HandleRespawn(const FInputActionValue& Value)
{
    RespawnCar();
}

// ============================================================================
// PAUSE
// ============================================================================

void ARacingPlayerController::TogglePause()
{
    bIsGamePaused = !bIsGamePaused;

    if (bIsGamePaused)
    {
        UGameplayStatics::SetGamePaused(GetWorld(), true);
        bShowMouseCursor = true;
        
        FInputModeGameAndUI InputMode;
        InputMode.SetHideCursorDuringCapture(false);
        SetInputMode(InputMode);

        UE_LOG(LogTemp, Log, TEXT("Game Paused"));
    }
    else
    {
        UGameplayStatics::SetGamePaused(GetWorld(), false);
        bShowMouseCursor = false;
        
        FInputModeGameOnly InputMode;
        SetInputMode(InputMode);

        UE_LOG(LogTemp, Log, TEXT("Game Unpaused"));
    }
}

// ============================================================================
// CAMERA
// ============================================================================

void ARacingPlayerController::CycleCamera()
{
    if (!ControlledCar) return;

    CurrentCameraMode = (CurrentCameraMode + 1) % 3;

    USpringArmComponent* CameraArm = ControlledCar->CameraArm;
    if (!CameraArm) return;

    switch (CurrentCameraMode)
    {
        case 0: // Chase (default)
            CameraArm->TargetArmLength = ControlledCar->CameraDistance;
            CameraArm->SetRelativeRotation(FRotator(ControlledCar->CameraPitch, 0.f, 0.f));
            CameraArm->SetRelativeLocation(FVector(0.f, 0.f, 0.f));
            UE_LOG(LogTemp, Log, TEXT("Camera: Chase"));
            break;

        case 1: // Close chase
            CameraArm->TargetArmLength = ControlledCar->CameraDistance * 0.5f;
            CameraArm->SetRelativeRotation(FRotator(-10.f, 0.f, 0.f));
            CameraArm->SetRelativeLocation(FVector(0.f, 0.f, 50.f));
            UE_LOG(LogTemp, Log, TEXT("Camera: Close"));
            break;

        case 2: // Hood cam
            CameraArm->TargetArmLength = 0.f;
            CameraArm->SetRelativeRotation(FRotator(0.f, 0.f, 0.f));
            CameraArm->SetRelativeLocation(FVector(100.f, 0.f, 50.f));
            UE_LOG(LogTemp, Log, TEXT("Camera: Hood"));
            break;
    }
}

// ============================================================================
// RESPAWN
// ============================================================================

void ARacingPlayerController::RespawnCar()
{
    if (!ControlledCar) return;

    UBoxComponent* CarBody = ControlledCar->CarBody;
    if (CarBody)
    {
        CarBody->SetPhysicsLinearVelocity(FVector::ZeroVector);
        CarBody->SetPhysicsAngularVelocityInRadians(FVector::ZeroVector);
    }

    FVector RespawnLocation = RespawnTransform.GetLocation() + FVector(0.f, 0.f, 100.f);
    FRotator RespawnRotation = RespawnTransform.GetRotation().Rotator();

    ControlledCar->SetActorLocationAndRotation(RespawnLocation, RespawnRotation, false, nullptr, ETeleportType::ResetPhysics);

    ControlledCar->CurrentGear = 2;
    ControlledCar->bIsDrifting = false;

    UE_LOG(LogTemp, Log, TEXT("Car Respawned"));
}