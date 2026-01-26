#include "RacingPlayerController.h"
#include "ArcadeCar.h"
#include "EnhancedInputComponent.h"
#include "EnhancedInputSubsystems.h"
#include "Kismet/GameplayStatics.h"
#include "Blueprint/UserWidget.h"
#include "ChaosWheeledVehicleMovementComponent.h"

ARacingPlayerController::ARacingPlayerController()
{
    bShowMouseCursor = false;
}

void ARacingPlayerController::BeginPlay()
{
    Super::BeginPlay();

    SetInputMode(FInputModeGameOnly());

    if (UEnhancedInputLocalPlayerSubsystem* Subsystem =
        ULocalPlayer::GetSubsystem<UEnhancedInputLocalPlayerSubsystem>(GetLocalPlayer()))
    {
        if (ControllerMappingContext)
        {
            Subsystem->AddMappingContext(ControllerMappingContext, 1);
        }
    }

    if (GetPawn())
    {
        RespawnTransform = GetPawn()->GetActorTransform();
    }

    ////////////////////////////////////////////////////////////////////////////
    if (HUDWidgetClass)
    {
        UUserWidget* HUD = CreateWidget<UUserWidget>(this, HUDWidgetClass);
        HUD->AddToViewport();
    }
    ////////////////////////////////////////////////////////////////////////////

}

void ARacingPlayerController::OnPossess(APawn* InPawn)
{
    Super::OnPossess(InPawn);

    ControlledCar = Cast<AArcadeCar>(InPawn);
    if (ControlledCar)
    {
        RespawnTransform = ControlledCar->GetActorTransform();
    }
}

void ARacingPlayerController::SetupInputComponent()
{
    Super::SetupInputComponent();

    if (UEnhancedInputComponent* Input = Cast<UEnhancedInputComponent>(InputComponent))
    {
        if (PauseAction)
        {
            Input->BindAction(PauseAction, ETriggerEvent::Started, this, &ARacingPlayerController::HandlePause);
        }
        if (RespawnAction)
        {
            Input->BindAction(RespawnAction, ETriggerEvent::Started, this, &ARacingPlayerController::HandleRespawn);
        }
    }
}

void ARacingPlayerController::HandlePause(const FInputActionValue& Value)
{
    TogglePause();
}

void ARacingPlayerController::HandleRespawn(const FInputActionValue& Value)
{
    RespawnCar();
}

void ARacingPlayerController::TogglePause()
{
    bIsPaused = !bIsPaused;

    if (bIsPaused)
    {
        UGameplayStatics::SetGamePaused(GetWorld(), true);
        bShowMouseCursor = true;
        SetInputMode(FInputModeGameAndUI().SetHideCursorDuringCapture(false));
    }
    else
    {
        UGameplayStatics::SetGamePaused(GetWorld(), false);
        bShowMouseCursor = false;
        SetInputMode(FInputModeGameOnly());
    }
}

void ARacingPlayerController::RespawnCar()
{
    if (!ControlledCar) return;

    if (UChaosWheeledVehicleMovementComponent* Vehicle =
        Cast<UChaosWheeledVehicleMovementComponent>(ControlledCar->GetVehicleMovementComponent()))
    {
        Vehicle->StopMovementImmediately();
    }

    const FVector SpawnPos = RespawnTransform.GetLocation() + FVector(0.f, 0.f, 100.f);
    const FRotator SpawnRot = RespawnTransform.GetRotation().Rotator();

    ControlledCar->SetActorLocationAndRotation(SpawnPos, SpawnRot, false, nullptr, ETeleportType::ResetPhysics);
}
