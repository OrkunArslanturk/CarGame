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

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Input")
    UInputMappingContext* ControllerMappingContext;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Input")
    UInputAction* PauseAction;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Input")
    UInputAction* RespawnAction;

    UFUNCTION(BlueprintCallable, Category = "Game")
    void TogglePause();

    UFUNCTION(BlueprintCallable, Category = "Game")
    void RespawnCar();

    UPROPERTY(BlueprintReadOnly, Category = "State")
    bool bIsPaused = false;

    UPROPERTY(BlueprintReadOnly, Category = "Reference")
    AArcadeCar* ControlledCar;

    UPROPERTY(BlueprintReadWrite, Category = "Respawn")
    FTransform RespawnTransform;

protected:
    void HandlePause(const FInputActionValue& Value);
    void HandleRespawn(const FInputActionValue& Value);
};
