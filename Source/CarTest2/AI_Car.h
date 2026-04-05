#pragma once

#include "CoreMinimal.h"
#include "WheeledVehiclePawn.h"
#include "AI_Car.generated.h"

UCLASS()
class CARTEST2_API AAI_Car : public AWheeledVehiclePawn
{
	GENERATED_BODY()

public:

	AAI_Car();

	virtual void Tick(float DeltaTime) override;

	void SetThrottle(float Value);
	void SetSteer(float Value);

protected:

	UPROPERTY(EditAnywhere)
	float EnginePower = 500.f;

	UPROPERTY(EditAnywhere)
	float MaxRPM = 5500.f;
};