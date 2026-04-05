#include "AI_Car.h"
#include "ChaosWheeledVehicleMovementComponent.h"
#include "RacingAIController.h"

AAI_Car::AAI_Car()
{
	PrimaryActorTick.bCanEverTick = true;

	AIControllerClass = ARacingAIController::StaticClass();
	AutoPossessAI = EAutoPossessAI::PlacedInWorldOrSpawned;
	

	auto* Vehicle =
		CastChecked<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent());

	Vehicle->Mass = 1800.f;

	Vehicle->WheelSetups.SetNum(4);

	// Vehicle->WheelSetups[0].WheelClass = UArcadeWheelFront::StaticClass();
	// Vehicle->WheelSetups[1].WheelClass = UArcadeWheelFront::StaticClass();
	// Vehicle->WheelSetups[2].WheelClass = UArcadeWheelRear::StaticClass();
	// Vehicle->WheelSetups[3].WheelClass = UArcadeWheelRear::StaticClass();

	Vehicle->EngineSetup.MaxTorque = EnginePower;
	Vehicle->EngineSetup.MaxRPM = MaxRPM;

	Vehicle->TransmissionSetup.bUseAutomaticGears = true;
	Vehicle->TransmissionSetup.FinalRatio = 4.f;

	Vehicle->DifferentialSetup.DifferentialType = EVehicleDifferential::RearWheelDrive;

	for (auto& Setup : Vehicle->WheelSetups)
	{
		
	}
}

void AAI_Car::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

void AAI_Car::SetThrottle(float Value)
{
	if (auto* Vehicle = Cast<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent()))
	{
		Vehicle->SetThrottleInput(Value);
	}
}

void AAI_Car::SetSteer(float Value)
{
	if (auto* Vehicle = Cast<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent()))
	{
		Vehicle->SetSteeringInput(Value);
	}
}