#include "AIArcadeCar.h"
#include "ChaosWheeledVehicleMovementComponent.h"

AAIArcadeCar::AAIArcadeCar()
{
	UChaosWheeledVehicleMovementComponent* Vehicle =
		CastChecked<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent());

	Vehicle->Mass = 2000.f;
	Vehicle->DragCoefficient = 0.35f;

	Vehicle->EngineSetup.MaxTorque = EnginePower;
	Vehicle->EngineSetup.MaxRPM = MaxRPM;
	Vehicle->EngineSetup.EngineIdleRPM = 900.f;

	Vehicle->TransmissionSetup.bUseAutomaticGears = true;
	Vehicle->TransmissionSetup.bUseAutoReverse = true;
	Vehicle->TransmissionSetup.FinalRatio = 4.f;

	Vehicle->DifferentialSetup.DifferentialType = EVehicleDifferential::RearWheelDrive;

	Vehicle->SteeringSetup.SteeringType = ESteeringType::AngleRatio;
	Vehicle->SteeringSetup.AngleRatio = 1.f;
}

void AAIArcadeCar::BeginPlay()
{
	Super::BeginPlay();
}

void AAIArcadeCar::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

void AAIArcadeCar::SetThrottle(float Value)
{
	if (auto* Vehicle = Cast<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent()))
	{
		Vehicle->SetThrottleInput(Value);
	}
}

void AAIArcadeCar::SetSteer(float Value)
{
	if (auto* Vehicle = Cast<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent()))
	{
		Vehicle->SetSteeringInput(Value);
	}
}

void AAIArcadeCar::SetBrake(float Value)
{
	if (auto* Vehicle = Cast<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent()))
	{
		Vehicle->SetBrakeInput(Value);
	}
}