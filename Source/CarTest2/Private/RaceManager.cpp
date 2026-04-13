// Fill out your copyright notice in the Description page of Project Settings.


#include "RaceManager.h"
#include "Kismet/GameplayStatics.h"
#include "Components/TextBlock.h"
#include "EngineUtils.h"
#include "ArcadeCar/ArcadeCar.h"

ARaceManager::ARaceManager()
{
	PrimaryActorTick.bCanEverTick = true;
}

void ARaceManager::BeginPlay()
{
	Super::BeginPlay();

	UE_LOG(LogTemp, Warning, TEXT("Cars found: %d"), Cars.Num());

	GetWorld()->GetTimerManager().SetTimer(
		InitTimer,
		this,
		&ARaceManager::InitCars,
		0.5f,
		false
	);

	if (RaceHUDClass)
	{
		RaceHUD = CreateWidget<UUserWidget>(GetWorld(), RaceHUDClass);

		if (RaceHUD)
		{
			RaceHUD->AddToViewport();
		}
	}

	int TotalCP = 0;

	for (TActorIterator<ARaceCheckpoint> It(GetWorld()); It; ++It)
	{
		TotalCP++;
	}

	for (AArcadeCar* Car : Cars)
	{
		Car->TotalCheckpoints = TotalCP;
	}
}

void ARaceManager::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	UE_LOG(LogTemp, Warning, TEXT("RaceManager Tick"));

	UpdateRanking();
}

void ARaceManager::InitCars()
{
	TArray<AActor*> Found;
	UGameplayStatics::GetAllActorsOfClass(GetWorld(), AArcadeCar::StaticClass(), Found);

	for (AActor* A : Found)
	{
		if (AArcadeCar* Car = Cast<AArcadeCar>(A))
		{
			Cars.Add(Car);

			UE_LOG(LogTemp, Warning, TEXT("Car added: %s"), *Car->DriverName);
		}
	}

	UE_LOG(LogTemp, Warning, TEXT("Cars found: %d"), Cars.Num());
}


void ARaceManager::UpdateRanking()
{
	Cars.Sort([](const AArcadeCar& A, const AArcadeCar& B)
	{
		float ScoreA =
			A.CurrentLap * 100000.f +
			A.CurrentCheckpoint * 1000.f +
			A.DistanceOnSpline;

		float ScoreB =
			B.CurrentLap * 100000.f +
			B.CurrentCheckpoint * 1000.f +
			B.DistanceOnSpline;
	});

	// DEBUG
	for (int i = 0; i < Cars.Num(); i++)
	{
		if (GEngine && Cars[i])
		{
			FString Name = Cars[i]->DriverName;

			FString Msg = FString::FromInt(i + 1) + TEXT(". ") + Name;

			GEngine->AddOnScreenDebugMessage(
				i,
				0.f,
				FColor::Green,
				Msg
			);
		}
	}

	for (AArcadeCar* Car : Cars)
	{
		if (Car && Car->IsPlayerControlled())
		{
			UTextBlock* LapTextBlock =
				Cast<UTextBlock>(RaceHUD->GetWidgetFromName(TEXT("LapText")));

			if (LapTextBlock)
			{
				FString LapStr = FString::Printf(TEXT("Lap: %d"), Car->CurrentLap);
				LapTextBlock->SetText(FText::FromString(LapStr));
			}

			break;
		}
	}


	if (RaceHUD)
	{
		UTextBlock* RankingTextBlock =
			Cast<UTextBlock>(RaceHUD->GetWidgetFromName(TEXT("RankingText")));

		if (RankingTextBlock)
		{
			RankingTextBlock->SetText(FText::FromString(BuildRankingText()));
		}

		if (!RankingTextBlock)
		{
			UE_LOG(LogTemp, Error, TEXT("RankingText NOT FOUND"));
		}
	}
}

FString ARaceManager::BuildRankingText()
{
	FString Result;

	for (int i = 0; i < Cars.Num(); i++)
	{
		if (!Cars[i]) continue;

		FString Name = Cars[i]->DriverName;

		Result += FString::FromInt(i + 1) + TEXT(". ") + Name + TEXT("\n");
	}

	return Result;
}
