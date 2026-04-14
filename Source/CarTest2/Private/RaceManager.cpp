#include "RaceManager.h"
#include "RaceCheckpoint.h"
#include "Kismet/GameplayStatics.h"
#include "Components/TextBlock.h"
#include "ArcadeCar/ArcadeCar.h"

ARaceManager::ARaceManager()
{
	PrimaryActorTick.bCanEverTick = true;
}

void ARaceManager::BeginPlay()
{
	Super::BeginPlay();

	// UI
	if (RaceHUDClass)
	{
		RaceHUD = CreateWidget<UUserWidget>(GetWorld(), RaceHUDClass);

		if (RaceHUD)
		{
			RaceHUD->AddToViewport();
		}
	}

	// Delay (For finding car after spawn)
	GetWorld()->GetTimerManager().SetTimer(
		InitTimer,
		this,
		&ARaceManager::InitCars,
		0.5f,
		false
	);
}

void ARaceManager::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

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

			// UE_LOG(LogTemp, Warning, TEXT("Car added: %s"), *Car->DriverName);
		}
	}

	UE_LOG(LogTemp, Warning, TEXT("Cars found: %d"), Cars.Num());

	// Checkpoint find
	TArray<AActor*> FoundCPs;
	UGameplayStatics::GetAllActorsOfClass(GetWorld(), ARaceCheckpoint::StaticClass(), FoundCPs);

	int TotalCP = FoundCPs.Num();

	// UE_LOG(LogTemp, Warning, TEXT("Total Checkpoints: %d"), TotalCP);

	for (AArcadeCar* Car : Cars)
	{
		if (Car)
		{
			Car->TotalCheckpoints = TotalCP;
		}
	}
}

void ARaceManager::UpdateRanking()
{
	if (!bRaceStarted)
	{
		return;
	}
	
	Cars.RemoveAll([](AArcadeCar* Car)
	{
		return Car == nullptr || !IsValid(Car);
	});
	
	if (bRaceFinished) return;

	Cars.Sort([](const AArcadeCar& A, const AArcadeCar& B)
	{
		return A.Progress > B.Progress;
	});

	// DEBUG RANK
	// for (int i = 0; i < Cars.Num(); i++)
	// {
	// 	if (GEngine && Cars[i])
	// 	{
	// 		FString Msg =
	// 			FString::FromInt(i + 1) + TEXT(". ") + Cars[i]->DriverName;
	//
	// 		GEngine->AddOnScreenDebugMessage(
	// 			i,
	// 			0.f,
	// 			FColor::Green,
	// 			Msg
	// 		);
	// 	}
	// }

	// Player UI
	for (AArcadeCar* Car : Cars)
	{
		if (Car && Car->IsPlayerControlled())
		{
			if (!RaceHUD) return;

			UTextBlock* LapTextBlock =
				Cast<UTextBlock>(RaceHUD->GetWidgetFromName(TEXT("LapText")));

			if (LapTextBlock)
			{
				FString LapStr =
					FString::Printf(TEXT("Lap: %d"), Car->CurrentLap);

				LapTextBlock->SetText(FText::FromString(LapStr));
			}

			break;
		}
	}

	// Ranking UI
	if (RaceHUD)
	{
		UTextBlock* RankingTextBlock =
			Cast<UTextBlock>(RaceHUD->GetWidgetFromName(TEXT("RankingText")));

		if (RankingTextBlock)
		{
			RankingTextBlock->SetText(
				FText::FromString(BuildRankingText())
			);
		}
		else
		{
			// UE_LOG(LogTemp, Error, TEXT("RankingText null"));
		}
	}

	// Player finish
	if (!bRaceFinished)
	{
		APlayerController* PC = GetWorld()->GetFirstPlayerController();

		if (PC && PC->GetPawn())
		{
			AArcadeCar* PlayerCar = Cast<AArcadeCar>(PC->GetPawn());

			if (PlayerCar)
			{
				UE_LOG(LogTemp, Warning, TEXT("CHECK FINISH (PLAYER): %d / %d"),
				       PlayerCar->CurrentLap,
				       MaxLap);

				if (PlayerCar->CurrentLap >= MaxLap && PlayerCar->CurrentCheckpoint == 0)
				{
					bRaceFinished = true;

					// UE_LOG(LogTemp, Warning, TEXT("Race finish"));

					ShowWinnerUI();
				}
			}
		}
	}
}

void ARaceManager::ShowWinnerUI()
{
	if (!WinnerHUDClass) return;

	WinnerHUD = CreateWidget<UUserWidget>(GetWorld(), WinnerHUDClass);

	if (WinnerHUD)
	{
		WinnerHUD->AddToViewport();
	}

	// Winner
	AArcadeCar* WinnerCar = nullptr;

	if (Cars.Num() > 0)
	{
		WinnerCar = Cars[0];
	}

	// Winner text
	if (WinnerHUD && WinnerCar)
	{
		UTextBlock* WinnerText =
			Cast<UTextBlock>(WinnerHUD->GetWidgetFromName(TEXT("WinnerText")));

		if (WinnerText)
		{
			FString Text = TEXT("WINNER: ") + WinnerCar->DriverName;
			WinnerText->SetText(FText::FromString(Text));
		}
	}

	// Ranking text
	UTextBlock* RankingText =
		Cast<UTextBlock>(WinnerHUD->GetWidgetFromName(TEXT("RankingText")));

	if (RankingText)
	{
		RankingText->SetText(FText::FromString(BuildRankingText()));
	}

	// input
	APlayerController* PC = GetWorld()->GetFirstPlayerController();
	if (PC && WinnerHUD)
	{
		PC->bShowMouseCursor = true;

		FInputModeUIOnly Mode;
		Mode.SetWidgetToFocus(WinnerHUD->TakeWidget());
		Mode.SetLockMouseToViewportBehavior(EMouseLockMode::DoNotLock);

		PC->SetInputMode(Mode);
	}
	
	// slow motion
	UGameplayStatics::SetGlobalTimeDilation(GetWorld(), 0.2f);

	// motor voice fade out
	for (AArcadeCar* Car : Cars)
	{
		if (Car && Car->EngineAudioComponent)
		{
			Car->EngineAudioComponent->FadeOut(0.5f, 0.0f);
		}
	}
	
	SetActorTickEnabled(false);
}

FString ARaceManager::BuildRankingText()
{
	FString Result;

	for (int i = 0; i < Cars.Num(); i++)
	{
		if (!Cars[i]) continue;

		Result +=
			FString::FromInt(i + 1) +
			TEXT(". ") +
			Cars[i]->DriverName +
			TEXT("\n");
	}

	return Result;
}
