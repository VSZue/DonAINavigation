// The MIT License(MIT)
//
// Copyright(c) 2015 Venugopalan Sreedharan
//
// Contributors: This EQS code was kindly contributed by Vladimir Ivanov (Github @ArCorvus)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), 
// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, 
// and / or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
// WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include "EnvironmentQuery/DonEnvQueryTest_Navigation.h"
#include "../DonAINavigationPrivatePCH.h"

#include "DonNavigationManager.h"
#include "AIController.h"
#include "EnvironmentQuery/Items/EnvQueryItemType_VectorBase.h"
#include "EnvironmentQuery/Contexts/EnvQueryContext_Querier.h"
#include "EnvironmentQuery/Items/EnvQueryItemType_Point.h"



UDonEnvQueryTest_Navigation::UDonEnvQueryTest_Navigation(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{
	Cost = EEnvTestCost::High;
	TestPurpose = EEnvTestPurpose::Filter;
	FilterType = EEnvTestFilterType::Match;
	ValidItemType = UEnvQueryItemType_VectorBase::StaticClass();
	//Context = UEnvQueryContext_Querier::StaticClass();
	SetWorkOnFloatValues(false);
	RandomLocationMaxAttempts = 3;
	RandomLocationRadius = 100.f;
}

void UDonEnvQueryTest_Navigation::RunTest(FEnvQueryInstance& QueryInstance) const
{
	UObject* QueryOwner = QueryInstance.Owner.Get();
	if (QueryOwner == nullptr) {
		return;
	}

	const AAIController *AICon = Cast<AAIController>(QueryOwner);
	AActor *ownerActor = nullptr;
	if (AICon) {
		ownerActor = AICon->GetPawn();
	}
	else {
		ownerActor = Cast<AActor>(QueryOwner);
	}
	if (!ownerActor)
		return;

	BoolValue.BindData(QueryOwner, QueryInstance.QueryID);
	bool bWantsValid = BoolValue.GetValue();

	ADonNavigationManager * DonNav = nullptr;

	for (TActorIterator<ADonNavigationManager> It(ownerActor->GetWorld(), ADonNavigationManager::StaticClass()); It; ++It) {
		const ADonNavigationManager *NavMgr = *It;
		if (NavMgr->IsLocationWithinNavigableWorld(ownerActor->GetActorLocation())) {
			DonNav = *It;
			break;
		}
	}

	if (!DonNav)
		return;

	UEnvQueryItemType_Point* ItemTypeCDO = nullptr;
	
	if (bSearchRandomLocation) {
		const auto &itemType = QueryInstance.ItemType;
		if (itemType && itemType->IsChildOf(UEnvQueryItemType_Point::StaticClass())) {
			ItemTypeCDO = QueryInstance.ItemType->GetDefaultObject<UEnvQueryItemType_Point>();
		}
	}

	for (FEnvQueryInstance::ItemIterator It(this, QueryInstance); It; ++It) {
		const FVector ItemLocation = GetItemLocation(QueryInstance, It.GetIndex());

		bool ItemValid = false;
		ItemValid = DonNav->CanNavigate(ItemLocation);
		FVector NavLocation;
		if (bSearchRandomLocation) {
			if (!ItemValid) {
				NavLocation = DonNav->FindRandomPointAroundOriginInNavWorld(ownerActor, ItemLocation, RandomLocationRadius, ItemValid, -1.f, 15.f, RandomLocationMaxAttempts);
			}

			if (ItemTypeCDO) {
				ItemTypeCDO->SetItemNavLocation(It.GetItemData(), FNavLocation(NavLocation));
			}
		}

		It.SetScore(TestPurpose, FilterType, ItemValid, bWantsValid);
	}
}

FText UDonEnvQueryTest_Navigation::GetDescriptionTitle() const
{
	return FText::FromString(FString::Printf(TEXT("Don Navigation: Item is navigable")));
}

FText UDonEnvQueryTest_Navigation::GetDescriptionDetails() const
{
	return FText::Format(FText::FromString("{0}\nDoN Location is navigable"),
		DescribeFloatTestParams());
}