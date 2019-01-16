// The MIT License(MIT)
//
// Copyright(c) 2015 Venugopalan Sreedharan
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

#include "BehaviorTree/BTTask_FlyTo.h"
#include "../DonAINavigationPrivatePCH.h"

#include "DonNavigatorInterface.h"

#include "BehaviorTree/BlackboardComponent.h"
#include "BehaviorTree/Blackboard/BlackboardKeyType_Object.h"
#include "BehaviorTree/Blackboard/BlackboardKeyType_Vector.h"
#include "BehaviorTree/BTFunctionLibrary.h"

#include "Runtime/AIModule/Classes/AIController.h"
#include "VisualLogger/VisualLogger.h"

UBTTask_FlyTo::UBTTask_FlyTo(const FObjectInitializer& ObjectInitializer) 
	: Super(ObjectInitializer)
	, bRecalcPathOnDestinationChanged(false)
	, RecalculatePathTolerance(50.f)
{
	NodeName = "Fly To";
	bNotifyTick = true;
	
	FlightLocationKey.AddVectorFilter(this,		    GET_MEMBER_NAME_CHECKED(UBTTask_FlyTo, FlightLocationKey));
	FlightResultKey.AddBoolFilter(this,				GET_MEMBER_NAME_CHECKED(UBTTask_FlyTo, FlightResultKey));
	KeyToFlipFlopWhenTaskExits.AddBoolFilter(this,  GET_MEMBER_NAME_CHECKED(UBTTask_FlyTo, KeyToFlipFlopWhenTaskExits));

	FlightLocationKey.AllowNoneAsValue(true);
	FlightResultKey.AllowNoneAsValue(true);
	KeyToFlipFlopWhenTaskExits.AllowNoneAsValue(true);
}

void UBTTask_FlyTo::InitializeFromAsset(UBehaviorTree& Asset)
{
	Super::InitializeFromAsset(Asset);

	auto blackboard = GetBlackboardAsset();
	if (!blackboard)
		return;
	
	FlightLocationKey.ResolveSelectedKey(*blackboard);
}

EBTNodeResult::Type UBTTask_FlyTo::ExecuteTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory)
{
	EBTNodeResult::Type NodeResult = SchedulePathfindingRequest(OwnerComp, NodeMemory);

	if (bRecalcPathOnDestinationChanged && (NodeResult == EBTNodeResult::InProgress))
	{
		UBlackboardComponent* BlackboardComp = OwnerComp.GetBlackboardComponent();
		auto myMemory = (FBT_FlyToTarget*)NodeMemory;
		if (ensure(BlackboardComp))
		{
			if (myMemory->BBObserverDelegateHandle.IsValid())
			{
				UE_VLOG(OwnerComp.GetAIOwner(), LogBehaviorTree, Warning, TEXT("UBTTask_MoveTo::ExecuteTask \'%s\' Old BBObserverDelegateHandle is still valid! Removing old Observer."), *GetNodeName());
				BlackboardComp->UnregisterObserver(FlightLocationKey.GetSelectedKeyID(), myMemory->BBObserverDelegateHandle);
			}
			myMemory->BBObserverDelegateHandle = BlackboardComp->RegisterObserver(FlightLocationKey.GetSelectedKeyID(), this, FOnBlackboardChangeNotification::CreateUObject(this, &UBTTask_FlyTo::OnBlackboardValueChange));
		}
	}

	return NodeResult;
}

EBTNodeResult::Type UBTTask_FlyTo::SchedulePathfindingRequest(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory)
{
	auto pawn        =  OwnerComp.GetAIOwner()->GetPawn();
	auto myMemory    =  (FBT_FlyToTarget*)NodeMemory;
	auto blackboard  =  pawn ? pawn->GetController()->FindComponentByClass<UBlackboardComponent>() : NULL;

	/*const float currentTime = GetWorld()->GetRealTimeSeconds();
	const float lastRequestTimestamp = LastRequestTimestamps.FindOrAdd(pawn);
	if (currentTime - lastRequestTimestamp < RequestThrottleInterval)
		return EBTNodeResult::Failed; // throttled
	else
		LastRequestTimestamps.Add(pawn, currentTime); //LastRequestTimestamp = currentTime;
		*/
	NavigationManager =  UDonNavigationHelper::DonNavigationManagerForActor(pawn);
	if (NavigationManager->HasTask(pawn) && !QueryParams.bForceRescheduleQuery)
		return EBTNodeResult::Failed; // early exit instead of going through the manager's internal checks and fallback via HandleTaskFailure (which isn't appropriate here)
	
	// Validate internal state:
	if (!pawn || !myMemory || !blackboard || !NavigationManager)
	{
		UE_LOG(DoNNavigationLog, Log, TEXT("BTTask_FlyTo has invalid data for AI Pawn or NodeMemory or NavigationManager. Unable to proceed."));

		return HandleTaskFailure(OwnerComp, NodeMemory, blackboard);
	}
	
	// Validate blackboard key data:
	if(FlightLocationKey.SelectedKeyType != UBlackboardKeyType_Vector::StaticClass())
	{
		UE_LOG(DoNNavigationLog, Log, TEXT("Invalid FlightLocationKey. Expected Vector type, found %s"), *(FlightLocationKey.SelectedKeyType ? FlightLocationKey.SelectedKeyType->GetName() : FString("?")));
		return HandleTaskFailure(OwnerComp, NodeMemory, blackboard);
	}

	// Prepare input:
	myMemory->Reset();	
	myMemory->Metadata.ActiveInstanceIdx = OwnerComp.GetActiveInstanceIdx();
	myMemory->Metadata.OwnerComp = &OwnerComp;
	myMemory->QueryParams = QueryParams;
	myMemory->QueryParams.CustomDelegatePayload = &myMemory->Metadata;
	myMemory->bIsANavigator = pawn->GetClass()->ImplementsInterface(UDonNavigator::StaticClass());

	FVector flightDestination = blackboard->GetValueAsVector(FlightLocationKey.SelectedKeyName);
	myMemory->TargetLocation = flightDestination;

	// Bind result notification delegate:
	FDoNNavigationResultHandler resultHandler;
	resultHandler.BindDynamic(this, &UBTTask_FlyTo::Pathfinding_OnFinish);

	// Bind dynamic collision updates delegate:		
	myMemory->DynamicCollisionListener.BindDynamic(this, &UBTTask_FlyTo::Pathfinding_OnDynamicCollisionAlert);

	// Schedule task:
	bool bTaskScheduled = false;
	bTaskScheduled = NavigationManager->SchedulePathfindingTask(pawn, flightDestination, myMemory->QueryParams, DebugParams, resultHandler, myMemory->DynamicCollisionListener);

	if (bTaskScheduled)
	{
		return EBTNodeResult::InProgress;
	}
	else
		return HandleTaskFailure(OwnerComp, NodeMemory, blackboard);
}

void UBTTask_FlyTo::AbortPathfindingRequest(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory)
{
	APawn* pawn = OwnerComp.GetAIOwner()->GetPawn();
	FBT_FlyToTarget* myMemory = (FBT_FlyToTarget*)NodeMemory;

	if (NavigationManager && pawn && myMemory)
	{
		NavigationManager->AbortPathfindingTask(pawn);

		// Unregister all dynamic collision listeners. We've completed our task and are no longer interested in listening to these:		
		NavigationManager->StopListeningToDynamicCollisionsForPath(myMemory->DynamicCollisionListener, myMemory->QueryResults);
	}
}

FBT_FlyToTarget* UBTTask_FlyTo::TaskMemoryFromGenericPayload(void* GenericPayload)
{
	// A brief explanation of the "NodeMemory" and "Generic Payload" business:

	// AFAICT, Behavior tree tasks operate as singletons and internally maintain an instance memory stack which maps instance data for every AI currently running this task.
	// So the BT Task itself is shared by all AI pawns and does not have sufficient information to handle our result delegate on its own.
	// Because of this, we use a custom delegate payload (which we passed earlier in "ExecuteTask") to lookup the actual AI owner and the correct NodeMemory 
	// inside which we store the pathfinding results.

	auto payload = static_cast<FBT_FlyToTarget_Metadata*> (GenericPayload);
	auto ownerComp = (payload && payload->OwnerComp.IsValid()) ? payload->OwnerComp.Get() : NULL;

	// Is the pawn's BrainComponent still alive and valid?
	if (!ownerComp)
		return NULL;

	// Is it still working on this task or has it moved on to another one?
	if (ownerComp->GetTaskStatus(this) != EBTTaskStatus::Active)
	{
		UE_LOG(DoNNavigationLog, Warning, TEXT("Task (Fly To) is not active."));
		return nullptr;
	}

	// Validations passed, should be safe to work on NodeMemory now:

	auto nodeMemory = ownerComp->GetNodeMemory(this, ownerComp->GetActiveInstanceIdx());
	auto myMemory = nodeMemory ? reinterpret_cast<FBT_FlyToTarget*> (nodeMemory) : NULL;

	return myMemory;
}

void UBTTask_FlyTo::Pathfinding_OnFinish(const FDoNNavigationQueryData& Data)
{	
	auto myMemory = TaskMemoryFromGenericPayload(Data.QueryParams.CustomDelegatePayload);
	if (!myMemory)
		return;

	auto ownerComp = myMemory->Metadata.OwnerComp.Get();

	// Store query results:	
	myMemory->QueryResults = Data;

	// Validate results:
	if (!Data.PathSolutionOptimized.Num())
	{
		if (bTeleportToDestinationUponFailure && ownerComp)
		{
			TeleportAndExit(*ownerComp, false);
			myMemory->QueryResults.QueryStatus = EDonNavigationQueryStatus::Success;
		}
		else
		{
			UE_LOG(DoNNavigationLog, Log, TEXT("Found empty pathsolution in Fly To node. Aborting task..."));
			myMemory->QueryResults.QueryStatus = EDonNavigationQueryStatus::Failure;
		}

		return;
	}

	// Inform pawn owner that we're about to start locomotion!
	if (myMemory->bIsANavigator)
	{
		if (!ownerComp)
			return;

		APawn* pawn = ownerComp->GetAIOwner()->GetPawn();

		IDonNavigator::Execute_OnLocomotionBegin(pawn);

		//UE_LOG(DoNNavigationLog, Verbose, TEXT("Segment 0"));
		IDonNavigator::Execute_OnNextSegment(pawn, myMemory->QueryResults.PathSolutionOptimized[0]);
	}
	
}

void UBTTask_FlyTo::Pathfinding_OnDynamicCollisionAlert(const FDonNavigationDynamicCollisionPayload& Data)
{
	auto myMemory = TaskMemoryFromGenericPayload(Data.CustomDelegatePayload);
	if (!myMemory)
		return;

	myMemory->bSolutionInvalidatedByDynamicObstacle = true;

}

void UBTTask_FlyTo::TickTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, float DeltaSeconds)
{
	FBT_FlyToTarget* myMemory = (FBT_FlyToTarget*)NodeMemory;	

	APawn* pawn = OwnerComp.GetAIOwner()->GetPawn();
	NavigationManager = UDonNavigationHelper::DonNavigationManagerForActor(pawn);	

	if (EDonNavigationQueryStatus::InProgress == myMemory->QueryResults.QueryStatus)
		return;

	switch (myMemory->QueryResults.QueryStatus)
	{

	case EDonNavigationQueryStatus::Success:

		// Is our path solution no longer valid?
		if (myMemory->bSolutionInvalidatedByDynamicObstacle)
		{	
			NavigationManager->StopListeningToDynamicCollisionsForPath(myMemory->DynamicCollisionListener, myMemory->QueryResults);

			// Recalculate path (a dynamic obstacle has probably come out of nowhere and invalidated our current solution)
			EBTNodeResult::Type bRes = SchedulePathfindingRequest(OwnerComp, NodeMemory);
			if (bRes == EBTNodeResult::Failed) 
				FinishLatentTask(OwnerComp, EBTNodeResult::Failed);
			
			break;
		}

		if (myMemory->bTargetLocationChanged)
		{
			EBTNodeResult::Type bRes = SchedulePathfindingRequest(OwnerComp, NodeMemory);
			if (bRes == EBTNodeResult::Failed) 
				FinishLatentTask(OwnerComp, EBTNodeResult::Failed);

			break;
		}

		// Move along the path solution towards our goal:
		TickPathNavigation(OwnerComp, myMemory, DeltaSeconds);

		break;

	// For advanced usecases we could support partial path traversal, etc (so we slowly progress towards the goal
	// with each cycle of query-timeout->partial-reschedule->partial-navigate->query-timeout->partial-reschedule, etc)
	// but for now, let's just keep things simple.

	case EDonNavigationQueryStatus::QueryHasNoSolution:
	case EDonNavigationQueryStatus::TimedOut:
	case EDonNavigationQueryStatus::Failure:
		HandleTaskFailureAndExit(OwnerComp, NodeMemory);
		break;

	default: // currently covers "Unscheduled" which in turn is acting as the "In Progress" state in the current flow
		if (!NavigationManager->HasTask(pawn))
			HandleTaskFailureAndExit(OwnerComp, NodeMemory);
	}
}

void UBTTask_FlyTo::TickPathNavigation(UBehaviorTreeComponent& OwnerComp, FBT_FlyToTarget* MyMemory, float DeltaSeconds)
{
	const auto& queryResults = MyMemory->QueryResults;

	APawn* pawn = OwnerComp.GetAIOwner()->GetPawn();
	
	if (DebugParams.bVisualizePawnAsVoxels)
		NavigationManager->Debug_DrawVoxelCollisionProfile(Cast<UPrimitiveComponent>(pawn->GetRootComponent()));

	if (!queryResults.PathSolutionOptimized.IsValidIndex(MyMemory->solutionTraversalIndex))
	{
		HandleTaskFailureAndExit(OwnerComp, (uint8*) (MyMemory)); // observed after recent multi-threading rewrite. Need to watch this branch closely and understand why it occurs!
		return;
	}
	
	FVector deltaToNextNode = queryResults.PathSolutionOptimized[MyMemory->solutionTraversalIndex] - pawn->GetActorLocation();
	FVector nextNodeDirection = deltaToNextNode.GetSafeNormal();

	//auto navigator = Cast<IDonNavigator>(pawn);

	// Add movement input:
	if (MyMemory->bIsANavigator)
	{
		// Customized movement handling for advanced users:
		IDonNavigator::Execute_AddMovementInputCustom(pawn, nextNodeDirection, 1.f);
	}
	else
	{
		// Default movement (handled by Pawn or Character class)
		pawn->AddMovementInput(nextNodeDirection, 1.f);
	}

	//UE_LOG(DoNNavigationLog, Verbose, TEXT("Segment %d Distance: %f"), MyMemory->solutionTraversalIndex, flightDirection.Size());

	// Reached next segment:
	if (deltaToNextNode.Size() <= MinimumProximityRequired)
	{
		// Goal reached?
		if (MyMemory->solutionTraversalIndex == queryResults.PathSolutionOptimized.Num() - 1)
		{
			auto controller = pawn->GetController();
			auto blackboard = controller ? controller->FindComponentByClass<UBlackboardComponent>() : NULL;
			if (blackboard)
			{
				blackboard->SetValueAsBool(FlightResultKey.SelectedKeyName, true);
				blackboard->SetValueAsBool(KeyToFlipFlopWhenTaskExits.SelectedKeyName, !blackboard->GetValueAsBool(KeyToFlipFlopWhenTaskExits.SelectedKeyName));
			}

			// Unregister all dynamic collision listeners. We've completed our task and are no longer interested in listening to these:
			NavigationManager->StopListeningToDynamicCollisionsForPath(MyMemory->DynamicCollisionListener, queryResults);

			// Inform the pawn owner that we're stopping locomotion (having reached the destination!)
			if (MyMemory->bIsANavigator)
				IDonNavigator::Execute_OnLocomotionEnd(pawn, true /*success*/);

			FinishLatentTask(OwnerComp, EBTNodeResult::Succeeded);

			return;
		}
		else
		{
			MyMemory->solutionTraversalIndex++;

			// Because we just completed a segment, we should stop listening to collisions on the previous voxel. 
			// If not, a pawn may needlessly recalculate its solution when a obstacle far behind it intrudes on a voxel it has already visited.
			if (!NavigationManager->bIsUnbound && queryResults.VolumeSolutionOptimized.IsValidIndex(MyMemory->solutionTraversalIndex - 1))
				NavigationManager->StopListeningToDynamicCollisionsForPathIndex(MyMemory->DynamicCollisionListener, queryResults, MyMemory->solutionTraversalIndex - 1);

			if (MyMemory->bIsANavigator)
			{
				if (!MyMemory->Metadata.OwnerComp.IsValid()) // edge case identified during high-speed time dilation. Need to gain a better understanding of exactly what triggers this issue.
				{
					HandleTaskFailureAndExit(OwnerComp, (uint8*)MyMemory);
					return;
				}

				if (queryResults.PathSolutionOptimized.IsValidIndex(MyMemory->solutionTraversalIndex))
				{
					FVector nextPoint = queryResults.PathSolutionOptimized[MyMemory->solutionTraversalIndex];
					//UE_LOG(DoNNavigationLog, Verbose, TEXT("Segment %d, %s"), MyMemory->solutionTraversalIndex, *nextPoint.ToString());

					IDonNavigator::Execute_OnNextSegment(pawn, nextPoint);
				}				
			}
			
		}
	}
}


void UBTTask_FlyTo::OnTaskFinished(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, EBTNodeResult::Type TaskResult)
{
	FBT_FlyToTarget* myMemory = (FBT_FlyToTarget*)NodeMemory;

	UBlackboardComponent* BlackboardComp = OwnerComp.GetBlackboardComponent();
	if (ensure(BlackboardComp) && myMemory->BBObserverDelegateHandle.IsValid())
		BlackboardComp->UnregisterObserver(FlightLocationKey.GetSelectedKeyID(), myMemory->BBObserverDelegateHandle);

	myMemory->BBObserverDelegateHandle.Reset();

	Super::OnTaskFinished(OwnerComp, NodeMemory, TaskResult);
}

EBTNodeResult::Type UBTTask_FlyTo::HandleTaskFailure(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, UBlackboardComponent* Blackboard)
{
	AbortPathfindingRequest(OwnerComp, NodeMemory);

	auto myMemory = NodeMemory ? reinterpret_cast<FBT_FlyToTarget*> (NodeMemory) : nullptr;
	if (!Blackboard)
		return EBTNodeResult::Failed;	

	bool bOverallStatus = false;
	if (bTeleportToDestinationUponFailure)
	{
		const bool bWrapUpLatentTask = false;
		bOverallStatus = TeleportAndExit(OwnerComp, bWrapUpLatentTask);
	}

	Blackboard->SetValueAsBool(FlightResultKey.SelectedKeyName, false);	
	Blackboard->SetValueAsBool(KeyToFlipFlopWhenTaskExits.SelectedKeyName, !Blackboard->GetValueAsBool(KeyToFlipFlopWhenTaskExits.SelectedKeyName));

	if (myMemory->bIsANavigator)
		IDonNavigator::Execute_OnLocomotionEnd(OwnerComp.GetAIOwner()->GetPawn(), bOverallStatus);

	if(bOverallStatus)
		return EBTNodeResult::Succeeded;
	else
		return EBTNodeResult::Failed;
}

void UBTTask_FlyTo::HandleTaskFailureAndExit(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory)
{
	auto pawn = OwnerComp.GetAIOwner()->GetPawn();
	UBlackboardComponent* blackboard = pawn && pawn->GetController() ? pawn->GetController()->FindComponentByClass<UBlackboardComponent>() : nullptr;

	if (HandleTaskFailure(OwnerComp, NodeMemory, blackboard) == EBTNodeResult::Failed)
		FinishLatentTask(OwnerComp, EBTNodeResult::Failed);
	else
		FinishLatentTask(OwnerComp, EBTNodeResult::Succeeded);
}

EBlackboardNotificationResult UBTTask_FlyTo::OnBlackboardValueChange(const UBlackboardComponent& Blackboard, FBlackboard::FKey ChangedKeyID)
{
	if (!bRecalcPathOnDestinationChanged)
		return EBlackboardNotificationResult::RemoveObserver;

	UBehaviorTreeComponent* BehaviorComp = Cast<UBehaviorTreeComponent>(Blackboard.GetBrainComponent());
	if (BehaviorComp == nullptr)
		return EBlackboardNotificationResult::RemoveObserver;

	uint8* RawMemory = BehaviorComp->GetNodeMemory(this, BehaviorComp->FindInstanceContainingNode(this));
	FBT_FlyToTarget* myMemory = reinterpret_cast<FBT_FlyToTarget*>(RawMemory);

	const EBTTaskStatus::Type TaskStatus = BehaviorComp->GetTaskStatus(this);
	if (TaskStatus != EBTTaskStatus::Active)
	{
		UE_VLOG(BehaviorComp, LogBehaviorTree, Error, TEXT("BT MoveTo \'%s\' task observing BB entry while no longer being active!"), *GetNodeName());

		// resetting BBObserverDelegateHandle without unregistering observer since 
		// returning EBlackboardNotificationResult::RemoveObserver here will take care of that for us
		myMemory->BBObserverDelegateHandle.Reset();

		return EBlackboardNotificationResult::RemoveObserver;
	}

	if (myMemory != nullptr)
	{
		const FVector flightDestination = Blackboard.GetValueAsVector(FlightLocationKey.SelectedKeyName);

		if (!myMemory->TargetLocation.Equals(flightDestination, RecalculatePathTolerance))
			myMemory->bTargetLocationChanged = true;
	}

	return EBlackboardNotificationResult::ContinueObserving;
}

EBTNodeResult::Type UBTTask_FlyTo::AbortTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory)
{	
	// safely abort nav task before we leave
	AbortPathfindingRequest(OwnerComp, NodeMemory);

	APawn* pawn = OwnerComp.GetAIOwner()->GetPawn();
	FBT_FlyToTarget* myMemory = (FBT_FlyToTarget*)NodeMemory;

	// Notify locomotion state:
	if (myMemory->QueryResults.PathSolutionOptimized.Num() && myMemory->bIsANavigator && pawn)
		IDonNavigator::Execute_OnLocomotionAbort(pawn);

	return Super::AbortTask(OwnerComp, NodeMemory);
}


FString UBTTask_FlyTo::GetStaticDescription() const
{
	FString ReturnDesc = Super::GetStaticDescription();
	
	ReturnDesc += FString::Printf(TEXT("\n%s: %s \n"), *GET_MEMBER_NAME_CHECKED(UBTTask_FlyTo, FlightLocationKey).ToString(), *FlightLocationKey.SelectedKeyName.ToString());
	ReturnDesc += FString("\nDebug Visualization:");
	ReturnDesc += FString::Printf(TEXT("Raw Path: %d \n"), DebugParams.VisualizeRawPath);
	ReturnDesc += FString::Printf(TEXT("Optimized Path: %d \n"), DebugParams.VisualizeOptimizedPath);

	return FString::Printf(TEXT("%s"), *ReturnDesc);
}

void UBTTask_FlyTo::DescribeRuntimeValues(const UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, EBTDescriptionVerbosity::Type Verbosity, TArray<FString>& Values) const
{
	Super::DescribeRuntimeValues(OwnerComp, NodeMemory, Verbosity, Values);	
}

uint16 UBTTask_FlyTo::GetInstanceMemorySize() const
{
	return sizeof(FBT_FlyToTarget);
}

#if WITH_EDITOR

FName UBTTask_FlyTo::GetNodeIconName() const
{
	return FName("BTEditor.Graph.BTNode.Task.Wait.Icon");
}

#endif	// WITH_EDITOR

bool UBTTask_FlyTo::TeleportAndExit(UBehaviorTreeComponent& OwnerComp, bool bWrapUpLatentTask /*= true*/)
{
	bool bTeleportSuccess = false;
	auto pawn = OwnerComp.GetAIOwner()->GetPawn();
	auto blackboard = pawn ? pawn->GetController()->FindComponentByClass<UBlackboardComponent>() : nullptr;

	if (blackboard)
	{
		FVector flightDestination = blackboard->GetValueAsVector(FlightLocationKey.SelectedKeyName);
		NavigationManager = UDonNavigationHelper::DonNavigationManagerForActor(pawn);

		bool bLocationValid = !NavigationManager->IsLocationBeneathLandscape(flightDestination);
		if(bLocationValid)
		{
			FVector flightDestination = blackboard->GetValueAsVector(FlightLocationKey.SelectedKeyName);
			pawn->SetActorLocation(flightDestination, false);
			GEngine->AddOnScreenDebugMessage(-1, 15.f, FColor::White, FString::Printf(TEXT("%s teleported, being unable to find pathfind aerially!"), pawn ? *pawn->GetName() : *FString("")));
			bTeleportSuccess = true;
		}
	}

	if (bWrapUpLatentTask)
	{
		if (bTeleportSuccess)
			FinishLatentTask(OwnerComp, EBTNodeResult::Succeeded);
		else
			FinishLatentTask(OwnerComp, EBTNodeResult::Failed);
	}

	return bTeleportSuccess;
}

//float UBTTask_FlyTo::LastRequestTimestamp;
const float UBTTask_FlyTo::RequestThrottleInterval = 0.25f;
