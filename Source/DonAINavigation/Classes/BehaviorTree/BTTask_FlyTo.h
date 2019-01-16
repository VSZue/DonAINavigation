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

#pragma once

#include "BehaviorTree/Tasks/BTTask_BlackboardBase.h"

#include "DonNavigationHelper.h"

#include "BTTask_FlyTo.generated.h"

USTRUCT()
struct FBT_FlyToTarget_DebugParams : public FDoNNavigationDebugParams
{
	GENERATED_USTRUCT_BODY()

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "DoN Navigation")
	bool bVisualizePawnAsVoxels;
};


struct FBT_FlyToTarget_Metadata
{
	uint16 ActiveInstanceIdx;

	TWeakObjectPtr<class UBehaviorTreeComponent> OwnerComp;	

	FBT_FlyToTarget_Metadata(){}

	FBT_FlyToTarget_Metadata(uint16 ActiveInstanceIdx, UBehaviorTreeComponent* OwnerComp) : ActiveInstanceIdx(ActiveInstanceIdx), OwnerComp(OwnerComp){}

};


struct FBT_FlyToTarget
{	
	FBT_FlyToTarget_Metadata Metadata;

	FDoNNavigationQueryParams QueryParams;

	FDonNavigationDynamicCollisionDelegate DynamicCollisionListener;

	int32 solutionTraversalIndex = 0;

	FDoNNavigationQueryData QueryResults;

	bool bSolutionInvalidatedByDynamicObstacle = false;	

	bool bIsANavigator = false;

	FVector TargetLocation;

	AActor* TargetActor;

	/* Whether this is a repath due to an actor target having moved */
	bool isMovingTargetRepath = false;

	FDelegateHandle BBObserverDelegateHandle;

	uint32 bTargetLocationChanged : 1;

	void Reset()
	{	
		isMovingTargetRepath = false;
		solutionTraversalIndex = 0;
		QueryResults = FDoNNavigationQueryData();
		QueryParams = FDoNNavigationQueryParams();
		Metadata = FBT_FlyToTarget_Metadata();
		bSolutionInvalidatedByDynamicObstacle = false;
		bTargetLocationChanged = false;
		TargetActor = nullptr;
	}
};

/**
 * 
 */
UCLASS()
class UBTTask_FlyTo : public UBTTaskNode
{
	GENERATED_BODY()

public:
	UBTTask_FlyTo(const FObjectInitializer& ObjectInitializer);	
	
	virtual EBTNodeResult::Type ExecuteTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory) override;
	virtual EBTNodeResult::Type AbortTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory) override;
	virtual uint16 GetInstanceMemorySize() const override;
	virtual void DescribeRuntimeValues(const UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, EBTDescriptionVerbosity::Type Verbosity, TArray<FString>& Values) const override;
	virtual FString GetStaticDescription() const override;	
	virtual void InitializeFromAsset(UBehaviorTree& Asset) override;

	EBTNodeResult::Type HandleTaskFailure(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, UBlackboardComponent* Blackboard);
	void HandleTaskFailureAndExit(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory);

	EBlackboardNotificationResult OnBlackboardValueChange(const UBlackboardComponent& Blackboard, FBlackboard::FKey ChangedKeyID);

#if WITH_EDITOR
	virtual FName GetNodeIconName() const override;
#endif // WITH_EDITOR

	// Behavior Tree Input:
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "DoN Navigation")
	FBlackboardKeySelector FlightGoalKey;

	/* Optional: Useful in somecases where you want failure or success of a task to automatically update a particular blackboard key*/
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "DoN Navigation")
	FBlackboardKeySelector FlightResultKey;

	/* Optional: This boolean will be flip-flopped at the end of this task (regardless of success or failure). This can be useful for certain types of behavior tree setups*/
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "DoN Navigation")
	FBlackboardKeySelector KeyToFlipFlopWhenTaskExits;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "DoN Navigation")
	float MinimumProximityRequired = 15.f;

	// Venu's Note:- Code for repath tolerance below was kindly contributed by Vladimir Ivanov (Github @ArCorvus). Thank you Vladimir!
	//
	/** Recalculate path enable */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DoN Navigation", meta = (InlineEditConditionToggle))
	uint32 bRecalcPathOnDestinationChanged : 1;

	/** Recalculate path if FlightLocation value changed. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "DoN Navigation", meta = (EditCondition = "bRecalcPathOnDestinationChanged"))
	float RecalculatePathTolerance;
	//

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "DoN Navigation")
	FDoNNavigationQueryParams QueryParams;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "DoN Navigation")
	FBT_FlyToTarget_DebugParams DebugParams;

	UFUNCTION(BlueprintCallable, Category="DoN Navigation")
	void Pathfinding_OnFinish(const FDoNNavigationQueryData& Data);

	UFUNCTION(BlueprintCallable, Category="DoN Navigation")
	void Pathfinding_OnDynamicCollisionAlert(const FDonNavigationDynamicCollisionPayload& Data);	

	UPROPERTY(BlueprintReadOnly, Category = "DoN Navigation")
	ADonNavigationManager* NavigationManager;

	/** In some scenarios, it may be desirable to allow the A.I. to teleport to its intended destination  if pathfinding failed for some reason
	*    Set this flag to true to if you want this behavior*/
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "DoN Navigation")
	bool bTeleportToDestinationUponFailure = false;

	// Makeshift arrangement until the Task Owner / Task List discrepancy bug is comprehensively conquered
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "DoN Navigation")
	float MaxTimeBeforeTeleport = 10.f;

protected:

	virtual void TickTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, float DeltaSeconds) override;

	FBT_FlyToTarget* TaskMemoryFromGenericPayload(void* GenericPayload);

	EBTNodeResult::Type SchedulePathfindingRequest(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, bool isMovingTargetRepath = false);	

	void AbortPathfindingRequest(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory);

	void TickPathNavigation(UBehaviorTreeComponent& OwnerComp, FBT_FlyToTarget* MyMemory, float DeltaSeconds);

	bool CheckTargetMoved(FBT_FlyToTarget* MyMemory);

	virtual void OnTaskFinished(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, EBTNodeResult::Type TaskResult) override;

	virtual bool TeleportAndExit(UBehaviorTreeComponent& OwnerComp, bool bWrapUpLatentTask = true);

	//
	TMap<TWeakObjectPtr<AActor>, float> LastRequestTimestamps;
	//float LastRequestTimestamp;
	const static float RequestThrottleInterval;
};