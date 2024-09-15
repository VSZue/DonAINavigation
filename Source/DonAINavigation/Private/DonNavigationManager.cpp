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

#include "DonNavigationManager.h"
#include "DonAINavigationPrivatePCH.h"
#include "Multithreading/DonNavigationWorker.h"

#include <stdio.h>
#include <limits>

DECLARE_CYCLE_STAT(TEXT("DonNavigation ~ PathfindingSolver"),            STAT_PathfindingSolver, STATGROUP_DonNavigation);
DECLARE_CYCLE_STAT(TEXT("DonNavigation ~ DynamicCollisionUpdates"),  STAT_DynamicCollisionUpdates, STATGROUP_DonNavigation);
DECLARE_CYCLE_STAT(TEXT("DonNavigation ~ DynamicCollisionSampling"), STAT_DynamicCollisionSampling, STATGROUP_DonNavigation);

#define DEBUG_DoNAI_THREADS 0

void FDonNavigationVoxel::BroadcastCollisionUpdates()
{
	// Protect ourselves from delegate owners reallocating the TArray while we're iterating:
	auto notifyees_safecopy = DynamicCollisionNotifyees; 

	for (const auto& notifyee : notifyees_safecopy)
		notifyee.Listener.ExecuteIfBound(notifyee.Payload);
}

ADonNavigationManager::ADonNavigationManager(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{
	// Scene Component
	SceneComponent = ObjectInitializer.CreateDefaultSubobject<USceneComponent>(this, TEXT("SceneComp"));
	SceneComponent->Mobility = EComponentMobility::Static;
	RootComponent = SceneComponent;

	Billboard = ObjectInitializer.CreateDefaultSubobject<UBillboardComponent>(this, TEXT("Billboard"));	
	static ConstructorHelpers::FObjectFinder<UTexture2D> BillboardTexture(TEXT("/DonAINavigation/Icons/Navigation_Volumer_Aerial.Navigation_Volumer_Aerial"));
	if(BillboardTexture.Succeeded())
		Billboard->Sprite = BillboardTexture.Object;
	Billboard->AttachToComponent(RootComponent, FAttachmentTransformRules::KeepRelativeTransform);

	// World Boundary Visualizer:
	WorldBoundaryVisualizer = ObjectInitializer.CreateDefaultSubobject<UBoxComponent>(this, TEXT("WorldBoundaryVisualizer"));	
	WorldBoundaryVisualizer->AttachToComponent(RootComponent, FAttachmentTransformRules::KeepRelativeTransform);
	WorldBoundaryVisualizer->SetVisibility(bDisplayWorldBoundary);
	WorldBoundaryVisualizer->SetHiddenInGame(true);	
	WorldBoundaryVisualizer->SetCollisionProfileName(FName("NoCollision"));
	Debug_RecalculateWorldBounds();

	VoxelSize = 100;

	XGridSize = 30;
	YGridSize = 30;
	ZGridSize = 30;

	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	bRegistrationCompleteForComponents = false;
	RegistrationIndexCurrent = 0;
	MaxRegistrationsPerTick = 200;

	ObstacleQueryChannels.Add(ECC_WorldStatic);
	ObstacleQueryChannels.Add(ECC_WorldDynamic);

	AutoCorrectionGuessList.Reserve(12);
	AutoCorrectionGuessList.Add(20);
	AutoCorrectionGuessList.Add(40);
	AutoCorrectionGuessList.Add(60);
	AutoCorrectionGuessList.Add(100);
	AutoCorrectionGuessList.Add(150);
	AutoCorrectionGuessList.Add(200);
	AutoCorrectionGuessList.Add(250);
	AutoCorrectionGuessList.Add(375);
	AutoCorrectionGuessList.Add(450);
	AutoCorrectionGuessList.Add(500);
	AutoCorrectionGuessList.Add(1000);
}

// Debug Helpers:
static ULineBatchComponent* GetDebugLineBatcher(const UWorld* InWorld, bool bPersistentLines, float LifeTime, bool bDepthIsForeground)
{
	return (InWorld ? (bDepthIsForeground ? InWorld->ForegroundLineBatcher : ((bPersistentLines || (LifeTime > 0.f)) ? InWorld->PersistentLineBatcher : InWorld->LineBatcher)) : NULL);
}

/* 
* Used to draw a debug voxel. This is based on code borrowed from DrawDebugHelpers, customized for our specific needs
*/
static void DrawDebugVoxel(const UWorld* InWorld, FVector const& Center, FVector const& Box, FColor const& Color, bool bPersistentLines, float LifeTime, uint8 DepthPriority, float thickness)
{
	// no debug line drawing on dedicated server
	if (GEngine->GetNetMode(InWorld) != NM_DedicatedServer)
	{
		// this means foreground lines can't be persistent 
		ULineBatchComponent* const LineBatcher = GetDebugLineBatcher(InWorld, bPersistentLines, LifeTime, (DepthPriority == SDPG_Foreground));
		if (LineBatcher != NULL)
		{
			float LineLifeTime = (LifeTime > 0.f) ? LifeTime : LineBatcher->DefaultLifeTime;

			LineBatcher->DrawLine(Center + FVector(Box.X, Box.Y, Box.Z), Center + FVector(Box.X, -Box.Y, Box.Z), Color, DepthPriority, thickness, LineLifeTime);
			LineBatcher->DrawLine(Center + FVector(Box.X, -Box.Y, Box.Z), Center + FVector(-Box.X, -Box.Y, Box.Z), Color, DepthPriority, thickness, LineLifeTime);
			LineBatcher->DrawLine(Center + FVector(-Box.X, -Box.Y, Box.Z), Center + FVector(-Box.X, Box.Y, Box.Z), Color, DepthPriority, thickness, LineLifeTime);
			LineBatcher->DrawLine(Center + FVector(-Box.X, Box.Y, Box.Z), Center + FVector(Box.X, Box.Y, Box.Z), Color, DepthPriority, thickness, LineLifeTime);

			LineBatcher->DrawLine(Center + FVector(Box.X, Box.Y, -Box.Z), Center + FVector(Box.X, -Box.Y, -Box.Z), Color, DepthPriority, thickness, LineLifeTime);
			LineBatcher->DrawLine(Center + FVector(Box.X, -Box.Y, -Box.Z), Center + FVector(-Box.X, -Box.Y, -Box.Z), Color, DepthPriority, thickness, LineLifeTime);
			LineBatcher->DrawLine(Center + FVector(-Box.X, -Box.Y, -Box.Z), Center + FVector(-Box.X, Box.Y, -Box.Z), Color, DepthPriority, thickness, LineLifeTime);
			LineBatcher->DrawLine(Center + FVector(-Box.X, Box.Y, -Box.Z), Center + FVector(Box.X, Box.Y, -Box.Z), Color, DepthPriority, thickness, LineLifeTime);

			LineBatcher->DrawLine(Center + FVector(Box.X, Box.Y, Box.Z), Center + FVector(Box.X, Box.Y, -Box.Z), Color, DepthPriority, thickness, LineLifeTime);
			LineBatcher->DrawLine(Center + FVector(Box.X, -Box.Y, Box.Z), Center + FVector(Box.X, -Box.Y, -Box.Z), Color, DepthPriority, thickness, LineLifeTime);
			LineBatcher->DrawLine(Center + FVector(-Box.X, -Box.Y, Box.Z), Center + FVector(-Box.X, -Box.Y, -Box.Z), Color, DepthPriority, thickness, LineLifeTime);
			LineBatcher->DrawLine(Center + FVector(-Box.X, Box.Y, Box.Z), Center + FVector(-Box.X, Box.Y, -Box.Z), Color, DepthPriority, thickness, LineLifeTime);
		}
	}
}

// Called every frame
void ADonNavigationManager::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);	

	if (!bMultiThreadingEnabled)
	{
		TickScheduledPathfindingTasks(DeltaSeconds, MaxPathSolverIterationsPerTick);

		TickScheduledCollisionTasks(DeltaSeconds, MaxCollisionSolverIterationsPerTick);
	}
	else
	{
		ReceiveAsyncResults();
		ReceiveAsyncDynamicCollisionUpdates();
		DrawAsyncDebugRequests();	
	}
}

void ADonNavigationManager::ReceiveAsyncResults()
{
	while (!CompletedNavigationTasks.IsEmpty())
	{
		FDonNavigationQueryTask task;
		CompletedNavigationTasks.Dequeue(task);
		ActiveNavigationTaskOwners.Remove(task.Data.Actor.Get());
		task.BroadcastResult();

#if DEBUG_DoNAI_THREADS
		auto owner = task.Data.Actor.Get();
		UE_LOG(DoNNavigationLog, Display, TEXT("[%s] [game thread] Received new nav result!"), owner ? *owner->GetName() : *FString("Unknown"));
#endif //DEBUG_DoNAI_THREADS*/
	}

	while (!CompletedCollisionTasks.IsEmpty())
	{
		FDonNavigationDynamicCollisionTask task;
		CompletedCollisionTasks.Dequeue(task);
		task.BroadcastResult();

		ActiveCollisionTaskOwners.Remove(task.MeshId.Mesh.Get());

#if DEBUG_DoNAI_THREADS
		auto owner = task.MeshId.Mesh.Get();
		UE_LOG(DoNNavigationLog, Display, TEXT("[%s] [game thread] Completed new dynamic collision task!"), owner ? *owner->GetOwner()->GetName() : *FString("Unknown"));
#endif //DEBUG_DoNAI_THREADS*/
	}
}

void ADonNavigationManager::ReceiveAsyncDynamicCollisionUpdates()
{
	while (!DynamicCollisionBroadcastQueue.IsEmpty())
	{
		FDonNavigationVoxel* voxel;
		DynamicCollisionBroadcastQueue.Dequeue(voxel);
		voxel->BroadcastCollisionUpdates();
	}
}

void ADonNavigationManager::DrawAsyncDebugRequests()
{
#if WITH_EDITOR

	while (!DrawDebugLinesQueue.IsEmpty())
	{
		FDrawDebugLineRequest line;
		DrawDebugLinesQueue.Dequeue(line);
		DrawDebugLine(GetWorld(), line.LineStart, line.LineEnd, line.Color, line.bPersistentLines, line.LifeTime, line.DepthPriority, line.Thickness);
	}

	while (!DrawDebugPointsQueue.IsEmpty())
	{
		FDrawDebugPointRequest point;
		DrawDebugPointsQueue.Dequeue(point);
		DrawDebugPoint(GetWorld(), point.PointLocation, point.PointThickness, point.Color, point.bPersistentLines, point.LifeTime);
	}

	while (!DrawDebugVoxelsQueue.IsEmpty())
	{
		FDrawDebugVoxelRequest voxel;
		DrawDebugVoxelsQueue.Dequeue(voxel);
		DrawDebugVoxel(GetWorld(), voxel.Center, voxel.Box, voxel.Color, voxel.bPersistentLines, voxel.LifeTime, voxel.DepthPriority, voxel.Thickness);
	}

	while (!DrawDebugSpheresQueue.IsEmpty())
	{
		FDrawDebugSphereRequest sphere;
		DrawDebugSpheresQueue.Dequeue(sphere);
		DrawDebugSphere(GetWorld(), sphere.Center, sphere.Radius, sphere.Segments, sphere.Color, sphere.bPersistentLines, sphere.LifeTime);
	}

#endif // WITH_EDITOR
}

void ADonNavigationManager::BeginPlay()
{
	Super::BeginPlay();

	UWorld* const World = GetWorld();

	if (!World)
		return;

	//Setup common collision parameters:
	VoxelCollisionShape = FCollisionShape::MakeBox(NavVolumeExtent());

	VoxelCollisionQueryParams = FCollisionQueryParams(FName("DonCollisionQuery", false)); // trace complex = false	
	VoxelCollisionQueryParams.AddIgnoredActors(ActorsToIgnoreForCollision);	

	VoxelCollisionQueryParams2 = FCollisionQueryParams(VoxelCollisionQueryParams);
	VoxelCollisionQueryParams2.bFindInitialOverlaps = false;

	for (auto collisionChannel : ObstacleQueryChannels)
		VoxelCollisionObjectParams.AddObjectTypesToQuery(collisionChannel);

	// Misc:
	VoxelSizeSquared = VoxelSize * VoxelSize;

	// Generate the world:
	ConstructBuilder();

	ActiveDynamicCollisionTasks.Reserve(60);

	RefreshPerformanceSettings();

	// Spawn dedicated worker thread:
	if (bMultiThreadingEnabled)
		WorkerThread = new FDonNavigationWorker(this, MaxPathSolverIterationsOnThread, MaxCollisionSolverIterationsOnThread);
}

void ADonNavigationManager::RefreshPerformanceSettings()
{
	if (bIsUnbound)
	{
		// Transfer Infinite World performance settings:
		MaxPathSolverIterationsPerTick = MaxPathSolverIterationsPerTick_Unbound;
		MaxCollisionSolverIterationsPerTick = MaxCollisionSolverIterationsPerTick_Unbound;
		MaxPathSolverIterationsOnThread = MaxPathSolverIterationsOnThread_Unbound;
		MaxCollisionSolverIterationsOnThread = MaxCollisionSolverIterationsOnThread_Unbound;
	}
}

void ADonNavigationManager::EndPlay(const EEndPlayReason::Type EndPlayReason)
{	
	if (WorkerThread)
	{
		WorkerThread->ShutDown();
	}
}

void ADonNavigationManager::OnConstruction(const FTransform& Transform)
{	
	Super::OnConstruction(Transform);

	Debug_RecalculateWorldBounds();
}

#if WITH_EDITOR
void ADonNavigationManager::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{	
	FProperty* PropertyThatChanged = PropertyChangedEvent.Property;
	FName PropertyName = PropertyThatChanged != NULL ? PropertyThatChanged->GetFName() : NAME_None;
	
	if (PropertyName == GET_MEMBER_NAME_CHECKED(ADonNavigationManager, bDisplayWorldBoundary))
	{
		Debug_RecalculateWorldBounds();
		WorldBoundaryVisualizer->SetVisibility(bDisplayWorldBoundary);		
	}
	else if (PropertyName == GET_MEMBER_NAME_CHECKED(ADonNavigationManager, bDisplayWorldBoundaryInGame))
	{
		WorldBoundaryVisualizer->SetHiddenInGame(!bDisplayWorldBoundaryInGame);
	}
	else
	{
		bool bWorldBoundsChanged = PropertyName == GET_MEMBER_NAME_CHECKED(ADonNavigationManager, VoxelSize)
								|| PropertyName == GET_MEMBER_NAME_CHECKED(ADonNavigationManager, XGridSize)
								|| PropertyName == GET_MEMBER_NAME_CHECKED(ADonNavigationManager, YGridSize)
								|| PropertyName == GET_MEMBER_NAME_CHECKED(ADonNavigationManager, ZGridSize);

		if (bWorldBoundsChanged)
		{
			Debug_RecalculateWorldBounds();
		}
	}	

	Super::PostEditChangeProperty(PropertyChangedEvent);
}
#endif // WITH_EDITOR

void ADonNavigationManager::ConstructBuilder()
{
	UWorld* const World = GetWorld();
	if (!World)
		return;

	if (bIsUnbound)
		return;
	
	uint64 timer = DoNNavigation::Debug_GetTimer();	
	GenerateNavigationVolumePixels();
	DoNNavigation::Debug_StopTimer(timer);

	UE_LOG(DoNNavigationLog, Log, TEXT("Time spent generating %d NAV volumes: %f seconds"), XGridSize * YGridSize * ZGridSize, timer / 1000.0);

	
	// This snippet is useful for studying and profiling behavior of the Nav Graph Cache behavior at full load. Not recommended for production.
	/*uint64 timerNAVNetwork = DoNNavigation::Debug_GetTimer();
	BuildNAVNetwork();
	DoNNavigation::Debug_StopTimer(timerNAVNetwork);
	
	UE_LOG(DoNNavigationLog, Log, TEXT("%s"), *FString::Printf(TEXT("Time spent building NAV network: %f seconds"), timerNAVNetwork / 1000.0));*/
}

void ADonNavigationManager::GenerateNavigationVolumePixels()
{
	UWorld* const World = GetWorld();

	if (!World)
		return;	

	float actorX = GetActorLocation().X;
	float actorY = GetActorLocation().Y;
	float actorZ = GetActorLocation().Z;	

	NAVVolumeData.X.Reserve(XGridSize);

	for (int i = 0; i < XGridSize; i++)
	{
		FDonNavVoxelX XPlaneVolumes;
		XPlaneVolumes.Y.Reserve(YGridSize);
		
		for (int j = 0; j < YGridSize; j++)
		{
			FDonNavVoxelY YPlaneVolumes;
			YPlaneVolumes.Z.Reserve(ZGridSize);

			for (int k = 0; k < ZGridSize; k++)
			{
				// Progress log: (this costs performance - uncomment only when necessary)
				//FString counter = FString::Printf(TEXT("Creating NAV Volume %d,%d,%d/%d,%d,%d"), i, j, k, XGridSize, YGridSize, ZGridSize);
				//UE_LOG(DoNNavigationLog, Log, TEXT("%s"), *counter);
				
				float x = i * VoxelSize + actorX + (VoxelSize / 2);
				float y = j * VoxelSize + actorY + (VoxelSize / 2);
				float z = k * VoxelSize + actorZ + (VoxelSize / 2);
				FVector Location = FVector(x, y, z);

				FDonNavigationVoxel volume;
				volume.X = i;
				volume.Y = j;
				volume.Z = k;
				volume.Location = Location;
				
				if (PerformCollisionChecksOnStartup)
					UpdateVoxelCollision(volume);

				YPlaneVolumes.AddZ(volume);				
			}

			XPlaneVolumes.AddY(YPlaneVolumes);
		}

		NAVVolumeData.AddX(XPlaneVolumes);
	}	
}

void ADonNavigationManager::BuildNAVNetwork()
{
	// This is a legacy function used back when the navigation system was static and baked into the map
	// For a dynamic solution with a large map the amount of time taken by this function is too huge to consider using it to pre-cache neighbors
	// Therefore, lazy loading is currently the preferred method of finding voxel neighbors. 
	//
	// This function is mainly used to profile nav graph cache performance at full saturation (i.e. all neighbor links calculated and cached)

	NavGraphCache.Reserve(XGridSize * YGridSize * ZGridSize);

	for (int32 i = 0; i < NAVVolumeData.X.Num(); i++)
	{
		for (int32 j = 0; j < NAVVolumeData.X[i].Y.Num(); j++)
		{
			for (int32 k = 0; k < NAVVolumeData.X[i].Y[j].Z.Num(); k++)
			{
				auto& volume = NAVVolumeData.X[i].Y[j].Z[k];
				FindOrSetupNeighborsForVolume(&volume);			
			}
		}
	}
}

void ADonNavigationManager::UpdateVoxelCollision(FDonNavigationVoxel& Volume)
{
	// Note: We're sampling overlaps here as this is MUCH faster than sweeping for hits, this approach hasn't caused any issues hitherto
	// That said, make sure that collision data for your map is being calculated along expected lines (eg: stray trigger volumes etc shouldn't be picked up as obstacles)

	TArray<FOverlapResult> outOverlaps;

	bool const bHit = GetWorld()->OverlapMultiByObjectType(outOverlaps, Volume.Location, FQuat::Identity, VoxelCollisionObjectParams, VoxelCollisionShape, VoxelCollisionQueryParams);

	bool CanNavigate = !outOverlaps.Num();
	Volume.SetNavigability(CanNavigate);

	// Profiling at max load (i.e. iterating over millions of voxels) reveals marginal performance boost for conditioned assignment. 
	// Please don't edit without profiling at max load and comparing results first.
	if (!Volume.bIsInitialized)
		Volume.bIsInitialized = true;	
}


void ADonNavigationManager::DiscoverNeighborsForVolume(int32 x, int32 y, int32 z, TArray<FDonNavigationVoxel*>& neighbors)
{
	bool bNeedsValidaion = x == 0 || y == 0 || z == 0 || x == XGridSize - 1 || y == YGridSize - 1 || z == ZGridSize - 1;

	neighbors.Reserve(Volume6DOF + VolumeImplicitDOF);

	// 6 DOF neighbors (Direct neighbors)
	for (int32 i = 0; i < Volume6DOF; i++)
	{
		int32 xN = x + x6DOFCoords[i];
		int32 yN = y + y6DOFCoords[i];
		int32 zN = z + z6DOFCoords[i];

		if (!bNeedsValidaion || IsValidVolume(xN, yN, zN))
			neighbors.Add(&VolumeAtUnsafe(xN, yN, zN));
	}
}

void ADonNavigationManager::AppendImplictDOFNeighborsForVolume(int32 x, int32 y, int32 z, TArray<FDonNavigationVoxel*>& Neighbors)
{
	bool bNeedsValidaion = x == 0 || y == 0 || z == 0 || x == XGridSize - 1 || y == YGridSize - 1 || z == ZGridSize - 1;

	if (!bNeedsValidaion || (IsValidVolume(x + 1, y, z + 1) && IsValidVolume(x - 1, y, z + 1) && IsValidVolume(x + 1, y, z - 1) && IsValidVolume(x - 1, y, z - 1) &&
		IsValidVolume(x, y + 1, z + 1) && IsValidVolume(x, y - 1, z + 1) && IsValidVolume(x, y + 1, z - 1) && IsValidVolume(x, y - 1, z - 1) &&
		IsValidVolume(x + 1, y + 1, z) && IsValidVolume(x - 1, y + 1, z) && IsValidVolume(x + 1, y - 1, z - 1) && IsValidVolume(x - 1, y - 1, z))
		)
	{
		// X		

		if (CanNavigate(&VolumeAtUnsafe(x + 1, y, z)) && CanNavigate(&VolumeAtUnsafe(x, y, z + 1)))
			Neighbors.Add(&VolumeAtUnsafe(x + 1, y, z + 1));

		if (CanNavigate(&VolumeAtUnsafe(x - 1, y, z)) && CanNavigate(&VolumeAtUnsafe(x, y, z + 1)))
			Neighbors.Add(&VolumeAtUnsafe(x - 1, y, z + 1));

		if (CanNavigate(&VolumeAtUnsafe(x + 1, y, z)) && CanNavigate(&VolumeAtUnsafe(x, y, z - 1)))
			Neighbors.Add(&VolumeAtUnsafe(x + 1, y, z - 1));

		if (CanNavigate(&VolumeAtUnsafe(x - 1, y, z)) && CanNavigate(&VolumeAtUnsafe(x, y, z - 1)))
			Neighbors.Add(&VolumeAtUnsafe(x - 1, y, z - 1));

		//Y
		if (CanNavigate(&VolumeAtUnsafe(x, y + 1, z)) && CanNavigate(&VolumeAtUnsafe(x, y, z + 1)))
			Neighbors.Add(&VolumeAtUnsafe(x, y + 1, z + 1));

		if (CanNavigate(&VolumeAtUnsafe(x, y - 1, z)) && CanNavigate(&VolumeAtUnsafe(x, y, z + 1)))
			Neighbors.Add(&VolumeAtUnsafe(x, y - 1, z + 1));

		if (CanNavigate(&VolumeAtUnsafe(x, y + 1, z)) && CanNavigate(&VolumeAtUnsafe(x, y, z - 1)))
			Neighbors.Add(&VolumeAtUnsafe(x, y + 1, z - 1));

		if (CanNavigate(&VolumeAtUnsafe(x, y - 1, z)) && CanNavigate(&VolumeAtUnsafe(x, y, z - 1)))
			Neighbors.Add(&VolumeAtUnsafe(x, y - 1, z - 1));

		//Z
		if (CanNavigate(&VolumeAtUnsafe(x + 1, y, z)) && CanNavigate(&VolumeAtUnsafe(x, y + 1, z)))
			Neighbors.Add(&VolumeAtUnsafe(x + 1, y + 1, z));

		if (CanNavigate(&VolumeAtUnsafe(x - 1, y, z)) && CanNavigate(&VolumeAtUnsafe(x, y + 1, z)))
			Neighbors.Add(&VolumeAtUnsafe(x - 1, y + 1, z));

		if (CanNavigate(&VolumeAtUnsafe(x + 1, y, z)) && CanNavigate(&VolumeAtUnsafe(x, y - 1, z)))
			Neighbors.Add(&VolumeAtUnsafe(x + 1, y - 1, z));

		if (CanNavigate(&VolumeAtUnsafe(x - 1, y, z)) && CanNavigate(&VolumeAtUnsafe(x, y - 1, z)))
			Neighbors.Add(&VolumeAtUnsafe(x - 1, y - 1, z));
	}
}


TArray<FDonNavigationVoxel*> ADonNavigationManager::FindOrSetupNeighborsForVolume(FDonNavigationVoxel* Volume)
{
	if (NavGraphCache.Contains(Volume))
	{
		auto neighbors = *NavGraphCache.Find(Volume); // copy by value so we don't pollute the cache implicit DOFs

		AppendImplictDOFNeighborsForVolume(Volume->X, Volume->Y, Volume->Z, neighbors);

		return neighbors;
	}
	else
	{
		// Neighbors not found, Lazy loading of NAV Graph for given volume commences...
		TArray<FDonNavigationVoxel*> neighbors;
		DiscoverNeighborsForVolume(Volume->X, Volume->Y, Volume->Z, neighbors);
		NavGraphCache.Add(Volume, neighbors);

		AppendImplictDOFNeighborsForVolume(Volume->X, Volume->Y, Volume->Z, neighbors);

		return neighbors;
	}
}

bool ADonNavigationManager::IsMeshBoundsWithinNavigableWorld(UPrimitiveComponent* Mesh, float BoundsScaleFactor/* = 1.f */)
{
	if (bIsUnbound)
		return true;

	FVector meshExtents = Mesh->Bounds.BoxExtent * BoundsScaleFactor;
	FVector meshMinBounds = (Mesh->Bounds.Origin - meshExtents);
	FVector meshMaxBounds = (Mesh->Bounds.Origin + meshExtents);

	// Min bounds
	int32 xMin = (meshMinBounds.X - GetActorLocation().X) / VoxelSize;
	int32 yMin = (meshMinBounds.Y - GetActorLocation().Y) / VoxelSize;
	int32 zMin = (meshMinBounds.Z - GetActorLocation().Z) / VoxelSize;

	// Max bounds
	int32 xMax = (meshMaxBounds.X - GetActorLocation().X) / VoxelSize;
	int32 yMax = (meshMaxBounds.Y - GetActorLocation().Y) / VoxelSize;
	int32 zMax = (meshMaxBounds.Z - GetActorLocation().Z) / VoxelSize;

	return xMin >= 0 && yMin >= 0 && zMin >= 0 && xMax < XGridSize && yMax < YGridSize && zMax < ZGridSize;
}

static FString GetMeshAssetName(UPrimitiveComponent* Mesh)
{
	FString assetName = FString("?");

	if (!Mesh)
		return assetName;

	auto skeletalMesh = Cast<USkeletalMeshComponent>(Mesh);
	auto staticMesh = Cast<UStaticMeshComponent>(Mesh);

	if (staticMesh)
		assetName = staticMesh->GetStaticMesh()->GetName();
	else if (skeletalMesh)
		assetName = skeletalMesh->SkeletalMesh->GetName();
	
	return assetName;
}

FString ADonNavigationManager::GetMeshLogIdentifier(UPrimitiveComponent* Mesh)
{
	if (!Mesh)
		return FString("?");
	
	FString meshLog = Mesh->GetName().Append(FString(" - "));
	meshLog.Append(GetMeshAssetName(Mesh));

	return meshLog;
}

FDonVoxelCollisionProfile ADonNavigationManager::GetVoxelCollisionProfileFromMesh(const FDonMeshIdentifier& MeshId, bool &bResultIsValid, DonVoxelProfileCache& PreferredCache, bool bIgnoreMeshOriginOccupancy /*= false*/, bool bDisableCacheUsage /*= false*/, FName CustomCacheIdentifier /*= NAME_None*/, bool bReloadCollisionCache /*= false*/, bool bUseCheapBoundsCollision /*= false*/, float BoundsScaleFactor /*= 1.f*/, bool DrawDebug /*= false*/)
{
	// Does the collision cache have an entry for this mesh?	
	if (!bDisableCacheUsage && !bReloadCollisionCache && PreferredCache.Contains(MeshId))
	{
		bResultIsValid = true;

		return *PreferredCache.Find(MeshId);
	}
	else
	{	
		auto collisionData = SampleVoxelCollisionForMesh(MeshId.Mesh.Get(), bResultIsValid, bIgnoreMeshOriginOccupancy, CustomCacheIdentifier, bUseCheapBoundsCollision, BoundsScaleFactor, DrawDebug);

		// Add to cache:
		if (bResultIsValid && !bDisableCacheUsage)
		{
			PreferredCache.Add(MeshId, collisionData);
		}

		return collisionData;
	}
}

FDonVoxelCollisionProfile ADonNavigationManager::SampleVoxelCollisionForMesh(UPrimitiveComponent* Mesh, bool &bResultIsValid, bool bIgnoreMeshOriginOccupancy/* = false*/, FName CustomCacheIdentifier/* = NAME_None*/, bool bUseCheapBoundsCollision/* = false*/, float BoundsScaleFactor/* = 1.f*/, bool DrawDebug/* = false*/)
{
	bResultIsValid = true;
	FDonVoxelCollisionProfile collisionData;

	// Input validations
	if (!Mesh || Mesh->GetCollisionProfileName().IsEqual(FName("NoCollision")))
	{	
		FString collisionProfileLog  =  Mesh ? Mesh->GetCollisionProfileName().ToString() : FString();
		UE_LOG(DoNNavigationLog, Error, TEXT("Invalid mesh passed to function GetVoxelCollisionProfileFromMesh, mesh: %s, physx collision profile: %s"), *GetMeshLogIdentifier(Mesh), *collisionProfileLog);

		bResultIsValid = false;

		return collisionData;
	}	

	// Are the bounds of the mesh completely within the navigable world?
	if (!IsMeshBoundsWithinNavigableWorld(Mesh))
	{
		UE_LOG(DoNNavigationLog, Error, TEXT("Mesh collision data is being tested from a location %s outside of the navigable world bounds. Aborting..."), *Mesh->Bounds.Origin.ToString());

		bResultIsValid = false;

		return collisionData;
	}

	// Calculate afresh and populate the cache:

	FVector meshExtents = Mesh->Bounds.BoxExtent * BoundsScaleFactor;
	FVector meshMinBounds = (Mesh->Bounds.Origin - meshExtents);
	FVector meshMaxBounds = (Mesh->Bounds.Origin + meshExtents);

	// Min bounds
	int32 xMin = (meshMinBounds.X - GetActorLocation().X) / VoxelSize;
	int32 yMin = (meshMinBounds.Y - GetActorLocation().Y) / VoxelSize;
	int32 zMin = (meshMinBounds.Z - GetActorLocation().Z) / VoxelSize;

	// Max bounds
	int32 xMax = (meshMaxBounds.X - GetActorLocation().X) / VoxelSize;
	int32 yMax = (meshMaxBounds.Y - GetActorLocation().Y) / VoxelSize;
	int32 zMax = (meshMaxBounds.Z - GetActorLocation().Z) / VoxelSize;
	
	auto meshOriginVolume = VolumeAt(Mesh->GetComponentLocation());
	if (!meshOriginVolume)
		return collisionData;

	// For optimal sampling results the mesh needs to be centered in its home voxel.
	const bool bShouldSweep = false;
	FVector originalMeshLocation = Mesh->GetComponentLocation();	
	Mesh->SetWorldLocation(meshOriginVolume->Location, bShouldSweep, NULL, ETeleportType::TeleportPhysics);

	// [Draw Debug] bounds visualization:
	// *** (Uncommented by default for manageability. Enable for debugging highly intricate scenarios.) ***
	//if (DrawDebug) DrawDebugVoxel_Safe(GetWorld(), Mesh->Bounds.Origin, Mesh->Bounds.BoxExtent, FColor::Green, true, 0, 0, 5.f);

	FCollisionObjectQueryParams objectParams;
	objectParams.AddObjectTypesToQuery(Mesh->GetCollisionObjectType());

	const bool bTraceComplex = false;
	FCollisionQueryParams collisionParams = FCollisionQueryParams(FName("GenerateNavigationVolumePixels", bTraceComplex));
	collisionParams.AddIgnoredActors(ActorsToIgnoreForCollision);

	for (int32 i = xMin; i <= xMax; i++)
	{
		for (int32 j = yMin; j <= yMax; j++)
		{
			for (int32 k = zMin; k <= zMax; k++)
			{
				auto& volumeToCheck = VolumeAtUnsafe(i, j, k);

				// [Draw Debug] visualize every volume sampled. 
				// *** (Uncommented by default for manageability. Enable for debugging highly intricate scenarios.) ***
				//if (DrawDebug) DrawDebugVoxel_Safe(GetWorld(), volumeToCheck.Location, NavVolumeExtent(), FColor::Black, true, 0, 0, DebugVoxelsLineThickness);

				bool collisionSampled = bUseCheapBoundsCollision ? true : false;

				// Sample this voxel against the mesh (skipped if cheap bounds based collision is used)
				if (!collisionSampled)
				{
					TArray<FOverlapResult> outOverlaps;
					bool const bHit = GetWorld()->OverlapMultiByObjectType(outOverlaps, volumeToCheck.Location, FQuat::Identity, objectParams, VoxelCollisionShape, collisionParams);

					for (const auto& overlap : outOverlaps)
					{
						if (overlap.GetComponent() == Mesh)
						{
							collisionSampled = true;

							break;
						}
					}
				}

				// Is the mesh occupying this voxel?
				if (collisionSampled)
				{
					// Draw voxel occupancy:
					if (DrawDebug)
						DrawDebugVoxel_Safe(GetWorld(), volumeToCheck.Location, NavVolumeExtent(), FColor::Red, false, 0.13f, 0, DebugVoxelsLineThickness);

					if (volumeToCheck == (*meshOriginVolume) && bIgnoreMeshOriginOccupancy)
						break;

					FVector relativeVoxelOffset = FVector(i - meshOriginVolume->X, j - meshOriginVolume->Y, k - meshOriginVolume->Z);
					collisionData.RelativeVoxelOccupancy.Add(relativeVoxelOffset);
				}				
			}
		}
	}

	// Make sure we revert the mesh back to its original location:		
	Mesh->SetWorldLocation(originalMeshLocation, bShouldSweep, NULL, ETeleportType::TeleportPhysics);	

	return collisionData;

}

// @wishlist: group this giant list of optional arguments into a nice struct...
bool ADonNavigationManager::ScheduleDynamicCollisionUpdate(UPrimitiveComponent* Mesh, FDonCollisionSamplerCallback ResultHandler, FName CustomCacheIdentifier /*= NAME_None*/, bool bReplaceExistingTask /*= false*/, bool bDisableCacheUsage /*= false*/, bool bReloadCollisionCache /*= false*/, bool bUseCheapBoundsCollision /*= false*/, float BoundsScaleFactor /*= 1.f*/, bool bForceSynchronousExecution /*= false*/, bool bDrawDebug /*= false*/)
{
	// Input validations
	if (!Mesh || Mesh->GetCollisionProfileName().IsEqual(FName("NoCollision")))
	{
		UE_LOG(DoNNavigationLog, Error, TEXT("Invalid mesh passed to function ScheduleDynamicCollisionUpdate, mesh: %s, physx collision profile: %s"), *GetMeshLogIdentifier(Mesh), Mesh ? *Mesh->GetCollisionProfileName().ToString() : *FString());

		return false;
	}

	auto meshOriginVolume = VolumeAt(Mesh->GetComponentLocation());

	if (!IsMeshBoundsWithinNavigableWorld(Mesh) || !meshOriginVolume)
	{
		UE_LOG(DoNNavigationLog, Error, TEXT("Mesh %s (%s) dynamic collision data is being tested from a location %s outside of the navigable world bounds. Aborting..."), *GetMeshLogIdentifier(Mesh), *CustomCacheIdentifier.ToString(), *Mesh->Bounds.Origin.ToString());

		return false;
	}
	
	// Prepare task
	auto meshId = FDonMeshIdentifier(Mesh, CustomCacheIdentifier);
	FDonNavigationDynamicCollisionTask task(meshId, ResultHandler, *meshOriginVolume, bDisableCacheUsage, bReloadCollisionCache, bUseCheapBoundsCollision, BoundsScaleFactor, bDrawDebug);

	if (!bReplaceExistingTask && IsDynamicCollisionTaskActive(task))
	{
		UE_LOG(DoNNavigationLog, Log, TEXT("Mesh %s (%s) already has a dynamic collision task running. Ignoring this request. Set bReplaceExistingTask to true to override. Also consider decreasing the frequency of calls or increasing iteration count in performance settings"), *GetMeshLogIdentifier(Mesh), *CustomCacheIdentifier.ToString());

		return false;
	}

	if (bForceSynchronousExecution)
	{
		UE_LOG(DoNNavigationLog, Verbose, TEXT("Forced synchronous execution of dynamic collision updates for %s (%s) in progress..."), *GetMeshLogIdentifier(Mesh), *CustomCacheIdentifier.ToString());		
		
		bool bResultIsValid = false;
		const bool bIgnoreMeshOriginOccupancy = false;
		FDonVoxelCollisionProfile VoxelCollisionProfile = GetVoxelCollisionProfileFromMesh(meshId, bResultIsValid, VoxelCollisionProfileCache_GameThread, bIgnoreMeshOriginOccupancy, bDisableCacheUsage, task.MeshId.CustomCacheIdentifier, task.bReloadCollisionCache, task.bUseCheapBoundsCollision, task.BoundsScaleFactor, bDrawDebug);

		if (!bResultIsValid)
		{
			UE_LOG(DoNNavigationLog, Log, TEXT("Synchronous voxel update for Mesh %s (%s) failed, voxel collisions could not be sampled. "), *GetMeshLogIdentifier(Mesh), *CustomCacheIdentifier.ToString());

			return false;
		}

		DynamicCollisionUpdateForMesh(task.MeshId, VoxelCollisionProfile, task.bDisableCacheUsage, task.bDrawDebug);

		task.ResultHandler.ExecuteIfBound(true);

		return true;
	}

	bool bOverallStatus;
	const bool bNeedsToScheduleTask = PrepareDynamicCollisionTask(task, bOverallStatus);
	if(bNeedsToScheduleTask)
		AddDynamicCollisionTask(task);

	UE_LOG(DoNNavigationLog, Verbose, TEXT("Num collision tasks: %d"), ActiveDynamicCollisionTasks.Num());

	return bOverallStatus;
}

bool ADonNavigationManager::IsDynamicCollisionTaskActive(const FDonNavigationDynamicCollisionTask& Task)
{
	if (!bMultiThreadingEnabled)
	{
		return ActiveDynamicCollisionTasks.Contains(Task);
	}
	else
	{
		return ActiveCollisionTaskOwners.Contains(Task.MeshId.Mesh.Get());
	}
}

bool ADonNavigationManager::PrepareDynamicCollisionTask(FDonNavigationDynamicCollisionTask& Task, bool &bOverallStatus)
{
	auto mesh = Task.MeshId.Mesh.Get();
	if (!ensure(mesh))
	{
		bOverallStatus = false;
		return false;
	}

	if (!Task.bDisableCacheUsage && !Task.bReloadCollisionCache && VoxelCollisionProfileCache_WorkerThread.Contains(Task.MeshId))
	{
		Task.FetchSuccess();

		Task.CollisionData = *VoxelCollisionProfileCache_WorkerThread.Find(Task.MeshId);

		bOverallStatus = true;
		return true;
	}
	// Are we using cheap bounds collision?
	else if (Task.bUseCheapBoundsCollision)
	{
		bool bResultIsValid = false;
		const bool bIgnoreMeshOriginOccupancy = false;
		FDonVoxelCollisionProfile VoxelCollisionProfile = GetVoxelCollisionProfileFromMesh(Task.MeshId, bResultIsValid, VoxelCollisionProfileCache_WorkerThread, bIgnoreMeshOriginOccupancy, Task.bDisableCacheUsage, Task.MeshId.CustomCacheIdentifier, Task.bReloadCollisionCache, Task.bUseCheapBoundsCollision, Task.BoundsScaleFactor, Task.bDrawDebug);

		if (!bResultIsValid)
		{
			UE_LOG(DoNNavigationLog, Log, TEXT("Cheap bounds voxel update for Mesh %s (%s) failed, voxel collisions could not be sampled. "), *GetMeshLogIdentifier(mesh), *Task.MeshId.CustomCacheIdentifier.ToString());
			bOverallStatus = false;
			return false;
		}

		DynamicCollisionUpdateForMesh(Task.MeshId, VoxelCollisionProfile, Task.bDisableCacheUsage, Task.bDrawDebug);

		UE_LOG(DoNNavigationLog, Verbose, TEXT("Dynamic collision updates using cheap bounds collision complete for mesh: %s (%s)"), *GetMeshLogIdentifier(mesh), *Task.MeshId.CustomCacheIdentifier.ToString());

		Task.ResultHandler.ExecuteIfBound(true);

		bOverallStatus = true;
		return false; // we don't need to schedule a task (it was already solved above)
	}
	// Prepare task data:
	else
	{
		// Add bounds data to the task	
		FVector meshExtents = mesh->Bounds.BoxExtent * Task.BoundsScaleFactor;
		Task.MeshOriginalExtents = meshExtents;
		Task.xLength = meshExtents.X * 2 / VoxelSize + 1;
		Task.yLength = meshExtents.Y * 2 / VoxelSize + 1;
		Task.zLength = meshExtents.Z * 2 / VoxelSize + 1;

		// Store collision params		
		Task.ObjectParams.AddObjectTypesToQuery(mesh->GetCollisionObjectType());

		const bool bTraceComplex = false;
		Task.CollisionParams = FCollisionQueryParams(FName("GenerateNavigationVolumePixels", bTraceComplex));
		Task.CollisionParams.AddIgnoredActors(ActorsToIgnoreForCollision);

		// Store the asset name (useful for debugging)		
		Task.MeshAssetName = GetMeshAssetName(mesh);

		// Reserve a resonable amount of space for the TArray sampler results:
		Task.CollisionData.RelativeVoxelOccupancy.Reserve((Task.xLength) * (Task.yLength) * (Task.zLength) / 4);

		bOverallStatus = true;
		return true;
	}
}

void ADonNavigationManager::AddDynamicCollisionTask(FDonNavigationDynamicCollisionTask& Task)
{
	if (!bMultiThreadingEnabled)
	{
		ActiveDynamicCollisionTasks.Add(Task);
	}
	else
	{
		NewDynamicCollisionTasks.Enqueue(Task);
		ActiveCollisionTaskOwners.Add(Task.MeshId.Mesh.Get());

#if DEBUG_DoNAI_THREADS
		auto owner = Task.MeshId.Mesh.Get();
		UE_LOG(DoNNavigationLog, Warning, TEXT("[%s] [game thread] Enqueued new collision task"), owner ? *owner->GetOwner()->GetName() : *FString("Unknown"));
#endif //DEBUG_DoNAI_THREADS*/
	}
}

void ADonNavigationManager::ReceiveAsyncCollisionTasks()
{
	if (NewDynamicCollisionTasks.IsEmpty())
		return;

	FDonNavigationDynamicCollisionTask task;
	bool bHasNewTask = NewDynamicCollisionTasks.Dequeue(task);

	///////////////	
	if (bHasNewTask)
	{
		bool bOverallStatus;
		const bool bNeedsToScheduleTask = PrepareDynamicCollisionTask(task, bOverallStatus);
		if(bNeedsToScheduleTask)
			ActiveDynamicCollisionTasks.Add(task);

#if DEBUG_DoNAI_THREADS
		auto owner = task.MeshId.Mesh.Get();
		UE_LOG(DoNNavigationLog, Display, TEXT("[%s] [async thread] Received new collision task!"), owner ? *owner->GetOwner()->GetName() : *FString("Unknown"));
#endif //DEBUG_DoNAI_THREADS*/
	}
}


void ADonNavigationManager::TickVoxelCollisionSampler(FDonNavigationDynamicCollisionTask& Task)
{
	//SCOPE_CYCLE_COUNTER(STAT_DynamicCollisionSampling);

	auto mesh = Task.MeshId.Mesh.Get();
	FVector meshMinBounds = (mesh->Bounds.Origin - Task.MeshOriginalExtents);
	FVector meshMinBoundsCoords = (meshMinBounds - GetActorLocation()) / VoxelSize;	

	int32 samplerCoordsX = (int32) meshMinBoundsCoords.X + Task.i;
	int32 samplerCoordsY = (int32) meshMinBoundsCoords.Y + Task.j;
	int32 samplerCoordsZ = (int32) meshMinBoundsCoords.Z + Task.k;

	auto volumeToCheck = VolumeAtSafe(samplerCoordsX, samplerCoordsY, samplerCoordsZ);	
	auto currentMeshOriginVolume = VolumeAt(mesh->GetComponentLocation());

	if (!currentMeshOriginVolume || !volumeToCheck)
	{
		Task.FetchFailure();

		return;
	}	

	// Draw every volume sampled: (** Uncomment for analyzing intricate scenarios **)
	//if (Task.bDrawDebug) DrawDebugVoxel_Safe(GetWorld(), volumeToCheck->Location, NavVolumeExtent(), FColor::Black, true, 0, 0, DebugVoxelsLineThickness);

	TArray<FOverlapResult> outOverlaps;
	bool const bHit = GetWorld()->OverlapMultiByObjectType(outOverlaps, volumeToCheck->Location, FQuat::Identity, Task.ObjectParams, VoxelCollisionShape, Task.CollisionParams);

	for (const auto& overlap : outOverlaps)
	{
		if (overlap.GetComponent() == mesh)
		{
			FVector relativeVoxelOffset = FVector(samplerCoordsX - currentMeshOriginVolume->X, samplerCoordsY - currentMeshOriginVolume->Y, samplerCoordsZ - currentMeshOriginVolume->Z);
			Task.CollisionData.RelativeVoxelOccupancy.Add(relativeVoxelOffset);

			break;
		}
	}

	Task.k++;

	if (Task.k > Task.zLength)
	{
		Task.j++;
		Task.k = 0;
	}

	if (Task.j > Task.yLength)
	{
		Task.i++;
		Task.j = 0;
		Task.k = 0;
	}

	if (Task.i > Task.xLength)
	{
		Task.FetchSuccess();

		if(!Task.bDisableCacheUsage)
			VoxelCollisionProfileCache_WorkerThread.Add(Task.MeshId, Task.CollisionData);	

		if (Task.bDrawDebug)
		{
			// Draw bounds:  (** Uncomment for analyzing intricate scenarios **)
			//DrawDebugVoxel_Safe(GetWorld(), mesh->Bounds.Origin, mesh->Bounds.BoxExtent, FColor::Green, true, 0, 0, 5.f); // Note-: this needs to be original bound origin for moving objects

			// Draw our solution:
			for (const auto& offset : Task.CollisionData.RelativeVoxelOccupancy)
			{
				auto& volume = VolumeAtUnsafe(Task.MeshOriginalVolume.X + offset.X, Task.MeshOriginalVolume.Y + offset.Y, Task.MeshOriginalVolume.Z + offset.Z);
				DrawDebugVoxel_Safe(GetWorld(), volume.Location, NavVolumeExtent(), FColor::Red, false, 0.13f, 0, DebugVoxelsLineThickness);
			}
		}
	}
}

void ADonNavigationManager::TickScheduledCollisionTasks(float DeltaSeconds, int32 MaxIterationsPerTick)
{
	const int32 numTasks = ActiveDynamicCollisionTasks.Num();	

	int32 maxTasksThisIteration = 0, maxIterationsPerTask = 0;

	if (!numTasks)
		return;

	if (numTasks <= MaxIterationsPerTick)
	{
		maxTasksThisIteration = numTasks;
		maxIterationsPerTask = MaxIterationsPerTick / numTasks;
	}
	else
	{
		maxTasksThisIteration = MaxIterationsPerTick;
		maxIterationsPerTask = 1;
	}
	
	int32 tasksProcessed = 0;

	for (int32 i=0; i < ActiveDynamicCollisionTasks.Num(); i++)
	{	
		auto& task = ActiveDynamicCollisionTasks[i];
		//SCOPE_CYCLE_COUNTER(STAT_DynamicCollisionSampling);

		if (!task.MeshId.Mesh.IsValid())
		{
			UE_LOG(DoNNavigationLog, Error, TEXT("Invalid mesh object found during dynamic collision updates for %s - %s"), *task.MeshAssetName, *task.MeshId.UniqueTag.ToString());			

			CompleteCollisionTask(i, false);

			continue;
		}

		int32 iterations = 0;
		task.TimeTaken += DeltaSeconds;

		while (!task.bCollisionProfileSamplingComplete && iterations < maxIterationsPerTask)
		{
			TickVoxelCollisionSampler(task);

			iterations++;
		}

		if (task.bCollisionProfileSamplingComplete)
		{
			if (task.MeshId.Mesh.IsValid() && task.bCollisionFetchSuccess)
			{
				DynamicCollisionUpdateForMesh(task.MeshId, task.CollisionData, task.bDisableCacheUsage, task.bDrawDebug);

				UE_LOG(DoNNavigationLog, Verbose, TEXT("Dynamic collision updates complete for mesh: %s (%s) in %f seconds"), *task.MeshAssetName, *task.MeshId.UniqueTag.ToString(), task.TimeTaken);
				
				CompleteCollisionTask(i, true);
			}				
			else
			{
				UE_LOG(DoNNavigationLog, Error, TEXT("Invalid mesh object found during dynamic collision updates for %s - %s"), *task.MeshAssetName, *task.MeshId.UniqueTag.ToString());
				
				CompleteCollisionTask(i, false);
			}

			UE_LOG(DoNNavigationLog, Verbose, TEXT("Num collision tasks: %d"), ActiveDynamicCollisionTasks.Num());
		}		

		tasksProcessed++;

		if (tasksProcessed >= maxTasksThisIteration)
			break;
	}
}

void ADonNavigationManager::TickScheduledCollisionTasks_Safe(float DeltaSeconds, int32 MaxIterationsPerTick)
{
	TickScheduledCollisionTasks(DeltaSeconds, MaxIterationsPerTick);
}

void ADonNavigationManager::CompleteCollisionTask(const int32 TaskIndex, bool bIsSuccess)
{
	auto& task = ActiveDynamicCollisionTasks[TaskIndex];
	task.bCollisionFetchSuccess = bIsSuccess;

	bool bSynchronousOperation = !bMultiThreadingEnabled;

	if (bSynchronousOperation)
	{
		// Just like with pathfinding tasks, synchronous execution of the delegate can create dependencies on removal of the task from the task list
		// so to ensure consistent behavior we first remove the task and only then execute the delegate on a safe copy of the result handler.

		auto resultHandler_safecopy = task.ResultHandler;

		ActiveDynamicCollisionTasks.RemoveAtSwap(TaskIndex);

		resultHandler_safecopy.ExecuteIfBound(bIsSuccess);
		
	}
	else
	{
		CompletedCollisionTasks.Enqueue(task);

		ActiveDynamicCollisionTasks.RemoveAtSwap(TaskIndex);

#if DEBUG_DoNAI_THREADS
		auto owner = task.MeshId.Mesh.Get();
		UE_LOG(DoNNavigationLog, Display, TEXT("[%s] [async thread] Enqueued new collision task result"), owner ? *owner->GetOwner()->GetName() : *FString("Unknown"));
#endif //DEBUG_DoNAI_THREADS*/
	}
	
}

void ADonNavigationManager::DynamicCollisionUpdateForMesh(const FDonMeshIdentifier& MeshId, FDonVoxelCollisionProfile& VoxelCollisionProfile, bool bDisableCacheUsage/* = false*/, bool bDrawDebug/* = false*/)
{	
	//SCOPE_CYCLE_COUNTER(STAT_DynamicCollisionUpdates);
	
	if (!MeshId.Mesh.IsValid())
	{
		UE_LOG(DoNNavigationLog, Error, TEXT("Invalid mesh. Skipping dynamic collision updates..."), *MeshId.UniqueTag.ToString());

		return;
	}

	auto Mesh = MeshId.Mesh.Get();

	auto meshOriginVolume = VolumeAt(Mesh->GetComponentLocation());
	
	if (!meshOriginVolume || !IsMeshBoundsWithinNavigableWorld(Mesh))
	{
		// Stricly speaking, we shouldn't skip dynamic collisions for objects whose bounds are only partially outside the navigable world (the object itself maybe within)
		// but considering this allows us to safely call "VolumeAtUnsafe" (which is more performant) I'm leaving this for now. Need to watch actual in-game usecases closely to understand the implications better.

		UE_LOG(DoNNavigationLog, Error, TEXT("DynamicCollisionUpdateForMesh was called from a location %s outside of the navigable world bounds. Skipping dynamic collision updates..."), *Mesh->Bounds.Origin.ToString());

		return;
	}
	
	// Note: Combining both loops (O(N2)) into a single loop possibly backed by a TSet for quick lookup of WorldVoxelsOccupied may boost performance. Profiling verification necessary.

	const int32 numVoxels = VoxelCollisionProfile.RelativeVoxelOccupancy.Num();

	// Flush out occupancy from previously occupied voxels:
	for (auto volume : VoxelCollisionProfile.WorldVoxelsOccupied)
	{
		if(volume)
			volume->SetNavigability(true);

		// Draw free'd voxels //if (bDrawDebug) DrawDebugVoxel_Safe(GetWorld(), volume->Location, NavVolumeExtent(), FColor::Green, true, 0, 0, DebugVoxelsLineThickness);
	}	

	VoxelCollisionProfile.WorldVoxelsOccupied.Empty(numVoxels);

	TArray<FDonNavigationVoxel*> newSpaceOccupied;
	newSpaceOccupied.Reserve(numVoxels);	

	// and move onto occupy the newly visited voxels:
	for (const auto& offset : VoxelCollisionProfile.RelativeVoxelOccupancy)
	{
		auto volume = VolumeAtSafe(meshOriginVolume->X + offset.X, meshOriginVolume->Y + offset.Y, meshOriginVolume->Z + offset.Z);
		if (!volume)
			continue;

		auto bPreviouslyNavigable = volume->CanNavigate();

		volume->SetNavigability(false);
		VoxelCollisionProfile.WorldVoxelsOccupied.Add(volume);

		// For reasons that I don't yet understand, using bPreviouslyNavigable to optimize the number of delegates we check for doesn't work 100% right.
		// There are edge cases where it causes us to miss out on valuable dynamic collision udpates that we truly need to listen to. 
		// For now I'm commenting it out as both are logically equivalent; filtering _is_ more efficient though.
		//if (bPreviouslyNavigable)
		{
			newSpaceOccupied.Add(volume); // We don't broadcast directly from here anymore to account for potential side-effects introduced by the delegate owner
		}

		// Draw occupied voxels
		if (bDrawDebug)
			DrawDebugVoxel_Safe(GetWorld(), volume->Location, NavVolumeExtent(), FColor::Red, false, 0.13f, 0, DebugVoxelsLineThickness);
	}

	// Broadcast dynamic collision updates!
	if (!bMultiThreadingEnabled)
	{
		for (auto volume : newSpaceOccupied)
			volume->BroadcastCollisionUpdates();
	}
	else
	{
		for (auto volume : newSpaceOccupied)
			DynamicCollisionBroadcastQueue.Enqueue(volume);
	}
	

	// Update the cache with latest occupany data:
	if(!bDisableCacheUsage)
		VoxelCollisionProfileCache_WorkerThread.Add(MeshId, VoxelCollisionProfile);
}

void ADonNavigationManager::Debug_ToggleWorldBoundaryInGame()
{
	bool bHiddenInGame = WorldBoundaryVisualizer->bHiddenInGame;
	WorldBoundaryVisualizer->SetHiddenInGame(!bHiddenInGame);
}

void ADonNavigationManager::Debug_DrawAllVolumes(float LineThickness)
{
	float actorX = GetActorLocation().X;
	float actorY = GetActorLocation().Y;
	float actorZ = GetActorLocation().Z;

	for (int i = 0; i < XGridSize; i++)
	{
		for (int j = 0; j < YGridSize; j++)
		{
			for (int k = 0; k < ZGridSize; k++)
			{
				float x = i * VoxelSize + actorX + (VoxelSize / 2);
				float y = j * VoxelSize + actorY + (VoxelSize / 2);
				float z = k * VoxelSize + actorZ + (VoxelSize / 2);
				FVector Location = FVector(x, y, z);

				DrawDebugVoxel_Safe(GetWorld(), Location, NavVolumeExtent() * 0.95f, FColor::Yellow, false, 2.5f, 0, LineThickness);
			}
		}
	}
}

void ADonNavigationManager::Debug_DrawVolumesAroundPoint(FVector Location, int32 CubeSize, bool DrawPersistentLines, float Duration, float LineThickness, bool bAutoInitializeVolumes/* = false*/)
{
	auto volume = VolumeAt(Location);
	if (!volume)
		return;

	for (int i = volume->X - CubeSize / 2; i < volume->X + CubeSize / 2; i++)
	{
		for (int j = volume->Y - CubeSize / 2; j < volume->Y + CubeSize / 2; j++)
		{
			for (int k = volume->Z - CubeSize / 2; k < volume->Z + CubeSize / 2; k++)
			{
				if (IsValidVolume(i, j, k))
				{
					auto& volumeToRender = VolumeAtUnsafe(i, j, k);
					bool canNavigate = false;
					if (bAutoInitializeVolumes)
						canNavigate = CanNavigate(&volumeToRender);
					else
						canNavigate = volumeToRender.CanNavigate();

					FColor color = canNavigate ? FColor::Green : FColor::Red;
					float lineThickness = canNavigate ? LineThickness : LineThickness / 2;
					FVector extents = canNavigate ? NavVolumeExtent() : NavVolumeExtent() * 0.95f;
					DrawDebugVoxel_Safe(GetWorld(), volumeToRender.Location, extents, color, DrawPersistentLines, 0, Duration, DebugVoxelsLineThickness);
				}
			}
		}
	}
}

void ADonNavigationManager::Debug_DrawVoxelCollisionProfile(UPrimitiveComponent* MeshOrPrimitive, bool bDrawPersistent/* = false*/, float Duration/* = 2.f*/)
{
	if (!MeshOrPrimitive)
		return;

	bool bResultIsValid;
	bool bIgnoreMeshOriginOccupancy = false;
	FName CustomCacheIdentifier = FName();
	bool bDisableCacheUsage = false;
	bool bReloadCollisionCache = false;
	bool bUseCheapBoundsCollision = false;
	float boundsScaleFactor = 1.f;
	bool bDrawDebug = false;
	auto voxelCollisionProfile = GetVoxelCollisionProfileFromMesh(FDonMeshIdentifier(MeshOrPrimitive), bResultIsValid, VoxelCollisionProfileCache_GameThread, bIgnoreMeshOriginOccupancy, bDisableCacheUsage,
		CustomCacheIdentifier, bReloadCollisionCache, bUseCheapBoundsCollision, boundsScaleFactor, bDrawDebug);

	// ~~~
	// Step 1. Draw the "center" voxel of the mesh
	FVector centerVoxel;
	FDonNavigationVoxel* meshOriginVolume = NULL;

	if (bIsUnbound)
		centerVoxel = VolumeOriginAt(MeshOrPrimitive->GetComponentLocation());
	else
	{
		meshOriginVolume = VolumeAt(MeshOrPrimitive->GetComponentLocation());
		if (!meshOriginVolume)
			return;

		centerVoxel = meshOriginVolume->Location;
	}

	// The navigation solver always assumes the origin voxel to occupy space, so should we:
	DrawDebugVoxel_Safe(GetWorld(), centerVoxel, NavVolumeExtent(), FColor::Red, bDrawPersistent, Duration, 0, DebugVoxelsLineThickness);

	// On this note, it is important to remember that the collision cache for pawn meshes will never have the mesh's origin volume listed in the occupancy list
	// because they're optimized away using bIgnoreMeshOriginOccupancy (which speeds up performance for simple meshes that have no need to maintain complex voxel occupancy data)

	// ~~~
	// Step 2. Draw all the other voxels (for meshes which are using a full-blown voxel profile representation)
	for (auto offset : voxelCollisionProfile.RelativeVoxelOccupancy)
	{
		const int32 voxelX = meshOriginVolume->X + offset.X;
		const int32 voxelY = meshOriginVolume->Y + offset.Y;
		const int32 voxelZ = meshOriginVolume->Z + offset.Z;

		FVector voxelLocation;
		if (bIsUnbound)
			voxelLocation = LocationAtId(voxelX, voxelY, voxelZ);
		else
		{
			auto volume = VolumeAtSafe(voxelX, voxelY, voxelZ);
			voxelLocation = volume->Location;
			if (!volume)
				continue;
		}

		DrawDebugVoxel_Safe(GetWorld(), voxelLocation, NavVolumeExtent(), FColor::Red, bDrawPersistent, Duration, 0, DebugVoxelsLineThickness);
	}
}

void ADonNavigationManager::Debug_ClearAllVolumes()
{
	FlushPersistentDebugLines(GetWorld());
}


////////////////////////////////////
// Core path solving algorithms follow:
///////////////////////////////////////

bool ADonNavigationManager::IsDirectPathLineSweep(UPrimitiveComponent* CollisionComponent, FVector Start, FVector End, FHitResult &OutHit, bool bFindInitialOverlaps/* = false*/, float CollisionShapeInflation/* = 0.f*/)
{
	if (!CollisionComponent)
		return false;

	TArray<AActor*> actorsToIgnore;
	actorsToIgnore.Add(CollisionComponent->GetOwner());

	if (IsDirectPathLineTrace(Start, End, OutHit, actorsToIgnore))
	{
		if (IsDirectPathSweep(CollisionComponent, Start, End, OutHit, bFindInitialOverlaps, CollisionShapeInflation))
			return true;
	}

	return false;
}

bool ADonNavigationManager::IsDirectPathSweep(UPrimitiveComponent* CollisionComponent, FVector Start, FVector End,FHitResult &OutHit, bool bFindInitialOverlaps/* = false*/, float CollisionShapeInflation/* = 0.f*/)
{
	bool bHit;
	TArray<FHitResult> OutHits;

	const bool bTraceComplex = false;
	FComponentQueryParams collisionParams(FName("IsDirectPathSweep"));
	collisionParams.bTraceComplex = bTraceComplex;	
	collisionParams.bFindInitialOverlaps = bFindInitialOverlaps; // Note:- Initial overlaps can be filtered from results by checking "hit.bStartPenetrating"	
	collisionParams.AddIgnoredActors(ActorsToIgnoreForCollision);
	collisionParams.AddIgnoredComponent(CollisionComponent);

	// Prepare inflated collision shape
	const FCollisionShape& collisionShape = CollisionComponent->GetCollisionShape(CollisionShapeInflation);
	collisionParams.AddIgnoredActor(CollisionComponent->GetOwner());
	bHit = GetWorld()->SweepMultiByObjectType(OutHits, Start, End, FQuat::Identity, VoxelCollisionObjectParams, collisionShape, collisionParams);

	// Check results:
	if (!bHit)
		return true;

	OutHit = OutHits.Last();
	UE_LOG(DoNNavigationLog, Verbose, TEXT("Optimizer hit %s-%s"), OutHit.GetActor() ? *OutHit.GetActor()->GetName() : *FString(), OutHit.GetComponent() ? *OutHit.GetComponent()->GetName() : *FString());
	
	return false;
}

bool ADonNavigationManager::IsDirectPathLineTrace(FVector start, FVector end, FHitResult &OutHit, const TArray<AActor*> &ActorsToIgnore, bool bFindInitialOverlaps/* = true*/)
{
	FCollisionQueryParams collisionParams(VoxelCollisionQueryParams);
	collisionParams.AddIgnoredActors(ActorsToIgnore);
	collisionParams.bFindInitialOverlaps = bFindInitialOverlaps;

	bool const bHit = GetWorld()->LineTraceSingleByObjectType(OutHit, start, end, VoxelCollisionObjectParams, collisionParams);

	return !bHit;
}

// Shape based tracing:
bool ADonNavigationManager::IsDirectPathLineSweepShape(const FCollisionShape& Shape, FVector Start, FVector End, FHitResult &OutHit, bool bFindInitialOverlaps/* = false*/)
{
	TArray<AActor*> actorsToIgnore;
	
	if (IsDirectPathLineTrace(Start, End, OutHit, actorsToIgnore, bFindInitialOverlaps))
	{
		if (IsDirectPathSweepShape(Shape, Start, End, OutHit, bFindInitialOverlaps))
			return true;
	}

	return false;

}

bool ADonNavigationManager::IsDirectPathSweepShape(const FCollisionShape& Shape, FVector Start, FVector End, FHitResult &OutHit, bool bFindInitialOverlaps/* = false*/)
{
	bool bHit;
	TArray<FHitResult> OutHits;

	const bool bTraceComplex = false;
	FComponentQueryParams collisionParams(FName("IsDirectPathSweepShape"));
	collisionParams.bTraceComplex = bTraceComplex;
	collisionParams.bFindInitialOverlaps = bFindInitialOverlaps; // Note:- Initial overlaps can be filtered from results by checking "hit.bStartPenetrating"	
	collisionParams.AddIgnoredActors(ActorsToIgnoreForCollision);

	bHit = GetWorld()->SweepMultiByObjectType(OutHits, Start, End, FQuat::Identity, VoxelCollisionObjectParams, Shape, collisionParams);

	// Check results:
	if (!bHit)
		return true;

	OutHit = OutHits.Last();
	
	return false;
}

void ADonNavigationManager::OptimizePathSolution_Pass1_LineTrace(UPrimitiveComponent* CollisionComponent, const TArray<FVector>& PathSolution, TArray<FVector> &PathSolutionOptimized, float CollisionShapeInflation/* = 0.f*/)
{
	if (PathSolution.Num() == 0)
	{
		UE_LOG(DoNNavigationLog, Error, TEXT("Could not perform line trace optimizations for this path - this path is empty"));
		PathSolutionOptimized = PathSolution;
		return;
	}
	
	PathSolutionOptimized.Add(PathSolution[0]);

	for (int32 i = 0; i < PathSolution.Num() - 1;)
	{
		if (i == PathSolution.Num() - 2)
		{
			PathSolutionOptimized.Add(PathSolution[PathSolution.Num() - 1]);
			return;
		}	

		bool foundDirectPath = false;

		for (int32 j = PathSolution.Num() - 1; j > i; j--)
		{
			FHitResult OutHit;
			TArray<AActor*> actorsToIgnore;
			const bool bConsiderInitialOverlaps = true;
			
			// Check if a direct path to goal exists from current node.			
			if (IsDirectPathLineSweep(CollisionComponent, PathSolution[i], PathSolution[j], OutHit, bConsiderInitialOverlaps, CollisionShapeInflation))
			{
				foundDirectPath = true;
				PathSolutionOptimized.Add(PathSolution[j]);
				i = j;
				break;
			}
			else if (j == i + 1) // ideally this should never happen if the pathfinding results are accurate
			{
				i++;
				PathSolutionOptimized.Add(PathSolution[i]);
			}
				
		}
		
		if (!foundDirectPath && i <= PathSolution.Num() - 2)
		{
			UE_LOG(DoNNavigationLog, Error, TEXT("Nav path alert: Couldn't find a single direct path among a series of points that were calculated as navigable. Potentially serious issue with pathing."));
			PathSolutionOptimized = PathSolution;
			return;
		}		
	}
}

void ADonNavigationManager::OptimizePathSolution(UPrimitiveComponent* CollisionComponent, const TArray<FVector>& PathSolution, TArray<FVector> &PathSolutionOptimized, float CollisionShapeInflation/* = 0.f*/)
{
	//PathSolutionOptimized = PathSolution;

	PathSolutionOptimized.Reserve(PathSolution.Num() + 2);

	OptimizePathSolution_Pass1_LineTrace(CollisionComponent, PathSolution, PathSolutionOptimized, CollisionShapeInflation);
}

void ADonNavigationManager::PathSolutionFromVolumeSolution(const TArray<FDonNavigationVoxel*>& VolumeSolution, TArray<FVector> &PathSolution, FVector Origin, FVector Destination, const FDoNNavigationDebugParams& DebugParams)
{	
	PathSolution.Reserve(VolumeSolution.Num() + 2);

	PathSolution.Add(Origin);

	for (auto volume : VolumeSolution)
		PathSolution.Add(volume->Location);

	PathSolution.Add(Destination);
}

static bool PathSolutionFromVolumeTrajectoryMap(FDonNavigationVoxel* OriginVolume, FDonNavigationVoxel* DestinationVolume, const TMap<FDonNavigationVoxel*, FDonNavigationVoxel*>& VolumeVsGoalTrajectoryMap, TArray<FDonNavigationVoxel*>& VolumeSolution, TArray<FVector> &PathSolution, FVector Origin, FVector Destination, const FDoNNavigationDebugParams& DebugParams)
{	
	// a rare edgecase, but worth handling gracefully in any case
	if (OriginVolume == DestinationVolume) 
	{
		VolumeSolution.Add(OriginVolume);
		VolumeSolution.Add(OriginVolume);
		
		PathSolution.Add(Origin);
		PathSolution.Add(Destination);

		return true;
	}

	// The trajectory map operates in reverse, so start from the destination:
	VolumeSolution.Insert(DestinationVolume, 0);
	PathSolution.Insert(Destination, 0);

	// Work our way back from destination to origin while generating a linear path solution list
	bool originFound = false;
	auto nextVolume = VolumeVsGoalTrajectoryMap.Find(DestinationVolume);

	while (nextVolume)
	{
		if (VolumeSolution.Contains(*nextVolume))
			break;

		VolumeSolution.Insert(*nextVolume, 0);
		PathSolution.Insert((*nextVolume)->Location, 0);

		if (*nextVolume == OriginVolume)
		{
			originFound = true;
			break;
		}

		nextVolume = VolumeVsGoalTrajectoryMap.Find(*nextVolume);
	}

	return originFound;
	
}

FDonNavigationVoxel* ADonNavigationManager::GetBestNeighborRecursive(FDonNavigationVoxel* Volume, int32 CurrentDepth, int32 NeighborSearchMaxDepth, FVector Location, UPrimitiveComponent* CollisionComponent, bool bConsiderInitialOverlaps, float CollisionShapeInflation, bool bShouldSweep)
{
	if (CurrentDepth > NeighborSearchMaxDepth)
		return NULL;

	FHitResult hit;
	const auto& neighbors = FindOrSetupNeighborsForVolume(Volume);

	for (auto neighbor : neighbors)
	{
		if (!CanNavigate(neighbor))
			continue;

		if (!bShouldSweep)
			return neighbor;
		else if (IsDirectPathLineSweep(CollisionComponent, Location, neighbor->Location, hit, bConsiderInitialOverlaps, CollisionShapeInflation))
			return neighbor;
	}

	// No suitable volume found, testing neighbors of neighbors:
	for (auto neighbor : neighbors)
	{
		// need to optimize redundancy. A large number of voxels will get queried multiple times due to multi-neighbor relationships. Consider maintaining a hash (TSet) of visited neighbors
		// @Bug - the function below should actually use "neighbor" and not "Volume"! As this needs more testing, the change is reserved for a future update.
		auto bestVolume = GetBestNeighborRecursive(Volume, CurrentDepth + 1, NeighborSearchMaxDepth, Location, CollisionComponent, bConsiderInitialOverlaps, CollisionShapeInflation, bShouldSweep);
		if (bestVolume)
			return bestVolume;
	}

	return NULL;		
}

FDonNavigationVoxel* ADonNavigationManager::GetClosestNavigableVolume(FVector Location, UPrimitiveComponent* CollisionComponent, bool &bInitialPositionCollides, float CollisionShapeInflation/* = 0.f;*/, bool bShouldSweep/* = true*/)
{
	if (bShouldSweep && !CollisionComponent)
		return NULL;

	auto volume = VolumeAt(Location);
	if (!volume)
	{
		UE_LOG(DoNNavigationLog, Error, TEXT("Location %s is outside navigable world boundary. Verify your Manager actor's setup and use Display World Boundary to see the navigable bounds."), *Location.ToString());

		return NULL; // location beyond world bounds		
	}
		
	
	// Case 1: Given volume is navigable. No further checks required:
	//
	if (CanNavigate(volume))
		return volume;	

	// Case 2: The voxel for this location is not navigable!	
	// Ideally a pawn should never be able to visit such a voxel, but for large maps with low collision accuray the location itself is often accessible
	// and so we need to find the closest neighboring voxel that has direct access to this location.  The purpose of doing this is to find a _substitute_
	// for this volume  which is both close and navigable. Only such a substitute can be used an entry-point or exit point for pathfinding.

	FHitResult hit;

	bool bConsiderInitialOverlaps = true; // This is really important to understand.

	//
	// Rationale:

	// Imagine a scenario where a flying creature has landed on a hard surface (so initial overlap is true) and is making a navigation query from that point.
	// For this case the home voxel is not navigable so we need to search for a suitable proxy. If we ignore hits from initial overlaps it is perfectly possible
	// for us to pick a volume right below the the hard surface! (because we're already overlapping with it, the hard surface will be ignored during collision checks). 
	// For this reason, we always consider initial overlaps.
	//
	// Implication: Because of this setup, you should always start navigation query while in free space (not colliding with any solid obstacle). It is alright to be inside a voxel
	// that is tagged not navigable (that is why this function was written, after all) but if you're colliding with another object while starting the query chances are this function
	// never find the "closest navigable neighbor" and your pathfinding request will simply fail.
	//

	TArray<FOverlapResult> outOverlaps;	
	if (bShouldSweep)
	{
		FCollisionObjectQueryParams objectParams = VoxelCollisionObjectParams;
		objectParams.AddObjectTypesToQuery(CollisionComponent->GetCollisionObjectType());
		FCollisionQueryParams queryParams = VoxelCollisionQueryParams;
		queryParams.AddIgnoredComponent(CollisionComponent);
		bInitialPositionCollides = GetWorld()->OverlapMultiByObjectType(outOverlaps, Location, FQuat::Identity, objectParams, CollisionComponent->GetCollisionShape(1.f), queryParams);

		if (bInitialPositionCollides)
			return NULL;
	}

	// 2 a) First we start with simple heuristics based checks for the closest accessible neighbor:
	static const int32 neighborGuessList = 9;
	static const int32 expansionStepSize = 2;
	static const int32 numIterations = 2;

	const int xGuessList[neighborGuessList] = { 0,  2,  2, -2, -2, -2,  2,  0, -2 };
	const int yGuessList[neighborGuessList] = { 0,  0,  2,  0, -2,  2, -2,  0, -2 };
	const int zGuessList[neighborGuessList] = { 1,  1,  1,  1,  1,  1,  1, -1, -1 };

	for (int32 step = 1; step <= numIterations; step++)
	{
		for (int32 i = 0; i < neighborGuessList; i++)
		{
			int32 stepScale = step * expansionStepSize;
			auto volumeGuess = VolumeAtSafe(volume->X + xGuessList[i], volume->Y + yGuessList[i], volume->Z + zGuessList[i] * stepScale);
			if (volumeGuess && CanNavigate(volumeGuess))
			{
				if (!bShouldSweep)
					return volumeGuess;
				else if (IsDirectPathLineSweep(CollisionComponent, Location, volumeGuess->Location, hit, bConsiderInitialOverlaps, CollisionShapeInflation))
					return volumeGuess;
			}
		}
	}
	
	// 2 b) So we still haven't found an ideal neighbor. It's time to methodically scan for best neighbors:
	const int32 neighborSearchMaxDepth = 2;
	auto result = GetBestNeighborRecursive(volume, 0, neighborSearchMaxDepth, Location, CollisionComponent, bConsiderInitialOverlaps, CollisionShapeInflation, bShouldSweep);	

	if (result)
		return result;

	// No solution:

	UE_LOG(DoNNavigationLog, Error, TEXT("Error: GetClosestNavigableVolume failed to resolve a given volume."));

#if WITH_EDITOR
	DrawDebugSphere_Safe(GetWorld(), Location, 64.f, 6.f, FColor::Magenta, true, -1.f);
	DrawDebugPoint_Safe(GetWorld(), Location, 6.f, FColor::Yellow, true, -1.f);
#endif // WITH_EDITOR	
	
 	return result;
}

FDonNavigationVoxel* ADonNavigationManager::ResolveVolume(FVector &DesiredLocation, UPrimitiveComponent* CollisionComponent, bool bFlexibleOriginGoal /*= true*/, float CollisionShapeInflation /*= 0.f*/, bool bShouldSweep /*= true*/)
{
	// Volume resolution for Finite Worlds

	if (bShouldSweep && !CollisionComponent)
		return nullptr;

	bool bInitialPositionCollides;
	auto volume = GetClosestNavigableVolume(DesiredLocation, CollisionComponent, bInitialPositionCollides, CollisionShapeInflation, bShouldSweep);	

	if (volume)
		return volume; // success


	// Failure handling:

	if (bFlexibleOriginGoal) // we could filter this adaptation to only consider bInitialPositionCollides scenarios, but there are some edge cases which aren't covered with that approach
	{
		UE_LOG(DoNNavigationLog, Warning, TEXT("Pawn's initial/final position overlaps an obstacle. Attempting to find substitute vector (a nearby free spot) for pathfinding..."));

		static const int32 numTweaks = 6;
		static const FVector locationTweaks[numTweaks] = { FVector(0, 0,  1), FVector(1, 0, 0), FVector(0,  1, 0), FVector(0, 0, -1), FVector(-1, 0, 0), FVector(0, -1, 0) };
		for (auto tweakMagnitude : AutoCorrectionGuessList)
		{
			for (const auto& tweakDir : locationTweaks)
			{
				FVector tweak = tweakDir * tweakMagnitude;
				FVector locationToSample = DesiredLocation + tweak;				
				FVector desiredLocationToSample = DesiredLocation + tweakDir * UnrealPhyxPenetrationDepth; // very important!

				//DrawDebugSphere_Safe(GetWorld(), locationToSample, 6.f, 16, FColor::Yellow, false, 5.f);

				auto volume = VolumeAt(locationToSample);
				if (!volume || !CanNavigate(volume))
					continue;

				FHitResult outHit;
				const bool bConsiderInitialOverlaps = true; // Direction of sweep is important: If the desired location is intersecting a collision body the only way we can trace to it is from the opposite end, i.e. the guess-list volume
				
				if (!IsDirectPathLineSweep(CollisionComponent, locationToSample, desiredLocationToSample, outHit, bConsiderInitialOverlaps, CollisionShapeInflation))
					continue;

				UE_LOG(DoNNavigationLog, Warning, TEXT("Substitute Origin or Destination (%s offset) is being used for pawn to overcome initial overlap. (Can be disabled in QueryParams)"), *tweak.ToString());

				DesiredLocation = locationToSample;

				return volume; // success
			}
		}		
	}
	
	return NULL;
}

bool ADonNavigationManager::GetClosestNavigableVector(FVector DesiredLocation, FVector &ResolvedLocation, UPrimitiveComponent* CollisionComponent, bool &bInitialPositionCollides, float CollisionShapeInflation/* = 0.f;*/, bool bShouldSweep/* = true*/)
{
	// Long-term goal: Eliminate code duplication between the Finite and Infinite representations of this function (i.e. GetClosestNavigableVector and GetClosestNavigableVolume)

	if (bShouldSweep && !CollisionComponent)
		return false;

	ResolvedLocation = VolumeOriginAt(DesiredLocation);
	
	if (CanNavigate(ResolvedLocation))
	{
		return true;
	}

	FHitResult hit;
	TArray<FOverlapResult> outOverlaps;
	bool bConsiderInitialOverlaps = true;

	if (bShouldSweep)
	{
		FCollisionObjectQueryParams objectParams = VoxelCollisionObjectParams;
		objectParams.AddObjectTypesToQuery(CollisionComponent->GetCollisionObjectType());
		// objectParams.RemoveObjectTypesToQuery(ECC_Pawn); // Some projects/usecases may benefit from enabling this line. It prevents initial overlaps with pawns from affecting pathfinding. Currently enabled for DoN The Nature Game!
		FCollisionQueryParams queryParams = VoxelCollisionQueryParams;
		queryParams.AddIgnoredComponent(CollisionComponent);
		bInitialPositionCollides = GetWorld()->OverlapMultiByObjectType(outOverlaps, DesiredLocation, FQuat::Identity, objectParams, CollisionComponent->GetCollisionShape(1.f), queryParams);

		if (bInitialPositionCollides)
			return false;
	}

	// 2 a) Simple heuristics:
	static const int32 neighborGuessList = 9;
	static const int32 expansionStepSize = 2;
	static const int32 numIterations = 2;

	static const int32 xGuessList[neighborGuessList] = { 0,  2,  2, -2, -2, -2,  2,  0, -2 };
	static const int32 yGuessList[neighborGuessList] = { 0,  0,  2,  0, -2,  2, -2,  0, -2 };
	static const int32 zGuessList[neighborGuessList] = { 1,  1,  1,  1,  1,  1,  1, -1, -1 };

	for (int32 step = 1; step <= numIterations; step++)
	{
		for (int32 i = 0; i < neighborGuessList; i++)
		{
			int32 stepScale = step * expansionStepSize;

			FVector newLocation = ResolvedLocation + VoxelSize * FVector(xGuessList[i], yGuessList[i], zGuessList[i] * stepScale);

			if (CanNavigate(newLocation))
			{
				if (!bShouldSweep || IsDirectPathLineSweep(CollisionComponent, DesiredLocation, newLocation, hit, bConsiderInitialOverlaps, CollisionShapeInflation))
				{
					ResolvedLocation = newLocation;
					return true;
				}
			}
		}
	}

	// No solution:

	UE_LOG(DoNNavigationLog, Error, TEXT("Error: GetClosestNavigableVolume failed to resolve a given volume."));

#if WITH_EDITOR
	DrawDebugSphere_Safe(GetWorld(), DesiredLocation, 64.f, 6.f, FColor::Magenta, true, -1.f);
	DrawDebugPoint_Safe(GetWorld(), DesiredLocation, 6.f, FColor::Yellow, true, -1.f);
#endif // WITH_EDITOR	

	return false;
}

bool ADonNavigationManager::ResolveVector(FVector &DesiredLocation, FVector &ResolvedLocation, UPrimitiveComponent* CollisionComponent, bool bFlexibleOriginGoal /*= true*/, float CollisionShapeInflation /*= 0.f*/, bool bShouldSweep /*= true*/)
{
	// Vector resolution for Infinite Worlds	

	if (bShouldSweep && !CollisionComponent)
		return false;

	bool bInitialPositionCollides;
	bool bFoundResult = GetClosestNavigableVector(DesiredLocation, ResolvedLocation, CollisionComponent, bInitialPositionCollides, CollisionShapeInflation, bShouldSweep);

	const int32 numTweaks = 6;
	FVector locationTweaks[numTweaks] = { FVector(0, 0,  1), FVector(1, 0, 0), FVector(0,  1, 0),
		FVector(0, 0, -1), FVector(-1, 0, 0), FVector(0, -1, 0)
	};

	if (bFoundResult)
		return true; // success

	// Failure handling:
	if (bFlexibleOriginGoal)
	{
		UE_LOG(DoNNavigationLog, Warning, TEXT("Pawn's initial/final position overlaps an obstacle. Attempting to find substitute vector (a nearby free spot) for pathfinding..."));

		for (auto tweakMagnitude : AutoCorrectionGuessList)
		{
			for (const auto& tweakDir : locationTweaks)
			{
				FVector tweak = tweakDir * tweakMagnitude;			
				FVector locationToSample = DesiredLocation + tweak;
				FVector desiredLocationToSample = DesiredLocation + tweakDir * UnrealPhyxPenetrationDepth; // very important!

				ResolvedLocation = VolumeOriginAt(locationToSample);
				if (!CanNavigate(ResolvedLocation))
					continue;

				FHitResult outHit;
				const bool bConsiderInitialOverlaps = true; // Direction of sweep is important: If the desired location is intersecting a collision body the only way we can trace to it is from the opposite end, i.e. the guess-list volume
				if (!IsDirectPathLineSweep(CollisionComponent, locationToSample, desiredLocationToSample, outHit, bConsiderInitialOverlaps, CollisionShapeInflation))
					continue;

				UE_LOG(DoNNavigationLog, Warning, TEXT("Substitute Origin or Destination (%s offset) is being used for pawn to overcome initial overlap. (Can be disabled in QueryParams)"), *tweak.ToString());

				DesiredLocation = locationToSample;

				return true; // success
			}
		}
	}

	return false;
}

bool ADonNavigationManager::CanNavigate(FVector Location)
{
	// Note:- UpdateVoxelCollision currently performs a similar check to determine navigability. For consistency both should use the same path.

	TArray<FOverlapResult> outOverlaps;

	bool const bHit = GetWorld()->OverlapMultiByObjectType(outOverlaps, Location, FQuat::Identity, VoxelCollisionObjectParams, VoxelCollisionShape, VoxelCollisionQueryParams);

	bool CanNavigate = !outOverlaps.Num();

	return CanNavigate;
}

bool ADonNavigationManager::CanNavigate(FDonNavigationVoxel* Volume)
{
	if (!Volume->bIsInitialized)
		UpdateVoxelCollision(*Volume);

	return Volume->CanNavigate();
}

bool ADonNavigationManager::CanNavigateByCollisionProfile(FDonNavigationVoxel* Volume, const FDonVoxelCollisionProfile& CollisionToTest)
{	
	if (!CanNavigate(Volume))
		return false;

	bool bCanNavigate = true;

	for (FVector voxelOffset : CollisionToTest.RelativeVoxelOccupancy)
	{
		auto neighborToTest = NeighborAt(Volume, voxelOffset);
		if (!neighborToTest || !CanNavigate(neighborToTest)) // invalid neighborToTest could indicate collision with world boundary
			return false;
	}
	
	return bCanNavigate;
}

bool ADonNavigationManager::CanNavigateByCollisionProfile(FVector Location, const FDonVoxelCollisionProfile& CollisionToTest)
{
	if (!CanNavigate(Location))
		return false;

	bool bCanNavigate = true;

	for (FVector voxelOffset : CollisionToTest.RelativeVoxelOccupancy)
	{
		FVector locationToTest = LocationAtId(Location, voxelOffset.X, voxelOffset.Y, voxelOffset.Z);

		if (!CanNavigate(locationToTest))
			return false;
	}

	return bCanNavigate;
}

void ADonNavigationManager::ExpandFrontierTowardsTarget(FDonNavigationQueryTask& Task, FDonNavigationVoxel* Current, FDonNavigationVoxel* Neighbor)
{	
	if (!CanNavigateByCollisionProfile(Neighbor, Task.Data.VoxelCollisionProfile))
		return;

	// In reality there are two possible segment distances: side and sqrt(2) * side. As a trade-off between accuracy and performance we're assuming all segments to be only equal to the pixel size (majority case are 6-DOF neighbors)
	float SegmentDist = VoxelSize;
	
	uint32 newCost = *Task.Data.VolumeVsCostMap.Find(Current) + SegmentDist;
	uint32* volumeCost = Task.Data.VolumeVsCostMap.Find(Neighbor);

	if (!volumeCost || newCost < *volumeCost)
	{	
		Task.Data.VolumeVsGoalTrajectoryMap.Add(Neighbor, Current);
		Task.Data.VolumeVsCostMap.Add(Neighbor, newCost);

		float heuristic = FVector::Dist(Neighbor->Location, Task.Data.Destination);
		uint32 priority = newCost + heuristic;

		Task.Data.Frontier.put(Neighbor, priority);
	}
}

void ADonNavigationManager::InvalidVolumeErrorLog(FDonNavigationVoxel* OriginVolume, FDonNavigationVoxel* DestinationDestination, FVector Origin, FVector Destination)
{
	bool bLogHelpInfo = true;
	FVector vectorInQuestion;

	if (!OriginVolume)
	{
		vectorInQuestion = Origin;

		UE_LOG(DoNNavigationLog, Error, TEXT("Error: Invalid Origin (%s) passed to navigation path solver"), *Origin.ToString());
	}

	if (!DestinationDestination)
	{
		vectorInQuestion = Destination;

		UE_LOG(DoNNavigationLog, Error, TEXT("Error: Invalid Destination (%s) passed to navigation path solver"), *Destination.ToString());
	}

	// Troubleshooting instructions already logged for this vector? If so, return to avoid swamping logs
	if (UnresolvableVectors.Contains(vectorInQuestion))
		bLogHelpInfo = false;
	else
		UnresolvableVectors.Add(vectorInQuestion);
	
	if (!bLogHelpInfo)
		return;

	FString message = FString("Rules for passing vectors to the system:\n");
	message += FString("----------------------------------------\n");
	message += FString("1. Origin and Destination points _must_ be in empty space. They cannot be inside any mesh or right on top of a floor with colliders\n");
	message += FString("--\n");
	message += FString("2. Characters and Pawns however, are allowed to use their collision component as origin or destination, but the character Mesh itself must _not_ be configured as BlockAll or BlockAllDynamic.\n");
	message += FString("You can use the default \"Pawn\" collision profile (default) for character meshes as long as you haven't added it to the ObstacleQueryChannels list in your Don Navigation Manager actor\n");
	message += FString("--\n");
	message += FString("3. Trigger volumes cannot the same object type as obstacles (don't use WorldStatic/WorldDynamic/etc). The reason for this is that the system uses overlaps instead of sweeps for optimal performance.\n");
	message += FString("Please create a new trigger object type for all triggers in your map and use that to avoid trigger volumes being from picked up as obstacles.\n");

	UE_LOG(DoNNavigationLog, Warning, TEXT("%s"), *message);
}

bool ADonNavigationManager::FindPathSolution_StressTesting(AActor* Actor, FVector Destination, TArray<FVector> &PathSolutionRaw, TArray<FVector> &PathSolutionOptimized, UPARAM(ref) const FDoNNavigationQueryParams& QueryParams, UPARAM(ref) const FDoNNavigationDebugParams& DebugParams)
{	
	UPrimitiveComponent* CollisionComponent = Actor ? Cast<UPrimitiveComponent>(Actor->GetRootComponent()) : NULL;

	// Input Validations - I
	if (!Actor || !CollisionComponent)
	{
		FString actorLog = Actor ? *Actor->GetName() : *FString("Invalid");
		FString collisionComponentLog = CollisionComponent ? *CollisionComponent->GetName() : *FString("Invalid");
		UE_LOG(DoNNavigationLog, Error, TEXT("%s"), *FString::Printf(TEXT("Invalid input parameters received. Actor: %s CollisionComponent: %s"), *actorLog, *collisionComponentLog));

		return false;
	}

	FVector Origin = Actor->GetActorLocation();

	// Do we have direct access to the goal?
	FHitResult hitResult;
	const bool bFindInitialOverlaps = true;
	if (IsDirectPathLineSweep(CollisionComponent, Origin, Destination, hitResult, bFindInitialOverlaps))
	{
		PathSolutionRaw.Add(Origin);
		PathSolutionRaw.Add(Destination);
		PathSolutionOptimized = PathSolutionRaw;

		VisualizeSolution(Origin, Destination, PathSolutionRaw, PathSolutionOptimized, DebugParams);

		return true;
	}

	// Input Visualization - I
	if (DebugParams.DrawDebugVolumes)
	{
		DrawDebugPoint_Safe(GetWorld(), Origin, 20.f, FColor::White, true, -1.f);
		DrawDebugPoint_Safe(GetWorld(), Destination, 20.f, FColor::Green, true, -1.f);
	}

	uint64 timerVolumeResolution = DoNNavigation::Debug_GetTimer();

	FDonNavigationVoxel* originVolume = NULL;
	FDonNavigationVoxel* destinationVolume = NULL;

	FVector resolvedOriginCenter = VolumeOriginAt(Origin);
	FVector resolvedDestinationCenter = VolumeOriginAt(Destination);

	bool bResolvedOrigin = false;
	bool bResolvedDestination = false;

	if (!bIsUnbound)
	{
		// Resolve origin and destination volumes
		originVolume = ResolveVolume(Origin, CollisionComponent, QueryParams.bFlexibleOriginGoal, QueryParams.CollisionShapeInflation);
		destinationVolume = ResolveVolume(Destination, CollisionComponent, QueryParams.bFlexibleOriginGoal, QueryParams.CollisionShapeInflation);

		// Input Visualization - II
		if (DebugParams.DrawDebugVolumes)
		{
			if (originVolume)
				DrawDebugVoxel_Safe(GetWorld(), originVolume->Location, NavVolumeExtent(), FColor::White, false, 0.13f, 0, DebugVoxelsLineThickness);

			if (destinationVolume)
				DrawDebugVoxel_Safe(GetWorld(), destinationVolume->Location, NavVolumeExtent(), FColor::Green, false, 0.13f, 0, DebugVoxelsLineThickness);
		}
	}
	else
	{
		bResolvedOrigin = ResolveVector(Origin, resolvedOriginCenter, CollisionComponent, QueryParams.bFlexibleOriginGoal, QueryParams.CollisionShapeInflation);
		bResolvedDestination = ResolveVector(Destination, resolvedDestinationCenter, CollisionComponent, QueryParams.bFlexibleOriginGoal, QueryParams.CollisionShapeInflation);
	}

	// Input Validations - II
	if ((!bIsUnbound && (!originVolume || !destinationVolume)) || (bIsUnbound && (!bResolvedOrigin || !bResolvedDestination)))
	{
		InvalidVolumeErrorLog(originVolume, destinationVolume, Origin, Destination);

		return false;
	}

	// Flexible Origin adaptation:
	if (Origin != Actor->GetActorLocation())
	{
		UE_LOG(DoNNavigationLog, Warning, TEXT("Forcibly moving %s to new origin for viable pathfinding. (Can be disabled in QueryParams)"), *Actor->GetName());
		Actor->SetActorLocation(Origin, false);
	}

	// Input Visualization - II
	if (DebugParams.DrawDebugVolumes)
	{
		DrawDebugVoxel_Safe(GetWorld(), originVolume->Location, NavVolumeExtent(), FColor::White, false, 0.13f, 0, DebugVoxelsLineThickness);
		DrawDebugVoxel_Safe(GetWorld(), destinationVolume->Location, NavVolumeExtent(), FColor::Green, false, 0.13f, 0, DebugVoxelsLineThickness);
	}

	// Load voxel collision profile:
	bool bResultIsValid = false;
	const bool bIgnoreMeshOriginOccupancy = true;
	auto voxelCollisionProfile = GetVoxelCollisionProfileFromMesh(FDonMeshIdentifier(CollisionComponent), bResultIsValid, VoxelCollisionProfileCache_GameThread, bIgnoreMeshOriginOccupancy);

	uint64 timerPathfinding = DoNNavigation::Debug_GetTimer();

	FDonNavigationQueryTask synchronousTask = FDonNavigationQueryTask(
		FDoNNavigationQueryData(Actor, CollisionComponent, Origin, Destination, QueryParams, DebugParams, originVolume, destinationVolume, resolvedOriginCenter, resolvedDestinationCenter, voxelCollisionProfile),
		FDoNNavigationResultHandler(),
		FDonNavigationDynamicCollisionDelegate()
	);

	auto& data = synchronousTask.Data;
	float timeSpend = 0;
	// Core pathfinding algorithm
	while (!data.bGoalFound && timeSpend <= QueryParams.QueryTimeout)
	{
		uint64 timeoutTimer = DoNNavigation::Debug_GetTimer();
		TickNavigationSolver(synchronousTask);
		DoNNavigation::Debug_StopTimer(timeoutTimer);
		timeSpend += timeoutTimer / 1000.0;
	}

	// Goal validation:
	if (!data.bGoalFound)
	{
		DoNNavigation::Debug_StopTimer(timerPathfinding);
		UE_LOG(DoNNavigationLog, Error, TEXT("%s"), *FString::Printf(TEXT("Error: Goal not found. Time spent attempting to solve - %f seconds"), timerPathfinding / 1000.0));

		return false;
	}

	// Translate volume path solution to vector path solution:

	data.bGoalFound = PrepareSolution(synchronousTask);
	if (!data.bGoalFound)
	{
		UE_LOG(DoNNavigationLog, Error, TEXT("%s"), *FString::Printf(TEXT("Goal not found among %d goal trajectory nodes"), data.VolumeVsGoalTrajectoryMap.Num()));

		return false;
	}
	
	PathSolutionRaw.Append(data.PathSolutionRaw);//synchronousTask.PathSolutionRaw.Num());
	// Log time spent:
	DoNNavigation::Debug_StopTimer(timerPathfinding);
	FString calcTime1 = FString::Printf(TEXT("[DoN Navigation]Time spent calculating best path volumes - %f seconds"), timerPathfinding / 1000.0);
	UE_LOG(DoNNavigationLog, Log, TEXT("%s"), *calcTime1);

	// Optimize solution:
	uint64 timerOptimizationPass = DoNNavigation::Debug_GetTimer();
	OptimizePathSolution(CollisionComponent, PathSolutionRaw, PathSolutionOptimized, QueryParams.CollisionShapeInflation);
	DoNNavigation::Debug_StopTimer(timerOptimizationPass);
	FString calcTime3 = FString::Printf(TEXT("Time spent optimizing path solution - %f seconds"), timerOptimizationPass / 1000.0);
	UE_LOG(DoNNavigationLog, Log, TEXT("%s"), *calcTime3);

	// Visualize solution:
	VisualizeSolution(Origin, Destination, PathSolutionRaw, PathSolutionOptimized, DebugParams);

	return true;
}

bool ADonNavigationManager::SchedulePathfindingTask(AActor* Actor, FVector Destination, UPARAM(ref) const FDoNNavigationQueryParams& QueryParams, UPARAM(ref) const FDoNNavigationDebugParams& DebugParams, FDoNNavigationResultHandler ResultHandlerDelegate, FDonNavigationDynamicCollisionDelegate DynamicCollisionListener)
{
	UPrimitiveComponent* CollisionComponent = Actor ? Cast<UPrimitiveComponent>(Actor->GetRootComponent()) : NULL;

	// Input Validations - I
	if (!Actor || !CollisionComponent)
	{	
		FString collisionComponentLog = CollisionComponent ? *CollisionComponent->GetName() : *FString("Invalid");
		UE_LOG(DoNNavigationLog, Error, TEXT("%s"), *FString::Printf(TEXT("Invalid input parameters received. Actor: %s CollisionComponent: %s"), Actor ? *Actor->GetName() : *FString("Invalid"), *collisionComponentLog));

		return false;
	}

	if (!bIsUnbound && !IsLocationWithinNavigableWorld(Destination))
	{
		UE_LOG(DoNNavigationLog, Error, TEXT("Destination %s is outside world bounds. Please clamp your destination within the navigable world or expand world size under settings if necessary."), *Destination.ToString());

		return false;
	}

	// Does this actor already have a running query scheduled with us?
	if (HasTask(Actor))
	{
		if (QueryParams.bForceRescheduleQuery)
		{
			// Aborts the existing task and removes it from the active task list
			CleanupExistingTaskForActor(Actor);
		}
		else
		{
			UE_LOG(DoNNavigationLog, Warning, TEXT("%s"), *FString::Printf(TEXT("%s already has an active task. If you really need to force reschedule a new task, set bForceRescheduleQuery to true in query params."), Actor ? *Actor->GetName() : *FString("Invalid")));

			return false;
		}
	}

	FVector Origin = Actor->GetActorLocation();

	// Do we have direct access to the goal?
	FHitResult hitResult;
	const bool bFindInitialOverlaps = true;
	if (IsDirectPathLineSweep(CollisionComponent, Origin, Destination, hitResult, bFindInitialOverlaps))
	{	
		FDonNavigationQueryTask task(FDoNNavigationQueryData(Actor, CollisionComponent, Origin, Destination, QueryParams, DebugParams, bIsUnbound ? NULL : VolumeAt(Origin), bIsUnbound ? NULL : VolumeAt(Destination), Origin, Destination, FDonVoxelCollisionProfile()), ResultHandlerDelegate, DynamicCollisionListener);

		PackageDirectSolution(task);

		VisualizeSolution(task.Data.Origin, task.Data.Destination, task.Data.PathSolutionRaw, task.Data.PathSolutionOptimized, task.Data.DebugParams);

		// call success delegate immediately
		task.Data.QueryStatus = EDonNavigationQueryStatus::Success;
		task.ResultHandler.ExecuteIfBound(task.Data);		

		UE_LOG(DoNNavigationLog, Verbose, TEXT("Query for %s, %s solved via simple direct pathing"), *task.Data.GetActorName(), *task.Data.Destination.ToString(), task.Data.SolverTimeTaken);

		return true;
	}

	// Input Visualization - I
	if (DebugParams.DrawDebugVolumes)
	{
		DrawDebugPoint_Safe(GetWorld(), Origin,      20.f, FColor::White, true, -1.f);
		DrawDebugPoint_Safe(GetWorld(), Destination, 20.f, FColor::Green, true, -1.f);
	}

	FDonNavigationVoxel* originVolume = NULL;
	FDonNavigationVoxel* destinationVolume = NULL;

	FVector resolvedOriginCenter = VolumeOriginAt(Origin);
	FVector resolvedDestinationCenter = VolumeOriginAt(Destination);

	bool bResolvedOrigin = false;
	bool bResolvedDestination = false;

	if (!bIsUnbound)
	{
		// Resolve origin and destination volumes
		originVolume = ResolveVolume(Origin, CollisionComponent, QueryParams.bFlexibleOriginGoal, QueryParams.CollisionShapeInflation);
		destinationVolume = ResolveVolume(Destination, CollisionComponent, QueryParams.bFlexibleOriginGoal, QueryParams.CollisionShapeInflation);			

		// Input Visualization - II
		if (DebugParams.DrawDebugVolumes)
		{
			if(originVolume)
				DrawDebugVoxel_Safe(GetWorld(), originVolume->Location, NavVolumeExtent(), FColor::White, false, 0.13f, 0, DebugVoxelsLineThickness);

			if(destinationVolume)
				DrawDebugVoxel_Safe(GetWorld(), destinationVolume->Location, NavVolumeExtent(), FColor::Green, false, 0.13f, 0, DebugVoxelsLineThickness);
		}
	}
	else
	{
		bResolvedOrigin = ResolveVector(Origin, resolvedOriginCenter, CollisionComponent, QueryParams.bFlexibleOriginGoal, QueryParams.CollisionShapeInflation);
		bResolvedDestination = ResolveVector(Destination, resolvedDestinationCenter, CollisionComponent, QueryParams.bFlexibleOriginGoal, QueryParams.CollisionShapeInflation);
	}

	// Input Validations - II
	if ((!bIsUnbound && (!originVolume || !destinationVolume)) || (bIsUnbound && (!bResolvedOrigin || !bResolvedDestination)))
	{
		InvalidVolumeErrorLog(originVolume, destinationVolume, Origin, Destination);

		return false;
	}

	// Flexible Origin adaptation:
	if (Origin != Actor->GetActorLocation())
	{
		UE_LOG(DoNNavigationLog, Warning, TEXT("Forcibly moving %s to new origin for viable pathfinding. (Can be disabled in QueryParams)"), *Actor->GetName());
		Actor->SetActorLocation(Origin, false);
	}

	// Load voxel collision profile:
	bool bResultIsValid = false;
	const bool bIgnoreMeshOriginOccupancy = true;
	auto voxelCollisionProfile = GetVoxelCollisionProfileFromMesh(FDonMeshIdentifier(CollisionComponent), bResultIsValid, VoxelCollisionProfileCache_GameThread, bIgnoreMeshOriginOccupancy);
		
	// Prepare task:
	FDonNavigationQueryTask request = FDonNavigationQueryTask( 
			FDoNNavigationQueryData(Actor, CollisionComponent, Origin, Destination, QueryParams, DebugParams, originVolume, destinationVolume, resolvedOriginCenter, resolvedDestinationCenter, voxelCollisionProfile),
			ResultHandlerDelegate, 
			DynamicCollisionListener
		);

	request.Data.QueryStatus = EDonNavigationQueryStatus::InProgress;

	// Schedule this task
	AddPathfindingTask(request);
	
	return true;
}

void ADonNavigationManager::AddPathfindingTask(const FDonNavigationQueryTask& Task)
{
	if (!bMultiThreadingEnabled)
	{
		ActiveNavigationTasks.Add(Task);
	}
	else
	{
		auto owner = Task.Data.Actor.Get();
		ensure(owner);
		ActiveNavigationTaskOwners.Add(owner);
	    NewNavigationTasks.Enqueue(Task);

#if DEBUG_DoNAI_THREADS
		UE_LOG(DoNNavigationLog, Display, TEXT("[%s] [game thread] Enqueued new nav task"), owner ? *owner->GetName() : *FString("Unknown"));
#endif //DEBUG_DoNAI_THREADS*/
	}
}

void ADonNavigationManager::ReceiveAsyncNavigationTasks()
{
	if (NewNavigationTasks.IsEmpty())
		return;

	FDonNavigationQueryTask newlyArrivedTask;
	bool bHasNewTask = NewNavigationTasks.Dequeue(newlyArrivedTask);

	if (bHasNewTask)
	{
		if (newlyArrivedTask.RequestType == EDonNavigationRequestType::New)
		{
			ActiveNavigationTasks.Add(newlyArrivedTask);

#if DEBUG_DoNAI_THREADS
			auto owner = newlyArrivedTask.Data.Actor.Get();
			UE_LOG(DoNNavigationLog, Display, TEXT("[%s] [async thread] Received new nav task"), owner ? *owner->GetName() : *FString("Unknown"));
#endif //DEBUG_DoNAI_THREADS*/
		}
		else if (newlyArrivedTask.RequestType == EDonNavigationRequestType::Abort)
		{
			AbortPathfindingTask_Internal(newlyArrivedTask.Data.Actor.Get());

#if DEBUG_DoNAI_THREADS
			UE_LOG(DoNNavigationLog, Display, TEXT("[%s] [async thread] Received new abort request"), actor ? *actor->GetName() : *FString("Unknown"));
#endif //DEBUG_DoNAI_THREADS*/
		}
	}
}

#if 0
void ADonNavigationManager::ReceiveAsyncNavigationTasks()
{
	if (NewNavigationTasks.IsEmpty())
		return;

	FDonNavigationQueryTask newlyArrivedTask;
	bool bHasNewTask = NewNavigationTasks.Dequeue(newlyArrivedTask);

	if (bHasNewTask)
	{
		ActiveNavigationTasks.Add(newlyArrivedTask);

#if DEBUG_DoNAI_THREADS
		auto owner = newlyArrivedTask.Data.Actor.Get();
		UE_LOG(DoNNavigationLog, Display, TEXT("[%s] [async thread] Received new nav task"), owner ? *owner->GetName() : *FString("Unknown"));
#endif //DEBUG_DoNAI_THREADS*/
	}
}
#endif

void ADonNavigationManager::AbortPathfindingTask(AActor* Actor)
{
	if (!HasTask(Actor))
		return; // no-op. Also essential to prevent multi-threading issues due to superfluous abort requests piling up in the newNavAborts TQueue!

	if (!bMultiThreadingEnabled)
	{
		AbortPathfindingTask_Internal(Actor);
	}
	else
	{
		ActiveNavigationTaskOwners.Remove(Actor);
		//NewNavigationAborts.Enqueue(Actor);
		FDonNavigationQueryTask abortTask(Actor, EDonNavigationRequestType::Abort);
		NewNavigationTasks.Enqueue(abortTask);

#if DEBUG_DoNAI_THREADS
		UE_LOG(DoNNavigationLog, Display, TEXT("[%s] [game thread] Enqueued new abort request"), Actor ? *Actor->GetName() : *FString("Unknown"));
#endif //DEBUG_DoNAI_THREADS*/
	}
}

#if 0
void ADonNavigationManager::ReceiveAsyncAbortRequests()
{
	if (NewNavigationAborts.IsEmpty())
		return;

	AActor* actor;
	bool bHasAbortRequest = NewNavigationAborts.Dequeue(actor);

	if (bHasAbortRequest)
	{

#if DEBUG_DoNAI_THREADS
		UE_LOG(DoNNavigationLog, Display, TEXT("[%s] [async thread] Received new abort request"), actor ? *actor->GetName() : *FString("Unknown"));
#endif //DEBUG_DoNAI_THREADS*/

		AbortPathfindingTask_Internal(actor);
	}
}
#endif

void ADonNavigationManager::AbortPathfindingTask_Internal(AActor* Actor)
{
	for (int32 i = ActiveNavigationTasks.Num() - 1; i >= 0; i--)
	{
		if (ActiveNavigationTasks[i].Data.Actor.Get() == Actor)
		{
			AbortPathfindingTaskByIndex(i);
		}
	}
}

void ADonNavigationManager::StopListeningToDynamicCollisionsForPath(FDonNavigationDynamicCollisionDelegate ListenerToClear, UPARAM(ref) const FDoNNavigationQueryData& QueryData)
{
	for (int32 i = 0; i < QueryData.VolumeSolutionOptimized.Num(); i++)
	{
		StopListeningToDynamicCollisionsForPathIndex(ListenerToClear, QueryData, i);
	}
}

void ADonNavigationManager::StopListeningToDynamicCollisionsForPathIndex(FDonNavigationDynamicCollisionDelegate ListenerToClear, UPARAM(ref) const FDoNNavigationQueryData& QueryData, const int32 VolumeIndex)
{
	auto volume = QueryData.VolumeSolutionOptimized[VolumeIndex]; // Unsafe, but this is a calculated performance-risk trade-off. The most common usecase (it's right above) iterates over fixed bounds.
	if (!volume)
	{
		UE_LOG(DoNNavigationLog, Warning, TEXT("Invalid path passed to StopListeningToDynamicCollisionsForPath. Ideally this should never happen, please check the source triggering this call."));

		return;
	}

	volume->DynamicCollisionNotifyees.RemoveAll([&ListenerToClear](const FDonNavigationDynamicCollisionNotifyee& notifyee) {return notifyee.Listener == ListenerToClear; });

	if (QueryData.QueryParams.bPreciseDynamicCollisionRepathing)
	{
		for (auto offset : QueryData.VoxelCollisionProfile.RelativeVoxelOccupancy)
		{
			auto volumeFromProfile = VolumeAtSafe(volume->X + offset.X, volume->Y + offset.Y, volume->Z + offset.Z);
			if (volumeFromProfile)
				volumeFromProfile->DynamicCollisionNotifyees.RemoveAll([&ListenerToClear](const FDonNavigationDynamicCollisionNotifyee& notifyee) {return notifyee.Listener == ListenerToClear; });
		}
	}
}

void ADonNavigationManager::AbortPathfindingTaskByIndex(int32 TaskIndex)
{
	auto owner = ActiveNavigationTasks[TaskIndex].Data.Actor.Get();

	auto collisionListener = ActiveNavigationTasks[TaskIndex].DynamicCollisionListener;

	StopListeningToDynamicCollisionsForPath(collisionListener, ActiveNavigationTasks[TaskIndex].Data);
		
	ActiveNavigationTasks.RemoveAtSwap(TaskIndex);

#if DEBUG_DoNAI_THREADS
	UE_LOG(DoNNavigationLog, Display, TEXT("[%s] [%s] Executing new abort request"), owner ? *owner->GetName() : *FString("Unknown"), IsInGameThread() ? *FString("[game thread]") : *FString("[async thread]"));
#endif //DEBUG_DoNAI_THREADS*/
}


void ADonNavigationManager::TickNavigationSolver(FDonNavigationQueryTask& task)
{	
	auto& data = task.Data;

	data.SolverIterationCount++;

	if (!data.Frontier.empty())
	{
		// Move towards goal by fetching the "best neighbor" of the previous volume from the Frontier priority queue
		// The best neighbor is defined as the node most likely to lead us towards the goal
		auto currentVolume = data.Frontier.get(); 

		// Have we reached the goal?
		if (currentVolume == data.DestinationVolume)
		{
			data.bGoalFound = true;
			return;
		}

		// Discover all neighbors for current volume:
		const auto& neighbors = FindOrSetupNeighborsForVolume(currentVolume);
		
		// Evaluate each neighbor for suitability, assign points, add to Frontier
		for (auto neighbor : neighbors)
		{	
			ExpandFrontierTowardsTarget(task, currentVolume, neighbor);
		}
	}
}

void ADonNavigationManager::PackageRawSolution(FDonNavigationQueryTask& task)
{
	task.Data.PathSolutionOptimized = task.Data.PathSolutionRaw;

	if (bIsUnbound)
		return;

	for (auto solutionNode : task.Data.PathSolutionRaw)
	{
		auto volume = AppendVolumeList(solutionNode, task);
		AddCollisionListenerToVolumeFromTask(volume, task);
	}	
}

void ADonNavigationManager::PackageDirectSolution(FDonNavigationQueryTask& Task)
{
	//Task.Data.PathSolutionRaw.Add(Task.Data.Origin);
	Task.Data.PathSolutionRaw.Add(Task.Data.Destination);
	Task.Data.PathSolutionOptimized = Task.Data.PathSolutionRaw;

	if (!bIsUnbound)
	{
		if (!Task.Data.QueryParams.bIgnoreDynamicCollisionRepathingForDirectGoals)
		{
			AppendVolumeListFromRange(Task.Data.Origin, Task.Data.Destination, Task);
			for(auto volume : Task.Data.VolumeSolutionOptimized)
				AddCollisionListenerToVolumeFromTask(volume, Task);

			Task.Data.VolumeSolution = Task.Data.VolumeSolutionOptimized;
		}
		else
		{
			Task.Data.VolumeSolution.Add(Task.Data.OriginVolume);
			Task.Data.VolumeSolution.Add(Task.Data.DestinationVolume);
			Task.Data.VolumeSolutionOptimized = Task.Data.VolumeSolution;
		}
	}
}

void ADonNavigationManager::TickScheduledPathfindingTasks(float DeltaSeconds, int32 MaxIterationsPerTick)
{
	const int32 numTasks = ActiveNavigationTasks.Num();
	int32 maxTasksThisIteration = 0, maxIterationsPerTask = 0;

	if (!numTasks)
		return;

	if (numTasks <= MaxIterationsPerTick)
	{
		maxTasksThisIteration = numTasks;
		maxIterationsPerTask = MaxIterationsPerTick / numTasks;
	}
	else
	{
		maxTasksThisIteration = MaxIterationsPerTick;
		maxIterationsPerTask = 1;
	}	

	for (int32 i = maxTasksThisIteration - 1; i >= 0; i--)
	{
		//SCOPE_CYCLE_COUNTER(STAT_PathfindingSolver);

		auto& task = ActiveNavigationTasks[i];
		auto& data = task.Data;

		// Query timeout?
		if (data.SolverTimeTaken >= data.QueryParams.QueryTimeout)
		{
			// Do we at least have the unoptimized solution ready yet? If yes, simply return it! The unoptimized solution is perfectly usable for navigation.
			if (data.bGoalFound && !data.bGoalOptimized)
			{
				UE_LOG(DoNNavigationLog, Warning, TEXT("Query timed out before optimization was complete, returning unoptimized solution for Actor %s. Num iterations : %d"), *data.GetActorName(), data.SolverIterationCount);
				
				PackageRawSolution(task); // @FeatureIdea: we can construct a partially optimized solution by merging optimized and unoptimized results.

				VisualizeSolution(data.Origin, data.Destination, data.PathSolutionRaw, data.PathSolutionOptimized, data.DebugParams);

				data.QueryStatus = EDonNavigationQueryStatus::Success;
			}			
			else
			{
				UE_LOG(DoNNavigationLog, Error, TEXT("Query timed out for Actor %s. Num iterations : %d"), *data.GetActorName(), data.SolverIterationCount);

				data.QueryStatus = EDonNavigationQueryStatus::TimedOut;

			#if WITH_EDITOR
				DrawDebugSphere_Safe(GetWorld(), data.Destination, 15.f, 8.f, FColor::Red, true, 15.f);
			#endif
			}
		}
		else
		{
			int32 iterationsProcessed = 1;

			// Core pathfinding algorithm
			while (!data.bGoalFound && iterationsProcessed <= maxIterationsPerTask)
			{
				TickNavigationSolver(task);
				iterationsProcessed++;
			}

			data.SolverTimeTaken += DeltaSeconds;

			// Is pathfinding complete?
			if (data.bGoalFound)
			{
				TickNavigationOptimizerCycle(task, iterationsProcessed, maxIterationsPerTask);
			}
			// Or path has no solution?
			else if (data.Frontier.empty() && data.Frontier_Unbound.empty())
			{
				UE_LOG(DoNNavigationLog, Error, TEXT("No pathfinding solution exists for query %s, %s"), *data.GetActorName(), *data.Destination.ToString());

				data.QueryStatus = EDonNavigationQueryStatus::QueryHasNoSolution;

				#if WITH_EDITOR
					DrawDebugSphere_Safe(GetWorld(), data.Destination, 15.f, 8.f, FColor::Red, true, 15.f);
				#endif
			}
		}
		
		if (task.IsQueryComplete())
		{	
			CompleteNavigationTask(i);
		}
	}
}

void ADonNavigationManager::TickScheduledPathfindingTasks_Safe(float DeltaSeconds, int32 MaxIterationsPerTick)
{
	TickScheduledPathfindingTasks(DeltaSeconds, MaxIterationsPerTick);
}

void ADonNavigationManager::CompleteNavigationTask(int32 TaskIndex)
{
	bool bSynchronousOperation = !bMultiThreadingEnabled;

	if (bSynchronousOperation)
	{
		// During synchronous calls the delegate owner is capable of internally launching of a new query that will check for the existing task when we execute the delegate.
		// Therefore, to ensure deterministic behavior we first remove the task and only _then_ launch the delegte on a safe copy:

		auto task_safecopy = ActiveNavigationTasks[TaskIndex];

		// Remove this task
		ActiveNavigationTasks.RemoveAtSwap(TaskIndex);

		// Notify owner
		task_safecopy.ResultHandler.ExecuteIfBound(task_safecopy.Data);


	}
	else
	{
		auto owner = ActiveNavigationTasks[TaskIndex].Data.Actor.Get();

		CompletedNavigationTasks.Enqueue(ActiveNavigationTasks[TaskIndex]);
		ActiveNavigationTasks.RemoveAtSwap(TaskIndex);

#if DEBUG_DoNAI_THREADS
		UE_LOG(DoNNavigationLog, Display, TEXT("[%s] [async thread] Enqueued new nav result"), owner ? *owner->GetName() : *FString("Unknown"));
#endif //DEBUG_DoNAI_THREADS*/
	}

}

bool ADonNavigationManager::PrepareSolution(FDonNavigationQueryTask& Task)
{
	auto& data = Task.Data;

	bool bGoalFound = PathSolutionFromVolumeTrajectoryMap(data.OriginVolume, data.DestinationVolume, data.VolumeVsGoalTrajectoryMap, data.VolumeSolution, data.PathSolutionRaw, data.Origin, data.Destination, data.DebugParams);

	return bGoalFound;
}

void ADonNavigationManager::TickNavigationOptimizerCycle(FDonNavigationQueryTask& task, int32& IterationsProcessed, const int32 MaxIterationsPerTask)
{
	auto& data = task.Data;

	// If optimization hasn't begun, perform setup work:

	if (!data.bOptimizationInProgress)
	{
		data.bGoalFound = PrepareSolution(task);

		if(!data.bGoalFound)
		{
			UE_LOG(DoNNavigationLog, Error, TEXT("%s"), *FString::Printf(TEXT("Query complete, but goal not found in %d trajectory nodes. Unable to proceed"), data.VolumeVsGoalTrajectoryMap.Num()));

			data.QueryStatus = EDonNavigationQueryStatus::Failure;

			#if WITH_EDITOR
			DrawDebugSphere_Safe(GetWorld(), data.Destination, 15.f, 8.f, FColor::Red, true, 15.f);
			#endif

			return;
		}

		bool startOptimizer = !data.QueryParams.bSkipOptimizationPass;

		if (startOptimizer)
			data.BeginOptimizationCycle();
		else
		{
			UE_LOG(DoNNavigationLog, Verbose, TEXT("Query for %s, %s is complete (with optimization disabled). Solved in %f seconds"), *data.GetActorName(), *data.Destination.ToString(), data.SolverTimeTaken);

			PackageRawSolution(task);

			VisualizeSolution(data.Origin, data.Destination, data.PathSolutionRaw, data.PathSolutionOptimized, data.DebugParams);

			data.QueryStatus = EDonNavigationQueryStatus::Success;
		}		

		return;
	}

	// Validate collision component:
	if (!data.CollisionComponent.IsValid())
	{
		data.QueryStatus = EDonNavigationQueryStatus::Failure;

		return;
	}

	// Optimization cycles:	
	while (!data.bGoalOptimized && IterationsProcessed <= MaxIterationsPerTask)
	{
		TickNavigationOptimizer(task);
		IterationsProcessed++;
	}

	// Is Optimization complete?
	if (data.bGoalOptimized)
	{
		// Add dynamic collision listeners
		if (!bIsUnbound)
		{
			for (auto volume : data.VolumeSolutionOptimized)
				AddCollisionListenerToVolumeFromTask(volume, task);
		}

		VisualizeSolution(data.Origin, data.Destination, data.PathSolutionRaw, data.PathSolutionOptimized, data.DebugParams);

		UE_LOG(DoNNavigationLog, Verbose, TEXT("Query for %s, %s is complete. Solved in %f seconds"), *data.GetActorName(), *data.Destination.ToString(), data.SolverTimeTaken);

		data.QueryStatus = EDonNavigationQueryStatus::Success; // Finally, the task has succeeded!
	}
}

void ADonNavigationManager::TickNavigationOptimizer(FDonNavigationQueryTask& task)
{
	auto& data = task.Data;

	// Tight-loop equivalent (provided just for clarity): 
	// for (int32 optimizer_i = 0; i < PathSolution.Num() - 1;)

	// Stop the optimizer if we have exhausted all optimization paths (i.e. all combinations of solution nodes have been tested)
	if (data.optimizer_i == data.PathSolutionRaw.Num() - 2)
	{	
		data.PathSolutionOptimized.Add(data.PathSolutionRaw.Last());
		
		if (!bIsUnbound)
			AppendVolumeList(data.PathSolutionRaw.Last(), task);

		data.bGoalOptimized = true;

		return;
	}

	// Tight-loop equivalent (provided just for clarity): 
	//for (int32 optimizer_j = PathSolution.Num() - 1; j > i; j--)

	FHitResult OutHit;
	TArray<AActor*> actorsToIgnore;
	
	const bool bConsiderInitialOverlaps = true;	
	FVector start = data.PathSolutionRaw[data.optimizer_i];
	FVector end = data.PathSolutionRaw[data.optimizer_j];
	
	// Do we see a direct path from start to end?
	if (IsDirectPathLineSweep(data.CollisionComponent.Get(), start, end, OutHit, bConsiderInitialOverlaps, task.Data.QueryParams.CollisionShapeInflation))
	{
		data.PathSolutionOptimized.Add(data.PathSolutionRaw[data.optimizer_j]);		

		// Optimizer has reached the goal?
		if (data.optimizer_j == data.PathSolutionRaw.Num() - 1)
		{		
			data.bGoalOptimized = true;			
		}
		else
		{	
			// proceed to next directly accessible point for next round of optimization
			data.optimizer_i = data.optimizer_j;
			data.optimizer_j = data.PathSolutionRaw.Num() - 1;
		}		

		// register dynamic collision listeners for the owner of this navigation query:
		if (!bIsUnbound)
			AppendVolumeListFromRange(start, end, task);

		return;
	}
	else
	{
		// Try forging a direct path to the next available node:
		data.optimizer_j--;

		// Are all possible optimization paths exhausted for this node?
		if (data.optimizer_j == data.optimizer_i || data.MaxSweepAttemptsReachedForNode())
		{	
			if (!bIsUnbound)
				AppendVolumeList(start, task);

			// Move on to the next point in the solution and try our luck from there:
			data.optimizer_i++;
			data.optimizer_j = data.PathSolutionRaw.Num() - 1;

			data.PathSolutionOptimized.Add(data.PathSolutionRaw[data.optimizer_i]);
		}
	}
}

// Dynamic Collision Listeners
void ADonNavigationManager::AddCollisionListenerToVolumeFromTask(FDonNavigationVoxel* Volume, FDonNavigationQueryTask& task)
{
	if (!Volume || !task.DynamicCollisionListener.IsBound())
		return;

	auto listener = task.DynamicCollisionListener;
	auto payload = FDonNavigationDynamicCollisionPayload(task.Data.QueryParams.CustomDelegatePayload, *Volume);
	auto notifyee = FDonNavigationDynamicCollisionNotifyee(listener, payload);

#if WITH_EDITOR	
	if (bRunDebugValidationsForDynamicCollisions && Volume->DynamicCollisionNotifyees.Contains(notifyee))
	{
		FString errorMessage = FString::Printf(TEXT("ALERT: Navigator %s is attempting to add a duplicate collision listener to volume %d %d %d \n"), *task.Data.GetActorName(), Volume->X, Volume->Y, Volume->Z);
		errorMessage += FString("This is usually a sign that you're not deregistering collision listeners after you're done using a navigation query.\n");
		errorMessage += FString("Please ensure that _any_ code scheduling a navigation task even once _must_ clear all the collision listeners it acquires \n");
		errorMessage += FString("after it is no longer interested in listening to dynamic collision along the requested path. This is also vital to maintain optimal performance.\n");
		UE_LOG(DoNNavigationLog, Error, TEXT("%s"), *errorMessage);
	}
#endif // WITH_EDITOR	
	
	
	// Add dynamic listeners:
	Volume->DynamicCollisionNotifyees.AddUnique(notifyee);	

	if (task.Data.QueryParams.bPreciseDynamicCollisionRepathing)
	{
		for (auto offset : task.Data.VoxelCollisionProfile.RelativeVoxelOccupancy)
		{
			auto volumeFromProfile = VolumeAtSafe(Volume->X + offset.X, Volume->Y + offset.Y, Volume->Z + offset.Z);
			if (volumeFromProfile)
			{
				volumeFromProfile->DynamicCollisionNotifyees.AddUnique(notifyee);				
			}
		}
	}
}

FDonNavigationVoxel* ADonNavigationManager::AppendVolumeList(FVector Location, FDonNavigationQueryTask& task)
{
	auto volume = VolumeAt(Location);
	
	task.Data.VolumeSolutionOptimized.Add(volume);

	return volume;
}

void ADonNavigationManager::AppendVolumeListFromRange(FVector Start, FVector End, FDonNavigationQueryTask& task)
{
	auto startVolume = VolumeAt(Start);
	auto endVolume = VolumeAt(End);
	auto directionNormal = (End - Start).GetSafeNormal();
	
	float deltaX = (End.X - Start.X);
	float deltaY = (End.Y - Start.Y);
	float deltaZ = (End.Z - Start.Z);	
	
	int32 countX = FMath::Abs(deltaX / VoxelSize);
	int32 countY = FMath::Abs(deltaY / VoxelSize);
	int32 countZ = FMath::Abs(deltaZ / VoxelSize);

	FVector current = Start;
	int32 counter = 1;
	int32 countMax = FMath::Max3(countX, countY, countZ);

	if (countMax == 0)
		return;

	while ( counter <= countMax)
	{	
		AppendVolumeList(current, task);

		float multiplier = (counter / (float)countMax);
		current = Start + multiplier * FVector(deltaX, deltaY, deltaZ);

		counter++;
	}
}

// AI Utility Functions

FVector ADonNavigationManager::FindRandomPointFromActorInNavWorld(AActor* Actor, float Distance, bool& bFoundValidResult, float MaxDesiredAltitude/* = -1.f*/, float MaxZAngularDispacement/* = 15.f*/, int32 MaxAttempts/* = 5*/)
{
	return FindRandomPointAroundOriginInNavWorld(Actor, Actor->GetActorLocation(), Distance, bFoundValidResult, MaxDesiredAltitude, MaxZAngularDispacement, MaxAttempts);
}

FVector ADonNavigationManager::FindRandomPointAroundOriginInNavWorld(AActor* NavigationActor, FVector Origin, float Distance, bool& bFoundValidResult, float MaxDesiredAltitude/* = -1.f*/, float MaxZAngularDispacement/* = 15.f*/, int32 MaxAttempts/* = 5*/)
{
	bFoundValidResult = false;

	if (!NavigationActor)
		return FVector::ZeroVector;
	
	FVector baseDisplacement = FVector::ForwardVector * Distance;
	FVector newDestination = Origin + baseDisplacement;

	for (int32 i = 0; i < MaxAttempts; i++)
	{
		float maxZAngularDispacement = FMath::Abs(MaxZAngularDispacement);
		FRotator newDirection = FRotator(FMath::FRandRange(-maxZAngularDispacement, maxZAngularDispacement), FMath::FRandRange(0.0, 360.0), FMath::FRandRange(0.0, 360.0));
		newDestination = Origin + newDirection.RotateVector(baseDisplacement);

		if (MaxDesiredAltitude != -1.f)
			newDestination = FVector(newDestination.X, newDestination.Y, FMath::Clamp(newDestination.Z, newDestination.Z, MaxDesiredAltitude));

		const bool shouldSweep = false;
		bool bInitialPositionCollides = false;

		if (IsLocationBeneathLandscape(newDestination))
			continue;

		if (bIsUnbound)
		{
			FVector resolvedDestination;
			bool bIsResolvable = ResolveVector(newDestination, resolvedDestination, Cast<UPrimitiveComponent>(NavigationActor->GetRootComponent()), bInitialPositionCollides, 0.f, shouldSweep);

			if (bIsResolvable)
			{
				bFoundValidResult = true;

				return newDestination;
			}
		}
		else
		{
			auto destinationVolume = GetClosestNavigableVolume(newDestination, Cast<UPrimitiveComponent>(NavigationActor->GetRootComponent()), bInitialPositionCollides, 0.f, shouldSweep);

			if (destinationVolume)
			{
				bFoundValidResult = true;

				return destinationVolume->Location;
			}
		}
	}

	return FVector::ZeroVector;
}


bool ADonNavigationManager::IsLocationBeneathLandscape(FVector Location, float LineTraceHeight/* = 3000..f*/)
{
	//DrawDebugLine_Safe(GetWorld(), Location, Location + FVector(0, 0, LineTraceHeight), FColor::Magenta, true, -1.f, 0, 1.f);

	FHitResult outHit;
	TArray<AActor*> actorsToIgnore;	

	bool bDirectPath = IsDirectPathLineTrace(Location, Location + FVector(0, 0, LineTraceHeight), outHit, actorsToIgnore);

	if (bDirectPath || !outHit.GetActor())
		return false;	

	return outHit.GetActor()->GetClass()->GetName().Equals(FString("Landscape")); // Ideally we should check for ALandscape after extracting class but ALandscape doesn't seem to be exposed outside editor
}

void ADonNavigationManager::VisualizeDynamicCollisionListeners(FDonNavigationDynamicCollisionDelegate Listener, UPARAM(ref) const FDoNNavigationQueryData& QueryData)
{
	for (auto volume : QueryData.VolumeSolutionOptimized)
	{	
		bool bContainsListener = volume->DynamicCollisionNotifyees.ContainsByPredicate([&Listener](const FDonNavigationDynamicCollisionNotifyee& notifyee) {return notifyee.Listener == Listener; });
		if (bContainsListener)
		{	
			DrawDebugVoxel_Safe(GetWorld(), volume->Location, NavVolumeExtent(), FColor::Yellow, true, -1.f, 0, DebugVoxelsLineThickness);
		}
		else
		{	
			DrawDebugVoxel_Safe(GetWorld(), volume->Location, NavVolumeExtent(), FColor::Red, true, -1.f, 0, DebugVoxelsLineThickness);
		}
	}
}

void ADonNavigationManager::VisualizeNAVResult(const TArray<FVector>& PathSolution, FVector Source, FVector Destination, bool Reset, const FDoNNavigationDebugParams& DebugParams, FColor const& LineColor)
{
	FVector entryPoint = Source;
	FVector previousEntryPoint;

	for (int32 i = 0; i < PathSolution.Num(); i++)
	{
		previousEntryPoint = entryPoint;
		entryPoint = PathSolution[i];

		DrawDebugLine_Safe(GetWorld(), previousEntryPoint, entryPoint, LineColor, true, DebugParams.LineDuration, 0.f, DebugParams.LineThickness);
		DrawDebugPoint_Safe(GetWorld(), entryPoint, 10.f, FColor::Blue, true, DebugParams.LineDuration);
	}
}

void ADonNavigationManager::VisualizeSolution(FVector source, FVector destination, const TArray<FVector>& PathSolutionRaw, const TArray<FVector>& PathSolutionOptimized, const FDoNNavigationDebugParams& DebugParams)
{
	if (DebugParams.VisualizeRawPath)
	{
		ADonNavigationManager::VisualizeNAVResult(PathSolutionRaw, source, destination, true, DebugParams, FColor::Yellow);
			
	}

	if (DebugParams.VisualizeOptimizedPath)
	{
		ADonNavigationManager::VisualizeNAVResult(PathSolutionOptimized, source, destination, true, DebugParams, FColor::Black);
	}
}

void ADonNavigationManager::DrawDebugLine_Safe(UWorld* World, FVector LineStart, FVector LineEnd, FColor Color, bool bPersistentLines, float LifeTime, uint8 DepthPriority, float Thickness)
{
	if(!bMultiThreadingEnabled)
		DrawDebugLine(World, LineStart, LineEnd, Color, bPersistentLines, LifeTime, DepthPriority, Thickness);
	else
		DrawDebugLinesQueue.Enqueue(FDrawDebugLineRequest(LineStart, LineEnd, Color, bPersistentLines, LifeTime, DepthPriority, Thickness));
}

void ADonNavigationManager::DrawDebugPoint_Safe(UWorld* World, FVector PointLocation, float PointThickness, FColor Color, bool bPersistentLines, float LifeTime)
{
	if (!bMultiThreadingEnabled)
		DrawDebugPoint(World, PointLocation, PointThickness, Color, bPersistentLines, LifeTime);
	else
		DrawDebugPointsQueue.Enqueue(FDrawDebugPointRequest(PointLocation, PointThickness, Color, bPersistentLines, LifeTime));
}

void ADonNavigationManager::DrawDebugSphere_Safe(UWorld* World, FVector Center, float Radius, float Segments, FColor Color, bool bPersistentLines, float LifeTime)
{
	if (!bMultiThreadingEnabled)
		DrawDebugSphere(GetWorld(), Center, Radius, Segments, Color, bPersistentLines, LifeTime);
	else
		DrawDebugSpheresQueue.Enqueue(FDrawDebugSphereRequest(Center, Radius, Segments, Color, bPersistentLines, LifeTime));
}

void ADonNavigationManager::DrawDebugVoxel_Safe(UWorld* World, FVector Center, FVector Box, FColor Color, bool bPersistentLines, float LifeTime, uint8 DepthPriority, float Thickness)
{
	if (!bMultiThreadingEnabled)
		DrawDebugVoxel(World, Center, Box, Color, bPersistentLines, LifeTime, DepthPriority, Thickness);
	else
		DrawDebugVoxelsQueue.Enqueue(FDrawDebugVoxelRequest(Center, Box, Color, bPersistentLines, LifeTime, DepthPriority, Thickness));
}

// New 2017 features!
void ADonNavigationManager::SetIsUnbound(bool bIsUnboundIn)
{
	bIsUnbound = bIsUnboundIn;

	RefreshPerformanceSettings();
}

