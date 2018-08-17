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

#include "DonNavigatorInterface.generated.h"

UINTERFACE(MinimalAPI)
class UDonNavigator : public UInterface
{
	GENERATED_BODY()
};

struct FDonVoxelCollisionProfile;

class DONAINAVIGATION_API IDonNavigator
{
	GENERATED_BODY()

public:

	// TBD - earlier this was meant to enforce that users of the API would provide minimum metadata for navigators which the system would then use in its calculations, 
	// but this has since been sidelined to make it easier for new consumers of the system to rapidly get going.

	/* Optional custom movement input. If not provided, default AddMovementInput behavior will be called on the pawn or character*/
	UFUNCTION(BlueprintNativeEvent, BlueprintCallable, Category = "Don Navigation")
	void AddMovementInputCustom(FVector WorldDirection, float ScaleValue);	

	UFUNCTION(BlueprintNativeEvent, BlueprintCallable, Category = "Don Navigation")
	void OnLocomotionBegin();

	UFUNCTION(BlueprintNativeEvent, BlueprintCallable, Category = "Don Navigation")
	void OnLocomotionEnd(const bool bLocomotionSuccess);

	UFUNCTION(BlueprintNativeEvent, BlueprintCallable, Category = "Don Navigation")
	void OnLocomotionAbort();

	UFUNCTION(BlueprintNativeEvent, BlueprintCallable, Category = "Don Navigation")
	void OnNextSegment(FVector NextPoint);	
};