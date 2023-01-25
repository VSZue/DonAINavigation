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

#include <functional>
#include <queue>
#include <vector>

#if PLATFORM_WINDOWS
#include "Windows/AllowWindowsPlatformTypes.h"
#elif PLATFORM_XBOXONE
#include "XboxOneAllowPlatformTypes.h"
#endif


namespace DoNNavigation
{
	// Priority Queue for the pathfinding algorithm to use:
	template<typename T, typename Number = uint32>
	struct PriorityQueue {
		typedef std::pair<Number, T> PQElement;
		std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement >> elements;

		inline bool empty() { return elements.empty(); }

		inline void put(T item, Number priority) {
			elements.emplace(priority, item);
		}

		inline T get() {
			T best_item = elements.top().second;
			elements.pop();
			return best_item;
		}
	};

	// Debug timer functions for profiling parts of the plugin that aren't easily profiled via Unreal's profiler
	// Eg: For profiling initial collision sampling on map load, etc
	static FORCEINLINE uint64 Debug_GetTimeMs64()
	{
#if PLATFORM_WINDOWS || PLATFORM_XBOXONE
		/* Windows */
		FILETIME ft;
		LARGE_INTEGER li;

		/* Get the amount of 100 nano seconds intervals elapsed since January 1, 1601 (UTC) and copy it
		* to a LARGE_INTEGER structure. */
		GetSystemTimeAsFileTime(&ft);
		li.LowPart = ft.dwLowDateTime;
		li.HighPart = ft.dwHighDateTime;

		uint64 ret = li.QuadPart;
		ret -= 116444736000000000LL; /* Convert from file time to UNIX epoch time. */
		ret /= 10000; /* From 100 nano seconds (10^-7) to 1 millisecond (10^-3) intervals */

		return ret;
#else
		/* Linux */
		struct timeval tv;

		gettimeofday(&tv, NULL);

		uint64 ret = tv.tv_usec;
		/* Convert from micro seconds (10^-6) to milliseconds (10^-3) */
		ret /= 1000;

		/* Adds the seconds (10^0) after converting them to milliseconds (10^-3) */
		ret += (tv.tv_sec * 1000);

		return ret;
#endif
	}

	uint64 FORCEINLINE Debug_GetTimer()
	{
		return Debug_GetTimeMs64();
	}

	void FORCEINLINE Debug_StopTimer(uint64& timer)
	{
		timer = Debug_GetTimeMs64() - timer;
	}
}

#if PLATFORM_WINDOWS
 #include "Windows/HideWindowsPlatformTypes.h"
 #endif