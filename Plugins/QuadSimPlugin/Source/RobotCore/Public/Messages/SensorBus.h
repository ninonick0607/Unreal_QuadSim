// SensorBus.h
#pragma once
#include "CoreMinimal.h"
#include <atomic>
#include "SensorMessages.h"
#include "SensorBus.generated.h"

template<typename T, int Capacity>
class TRingBufferSPSC
{
public:
	TRingBufferSPSC() : head(0), tail(0) {}
	bool Push(const T& item)
	{
		const int h = head.load(std::memory_order_relaxed);
		const int n = (h + 1) % Capacity;
		if (n == tail.load(std::memory_order_acquire)) return false; // full
		buffer[h] = item;
		head.store(n, std::memory_order_release);
		return true;
	}
	bool Pop(T& out)
	{
		const int t = tail.load(std::memory_order_relaxed);
		if (t == head.load(std::memory_order_acquire)) return false; // empty
		out = buffer[t];
		tail.store((t + 1) % Capacity, std::memory_order_release);
		return true;
	}
	bool PeekLatest(T& out) const
	{
		const int h = head.load(std::memory_order_acquire);
		const int t = tail.load(std::memory_order_acquire);
		if (t == h) return false;
		const int last = (h + Capacity - 1) % Capacity;
		out = buffer[last];
		return true;
	}
private:
	T buffer[Capacity];
	std::atomic<int> head, tail;
};

UCLASS(BlueprintType)
class USensorBus final : public UObject
{
	GENERATED_BODY()
public:
	static constexpr int CAP = 256;

	TRingBufferSPSC<FImuMsg,  CAP>   ImuQ;
	TRingBufferSPSC<FGpsFixMsg, CAP> GpsQ;
	TRingBufferSPSC<FBaroMsg,  CAP>  BaroQ;
	TRingBufferSPSC<FMagMsg,   CAP>  MagQ;

	static USensorBus* Get(UWorld* World)
	{
		// simple world-owned singleton
		static TWeakObjectPtr<USensorBus> Inst;
		if (!Inst.IsValid())
		{
			Inst = NewObject<USensorBus>(World, USensorBus::StaticClass());
			Inst->AddToRoot();
		}
		return Inst.Get();
	}
};
