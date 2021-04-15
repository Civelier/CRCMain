#ifndef _REQUEST_h
#define _REQUEST_h

#include "Arduino.h"


typedef uint8_t* Packet;

struct Request
{
	uint8_t Length = sizeof(Request);
	uint8_t DeviceID;
};

struct ClearRequest
{
	uint8_t Length = sizeof(ClearRequest);
	uint8_t DeviceID;
	uint8_t Clear = 0;
};

enum DistanceComparison
{
	DCClear = 0,
	DCEqualTreshold = 1,
	DCOutsideTreshold = 2,
	DCMoreThan = 3,
	DCMoreThanOrEqualTo = 4,
	DCLessThan = 5,
	DCLessThanOrEqualTo = 6
};

struct DistanceRequest
{
	DistanceRequest(uint8_t deviceID, uint32_t mm, DistanceComparison comparisson)
	{
		DeviceID = deviceID;
		MM = mm;
		Comparisson = comparisson;
	}
	uint8_t Length = sizeof(DistanceRequest);
	uint8_t DeviceID;
	DistanceComparison Comparisson;
	uint32_t MM;
};

struct DistanceTresholdRequest
{
	DistanceTresholdRequest(uint8_t deviceID, uint32_t mm, uint32_t treshold, DistanceComparison comparison)
	{
		DeviceID = deviceID;
		MM = mm;
		Treshold = treshold;
		Comparisson = comparison;
	}
	uint8_t Length = sizeof(DistanceTresholdRequest);
	uint8_t DeviceID;
	DistanceComparison Comparisson;
	uint32_t MM;
	uint32_t Treshold;
};

#define MaxMessageLength 20

//namespace Local
//{
//	union RequestBytes
//	{
//		uint8_t* Buff;
//		Request Req;
//		DistanceRequest DistanceReq;
//		DistanceTresholdRequest DistanceTreshReq;
//		ClearRequest ClearReq;
//	};
//}

#endif