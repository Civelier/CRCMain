#ifndef _IREMOTEDEVICE_h
#define _IREMOTEDEVICE_h

#include "Transmitter.h"
#include "Request.h"

namespace Remote
{
class IRemoteDevice
{
protected:
	template <typename T>
	void SendRequest(T* request)
	{
		uint8_t* buff = reinterpret_cast<uint8_t*>(request);
		Transmit.SendRequest(buff);
		delete(request);
	}
public:
	uint8_t ID;
	virtual void ClearRequests()
	{
		ClearRequest* r = new ClearRequest();
		r->DeviceID = ID;
		r->Length = 3;
		SendRequest(r);
	}
};
}

#endif