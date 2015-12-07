#include "Kinect2Manager.h"

using namespace std;

// Safe release for interfaces
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

// Kinect2Manager constructor
Kinect2Manager::Kinect2Manager()
{

}

// Kinect2Manager destructor
Kinect2Manager::~Kinect2Manager()
{	
	SafeRelease(multisourceFrameReader);
	disconnect();
}

// getLastError
string Kinect2Manager::getLastError() {
	return lastErrorMessage;
}

// isAvailable
bool Kinect2Manager::isAvailable() {
	// Try to query sensor availability
	// TODO: discriminate between sensor unavailability and query error
	HRESULT res = S_OK;
	BOOLEAN available;
	do {
		res = kinect->get_IsAvailable(&available);
	} while (res != S_OK);
	return available;
}

// isAcquiring
bool Kinect2Manager::isAcquiring() {
	// Try to query sensor acquisition status
	// TODO: discriminate between sensor acquisition status and query error
	HRESULT res;
	BOOLEAN open;
	res = kinect->get_IsOpen(&open);
	if (res != S_OK) {
		// Query error
		lastErrorMessage = "Unable to query kinect2 sensor acquisitions status!";
		return false;
	}
	else
		return open;
}

// getID
string Kinect2Manager::getID() {
	// Try to query sensor unique ID
	HRESULT res;
	WCHAR id[50];
	res = kinect->get_UniqueKinectId(50, id);
	if (res != S_OK) {
		// Query error
		lastErrorMessage = "Unable to query kinect2 sensor ID!";
		return "";
	}
	else {
		wstring wid(id);
		string sid;
		sid.assign(wid.begin(), wid.end());
		return sid;
	}
}

// connect
bool Kinect2Manager::connect() {
	// Try to connect to the first plugged kinect
	HRESULT res;
	// Get default kinect
	res = GetDefaultKinectSensor(&kinect);
	if (res != S_OK) {
		// Connection error
		lastErrorMessage = "Unable to get default kinect2 sensor!";
		return false;
	}
	// Else default kinect has been found; open it
	res = kinect->Open();
	if (res != S_OK) {
		lastErrorMessage = "Unable to start acquisition!";
		return false;
	}

	// Get depth camera intrinsic parameters matrix
	res = kinect->get_CoordinateMapper(&coordinateMapper);
	if (res != S_OK) {
		lastErrorMessage = "Unable to get coordinate mapper!";
		return false;
	}

	do {
		res = coordinateMapper->GetDepthCameraIntrinsics(&cameraIntrinsics);
		if (res != S_OK) {
			lastErrorMessage = "Unable to get depth camera intrinsics!";
			return false;
		}
	} while (cameraIntrinsics.FocalLengthX == 0);

	return true;
}

// disconnect
bool Kinect2Manager::disconnect() {	
	HRESULT res;
	if (kinect) {
		res = kinect->Close();
		if (res != S_OK) {
			lastErrorMessage = "Error disconnecting from Kinect2!";			
		}
	}

	SafeRelease(kinect);

	return res == S_OK;	
}


// startAcquisition
bool Kinect2Manager::startAcquisition() {
	HRESULT res;

	// Open streams
	hEvents = reinterpret_cast<HANDLE>(hFrameWaitable);

	int frameSourceTypes = FrameSourceTypes_None;
	if (acquireColor)
		frameSourceTypes |= FrameSourceTypes_Color;
	if (acquireDepth)
		frameSourceTypes |= FrameSourceTypes_Depth;
	if (acquireInfrared)
		frameSourceTypes |= FrameSourceTypes_Infrared;
	if (acquireLongExposureInfrared)
		frameSourceTypes |= FrameSourceTypes_LongExposureInfrared;

	res = kinect->OpenMultiSourceFrameReader(frameSourceTypes, &multisourceFrameReader);
	if (res != S_OK) {
		lastErrorMessage = "Unable to open input stream!";
		return false;
	}

	res = multisourceFrameReader->SubscribeMultiSourceFrameArrived(&hFrameWaitable);
	if (res != S_OK) {
		lastErrorMessage = "Unable to register waitable handle!";
		return false;
	}	

	return true;
}

// stopAcquisition
bool Kinect2Manager::stopAcquisition() {
	HRESULT res;
	res = kinect->Close();
	if (res != S_OK) {
		lastErrorMessage = "Unable to stop acquisition!";
		return false;
	}
	
	return true;
}

// grabSingleFrame
bool Kinect2Manager::grabSingleFrame(unsigned short* depthData, unsigned char* colorData) {
	IFrameDescription *depthFrameDescription;
	IFrameDescription *colorFrameDescription;
	IMultiSourceFrameReference *frameReference;
	IColorFrameReference* colorFrameReference;
	IColorFrame* colorFrame;
	IMultiSourceFrame* frame;
	IDepthFrame *depthFrame;
	IDepthFrameReference *depthFrameReference;
	//IDepthFrameArrivedEventArgs *args;
	IMultiSourceFrameArrivedEventArgs* frameArgs;
	HRESULT res = S_OK;
	lastErrorMessage = "";

	// Wait for frame ready
	WaitForSingleObject(hEvents, INFINITE);
	//WaitForMultipleObjects(1, &hEvents, true, INFINITE);
	res = multisourceFrameReader->GetMultiSourceFrameArrivedEventData(hFrameWaitable, &frameArgs);
	if (res == S_OK) {
		// Grab multi source frame reference
		res = frameArgs->get_FrameReference(&frameReference);
		if (res == S_OK) {
			// Grab multi source frame
			res = frameReference->AcquireFrame(&frame);
			if (res == S_OK) {
				if (acquireDepth) {
					//res = depthFrameReader->GetFrameArrivedEventData(hFrameWaitable, &args);
					//if (res == S_OK) {
					//args->get_FrameReference(&depthFrameReference);
					frame->get_DepthFrameReference(&depthFrameReference);
					if (res == S_OK) {
						res = depthFrameReference->AcquireFrame(&depthFrame);
						//res = depthFrameReader->AcquireLatestFrame(&depthFrame);
						if (res == S_OK) {
							// Control frame validity
							res = depthFrame->get_FrameDescription(&depthFrameDescription);
							if (res == S_OK) {
								int width, height;
								res = depthFrameDescription->get_Width(&width);
								if (res != S_OK)
									lastErrorMessage = "Unable to retrieve depth frame width!";
								res = depthFrameDescription->get_Height(&height);
								if (res != S_OK)
									lastErrorMessage = "Unable to retrieve depth frame height!";
								if (res == S_OK) {
									// Frame valid; copy depth to buffer
									res = depthFrame->CopyFrameDataToArray(512 * 424, depthData);
									if (res != S_OK)
										lastErrorMessage = "Unable to copy depth data to the buffer!";
								}
								else
									lastErrorMessage = "Received invalid depth frame!";

								SafeRelease(depthFrameDescription);
							}
							else
								lastErrorMessage = "Unable to retrieve depth frame description!";

							SafeRelease(depthFrame);
						}
						else
							lastErrorMessage = "Unable to get depth map from frame!";
					}
					else
						lastErrorMessage = "Unable to get depth frame reference!";

					SafeRelease(depthFrameReference);
					//}
					//else
					//lastErrorMessage = "Unable to manage frame arrived event!";
				} // End acquire depth

				if (acquireColor) {
					//res = depthFrameReader->GetFrameArrivedEventData(hFrameWaitable, &args);
					//if (res == S_OK) {
					//args->get_FrameReference(&depthFrameReference);
					frame->get_ColorFrameReference(&colorFrameReference);
					if (res == S_OK) {
						res = colorFrameReference->AcquireFrame(&colorFrame);
						//res = depthFrameReader->AcquireLatestFrame(&depthFrame);
						if (res == S_OK) {
							// Control frame validity
							res = colorFrame->get_FrameDescription(&colorFrameDescription);
							if (res == S_OK) {
								int width, height;
								res = colorFrameDescription->get_Width(&width);
								if (res != S_OK)
									lastErrorMessage = "Unable to retrieve color frame width!";
								res = colorFrameDescription->get_Height(&height);
								if (res != S_OK)
									lastErrorMessage = "Unable to retrieve color frame height!";
								if (res == S_OK) {
									// Frame valid; copy color data to buffer
									res = colorFrame->CopyConvertedFrameDataToArray(1920 * 1080*4, colorData, ColorImageFormat_Bgra);				
									if (res != S_OK)
										lastErrorMessage = "Unable to copy color data to the buffer!";
								}
								else
									lastErrorMessage = "Received invalid color frame!";

								SafeRelease(colorFrameDescription);
							}
							else
								lastErrorMessage = "Unable to retrieve color frame description!";

							SafeRelease(colorFrame);
						}
						else
							lastErrorMessage = "Unable to get color image from frame!";
					}
					else
						lastErrorMessage = "Unable to get color frame reference!";

					SafeRelease(colorFrameReference);
					//}
					//else
					//lastErrorMessage = "Unable to manage frame arrived event!";
				} // End acquire depth
			}
			else
				lastErrorMessage = "Error grabbing multi source frame!";		
		}
		else
			lastErrorMessage = "Error grabbing multi source frame reference!";
		SafeRelease(frameReference);			
	}
	else
		lastErrorMessage = "Error reading arrived frame event!";	

	SafeRelease(frameArgs);

	return res == S_OK;
}

// getDepthIntrinsics
CameraIntrinsics Kinect2Manager::getDepthIntrinsics() {
	return cameraIntrinsics;
};

// acquireDepthData
void Kinect2Manager::acquireDepthData(bool flag) {
	acquireDepth = flag;
}

// acquireColorData
void Kinect2Manager::acquireColorData(bool flag) {
	acquireColor = flag;
}

// acquireInfraredData
void Kinect2Manager::acquireInfraredData(bool flag) {
	acquireInfrared = flag;
}

// acquireLongExposureInfraredData
void Kinect2Manager::acquireLongExposureInfraredData(bool flag) {
	acquireLongExposureInfrared = flag;
}
