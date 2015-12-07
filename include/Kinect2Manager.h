#ifndef KINECT2MANAGER_H
#define KINECT2MANAGER_H
/**
Kinect2Manager
Kinect2Manager.h

@author Fabio Dominio
@version 0.1
@date 12/07/2015
*/

// Include required headers
#include <string>
#include <Kinect.h>

/**
Represents a Kinect2 sensor
*/
class Kinect2Manager {
public:
	/**
	Empty constructor
	*/
	Kinect2Manager(void);

	/**
	Destructor
	*/
	~Kinect2Manager();

public:
	/**
	Connects to the default Kinect2 sensor
	@return TRUE in case of success, FALSE otherwise
	*/
	bool connect();

	/**
	Disconnects from the default Kinect2 sensor
	@return TRUE in case of success, FALSE otherwise
	*/
	bool disconnect();

	/**
	Starts data acquisition from the default Kinect2 sensor
	@return TRUE in case of success, FALSE otherwise
	*/
	bool startAcquisition();

	/**
	Stops data acquisition from the default Kinect2 sensor
	@return TRUE in case of success, FALSE otherwise
	*/
	bool stopAcquisition();

	/**
	Grabs a single frame from the default Kinect2 sensor
	@param pointer to the depth data buffer
	@return TRUE in case of success, FALSE otherwise
	*/
	bool grabSingleFrame(unsigned short* depthData, unsigned char* colorData);

	/**
	Checks default Kinect2 sensor availability
	@return TRUE if the sensor is available, FALSE otherwise

	ATTENTION: check the real isAvailable method description from Kinect2 SDK
	*/
	bool isAvailable();

	/**
	Checks default Kinect2 sensor acquisition status
	@return TRUE if the sensor is acquiring, FALSE otherwise
	*/
	bool isAcquiring();

	/**
	Returns a string describing the most recent Kinect2 error
	@return a string describing the most recent error
	*/
	std::string getLastError();

	/**
	Returns a string containing the default Kinect2 ID
	@return a string containing the default Kinect2 ID
	*/
	std::string getID();

	/**
	Return the Kinect2 depth camera intrinsics
	@return the depth camera intrinsics
	*/
	CameraIntrinsics getDepthIntrinsics();

	/**
	Enable or disables the depth data stream
	@param flag indicating the depth stream abilitation status
	*/
	void acquireDepthData(bool flag);

	/**
	Enable or disables the color data stream
	@param flag indicating the color stream abilitation status
	*/
	void acquireColorData(bool flag);

	/**
	Enable or disables the infrared data stream
	@param flag indicating the infrared stream abilitation status
	*/
	void acquireInfraredData(bool flag);

	/**
	Enable or disables the long exposure infrared data stream
	@param flag indicating the long exposure infrared stream abilitation status
	*/
	void acquireLongExposureInfraredData(bool flag);

private:
	// Define Kinect2 required interfaces and variables
	IKinectSensor* kinect;
	IDepthFrameSource* depthFrameSource;
	IDepthFrameReader* depthFrameReader;
	IMultiSourceFrameReader* multisourceFrameReader;
	ICoordinateMapper* coordinateMapper;
	CameraIntrinsics cameraIntrinsics;

	HANDLE hEvents;
	WAITABLE_HANDLE hFrameWaitable;

	bool acquireColor = true;
	bool acquireDepth = true;
	bool acquireInfrared = false;
	bool acquireLongExposureInfrared = false;

	// Define other required variables and data structures
	std::string lastErrorMessage;
};
#endif
