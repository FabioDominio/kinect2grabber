# Kinect2 Grabber
This simple command-line software connect to a Microsoft Kinect v2 device to acquire, visualize and optionally save multiple source frames.

Visualization is performed by mean of OpenCV.

Currently the software is set to both acquire depth and color data, although the code is easily editable to support the acquisition of the infrared image as well or to disable one or more streams.

## Build instructions
Kinect2 Grabber is almost self-contained and just requires:

* CMake for automatic project generation for Visual Studio, g++ and other environments;
* OpenCV library v3.0 as external dependence (unofficial builds, ready for CMake, for Windows and Linux can be found at http://www.fabiodominio.com/tools.html)
* Kinect v2 SDK (https://www.microsoft.com/en-us/download/details.aspx?id=44561)

Note how in CMake openCV is generally detected when specifying the path of "OpenCVConfig.cmake" file, often provided with any openCV build. Note also how, unless you are linking against a static version of openCV, the program executable also needs the "core", "highgui", "imgcodecs", "imgproc", "videoio" dynamic libraries.

## Usage
Due to Microsoft Kinect2 SDK requirement, this software only runs on Windows.

Note how for running the software you don't need the Kinect v2 SDK but only the Kinect v2 Runtime driver (https://www.microsoft.com/en-us/download/details.aspx?id=44559) to be installed on the system.

To run the software, simply execute "Kinect2Grabber.exe". To exit, push "Esc" key when the openCV windows is on the foreground, to grab a frame (it will be stored in the same executable directory unless differently specified in the code) push "g" or "G" key. After grabbing a frame, the software stores the full hd color image and the grayscale depth image in .png format, and the depth map in a text file in .dat format. Note how the saved frames are automatically numbered, so no worries about overriding previous grabs. 
