#ifndef IMAGEREADER_HH
# define IMAGEREADER_HH

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <aruco/aruco.h>
#include <urbi/uobject.hh>

class ARTagReader : public urbi::UObject
{
public:
	ARTagReader(const std::string& name);

	void        init(std::string camParamFile, ufloat markSize);
	int         update();

private:
	urbi::UVar outputImage;     // image    // output
	urbi::UVar markerPositions; // UList    // output
	urbi::UVar inputImage;      // image    // input
	urbi::UVar markerSize;      // float    // input
	urbi::UVar showOutput;      // bool     // input
	aruco::CameraParameters camParam;
	aruco::MarkerDetector markerDetector;
	std::vector<aruco::Marker> markers;
};

#endif //IMAGEREADER_HH
