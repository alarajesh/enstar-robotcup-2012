#ifndef IMAGEREADER_HH
# define IMAGEREADER_HH

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <aruco/aruco.h>
#include <urbi/uobject.hh>

class ARTagPosition : public urbi::UObject
{
public:
	ARTagPosition(const std::string& name);

	void        init();
	int         update();

private:
	urbi::UVar markerPosition; // UList   // input
	urbi::UVar markerGroup;    // UList   // output
	urbi::UVar groupPosition;  // UList   // output
};

#endif //IMAGEREADER_HH
