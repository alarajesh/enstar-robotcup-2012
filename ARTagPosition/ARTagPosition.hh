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
	urbi::UVar markerPosition; // UList
	urbi::UVar markerGroup;    // UList
	urbi::UVar groupPosition;  // UList
};

#endif //IMAGEREADER_HH
