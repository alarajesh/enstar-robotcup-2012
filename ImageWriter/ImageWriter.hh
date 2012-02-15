#ifndef IMAGEVIEWER_HH
# define IMAGEVIEWER_HH

#include <urbi/uobject.hh>
#include <opencv2/opencv.hpp>
#include <iostream>

class ImageWriter : public urbi::UObject
{
public:
	ImageWriter(const std::string& name);

	void init();
	void writeImage(urbi::UVar& source);
	void writeSingleImage(urbi::UVar& source, std::string filename);
	void onGetImage(urbi::UVar& source);

private:
	urbi::UVar inputImage;
	urbi::UVar folderName;
	urbi::UVar fileName;
	urbi::UVar currentId;
	urbi::UVar imageFormat;
};


#endif //IMAGEVIEWER_HH
