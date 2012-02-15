#ifndef IMAGEREADER_HH
# define IMAGEREADER_HH

#include <urbi/uobject.hh>
#include <opencv2/opencv.hpp>
#include <iostream>

class ImageReader : public urbi::UObject
{
public:
	ImageReader(const std::string& name);

	void        init();
	int         update();
	
	inline
	std::string getFileName();

private:
	urbi::UVar  outputImage;
	urbi::UVar  fileName;
	urbi::UVar  folderName;
	urbi::UVar  currentId;
	urbi::UVar  imageFormat;
};

#endif //IMAGEREADER_HH

