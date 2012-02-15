#include "ImageReader.hh"
#include <fstream>

ImageReader::ImageReader(const std::string& name) : 
		urbi::UObject (name)
	{
	UBindFunction(ImageReader, init);
}

void ImageReader::init()
{
	// binding
	UBindVar(ImageReader, outputImage);
	UBindVar(ImageReader, fileName);
	UBindVar(ImageReader, folderName);
	UBindVar(ImageReader, currentId);
	UBindVar(ImageReader, imageFormat);
	
	// settings
	fileName    = std::string("image");
	folderName  = std::string("");
	currentId   = 0;
	imageFormat = std::string("png");

	USetUpdate(0.05);
}

int ImageReader::update()
{
	// test if image exists
	std::ifstream imageFile(getFileName().c_str());
	//std::cout << imageFile.fail() << std::endl;
	if (imageFile.fail()) return 1;

	// process
	cv::Mat frame = cv::imread(getFileName());
	currentId = (int) currentId + 1;
	urbi::UBinary bin;
	bin.type = urbi::BINARY_IMAGE;
	bin.image.height = frame.rows;
	bin.image.width = frame.cols;
	bin.image.size = frame.rows*frame.cols*3;
	bin.image.imageFormat = urbi::IMAGE_RGB;
	bin.image.data = frame.data;
	bin.allocated_ = false;

    outputImage = bin;
	return 0;
}

std::string ImageReader::getFileName()
{
	std::ostringstream tmpName;
	tmpName << (std::string) folderName << (std::string) fileName << 
		(int) currentId << "." << (std::string) imageFormat;
	return tmpName.str();
}

UStart(ImageReader);
