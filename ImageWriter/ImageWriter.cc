#include "ImageWriter.hh"

ImageWriter::ImageWriter(const std::string& name) :
        urbi::UObject (name)
{
	UBindFunction(ImageWriter, init);
}

void ImageWriter::init()
{
	// binding
	UBindVar(ImageWriter, inputImage);
	UBindVar(ImageWriter, fileName);
	UBindVar(ImageWriter, folderName);
	UBindVar(ImageWriter, imageFormat);
	UBindVar(ImageWriter, currentId);
	UBindFunction(ImageWriter, writeSingleImage);
	
	// on change
	UNotifyChange(inputImage, &ImageWriter::onGetImage);

	// settings
	folderName  = std::string("");
	fileName    = std::string("image");
	imageFormat = std::string("png");
	currentId   = 0;
}

void ImageWriter::writeImage(urbi::UVar& source)
{
	std::ostringstream tmpName;
	tmpName << (std::string) folderName << "/" << (std::string) fileName
		<< (int) currentId << "." << (std::string) imageFormat;
	//std::cout << tmpName.str() << std::endl;
	currentId = (int) currentId + 1;
	writeSingleImage(source, tmpName.str());
}

void ImageWriter::writeSingleImage(urbi::UVar& source, std::string filename)
{
	urbi::UBinary bin = source;
	bin.allocated_ = true;
	cv::Mat image_tmp(bin.image.height, bin.image.width, CV_8UC3, bin.image.data);
	cv::imwrite(filename, image_tmp);
}

void ImageWriter::onGetImage(urbi::UVar& source)
{
	writeImage(source);
}

UStart(ImageWriter);
