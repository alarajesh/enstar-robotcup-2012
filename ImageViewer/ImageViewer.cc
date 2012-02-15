#include "ImageViewer.hh"

namespace cv {
	typedef Vec<uchar, 1> Vec1b;
};

ImageViewer::ImageViewer(const std::string& name) /*{{{*/ :
	urbi::UObject (name)
{
	UBindFunction(ImageViewer, init);
} /*}}}*/

void ImageViewer::init(std::string name) /*{{{*/
{
	// binding
	UBindVar(ImageViewer, inputImage);
	UBindVar(ImageViewer, frameRate);
	UBindVar(ImageViewer, selectedColor);
	UBindFunction(ImageViewer, showImage);
	
	// when inputImage change => onGetImage
	UNotifyChange(inputImage, &ImageViewer::onGetImage);

	// create a window for displaying
	cv::namedWindow(name);
	cv::setMouseCallback(name.c_str(), &onMouse, this);

	// settings
	frameRate = 20;
	m_name = name;
	urbi::UBinary color = selectedColor;
	color.common.data = NULL;
} /*}}}*/

void ImageViewer::show(urbi::UVar& source, int time_ms) /*{{{*/
{
	urbi::UBinary bin = source;
	bin.allocated_ = true;

	cv::Mat image_tmp(bin.image.height,
			bin.image.width,
			openCVImageType(bin.image.imageFormat),
			bin.image.data);
	cv::imshow(m_name,image_tmp);
	cv::waitKey(time_ms);
} /*}}}*/

void ImageViewer::showImage(urbi::UVar& source) /*{{{*/
{
	show(source, 0);
} /*}}}*/

void ImageViewer::onGetImage(urbi::UVar& source) /*{{{*/
{
	show(source, frameRate);
} /*}}}*/

void ImageViewer::onMouse(int event, int x, int y, int flags, void* imView) /*{{{*/
{
	if(event == CV_EVENT_LBUTTONUP)
	{
		urbi::UBinary bin   = ((ImageViewer*) imView)->selectedColor;
		urbi::UBinary input = ((ImageViewer*) imView)->inputImage;
		int frametype = openCVImageType(input.image.imageFormat);
		cv::Mat inframe(input.image.height,
						input.image.width,
						frametype,
						input.image.data);
		void* color;
		if (frametype == CV_8UC3)
		{
			color = new cv::Vec3b(inframe.at<cv::Vec3b>(x,y));
			std::cout << (int)(*((cv::Vec3b*) color))[1] << ","
				<< (int)(*((cv::Vec3b*) color))[2] << ","
				<< (int)(*((cv::Vec3b*) color))[3] << std::endl;
		}
		if (frametype == CV_8UC1) 
		{
			color = new cv::Vec1b(inframe.at<cv::Vec1b>(x,y));
			std::cout << (int)(*((cv::Vec1b*) color))[1] << std::endl;
		}
		delete bin.common.data;
		bin.common.data = color;
	}
} /*}}}*/

int ImageViewer::openCVImageType(urbi::UImageFormat format) /*{{{*/
{
	int frametype;
	switch (format)
	{
		case (urbi::IMAGE_RGB) :
			frametype = CV_8UC3;
			break;
		case (urbi::IMAGE_GREY8) :
			frametype = CV_8UC1;
			break;
	}
	return frametype;
}

UStart(ImageViewer);
