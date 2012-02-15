#include "CamDriver.hh"

CamDriver::CamDriver(const std::string& name) :
	urbi::UObject (name)
{
	UBindFunction(CamDriver, init);
}

void CamDriver::init(int device)
{
	UBindVar(CamDriver, outputImage);

	m_cap = new cv::VideoCapture(device);

	if(!m_cap->isOpened())
	{
		std::cout << "file : " << __FILE__ << " at line " << __LINE__ 
			<< " error : capture device not loaded" << std::endl;
		return;
	}

	USetUpdate(0.04);
}

int CamDriver::update()
{
	if(m_cap->isOpened()) {
		cv::Mat frame;
		*m_cap >> frame;

		urbi::UBinary bin;
		bin.type = urbi::BINARY_IMAGE;
		bin.image.height = frame.rows;
		bin.image.width = frame.cols;
		bin.image.size = frame.rows*frame.cols*3;
		bin.image.imageFormat = urbi::IMAGE_RGB;
		bin.image.data = frame.data;
		bin.allocated_ = false;

		outputImage = bin;
	} else {
		throw std::string("Cannot open the camera device !");
	}
}

UStart(CamDriver);
