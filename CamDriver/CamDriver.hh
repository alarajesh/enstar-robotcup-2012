#ifndef CAMDRIVER_HH
# define CAMDRIVER_HH

#include <urbi/uobject.hh>
#include <opencv2/opencv.hpp>

class CamDriver : public urbi::UObject
{
public:
	CamDriver(const std::string& name);

	int update();
	void init(int device=0);

private:
	urbi::UVar outputImage;
	cv::VideoCapture* m_cap;
};

#endif //CAMDRIVER_HH
