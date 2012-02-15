#ifndef IMAGEVIEWER_HH
# define IMAGEVIEWER_HH

#include <urbi/uobject.hh>
#include <opencv2/opencv.hpp>

class ImageViewer : public urbi::UObject
{
public:
	ImageViewer(const std::string& name);

	void init(std::string name);
	void showImage(urbi::UVar& source);
	void show(urbi::UVar& source, int time_ms);
	void onGetImage(urbi::UVar& source);
	static void onMouse(int event, int x, int y,
						int flags, void* param);
	static int openCVImageType(urbi::UImageFormat format);

private:
	urbi::UVar inputImage;
	urbi::UVar frameRate;
	urbi::UVar selectedColor;
	std::string m_name;
};

#endif //IMAGEVIEWER_HH
