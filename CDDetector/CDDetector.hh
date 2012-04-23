#ifndef CDDETECTOR_HH
#define CDDETECTOR_HH

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <urbi/uobject.hh>

class CDDetector : public urbi::UObject
{
	public:
		CDDetector(const std::string& name);

		void init();
		void onNewImage(urbi::UVar& source);

		/*!
		 * \brief This function find the contours in the input image
		 */
		void firstFilter(
				const cv::Mat& src,
				cv::Mat& dst);

		/*!
		 * brief This function approximate contours with polygon
		 *       and remove short contours
		 */
		void lineSegApprox();

		void removeShortContours();

		void curveSegmentation();

		void neighborhoodCurveGroup();

	private:
		urbi::UVar inputImage;
		urbi::UVar outputImage;
		std::vector< std::vector< cv::Point > > contours;
		std::vector< std::vector< cv::Point > > approx_contours;
		std::vector< std::vector< cv::Point > > segm_contours;
};

#endif // CDDETECTOR_HH
