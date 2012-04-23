#include <cmath>

#include "CDDetector.hh"

#define PI 3.14159265358

/*!
 * \brief Compute the angle between (center,pt_1)
 *        and (center,pt_2) lines.
 */
static float angle( cv::Point pt_1, cv::Point pt_2, cv::Point center )
{
	std::complex< float > cpx1( pt_1.x, pt_1.y );
	std::complex< float > cpx2( pt_2.x, pt_2.y );
	std::complex< float > cpx_( center.x, center.y );

	return arg( ( cpx1 - cpx_ ) / ( cpx2 - cpx_ ) );
}

/*!
 * \brief Compute the angle between (pt_1_,pt_1)
 *        and (pt_2_,pt_2) lines.
 */
static float angle2( cv::Point pt_1, cv::Point pt_2,
		cv::Point pt_1_, cv::Point pt_2_ )
{
	std::complex< float > cpx1( pt_1.x, pt_1.y );
	std::complex< float > cpx2( pt_2.x, pt_2.y );
	std::complex< float > cpx1_( pt_1_.x, pt_1_.y );
	std::complex< float > cpx2_( pt_2_.x, pt_2_.y );

	return arg( ( cpx1 - cpx1_ ) / ( cpx2 - cpx2_ ) );
}

static std::vector< cv::Point > extract_vector(
		std::vector< cv::Point >& ptVec,
		int minRange, int maxRange )
{
	std::vector< cv::Point > res;

	for ( int i = minRange; i < maxRange; ++i)
	{
		res.push_back( ptVec.at(i) );
	}

	return res;
}

CDDetector::CDDetector(const std::string& name) :
		urbi::UObject (name)
{
	UBindFunction(CDDetector, init);
}

void CDDetector::init()
{
	// binding
	UBindVar(CDDetector, inputImage);
	UBindVar(CDDetector, outputImage);

	// on change
	UNotifyChange(inputImage, &CDDetector::onNewImage);

	// initialize vars
	// nothing to do
}

void CDDetector::onNewImage(urbi::UVar& source_)
{
	urbi::UBinary source = source_;

	cv::Mat src(
			source.image.height,
			source.image.width,
			CV_8UC3,
			source.image.data);

	cv::Mat dst(
			source.image.height,
			source.image.width,
			CV_8UC1
			);

	firstFilter(src, dst);

	lineSegApprox();

	curveSegmentation();

#ifdef ON_TEST /*{*/
#define LINE_COLOR (uchar) 255 ///< print contour in white
#ifdef SHOW_APPROX_CONTOURS /*{*/
	// drawing approx_contours on dst
	std::vector< cv::Point > approx_contour;
	std::vector< std::vector< cv::Point > >::iterator it
		= approx_contours.begin();
	std::vector< cv::Point >::iterator it1;
	std::vector< cv::Point >::iterator it2;


	for ( ; it != approx_contours.end(); ++it)
	{
		approx_contour = *it;
		it1 = approx_contour.begin();
		it2 = ++approx_contour.begin();

		for ( ; it1 != approx_contour.end()
				&& it2 != approx_contour.end();
				++it1, ++it2)
		{
			cv::line(dst, *it1, *it2, LINE_COLOR);
		}
	}
	/*}*/
#else /*{*/
#ifdef SHOW_SEGM_CONTOURS
	// drawing segm_contours on dst
	std::vector< cv::Point > segm_contour;
	std::vector< std::vector< cv::Point > >::iterator it
		= segm_contours.begin();
	std::vector< cv::Point >::iterator it1;
	std::vector< cv::Point >::iterator it2;


	for ( ; it != segm_contours.end(); ++it)
	{
		segm_contour = *it;
		it1 = segm_contour.begin();
		it2 = ++segm_contour.begin();

		for ( ; it1 != segm_contour.end()
				&& it2 != segm_contour.end();
				++it1, ++it2)
		{
			cv::line(dst, *it1, *it2, LINE_COLOR);
		}
	}
#endif // SHOW_SEGM_CONTOURS
#endif // SHOW_APPROX_CONTOURS /*}*/
#endif // ON_TEST /*}*/

	urbi::UBinary bin;
	bin.type = urbi::BINARY_IMAGE;
	bin.image.height = dst.rows;
	bin.image.width  = dst.cols;
	bin.image.size   = dst.rows * dst.cols;
	bin.image.imageFormat = urbi::IMAGE_GREY8;
	bin.image.data   = dst.data;
	bin.allocated_   = false;

	outputImage = bin;
}

void CDDetector::firstFilter(const cv::Mat& src, cv::Mat& dst)
{
	// convert in grayscale
	cv::cvtColor( src, dst, CV_BGR2GRAY );

#define THRESHOLD1 90
#define THRESHOLD2 180
#define APERTURE_SIZE 3

	// detect edges
	cv::Canny( dst, dst, THRESHOLD1, THRESHOLD2, APERTURE_SIZE );

	// find contours
	cv::findContours( dst, contours,
			CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0) );
}

void CDDetector::lineSegApprox()
{
	std::vector< cv::Point > approx_contour;
	approx_contours.clear();

	std::vector< std::vector< cv::Point > >::iterator it
		= contours.begin();

#define CLOSED_CONTOUR true ///< Contours are closed
#define MIN_POINT_NUMBER 4  ///< Minimum contour point number admitted
#define MIN_CONTOUR_SIZE 60 ///< Minimum contour size admitted (pixel)

	for ( ; it != contours.end(); ++it )
	{
		approx_contour.clear();
		cv::approxPolyDP( *it, approx_contour, 1, CLOSED_CONTOUR );
		if ( approx_contour.size() < MIN_POINT_NUMBER
				|| cv::arcLength( approx_contour, CLOSED_CONTOUR ) )
		approx_contours.push_back( approx_contour );
	}
}

void CDDetector::curveSegmentation()
{
	std::vector< std::vector< cv::Point > >::iterator it
		= approx_contours.begin();
	segm_contours.clear();
	std::vector< cv::Point > tmp;

#define POINT_NUMBER_STEP 1
#define MIN_STEP_NUMBER 5
#define MAX_ANGLE (PI / 6)
#define LENGTH_THRESHOLD 16

	for ( ; it != approx_contours.end(); ++it )
	{
		if ( it->size() < MIN_STEP_NUMBER * POINT_NUMBER_STEP )
		{
			continue;
		}

		tmp.clear();

		for ( int i = 2 ; i + 2 < it->size(); ++i )
		{
			double norm_cond =
				norm( it->at(i) - it->at(i - 1) ) / norm ( it->at(i + 1) - it->at(i) );

			// alpha angles
			float alpha1 = angle(
					it->at(i - 1), it->at(i), it->at(i - 2) );
			float alpha2 = angle(
					it->at(i - 1), it->at(i + 1), it->at(i - 2) );
			float alpha3 = angle(
					it->at(i + 1), it->at(i), it->at(i + 2) );
			float alpha4 = angle(
					it->at(i + 1), it->at(i - 1), it->at(i + 2) );

			// beta angles
			float beta1 = angle(
					it->at(i - 2), it->at(i), it->at(i - 1) );
			float beta2 = angle(
					it->at(i + 1), it->at(i - 1), it->at(i) );
			float beta3 = angle(
					it->at(i), it->at(i + 2), it->at(i + 1) );

			if (       ( alpha1 * alpha2 < 0 )
					|| ( alpha3 * alpha4 < 0 )
					|| ( abs( alpha1 ) > abs( alpha2 ) )
					|| ( abs( alpha3 ) > abs( alpha4 ) )
					|| ( abs( beta1 ) - abs( beta2 ) > MAX_ANGLE )
					|| ( abs( beta3 ) - abs( beta2 ) > MAX_ANGLE )
					|| ( norm_cond > LENGTH_THRESHOLD )
					|| ( norm_cond < 1.0 / LENGTH_THRESHOLD ) )
			{
					// if the number of point explored >= min
					// number of point addmitted to be an arc
					if ( i + 2 >= 2 * POINT_NUMBER_STEP )
					{
						segm_contours.push_back( tmp );
					}

					// continue on the same contour
					tmp.clear();
					continue;
			}

			// add the cuurent point the the tmp
			tmp.push_back( it->at(i) );
		}
	}
}

void CDDetector::neighborhoodCurveGroup()
{
	std::vector< std::vector< cv::Point > >::iterator it1
		= segm_contours.begin();
	std::vector< std::vector< cv::Point > >::iterator it2
		= segm_contours.begin();

#define CONT_MAX_DIST 60

	for ( ; it1 != segm_contours.end(); ++it1 )
	{
		cv::Point m_cont1[2];
		cv::Point m_cont2[2];
		int m_extr1, m_extr2, m_int1, m_int2;
		double m_mindist;
		float m_angle = PI;
		bool match_found = false;

		for ( ; it2 != segm_contours.end(); ++it2 )
		{
			if ( it1 == it2 ) continue;

			cv::Point cont1[2] = { it1->front(), it1->back() };
			cv::Point cont2[2] = { it2->front(), it2->back() };

			int extr1, extr2, int1, int2;

			double dist, mindist = CONT_MAX_DIST;
			float angle_;

			for ( int i = 0; i < 2; ++i )
				for ( int j = 0; j < 2; ++j )
				{
					dist = norm(cont1[i] - cont2[j]);
					if ( dist < mindist )
					{
						mindist = dist;
						extr1   = i;
						int1    = 1 - i;
						extr2   = j;
						int2    = 1 - j;
					}
				}

			if ( mindist < CONT_MAX_DIST )
			{
				// get the points after the two extremities
				cont1[int1] = ( extr1 == 0 ) ?
					it1->at(1) : it1->at(it1->size() - 2);
				cont2[int2] = ( extr2 == 0 ) ?
					it2->at(1) : it2->at(it2->size() - 2);

				// process the angle between the two contours
				angle_ = angle2( cont1[extr1], cont2[int2],
						cont1[int1], cont2[extr2] );

				if ( abs(angle_) < abs(m_angle) )
				{
					match_found = true;
					m_angle     = angle_;
					m_mindist   = mindist;
					m_cont1[0]  = cont1[0];
					m_cont1[1]  = cont1[1];
					m_cont2[0]  = cont2[0];
					m_cont2[1]  = cont2[1];
					m_extr1     = extr1;
					m_extr2     = extr2;
					m_int1      = int1;
					m_int2      = int2;
					//TODO
				}
			}
		}
	}
}

UStart(CDDetector);
