#include <cmath>

#include "CDDetector.hh"

#define PI 3.14159265358

/*!
 * \brief Node used for contour grouping
 *
 * This is used as a component of a std::vector to get the relation between contour
 * sequences before grouping.
 */
typedef struct node
{
	std::vector< cv::Point >::iterator next_begin; ///< index of the first point to visit in the next part
	std::vector< cv::Point >::iterator next_end;   ///< index of the last point to visit in the next part
	size_type previous_position;                   ///< index of the previous contour part
	size_type next_position;                       ///< index of the previous contour part
} node;

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

#define THRESHOLD1 90    ///< cv::Canny parameter
#define THRESHOLD2 180   ///< cv::Canny parameter
#define APERTURE_SIZE 3  ///< cv::Canny parameter

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
#define CONT_MAX_DIST 60

	for ( int i = 0 ; i < segm_contours.size(); ++i )
	{
		node& current_node = nodes.at(i);
		std::vector< cv::Point >& contour_i = segm_contours.at(i);

		// each pair is seen just one time
		for (int j = i + 1; j < segm_contours.size(); ++j )
		{
			std::vector< cv::Point >& contour_j = segm_contours.at(j);

			cv::Point cont1[2] = { contour_i.front(), contour_i.back() };
			cv::Point cont2[2] = { contour_j.front(), contour_j.back() };

			int extr1, extr2, int1, int2;

			double dist, mindist = CONT_MAX_DIST;
			float angle_;

			for ( int k = 0; k < 2; ++i )
				for ( int l = 0; l < 2; ++j )
				{
					dist = norm(cont1[k] - cont2[k]);
					if ( dist < mindist )
					{
						mindist = dist;
						extr1   = k;
						int1    = 1 - k;
						extr2   = l;
						int2    = 1 - k;
					}
				}

			// if contours are enough near
			if ( mindist < CONT_MAX_DIST )
			{
				// get the points after the two extremities
				cont1[int1] = ( extr1 == 0 ) ?
					contour_i.at(1) :
					contour_i.at(contour_i.size() - 2);
				cont2[int2] = ( extr2 == 0 ) ?
					contour_j.at(1) :
					contour_j.at(contour_j.size() - 2);

				// process the angle between the two contours
				angle_ = angle2( cont1[extr1], cont2[int2],
						cont1[int1], cont2[extr2] );

				// if the contour i have the better continuity with the j one on the extr1 side
				if ( abs(angle_) < abs(current_node.angle[extr1]) )
				{
					// keep the results on the corresponding side
					current_node.angle[extr1]      = angle_;
					current node.dist[extr1]       = mindist;
					current_node.link_index[extr1] = j;
					current_node.link_side[extr1]  = extr2;
				}
			}
		}

		for ( side = 0; side < 2; ++side )
		{
			if ( current_node.assigned( side ) )
			{
				node& linked_node = current_node.linkedNode( side );
				int linked_node_related_side = current_node.link_side[ side ];

				// if the current_node is assigned at the left and that the left linked node
				// is already assigned at the corresponding side => conflict
				if ( linked_node.assigned( linked_node_related_side ) )
				{
					if ( current_node.angle[ side ] < linked_node.angle[ linked_node_related_side ] )
					{
						// TODO : change linked node related side arguments and linked to the linked...
					}
					else
					{
						current_node.unassign( side );
					}
				}
			}
		}
	}
}

UStart(CDDetector);
