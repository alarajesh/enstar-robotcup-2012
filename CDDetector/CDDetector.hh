#ifndef CDDETECTOR_HH
#define CDDETECTOR_HH

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <urbi/uobject.hh>

#define PI 3.14159265358

/*!
 * \brief Node used for contour grouping
 *
 * This is used as a component of a std::vector to get the relation between contour
 * sequences before grouping.
 */
class Node
{
	public:
		Node( std::vector< Node >& nodes_ )
			: nodes( &nodes_ )
		{
			for (int side = 0; side < 2; ++side )
			{
				angle[ side ]      = PI;
				dist[ side ]       = INFINITY;
				link_index[ side ] = -1;
				link_side[ side ]  = -1;
			}
		}

		bool assigned( int side )
		{
			return ( link_index[ side ] >= 0 );
		}

		Node& linkedNode( int side )
		{
			return nodes->at( link_index[ side ] );
		}

		void unassign( int side )
		{
			link_index[ side ] = -1;
		}

		void linkSide( int side, int hangedNodeRange, int hangedSide )
		{
			Node& hangedNode = nodes->at( hangedNodeRange );
			angle[ side ]      = -hangedNode.angle[ hangedSide ];
			link_index[ side ] = hangedNodeRange;
			link_side[ side ]  = hangedSide;
			dist[side]         = hangedNode.dist[ hangedSide ];
		}

		float angle[2];    // angle between the node and the related
		float dist[2];     // distance between the node and the related
		int link_index[2]; // place of the related nodes
		int link_side[2];  // side of the linked node with wich the node is linked

	private:
		std::vector< Node >* nodes;
};

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
		std::vector< Node >                     nodes;
};

#endif // CDDETECTOR_HH
