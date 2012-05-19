#ifndef POSSERVO_HH
# define POSSERVO_HH

#include <urbi/uobject.hh>
#include <math/quaternion.hpp>
#include <pthread.h>
#include <queue>
#include <opencv2/opencv.hpp>

#define BEGIN_TRANSMISSION (uint8_t) '\001'
#define END_TRANSMISSION   (uint8_t) '\004'

#define TAG1_POS Complex( 3, 1 )
#define TAG2_POS Complex( 0, 0 )
#define TAG3_POS Complex( 0, 2 )

#define Q_x() R_component_2()
#define Q_y() R_component_3()
#define Q_z() R_component_4()
#define Q_conj( quaternion ) boost::math::conj( quaternion )

typedef boost::math::quaternion< ufloat > Quaternion;
typedef std::complex< ufloat > Complex;

class PointE : public cv::Point2i
{
public:
	float theta;
	float cost;
	float dist;
};

class openQueuePointComparison
{
public:
	openQueuePointComparison(const bool& revparam = true)
	{
		reverse = revparam;
	}

	inline bool operator()(const PointE& lhs, const PointE& rhs) const
	{
		if (reverse)
			return (lhs.cost > rhs.cost);
		else
			return (lhs.cost < rhs.cost);
	}

private:
	bool reverse;
};

class PosServo: public urbi::UObject
{
	public:
		PosServo( const std::string &n );

		int  init();
		void getNextMotion();

		void onNewMarkerPositions( urbi::UVar& markers );
		void onNewOdometry( urbi::UVar& pos );

		void setTarget( std::vector< ufloat > targ );
		void setPosition( std::vector< ufloat > pos );

		void pathFindDijkstra();
		bool computePathDijkstra( cv::Point2i start_position );

	private:
		urbi::UVar markerPositions; // UList(UList(ufloat)) // input
		urbi::UVar positionOdom;    // UList(ufloat) xyt    // input

		Complex bioloid_position;
		Complex adv_robot_1_position;
		Complex adv_robot_2_position;

		// current position
		Complex position;
		ufloat  angle;

		// target
		Complex targetToReach;

		// dijkstra
		std::vector< cv::Point2i > path_to_follow;
		std::vector< Complex > m_path;
		cv::Mat freeMap;
		cv::Mat m_map;
		cv::Mat m_safeArea;
		cv::Mat m_pathMap;
		cv::Mat m_previous;
		cv::Mat m_distToObst;

		// event
		urbi::UEvent unreachable;
};

#endif
