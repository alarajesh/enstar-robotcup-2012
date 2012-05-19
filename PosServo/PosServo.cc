#include <cmath>

#include "PosServo.hh"

#define ABS( x) ( (x ) < 0 ) ? (-( x)) : (x )
#define STEP 0.01
#define ARC_LENGTH 0.5
#define ROBOT_RADIUS 0.20

#define GROUND_COLOR ( uchar ) 255
#define OBJECT_COLOR ( uchar ) 0
#define PATH_COLOR   ( uchar ) 100
#define MAP_FILE "/home/enstar/enstar-robotcup-2012/PosServo/map2b.png"
#define NB_XX 200
#define NB_YY 300

static bool isInAchievableArea( cv::Point2i start, cv::Point2i end, cv::Mat& map )
{
	cv::LineIterator it( map, start, end, 8 );

	for ( int i = 0; i < it.count; ++i, ++it )
	{
		if ( !(map.at<float>( it.pos() ) > 0 ) )
			return false;
	}

	return true;
}

static PointE globToLoc( Complex glob )
{
	PointE res;
	res.x = ( int) (real(glob ) * 100);
	res.y = ( int) (imag(glob ) * 100);
	return res;
}

static Complex locToGlob( cv::Point2i loc )
{
	return Complex( loc.y / 100, loc.x / 100 );
}

PosServo::PosServo( const std::string &n ) :
  urbi::UObject(n)
{
	UBindFunction( PosServo, init );
}

int PosServo::init()
{
	// Binding
	UBindVar( PosServo, markerPositions );
	UBindVar( PosServo, positionOdom );
	//UBindVar( PosServo, movement );
	//UBindVar( PosServo, motion );

	UBindFunction( PosServo, getNextMotion );
	UBindFunction( PosServo, setTarget );
	UBindFunction( PosServo, setPosition );
	UBindFunction( PosServo, pathFindDijkstra );

	UBindEvent( PosServo, unreachable );

	UNotifyChange( markerPositions, &PosServo::onNewMarkerPositions );
	UNotifyChange( positionOdom,    &PosServo::onNewOdometry );

	freeMap = cv::imread( MAP_FILE, 0 );
	m_map   = freeMap;

	position = Complex( 0.45, 0.25 );
	targetToReach = Complex( 0, 0 );

	return 0;
}

void PosServo::setTarget( std::vector<ufloat> targ )
{
	if ( targ.size() < 2 )
	{
		std::cout << "bad target size" << std::endl;
		return;
	}
	targetToReach = Complex( targ[0], targ[1] );
}

void PosServo::setPosition( std::vector<ufloat> pos )
{
	if ( pos.size() < 3 )
	{
		std::cout << "bad position size" << std::endl;
		return;
	}

	position = Complex( pos[0], pos[1] );
	angle = pos[2];
}

void PosServo::getNextMotion()
{
}

void PosServo::onNewMarkerPositions( urbi::UVar& markers_ )
{
	urbi::UList markers = markers_;

	Complex new_position( 0, 0 );
	ufloat  new_angle = 0;
	int     new_pos_count = 0;

	Complex new_bioloid_pos( 0, 0 );
	int     new_bio_pos_count = 0;

	Complex new_adv_robot_1_pos( 0, 0 );
	int     new_adv_r_1_pos_count = 0;
	Complex new_adv_robot_2_pos( 0, 0 );
	int     new_adv_r_2_pos_count = 0;

	for ( int i = 0; i < markers.size(); ++i )
	{
		urbi::UList& marker = *( markers[i].list );

		if ( marker.size() != 8 ) continue;

		int id = marker[0];

		Quaternion position_( 0, marker[1], marker[2], marker[3] );
		Quaternion orientation( marker[4], marker[5], marker[6], marker[7] );

		Quaternion res = orientation * position_ * Q_conj( orientation );

		// tag 1 => robot position
		if ( 0 <= id && id < 2 )
		{
			Complex rel_pos = conj( Complex( res.Q_x(), res.Q_y() ) );
			Complex pos = TAG1_POS - rel_pos;
			new_position += pos;
			ufloat ang = arg( rel_pos ) - arg( Complex( marker[1], marker[3] ) ) + M_PI/2;
			while ( ang > M_PI )
			{
				ang -= 2*M_PI;
			}
			while ( ang < -M_PI )
			{
				ang += 2*M_PI;
			}
			new_angle += ang;
			/*
			std::cout << "tag 1" << std::endl;
			std::cout << "angle   : " << ang << std::endl;
			std::cout << "positi  : " << rel_pos << std::endl;
			std::cout << "(X',Y') : " << arg( rel_pos ) << std::endl;
			std::cout << "pos_rob : " << position_ << std::endl;
			std::cout << "( X,Y )   : " << arg( Complex( marker[1], marker[3] ) ) << std::endl;
			*/
			++new_pos_count;
		}

		// tag 2 => robot position
		if ( 2 <= id && id < 4 )
		{
			Complex rel_pos = Complex( res.Q_x(), res.Q_y() );
			Complex pos = TAG2_POS + rel_pos;
			new_position += pos;
			ufloat ang = arg( rel_pos ) - arg( Complex( marker[1], marker[3] ) ) - M_PI/2;
			while ( ang > M_PI )
			{
				ang -= 2*M_PI;
			}
			while ( ang < -M_PI )
			{
				ang += 2*M_PI;
			}
			new_angle += ang;
			/*std::cout << "tag 2" << std::endl;
			std::cout << "angle   : " << ang << std::endl;
			std::cout << "positi  : " << rel_pos << std::endl;
			std::cout << "(X',Y') : " << arg( rel_pos ) << std::endl;
			std::cout << "pos_rob : " << position_ << std::endl;
			std::cout << "( X,Y )   : " << arg( Complex( marker[1], marker[3] ) ) << std::endl;*/
			++new_pos_count;
		}

		// tag 3 => robot position
		if ( 4 <= id && id < 6 )
		{
			Complex rel_pos = Complex( res.Q_x(), res.Q_y() );
			Complex pos = TAG3_POS + rel_pos;
			new_position += pos;
			ufloat ang = arg( rel_pos ) - arg( Complex( marker[1], marker[3] ) ) - M_PI/2;
			while ( ang > M_PI )
			{
				ang -= 2*M_PI;
			}
			while ( ang < -M_PI )
			{
				ang += 2*M_PI;
			}
			new_angle += ang;
			/*
			std::cout << "tag 2" << std::endl;
			std::cout << "angle   : " << ang << std::endl;
			std::cout << "positi  : " << rel_pos << std::endl;
			std::cout << "(X',Y') : " << arg( rel_pos ) << std::endl;
			std::cout << "pos_rob : " << position_ << std::endl;
			std::cout << "( X,Y )   : " << arg( Complex( marker[1], marker[3] ) ) << std::endl;
			*/
			++new_pos_count;
		}

		// tag 4 => bioloid
		if ( 6 <= id && id < 10 )
		{
			//std::cout << "go here : " << __LINE__ << std::endl;
			Complex tmp( marker[3], -((ufloat) marker[1]) );
			std::cout << angle << std::endl;
			Complex rot( cos(angle), sin(angle) );
			std::cout << "marker : " << tmp << std::endl;
			std::cout << "rot : " << rot << std::endl;
			std::cout << "rel pos : " << rot * tmp << std::endl;

			new_bioloid_pos += (position + rot * tmp);
			++new_bio_pos_count;
		}

		// tag 5 => robot to destroy 1
		if ( 10 <= id && id < 14 )
		{
		}

		// tag 6 => robot to destroy 2
		if ( 14 <= id && id < 18 )
		{
		}
	}

	// update position
	if ( new_pos_count > 0 )
	{
		position = new_position * ( 1 / ( ufloat) new_pos_count );
		angle = new_angle * ( 1 / ( ufloat) new_pos_count );
		std::cout << "position : " << position << std::endl;
	}

	// update position bioloid
	if ( new_bio_pos_count > 0 )
	{
		bioloid_position = new_bioloid_pos * ( 1 / ( ufloat) new_bio_pos_count );
		std::cout << "position biol : " << new_bioloid_pos << std::endl;
	}
}

void PosServo::onNewOdometry( urbi::UVar& pos_ )
{
	urbi::UList pos = pos_;

	if ( pos.size() < 3 )
	{
		std::cout << "bad size odometry" << std::endl;
		return;
	}

	position = Complex( pos[0], pos[1] );
	angle = pos[2];
}

void PosServo::pathFindDijkstra()
{
	PointE robotpos = globToLoc( position );
	m_safeArea = ( m_map == GROUND_COLOR );

#ifdef DEBUG_DIJKSTRA // {
	std::cout << "dijkstra started \n";

	// check if the pixel is in the grid
	std::cout << "pos: " << robotpos << std::endl;
	std::cout << "val: " << ( m_safeArea.at<uchar> ( robotpos ) == 0 ) << std::endl;
	std::cout << "size: " << m_safeArea.size().width << " "
	<< m_safeArea.size().height << std::endl;

	cv::imshow( "safeArea", m_safeArea );
#endif // DEBUG_DIJKSTRA }

	if ( robotpos.x >= NB_XX
			|| robotpos.y >= NB_YY
			|| robotpos.x < 0
			|| robotpos.y < 0 )
	{
		std::cerr << "dijkstra: position is outside of the map" << std::endl;
		return;
	}

	// check if v is valid
	if ( m_safeArea.at<uchar> ( robotpos ) == ( uchar) 0 )
	{
		//obstacle
		std::cerr << "dijkstra: position is in black\n";
		return;
	}

	// initialise path map, la carte qui va contenir dijkstra
	m_pathMap = cv::Mat::zeros( m_map.size(), CV_32FC1 );
	m_pathMap.at<float> ( robotpos ) = 1.f;// 0 = obstacle, 1 est la plus petite distance ( score )
	robotpos.cost = 1.f;

	// initialise previous map
	m_previous = cv::Mat::zeros( m_map.size(), CV_32SC2 );

	// calcul distance to obstacle
	// calcul en L1 plus simple et suffisant
	cv::distanceTransform( m_safeArea, m_distToObst, CV_DIST_L2, 3 );
	double distmax, distmin, costObst;
	cv::minMaxLoc( m_distToObst, &distmin, &distmax );

	// a queue which reorder the point base on the score theta
	std::priority_queue<PointE, std::vector<PointE>, openQueuePointComparison>
			queue;

	//m_maxPathLength = 0;

	queue.push( robotpos );

	while (!queue.empty())
	{
		PointE pt = queue.top();
		queue.pop();

		// instanciate the 8 neighbor of the current node
		// Point voisin dans le sens horaire autour du point pt
		PointE ptNeighbor[8];

		ptNeighbor[0].x = pt.x + 1;
		ptNeighbor[0].y = pt.y;
		ptNeighbor[0].dist = 1;

		ptNeighbor[1].x = pt.x + 1;
		ptNeighbor[1].y = pt.y + 1;
		ptNeighbor[1].dist = 1.414213562; // racine de 2

		ptNeighbor[2].x = pt.x;
		ptNeighbor[2].y = pt.y + 1;
		ptNeighbor[2].dist = 1;

		ptNeighbor[3].x = pt.x - 1;
		ptNeighbor[3].y = pt.y + 1;
		ptNeighbor[3].dist = 1.414213562;

		ptNeighbor[4].x = pt.x - 1;
		ptNeighbor[4].y = pt.y;
		ptNeighbor[4].dist = 1;

		ptNeighbor[5].x = pt.x - 1;
		ptNeighbor[5].y = pt.y - 1;
		ptNeighbor[5].dist = 1.414213562;

		ptNeighbor[6].x = pt.x;
		ptNeighbor[6].y = pt.y - 1;
		ptNeighbor[6].dist = 1;

		ptNeighbor[7].x = pt.x + 1;
		ptNeighbor[7].y = pt.y - 1;
		ptNeighbor[7].dist = 1.414213562;

		// iterate the neighbor
		for (int i = 0; i < 8; i++) {
			// check if the pixel is in the grid
			if (ptNeighbor[i].x >= NB_YY || ptNeighbor[i].y >= NB_XX
					|| ptNeighbor[i].x < 0 || ptNeighbor[i].y < 0)
				continue;

			// check if pt is valid
			if (m_safeArea.at<uchar> ( ptNeighbor[i] ) == 0)//obstacle
				continue;

			// calcul de la nouvelle distance
			 if ( m_distToObst.at<float> ( ptNeighbor[i] ) < ROBOT_RADIUS/STEP )
				continue;

			// calcul du cout de déplacement lié aux obstacles
			costObst = ( ROBOT_RADIUS / STEP ) * 2 - m_distToObst.at<float> (
					ptNeighbor[i]) + 1;
			if (costObst < 1)
				costObst = 1;

			//int alt = pt.cost + abs(ptNeighbor[i].x - pt.x) + abs(ptNeighbor[i].y - pt.y) + costObst;
			float alt = pt.cost + ptNeighbor[i].dist + costObst;

			ptNeighbor[i].cost = m_pathMap.at<float> ( ptNeighbor[i] );

			// relax le graph
			if (alt < ptNeighbor[i].cost || ptNeighbor[i].cost == 0) {
				ptNeighbor[i].cost = alt;
				std::cout << ((cv::Point2i) ptNeighbor[i]) << std::endl;
				m_pathMap.at< float > ( ptNeighbor[i] ) = alt;
				m_previous.at< cv::Vec2i > ( ptNeighbor[i] )
						= cv::Vec2i( pt.x, pt.y );
				queue.push( ptNeighbor[i] );
				/*if (ptNeighbor[i].cost > m_maxPathLength)
					m_maxPathLength = ptNeighbor[i].cost;*/
			}
		}
	}

#ifdef DEBUG_DIJKSTRA {
	double toto, tata;
	cv::minMaxLoc( m_pathMap, &toto, &tata );
	cv::imshow( "pathMap", m_pathMap / tata );
	cv::waitKey(0);
	cv::imshow( "distObst", m_distToObst / distmax );
	cv::waitKey(0);
#endif // DEBUG_DIJKSTRA }

	computePathDijkstra( robotpos );
}

bool PosServo::computePathDijkstra( cv::Point2i start_position )
{
	PointE target = globToLoc( targetToReach );

	// check if target is in safeArea otherwise => badaboum
	if ( m_pathMap.at< float >( target ) <= 0 )
	{
		unreachable.emit();
		std::cout << "unreachable" << std::endl;
		return false;
	}

	std::cout << " come here : " << __LINE__ << std::endl;

	cv::Point2i current_point = target;
	std::vector< cv::Point2i > path;

#ifdef DEBUG_DIJKSTRA // {
	cv::Mat pathNoSmooth = m_safeArea;
#endif //DEBUG_DIJKSTRA }

	// follow the lightest path in the dijkstra tree
	while ( current_point != start_position )
	{
#ifdef DEBUG_DIJKSTRA // {
		// print path in safeArea
		pathNoSmooth.at< uchar >( current_point ) = PATH_COLOR;
#endif // DEBUG_DIJKSTRA }

		path.push_back(current_point);
		current_point = m_previous.at< cv::Point2i >( current_point );
	}

#ifdef DEBUG_DIJKSTRA // {

	// display brut path
	cv::imshow( "path without smooth", pathNoSmooth);
	cv::waitKey(0);
#endif // DEBUG_DIJKSTRA }

#ifdef DEBUG_DIJKSTRA // {

	// keep the smooth path in local coord
	std::vector< cv::Point2i > smooth_path;
	smooth_path.push_back( current_point );
#endif // DEBUG_DIJKSTRA }

	int path_size = path.size();
	int sup = path_size - 1;
	int inf = MAX( sup - (ARC_LENGTH/STEP), 0 );
	cv::Point2i start = current_point;

	m_path.clear();

	Complex currentPoint = locToGlob( current_point );
	m_path.push_back( currentPoint );

	// smooth the path
	while ( sup > 0 )
	{
		// if segment is in achieved area
		if ( isInAchievableArea(
					path.at( inf ),
					path.at( sup ),
					m_pathMap ) )
		{
			currentPoint = locToGlob( path.at( inf ) );
			m_path.push_back( currentPoint );
#ifdef DEBUG_DIJKSTRA // {
			smooth_path.push_back( path.at( inf ) );
#endif // DEBUG_DIJKSTRA }

			sup = inf;
			inf = MAX( 0, sup - ( ARC_LENGTH / STEP ) );
		}
		else // reduce segment length
		{
			if ( sup - inf <= 1 )
			{
				currentPoint = locToGlob( path.at( inf ) );
#ifdef DEBUG_DIJKSTRA
				smooth_path.push_back( path.at( inf ) );
#endif // DEBUG_DIJKSTRA
				m_path.push_back( currentPoint );
				sup = inf;
				inf = MAX( 0, sup - ( ARC_LENGTH / STEP ) );
			}
			else
			{
				inf = sup - MAX( (sup - inf) / 2, 1 );
			}
		}
	}

#ifdef DEBUG_DIJKSTRA //{

	cv::Mat pathSmooth = m_safeArea;
	std::vector< cv::Point >::iterator it1 = smooth_path.begin();
	std::vector< cv::Point >::iterator it2 = ++( smooth_path.begin() );

	for ( ; ( it1 != smooth_path.end() ) && ( it2 != smooth_path.end() ); ++it1, ++it2 )
	{
		cv::line( pathSmooth,
				*it1,
				*it2,
				(uchar) 80 );
	}

	cv::imshow( "path smooth", pathSmooth );
	cv::waitKey( 0 );

#endif // DEBUG_DIJKSTRA }
	return true;
}

UStart( PosServo );
