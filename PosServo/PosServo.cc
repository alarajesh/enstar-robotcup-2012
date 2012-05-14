#include "PosServo.hh"

#include <cmath>

PosServo::PosServo(const std::string &n) :
	urbi::UObject(n)
{
	UBindFunction( PosServo, init );
}

int PosServo::init()
{
	// Binding
	UBindVar( PosServon, markerPositions );
	//UBindFunction( PosServo, declareDPosition );
	UBindFunction( PosServo, getNextMovement );

	UNotifyChange( markerPositions, &PosServo::onNewMarkerPositions );

	return 0;
}

void PosServo::getNextMovement()
{
}

void PosServo::onNewMarkerPositions( urbi::UVar& markers_ )
{
	urbi::UList markers = markers_;

	if ( markers.size() > 0 )
	{
		urbi::UList& marker = *(markers[0].list);
		if ( marker.size() == 8 )
		{
			int id = marker[0];

			Quaternion position( 0, marker[1], marker[2], marker[3] );
			Quaternion orientation( marker[4], marker[5], marker[6], marker[7] );

			//
			Quaternion toto = orientation * position * boost::math::conj( orientation );
			std::cout << "toto : " << toto << std::endl;
		}
	}
}

UStart(PosServo);
