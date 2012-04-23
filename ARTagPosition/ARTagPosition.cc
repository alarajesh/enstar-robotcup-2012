#include "ARTagPosition.hh"
#include <fstream>

ARTagPosition::ARTagPosition(const std::string& name) : /*{{{*/
		urbi::UObject (name)
{
	UBindFunction(ARTagPosition, init);
} /*}}}*/

void ARTagPosition::init() /*{{{*/
{
	// binding /*{{{*/
		UBindVar(ARTagPosition, markerGroup);
		UBindVar(ARTagPosition, markerPosition);
		UBindVar(ARTagPosition, groupPosition);
	/*}}}*/

	// on change /*{{{*/
		UNotifyChange(markerPosition, &ARTagPosition::update);
	/*}}}*/

	// initialize vars /*{{{*/
		markerPosition = urbi::UList();
		markerGroup =    urbi::UList();
		groupPosition =  urbi::UList();
	/*}}}*/
} /*}}}*/

int ARTagPosition::update() /*{{{*/
{
	// TODO
	urbi::UList markers = markerPosition;

	//markers.front().print(std::cout);
	/*
	urbi::UList::iterator it;

	for (it = markers.begin(); it != markers.end(); it++)
	{
		if ((*it)[0] == 10)
		{
			(*it).print(std::cout);
		}
	}
	return 0;
	*/
} /*}}}*/

UStart(ARTagPosition);
