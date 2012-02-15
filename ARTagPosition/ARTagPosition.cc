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
		UNotifyChange(markerGroup,    &ARTagPosition::update);
	/*}}}*/

	// initialize vars /*{{{*/
		markerPosition = urbi::UList();
		markerGroup = urbi::UList();
		groupPosition = urbi::UList();
	/*}}}*/
} /*}}}*/

int ARTagPosition::update() /*{{{*/
{
	// TODO
	return 0;
} /*}}}*/

UStart(ARTagPosition);
