#include "ARTagReader.hh"
#include <fstream>

#define PRINT_MARKERS

ARTagReader::ARTagReader(const std::string& name) :
	urbi::UObject ( name )
{
	UBindFunction( ARTagReader, init );
}

void ARTagReader::init( std::string camParamFile, ufloat markSize )
{
	// binding
	UBindVar( ARTagReader, outputImage );
	UBindVar( ARTagReader, inputImage );
	UBindVar( ARTagReader, markerSize );
	UBindVar( ARTagReader, markerPositions );
	UBindVar( ARTagReader, showOutput );

	// on change
	UNotifyChange(inputImage, &ARTagReader::update);

	// initialize vars
	markerSize = markSize;
	camParam.readFromXMLFile( camParamFile );
	markerPositions = urbi::UList();
	showOutput = false;
}

int ARTagReader::update()
{
	urbi::UBinary bin = inputImage;

	cv::Mat inputFrame(bin.image.height,
			bin.image.width,
			CV_8UC3, bin.image.data);

	cv::Mat outputFrame;

	cv::undistort(
			inputFrame,
			outputFrame,
			camParam.CameraMatrix,
			camParam.Distorsion );
	markerDetector.detect(
			outputFrame,
			markers,
			camParam.CameraMatrix,
			camParam.Distorsion,
			(ufloat) markerSize );

	double* modelViewMatrix = new double[16];
	double* position = new double[3];
	double* orientation = new double[4];

	urbi::UList positionMarker;

	for (unsigned int i=0; i < markers.size(); i++)
	{
		markers[i].draw( outputFrame, cv::Scalar( 0,0,255), 2 );
		urbi::UList tmp;

		markers[i].OgreGetPoseParameters( position, orientation );
		tmp.push_back( markers[i].id );
		tmp.push_back( position[0] );
		tmp.push_back( position[1] );
		tmp.push_back( position[2] );
		tmp.push_back( orientation[0] );
		tmp.push_back( orientation[1] );
		tmp.push_back( orientation[2] );
		tmp.push_back( orientation[3] );

		positionMarker.push_back( tmp );

#define SQUARE(x) ( (x) * (x) )
		/*
		std::cout << "angle  : " << ( 2 * acos( orientation[0] ) * 180 / M_PI ) << std::endl;
		std::cout << "vector : " << ( orientation[1] / sqrt( 1 - SQUARE( orientation[0] ) ) ) << std::endl;
		std::cout << "         " << ( orientation[2] / sqrt( 1 - SQUARE( orientation[0] ) ) ) << std::endl;
		std::cout << "         " << ( orientation[3] / sqrt( 1 - SQUARE( orientation[0] ) ) ) << std::endl;
		*/

		
	}

	markerPositions = positionMarker;

	delete modelViewMatrix;
	delete position;
	delete orientation;

	if ( (bool) showOutput )
	{
		urbi::UBinary out;
		out.type              = urbi::BINARY_IMAGE;
		out.image.height      = outputFrame.rows;
		out.image.width       = outputFrame.cols;
		out.image.size        = outputFrame.rows*outputFrame.cols*3;
		out.image.imageFormat = urbi::IMAGE_RGB;
		out.image.data        = outputFrame.data;
		out.allocated_        = false;

		outputImage = out;
	}
	return 0;
}

UStart( ARTagReader );
