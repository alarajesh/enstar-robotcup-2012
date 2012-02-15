#include "ARTagReader.hh"
#include <fstream>

ARTagReader::ARTagReader(const std::string& name) : /*{{{*/
		urbi::UObject (name)
{
	UBindFunction(ARTagReader, init);
} /*}}}*/

void ARTagReader::init(std::string camParamFile, ufloat markSize) /*{{{*/
{
	// binding /*{{{*/
		UBindVar(ARTagReader, outputImage);
		UBindVar(ARTagReader, inputImage);
		UBindVar(ARTagReader, markerSize);
		UBindVar(ARTagReader, markerPosition);
		UBindVar(ARTagReader, showOutput);
	/*}}}*/

	// on change /*{{{*/
		UNotifyChange(inputImage, &ARTagReader::update);
	/*}}}*/

	// initialize vars /*{{{*/
		markerSize = markSize;
		camParam.readFromXMLFile(camParamFile);
		markerPosition = urbi::UList();
		showOutput = false;
	/*}}}*/
} /*}}}*/

int ARTagReader::update() /*{{{*/
{
	urbi::UBinary bin = inputImage;

	cv::Mat inputFrame(bin.image.height,
					bin.image.width,
					CV_8UC3, bin.image.data);

	cv::Mat outputFrame;

	cv::undistort(inputFrame, outputFrame, camParam.CameraMatrix, camParam.Distorsion);
	markerDetector.detect(outputFrame, markers, camParam.CameraMatrix, camParam.Distorsion, (ufloat) markerSize);

	double* modelViewMatrix = new double[16];
	double* position = new double[3];
	double* orientation = new double[4];

	urbi::UList positionMarker;
	urbi::UList tmp;

	for (unsigned int i=0; i < markers.size(); i++) /*{{{*/
	{
		markers[i].draw(outputFrame, cv::Scalar(0,0,255), 2);
		tmp = urbi::UList();

		markers[i].OgreGetPoseParameters(position, orientation);
		tmp.push_back(markers[i].id);
		tmp.push_back(position[0]);
		tmp.push_back(position[1]);
		tmp.push_back(position[2]);
		tmp.push_back(orientation[0]);
		tmp.push_back(orientation[1]);
		tmp.push_back(orientation[2]);
		tmp.push_back(orientation[3]);

		positionMarker.push_back(tmp);
		
		// print tmp /*{{{*/
		/*
			tmp.print(std::cout);
			std::cout << std::endl;
		*/
		/*}}}*/

		// print marker data /*{{{*/
		/*
			std::cout << "marker " << markers[i].id << " :" << std::endl;
			std::cout << "       " << markers.at(i) << std::endl;
			std::cout << "       position : " << position[0] << "\t"
								<< position[1] << "\t"
								<< position[2] << std::endl;
			std::cout << "    orientation : " << orientation[0] << "\t"
								<< orientation[1] << "\t"
								<< orientation[2] << "\t"
								<< orientation[3] << std::endl;
		*/
		/*}}}*/
	} /*}}}*/

	// print positionMarker {{{
	/*
		positionMarker.print(std::cout);
		std::cout << std::endl;
	*/
	/*}}}*/

	markerPosition = positionMarker;

	delete modelViewMatrix;
	delete position;
	delete orientation;

	if ((bool) showOutput)
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
} /*}}}*/

UStart(ARTagReader);
