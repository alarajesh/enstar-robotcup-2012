/*
	Real-Time Ellipse Detection

	Objectif: Déterminer les ellipsoides présents sur une image ainsi que leurs parametres(centre, a,b)
*/

#include <sys/types.h>
#include <cv.h>
#include <highgui.h>

#include "./stdafx.h"
#include "./algos.h"
#include "./params.h"

//TODO: creer une liste(<vector> pour les contours
	  //regarder du coté de CvSeqWriter
	  //calcul récursif des alphas/betas
	  //eliminer les variables globales

int main(int argc, char** argv)
{
	//CvCapture* capture =cvCreateCameraCapture(0);
	//CvSize size = cvSize((int)cvGetCaptureProperty( capture, CV_CAP_PROP_FRAME_WIDTH),(int)cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT));
	time_t seconds;
	CvSeq* contours;
	CvMemStorage* storage,*storage2;
	frame = cvLoadImage("ellipses_algo.png");//cvLoadImage("ellipses_algo.png"); //image d'origine à traiter
	edge=cvCreateImage(cvGetSize(frame),IPL_DEPTH_8U,1); //image en gris
	frame2=cvCreateImage(cvGetSize(frame),IPL_DEPTH_8U,3);
	
	contours = NULL;
	storage = cvCreateMemStorage(0);
	storage2 = cvCreateMemStorage(0);
	srand(time(NULL));
	cvNamedWindow( "Ellipses Detection", CV_WINDOW_AUTOSIZE );
	//Debut du filtre!
	/*
		Partie 1
	*/
	//printf("%lf\n",angle(&cvPoint(-1,-1),&cvPoint(0,-1),&cvPoint(0,0)));
	cvWaitKey(1000);
	seconds=time(NULL);
	//for(int i=0;i<1000;i++)
	{
		cvReleaseImage( &frame );
		frame = cvLoadImage("ellipses_algo.png");//cvLoadImage("ellipses_algo.png");
		vider_contours();
		cvClearMemStorage( storage );
		cvClearMemStorage( storage2 );
		//frame=cvQueryFrame(capture);
		premier_filtre(frame,edge,storage);	//detection des contours

		line_seg_appr(storage);	//Selection des contours
		afficher_contours();cvWaitKey(0);
		curve_seg_appr(storage2);	//decoupage des contours
		afficher_contours();cvWaitKey(0);
		supprimer_redondances();
		afficher_contours();cvWaitKey(0);
		/*
			Partie 2
		*/
		Neighborhood_curve_grouping();
		afficher_contours();cvWaitKey(0);
		Global_curve_grouping();
		afficher_contours();cvWaitKey(0);
		vider_contours();
	}
	seconds=time(NULL)-seconds;
	printf("\n-------\n\t%d \n", (int) seconds);
	//cvReleaseCapture( &capture );
	vider_contours();
	cvClearMemStorage( storage );
	cvClearMemStorage( storage2 );
	cvWaitKey(0);
	
	return 0;
}

