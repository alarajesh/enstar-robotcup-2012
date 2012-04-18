#include "stdafx.h"

double angle(CvPoint* p1,CvPoint* p2,CvPoint* Centre)	//OK!
{
	int a=(p1->x-Centre->x),b=-(p1->y-Centre->y),c=(p2->x-Centre->x),d=-(p2->y-Centre->y);
	double angle=(180*acos((a*c+b*d)/sqrt((double)(a*a+b*b)*(c*c+d*d))))/RT_PI;
	if((a*d-b*c)<0)angle=-angle;
	return angle;
}

double angle_bis(CvPoint* p1,CvPoint* p2,CvPoint* o1,CvPoint* o2)
{
	int a=(p1->x-o1->x),b=-(p1->y-o1->y),c=(p2->x-o2->x),d=-(p2->y-o2->y);
	double angle=(180*acos((a*c+b*d)/sqrt((double)(a*a+b*b)*(c*c+d*d))))/RT_PI;
	if((a*d-b*c)<0)angle=-angle;
	return angle;
}

void afficher_contours()	//OK!
{
	unsigned int i=0,j;
	CvBox2D box;
	CvPoint* point,*ptest;
	CvPoint2D32f p;
	double erreur,erreur2,err,taux;
	double alpha, beta,gamma,epsilon,tmp1,tmp2;
	cvCvtColor(edge,frame2,CV_GRAY2BGR);
	for(i=0;i<List_Contours.size();++i)
	{
		if(List_Contours[i]->total<5 || cvContourPerimeter(List_Contours[i])<80)continue;
		 box = cvFitEllipse2(List_Contours[i]);
		 box.size.height/=2;
		 box.size.width/=2;
		 //cvEllipse(frame2,cvPoint(floor(box.center.x),floor(box.center.y)),cvSize(floor(box.size.width),floor(box.size.height)),box.angle,0,360,CV_RGB(rand()%241+15,rand()%241+15,rand()%241+15),3);
		 cvDrawContours( frame2, List_Contours[i], CV_RGB(rand()%256,rand()%256,rand()%256), CV_RGB(rand()%256,rand()%256,rand()%256), 0, 2, CV_AA, cvPoint(0,0) );
	}
	cvShowImage( "Ellipses Detection", frame2);
}

void vider_contours(void)
{
	int i;
	List_Contours.clear();
}