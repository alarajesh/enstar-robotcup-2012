#ifndef ALGOS_H
#define ALGOS_H
#include "stdafx.h"

/*
	variables globales!
	NB: c'est moche, mais c'est necessaire...
	    à vous le choix, laisser ces variables globales tranquilles ou le segmentation fault!
*/
IplImage* frame,*frame2;
IplImage* edge;
CvSeq* contours;
vector<CvSeq*> List_Contours;
//
#define RT_PAS 1	/*pas entre deux points successifs*/
#define RT_TH 30
#define RT_LENGTHCONDITION 16
void premier_filtre(const CvArr* src, CvArr* dst,CvMemStorage *storage);
void line_seg_appr(CvMemStorage *storage);
void curve_seg_appr(CvMemStorage *storage2);

void Neighborhood_curve_grouping(void);
void Global_curve_grouping(void);
void RT_grouper_arcs(int c1,int c2,int ext1,int ext2);
void supprimer_redondances(void);


struct distance
{
	double D;	//distance entre les extremités
	int contour2;
	int ext1;	//extremité du premier contour
	int ext2;	//extremité du deuxieme contour
	double grad_angle;	//angle gradient entre les deux courbes
};

#endif