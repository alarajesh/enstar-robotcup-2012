#ifndef PARAMS_H
#define PARAMS_H
#include "stdafx.h"
/*
	D�terminaison des param�tres n�cessaires pour faire tourner les diff�rents algorithmes
*/
//Partie 1
//
void afficher_contours(void);
double angle(CvPoint* p1,CvPoint* p2,CvPoint* Centre);
double angle_bis(CvPoint* p1,CvPoint* p2,CvPoint* o1,CvPoint* o2);
void vider_contours(void);
#endif