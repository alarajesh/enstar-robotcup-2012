#include "stdafx.h"
/*
	I.Curve Ellipse Extraction
*/
//Etape 1:	Detection des contours dans l'image
void premier_filtre(const CvArr* src, CvArr* dst,CvMemStorage *storage)
{
	cvCvtColor(src,dst,CV_BGR2GRAY);
	cvCanny(dst,dst,90,180,3);  //les deux parametres choisis ici sont 100 et 150
	cvFindContours( dst, storage, &contours, sizeof(CvContour),
                CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0) ); //parametres à vérifier!!
}
//Etape 2:	Suppression des contours courts
void line_seg_appr(CvMemStorage *storage)
{
	#define MIN_TOTAL 4
	#define MIN_PIXEL 60
	CvSeq* tmp=contours,*tmp2;
	if(!contours)return;
	//approximation des contours par des polygones
	contours = cvApproxPoly( contours, sizeof(CvContour), storage, CV_POLY_APPROX_DP, 1, 1);
	for(tmp=contours;tmp;tmp=tmp2)
	{
		tmp2=tmp->h_next;
		if(tmp->total<5*RT_PAS || cvContourPerimeter(tmp)<MIN_PIXEL)
			continue;
		List_Contours.push_back(tmp);//cvCloneSeq(tmp,storage));
	}
}

//Etape 3:	Curve segmentation
//TODO: decouper les contours de taille 3 ou 4
void curve_seg_appr(CvMemStorage *storage2)
{
	CvSeq* tmp,*tmp2,*tmp3;
	CvPoint* p1,*p2,*p3,*p4,*p5;	//points
	double alpha1,alpha2,alpha3,alpha4;	//angles alphas (voir la description de l'algorithme)
	double beta1,beta2,beta3;	//angles betas (voir la description de l'algorithme)
	double norme_cond;	//condition sur le rapport des normes
	int i,j,n;

	#define RT_point(i) (CvPoint*)cvGetSeqElem (tmp, (i)*RT_PAS)

	for(j=0;j<List_Contours.size();++j)
	{
		tmp=List_Contours[j];
		n=tmp->total;
		if(n<5*RT_PAS)
			continue;

		p1= RT_point(0);// Om,n-2
		p2= RT_point(1);// Om,n-1
		p3= RT_point(2);// Om,n
		p4= RT_point(3);// Om,n+1
		for(i=2; (i+2)*RT_PAS<n; i++ )
		{
			p5=RT_point(i+2); // Om,n+2
			norme_cond=RT_norme2(p2,p3)/RT_norme2(p4,p3);
			//calcul des alphas
			alpha1=angle(p2,p3,p1);
			alpha2=angle(p2,p4,p1);
			alpha3=angle(p4,p3,p5);
			alpha4=angle(p4,p2,p5);
			//calcul des betas
			beta1=angle(p1,p3,p2);
			beta2=angle(p4,p2,p3);
			beta3=angle(p3,p5,p4);

			if(	((alpha1*alpha2)<0 || (alpha3*alpha4)<0)
			||	(abs(alpha1)>abs(alpha2) || abs(alpha3)>abs(alpha4))
			||	((abs(beta1)-abs(beta2))>RT_TH || (abs(beta3)-abs(beta2))>RT_TH)
			||	(norme_cond>RT_LENGTHCONDITION || norme_cond<1.0/RT_LENGTHCONDITION)  )
			{
				tmp2=cvSeqSlice(tmp,cvSlice(0,i*RT_PAS+1));
				tmp3=cvSeqSlice(tmp,cvSlice(i*RT_PAS,n));
				//cvCvtColor(edge,frame2,CV_GRAY2BGR);
				if(tmp3->total>2*RT_PAS)
				{
					List_Contours.push_back(tmp3);
					//cvDrawContours( frame2, tmp3, CV_RGB(rand()%256,rand()%256,rand()%256), CV_RGB(rand()%256,rand()%256,rand()%256), 0, 2, CV_AA, cvPoint(0,0) );
				}
				if(tmp2->total>2*RT_PAS)
				{
					List_Contours[j]=tmp2;
					//cvDrawContours( frame2, tmp2, CV_RGB(rand()%256,rand()%256,rand()%256), CV_RGB(rand()%256,rand()%256,rand()%256), 0, 2, CV_AA, cvPoint(0,0) );
				}
				else
				{
					List_Contours.erase (List_Contours.begin()+j);
					j--;
				}/*
				printf("p:\t%d %d|%d %d|%d %d|%d %d|%d %d\n",p1->x,p1->y,p2->x,p2->y,p3->x,p3->y,p4->x,p4->y,p5->x,p5->y);
				printf("alpha:\t%lf %lf %lf %lf\n",alpha1,alpha2,alpha3,alpha4);
				printf("beta:\t%lf %lf %lf\n",beta1,beta2,beta3);
				printf("norme %lf\n-----\n",norme_cond);
				cvShowImage( "Ellipses Detection", frame2);
				cvWaitKey(0);*/
				
				break;
			}
			//on passe au point suivant
			p1=p2; p2=p3; p3=p4; p4=p5;
		}
	}
}

/*
	II. Curve Grouping Approaches
*/
//Etape 1:	Neighborhood Curve Grouping

void Neighborhood_curve_grouping(void)
{
	int i,j;
	int k1,k2;	//necessaires pour le calcul de Dmn
	double dist;
	vector<struct distance> D;
	struct distance D_tmp,D_min;
	CvPoint p[4];	//extremités des deux contours (voir "l'algorithme1")
	//defintion des macros
	#define RT_acces_point(a,b)	(CvPoint*)cvGetSeqElem (List_Contours[(a)], (b))
	#define RT_contour_debut(a) *RT_acces_point((a), 0)
	#define RT_contour_fin(a) *RT_acces_point((a), List_Contours[(a)]->total-1)
	#define RT_contour_deuxieme_point(a,b) *RT_acces_point((a),((b)==0?1:(List_Contours[(a)]->total-2)))
	#define RT_MAX_DIST 60
	#define RT_MINGRAD 20

	for(i=0;i<List_Contours.size();++i)
	{
		for(j=0;j<List_Contours.size();j++)
		{
			if(j==i)continue;

			p[0]=RT_contour_debut(i);
			p[1]=RT_contour_fin(i);
			p[2]=RT_contour_debut(j);
			p[3]=RT_contour_fin(j);

			D_tmp.D=RT_MAX_DIST;
			D_tmp.ext1=D_tmp.ext2=0;
			for(k1=0;k1<2;++k1)
			{
				for(k2=0;k2<2;++k2)
				{
					dist=RT_norme2(&p[k1],&p[2+k2]);
					if(dist<D_tmp.D)
					{
						D_tmp.D=dist;
						D_tmp.ext1=k1;
						D_tmp.ext2=k2;
					}
				}
			}
			if(D_tmp.D<RT_MAX_DIST)
			{
				D_tmp.contour2=j;
				p[1-D_tmp.ext1]=RT_contour_deuxieme_point(i,D_tmp.ext1);	//egale à Om2 is ext1 represente Om1
				p[2+1-D_tmp.ext2]=RT_contour_deuxieme_point(j,D_tmp.ext2);	//egale à On2 is ext2 represente On1
				//calcul de l'angle
				D_tmp.grad_angle=angle_bis(&p[1-D_tmp.ext1],&p[D_tmp.ext2],&p[D_tmp.ext1],&p[2+1-D_tmp.ext2]);
				D.push_back(D_tmp);
			}
		}
		if(D.size())
		{
			D_min=D[0];
			while(D.size())
			{
				D_tmp=D.back();
				D.pop_back();
				if(abs(D_tmp.grad_angle)<abs(D_min.grad_angle))
					D_min=D_tmp;
			}

			if(abs(D_min.grad_angle)<RT_MINGRAD)
			{
				RT_grouper_arcs(i,D_min.contour2,D_min.ext1,D_min.ext2);
				List_Contours.erase(List_Contours.begin()+D_min.contour2);
				i--;
			}
		}
	}
}

void RT_grouper_arcs(int c1,int c2,int ext1,int ext2)	//ext1==0 pour indiquer que c'est l'extremité du debut de c1 qui est concerné
														//et 1 si c'est la fin du contour qui se traite
{
	CvPoint* p;
	int i;	//!!! fuite de memoire: ne pas oublier de supprimer c2 avec cvClearSeq(List_Contours[c2])!
	if(ext1==1)
	{
		if(ext2==0)
		{
			for(i=0;i<List_Contours[c2]->total;++i)
			{
				p=RT_acces_point(c2,i);
				cvSeqInsert(List_Contours[c1],List_Contours[c1]->total,p);
			}
			//cvClearSeq(List_Contours[c2]);
		}
		else
		{
			for(i=List_Contours[c2]->total-1;i>=0;--i)
			{
				p=RT_acces_point(c2,i);
				cvSeqInsert(List_Contours[c1],List_Contours[c1]->total,p);
			}
			//cvClearSeq(List_Contours[c2]);
		}
	}
	else
	{
		if(ext2==0)
		{
			for(i=0;i<List_Contours[c2]->total;++i)
			{
				p=RT_acces_point(c2,i);
				cvSeqInsert(List_Contours[c1],0,p);
			}
			//cvClearSeq(List_Contours[c2]);
		}
		else
		{
			for(i=List_Contours[c2]->total-1;i>=0;--i)
			{
				p=RT_acces_point(c2,i);
				cvSeqInsert(List_Contours[c1],0,p);
			}
			//cvClearSeq(List_Contours[c2]);
		}
	}
}


void Global_curve_grouping(void)
{
	int i,j;
	int k1,k2;	//necessaires pour le calcul de Dmn
	double dist,d;
	vector<struct distance> D;
	struct distance D_tmp,D_min;
	CvPoint p[4];	//extremités des deux contours (voir "l'algorithme1")
	CvPoint O[2];
	CvPoint C[2];
	//defintion des macros
	#define RT_acces_point(a,b)	(CvPoint*)cvGetSeqElem (List_Contours[(a)], (b))
	#define RT_contour_debut(a) *RT_acces_point((a), 0)
	#define RT_contour_fin(a) *RT_acces_point((a), List_Contours[(a)]->total-1)
	#define RT_contour_deuxieme_point(a,b) *RT_acces_point((a),((b)==0?1:(List_Contours[(a)]->total-2)))

	for(i=0;i<List_Contours.size();++i)
	{
		for(j=i+1;j<List_Contours.size();j++)
		{
			p[0]=RT_contour_debut(i);
			p[1]=RT_contour_fin(i);
			p[2]=RT_contour_debut(j);
			p[3]=RT_contour_fin(j);

			D_tmp.D=RT_norme2(&p[0],&p[2]);
			D_tmp.ext1=D_tmp.ext2=0;
			for(k1=0;k1<2;++k1)
			{
				for(k2=0;k2<2;++k2)
				{
					dist=RT_norme2(&p[k1],&p[2+k2]);
					if(dist<D_tmp.D && dist >=RT_MAX_DIST)
					{
						D_tmp.D=dist;
						D_tmp.ext1=k1;
						D_tmp.ext2=k2;
					}
				}
			}
			if(D_tmp.D>=RT_MAX_DIST)
			{
				D_tmp.contour2=j;
				//calcul de C
				C[0].x=(p[0].x+p[1].x)/2;
				C[0].y=(p[0].y+p[1].y)/2;
				C[1].x=(p[2].x+p[3].x)/2;
				C[1].y=(p[2].y+p[3].y)/2;
				//calcul de O
				O[0]=*(CvPoint*)cvGetSeqElem (List_Contours[i], List_Contours[i]->total/2);
				O[1]=*(CvPoint*)cvGetSeqElem (List_Contours[j], List_Contours[j]->total/2);
				//
				d=RT_norme2(&O[0],&O[1]);
				if(d>RT_norme2(&O[0],&C[1]) && d>RT_norme2(&O[1],&C[0]) )
				{
					D.push_back(D_tmp);
				}
			}
		}
		if(D.size())
		{
			for(j=0;j<D.size();++j)
			{
				RT_grouper_arcs(i,D[j].contour2,D[j].ext1,D[j].ext2);
			}
			for(j=D.size()-1;j>=0;--j)
			{
				List_Contours.erase(List_Contours.begin()+D[j].contour2);
			}
			printf("%d\n",D.size());
			D.clear();
		}
	}
}