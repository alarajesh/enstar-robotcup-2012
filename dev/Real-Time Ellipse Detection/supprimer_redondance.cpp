#define MIN_ERREUR 10
#define MIN_OK 3

#include <cv.h>
#include <cmath>

#include "./stdafx.h"
#include "./algos.h"

int mesure_erreur(CvSeq* S1,CvSeq* S2,int i)
{
	int n=S2->total;
	int ok=0;
	long double erreur=0,t;
	CvPoint* p=(CvPoint*)cvGetSeqElem(S1,i),*p1=(CvPoint*)cvGetSeqElem(S2,0),*p2;
	int a,b,u,v;
	for(i=0;i<n-1;++i)
	{
		p2=(CvPoint*)cvGetSeqElem(S2,i+1);
		a=p2->x-p->x;	b=p2->y-p->y;
		u=p2->x-p1->x;	v=p2->y-p1->y;
		t=(a*u+b*v)/(0.0+u*u+v*v);
		if(t<0 || t>1)
		{
			erreur=sqrt(std::min(0.0+a*a+b*b,RT_norme2(p1,p)));
		}
		else
			erreur=abs(b*u-a*v)/sqrt(u*u+v*v+0.0);
		if(erreur<MIN_ERREUR)
		{
			ok++;
			break;
		}
		p1=p2;
	}
	return ok;
}

int reperer_redondance(CvSeq* S1,CvSeq* S2)
{
	int i,n;
	int nb_ok=0;
	//
	if(S2->total>S1->total)
	{
		n=S1->total;
		for(i=0;i<n;i++)
		{
			nb_ok+=mesure_erreur(S1,S2,i);
			if(nb_ok>MIN_OK)
				goto fin_redondance;
		}
	}
	else
	{
		n=S2->total;
		for(i=0;i<n;i++)
		{
			nb_ok+=mesure_erreur(S2,S1,i);
			if(nb_ok>MIN_OK)
				goto fin_redondance;
		}
	}
fin_redondance:
	
	if(nb_ok>MIN_OK)
		return 1;
	return 0;
}

void supprimer_redondances(void)
{
	int i,j,redondance;
	for( i = 0; i < (int) List_Contours.size(); ++i )
	{
		if(List_Contours[i]->total<5 || cvContourPerimeter(List_Contours[i])<20)
		{
			List_Contours.erase(List_Contours.begin()+i);
			i--;
			continue;
		}
		for( j = 0; j < (int) List_Contours.size(); ++j )
		{
			if(i==j || List_Contours[j]->total<5 || cvContourPerimeter(List_Contours[j])<20)continue;
			redondance=reperer_redondance(List_Contours[i],List_Contours[j]);
			if(redondance)
			{
				if(cvContourPerimeter(List_Contours[i]) < cvContourPerimeter(List_Contours[j]))
				{
					List_Contours.erase(List_Contours.begin()+i);
					i--;
					break;
				}
				else
				{
					List_Contours.erase(List_Contours.begin()+j);
					if(j>i)
					{
						j--;
						continue;
					}
					i-=2;
					break;
				}
			}
		}
	}
}
