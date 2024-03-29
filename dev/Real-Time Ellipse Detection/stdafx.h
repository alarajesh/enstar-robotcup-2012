#ifndef STDAFX_H
#define STDAFX_H

#define RT_PI 3.1415926535
#define RT_signe(a) ((a)>=0?1:-1)

//carr� d'un nombre
#define RT_pow2(a) ((a)*(a))
//p1 est consid�r� comme origine du rep�re
#define RT_abs_rel(p1,p2) ((p2)->x-(p1)->x)
#define RT_ord_rel(p1,p2) (-(p2)->y+(p1)->y)
//carre de la taille du segment [p1,p2]
#define RT_norme2(p1,p2) (double)( RT_pow2(RT_abs_rel(p1,p2))+RT_pow2(RT_ord_rel(p1,p2)))
//minimum
#define min2(a,b) ((a)<(b)?a:b)

#endif
