/* Programmer : Kevin Greenan <kmgreen2@ilstu.edu>
 * File Name : point3d.h
 *
 * Description : I shouldn't have to explain this one
 *
 * Date : 2/28/2003
 */

#ifndef __POINT3D_H__
#define __POINT3D_H__

#ifndef UNIX
#include <windows.h>
#endif

class Point3D
{
public:
   Point3D() { m_x = m_y = m_z = 0.0f; }
   Point3D(float p_x, float p_y, float p_z);
   Point3D(const Point3D& p_Pt);
   float Dist(float p_x, float p_y, float p_z);

   float Dist(Point3D p_pt) { return(Dist(p_pt.m_x, p_pt.m_y, p_pt.m_z)); }
   float DistSquare(float p_x, float p_y, float p_z);

   float DistSquare(Point3D p_pt) { return(DistSquare(p_pt.m_x, p_pt.m_y, p_pt.m_z)); }
   Point3D operator=(const Point3D& p_Pt);

   void set(float x, float y, float z) { m_x = x;
                                         m_y = y;
                                         m_z = z; }
   float m_x, m_y, m_z;
};
#endif
