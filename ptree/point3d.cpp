/* Programmer : Kevin Greenan <kmgreen2@ilstu.edu>
 * File Name : point3d.cpp
 *
 * Description : I shouldn't have to explain this one
 *
 * Date : 2/28/2003
 */

#include "point3d.h"
#include <math.h>

Point3D::Point3D(float p_x, float p_y, float p_z)
{
   m_x = p_x;
   m_y = p_y;
   m_z = p_z;
}


Point3D::Point3D(const Point3D& p_Pt)
{
   m_x = p_Pt.m_x;
   m_y = p_Pt.m_y;
   m_z = p_Pt.m_z;
}


float Point3D::Dist(float p_x, float p_y, float p_z)
{
   return(float(sqrt(DistSquare(p_x, p_y, p_z))));
}


float Point3D::DistSquare(float p_x, float p_y, float p_z)
{
   float xdist(m_x - p_x), ydist(m_y - p_y), zdist(m_z - p_z);

   return(float((xdist * xdist) + (ydist * ydist) + (zdist * zdist)));
}


Point3D Point3D::operator=(const Point3D& p_Pt)
{
   Point3D pt(p_Pt);

   m_x = p_Pt.m_x;
   m_y = p_Pt.m_y;
   m_z = p_Pt.m_z;

   return(pt);
}
