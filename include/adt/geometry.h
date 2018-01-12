/*
 * geometry.h
 *
 *  Created on: Jan 9, 2018
 *      Author: bibei
 */

#ifndef INCLUDE_ADT_GEOMETRY_H_
#define INCLUDE_ADT_GEOMETRY_H_

#include <Eigen/Dense>

/*!
 * @brief This h-file is a kit of geometry.
 */

namespace qr_control {
namespace geometry {

///! The convenient alias.
using Point2d = Eigen::Vector2d;
using Point3d = Eigen::Vector3d;

///! ax + by + c = 0;
typedef struct {
  double a;
  double b;
  double c;
} Linear;

///////////////////////////////////////////////////////////////////////////////
////////////           The helper methods of 2-D plane             ////////////
///////////////////////////////////////////////////////////////////////////////
///! the distance between the point p1 to the point p2
double  distance(const Point2d& p1, const Point2d& p2);

///! the linear cross both the a and b, if a == b, return (0, 0, 0);
Linear  linear(const Point2d& a, const Point2d& b);
///! the cross point of l1 and l2, if these is not exist cross point, return (NaN, NaN);
Point2d cross_point(const Linear& l1, const Linear& l2);
///! the distance between the point p to the linear l.
double  distance(const Linear& l, const Point2d& p);

///! the area of triangle.
double  area_of_triangle        (const Point2d& a, const Point2d& b, const Point2d& c);
///! the incenter of triangle.
Point2d incenter_of_triangle    (const Point2d& a, const Point2d& b, const Point2d& c);
///! the orthocentre of triangle
Point2d orthocentre_of_triangle (const Point2d& a, const Point2d& b, const Point2d& c);
///! the circumcentre of triangle
Point2d circumcentre_of_triangle(const Point2d& a, const Point2d& b, const Point2d& c);

} /* end namespace geometry */
} /* end namespace qr_control */

#endif /* INCLUDE_ADT_GEOMETRY_H_ */
