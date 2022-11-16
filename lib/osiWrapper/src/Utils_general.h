#pragma once

#ifndef __UTILS_GENERAL_H__
#define __UTILS_GENERAL_H__

#endif // __UTILS_GENERAL_H__ 


/**
 * @brief computes euclidean distance between two points
 * 
 * @param p 
 * @param q 
 * @return double euclidean distance
 */
double euclideanDistance(Point2D p, Point2D q){

  return sqrt(pow(p.x-q.x,2) + pow(p.y-q.y,2));
}


/**
 * @brief coordinate rotation with angle phi
 *
 * @param pre-rotation point
 * @param post-rotation point
 * @param angle
 */
void transform(Point2D& pre, Point2D& post, double phi) {

  double c = std::cos(phi);
  double s = std::sin(phi);

  post.x = pre.x * c + pre.y * s;
  post.y = pre.x * (-1 * s) + pre.y * c;
}


/**
 * @brief get norm of osi3::Vector3d
 *
 * @param v vector
 * @return euclidean norm
 */
double getNorm(osi3::Vector3d v) {

  return sqrt(pow(v.x(), 2) + pow(v.y(), 2) + pow(v.z(), 2));
}


/**
 * @brief get wrapped angle to [-pi,pi]
 *
 * @param psi input angle
 * @return wrapped angle
 */
double wrapAngle(double psi) {

  return std::atan2(std::sin(psi), std::cos(psi));
}


/**
 * @brief function removes duplicates from a vector if dv < 0.01
 * 
 * @param v vector entries
 * @param eps threshold for removing
 */
void removeDuplicates(std::vector<Point2D> &v, double eps = 0.01) {
  
  if (v.size() == 0) {
    return;
  }

  for (int i = 0; i < v.size() - 1; i++) {
    double dv = pow(v[i + 1].x - v[i].x, 2) + pow(v[i + 1].y - v[i].y, 2);

    if (dv < eps) {
        v.erase(v.begin() + (i+1));
        i--;
    }
  }
}


/**
 * @brief find a point in a point vector based on distanc (small tolerance eps) 
 * 
 * @param points vector of points
 * @param point specfic point for checking
 * @param eps threshold for distance between two points
 * @return int  index of point in vector
 */
int findPointInPoints(std::vector<Point2D> points, Point2D point, double eps = 0.001) {

  for (int i = 0; i < points.size(); i++)
  {
    if (euclideanDistance(point, points[i]) < eps) {
        return i;
    }
  }
  return -1;
}