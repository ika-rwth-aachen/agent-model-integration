#pragma once

#ifndef __UTILS_GEOMETRY_H__
#define __UTILS_GEOMETRY_H__

#endif // __UTILS_GEOMETRY_H__ 


/**
 * @brief computes gradients with finite differences
 *
 * @param x 
 * @param y
 * @param order of differentiation
 * @return gradient at every point
 */
std::vector<double> gradient(std::vector<double> x, std::vector<double> y,
                             int order) {

  std::vector<double> res;
  if (x.size() != y.size()){
    SPDLOG_ERROR("x and y do not have the same size (x:{}, y:{})", x.size(), y.size());
    exit(EXIT_FAILURE);
  }
  int n = x.size();
  res.assign(n, 0);

  if (order == 1) {
    // calculate interior gradient values with central differences
    for (int i = 1; i < n - 1; i++)
      res[i] = (y[i + 1] - y[i - 1]) / (x[i + 1] - x[i - 1]);

    // calculate gradient at edges with one sided differences
    res[0] = (y[1] - y[0]) / (x[1] - x[0]);
    res[n - 1] = (y[n - 1] - y[n - 2]) / (x[n - 1] - x[n - 2]);
  }

  if (order == 2) {
    // calculate interior gradient values with central differences
    for (int i = 1; i < n - 1; i++) {
      double h1 = (x[i] - x[i - 1]), h2 = (x[i + 1] - x[i]);
      res[i] =
        (2.0 / (h1 + h2)) * ((y[i + 1] - y[i]) / h2 - (y[i] - y[i - 1]) / h1);
    }

    // calculate gradient at edges with one sided differences
    // res[0] = (y[2] - 2 * y[1] + y[0]) / pow(x[1] - x[0], 2);
    double h1 = (x[1] - x[0]), h2 = (x[2] - x[1]);
    res[0] = (2.0 / (h1 + h2)) * ((y[2] - y[1]) / h2 - (y[1] - y[0]) / h1);
    h1 = (x[n - 1] - x[n - 2]), h2 = (x[n - 2] - x[n - 3]);
    res[n - 1] = (2.0 / (h1 + h2)) *
                 ((y[n - 1] - y[n - 2]) / h1 - (y[n - 2] - y[n - 3]) / h2);
  }
  if (order > 2) {
    SPDLOG_ERROR("Orders of higher degrees are not yet implemented");
    exit(EXIT_FAILURE);
  }

  return res;
}


/**
 * @brief calculates s (distance), psi (global angle w.r.t. x-axis), kappa
 * (curvature) from xy coordinates
 *
 *
 * @param pos position points in global xy-coordinates
 * @param s distance result
 * @param psi angle result
 * @param k curvature result
 * @return int success
 */
int xy2Curv(std::vector<Point2D> pos, std::vector<double>& s,
            std::vector<double>& psi, std::vector<double>& k) {

  s.clear();
  psi.clear();
  k.clear();
  
  if (pos.size() == 0) return 0;

  if (pos.size() == 1) {
    s.push_back(0);
    psi.push_back(0);
    k.push_back(0);
    
    return 0;
  }

  // calculate s from xy coordinates
  s.push_back(0);
  
  std::vector<double> x(1, pos[0].x), y(1, pos[0].y);

  // calculate s from xy coordinates
  for (int i = 1; i < pos.size(); i++) {
    double dx = pos[i].x - pos[i - 1].x;
    double dy = pos[i].y - pos[i - 1].y;
    x.push_back(pos[i].x);
    y.push_back(pos[i].y);

    s.push_back(s.back() + sqrt(pow(dx, 2) + pow(dy, 2)));
    psi.push_back(atan2(dy, dx));
  }
  psi.push_back(psi.back()); // keep psi for last entry to ensure same dimension

  // calculate psi and kappa
  std::vector<double> dxds = gradient(s, x, 1);
  std::vector<double> dyds = gradient(s, y, 1);
  std::vector<double> dxxdss = gradient(s, x, 2);
  std::vector<double> dyydss = gradient(s, y, 2);

  for (int i = 0; i < pos.size(); i++) {
    if (pos.size() > 2) {
      double top = dxds[i] * dyydss[i] - dxxdss[i] * dyds[i];
      double bot = pow(dxds[i] * dxds[i] + dyds[i] * dyds[i], 1.5);
      k.push_back(top / bot);
    } else {
      k.push_back(0.0);
    }
  }

  return 0;
}


/**
 * @brief find closest point on centerline of current lane to a point(x,y)
 *
 * @param point
 * @param cl centerline (can also be boundary line)
 * @param closest point
 * @param extrap if closest point can be also an extrapolation beyond boundaries
 * @return index of the next centerline point following the projected point. 
 *    0 when before scope of cl
 *    cl.size() when after scope of cl
 */
int closestCenterlinePoint(const Point2D point, const std::vector<Point2D>& cl,
                           Point2D& closest, bool extrap=true) {

  if (cl.size() <= 1){
    SPDLOG_ERROR("the centerline path does not have multiple entries: cl.size() = {}", cl.size());
    exit(EXIT_FAILURE);
  }

  // initialize variables
  double min_dist = INFINITY;
  int min_i;
  Point2D tmp;

  // iterate over all segments of centerline
  for (int i = 1; i < cl.size(); i++) {

    // get coordinates
    double x1 = cl[i - 1].x;
    double x2 = cl[i].x;
    double y1 = cl[i - 1].y;
    double y2 = cl[i].y;

    double dx = x2 - x1;
    double dy = y2 - y1;

    // skip if identical centerline points
    if (x1 == x2 && y1 == x2) continue;

    // compute orthogonal projection of (x,y) onto the parameterized line
    // connecting (x1,y1) and (x2,y2)
    double l2 = dx * dx + dy * dy;
    double dot = (point.x - x1) * dx + (point.y - y1) * dy;
    double t = dot / l2;

    // if in segment
    if (t >= 0 && t <= 1) {
      
      // compute point on centerline
      tmp.x = x1 + t * dx;
      tmp.y = y1 + t * dy;
      
      double dist = pow(tmp.x - point.x, 2) + pow(tmp.y - point.y, 2);

      if (dist < min_dist) {
        min_dist = dist;
        closest = tmp;
        min_i = i;
      }
    }
    // if outside of segment
    else {
      double dist_end = pow(point.x - x2, 2) + pow(point.y - y2, 2);

      if (dist_end < min_dist) {
        min_dist = dist_end;
        closest = cl[i];
        min_i = i;
      }
    }
  }

  // check if distance to start point is smaller
  double dist_start = pow(point.x - cl.front().x, 2) + pow(point.y - cl.front().y, 2);
  double dist_end = pow(point.x - cl.back().x, 2) + pow(point.y - cl.back().y, 2);

  if (dist_start < min_dist || dist_end < min_dist) {
    
    Point2D start, end;

    // if distance to start is minimal
    if (dist_start < dist_end){
      min_i = 0;

      start = cl[0];
      end = cl[1];

      closest = start;
    }

    // if distance to end is minimal
    if (dist_end < dist_start){
      min_i = cl.size();

      start = cl[cl.size() - 2];
      end = cl[cl.size() - 1];

      closest = end;
    }

    // calculate closest if extrapolation desired
    if (extrap) {
      double dx = end.x - start.x;
      double dy = end.y - start.y;


      double l2 = dx * dx + dy * dy;
      double dot = (point.x - start.x) * dx + (point.y - start.y) * dy;
      double t = dot / l2;

      closest.x = start.x + t * dx;
      closest.y = start.y + t * dy;
    }
  }

  return min_i;
}


/**
 * @brief ds along xy starting at (startX,startY) and ending at (endX,endY) with
 * correct sign
 *
 * @param start point
 * @param end point
 * @param cl centerline
 * @return resulting distance
 */
double xy2s(const Point2D start, const Point2D end,
               const std::vector<Point2D>& cl, double start_psi) {

  double s = 0;

  Point2D start_centerline, end_centerline;
  int start_idx = closestCenterlinePoint(start, cl, start_centerline);
  int end_idx = closestCenterlinePoint(end, cl, end_centerline);

  // if start and end in same interval 
  if (start_idx == end_idx) {

    start_psi = wrapAngle(start_psi);
    double start_end_psi = atan2(end.y - start.y, end.x - start.x);
    
    // check direction
    int dir = 0;
    if (abs(wrapAngle(start_end_psi - start_psi)) > M_PI/2) {
      dir = -1;
    }
    else {
      dir = 1;
    }
    
    return dir * sqrt(pow(start_centerline.x - end_centerline.x, 2) 
              + pow(start_centerline.y - end_centerline.y, 2));
  }
  else if (start_idx < end_idx) {
    for (int i = start_idx; i < end_idx-1; i++) {
      double dx = cl[i+1].x - cl[i].x;
      double dy = cl[i+1].y - cl[i].y;

      s += sqrt(dx * dx + dy * dy);
    }
    s += sqrt(pow(start_centerline.x - cl[start_idx].x, 2) 
            + pow(start_centerline.y - cl[start_idx].y, 2));
    s += sqrt(pow(end_centerline.x - cl[end_idx-1].x, 2) 
            + pow(end_centerline.y - cl[end_idx-1].y, 2));
    return s;
  }
  else if (start_idx > end_idx) {
    for (int i = end_idx; i < start_idx-1; i++) {
      double dx = cl[i+1].x - cl[i].x;
      double dy = cl[i+1].y - cl[i].y;

      s -= sqrt(dx * dx + dy * dy);
    }
    s -= sqrt(pow(start_centerline.x - cl[start_idx-1].x, 2) +
              pow(start_centerline.y - cl[start_idx-1].y, 2));
    s -= sqrt(pow(end_centerline.x - cl[end_idx].x, 2) 
            + pow(end_centerline.y - cl[end_idx].y, 2));
    return s;
  }
  else{
    return 0;
  }
}


/**
 * @brief 
 * 
 * @param y 
 * @param xy 
 * @param pos 
 * @return int 
 */
int interpolateXY2Value(std::vector<double> y, std::vector<Point2D> xy,
                        Point2D pos) {

  Point2D dummy;
  int i = closestCenterlinePoint(pos, xy, dummy, false);

  // crop idx to boundaries
  if (i == 0) i = 1;
  if (i == xy.size()) i = xy.size()-1;

  double x1 = xy[i - 1].x;
  double x2 = xy[i].x;
  double y1 = xy[i - 1].y;
  double y2 = xy[i].y;

  // compute orthogonal projection of (x,y) onto the parameterized line
  // connecting (x1,y1) and (x2,y2)
  double l2 = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
  double dot = (pos.x - x1) * (x2 - x1) + (pos.y - y1) * (y2 - y1);
  double t = dot / l2;

  return y[i - 1] + t * (y[i] - y[i - 1]);
}


/**
 * @brief Calculate approximate centerline connection for intersection area
 *
 * This function creates centerline points between two road ends. To do so,
 * two points at each road end are required. With that information, the
 * parameterized cubic polynomials x(t) and y(t) can be solved unique.
 * The ODE system for x(t) is set up as follows:
 * x(0) = x1, x(\delta x) = x2 with \delta x = abs(x2-x1)
 * x'(0) = cos(ang1), x(\delta x) = cos(ang2) where ang1 and ang2 are the angles
 * at the end of each road. With those four equations, the ODE A*c=b can be
 * solved for c
 *
 * @param p1_p2 a vector of four points that describe both road ends
 * @param pos the centerline with a gap that should be filled by this function
 * @param idx index within pos where the gap is located
 * @return length of inserted gap vector
 */

int calcGap(std::vector<Point2D> p1_p2, std::vector<Point2D>& pos, int idx) {

  // gap start/end plus second last point, respectively
  double x1 = p1_p2[1].x;
  double y1 = p1_p2[1].y;
  double x1_h = p1_p2[0].x;
  double y1_h = p1_p2[0].y;
  double x2 = p1_p2[2].x;
  double y2 = p1_p2[2].y;
  double x2_h = p1_p2[3].x;
  double y2_h = p1_p2[3].y;

  // help/intermediate variables
  double dx = abs(x2 - x1);
  double dy = abs(y2 - y1);
  double ang1 = atan2(y1 - y1_h, x1 - x1_h);
  double ang2 = atan2(y2_h - y2, x2_h - x2);

  // manual coefficient calculation after paper+pen
  // x(0) = x1, x(dx) = x2, x'(0) = cos(ang1), x'(dx) = cos(ang2)
  // y similar but with sin()
  // Remark: Those calculation are the solution to an ODE of the form A \ b = c
  double ca_x = (cos(ang1)*dx + 2*x1 - 2*x2 + dx*cos(ang2)) / pow(dx, 3);
  double cb_x = (-2*cos(ang1)*dx - 3*x1 + 3*x2 - dx*cos(ang2)) / pow(dx, 2);
  double cc_x = cos(ang1);
  double cd_x = x1;
  double ca_y = (sin(ang1)*dy + 2*y1 - 2*y2 + dy*sin(ang2)) / pow(dy, 3);
  double cb_y = (-2*sin(ang1)*dy - 3*y1 + 3*y2 - dy*sin(ang2)) / pow(dy, 2);
  double cc_y = sin(ang1);
  double cd_y = y1;

  std::vector<Point2D> gapXY;
  int N = 30;  // number of interpolated points
  double ddx = dx / (N - 1);
  double ddy = dy / (N - 1);
  double tx = 0;
  double ty = 0;
  for (int i = 0; i < N; i++) {
    Point2D tmp;
    // calculate parameter polynomial function
    tmp.x = ca_x * pow(tx, 3) + cb_x * pow(tx, 2) + cc_x * tx + cd_x;
    tmp.y = ca_y * pow(ty, 3) + cb_y * pow(ty, 2) + cc_y * ty + cd_y;

    tx += ddx;
    ty += ddy;
    // add point
    gapXY.push_back(tmp);
  }

  pos.insert(pos.begin() + idx, gapXY.begin(), gapXY.end());

  // return size of gap
  return gapXY.size();
}


/**
 * @brief calculates the offset with sign from the ego to the centerline
 * 
 * @param ego ego position
 * @param centerline centerline point
 * @param ref_angle angle of 
 * @return double 
 */
double calcSignedTOffset(Point2D ego, Point2D centerline, double ref_angle) {

  // calculate orientation from position to centerline projection
  double orientation = ref_angle - atan2(centerline.y - ego.y,
                                       centerline.x - ego.x);

  // adjust orientation
  orientation = std::atan2(std::sin(orientation), std::cos(orientation));
  int d_sig = (orientation > 0) - (orientation < 0);

  // calculate distance to centerline point
  double d = sqrt(pow(ego.x - centerline.x, 2) +
                       pow(ego.y - centerline.y, 2));
  
  return d_sig * d;
}


// --- DEPRECATED FUNCTIONS ----------------------------------------------------

/**
 * @brief create 3rd order spline connecting start and end
 * d_start, d_end contain direction vectors in Start and End point. Spline
 * points are generated in vector cl
 *
 * @param start point
 * @param end point
 * @param d_start heading at start
 * @param d_end heading at end
 * @param cl resulting spline
 *
 */
void spline3(Point2D start, Point2D end, Point2D d_start, Point2D d_end,
             std::vector<Point2D>& cl) {

  // transform into local coordinate system (rotation so that new x-axis passes
  // through start and end)
  Point2D s, e, ds, de, se, der;
  se.x = end.x - start.x;
  se.y = end.y - start.y;

  // angle between coordinate systems. Ensured that start --> end is positive
  // x-direction in new coordiante system
  double phi = (se.y > 0)
                 ? std::acos(se.x / sqrt(se.x * se.x + se.y * se.y))
                 : -1 * std::acos(se.x / sqrt(se.x * se.x + se.y * se.y));


  transform(start, s, phi);
  transform(end, e, phi);
  transform(d_start, ds, phi);
  transform(d_end, de, phi);

  // maximum derivative is set to 50 ~ 89 degrees
  der.x = abs(ds.x) > abs(ds.y / 50.0) ? ds.y / ds.x
                                       : std::copysignf(1, ds.x * ds.y) * 50;
  der.y = abs(de.x) > abs(de.y / 50.0) ? de.y / de.x
                                       : std::copysignf(1, de.x * de.y) * 50;

  // creating spline of form f(x)= a + bx + cx^2 + dx^3, der.x contains
  // f'(start.x), der.y contains f'(end.x)

  double tmp = 1.0 / (pow(s.x - e.x, 3));

  double a = -tmp * (der.y * e.x * pow(s.x, 3) + der.x * e.x * e.x * s.x * s.x -
                     der.y * e.x * e.x * s.x * s.x - der.x * pow(e.x, 3) * s.x -
                     pow(s.x, 3) * e.y + 3 * e.x * s.x * s.x * e.y -
                     3 * e.x * e.x * s.x * s.y + pow(e.x, 3) * s.y);

  double b = -tmp * (-der.y * pow(s.x, 3) - 2 * der.x * e.x * s.x * s.x -
                     der.y * e.x * s.x * s.x + der.x * e.x * e.x * s.x +
                     2 * der.y * e.x * e.x * s.x + der.x * pow(e.x, 3) +
                     6 * e.x * s.x * s.y - 6 * e.x * s.x * e.y);

  double c =
    -tmp * (der.x * s.x * s.x + 2 * der.y * s.x * s.x + der.x * e.x * s.x -
            der.y * e.x * s.x - 2 * der.x * e.x * e.x - der.y * e.x * e.x -
            3 * s.x * s.y + 3 * s.x * e.y - 3 * e.x * s.y + 3 * e.x * e.y);

  double d = -tmp * (-der.x * s.x - der.y * s.x + der.x * e.x + der.y * e.x +
                     2 * s.y - 2 * e.y);

  const float max_error = 0.035;

  // generate "centerline" points.
  double x = s.x;
  double y = 0;

  while (x < e.x) {
    double kappa = abs(2 * c + 6 * d * x);
    double delta_x =
      (kappa < 0.001)
        ? 1.0
        : 2 * sqrt(2 * max_error * 1 / kappa - max_error * max_error);
    if (2 * max_error * 1 / kappa < max_error * max_error) delta_x = 1 / kappa;
    x += delta_x;
    y = a + b * x + c * x * x + d * x * x * x;
    if (x < e.x) cl.push_back(Point2D(x, y));
  }

  // transform back

  Point2D temp;
  for (int i = 0; i < cl.size(); i++) {
    transform(cl[i], temp, -1.0 * phi);
    cl[i] = temp;
  }
}


/**
 * @brief create linear spline connecting start and end
 *
 * @param start point
 * @param end point
 * @param cl resulting spline
 *
 */
void spline1(Point2D start, Point2D end, std::vector<Point2D>& cl) {

  Point2D se(end.x - start.x, end.y - start.y);
  double norm = sqrt(se.x * se.x + se.y * se.y);

  se.x = se.x / norm;
  se.y = se.y / norm;

  if (norm < 1) {
    cl.push_back(Point2D(start.x + 0.5 * se.x, start.y + 0.5 * se.y));
    return;
  }

  for (int i = 1; i < norm; i++) {
    cl.push_back(Point2D(start.x + i * se.x, start.y + i * se.y));
  }
}