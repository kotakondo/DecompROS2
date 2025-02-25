/**
 * @file line_segment.h
 * @brief LineSegment Class
 */
#ifndef LINE_SEGMENT_H
#define LINE_SEGMENT_H

#include <decomp_util/decomp_base.h>
#include <decomp_geometry/geometric_utils.h>

/**
 * @brief Line Segment Class
 *
 * The basic element in EllipsoidDecomp
 */
template <int Dim>
class LineSegment : public DecompBase<Dim>
{
public:
  /// Simple constructor
  LineSegment(){};
  /**
   * @brief Basic constructor
   * @param p1 One end of the line seg
   * @param p2 The other end of the line seg
   */
  LineSegment(const Vecf<Dim> &p1, const Vecf<Dim> &p2) : p1_(p1), p2_(p2)
  {
  }

  /**
   * @brief Infalte the line segment
   * @param radius the offset added to the long semi-axis
   */
  void dilate(decimal_t radius)
  {
    find_ellipsoid(radius);
    this->find_polyhedron();
    add_local_bbox(this->polyhedron_);
  }

  /**
   * @brief Infalte the line segment
   * @param radius the offset added to the long semi-axis
   */
  void dilate(decimal_t radius, bool &result)
  {
    find_ellipsoid(radius, result);
    this->find_polyhedron();
    add_local_bbox(this->polyhedron_);
  }

  void set_inflate_distance(double d)
  {
    inflate_distance_ = d;
  }

  /// Get the line
  vec_Vecf<Dim> get_line_segment() const
  {
    vec_Vecf<Dim> line;
    line.push_back(p1_);
    line.push_back(p2_);
    return line;
  }

protected:
  /// Add the bounding box
  void add_local_bbox(Polyhedron<Dim> &Vs)
  {
    if (this->local_bbox_.norm() == 0)
      return;
    //**** virtual walls parallel to path p1->p2
    Vecf<Dim> dir = (p2_ - p1_).normalized();
    Vecf<Dim> dir_h = Vecf<Dim>::Zero();
    dir_h(0) = dir(1), dir_h(1) = -dir(0);
    if (dir_h.norm() == 0)
    {
      if (Dim == 2)
        dir_h << -1, 0;
      else
        dir_h << -1, 0, 0;
    }
    dir_h = dir_h.normalized();

    // along x
    Vecf<Dim> pp1 = p1_ + dir_h * this->local_bbox_(1);
    Vecf<Dim> pp2 = p1_ - dir_h * this->local_bbox_(1);
    Vs.add(Hyperplane<Dim>(pp1, dir_h));
    Vs.add(Hyperplane<Dim>(pp2, -dir_h));

    // along y
    Vecf<Dim> pp3 = p2_ + dir * this->local_bbox_(0);
    Vecf<Dim> pp4 = p1_ - dir * this->local_bbox_(0);
    Vs.add(Hyperplane<Dim>(pp3, dir));
    Vs.add(Hyperplane<Dim>(pp4, -dir));

    // along z
    if (Dim > 2)
    {
      Vecf<Dim> dir_v;
      dir_v(0) = dir(1) * dir_h(2) - dir(2) * dir_h(1);
      dir_v(1) = dir(2) * dir_h(0) - dir(0) * dir_h(2);
      dir_v(2) = dir(0) * dir_h(1) - dir(1) * dir_h(0);
      Vecf<Dim> pp5 = p1_ + dir_v * this->local_bbox_(2);
      Vecf<Dim> pp6 = p1_ - dir_v * this->local_bbox_(2);
      Vs.add(Hyperplane<Dim>(pp5, dir_v));
      Vs.add(Hyperplane<Dim>(pp6, -dir_v));
    }
  }

  /// Find ellipsoid in 2D
  template <int U = Dim>
  typename std::enable_if<U == 2>::type find_ellipsoid(double offset_x)
  {
    const decimal_t f = (p1_ - p2_).norm() / 2;
    Matf<Dim, Dim> C = f * Matf<Dim, Dim>::Identity();
    Vecf<Dim> axes = Vecf<Dim>::Constant(f);
    C(0, 0) += offset_x;
    axes(0) += offset_x;

    if (axes(0) > 0)
    {
      double ratio = axes(1) / axes(0);
      axes *= ratio;
      C *= ratio;
    }

    const auto Ri = vec2_to_rotation(p2_ - p1_);
    C = Ri * C * Ri.transpose();

    Ellipsoid<Dim> E(C, (p1_ + p2_) / 2);

    auto obs = E.points_inside(this->obs_);

    auto obs_inside = obs;
    //**** decide short axes
    while (!obs_inside.empty())
    {
      const auto pw = E.closest_point(obs_inside);
      Vecf<Dim> p = Ri.transpose() * (pw - E.d());  // to ellipsoid frame
      if (p(0) < axes(0))
        axes(1) = std::abs(p(1)) / std::sqrt(1 - std::pow(p(0) / axes(0), 2));
      Matf<Dim, Dim> new_C = Matf<Dim, Dim>::Identity();
      new_C(0, 0) = axes(0);
      new_C(1, 1) = axes(1);
      E.C_ = Ri * new_C * Ri.transpose();

      vec_Vecf<Dim> obs_new;
      for (const auto &it : obs_inside)
      {
        if (1 - E.dist(it) > epsilon_)
          obs_new.push_back(it);
      }
      obs_inside = obs_new;
    }

    this->ellipsoid_ = E;
  }

  template <typename T>
  int sgn(T val)
  {
    return (T(0) < val) - (val < T(0));
  }

  /// Find ellipsoid in 3D
  template <int U = Dim>
  typename std::enable_if<U == 3>::type find_ellipsoid(double offset_x)
  {
    const decimal_t f = (p1_ - p2_).norm() / 2;
    Matf<Dim, Dim> C = f * Matf<Dim, Dim>::Identity();
    Vecf<Dim> axes = Vecf<Dim>::Constant(f);
    C(0, 0) += offset_x;
    axes(0) += offset_x;

    if (axes(0) > 0)
    {
      double ratio = axes(1) / axes(0);
      axes *= ratio;
      C *= ratio;
    }

    const auto Ri = vec3_to_rotation(p2_ - p1_);
    C = Ri * C * Ri.transpose();

    Ellipsoid<Dim> E(C, (p1_ + p2_) / 2);
    auto Rf = Ri;

    ////// Let's inflate the obstacles now: Substitute all the points in obs_ by the nearest vertex of the cube centered
    /// at that point and paralell to the axis of the ellipsoid
    for (auto &it : this->obs_)
    {
      // std::cout << "Inflating the obstacles!!" << std::endl;
      Vecf<Dim> p = Ri.transpose() * (it - E.d());  // To Ellipsoid frame
      Vecf<Dim> tmp;                                // New Point in Ellipsoid frame

      tmp(0) = p(0) - sgn(p(0)) * inflate_distance_;
      tmp(1) = p(1) - sgn(p(1)) * inflate_distance_;
      tmp(2) = p(2) - sgn(p(2)) * inflate_distance_;
      it = Ri * tmp + E.d();  // Substitute previous point by the new Point in World frame
    }
    ///// Obstacles inflated

    auto obs = E.points_inside(this->obs_);
    auto obs_inside = obs;
    //**** decide short axes

    // This while loop could get stuck if the obstacles are too close to the line segment
    while (!obs_inside.empty())
    {
      const auto pw = E.closest_point(obs_inside);
      Vecf<Dim> p = Ri.transpose() * (pw - E.d());  // to ellipsoid frame
      const decimal_t roll = atan2(p(2), p(1));
      Rf = Ri * Quatf(cos(roll / 2), sin(roll / 2), 0, 0);
      p = Rf.transpose() * (pw - E.d());

      if (p(0) < axes(0))
        axes(1) = std::abs(p(1)) / std::sqrt(1 - std::pow(p(0) / axes(0), 2));
      Matf<Dim, Dim> new_C = Matf<Dim, Dim>::Identity();
      new_C(0, 0) = axes(0);
      new_C(1, 1) = axes(1);
      new_C(2, 2) = axes(1);
      E.C_ = Rf * new_C * Rf.transpose();

      vec_Vecf<Dim> obs_new;
      for (const auto &it : obs_inside)
      {
        if (1 - E.dist(it) > epsilon_)
          obs_new.push_back(it);
      }
      obs_inside = obs_new;
    }

    //**** reset ellipsoid with old axes(2)
    C = f * Matf<Dim, Dim>::Identity();
    C(0, 0) = axes(0);
    C(1, 1) = axes(1);
    C(2, 2) = axes(2);
    E.C_ = Rf * C * Rf.transpose();
    obs_inside = E.points_inside(obs);

    while (!obs_inside.empty())
    {
      const auto pw = E.closest_point(obs_inside);
      Vec3f p = Rf.transpose() * (pw - E.d());
      decimal_t dd = 1 - std::pow(p(0) / axes(0), 2) - std::pow(p(1) / axes(1), 2);
      if (dd > epsilon_)
        axes(2) = std::abs(p(2)) / std::sqrt(dd);
      Matf<Dim, Dim> new_C = Matf<Dim, Dim>::Identity();
      new_C(0, 0) = axes(0);
      new_C(1, 1) = axes(1);
      new_C(2, 2) = axes(2);
      E.C_ = Rf * new_C * Rf.transpose();

      vec_Vecf<Dim> obs_new;
      for (const auto &it : obs_inside)
      {
        if (1 - E.dist(it) > epsilon_)
          obs_new.push_back(it);
      }
      obs_inside = obs_new;
    }

    this->ellipsoid_ = E;
  }

  /// Find ellipsoid in 3D
  template <int U = Dim>
  typename std::enable_if<U == 3>::type find_ellipsoid(double offset_x, bool &result)
  {
    const decimal_t f = (p1_ - p2_).norm() / 2;
    Matf<Dim, Dim> C = f * Matf<Dim, Dim>::Identity();
    Vecf<Dim> axes = Vecf<Dim>::Constant(f);
    C(0, 0) += offset_x;
    axes(0) += offset_x;

    if (axes(0) > 0)
    {
      double ratio = axes(1) / axes(0);
      axes *= ratio;
      C *= ratio;
    }

    const auto Ri = vec3_to_rotation(p2_ - p1_);
    C = Ri * C * Ri.transpose();

    Ellipsoid<Dim> E(C, (p1_ + p2_) / 2);
    auto Rf = Ri;

    ////// Let's inflate the obstacles now: Substitute all the points in obs_ by the nearest vertex of the cube centered
    /// at that point and paralell to the axis of the ellipsoid
    for (auto &it : this->obs_)
    {
      // std::cout << "Inflating the obstacles!!" << std::endl;
      Vecf<Dim> p = Ri.transpose() * (it - E.d());  // To Ellipsoid frame
      Vecf<Dim> tmp;                                // New Point in Ellipsoid frame

      tmp(0) = p(0) - sgn(p(0)) * inflate_distance_;
      tmp(1) = p(1) - sgn(p(1)) * inflate_distance_;
      tmp(2) = p(2) - sgn(p(2)) * inflate_distance_;
      it = Ri * tmp + E.d();  // Substitute previous point by the new Point in World frame
    }
    ///// Obstacles inflated

    auto obs = E.points_inside(this->obs_);
    auto obs_inside = obs;
    //**** decide short axes

    // This while loop could get stuck if the obstacles are too close to the line segment
    int counter1 = 0;
    while (!obs_inside.empty())
    {
      const auto pw = E.closest_point(obs_inside);
      Vecf<Dim> p = Ri.transpose() * (pw - E.d());  // to ellipsoid frame
      const decimal_t roll = atan2(p(2), p(1));
      Rf = Ri * Quatf(cos(roll / 2), sin(roll / 2), 0, 0);
      p = Rf.transpose() * (pw - E.d());

      if (p(0) < axes(0))
        axes(1) = std::abs(p(1)) / std::sqrt(1 - std::pow(p(0) / axes(0), 2));
      Matf<Dim, Dim> new_C = Matf<Dim, Dim>::Identity();
      new_C(0, 0) = axes(0);
      new_C(1, 1) = axes(1);
      new_C(2, 2) = axes(1);
      E.C_ = Rf * new_C * Rf.transpose();

      vec_Vecf<Dim> obs_new;
      for (const auto &it : obs_inside)
      {
        if (1 - E.dist(it) > epsilon_)
          obs_new.push_back(it);
      }
      obs_inside = obs_new;
      
      if (counter1 > counter_max_)
      {
        result = false;
        return;
      }

      counter1++;
    }

    //**** reset ellipsoid with old axes(2)
    C = f * Matf<Dim, Dim>::Identity();
    C(0, 0) = axes(0);
    C(1, 1) = axes(1);
    C(2, 2) = axes(2);
    E.C_ = Rf * C * Rf.transpose();
    obs_inside = E.points_inside(obs);

    int counter2 = 0;
    while (!obs_inside.empty())
    {
      const auto pw = E.closest_point(obs_inside);
      Vec3f p = Rf.transpose() * (pw - E.d());
      decimal_t dd = 1 - std::pow(p(0) / axes(0), 2) - std::pow(p(1) / axes(1), 2);
      if (dd > epsilon_)
        axes(2) = std::abs(p(2)) / std::sqrt(dd);
      Matf<Dim, Dim> new_C = Matf<Dim, Dim>::Identity();
      new_C(0, 0) = axes(0);
      new_C(1, 1) = axes(1);
      new_C(2, 2) = axes(2);
      E.C_ = Rf * new_C * Rf.transpose();

      vec_Vecf<Dim> obs_new;
      for (const auto &it : obs_inside)
      {
        if (1 - E.dist(it) > epsilon_)
          obs_new.push_back(it);
      }
      obs_inside = obs_new;
      
      if (counter2 > counter_max_)
      {
        result = false;
        return;
      }

      counter2++;
    }

    this->ellipsoid_ = E;

    result = true;

  }

  /// One end of line segment, input
  Vecf<Dim> p1_;
  /// The other end of line segment, input
  Vecf<Dim> p2_;

  double inflate_distance_ = 0;
  int counter_max_ = 50;
};

typedef LineSegment<2> LineSegment2D;

typedef LineSegment<3> LineSegment3D;
#endif
