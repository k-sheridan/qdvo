#pragma once

#include "Types.h"
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Lie.h>

#include <cmath>
#include <iostream>


namespace QDVO {

struct InverseDepth {
  QDVO::Scalar dinv;
  
  InverseDepth(QDVO::Scalar dinv) : dinv(dinv) {}
};

} // namespace QDVO


/** 
 * traits for InverseDepth
 *
 * Any type compatible with GTSAM mush have (in its traits):
 *   - Print function with an optional begining string, in format:
 *     static void Print(const T& m, const std::string& str = "");
 *   - Equal function with optional tolerance, in format:
 *     static bool Equals(const T& m1, const T& m2, double tol = 1e-8);
 *
 * A manifold type must have: 
 *   - int dimension (not fully support dynamic dimensionality yet)
 *   - Typedefs TangentVector, where TangentVector = Eigen::Matrix<double, dimension, 1>
 *   - Local coordinate function, in format:
 *     static TangentVector Local(const Class& origin, const Class& other);
 *   - Retraction back to manifold, in format:
 *     static Class Retract(const Class& origin, const TangentVector& v);
 *
 * A lie group types must have:
 *   - Identity function
 *   - Logmap function, with optional jacobians
 *   - Expmap function, with optional jacobians
 *   - Compose function, with optional jacobians
 *   - Between function, with optional jacobians
 *   - Inverse function, with optional jacobians
 *
 * For lie group types, other than traits, operator * or (+ and -) should be defined
 * for compose / between operation. Can be defined inside or outside class.
 * In this example we defined operator * outside class in the end
 */


// traits must in namespace gtsam
namespace gtsam {

template<>
struct traits<QDVO::InverseDepth> {

  // strcutural category: this is a lie group
  // avaible options: manifold_tag, group_tag, lie_group_tag
  typedef lie_group_tag structure_category;

  /**
   * Basic (Testable)
   */
   
  // print
  static void Print(const QDVO::InverseDepth& m, const std::string& str = "") {
    std::cout << str << "(" << m.dinv << ")" << std::endl;
  }
  
  // equality with optional tol
  static bool Equals(const QDVO::InverseDepth& m1, const QDVO::InverseDepth& m2, 
      double tol = 1e-8) {
    if (fabs(m1.dinv - m2.dinv) < tol)
      return true;
    else
      return false;
  }

  /**
   * Manifold
   */

  // use enum dimension
  enum { dimension = 1 };
  static int GetDimension(const QDVO::InverseDepth&) { return dimension; }
  
  // Typedefs needed
  typedef QDVO::InverseDepth ManifoldType;
  typedef Eigen::Matrix<double, dimension, 1> TangentVector;
  
  // Local coordinate of InverseDepth is naive (since vectorspace)
  static TangentVector Local(const QDVO::InverseDepth& origin, 
      const QDVO::InverseDepth& other) {
    return TangentVector(other.dinv - origin.dinv);
  }
  
  // Retraction back to manifold of InverseDepth is naive (since vectorspace)
  static QDVO::InverseDepth Retract(const QDVO::InverseDepth& origin, 
      const TangentVector& v) {
    return QDVO::InverseDepth(std::max(origin.dinv + v(0), 0.0));
  }

  /**
   * Lie group
   */
  
  // indicate this group using operator *, 
  // if uses +/- then use option additive_group_tag 
  typedef multiplicative_group_tag group_flavor;
   
  // typedefs
  typedef OptionalJacobian<1, 1> ChartJacobian;
  
  static QDVO::InverseDepth Identity() { 
    return QDVO::InverseDepth(0);
  }
  
  static TangentVector Logmap(const QDVO::InverseDepth& m, 
      ChartJacobian Hm = boost::none) {
    if (Hm) *Hm << 1; 
    return TangentVector(m.dinv);
  }

  static QDVO::InverseDepth Expmap(const TangentVector& v, 
      ChartJacobian Hv = boost::none) {
    if (Hv) *Hv << 1; 
    return QDVO::InverseDepth(v(0));
  }

  static QDVO::InverseDepth Compose(const QDVO::InverseDepth& m1, 
      const QDVO::InverseDepth& m2,
      ChartJacobian H1 = boost::none, ChartJacobian H2 = boost::none) {
    if (H1) *H1 << 1;
    if (H2) *H2 << 1; 

    return QDVO::InverseDepth(std::max(m1.dinv + m2.dinv, 0.0));
  }

  static QDVO::InverseDepth Between(const QDVO::InverseDepth& m1, 
      const QDVO::InverseDepth& m2, //
      ChartJacobian H1 = boost::none, ChartJacobian H2 = boost::none) {
    if (H1) *H1 << -1;
    if (H2) *H2 << 1; 
    return QDVO::InverseDepth(m2.dinv - m1.dinv);
  }

  static QDVO::InverseDepth Inverse(const QDVO::InverseDepth& m, //
      ChartJacobian H = boost::none) {
    if (H) *H << -1; 
    return QDVO::InverseDepth(-m.dinv);
  }
};

} // namespace gtsam


namespace QDVO {

  // operator *
  InverseDepth operator*(const InverseDepth& m1, const InverseDepth& m2) {
    return InverseDepth(m1.dinv + m2.dinv);
  }
}
