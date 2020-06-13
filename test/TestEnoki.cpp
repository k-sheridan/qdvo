#include <enoki/array.h>
#include <enoki/dynamic.h>
#include <enoki/matrix.h>
#include <enoki/stl.h>
#include <gtest/gtest.h>

#include <type_traits>

#include "Optimizer/BlockVector.h"
#include "Optimizer/ErrorTermBase.h"
#include "Optimizer/GaussianPrior.h"
#include "Optimizer/HuberLossFunction.h"
#include "Optimizer/Key.h"
#include "Optimizer/Marginalizer.h"
#include "Optimizer/MetaHelpers.h"
#include "Optimizer/PSDSchurSolver.h"
#include "Optimizer/SSEOptimizer.h"
#include "Optimizer/SparseBlockMatrix.h"
#include "Optimizer/SparseBlockRow.h"
#include "Optimizer/Variables/InverseDepth.h"
#include "Optimizer/Variables/SE3.h"
#include "Optimizer/Variables/SimpleScalar.h"

template <typename Value_>
struct Custom {
  using Value = Value_;
  using FloatVector = enoki::Array<Value, 3>;
  using DoubleVector = enoki::float64_array_t<FloatVector>;
  using IntVector = enoki::int32_array_t<Value>;

  FloatVector o;
  DoubleVector d;
  IntVector i = 3;

  template <typename T>
  bool operator==(const Custom<T> &other) const {
    return other.o == o && other.d == d && other.i == i;
  }

  template <typename T>
  bool operator!=(const Custom<T> &other) const {
    return !operator==(other);
  }

  bool testFn1() { return true; }
  bool testFn2() { return i == 3; }

  ENOKI_STRUCT(Custom, o, d, i)
};
ENOKI_STRUCT_SUPPORT(Custom, o, d, i)

TEST(Enoki, CustomArray) {
  using FloatP = enoki::Packet<float>;
  using CustomP = Custom<FloatP>;
  using FloatX = enoki::DynamicArray<FloatP>;

  CustomP a;
  a.o = 2;

  for (int i = 0; i < enoki::slices(a); ++i) {
    auto &&ref = enoki::slice(a, i);
    EXPECT_EQ(ref.i, 3);
    EXPECT_TRUE(ref.testFn1());
    EXPECT_TRUE(ref.testFn2());
  }

  using Vector3dP = enoki::Array<Eigen::Vector3d, 4>;
  Vector3dP vecs;
  vecs[0] = {0, 0, 0};
  vecs[1] = {1, 2, 3};
  vecs[2] = {1, 3, 3};
  vecs[3] = {1, 4, 3};

  Vector3dP vecs2 = vecs;
  EXPECT_EQ(vecs[2].y(), 3);

  auto sum = enoki::hsum(vecs);
  std::cout << typeid(sum).name() << std::endl;
  EXPECT_EQ(sum[0], 3);
  EXPECT_EQ(sum[1], 9);
  EXPECT_EQ(sum[2], 9);
}

template <typename Value>
struct MyData {
  using FloatA = enoki::float32_array_t<Value>;
  using FloatB = enoki::float32_array_t<Value>;
  using FloatC = enoki::float32_array_t<Value>;
  FloatA a = 1;
  FloatB b = 2;
  FloatC c = 0;

  void fn() { c = (a * b); }

  ENOKI_STRUCT(MyData, a, b, c)
};
ENOKI_STRUCT_SUPPORT(MyData, a, b, c)

TEST(Enoki, DynamicArray) {
  MyData<enoki::DynamicArray<enoki::Packet<float>>> dynamicData;

  enoki::set_slices(dynamicData, 100000);
  EXPECT_EQ(enoki::slices(dynamicData), 100000);

  for (int i = 0; i < enoki::packets(dynamicData); ++i) {
    auto &&slice = enoki::packet(dynamicData, i);
    slice.b = 3;
  }

  EXPECT_EQ(enoki::slices(dynamicData), 100000);

  for (int i = 0; i < enoki::packets(dynamicData); ++i) {
    auto &&packet = enoki::packet(dynamicData, i);
    packet.fn();
  }

  EXPECT_EQ(enoki::slices(dynamicData), 100000);

  for (int i = 0; i < enoki::slices(dynamicData); ++i) {
    auto &&slice = enoki::slice(dynamicData, i);
    EXPECT_EQ(slice.c, 3);
  }

  EXPECT_EQ(enoki::slices(dynamicData), 100000);

  std::vector<MyData<float>> data;
  data.resize(100000);
  for (int i = 0; i < enoki::slices(dynamicData); ++i) {
    auto &&slice = enoki::slice(dynamicData, i);
    data.push_back(slice);
  }
}

struct RandomData {};

template <typename Value>
struct RealData {
  using EigenM = enoki::replace_scalar_t<Value, Eigen::Matrix<double, 3, 6>>;
  using Ptr = enoki::replace_scalar_t<Value, RandomData *>;
  using FloatA = enoki::float32_array_t<Value>;
  using FloatB = enoki::float32_array_t<Value>;
  using FloatC = enoki::float32_array_t<Value>;
  FloatA a = 1;
  FloatB b = 2;
  FloatC c = 0;
  EigenM m;
  MyData<Value> d;
  Ptr p;

  void fn() { c = (a * b); }

  void fn2() {
    std::cout << Value::Size << std::endl;
    std::cout << sizeof(EigenM) << std::endl;
    for (int i = 0; i < EigenM::Size; ++i) {
      m[i] = m[i] * c[i];
    }

    d.a = c * 2;
  }

  ENOKI_STRUCT(RealData, a, b, c, m, d, p)
};
ENOKI_STRUCT_SUPPORT(RealData, a, b, c, m, d, p)

namespace enoki {
namespace detail {
template <typename T, int R, int C>
struct scalar<Eigen::Matrix<T, R, C>> {
  using type = T;
};
}  // namespace detail
}  // namespace enoki

TEST(Enoki, EigenMatrix) {
  using M_eigen = Eigen::Matrix<double, 3, 6>;

  std::cout << sizeof(enoki::scalar_t<M_eigen>) << std::endl;

  RealData<enoki::Array<float, 11>> arr;
  // RealData<float> arr;

  arr.fn();
  arr.fn2();

  arr.c = 10;
  EXPECT_EQ(arr.c.size(), 11);
  EXPECT_EQ(arr.c, 10);

  enoki::Packet<float> p;
  std::cout << p.size() << std::endl;

  enoki::DynamicArray<enoki::Packet<M_eigen, 1>> dArr;

  // enoki::set_slice(dArr, 10);
  enoki::packet(dArr, 0);

  for (int i = 0; i < dArr.size(); ++i) {
    auto &&s = slice(dArr, i);
    s.llt();
  }

  enoki::vectorize([](auto &&m) { std::cout << m << std::endl; }, dArr);
}
