#include <gtest/gtest.h>

#include <numeric>

#include "Optimizer/SlotArray.h"
#include "Optimizer/SlotMap.h"

using namespace ArgMin;

template <typename T>
struct TypedKey : public SlotMapKeyBase {};

template <bool Direct>
void test1() {
  SlotMap<int, TypedKey<int>, std::vector<int>, Direct> map;

  auto key1 = map.insert(10);
  auto key2 = map.insert(20);
  auto key3 = map.insert(30);

  EXPECT_TRUE(*(map.at(key1)) == 10);
  EXPECT_TRUE(*(map.at(key2)) == 20);
  EXPECT_TRUE(*(map.at(key3)) == 30);

  EXPECT_EQ(map.size(), 3);

  map.erase(key2);

  EXPECT_TRUE(*(map.at(key3)) == 30);
  EXPECT_TRUE(*(map.at(key1)) == 10);
  EXPECT_EQ(map.size(), 2);

  EXPECT_EQ(map.at(key2), map.end());

  auto key4 = map.insert(20);

  EXPECT_EQ(key4.generation, 1);
  EXPECT_EQ(key4.index, key2.index);
  EXPECT_EQ(key4.index, 1);

  auto key5 = map.insert(40);

  EXPECT_EQ(map.size(), 4);
  EXPECT_EQ(key5.generation, 0);
  EXPECT_EQ(key5.index, 3);

  int i = 0;
  for (auto& e : map) {
    EXPECT_TRUE(e > 0);
    ++i;
  }
  EXPECT_EQ(i, 4);

  auto generatedKey = map.getKeyFromDataIndex(map.at(key5) - map.begin());

  EXPECT_EQ(generatedKey.index, key5.index);
  EXPECT_EQ(generatedKey.generation, key5.generation);

  map.erase(key1);
  EXPECT_EQ(map.size(), 3);

  i = 0;
  for (auto& e : map) {
    EXPECT_TRUE(e > 0);
    ++i;
  }
  EXPECT_EQ(i, map.size());
}
TEST(ContiguousSlotMap, Simple) { test1<false>(); }
TEST(DirectSlotMap, Simple) { test1<true>(); }

template <bool Direct>
void test2() {
  std::vector<int> values(1000000);
  std::iota(values.begin(), values.end(), 0);

  SlotMap<int, TypedKey<int>, std::vector<int>, Direct> map;

  std::vector<TypedKey<int>> keys;

  for (size_t idx = 0; idx < values.size(); ++idx) {
    keys.push_back(map.insert(values.at(idx)));
  }

  for (size_t idx = 0; idx < keys.size(); ++idx) {
    EXPECT_EQ(*(map.at(keys.at(idx))), values.at(idx));
  }

  values.erase(values.begin() + 100);
  map.erase(keys.at(100));
  keys.erase(keys.begin() + 100);

  for (size_t idx = 0; idx < keys.size(); ++idx) {
    EXPECT_EQ(*(map.at(keys.at(idx))), values.at(idx));
  }

  int i = 0;
  for (const auto& e : map) {
    ++i;
  }
  EXPECT_EQ(map.size(), i);
  EXPECT_EQ(map.size(), keys.size());
}
TEST(ContiguousSlotMap, Stress) { test2<false>(); }
TEST(DirectSlotMap, Stress) { test2<true>(); }

template <bool Direct>
void test3() {
  using SA = SlotArray<int, TypedKey<int>, std::vector<int>, Direct>;

  SA map;

  TypedKey<int> key1, key2, key3;
  key1.index = 1;
  key2.index = 5;
  key3.index = 3;

  EXPECT_EQ(map.insert(key1, 10), SA::SUCCESS_NO_OVERWRITE);
  EXPECT_EQ(map.insert(key3, 30), SA::SUCCESS_NO_OVERWRITE);
  map.insert(key2) = 20;

  EXPECT_TRUE(*(map.at(key1)) == 10);
  EXPECT_TRUE(*(map.at(key2)) == 20);
  EXPECT_TRUE(*(map.at(key3)) == 30);

  EXPECT_TRUE(map.size() == 3);

  map.erase(key2);

  EXPECT_TRUE(*(map.at(key3)) == 30);
  EXPECT_TRUE(*(map.at(key1)) == 10);
  EXPECT_TRUE(map.size() == 2);

  EXPECT_EQ(map.at(key2), map.end());

  TypedKey<int> key4, key5;
  key4.index = 5;
  key5.index = 3;

  EXPECT_EQ(map.insert(key4, 20), SA::SUCCESS_NO_OVERWRITE);

  EXPECT_EQ(map.insert(key5, 50), SA::SUCCESS_OVERWRITE);

  EXPECT_TRUE(map.size() == 3);

  int i = 0;
  for (auto& e : map) {
    EXPECT_TRUE(e > 0);
    ++i;
  }
  EXPECT_EQ(i, 3);

  auto generatedKey = map.getKeyFromDataIndex(map.at(key5) - map.begin());

  EXPECT_EQ(generatedKey.index, key5.index);
}
TEST(ContiguousSlotArray, Simple) { test3<false>(); }
TEST(DirectSlotArray, Simple) { test3<true>(); }

template <bool Direct>
void test4() {
  using SA = SlotArray<int, TypedKey<int>, std::vector<int>, Direct>;
  using SM = SlotMap<double, TypedKey<int>, std::vector<int>, Direct>;

  SA array;
  SM map;

  auto key1 = map.insert(1.0);
  auto key2 = map.insert(2.0);

  array.insert(key1, 1);
  array.insert(key2, 2);

  map.erase(key2);
  auto key3 = map.insert(3);
  ASSERT_EQ(key2.index, key3.index);
  EXPECT_NE(key2.generation, key3.generation);

  array.erase(key2);
  array.insert(key3, 3);

  auto generatedKey = array.getKeyFromDataIndex(array.at(key3) - array.begin());

  // Now the generated key should work with the slot map.
  EXPECT_NE(map.at(generatedKey), map.end());
}
TEST(ContiguousSlotArray, VerifyGeneration) { test4<false>(); }
TEST(DirectSlotArray, VerifyGeneration) { test4<true>(); }
