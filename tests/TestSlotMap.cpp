#include "gtest/gtest.h"
#include "Key.h"
#include "SlotMap.h"
#include "Variables/SE3.h"
#include "Variables/InverseDepth.h"
#include <type_traits>

TEST(SlotMap, TemplateKey) {

    SlotMap<SE3, ArgMin::VariableKey<SE3>> map;
}

