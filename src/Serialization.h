#pragma once

#include <cereal/archives/json.hpp>
#include <cereal/cereal.hpp>
#include <cereal/types/array.hpp>
#include <cereal/types/base_class.hpp>
#include <cereal/types/complex.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/utility.hpp>
#include <cereal/types/vector.hpp>

#include "DataStructures/Graph.h"

namespace cereal {

template <class Archive>
void serialize(Archive& ar, QDVO::Graph& graph) {
  std::pair<int, int> key = {graph.getCurrentFrameKey().index,
                             graph.getCurrentFrameKey().generation};

  ar& cereal::make_nvp("currentFrameKey", key);
}

}  // namespace cereal
