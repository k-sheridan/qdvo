#pragma once

#include <Eigen/Core>
#include <iostream>
#include <vector>

#define DEFAULT_BIN_WIDTH 1

namespace QDVO {

template <typename T>
class GenericQuadTree {
 public:
  struct Quad {
    Quad* parentQuad =
        nullptr;  // pointer to this quad's parent. If null, this is the root.

    // all children quadrants.
    std::vector<std::unique_ptr<Quad> >
        quadrants;  // standard unit circle quadrant layout

    unsigned nChildren =
        0;  // stores the total number of leafs (T) in all children quads.

    unsigned level = 0;  // the level of this quad.

    T data;

    Eigen::Vector2i center,
        dimensions;  // center point of quad, and dimensions of the quad.

    Quad() {
      // ensures that the quadrant finder is always given 4 quadrants, even if
      // they
      this->quadrants.resize(4);

      // assert(this->quadrants[0] == nullptr && this->quadrants[1] == nullptr
      // && this->quadrants[2] == nullptr && this->quadrants[3] == nullptr);
    }

    /// Recursively get all children.
    void getChildren(std::vector<T>& children) {
      if (nChildren == 0) {
        // insert the data.
        children.push_back(data);
      } else {
        for (auto& quad : quadrants) {
          if (quad != nullptr) {
            // recurse.
            quad->getChildren(children);
          }
        }
      }
    }

    /*
     * gets the quad at the desired level. If the level is < 0, then the lowest
     * level is returned.
     */
    Quad* getQuad(const Eigen::Vector2i& pos, int level = -1) {
      bool bottomFound = false;
      Quad* currentQuad = this;

      assert(parentQuad == nullptr);  // must start from the top level
      unsigned currentLevel = 0;

      while (!bottomFound) {
        // Get the index of the quadrant this point lies in.
        const int quadrantIdx = currentQuad->quadrant(pos);

        // If the quadrant is a null ptr, then this is the bottom of the tree
        // for this branch.
        Quad* quadPtr = currentQuad->quadrants.at(quadrantIdx).get();

        // std::cout << quadPtr << std::endl;

        if (quadPtr == nullptr || (level >= 0 && currentLevel == level)) {
          bottomFound = true;
          break;
        }

        currentQuad = quadPtr;
        currentLevel++;
      }

      return currentQuad;
    }

    /*
     * determines the quadrant, (0, 1, 2, 3).
     */
    unsigned quadrant(const Eigen::Vector2i& pos) {
      if (pos(0) >= this->center(0)) {
        if (pos(1) >= this->center(1)) {
          return 0;
        } else {
          return 3;
        }
      } else {
        if (pos(1) >= this->center(1)) {
          return 1;
        } else {
          return 2;
        }
      }
    }

    /*
     * adds a new quadrant (if it does not already exist) and sets it up.
     */
    Quad* insertQuad(const unsigned quadrant) {
      if (this->quadrants.at(quadrant) == nullptr) {
        // create a new quad at the specified quadrant.
        this->quadrants[quadrant] = std::unique_ptr<Quad>(new Quad());

        // set up the quad.
        this->setupChildQuad(this->quadrants[quadrant].get(), quadrant);

        // set the parent of the quad
        this->quadrants[quadrant]->parentQuad = this;

        // increment the level counter
        this->quadrants[quadrant]->level = this->level + 1;
      }

      return this->quadrants[quadrant].get();
    }

    /*
     * sets up the quad using the paramenters of the parent quad.
     */
    void setupChildQuad(Quad* quad, const unsigned quadrant) {
      if (quadrant == 0) {
        // 1, 1
        quad->center = this->center + this->dimensions / 4;
        quad->dimensions = this->dimensions / 2;
      } else if (quadrant == 1) {
        // -1, 1
        quad->center =
            this->center +
            Eigen::Vector2i(-this->dimensions(0), this->dimensions(1)) / 4;
        quad->dimensions = this->dimensions / 2;
      } else if (quadrant == 2) {
        // -1, -1
        quad->center = this->center - this->dimensions / 4;
        quad->dimensions = this->dimensions / 2;
      } else {
        // 1, -1
        quad->center =
            this->center +
            Eigen::Vector2i(this->dimensions(0), -this->dimensions(1)) / 4;
        quad->dimensions = this->dimensions / 2;
      }
    }
  };

  // default constructor
  GenericQuadTree();

  /**
   * Constructs a quadtree with the desired size.
   */
  GenericQuadTree(unsigned width, unsigned height,
                  unsigned desiredBinWidth = DEFAULT_BIN_WIDTH);

  /**
   * adds data at the level determined based on the desired resolution.
   */
  void insert(const Eigen::Vector2i& pos, T& data);

  /**
   * Find the lowest quad which contains data.
   * If the max level is -1, The search is as deep as possible.
   */
  Quad& getLowestQuad(const Eigen::Vector2i& pos, int maxLevel = -1) {
    Quad* result = root->getQuad(pos, maxLevel);
    if (result == nullptr) {
      return *root;
    }
    return *result;
  }

  /**
   * Try to find a quadrant at or above the passed in quadrant which contains at
   * least the number of children desried.
   */
  Quad& getParentQuadWithAtLeastNChildren(Quad& startQuad, int nChildren) {
    Quad* quad = &startQuad;
    while (quad != nullptr) {
      // If there are enough children, return this quad.
      if (quad->nChildren >= nChildren) {
        return *quad;
      }
      // If the parent is null return the best result.
      if (quad->parentQuad == nullptr) {
        return *quad;
      }
      // Try the parent.
      quad = quad->parentQuad;
    }
    return startQuad;
  }

  /// Get approximate neighbors
  /// @param pos search around this point.
  /// @param n number of neighbors to find.
  /// @return vector of all neighbors.
  std::vector<T> getApproximateNeighbors(const Eigen::Vector2i& pos, int n) {
    std::vector<T> result;
    getParentQuadWithAtLeastNChildren(getLowestQuad(pos), n)
        .getChildren(result);
    return result;
  }

  /// Get  neighbors
  /// @param pos search around this point.
  /// @param level The level where the search will be performed.
  /// @param n number of neighbors to find.
  /// @return vector of all neighbors.
  std::vector<T> getNeighbors(const Eigen::Vector2i& pos, int level, int n) {
    std::vector<T> result;

    // Compute the search increment at level n.
    double buckets = std::pow(2, level);
    double widthIncrement = (root->dimensions(0) / buckets);
    double heightIncrement = (root->dimensions(1) / buckets);

    auto isPixelInTree = [&](const Eigen::Vector2i& pos) -> bool {
      if (pos(0) < 0 || pos(1) < 0 || pos(0) >= root->dimensions(0) ||
          pos(1) >= root->dimensions(1)) {
        return false;
      }
      return true;
    };

    auto searchPoint = [&](const Eigen::Vector2i& pos) {
      Quad& quad = getLowestQuad(pos, level);
      // If this search was a success append the quads' children.
      if (quad.level == level) {
        quad.getChildren(result);
      }
    };

    auto searchLine = [&](int start, int stop, bool horizontal, int radius) {
      for (int i = start; i <= stop; ++i) {
        Eigen::Vector2i testPoint;
        if (horizontal) {
          testPoint =
              (Eigen::Vector2d(i, radius) * widthIncrement).cast<int>() + pos;
        } else {
          testPoint =
              (Eigen::Vector2d(radius, i) * heightIncrement).cast<int>() + pos;
        }

        // Search if necessary.
        if (isPixelInTree(testPoint)) {
          searchPoint(testPoint);
        }
      }
    };

    // First search the center point.
    searchPoint(pos);
    // Search
    for (int radius = 1; radius <= buckets; ++radius) {
      searchLine(-radius, radius, true, radius);
      searchLine(-radius, radius, true, -radius);
      searchLine(-radius + 1, radius - 1, false, radius);
      searchLine(-radius + 1, radius - 1, false, -radius);
      // Return early if we found enough points.
      if (result.size() >= n) {
        return result;
      }
    }

    return result;
  }

  /**
   * deletes the whole quadtree.
   */
  void clear();

  std::unique_ptr<Quad> root;

  unsigned bottomLevel = 1;
};

template <typename T>
GenericQuadTree<T>::GenericQuadTree() {}

template <typename T>
GenericQuadTree<T>::GenericQuadTree(unsigned width, unsigned height,
                                    unsigned desiredBinWidth) {
  this->root = std::unique_ptr<Quad>(new Quad());
  this->root->center = Eigen::Vector2i(round(width / 2), round(height / 2));

  // find an n where 2^n >= max(width, height)
  const unsigned dim =
      std::pow(2, std::ceil(std::log2(std::max(width, height))));

  this->root->dimensions = Eigen::Vector2i(dim, dim);

  // a / 2^(n) <= w
  // a <= w * 2^n
  // a/w <= 2^n
  // std::ceil(log2(a/w)) = n
  this->bottomLevel =
      std::ceil(std::log2(std::max(dim, dim) / desiredBinWidth));

  this->root->level = 0;
}

template <typename T>
void GenericQuadTree<T>::insert(const Eigen::Vector2i& pos, T& data) {
  // start inserting the data into the quadtree
  bool finishedInsertion = false;
  Quad* currentQuad = this->root.get();
  unsigned nextQuadrant;

  // increment the children counter for the top quad
  currentQuad->nChildren++;

  while (!finishedInsertion) {
    if (currentQuad->level == this->bottomLevel) {
      // finished
      currentQuad->data = (data);

      finishedInsertion = true;  // not necessary

      // Remove one child.
      currentQuad->nChildren--;

      break;
    }

    // get the quadrant of the point
    nextQuadrant = currentQuad->quadrant(pos);

    // insert a new quad if necessary
    currentQuad = currentQuad->insertQuad(nextQuadrant);

    // increment child counter of the current quad.
    currentQuad->nChildren++;
  }
}

template <typename T>
void GenericQuadTree<T>::clear() {
  this->root->nChildren = 0;
  this->root->quadrants.clear();  // should delete/free the subquads.
  this->root->quadrants.resize(4);
}

}  // namespace QDVO
