#ifndef DEQUEARRAY_H
#define DEQUEARRAY_H

#include <deque>
#include <Eigen/Core>

namespace QDVO {

/*
 * this datastructure is meant to help searching for nearest potential correspondences in QDVO.
 * This deque array will keep track of which pixels have been evaluated using the chosen patch comparison metric to avoid duplicate operations.
 * This is meant to be used in combination.
 *
 * The main down side to this datastructure is that it suffers from some random access overhead, but it still has constant time random access.
 * The main benefit to this datastructure is that, for its task, it is very memory eficient because it will only allocate more memory if the search extends beyond
 * the current border.
 */
template<typename T>
class DequeArray
{

public:

    struct PixelState {
      bool searched = false;
      T data;
    };

    DequeArray();

    DequeArray(const unsigned width, const unsigned height);

    /*
     * gets a
     */
    PixelState& get(const Eigen::Vector2i& pixel){
        return this->array.at(centerIndex(1)).at(centerIndex(0));
    }

    Eigen::Vector2i pixel2Index(const Eigen::Vector2i& pixel);

private:
    Eigen::Vector2f centerIndex; // the deque array space center coordinate (col, row) / (x, y) !!!!!!!.
    Eigen::Vector2i centerPixel; // the image space pixel which the deque array is built around.
    std::deque<std::deque<PixelState> > array;

};

template <typename T>
DequeArray<T>::DequeArray(){}

template <typename T>
DequeArray<T>::DequeArray(const unsigned width, const unsigned height)
{
    this->array.resize(height, std::deque<DequeArray<T>::PixelState>(width));

    // choose a center index
    //this->centerIndex = Eigen::Vector2i(width/2, height/2);
}

template <typename T>
Eigen::Vector2i DequeArray<T>::pixel2Index(const Eigen::Vector2i& pixel)
{
    return (pixel - this->centerPixel) + this->centerIndex;
}



}



#endif // DEQUEARRAY_H
