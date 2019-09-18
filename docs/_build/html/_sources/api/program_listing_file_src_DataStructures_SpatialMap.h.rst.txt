
.. _program_listing_file_src_DataStructures_SpatialMap.h:

Program Listing for File SpatialMap.h
=====================================

|exhale_lsh| :ref:`Return to documentation for file <file_src_DataStructures_SpatialMap.h>` (``src/DataStructures/SpatialMap.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef DEQUEARRAY_H
   #define DEQUEARRAY_H
   
   #include <vector>
   #include <Eigen/Core>
   
   #define TOP_HASH_BIT_COUNT 4
   
   namespace QDVO {
   
   /*
    * This datastructure is a series of nested hash tables where keys are guaranteed not to collide.
    * It provides nearly instantaneous access of the elements within the table.
    *
    * The hash tables are only nested once because this is meant to be used for an image.
    *
    * You can query this table with a negative indexed point. I intend this datastructure to be used for radial searches.
    *
    * ( 1 0 1 0 || 0 1 0 1 0 1) left: top index, right: bottom index
    *
    * While the sizeof integers may be up to 64 bits depending on the OS / hardware, I will assume that on the bottom n bits are used.
    * This will allow me to create the hash functions using bitshifts.
    *
    * this datastructure operates with a center index of (0, 0)
    */
   template <typename T>
   class SpatialMap{
   
   public:
       SpatialMap();
   
       SpatialMap(const unsigned width);
   
       T& get(const Eigen::Vector2i& pixel)
       {
           this->index1 = this->topHash(pixel(0), pixel(1));
           this->index2 = this->bottomHash(pixel(0), pixel(1));
   
   
           std::unique_ptr<std::vector<T> >& bottomHashTableRef = this->mapOfMaps.at(this->index1);
   
   
           // it is likely that this bottom hash table has not been created yet.
           if (bottomHashTableRef == nullptr)
           {
               // create the bottom table.
               bottomHashTableRef = std::unique_ptr<std::vector<T> >(new std::vector<T> (this->bottomHashTableSize));
           }
   
   
           return bottomHashTableRef->at(this->index2);
       }
   
       void reset()
       {
           for (auto& e : this->mapOfMaps)
           {
               // if there is data here, reset it.
               if (e != nullptr)
               {
                   for (auto& pc : *(e))
                   {
                       pc.reset();
                   }
               }
           }
       }
   
   private:
   
       /*
        * computes the n bit index for the top hash table.
        * ASSUMES positive values
        */
       size_t topHash(const int x, const int y)
       {
           assert(x >= 0 && y >= 0);
           return (x & this->topHashMask) + ((y & this->topHashMask) >> this->topHashBitCount);
       }
   
       /*
        * computes the m bit index for the top hash table.
        * ASSUMES positive values
        */
       size_t bottomHash(const int x, const int y)
       {
           assert(x >= 0 && y >= 0);
           return (x & this->bottomHashMask) + ((y & this->bottomHashMask) >> this->bottomHashBitCount);
       }
   
   
   
       std::vector<std::unique_ptr<std::vector<T> > > mapOfMaps;
   
       unsigned maximumIndexBitCount;
       //Eigen::Vector2i pixelShift; // used to shift the pixel to a top left zero index.
       unsigned topHashBitCount = TOP_HASH_BIT_COUNT; // ideally 4 bits
       unsigned bottomHashBitCount; // ideally n - 4 bits
       size_t bottomHashTableSize;
   
       //bit masks
       unsigned topHashMask;
       unsigned bottomHashMask;
   
       // preallocated temporary variables
       //Eigen::Vector2i temp;
       size_t index1, index2;
   
   };
   
   template <typename T> SpatialMap<T>::SpatialMap()
   {
   
   }
   
   template <typename T> SpatialMap<T>::SpatialMap(unsigned width)
   {
       this->maximumIndexBitCount = std::ceil(std::log2(width));
       //const int shift = (0x01 << this->maximumIndexBitCount) / 2 - 1;
       //this->pixelShift = Eigen::Vector2i(shift, shift);
   
       this->topHashBitCount = TOP_HASH_BIT_COUNT;
       this->bottomHashBitCount = this->maximumIndexBitCount - this->topHashBitCount;
   
       this->topHashMask = ((0x01 << this->topHashBitCount) - 1) << this->topHashBitCount;
       this->bottomHashMask = ((0x01 << this->bottomHashBitCount) - 1);
   
   
       this->mapOfMaps.resize(0x01 << 2*this->topHashBitCount);
   
       this->bottomHashTableSize = 0x01 << 2*this->bottomHashBitCount;
   }
   
   }
   
   
   
   #endif // DEQUEARRAY_H
