
.. _program_listing_file_src_Optimizer_MetaHelpers.h:

Program Listing for File MetaHelpers.h
======================================

|exhale_lsh| :ref:`Return to documentation for file <file_src_Optimizer_MetaHelpers.h>` (``src/Optimizer/MetaHelpers.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #pragma once
   
   namespace LittleOptimizer {
   
   template <typename... T>
   struct VariableGroup {};
   
   template <typename... T>
   struct ErrorTermGroup {};
   
   template <size_t T>
   struct Dimension {};
   
   template <typename T>
   struct Scalar {};
   
   template <typename T>
   struct TypedIndex {
       size_t index;
   };
   
   }
   
   namespace LittleOptimizer::internal {
       // Used to get the index of a tuple.
       template <class T, class Tuple>
       struct Index;
   
       template <class T, class... Types>
       struct Index<T, std::tuple<T, Types...>> {
           static const std::size_t value = 0;
       };
   
       template <class T, class U, class... Types>
       struct Index<T, std::tuple<U, Types...>> {
           static const std::size_t value = 1 + Index<T, std::tuple<Types...>>::value;
       };
   }
