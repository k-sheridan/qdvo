
.. _program_listing_file_src_Types.h:

Program Listing for File Types.h
================================

|exhale_lsh| :ref:`Return to documentation for file <file_src_Types.h>` (``src/Types.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #pragma once
   
   #include "GlobalDefinitions.h"
   #include <sophus/se3.hpp>
   #include <optional>
   
   namespace QDVO
   {
   
   using ID = ID_TYPE;
   using Scalar = SCALAR_TYPE;
   using SE3 = Sophus::SE3<Scalar>;
   using SO3 = Sophus::SO3<Scalar>;
   using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
   using Vector2 = Eigen::Matrix<Scalar, 2, 1>;
   using Matrix2 = Eigen::Matrix<Scalar, 2, 2>;
   
   template<typename T>
   using Result = std::optional<T>;
   
   } // namespace QDVO
