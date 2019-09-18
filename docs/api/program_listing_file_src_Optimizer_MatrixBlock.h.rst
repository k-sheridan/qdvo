
.. _program_listing_file_src_Optimizer_MatrixBlock.h:

Program Listing for File MatrixBlock.h
======================================

|exhale_lsh| :ref:`Return to documentation for file <file_src_Optimizer_MatrixBlock.h>` (``src/Optimizer/MatrixBlock.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #pragma once
   
   #include <Eigen/Core>
   
   namespace LittleOptimizer {
   
   template<typename Scalar, size_t rows, size_t cols>
   class MatrixBlock {
   
   typedef Eigen::Matrix<Scalar, rows, cols> matrix_t;
   
   bool isZero() const {
       return zero;
   }
   
   void setZero() {
       this->zero = true;
   }
   
   matrix_t& getMatrix() {
       return matrix;
   }
   
   void setMatrix(matrix_t& m) {
       this->zero = false;
       this->matrix = m;
   }
   
   MatrixBlock<Scalar, rows, cols>& operator += (MatrixBlock<Scalar, rows, cols>& b) {
       if (this->isZero()) {
           return b;
       } else {
           if (b.isZero()) {
               return *this;
           } else {
               this->matrix += b.getMatrix();
               return *this;
           }
       }
   }
   
   MatrixBlock<Scalar, rows, cols> operator * (MatrixBlock<Scalar, rows, cols>& b) {
       MatrixBlock<Scalar, rows, cols> result;
       if (!this->isZero() && !b.isZero()) {
           result.setMatrix(this->matrix * b.getMatrix());
       }
       return result;
   }
   
   
   private:
   
   bool zero = true; // tracks whether the sub matrix is zero or not.
   
   matrix_t matrix;
   
   };
   }
