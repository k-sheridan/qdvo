
.. _program_listing_file_src_ParallelAlgorithms_ParallelAlgorithms.h:

Program Listing for File ParallelAlgorithms.h
=============================================

|exhale_lsh| :ref:`Return to documentation for file <file_src_ParallelAlgorithms_ParallelAlgorithms.h>` (``src/ParallelAlgorithms/ParallelAlgorithms.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #pragma once
   
   #include <algorithm>
   #include <mutex>
   #include <thread>
   #include <vector>
   #include <math.h>
   #include "ThreadingHelpers.h"
   //#include <thrust>
   
   namespace QDVO::ParallelAlgorithms
   {
   enum ExecutionType
   {
       SEQUENTIAL,
       PARALLEL_CPU,
       PARALLEL_CUDA
   };
   
   template <class InputIterator, class OutputIterator, class UnaryOperation>
   void transform(ExecutionType execution, InputIterator first, InputIterator last, OutputIterator result, UnaryOperation op)
   {
       if (execution == ExecutionType::SEQUENTIAL)
       {
           std::transform(first, last, result, op);
       }
       else if (execution == PARALLEL_CPU)
       {
           auto problemDividers = createListDividers(last - first);
           std::vector<std::thread> threads;
           threads.reserve(problemDividers.size());
   
           // Launch the threads.
           for (auto &divider : problemDividers)
               threads.emplace_back(std::transform<InputIterator, OutputIterator, UnaryOperation>, first + std::get<0>(divider), first + std::get<1>(divider), result + std::get<0>(divider), op);
   
           joinThreads(threads);
       }
       else if (execution == PARALLEL_CUDA)
       {
           assert(false);
       }
   }
   
   template <class InputIterator1, class InputIterator2,
             class OutputIterator, class BinaryOperation>
   void transform(ExecutionType execution, InputIterator1 first1, InputIterator1 last1,
                            InputIterator2 first2, OutputIterator result,
                            BinaryOperation binary_op)
   {
       if (execution == ExecutionType::SEQUENTIAL)
       {
           std::transform(first1, last1, first2, result, binary_op);
       }
       else if (execution == PARALLEL_CPU)
       {
           auto problemDividers = createListDividers(last1 - first1);
           std::vector<std::thread> threads;
           threads.reserve(problemDividers.size());
   
           // Launch the threads.
           for (auto &divider : problemDividers)
               threads.emplace_back(std::transform<InputIterator1, InputIterator2, OutputIterator, BinaryOperation>, first1 + std::get<0>(divider), first1 + std::get<1>(divider), first2 + std::get<0>(divider), result + std::get<0>(divider), binary_op);
   
           joinThreads(threads);
       }
       else if (execution == PARALLEL_CUDA)
       {
           assert(false);
       }
   }
   
   template <class InputIterator, class UnaryFunction>
   void for_each(ExecutionType execution, InputIterator first, InputIterator last, UnaryFunction f)
   {
       if (execution == ExecutionType::SEQUENTIAL)
       {
           std::for_each(first, last, f);
       }
       else if (execution == PARALLEL_CPU)
       {
           auto problemDividers = createListDividers(last - first);
           std::vector<std::thread> threads;
           threads.reserve(problemDividers.size());
   
           // Launch the threads.
           for (auto &divider : problemDividers)
               threads.emplace_back(std::for_each<InputIterator, UnaryFunction>, first + std::get<0>(divider), first + std::get<1>(divider), f);
   
           joinThreads(threads);
       }
       else if (execution == PARALLEL_CUDA)
       {
           assert(false);
       }
   }
   
   } // namespace QDVO::ParallelAlgorithms
