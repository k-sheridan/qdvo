
.. _program_listing_file_src_Optimizer_Key.h:

Program Listing for File Key.h
==============================

|exhale_lsh| :ref:`Return to documentation for file <file_src_Optimizer_Key.h>` (``src/Optimizer/Key.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #pragma once
   
   #include "Types.h"
   #include "SlotMap.h"
   
   namespace ArgMin
   {
   
   template <typename T>
   class VariableKey : public SlotMapKeyBase
   {
   public:
       typedef T variable_type;
       // Slot Map key (index, generation).
       bool operator<(const VariableKey<T> &other) const
       {
           return this->index < other.index;
       }
   };
   
   template <typename T>
   class ErrorTermKey : public SlotMapKeyBase
   {
   public:
       typedef T errorterm_type;
       // Slot Map key (index, generation).
       bool operator<(const ErrorTermKey<T> &other) const
       {
           return this->index < other.index;
       }
   };
   
   } // namespace ArgMin
