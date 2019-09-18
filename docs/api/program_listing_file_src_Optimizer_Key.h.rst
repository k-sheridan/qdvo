
.. _program_listing_file_src_Optimizer_Key.h:

Program Listing for File Key.h
==============================

|exhale_lsh| :ref:`Return to documentation for file <file_src_Optimizer_Key.h>` (``src/Optimizer/Key.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #pragma once
   
   #include "Types.h"
   #include "slot_map.h"
   
   namespace QDVO
   {
   
   class Key
   {
   public:
       QDVO::ID inner, outer;
   
       Key(QDVO::ID outerKey, QDVO::ID innerKey) : inner(innerKey), outer(outerKey) {}
   
       bool operator<(const Key &comp) const
       {
           return outer < comp.outer || (outer == comp.outer && inner < comp.inner);
       }
   };
   
   } // namespace QDVO
   
   namespace LittleOptimizer {
   
   template <typename T>
   struct VariableKey {
       std::pair<unsigned, unsigned> slotMapKey; // Slot Map key (index, generation).
   };
   
   } // namespace LittleOptimizer
