
.. _program_listing_file_src_Optimizer_SlotMap.h:

Program Listing for File SlotMap.h
==================================

|exhale_lsh| :ref:`Return to documentation for file <file_src_Optimizer_SlotMap.h>` (``src/Optimizer/SlotMap.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #pragma once
   
   #include <vector>
   #include <set>
   
   namespace ArgMin
   {
   
   struct SlotMapKeyBase
   {
   public:
       using index_type = size_t;
       using generation_type = size_t;
   
       index_type index;
       generation_type generation;
   
       void setInvalid()
       {
           index = std::numeric_limits<index_type>::max();
       }
   
       bool isInvalid()
       {
           return index == std::numeric_limits<index_type>::max();
       }
   };
   
   template <typename DataType, typename KeyType>
   class SlotMap
   {
   
       static_assert(std::is_integral<typename KeyType::index_type>::value);
       static_assert(std::is_integral<typename KeyType::generation_type>::value);
   
       struct Slot
       {
           size_t dataIndex; // index of the data in the data array.
           typename KeyType::generation_type generation = 0;
           bool free = true;
       };
   
       std::vector<Slot> slots;       // Set of keys currently in use.
       std::vector<size_t> freeSlots; // Stores indices of which slots can be inserted to.
   
       std::vector<DataType> data;          // vector containing all data.
       std::vector<size_t> dataToSlotIndex; // vector which points data to its slot index.
   
   public:
       SlotMap()
       {
       }
   
       KeyType insert(DataType value)
       {
           size_t slotIndex;
   
           if (freeSlots.empty())
           {
               // add new slot
               slotIndex = slots.size();
               slots.emplace_back();
           }
           else
           {
               slotIndex = freeSlots.back();
               freeSlots.pop_back();
           }
   
           // get slot reference
           Slot &slot = slots.at(slotIndex);
           assert(slot.free == true);
   
           // set up slot
           // set slot to not free
           slot.free = false;
           slot.dataIndex = data.size();
   
           // push a new data member to the back of the data arrays.
           data.push_back(value);
           dataToSlotIndex.push_back(slotIndex);
   
           // setup key.
           KeyType key;
           key.index = slotIndex;
           key.generation = slot.generation;
   
           return key;
       }
   
       void erase(KeyType &key)
       {
           // check if there are enough slots
           if (slots.size() <= key.index)
           {
               return;
           }
   
           const auto &slot = slots.at(key.index);
   
           // check if the generations match
           if (slot.generation != key.generation)
           {
               return;
           }
   
           assert(slot.dataIndex < data.size());
   
           // swap the data to be deleted with the last data element.
           std::swap(data.at(slot.dataIndex), data.back());
           std::swap(dataToSlotIndex.at(slot.dataIndex), dataToSlotIndex.back());
   
           // fix the slot pointing to the swapped data
           slots.at(dataToSlotIndex.at(slot.dataIndex)).dataIndex = slot.dataIndex;
   
           // increment the deleted slots generation
           const size_t deletedSlotIndex = dataToSlotIndex.back();
           ++slots.at(deletedSlotIndex).generation;
   
           // pop the data from the back
           data.pop_back();
           dataToSlotIndex.pop_back();
   
           // free the slot
           slots.at(deletedSlotIndex).free = true;
           freeSlots.push_back(deletedSlotIndex);
       }
   
       typename std::vector<DataType>::iterator at(const KeyType &key)
       {
           // check if there are enough slots
           if (slots.size() <= key.index)
           {
               return data.end();
           }
   
           const auto &slot = slots.at(key.index);
   
           // check if the generations match
           if (slot.generation != key.generation)
           {
               return data.end();
           }
   
           assert(slot.dataIndex < data.size());
   
           return data.begin() + slot.dataIndex;
       }
   
       typename std::vector<DataType>::const_iterator at(const KeyType &key) const
       {
           return at(key);
       }
   
       typename std::vector<DataType>::iterator begin()
       {
           return data.begin();
       }
   
       typename std::vector<DataType>::const_iterator begin() const
       {
           return data.begin();
       }
   
       typename std::vector<DataType>::iterator end()
       {
           return data.end();
       }
   
       typename std::vector<DataType>::const_iterator end() const
       {
           return data.end();
       }
   
       size_t size() const
       {
           return data.size();
       }
   
       KeyType getKeyFromDataIndex(size_t dataIndex)
       {
           size_t slotIndex = dataToSlotIndex.at(dataIndex);
   
           KeyType result;
   
           result.index = slotIndex;
           result.generation = slots.at(slotIndex).generation;
   
           return result;
       }
   };
   
   } //namespace ArgMin
