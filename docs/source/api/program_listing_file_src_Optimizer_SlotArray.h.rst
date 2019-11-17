
.. _program_listing_file_src_Optimizer_SlotArray.h:

Program Listing for File SlotArray.h
====================================

|exhale_lsh| :ref:`Return to documentation for file <file_src_Optimizer_SlotArray.h>` (``src/Optimizer/SlotArray.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #pragma once
   
   #include <vector>
   
   namespace ArgMin
   {
   
   template <typename DataType, typename KeyType>
   class SlotArray
   {
   
       static_assert(std::is_integral<typename KeyType::index_type>::value);
   
       struct Slot
       {
           size_t dataIndex; // index of the data in the data array.
           bool free = true;
       };
   
       std::vector<Slot> slots;       // Set of keys currently in use.
   
       std::vector<DataType> data;          // vector containing all data.
       std::vector<size_t> dataToSlotIndex; // vector which points data to its slot index.
   
   public:
   
       enum InsertResult {
           SUCCESS_NO_OVERWRITE,
           SUCCESS_OVERWRITE,
           FAILURE
       };
   
       SlotArray()
       {
       }
   
       InsertResult insert(KeyType key, DataType value)
       {
           const size_t slotIndex = key.index;
   
           // Insert a new slot if necessary.
           if (slotIndex >= slots.size()) {
               slots.resize(slotIndex + 1);
           }
   
           // get slot reference
           Slot &slot = slots.at(slotIndex);
   
           // Is the slot free?
           if (slot.free) {
               // set up slot
               // set slot to not free
               slot.free = false;
               slot.dataIndex = data.size();
   
               // push a new data member to the back of the data arrays.
               data.push_back(value);
               dataToSlotIndex.push_back(slotIndex);
   
               return InsertResult::SUCCESS_NO_OVERWRITE;
   
           } else {
               // Overwrite the slot.
               data.at(slot.dataIndex) = value;
               assert(dataToSlotIndex.at(slot.dataIndex) == slotIndex);
   
               return InsertResult::SUCCESS_OVERWRITE;
           }
       }
   
       void erase(KeyType &key)
       {
           // check if there are enough slots
           if (slots.size() < key.index)
           {
               return;
           }
   
           const auto &slot = slots.at(key.index);
   
           if (slot.free) {
               return;
           }
   
           assert(slot.dataIndex < data.size());
           assert(dataToSlotIndex.at(slot.dataIndex) == key.index);
   
           // swap the data to be deleted with the last data element.
           std::swap(data.at(slot.dataIndex), data.back());
           std::swap(dataToSlotIndex.at(slot.dataIndex), dataToSlotIndex.back());
   
           // fix the slot pointing to the swapped data
           slots.at(dataToSlotIndex.at(slot.dataIndex)).dataIndex = slot.dataIndex;
   
           // pop the data from the back
           data.pop_back();
           dataToSlotIndex.pop_back();
   
           // free the slot
           slots.at(key.index).free = true;
       }
   
       typename std::vector<DataType>::iterator at(const KeyType &key)
       {
           // check if there are enough slots
           if (slots.size() <= key.index)
           {
               return data.end();
           }
   
           const auto &slot = slots.at(key.index);
   
           // If the slot is earased return nothing.
           if (slot.free) {
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
           assert(dataIndex < dataToSlotIndex.size());
           size_t slotIndex = dataToSlotIndex.at(dataIndex);
   
           KeyType result;
   
           result.index = slotIndex;
   
           return result;
       }
   };
   
   } //namespace ArgMin 
