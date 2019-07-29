#pragma once

namespace QDVO
{
template <typename T>
class ResultType {
    public:

    ResultType() = default;
    ResultType(T& value) : result(value)
    {
        this->success = true;
    }

    // Check if the result exists.
    bool failedToReturn(){return !success;}

    T& getResult(){
        if (!success)
        {
            throw std::runtime_error("No result to return!");
        } 
        return result;
    }

    private:
    bool success = false; // Flag for whether the function was successful.
    T result;


};
} // namespace QDVO

