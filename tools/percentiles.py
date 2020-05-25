import math

# Function which computes the percentiles of a list
# data must be a sortable list.
# percentiles is a list of percentiles [0,100] 
# output: A dictionary in the form of ("pXXX", value) (ie "p50", 0.0388)
def computePercentiles(data, percentiles):
    # clean the data.
    data = [x for x in data if str(x) != 'nan' and str(x) != 'inf'] 
    data.sort()
    # return early if there is no data.
    if len(data) is 0:
        return
    result = {}
    for percentile in percentiles:
        pN = data[int(math.floor(percentile/100 * (len(data) - 1)))]
        result["p" + str(percentile)] = pN

    return result
