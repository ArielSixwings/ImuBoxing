#include "Data.h"

#include <vector>
#include <optional>

namespace classifier::classes
{
    class Group
    {
    public:
        Group(const std::vector<Data> &points);

        std::optional<Data> Mean();
        std::optional<double> StandardDeviation();

        std::optional<Data> GetMean();
        std::optional<double> GetStandardDeviation();

    private:
        std::vector<Data> m_points;
        std::optional<Data> m_mean;
        std::optional<double> m_standardDeviation;
    };
}
