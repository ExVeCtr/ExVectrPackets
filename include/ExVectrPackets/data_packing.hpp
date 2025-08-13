#ifndef EXVECTRPACKET_DATAPACKING_HPP
#define EXVECTRPACKET_DATAPACKING_HPP

#include "stdint.h"
#include <limits>

namespace VCTR::Net
{

    // Pack any value into a fixed point representation with a given min and max value
    template <typename INTYPE, typename OUTTYPE>
    OUTTYPE packFixedPoint(const INTYPE &value, const INTYPE &limits)
    {
        static_assert(sizeof(OUTTYPE) < sizeof(INTYPE), "Packing does not make sense to pack into a larger type than input");
        static_assert(std::numeric_limits<OUTTYPE>::is_signed, "Output type must be signed");

        if (value > limits)
            return std::numeric_limits<OUTTYPE>::max();
        if (value < -limits)
            return std::numeric_limits<OUTTYPE>::min();

        return static_cast<OUTTYPE>(value / limits * std::numeric_limits<OUTTYPE>::max());
    }

    // Unpack any value from a fixed point representation with a given min and max value
    template <typename INTYPE, typename OUTTYPE>
    OUTTYPE unpackFixedPoint(const INTYPE &value, const INTYPE &limits)
    {
        static_assert(std::numeric_limits<INTYPE>::is_signed, "Input type must be signed");

        return static_cast<OUTTYPE>(value / std::numeric_limits<OUTTYPE>::max() * limits);
    }

} // namespace VCTR::Net

#endif // EXVECTRPACKET_DATAPACKING_HPP