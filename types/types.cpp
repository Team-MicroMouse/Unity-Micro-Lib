//
// Created by Jeroen on 15-4-2025.
//

#include "types.h"

Guid Guid::FromUint128_t(__uint128_t val) {
    return Guid {
        val << 64,
        (uint64_t)val
    };
}
