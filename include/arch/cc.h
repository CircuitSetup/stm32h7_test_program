/*
 * Minimal compiler abstraction for lwIP on GCC/ARM.
 */
#ifndef LWIP_ARCH_CC_H
#define LWIP_ARCH_CC_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#ifndef LITTLE_ENDIAN
#define LITTLE_ENDIAN 1234
#endif

#ifndef BIG_ENDIAN
#define BIG_ENDIAN 4321
#endif

#ifndef BYTE_ORDER
#define BYTE_ORDER LITTLE_ENDIAN
#endif

#ifndef PACK_STRUCT_BEGIN
#define PACK_STRUCT_BEGIN
#endif

#ifndef PACK_STRUCT_END
#define PACK_STRUCT_END
#endif

#ifndef PACK_STRUCT_STRUCT
#define PACK_STRUCT_STRUCT __attribute__((packed))
#endif

#ifndef PACK_STRUCT_FIELD
#define PACK_STRUCT_FIELD(x) x
#endif

#ifndef LWIP_PLATFORM_DIAG
#define LWIP_PLATFORM_DIAG(x) do { printf x; } while (0)
#endif

#ifndef LWIP_PLATFORM_ASSERT
#define LWIP_PLATFORM_ASSERT(x) do { \
    printf("lwIP assert: %s\n", x); \
    for (;;) { } \
  } while (0)
#endif

#ifndef LWIP_RAND
#define LWIP_RAND() ((uint32_t)rand())
#endif

#endif /* LWIP_ARCH_CC_H */
