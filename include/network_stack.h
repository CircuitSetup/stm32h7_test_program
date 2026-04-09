#ifndef NETWORK_STACK_H
#define NETWORK_STACK_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool network_stack_init(void);
bool network_stack_is_ready(void);

#ifdef __cplusplus
}
#endif

#endif /* NETWORK_STACK_H */
