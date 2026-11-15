/* Optional timing of the expensive library parts (build with DD2_PROFILE; the platform shows the result).
 * Without DD2_PROFILE the macros are empty. */
#ifndef PSX_PROFILE_H
#define PSX_PROFILE_H

#ifdef DD2_PROFILE
#include "platform/platform.h"

#define PROFILE_START() uint64_t profile_start_ = platform_ticks_ns()
#define PROFILE_END(part) platform_profile_add((part), platform_ticks_ns() - profile_start_)
#define PROFILE_FRAME() platform_profile_frame()
#else
#define PROFILE_START() ((void)0)
#define PROFILE_END(part) ((void)0)
#define PROFILE_FRAME() ((void)0)
#endif

#endif
