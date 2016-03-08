#ifndef _LINUX_NTP_INTERNAL_H
#define _LINUX_NTP_INTERNAL_H

#include "linux/pps-profiler-utils.h"

/**
 * Structure for storing parameters of Kalman filter for
 * filtering timestamps in __hardpps procedure.
 * Internal structures:
 * Current state estimate. It is based on results of prediction
 * and last measurement vector.
 *  [cse11] - raw time
 *  [cse21] - real time
 * Current probe estimate. Expresses error of predicted state.
 *  [cpe11  0] - for raw time
 *  [0  cpe22] - for real time
 * Estimated error in process. It is degree of trust to measurements.
 * The bigger q than more trust to measurements.
 *  [q11  0] - for raw time
 *  [0  q22] - for real time
 * Estimated error in measurements. Is is level of noise in measurements
 * making.
 *  [r11  0] - for raw time
 *  [0  r22] - for real time
 * 
 * Is is simplified filter implementation because of floating point
 * operations absense in kernel, and because full implementation would be
 * more expensive in execution time.
 */
typedef struct {
	s64 cse11; //element in current state estimate matrix [1][1]
	s64 cse21; //element in current state estimate matrix [2][1]

	s64 cpe11; //element in current probe estimate matrix [1][1]
	s64 cpe22; //element in current probe estimate matrix [2][2]

	s64 q11; //element in estimated error in process matrix [1][1]
	s64 q22; //element in estimated error in process matrix [2][2]

	s64 r11; //element in estimated error in measurements matrix [1][1]
	s64 r22; //element in estimated error in measurements matrix [2][2]
} PPSKalman;

void PPSKalman_init(PPSKalman *ob, s64 cse11, s64 cse21,
	s64 cpe11, s64 cpe22, s64 q11, s64 q22, s64 r11, s64 r22);

/**
 * Make Step by kalman filter. cv - control vector matrix. m - measurements
 * cse = cse + cv + (cpe + q) / (cpe + q + r) * (m - cse - cv)
 * cpe = r * (1 - r / (cpe + q + r))
 */
void PPSKalman_Step(PPSKalman *ob, s64 cv11, s64 cv21, s64 m11, s64 m22);

extern void ntp_init(void);
extern void ntp_clear(void);
/* Returns how long ticks are at present, in ns / 2^NTP_SCALE_SHIFT. */
extern u64 ntp_tick_length(void);
extern ktime_t ntp_get_next_leap(void);
extern int second_overflow(unsigned long secs);
extern int ntp_validate_timex(struct timex *);
extern int __do_adjtimex(struct timex *, struct timespec64 *, s32 *);
extern void __hardpps(const struct timespec64 *, const struct timespec64 *);
#endif /* _LINUX_NTP_INTERNAL_H */
