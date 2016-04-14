/*
 * NTP state machine interfaces and logic.
 *
 * This code was mainly moved from kernel/timer.c and kernel/time.c
 * Please see those files for relevant copyright info and historical
 * changelogs.
 */
#include <linux/capability.h>
#include <linux/clocksource.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/jiffies.h>
#include <linux/kthread.h>
#include <linux/math64.h>
#include <linux/timex.h>
#include <linux/time.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/rtc.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/spinlock.h>
#include <linux/gfp.h>
#include <linux/string.h>
#include <linux/init.h>

#include <linux/fs.h>

#include "ntp_internal.h"

/*
 * NTP timekeeping variables:
 *
 * Note: All of the NTP state is protected by the timekeeping locks.
 */


/* USER_HZ period (usecs): */
unsigned long			tick_usec = TICK_USEC;

/* SHIFTED_HZ period (nsecs): */
unsigned long			tick_nsec;

static u64			tick_length;
static u64			tick_length_base;

#define SECS_PER_DAY		86400
#define MAX_TICKADJ		500LL		/* usecs */
#define MAX_TICKADJ_SCALED \
	(((MAX_TICKADJ * NSEC_PER_USEC) << NTP_SCALE_SHIFT) / NTP_INTERVAL_FREQ)

/*
 * phase-lock loop variables
 */

/*
 * clock synchronization status
 *
 * (TIME_ERROR prevents overwriting the CMOS clock)
 */
static int			time_state = TIME_OK;

/* clock status bits:							*/
static int			time_status = STA_UNSYNC;

/* time adjustment (nsecs):						*/
static s64			time_offset;

/* pll time constant:							*/
static long			time_constant = 2;

/* maximum error (usecs):						*/
static long			time_maxerror = NTP_PHASE_LIMIT;

/* estimated error (usecs):						*/
static long			time_esterror = NTP_PHASE_LIMIT;

/* frequency offset (scaled nsecs/secs):				*/
static s64			time_freq;

/* time at last adjustment (secs):					*/
static long			time_reftime;

static long			time_adjust;

/* constant (boot-param configurable) NTP tick adjustment (upscaled)	*/
static s64			ntp_tick_adj;

/* second value of the next pending leapsecond, or TIME64_MAX if no leap */
static time64_t			ntp_next_leap_sec = TIME64_MAX;

#ifdef CONFIG_NTP_PPS

/*
 * The following variables are used when a pulse-per-second (PPS) signal
 * is available. They establish the engineering parameters of the clock
 * discipline loop when controlled by the PPS signal.
 */
#define PPS_VALID	10	/* PPS signal watchdog max (s) */
#define PPS_POPCORN	4	/* popcorn spike threshold (shift) */
#define PPS_INTMIN	2	/* min freq interval (s) (shift) */
#define PPS_INTMAX	8	/* max freq interval (s) (shift) */
#define PPS_INTCOUNT	4	/* number of consecutive good intervals to
				   increase pps_shift or consecutive bad
				   intervals to decrease it */
#define PPS_MAXWANDER	100000	/* max PPS freq wander (ns/s) */

#define PPS_FILTER_SIZE	4

static int pps_valid;		/* signal watchdog counter */
static long pps_tf[PPS_FILTER_SIZE];		/* history data for phase filter */
static unsigned pps_tf_pos;	/* current element in filter ring */
static long pps_jitter;		/* current jitter (ns) */
static struct timespec64 pps_fbase; /* beginning of the last freq interval */
static int pps_shift;		/* current interval duration (s) (shift) */
static int pps_intcnt;		/* interval counter */
static s64 pps_freq;		/* frequency offset (scaled ns/s) */
static long pps_stabil;		/* current stability (scaled ns/s) */

/*
 * PPS signal quality monitors
 */
static long pps_calcnt;		/* calibration intervals */
static long pps_jitcnt;		/* jitter limit exceeded */
static long pps_stbcnt;		/* stability limit exceeded */
static long pps_errcnt;		/* calibration errors */

PPSKalman kalman_filter;
char kalman_is_inited = 0;
char kalman_force_stop = 1;
s64 kalman_q_core_param = 0;
s64 kalman_start_offset_core_param = 0;
long (*pps_phase_filter_ptr)(long *) = NULL;

void PPSKalman_stop(void);
void PPSKalman_start(void);
void PPSKalman_restart(void);

char freq_unstable_mask = 0;
long freq_unstable_distance = 0;
int inversions_percent = 0;
long freq_ring[PPS_FILTER_SIZE];
int freq_ring_pos = PPS_FILTER_SIZE - 1;
char freq_is_unstable = 0;
long freq_invers_admis_error = 500;
#define FREQ_UNSTABLE_INVERSIONS 1
#define FREQ_UNSTABLE_LAST_RING 2

#define KalmanProcName "kalmanctl"
struct proc_dir_entry *kalman_proc = NULL;
int kalman_open_cnt = 0;
#define KalmanBufferSize 256

//previous timestamps from __hardpps
struct timespec64 phase_ts_prev, raw_ts_prev;

#ifdef CONFIG_PPS_PROFILER

/* Sets all global variables from values within profiler_vars_shot object */
void set_vars_from_dump(profiler_vars_shot *shot) {
	tick_usec = shot->shot_tick_usec;
	tick_nsec = shot->shot_tick_nsec;
	tick_length = shot->shot_tick_length;
	tick_length_base = shot->shot_tick_length_base;
	time_state = shot->shot_time_state;
	time_status = shot->shot_time_status;
	time_offset = shot->shot_time_offset;
	time_constant = shot->shot_time_constant;
	time_maxerror = shot->shot_time_maxerror;
	time_esterror = shot->shot_time_esterror;
	time_freq = shot->shot_time_freq;
	time_reftime = shot->shot_time_reftime;
	time_adjust = shot->shot_time_adjust;
	ntp_tick_adj = shot->shot_ntp_tick_adj;
	pps_valid = shot->shot_pps_valid;
	memcpy(pps_tf, shot->shot_pps_tf, sizeof(long) * PPS_FILTER_SIZE);
	pps_tf_pos = shot->shot_pps_tf_pos;
	pps_jitter = shot->shot_pps_jitter;
	pps_fbase = shot->shot_pps_fbase;
	pps_shift = shot->shot_pps_shift;
	pps_intcnt = shot->shot_pps_intcnt;
	pps_freq = shot->shot_pps_freq;
	pps_stabil = shot->shot_pps_stabil;
	pps_calcnt = shot->shot_pps_calcnt;
	pps_jitcnt = shot->shot_pps_jitcnt;
	pps_stbcnt = shot->shot_pps_stbcnt;
	pps_errcnt = shot->shot_pps_errcnt;
}

/* Parse kernel boot parameters */
static int __init setup_kernel_param_kalman_q(char *str) {
	char *iter;
	s64 res;
	s64 delimiter;

	printk(KERN_ALERT "Kernel param for Kalman: %s\n", str);
	res = 0;
	iter = str;
	delimiter = -1;
	while(iter && (*iter)) {
		if (*iter != '.') {
			res = res * 10 + (*iter) - '0';
		} else {
			delimiter = 1;
			++iter;
			continue;
		}
		if (delimiter > 0) {
			delimiter *= 10;
		}
		++iter;
	}
	printk(KERN_ALERT "Kernel param q: %lld / %lld\n", res, delimiter);
	if (delimiter < 0) delimiter = 1;
	kalman_q_core_param = div_s64((res << (NTP_SCALE_SHIFT - 2)), delimiter);
	return 0;
}

/* Parse kernel boot parameters */
static int __init setup_kernel_param_kalman_start(char *str) {
	printk(KERN_ALERT "Kernel param for Kalman: %s\n", str);
	kalman_start_offset_core_param = simple_strtoll(str, NULL, 0);
	return 0;
}

static int __init setup_kernel_param_kalman_force_stop(char *str) {
	printk(KERN_ALERT "Kernel param for Kalman: %s\n", str);
	kalman_force_stop = simple_strtoll(str, NULL, 0);
	return 0;
}

static int __init setup_kernel_param_freq_inversions(char *str) {
	printk(KERN_ALERT "Kernel param freq inversions: %s\n", str);
	inversions_percent = simple_strtoll(str, NULL, 0);
	freq_unstable_mask |= FREQ_UNSTABLE_INVERSIONS;
	return 0;
}

static int __init setup_kernel_param_freq_ring_dist(char *str) {
	printk(KERN_ALERT "Kernel param freq ring\n");
	freq_unstable_distance = simple_strtoll(str, NULL, 0);
	freq_unstable_mask |= FREQ_UNSTABLE_LAST_RING;
	return 0;
}

static int __init setup_kernel_param_freq_admis_error(char *str) {
	printk(KERN_ALERT "Kernel param freq admis error: %s\n", str);
	freq_invers_admis_error = simple_strtoll(str, NULL, 0);
	return 0;
}

early_param("kalman_q", setup_kernel_param_kalman_q);
early_param("kalman_start_offset", setup_kernel_param_kalman_start);
early_param("kalman_force_stop", setup_kernel_param_kalman_force_stop);
early_param("pps_freq_inversions", setup_kernel_param_freq_inversions);
early_param("pps_freq_ring_dist", setup_kernel_param_freq_ring_dist);
early_param("pps_freq_invers_admis_error", setup_kernel_param_freq_admis_error);

/* Sets values within profiler_vars_shot object from global variables */
void get_vars_to_dump(profiler_vars_shot *dump);

/* Monotinoc iterator for getting unic id for each log_entry */
unsigned long profiler_iterator = 0;

struct page *ring_pages = NULL;
int ring_pages_order = 0;
int ring_pages_cnt;

struct page *ring_shot_pages = NULL;

//#define PPS_PROFILER_DEBUG 1
/* array of log records about hardpps working */
profiler_log_entry *profiler_ring;
/* spinlock for protecting main ring */
spinlock_t profiler_ring_lock;

/* last copy of profiler_ring for one pps_reader */
profiler_log_entry *profiler_ring_shot;
/* spinlock for protecting copy of ring */
loff_t profiler_ring_copy_size;

/* buffer, in which each action during one hardpps call is registered */
profiler_log_entry cur_entry;

/* parameters of main ring */
loff_t profiler_ring_size = 0;
loff_t profiler_ring_begin = 0;
loff_t profiler_ring_end = 0;

/* Array of dumps with all global variables */
profiler_vars_shot dumps[DUMPS_CNT];
int dumps_size = 0;

/* 
 * Dump that must be showed in log file before any timestamps,
 * and that corresponds to first pair of timestamps.
 */
profiler_vars_shot shot_dump;

/* entry for proc file */
struct proc_dir_entry *ring_entry = NULL;

/* constructor for profiler_ring */
static void profiler_ring_init(void);

/* constructor for profiler_log_entry */
void clear_log_entry(profiler_log_entry *ob) {
	ob->id = 0;
}

/* add timespec pair (real and raw) to entry */
void add_pair_to_entry(profiler_log_entry *entry,
	const struct timespec64 *phase_ts, const struct timespec64 *raw_ts);

/* add entry object to the end of main ring */
void push_entry_to_ring(profiler_log_entry *entry);

#endif

/* PPS kernel consumer compensates the whole phase error immediately.
 * Otherwise, reduce the offset by a fixed factor times the time constant.
 */
static inline s64 ntp_offset_chunk(s64 offset)
{
	if (time_status & STA_PPSTIME && time_status & STA_PPSSIGNAL)
		return offset;
	else
		return shift_right(offset, SHIFT_PLL + time_constant);
}

static inline void pps_reset_freq_interval(void)
{
	/* the PPS calibration interval may end
	   surprisingly early */
	pps_shift = PPS_INTMIN;
	pps_intcnt = 0;
	pps_fbase.tv_sec = pps_fbase.tv_nsec = 0;
}

/**
 * pps_clear - Clears the PPS state variables
 */
static inline void pps_clear(void)
{
	unsigned i;
	pps_reset_freq_interval();

	for (i = 0; i < PPS_FILTER_SIZE; i++)
		pps_tf[i] = 0;

	pps_tf_pos = 0;
	pps_freq = 0;
}

/* Decrease pps_valid to indicate that another second has passed since
 * the last PPS signal. When it reaches 0, indicate that PPS signal is
 * missing.
 */
static inline void pps_dec_valid(void)
{
	/* Silently ignore if PPS was not turned on */
	if (!(time_status&STA_PPSSIGNAL))
		return;

	if (pps_valid > 0)
		pps_valid--;
	else {
		time_status &= ~(STA_PPSSIGNAL | STA_PPSJITTER |
				 STA_PPSWANDER | STA_PPSERROR);
		pps_clear();
	}
}

static inline void pps_set_freq(s64 freq)
{
	pps_freq = freq;
}

static inline int is_error_status(int status)
{
	return (status & (STA_UNSYNC|STA_CLOCKERR))
		/* PPS signal lost when either PPS time or
		 * PPS frequency synchronization requested
		 */
		|| ((status & (STA_PPSFREQ|STA_PPSTIME))
			&& !(status & STA_PPSSIGNAL))
		/* PPS jitter exceeded when
		 * PPS time synchronization requested */
		|| ((status & (STA_PPSTIME|STA_PPSJITTER))
			== (STA_PPSTIME|STA_PPSJITTER))
		/* PPS wander exceeded or calibration error when
		 * PPS frequency synchronization requested
		 */
		|| ((status & STA_PPSFREQ)
			&& (status & (STA_PPSWANDER|STA_PPSERROR)));
}

static inline void pps_fill_timex(struct timex *txc)
{
	txc->ppsfreq	   = shift_right((pps_freq >> PPM_SCALE_INV_SHIFT) *
					 PPM_SCALE_INV, NTP_SCALE_SHIFT);
	txc->jitter	   = pps_jitter;
	if (!(time_status & STA_NANO))
		txc->jitter /= NSEC_PER_USEC;
	txc->shift	   = pps_shift;
	txc->stabil	   = pps_stabil;
	txc->jitcnt	   = pps_jitcnt;
	txc->calcnt	   = pps_calcnt;
	txc->errcnt	   = pps_errcnt;
	txc->stbcnt	   = pps_stbcnt;
}

#else /* !CONFIG_NTP_PPS */

static inline s64 ntp_offset_chunk(s64 offset)
{
	return shift_right(offset, SHIFT_PLL + time_constant);
}

static inline void pps_reset_freq_interval(void) {}
static inline void pps_clear(void) {}
static inline void pps_dec_valid(void) {}
static inline void pps_set_freq(s64 freq) {}

static inline int is_error_status(int status)
{
	return status & (STA_UNSYNC|STA_CLOCKERR);
}

static inline void pps_fill_timex(struct timex *txc)
{
	/* PPS is not implemented, so these are zero */
	txc->ppsfreq	   = 0;
	txc->jitter	   = 0;
	txc->shift	   = 0;
	txc->stabil	   = 0;
	txc->jitcnt	   = 0;
	txc->calcnt	   = 0;
	txc->errcnt	   = 0;
	txc->stbcnt	   = 0;
}

#endif /* CONFIG_NTP_PPS */


/**
 * ntp_synced - Returns 1 if the NTP status is not UNSYNC
 *
 */
static inline int ntp_synced(void)
{
	return !(time_status & STA_UNSYNC);
}


/*
 * NTP methods:
 */

/*
 * Update (tick_length, tick_length_base, tick_nsec), based
 * on (tick_usec, ntp_tick_adj, time_freq):
 */
static void ntp_update_frequency(void)
{
	u64 second_length;
	u64 new_base;

	second_length		 = (u64)(tick_usec * NSEC_PER_USEC * USER_HZ)
						<< NTP_SCALE_SHIFT;

	second_length		+= ntp_tick_adj;
	second_length		+= time_freq;

	tick_nsec		 = div_u64(second_length, HZ) >> NTP_SCALE_SHIFT;
	new_base		 = div_u64(second_length, NTP_INTERVAL_FREQ);

	/*
	 * Don't wait for the next second_overflow, apply
	 * the change to the tick length immediately:
	 */
	tick_length		+= new_base - tick_length_base;
	tick_length_base	 = new_base;
}

static inline s64 ntp_update_offset_fll(s64 offset64, long secs)
{
	time_status &= ~STA_MODE;

	if (secs < MINSEC)
		return 0;

	if (!(time_status & STA_FLL) && (secs <= MAXSEC))
		return 0;

	time_status |= STA_MODE;

	return div64_long(offset64 << (NTP_SCALE_SHIFT - SHIFT_FLL), secs);
}

static void ntp_update_offset(long offset)
{
	s64 freq_adj;
	s64 offset64;
	long secs;

	if (!(time_status & STA_PLL))
		return;

	if (!(time_status & STA_NANO))
		offset *= NSEC_PER_USEC;

	/*
	 * Scale the phase adjustment and
	 * clamp to the operating range.
	 */
	offset = min(offset, MAXPHASE);
	offset = max(offset, -MAXPHASE);

	/*
	 * Select how the frequency is to be controlled
	 * and in which mode (PLL or FLL).
	 */
	secs = get_seconds() - time_reftime;
	if (unlikely(time_status & STA_FREQHOLD))
		secs = 0;

	time_reftime = get_seconds();

	offset64    = offset;
	freq_adj    = ntp_update_offset_fll(offset64, secs);

	/*
	 * Clamp update interval to reduce PLL gain with low
	 * sampling rate (e.g. intermittent network connection)
	 * to avoid instability.
	 */
	if (unlikely(secs > 1 << (SHIFT_PLL + 1 + time_constant)))
		secs = 1 << (SHIFT_PLL + 1 + time_constant);

	freq_adj    += (offset64 * secs) <<
			(NTP_SCALE_SHIFT - 2 * (SHIFT_PLL + 2 + time_constant));

	freq_adj    = min(freq_adj + time_freq, MAXFREQ_SCALED);

	time_freq   = max(freq_adj, -MAXFREQ_SCALED);

	time_offset = div_s64(offset64 << NTP_SCALE_SHIFT, NTP_INTERVAL_FREQ);
}

/**
 * ntp_clear - Clears the NTP state variables
 */
void ntp_clear(void)
{
	time_adjust	= 0;		/* stop active adjtime() */
	time_status	|= STA_UNSYNC;
	time_maxerror	= NTP_PHASE_LIMIT;
	time_esterror	= NTP_PHASE_LIMIT;

	ntp_update_frequency();

	tick_length	= tick_length_base;
	time_offset	= 0;

	ntp_next_leap_sec = TIME64_MAX;
	/* Clear PPS state variables */
	pps_clear();
}


u64 ntp_tick_length(void)
{
	return tick_length;
}

/**
 * ntp_get_next_leap - Returns the next leapsecond in CLOCK_REALTIME ktime_t
 *
 * Provides the time of the next leapsecond against CLOCK_REALTIME in
 * a ktime_t format. Returns KTIME_MAX if no leapsecond is pending.
 */
ktime_t ntp_get_next_leap(void)
{
	ktime_t ret;

	if ((time_state == TIME_INS) && (time_status & STA_INS))
		return ktime_set(ntp_next_leap_sec, 0);
	ret.tv64 = KTIME_MAX;
	return ret;
}

/*
 * this routine handles the overflow of the microsecond field
 *
 * The tricky bits of code to handle the accurate clock support
 * were provided by Dave Mills (Mills@UDEL.EDU) of NTP fame.
 * They were originally developed for SUN and DEC kernels.
 * All the kudos should go to Dave for this stuff.
 *
 * Also handles leap second processing, and returns leap offset
 */
int second_overflow(unsigned long secs)
{
	s64 delta;
	int leap = 0;

	/*
	 * Leap second processing. If in leap-insert state at the end of the
	 * day, the system clock is set back one second; if in leap-delete
	 * state, the system clock is set ahead one second.
	 */
	switch (time_state) {
	case TIME_OK:
		if (time_status & STA_INS) {
			time_state = TIME_INS;
			ntp_next_leap_sec = secs + SECS_PER_DAY -
						(secs % SECS_PER_DAY);
		} else if (time_status & STA_DEL) {
			time_state = TIME_DEL;
			ntp_next_leap_sec = secs + SECS_PER_DAY -
						 ((secs+1) % SECS_PER_DAY);
		}
		break;
	case TIME_INS:
		if (!(time_status & STA_INS)) {
			ntp_next_leap_sec = TIME64_MAX;
			time_state = TIME_OK;
		} else if (secs % SECS_PER_DAY == 0) {
			leap = -1;
			time_state = TIME_OOP;
			printk(KERN_NOTICE
				"Clock: inserting leap second 23:59:60 UTC\n");
		}
		break;
	case TIME_DEL:
		if (!(time_status & STA_DEL)) {
			ntp_next_leap_sec = TIME64_MAX;
			time_state = TIME_OK;
		} else if ((secs + 1) % SECS_PER_DAY == 0) {
			leap = 1;
			ntp_next_leap_sec = TIME64_MAX;
			time_state = TIME_WAIT;
			printk(KERN_NOTICE
				"Clock: deleting leap second 23:59:59 UTC\n");
		}
		break;
	case TIME_OOP:
		ntp_next_leap_sec = TIME64_MAX;
		time_state = TIME_WAIT;
		break;
	case TIME_WAIT:
		if (!(time_status & (STA_INS | STA_DEL)))
			time_state = TIME_OK;
		break;
	}


	/* Bump the maxerror field */
	time_maxerror += MAXFREQ / NSEC_PER_USEC;
	if (time_maxerror > NTP_PHASE_LIMIT) {
		time_maxerror = NTP_PHASE_LIMIT;
		time_status |= STA_UNSYNC;
	}

	/* Compute the phase adjustment for the next second */
	tick_length	 = tick_length_base;

	delta		 = ntp_offset_chunk(time_offset);
	time_offset	-= delta;
	tick_length	+= delta;

	/* Check PPS signal */
	pps_dec_valid();

	if (!time_adjust)
		goto out;

	if (time_adjust > MAX_TICKADJ) {
		time_adjust -= MAX_TICKADJ;
		tick_length += MAX_TICKADJ_SCALED;
		goto out;
	}

	if (time_adjust < -MAX_TICKADJ) {
		time_adjust += MAX_TICKADJ;
		tick_length -= MAX_TICKADJ_SCALED;
		goto out;
	}

	tick_length += (s64)(time_adjust * NSEC_PER_USEC / NTP_INTERVAL_FREQ)
							 << NTP_SCALE_SHIFT;
	time_adjust = 0;

out:
	return leap;
}

#ifdef CONFIG_GENERIC_CMOS_UPDATE
int __weak update_persistent_clock(struct timespec now)
{
	return -ENODEV;
}

int __weak update_persistent_clock64(struct timespec64 now64)
{
	struct timespec now;

	now = timespec64_to_timespec(now64);
	return update_persistent_clock(now);
}
#endif

#if defined(CONFIG_GENERIC_CMOS_UPDATE) || defined(CONFIG_RTC_SYSTOHC)
static void sync_cmos_clock(struct work_struct *work);

static DECLARE_DELAYED_WORK(sync_cmos_work, sync_cmos_clock);

static void sync_cmos_clock(struct work_struct *work)
{
	struct timespec64 now;
	struct timespec64 next;
	int fail = 1;

	/*
	 * If we have an externally synchronized Linux clock, then update
	 * CMOS clock accordingly every ~11 minutes. Set_rtc_mmss() has to be
	 * called as close as possible to 500 ms before the new second starts.
	 * This code is run on a timer.  If the clock is set, that timer
	 * may not expire at the correct time.  Thus, we adjust...
	 * We want the clock to be within a couple of ticks from the target.
	 */
	if (!ntp_synced()) {
		/*
		 * Not synced, exit, do not restart a timer (if one is
		 * running, let it run out).
		 */
		return;
	}

	getnstimeofday64(&now);
	if (abs(now.tv_nsec - (NSEC_PER_SEC / 2)) <= tick_nsec * 5) {
		struct timespec64 adjust = now;

		fail = -ENODEV;
		if (persistent_clock_is_local)
			adjust.tv_sec -= (sys_tz.tz_minuteswest * 60);
#ifdef CONFIG_GENERIC_CMOS_UPDATE
		fail = update_persistent_clock64(adjust);
#endif

#ifdef CONFIG_RTC_SYSTOHC
		if (fail == -ENODEV)
			fail = rtc_set_ntp_time(adjust);
#endif
	}

	next.tv_nsec = (NSEC_PER_SEC / 2) - now.tv_nsec - (TICK_NSEC / 2);
	if (next.tv_nsec <= 0)
		next.tv_nsec += NSEC_PER_SEC;

	if (!fail || fail == -ENODEV)
		next.tv_sec = 659;
	else
		next.tv_sec = 0;

	if (next.tv_nsec >= NSEC_PER_SEC) {
		next.tv_sec++;
		next.tv_nsec -= NSEC_PER_SEC;
	}
	queue_delayed_work(system_power_efficient_wq,
			   &sync_cmos_work, timespec64_to_jiffies(&next));
}

#ifdef CONFIG_PREEMPT_RT_FULL
/*
 * RT can not call schedule_delayed_work from real interrupt context.
 * Need to make a thread to do the real work.
 */
static struct task_struct *cmos_delay_thread;
static bool do_cmos_delay;

static int run_cmos_delay(void *ignore)
{
	while (!kthread_should_stop()) {
		set_current_state(TASK_INTERRUPTIBLE);
		if (do_cmos_delay) {
			do_cmos_delay = false;
			queue_delayed_work(system_power_efficient_wq,
					   &sync_cmos_work, 0);
		}
		schedule();
	}
	__set_current_state(TASK_RUNNING);
	return 0;
}

void ntp_notify_cmos_timer(void)
{
	do_cmos_delay = true;
	/* Make visible before waking up process */
	smp_wmb();
	wake_up_process(cmos_delay_thread);
}

static __init int create_cmos_delay_thread(void)
{
	cmos_delay_thread = kthread_run(run_cmos_delay, NULL, "kcmosdelayd");
	BUG_ON(!cmos_delay_thread);
	return 0;
}
early_initcall(create_cmos_delay_thread);

#else

void ntp_notify_cmos_timer(void)
{
	queue_delayed_work(system_power_efficient_wq, &sync_cmos_work, 0);
}
#endif /* CONFIG_PREEMPT_RT_FULL */

#else
void ntp_notify_cmos_timer(void) { }
#endif


/*
 * Propagate a new txc->status value into the NTP state:
 */
static inline void process_adj_status(struct timex *txc, struct timespec64 *ts)
{
	if ((time_status & STA_PLL) && !(txc->status & STA_PLL)) {
		time_state = TIME_OK;
		time_status = STA_UNSYNC;
		ntp_next_leap_sec = TIME64_MAX;
		/* restart PPS frequency calibration */
		pps_reset_freq_interval();
	}

	/*
	 * If we turn on PLL adjustments then reset the
	 * reference time to current time.
	 */
	if (!(time_status & STA_PLL) && (txc->status & STA_PLL))
		time_reftime = get_seconds();

	/* only set allowed bits */
	time_status &= STA_RONLY;
	time_status |= txc->status & ~STA_RONLY;
}


static inline void process_adjtimex_modes(struct timex *txc,
						struct timespec64 *ts,
						s32 *time_tai)
{
	if (txc->modes & ADJ_STATUS)
		process_adj_status(txc, ts);

	if (txc->modes & ADJ_NANO)
		time_status |= STA_NANO;

	if (txc->modes & ADJ_MICRO)
		time_status &= ~STA_NANO;

	if (txc->modes & ADJ_FREQUENCY) {
		time_freq = txc->freq * PPM_SCALE;
		time_freq = min(time_freq, MAXFREQ_SCALED);
		time_freq = max(time_freq, -MAXFREQ_SCALED);
		/* update pps_freq */
		pps_set_freq(time_freq);
	}

	if (txc->modes & ADJ_MAXERROR)
		time_maxerror = txc->maxerror;

	if (txc->modes & ADJ_ESTERROR)
		time_esterror = txc->esterror;

	if (txc->modes & ADJ_TIMECONST) {
		time_constant = txc->constant;
		if (!(time_status & STA_NANO))
			time_constant += 4;
		time_constant = min(time_constant, (long)MAXTC);
		time_constant = max(time_constant, 0l);
	}

	if (txc->modes & ADJ_TAI && txc->constant > 0)
		*time_tai = txc->constant;

	if (txc->modes & ADJ_OFFSET)
		ntp_update_offset(txc->offset);

	if (txc->modes & ADJ_TICK)
		tick_usec = txc->tick;

	if (txc->modes & (ADJ_TICK|ADJ_FREQUENCY|ADJ_OFFSET))
		ntp_update_frequency();
}



/**
 * ntp_validate_timex - Ensures the timex is ok for use in do_adjtimex
 */
int ntp_validate_timex(struct timex *txc)
{
	if (txc->modes & ADJ_ADJTIME) {
		/* singleshot must not be used with any other mode bits */
		if (!(txc->modes & ADJ_OFFSET_SINGLESHOT))
			return -EINVAL;
		if (!(txc->modes & ADJ_OFFSET_READONLY) &&
		    !capable(CAP_SYS_TIME))
			return -EPERM;
	} else {
		/* In order to modify anything, you gotta be super-user! */
		 if (txc->modes && !capable(CAP_SYS_TIME))
			return -EPERM;
		/*
		 * if the quartz is off by more than 10% then
		 * something is VERY wrong!
		 */
		if (txc->modes & ADJ_TICK &&
		    (txc->tick <  900000/USER_HZ ||
		     txc->tick > 1100000/USER_HZ))
			return -EINVAL;
	}

	if ((txc->modes & ADJ_SETOFFSET) && (!capable(CAP_SYS_TIME)))
		return -EPERM;

	/*
	 * Check for potential multiplication overflows that can
	 * only happen on 64-bit systems:
	 */
	if ((txc->modes & ADJ_FREQUENCY) && (BITS_PER_LONG == 64)) {
		if (LLONG_MIN / PPM_SCALE > txc->freq)
			return -EINVAL;
		if (LLONG_MAX / PPM_SCALE < txc->freq)
			return -EINVAL;
	}

	return 0;
}


/*
 * adjtimex mainly allows reading (and writing, if superuser) of
 * kernel time-keeping variables. used by xntpd.
 */
int __do_adjtimex(struct timex *txc, struct timespec64 *ts, s32 *time_tai)
{
	int result;

	if (txc->modes & ADJ_ADJTIME) {
		long save_adjust = time_adjust;

		if (!(txc->modes & ADJ_OFFSET_READONLY)) {
			/* adjtime() is independent from ntp_adjtime() */
			time_adjust = txc->offset;
			ntp_update_frequency();
		}
		txc->offset = save_adjust;
	} else {

		/* If there are input parameters, then process them: */
		if (txc->modes)
			process_adjtimex_modes(txc, ts, time_tai);

		txc->offset = shift_right(time_offset * NTP_INTERVAL_FREQ,
				  NTP_SCALE_SHIFT);
		if (!(time_status & STA_NANO))
			txc->offset /= NSEC_PER_USEC;
	}

	result = time_state;	/* mostly `TIME_OK' */
	/* check for errors */
	if (is_error_status(time_status))
		result = TIME_ERROR;

	txc->freq	   = shift_right((time_freq >> PPM_SCALE_INV_SHIFT) *
					 PPM_SCALE_INV, NTP_SCALE_SHIFT);
	txc->maxerror	   = time_maxerror;
	txc->esterror	   = time_esterror;
	txc->status	   = time_status;
	txc->constant	   = time_constant;
	txc->precision	   = 1;
	txc->tolerance	   = MAXFREQ_SCALED / PPM_SCALE;
	txc->tick	   = tick_usec;
	txc->tai	   = *time_tai;

	/* fill PPS status fields */
	pps_fill_timex(txc);

	txc->time.tv_sec = (time_t)ts->tv_sec;
	txc->time.tv_usec = ts->tv_nsec;
	if (!(time_status & STA_NANO))
		txc->time.tv_usec /= NSEC_PER_USEC;

	/* Handle leapsec adjustments */
	if (unlikely(ts->tv_sec >= ntp_next_leap_sec)) {
		if ((time_state == TIME_INS) && (time_status & STA_INS)) {
			result = TIME_OOP;
			txc->tai++;
			txc->time.tv_sec--;
		}
		if ((time_state == TIME_DEL) && (time_status & STA_DEL)) {
			result = TIME_WAIT;
			txc->tai--;
			txc->time.tv_sec++;
		}
		if ((time_state == TIME_OOP) &&
					(ts->tv_sec == ntp_next_leap_sec)) {
			result = TIME_WAIT;
		}
	}

	return result;
}

#ifdef	CONFIG_NTP_PPS

/* actually struct pps_normtime is good old struct timespec, but it is
 * semantically different (and it is the reason why it was invented):
 * pps_normtime.nsec has a range of ( -NSEC_PER_SEC / 2, NSEC_PER_SEC / 2 ]
 * while timespec.tv_nsec has a range of [0, NSEC_PER_SEC) */
struct pps_normtime {
	s64		sec;	/* seconds */
	long		nsec;	/* nanoseconds */
};

/* normalize the timestamp so that nsec is in the
   ( -NSEC_PER_SEC / 2, NSEC_PER_SEC / 2 ] interval */
static inline struct pps_normtime pps_normalize_ts(struct timespec64 ts)
{
	struct pps_normtime norm = {
		.sec = ts.tv_sec,
		.nsec = ts.tv_nsec
	};

	if (norm.nsec > (NSEC_PER_SEC >> 1)) {
		norm.nsec -= NSEC_PER_SEC;
		norm.sec++;
	}

	return norm;
}

/* get current phase correction and jitter */
static inline long pps_phase_filter_get(long *jitter)
{
	unsigned prev = (pps_tf_pos + PPS_FILTER_SIZE - 1) % PPS_FILTER_SIZE;
	*jitter = abs(pps_tf[pps_tf_pos] - pps_tf[prev]);

	/* TODO: test various filters */
	return pps_tf[pps_tf_pos];
}

/* add the sample to the phase filter */
static inline void pps_phase_filter_add(long err)
{
	pps_tf_pos = (pps_tf_pos + 1) % PPS_FILTER_SIZE;
	pps_tf[pps_tf_pos] = err;
}

static inline void pps_freq_ring_add(long freq)
{
	freq_ring_pos = (freq_ring_pos + 1)%PPS_FILTER_SIZE;
	freq_ring[freq_ring_pos] = freq;
}

/* get smart value from the phase filter */
static inline long pps_phase_filter_smart(long *jitter)
{
	unsigned i;
	long res = LONG_MAX;
	unsigned prev = (pps_tf_pos + PPS_FILTER_SIZE - 1) % PPS_FILTER_SIZE;
	for (i = 0; i < PPS_FILTER_SIZE; i++) {
		res = min(res, abs(pps_tf[i]));
	}

	res = pps_tf[prev] > 0 ? res : -res;
	*jitter = abs(pps_tf[prev] - pps_tf[pps_tf_pos]);
	return res;
}

static inline long pps_phase_filter_average(long *jitter)
{
	long res;
	int i;
	unsigned prev = (pps_tf_pos + PPS_FILTER_SIZE - 1) % PPS_FILTER_SIZE;
	*jitter = abs(pps_tf[pps_tf_pos] - pps_tf[prev]);
	res = 0;
	for (i = 0; i < PPS_FILTER_SIZE; ++i) {
		res += pps_tf[i];
	}
	res = abs(res);
	res = pps_tf[prev] > 0 ? res : -res;
	return res / PPS_FILTER_SIZE;
}

static inline long pps_phase_filter_median(long *jitter)
{
	static long sorted[PPS_FILTER_SIZE];
	int i, j;
	long tmp, res;
	unsigned prev = (pps_tf_pos + PPS_FILTER_SIZE - 1) % PPS_FILTER_SIZE;
	*jitter = abs(pps_tf[pps_tf_pos] - pps_tf[prev]);
	memcpy(sorted, pps_tf, PPS_FILTER_SIZE * sizeof(long));
	for (i = 0; i < PPS_FILTER_SIZE; ++i) {
		for (j = 0; j < PPS_FILTER_SIZE - 1 - i; ++j) {
			if (sorted[j] > sorted[j + 1]) {
				tmp = sorted[j];
				sorted[j] = sorted[j + 1];
				sorted[j + 1] = tmp;
			}
		}
	}
	res = abs(sorted[PPS_FILTER_SIZE / 2]);
	res = pps_tf[prev] > 0 ? res : -res; 	
	return res;
}

static int __init setup_kernel_param_kalman_filter_algo(char *str) {
	if (!strcmp(str, "smart")) {
		printk(KERN_ALERT "Kalman smart choosen\n");
		pps_phase_filter_ptr = pps_phase_filter_smart;
	} else if (!strcmp(str, "last")) {
		printk(KERN_ALERT "Kalman last choosen\n");
		pps_phase_filter_ptr = pps_phase_filter_get;
	} else if (!strcmp(str, "median")) {
		printk(KERN_ALERT "Kalman median choosen\n");
		pps_phase_filter_ptr = pps_phase_filter_median;
	} else if (!strcmp(str, "average")) {
		printk(KERN_ALERT "Kalman average choosen\n");
		pps_phase_filter_ptr = pps_phase_filter_average;
	}
	return 0;
}

early_param("kalman_filter_algo", setup_kernel_param_kalman_filter_algo);

static inline long pps_phase_filter_impl(long *jitter)
{
	if (pps_phase_filter_ptr) {
		return pps_phase_filter_ptr(jitter);
	} else return pps_phase_filter_smart(jitter);
}

/* decrease (half) frequency calibration interval length.
 */
static inline void pps_dec_freq_interval(void)
{
	if (pps_shift > PPS_INTMIN) {
		pps_shift--;
	}
	pps_intcnt = 0;
}

/* increase frequency calibration interval length.
 * It is doubled after four consecutive stable intervals.
 */
static inline void pps_inc_freq_interval(void)
{
	if (++pps_intcnt >= PPS_INTCOUNT) {
		pps_intcnt = PPS_INTCOUNT;
		if (pps_shift < PPS_INTMAX) {
			pps_shift++;
			pps_intcnt = 0;
		}
	}
}

/* update clock frequency based on MONOTONIC_RAW clock PPS signal
 * timestamps
 *
 * At the end of the calibration interval the difference between the
 * first and last MONOTONIC_RAW clock timestamps divided by the length
 * of the interval becomes the frequency update. If the interval was
 * too long, the data are discarded.
 * Returns the difference between old and new frequency values.
 */
static long hardpps_update_freq(struct pps_normtime freq_norm)
{
	long delta;
	s64 ftemp;

	/* check if the frequency interval was too long */
	if (freq_norm.sec > (2 << pps_shift)) {
		printk(KERN_DEBUG "hardpps_update_freq: frequency interval was too long\n");
		time_status |= STA_PPSERROR;
		pps_errcnt++;
		pps_dec_freq_interval();
		printk_deferred(KERN_ERR
			"hardpps: PPSERROR: interval too long - %lld s\n",
			freq_norm.sec);
		return 0;
	}
	printk(KERN_DEBUG "hardpps_update_freq: frequency interval is normal\n");

	/* here the raw frequency offset and wander (stability) is
	 * calculated. If the wander is less than the wander threshold
	 * the interval is increased; otherwise it is decreased.
	 */
	ftemp = div_s64(((s64)(-freq_norm.nsec)) << NTP_SCALE_SHIFT,
			freq_norm.sec);
	delta = shift_right(ftemp - pps_freq, NTP_SCALE_SHIFT);
	pps_freq = ftemp;
	printk(KERN_DEBUG "hardpps_update_freq: delta: %ld, freq_norm.nsec = %ld, freq_norm.sec = %lld\n",
		delta, freq_norm.nsec, freq_norm.sec);
	if (abs(delta) > PPS_MAXWANDER) {
		printk_deferred(KERN_WARNING
				"hardpps: PPSWANDER: change=%ld\n", delta);
		time_status |= STA_PPSWANDER;
		pps_stbcnt++;
		pps_dec_freq_interval();
		printk(KERN_DEBUG "hardpps_update_freq: bad sample\n");
	} else if (!freq_is_unstable) {	/* good sample */
		printk(KERN_DEBUG "hardpps_update_freq: good sample\n");
		pps_inc_freq_interval();
	}

	/* the stability metric is calculated as the average of recent
	 * frequency changes, but is used only for performance
	 * monitoring
	 */
	pps_stabil += (div_s64(((s64)abs(delta)) <<
				(NTP_SCALE_SHIFT - SHIFT_USEC),
				NSEC_PER_USEC) - pps_stabil) >> PPS_INTMIN;

	/* if enabled, the system clock frequency is updated */
	if ((time_status & STA_PPSFREQ) != 0 &&
	    (time_status & STA_FREQHOLD) == 0) {
		time_freq = pps_freq;
		ntp_update_frequency();
	}

	return delta;
}

static int get_inversions_cnt(int *all) {
	int i, j;
	int res;
	if (all) *all = 0;
	for (i = (pps_tf_pos + 1)%PPS_FILTER_SIZE,
		res = 0; i != pps_tf_pos; i = (i + 1)%PPS_FILTER_SIZE)
	{
		for (j = (i + 1)%PPS_FILTER_SIZE; j != pps_tf_pos;
			j = (j + 1)%PPS_FILTER_SIZE)
		{
			if (abs(pps_tf[i]) >= abs(pps_tf[j])
				- freq_invers_admis_error) ++res;
			if (all) *all += 1;
		}
		if (abs(pps_tf[i]) >= abs(pps_tf[pps_tf_pos])
			- freq_invers_admis_error) ++res;
		if (all) *all += 1;
	}
	printk(KERN_ALERT "Inversions cnt: %d\n", res);
	return res;
}

static long get_distance_to_freqs(long freq) {
	int i;
	long tmp;
	long res = abs(freq_ring[0] - freq);
	for (i = 1; i < PPS_FILTER_SIZE; ++i) {
		tmp = abs(freq_ring[i] - freq);
		if (tmp < res) res = tmp;
	}
	printk(KERN_ALERT "get_distance_to_freqs: need=%ld, get=%ld", freq, res);
	return res;
}

/* Return true if frequency is unstable */
static char check_freq_unstable(void) {
	int all_inv;
	int inv;
	if (freq_unstable_mask & FREQ_UNSTABLE_INVERSIONS) {
		inv = get_inversions_cnt(&all_inv);
		if (inv * 100 < inversions_percent * all_inv) {
			/* Too small inversions count - phase error is increasing */
			freq_is_unstable = 1;
			return 1;
		}
	}
	if (freq_unstable_mask & FREQ_UNSTABLE_LAST_RING) {
		if (get_distance_to_freqs(pps_freq >> NTP_SCALE_SHIFT) >
			freq_unstable_distance)
		{
			/* Too big distance to frequency ring - too big change of freq */
			freq_is_unstable = 1;
			return 1;
		}
	}
	freq_is_unstable = 0;
	return 0;
}

/* correct REALTIME clock phase error against PPS signal */
static void hardpps_update_phase(long error)
{
	long correction = -error;
	long jitter;

	/* add the sample to the median filter */
	pps_phase_filter_add(correction);
	correction = pps_phase_filter_impl(&jitter);

	/* Nominal jitter is due to PPS signal noise. If it exceeds the
	 * threshold, the sample is discarded; otherwise, if so enabled,
	 * the time offset is updated.
	 */
	if (pps_jitter && (jitter > ((long long)pps_jitter << PPS_POPCORN))) {
		printk_deferred(KERN_WARNING
				"hardpps: PPSJITTER: jitter=%ld, limit=%ld\n",
				jitter, (pps_jitter << PPS_POPCORN));
		time_status |= STA_PPSJITTER;
		pps_jitcnt++;
	} else if (time_status & STA_PPSTIME) {
		/* correct the time using the phase offset */
		time_offset = div_s64(((s64)correction) << NTP_SCALE_SHIFT,
				NTP_INTERVAL_FREQ);
		/* cancel running adjtime() */
		time_adjust = 0;
	}
	/* update jitter */
	pps_jitter += (jitter - pps_jitter) >> PPS_INTMIN;
}

#ifdef CONFIG_PPS_PROFILER

static void profiler_ring_init(void) {
	int one_size;
	int all_size;
	int pages_cnt;

	profiler_ring_size = 0;
	profiler_ring_end = 0;
	profiler_ring_begin = 0;
	profiler_iterator = 0;

	if (ring_pages != NULL) {
		__free_pages(ring_pages, ring_pages_order);
		ring_pages = NULL;
	}
	if (ring_shot_pages != NULL) {
		__free_pages(ring_shot_pages, ring_pages_order);
		ring_shot_pages = NULL;
	}
	ring_pages_order = 0;
	ring_pages_cnt = 1;
	one_size = sizeof(profiler_log_entry);
	all_size = one_size * PROFILER_RING_SIZE;
	pages_cnt = all_size / PAGE_SIZE + 1;
	while (ring_pages_cnt < pages_cnt) {
		++ring_pages_order;
		ring_pages_cnt = ring_pages_cnt << 1;
	}
	ring_pages = alloc_pages(GFP_DMA, ring_pages_order);
	profiler_ring = page_address(ring_pages);
	memset(profiler_ring, 0, PAGE_SIZE * ring_pages_cnt);

	ring_shot_pages = alloc_pages(GFP_DMA, ring_pages_order);
	profiler_ring_shot = page_address(ring_shot_pages);
	memset(profiler_ring_shot, 0, PAGE_SIZE * ring_pages_order);
	printk(KERN_ALERT "profiler_ring_init(): ring is inited\n");
}

void push_entry_to_ring(profiler_log_entry *entry) {
#ifdef PROFILER_MAX_ITERS
	if (profiler_iterator >= PROFILER_MAX_ITERS) return;
#endif
	entry->id = profiler_iterator;
	if (profiler_ring_size < PROFILER_RING_SIZE) {
		if (profiler_ring_size != 0) {
			profiler_ring_end = ((int)profiler_ring_end + 1) % (int)PROFILER_RING_SIZE;
			profiler_ring[profiler_ring_end] = *entry;
			++profiler_ring_size;
			if (profiler_ring_end == profiler_ring_begin) {
				++profiler_ring_begin;
				profiler_ring_size = PROFILER_RING_SIZE;
			}
		} else {
			profiler_ring_init();
			profiler_ring[0] = *entry;
			profiler_ring_size = 1;
		}
	} else {
		profiler_ring_begin = ((int)profiler_ring_begin + 1) % (int)PROFILER_RING_SIZE;
		profiler_ring_end = ((int)profiler_ring_end + 1) % (int)PROFILER_RING_SIZE;
		profiler_ring[profiler_ring_end] = *entry;
	}
	++profiler_iterator;
#ifdef PPS_PROFILER_DEBUG
	printk(KERN_ALERT "push_entry_to_ring(): size = %d, begin = %d, end = %d\n", (int)profiler_ring_size, (int)profiler_ring_begin, (int)profiler_ring_end);
#endif
}

void add_pair_to_entry(profiler_log_entry *entry,
	const struct timespec64 *phase_ts, const struct timespec64 *raw_ts) {
	entry->phase_ts = *phase_ts;
	entry->raw_ts = *raw_ts;
}

#endif

/*
 * __hardpps() - discipline CPU clock oscillator to external PPS signal
 *
 * This routine is called at each PPS signal arrival in order to
 * discipline the CPU clock oscillator to the PPS signal. It takes two
 * parameters: REALTIME and MONOTONIC_RAW clock timestamps. The former
 * is used to correct clock phase error and the latter is used to
 * correct the frequency.
 *
 * This code is based on David Mills's reference nanokernel
 * implementation. It was mostly rewritten but keeps the same idea.
 */
void __hardpps(const struct timespec64 *phase_ts, const struct timespec64 *raw_ts)
{
	static const char *__func_name = "__hardpps";
	struct pps_normtime pts_norm, freq_norm;
	long jitter;
	struct timespec64 raw_time = *raw_ts;
	struct timespec64 real_time = *phase_ts, buf_timespec1, buf_timespec2;
	int j, t;
	long phase_filer_res = pps_phase_filter_impl(&jitter);
#ifdef CONFIG_PPS_DEBUG
	printk(KERN_DEBUG "phase_filer_res: %ld\n", phase_filer_res);
#endif

	if (!kalman_force_stop && !kalman_is_inited && ((kalman_start_offset_core_param <= 0) ||
		(kalman_start_offset_core_param >= abs(phase_filer_res)))) {
		PPSKalman_init(&kalman_filter, timespec64_to_ns(raw_ts), timespec64_to_ns(phase_ts),
			((s64)1) << (NTP_SCALE_SHIFT - 2), ((s64)1) << (NTP_SCALE_SHIFT - 2), //current probe estimate is 1
			kalman_q_core_param,
			kalman_q_core_param,
			div_s64((((s64)1) << (NTP_SCALE_SHIFT - 2)), 10),  //r11 = 0.1
			div_s64((((s64)1) << (NTP_SCALE_SHIFT - 2)), 10)); //r22 = 0.1
		kalman_is_inited = 1;
#ifdef CONFIG_PPS_DEBUG
		printk(KERN_DEBUG "Kalman filter is inited\n");
#endif
	} else if (!kalman_force_stop) {
		PPSKalman_Step(&kalman_filter, NSEC_PER_SEC,
			NSEC_PER_SEC, timespec64_to_ns(raw_ts),
			timespec64_to_ns(phase_ts));
#ifdef CONFIG_PPS_DEBUG
		printk(KERN_DEBUG "kalman raw: %lld, kalman real: %lld\n", kalman_filter.cse11, kalman_filter.cse21);
		printk(KERN_DEBUG "kraw - original: %lld, kreal - original: %lld\n", kalman_filter.cse11 - timespec64_to_ns(raw_ts),
			kalman_filter.cse21 - timespec64_to_ns(phase_ts));
		printk(KERN_DEBUG "nsecs from last: real = %lld, raw = %lld\n",
			timespec64_to_ns(phase_ts) - timespec64_to_ns(&phase_ts_prev),
			timespec64_to_ns(raw_ts) - timespec64_to_ns(&raw_ts_prev));
#endif
		raw_time = ns_to_timespec64(kalman_filter.cse11);
		real_time = ns_to_timespec64(kalman_filter.cse21);
	}
#ifdef CONFIG_PPS_DEBUG
	buf_timespec1 = timespec64_sub(*phase_ts, phase_ts_prev);
	buf_timespec2 = timespec64_sub(*raw_ts, raw_ts_prev);
#endif
	if (raw_ts_prev.tv_nsec) {
		pps_freq_ring_add(NSEC_PER_SEC - timespec64_to_ns(&buf_timespec2));
		printk(KERN_ALERT "Add to freq ring: %ld\n", freq_ring[freq_ring_pos]);
	}
	phase_ts_prev = *phase_ts;
	raw_ts_prev = *raw_ts;

#ifdef CONFIG_PPS_PROFILER
	spin_lock(&profiler_ring_lock);

	t = (int)(profiler_ring_end + 1) % PROFILER_RING_SIZE;
	if (t % ENTRIES_PER_DUMP == 0) {
		j = t / ENTRIES_PER_DUMP;
		dumps_size = (dumps_size + 1 >= DUMPS_CNT) ? dumps_size : (dumps_size + 1);
		get_vars_to_dump(dumps + j);
	} else if (!profiler_ring_size) {
		get_vars_to_dump(dumps);
	}
#ifdef PPS_PROFILER_DEBUG
	printk(KERN_ALERT "phase_ts: %llu, raw_ts: %llu\n", timespec64_to_ns(phase_ts), timespec64_to_ns(raw_ts));
	printk(KERN_ALERT "phase_ts-diff: %lld, raw_ts-diff: %lld, raw-diff - 1s: %lld\n", timespec64_to_ns(&buf_timespec1),
		timespec64_to_ns(&buf_timespec2), NSEC_PER_SEC - timespec64_to_ns(&buf_timespec2));
	printk(KERN_ALERT "tick_usec: %lu, tick_nsec: %lu, tick_length: %llu, tick_length_base: %llu\n", tick_usec, tick_nsec,
		tick_length, tick_length_base);
	printk(KERN_ALERT "time_state: %d, time_status: %d, time_offset: %lld, time_constant: %ld\n", time_state, time_status,
		time_offset, time_constant);
	printk(KERN_ALERT "time_maxerror: %ld, time_esterror: %ld, time_freq: %lld, time_reftime: %ld\n", time_maxerror,
		time_esterror, time_freq, time_reftime);
	printk(KERN_ALERT "time_adjust: %ld, ntp_tick_adj: %lld, pps_valid: %d, pps_tf: \n", time_adjust, ntp_tick_adj,
		pps_valid);
	for (j = 0; j < PPS_FILTER_SIZE; ++j) {
		printk(KERN_ALERT "[%d]: %ld, ", j, pps_tf[j]);
	}
	printk(KERN_ALERT "pps_tf_pos: %u, pps_jitter: %ld, pps_fbase: %lld, pps_shift: %d\n", pps_tf_pos, pps_jitter,
		timespec64_to_ns(&pps_fbase), pps_shift);
	printk(KERN_ALERT "pps_intcnt: %d, pps_freq: %lld, pps_stabil: %ld, pps_calcnt: %ld\n", pps_intcnt, pps_freq,
		pps_stabil, pps_calcnt);
	printk(KERN_ALERT "pps_jitcnt: %ld, pps_stbcnt: %ld, pps_errcnt: %ld\n", pps_jitcnt, pps_stbcnt, pps_errcnt);
#endif
	printk(KERN_ALERT "freq_pos: %d, freq_ring: \n", freq_ring_pos);
	for (j = 0; j < PPS_FILTER_SIZE; ++j) {
		printk(KERN_ALERT "[%d]: %ld, ", j, freq_ring[j]);
	}
	pr_warning("%s: correction = %ld\n", __func_name, phase_filer_res);
	/* clear log entry for new log record creating */
	clear_log_entry(&cur_entry);

	/* add current timestamps to log entry */
	add_pair_to_entry(&cur_entry, phase_ts, raw_ts);
#endif

	pts_norm = pps_normalize_ts(real_time);

	/* clean the error bits, they will be set again if needed */
	time_status &= ~(STA_PPSJITTER | STA_PPSWANDER | STA_PPSERROR);

	/* indicate signal presence */
	time_status |= STA_PPSSIGNAL;
	pps_valid = PPS_VALID;

	/* when called for the first time,
	 * just start the frequency interval */
	if (unlikely(pps_fbase.tv_sec == 0)) {
		pps_fbase = raw_time;
#ifdef CONFIG_PPS_PROFILER
		push_entry_to_ring(&cur_entry);
		spin_unlock(&profiler_ring_lock);
#endif
		memset(freq_ring, 0, sizeof(long) * PPS_FILTER_SIZE);
		return;
	}

	/* ok, now we have a base for frequency calculation */
	freq_norm = pps_normalize_ts(timespec64_sub(raw_time, pps_fbase));
#ifdef PPS_DEBUG
	pr_warning("pts_norm={%ld,%ld} raw_time.nsec=%ld freq_norm={%ld,%ld}\n", pts_norm.sec, pts_norm.nsec, raw_time.tv_nsec, freq_norm.sec, freq_norm.nsec);
	printk(KERN_DEBUG "pts_norm={%ld,%ld} raw_time.nsec=%ld freq_norm={%ld,%ld}\n", pts_norm.sec, pts_norm.nsec, raw_time.tv_nsec, freq_norm.sec, freq_norm.nsec);
#endif
	/* check that the signal is in the range
	 * [1s - MAXFREQ us, 1s + MAXFREQ us], otherwise reject it */
	if (abs(freq_norm.nsec) > MAXFREQ * freq_norm.sec) {
		printk(KERN_DEBUG "Freq signal is not in range, reject\n");
		time_status |= STA_PPSJITTER;
		/* restart the frequency calibration interval */
		pps_fbase = raw_time;
		pps_dec_freq_interval();
		pr_warning("hardpps: PPSJITTER: bad pulse, freq_norm={%lld,%ld}\n", freq_norm.sec, freq_norm.nsec);
#ifdef CONFIG_PPS_PROFILER
		push_entry_to_ring(&cur_entry);
		spin_unlock(&profiler_ring_lock);
#endif
		return;
	}
	printk(KERN_DEBUG "frequency signal is ok");

	/* signal is ok */

	/* check if frequency signal is unstable */
	if (check_freq_unstable()) {
		/* apply current interval, decrease length and start new */
		printk(KERN_ALERT "frequency is unstable\n");
		pps_fbase = raw_time;
		pps_calcnt++;
		hardpps_update_freq(freq_norm);
		pps_dec_freq_interval();
	} else
	/* check if the current frequency interval is finished */
	if (freq_norm.sec >= (1 << pps_shift)) {
		printk(KERN_DEBUG "frequency interval is finished\n");
		pps_calcnt++;
		/* restart the frequency calibration interval */
		pps_fbase = raw_time;
		hardpps_update_freq(freq_norm);
	}

	hardpps_update_phase(pts_norm.nsec);
#ifdef CONFIG_PPS_PROFILER
	push_entry_to_ring(&cur_entry);
	spin_unlock(&profiler_ring_lock);
#endif
}
#endif	/* CONFIG_NTP_PPS */

static int __init ntp_tick_adj_setup(char *str)
{
	int rc = kstrtol(str, 0, (long *)&ntp_tick_adj);

	if (rc)
		return rc;
	ntp_tick_adj <<= NTP_SCALE_SHIFT;

	return 1;
}

__setup("ntp_tick_adj=", ntp_tick_adj_setup);

#ifdef CONFIG_PPS_PROFILER

static void *pps_seq_start(struct seq_file *f, loff_t *pos) {
#ifdef PPS_PROFILER_DEBUG
	printk(KERN_ALERT "pps_seq_start(): pos = %d\n", (int)*pos);
#endif
	if (*pos >= profiler_ring_copy_size) return NULL;
	return pos;
}

static void *pps_seq_next(struct seq_file *f, void *v, loff_t *pos) {
#ifdef PPS_PROFILER_DEBUG
	printk(KERN_ALERT "pps_seq_next(): pos = %d, size = %d\n", (int)*pos, (int)profiler_ring_copy_size);
#endif
	if (++(*pos) >= profiler_ring_copy_size) return NULL;
	return pos;
}

static void pps_seq_stop(struct seq_file *f, void *v) {
#ifdef PPS_PROFILER_DEBUG
	printk(KERN_ALERT "pps_seq_stop():\n");
#endif
}

static int pps_seq_show(struct seq_file *f, void *v) {
	loff_t index;
	profiler_log_entry *ob;
	int i;

	index = *(loff_t *)v;
	ob = &(profiler_ring_shot[index]);
#ifdef PPS_PROFILER_DEBUG
	printk(KERN_ALERT "pps_seq_show(): index = %d\n", (int)index);
#endif
	if (index == 0) {
		seq_printf(f, "%lu %lu %llu %llu %d %d %lld %ld %ld %ld %lld %ld %ld ", shot_dump.shot_tick_usec,
			shot_dump.shot_tick_nsec, shot_dump.shot_tick_length, shot_dump.shot_tick_length_base,
			shot_dump.shot_time_state, shot_dump.shot_time_status, shot_dump.shot_time_offset,
			shot_dump.shot_time_constant, shot_dump.shot_time_maxerror, shot_dump.shot_time_esterror,
			shot_dump.shot_time_freq, shot_dump.shot_time_reftime, shot_dump.shot_time_adjust);
		seq_printf(f, "%lld %d ", shot_dump.shot_ntp_tick_adj, shot_dump.shot_pps_valid);
		for (i = 0; i < PPS_FILTER_SIZE; ++i) {
			seq_printf(f, "%ld ", shot_dump.shot_pps_tf[i]);
		}
		seq_printf(f, "%u %ld %lld %d %d %lld %ld %ld %ld %ld %ld\n", shot_dump.shot_pps_tf_pos,
			shot_dump.shot_pps_jitter, timespec64_to_ns(&(shot_dump.shot_pps_fbase)), shot_dump.shot_pps_shift,
			shot_dump.shot_pps_intcnt, shot_dump.shot_pps_freq, shot_dump.shot_pps_stabil,
			shot_dump.shot_pps_calcnt, shot_dump.shot_pps_jitcnt, shot_dump.shot_pps_stbcnt, shot_dump.shot_pps_errcnt);
	}
	seq_printf(f, "%lu # %llu %llu #\n", ob->id, timespec64_to_ns(&(ob->phase_ts)), timespec64_to_ns(&(ob->raw_ts)));
	return 0;
}

static struct seq_operations pps_seq_ops = {
	.start = pps_seq_start,
	.next = pps_seq_next,
	.stop = pps_seq_stop,
	.show = pps_seq_show
};

void get_vars_to_dump(profiler_vars_shot *dump) {
	dump->shot_tick_usec = tick_usec;
	dump->shot_tick_nsec = tick_nsec;
	dump->shot_tick_length = tick_length;
	dump->shot_tick_length_base = tick_length_base;
	dump->shot_time_state = time_state;
	dump->shot_time_status = time_status;
	dump->shot_time_offset = time_offset;
	dump->shot_time_constant = time_constant;
	dump->shot_time_maxerror = time_maxerror;
	dump->shot_time_esterror = time_esterror;
	dump->shot_time_freq = time_freq;
	dump->shot_time_reftime = time_reftime;
	dump->shot_time_adjust = time_adjust;
	dump->shot_ntp_tick_adj = ntp_tick_adj;
	dump->shot_pps_valid = pps_valid;
	dump->shot_pps_tf_pos = pps_tf_pos;
	dump->shot_pps_jitter = pps_jitter;
	dump->shot_pps_fbase = pps_fbase;
	dump->shot_pps_shift = pps_shift;
	dump->shot_pps_intcnt = pps_intcnt;
	dump->shot_pps_freq = pps_freq;
	dump->shot_pps_stabil = pps_stabil;
	dump->shot_pps_calcnt = pps_calcnt;
	dump->shot_pps_jitcnt = pps_jitcnt;
	dump->shot_pps_stbcnt = pps_stbcnt;
	dump->shot_pps_errcnt = pps_errcnt;
	memcpy(dump->shot_pps_tf, pps_tf, PPS_FILTER_SIZE * sizeof(long));
} 

static int pps_profiler_open(struct inode *inode, struct file *file) {
	int i;
	int j;
	int start;
#ifdef PPS_PROFILER_DEBUG
	printk(KERN_ALERT "pps_profiler_open():\n");
#endif
	spin_lock(&profiler_ring_lock);

	start = profiler_ring_begin;
	for (i = profiler_ring_begin; i != profiler_ring_end; i = (i + 1) % PROFILER_RING_SIZE) {
		if (i % ENTRIES_PER_DUMP == 0) {
			start = i;
			shot_dump = dumps[i / ENTRIES_PER_DUMP];
			break;
		}
	}
	for (i = start, j = 0; i != profiler_ring_end; i = (i + 1) % PROFILER_RING_SIZE) {
		profiler_ring_shot[j++] = profiler_ring[i];
	}
	if (i == profiler_ring_end) {
		profiler_ring_shot[j] = profiler_ring[i];
	}
	profiler_ring_copy_size = j + 1;
	printk(KERN_ALERT "profiler_ring_copy_size: %d\n", (int)profiler_ring_copy_size);

	spin_unlock(&profiler_ring_lock);

	return seq_open(file, &pps_seq_ops);
}

static struct file_operations pps_proc_ops = {
	.owner = THIS_MODULE,
	.open = pps_profiler_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release
};

EXPORT_SYMBOL(set_vars_from_dump);
#endif

static int kalman_open(struct inode *nodep, struct file *filep) {
	kalman_open_cnt++;
	printk(KERN_DEBUG "Kalman ctl was opened\n");
	return 0;
}

static int kalman_release(struct inode *nodep, struct file *filep) {
	--kalman_open_cnt;
	printk(KERN_DEBUG "Kalman ctl was released\n");
	return 0;
}

static ssize_t kalman_write(struct file *filep,
	const char *buf, size_t len, loff_t *offset)
{
	char buffer[KalmanBufferSize];
	memset(buffer, 0, sizeof(buffer));
	if (KalmanBufferSize < len) {
		printk(KERN_ALERT "Attempt to write in kalman too big data\n");
		return 0;
	}
	if (copy_from_user(buffer, buf, len)) {
		printk(KERN_ALERT "Error while copying from user in kalman_write\n");
		return -EFAULT;
	}
	*offset += len;
	if (!strncmp(buffer, "stop", 4)) {
		PPSKalman_stop();
	} else if (!strncmp(buffer, "start", 5)) {
		PPSKalman_start();
	} else if (!strncmp(buffer, "restart", 7)) {
		PPSKalman_restart();
	} else {
		printk(KERN_ALERT "Unknown command for kalman ctl: %s\n", buffer);
	}
	return len;
}

static struct file_operations kalman_proc_ops = {
	.owner = THIS_MODULE,
	.open = kalman_open,
	.write = kalman_write,
	.release = kalman_release
};

void __init ntp_init(void)
{
#ifdef CONFIG_PPS_PROFILER
	printk(KERN_ALERT "ntp_init():\n");
	/* create proc file pps_profiler */
	ring_entry = proc_create("pps_profiler", 0, NULL, &pps_proc_ops);
	printk(KERN_ALERT "pps_profiler file was created\n");
	profiler_ring_init();
#endif

	ntp_clear();

	kalman_proc = proc_create(KalmanProcName, 0666, NULL, &kalman_proc_ops);
	printk(KERN_ALERT "Kalman filter proc file was created\n");
	memset(&raw_ts_prev, 0, sizeof(struct timespec64));
	phase_ts_prev = raw_ts_prev;
}

// Kalman filter implementation

void PPSKalman_init(PPSKalman *ob, s64 cse11, s64 cse21,
	s64 cpe11, s64 cpe22, s64 q11, s64 q22, s64 r11, s64 r22)
{
	ob->cse11 = cse11;
	ob->cse21 = cse21;
	ob->cpe11 = cpe11;
	ob->cpe22 = cpe22;
	ob->q11 = q11;
	ob->q22 = q22;
	ob->r11 = r11;
	ob->r22 = r22;
}

s64 kalman_step_update_cse_impl(s64 cse, s64 cv, s64 m, s64 cpe, s64 q, s64 r)
{
	s64 tmp1, tmp2, tmp3, right;
	char sign;
	u32 l, h;
	tmp1 = cse + cv;
	tmp3 = m - cse - cv;
	tmp2 = div_s64(((cpe << NTP_SCALE_SHIFT) + (q << NTP_SCALE_SHIFT)),
		(cpe + q + r));
	if (((tmp2 > 0) && (tmp3 > 0)) || ((tmp2 < 0) && (tmp3 < 0))) sign = 1;
	else sign = -1;
	if (tmp2 < 0) tmp2 *= -1;
	if (tmp3 < 0) tmp3 *= -1;
	l = tmp3 & 0xffffffff;
	h = (tmp3 >> NTP_SCALE_SHIFT) & 0xffffffff;
	right = tmp2 * h + (((tmp2 * l) >> NTP_SCALE_SHIFT) & 0xffffffff);
	right *= sign;
	return right + tmp1;
}

inline s64 kalman_step_update_cpe_impl(s64 r, s64 cpe, s64 q)
{
	return (r * ((((s64)1) << NTP_SCALE_SHIFT) -
		div_s64((r * (((s64)1) << NTP_SCALE_SHIFT)),
		(cpe + q + r)))) >> NTP_SCALE_SHIFT;
}

void PPSKalman_Step(PPSKalman *ob, s64 cv11, s64 cv21, s64 m11, s64 m21)
{
	ob->cse11 = kalman_step_update_cse_impl(ob->cse11, cv11,
		m11, ob->cpe11, ob->q11, ob->r11);
	ob->cse21 = kalman_step_update_cse_impl(ob->cse21, cv21,
		m21, ob->cpe22, ob->q22, ob->r22);

	ob->cpe11 = kalman_step_update_cpe_impl(ob->r11, ob->cpe11, ob->q11);
	ob->cpe22 = kalman_step_update_cpe_impl(ob->r22, ob->cpe22, ob->q22);
}

void PPSKalman_stop() {
	kalman_force_stop = 1;
	kalman_is_inited = 0;
	printk(KERN_ALERT "Kalman is stopped\n");
}

void PPSKalman_start() {
	kalman_force_stop = 0;
	printk(KERN_ALERT "Kalman is started\n");
}

void PPSKalman_restart() {
	kalman_is_inited = 0;
	kalman_force_stop = 0;
	printk(KERN_ALERT "Kalman is restarted\n");
}
