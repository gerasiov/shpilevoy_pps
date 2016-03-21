#ifndef _LINUX_PPS_PROFILER_UTILS_H
#define _LINUX_PPS_PROFILER_UTILS_H

#ifdef CONFIG_NTP_PPS
#define PPS_FILTER_SIZE	4
#endif

#ifdef CONFIG_PPS_PROFILER

#define PROFILER_RING_SIZE 10000
//#define MAX_LOG_ENTRY_SIZE 10
#define ENTRIES_PER_DUMP 100
#define DUMPS_CNT (PROFILER_RING_SIZE / ENTRIES_PER_DUMP + 1)
#define PPS_PROFILER_DEBUG 1

/* Types of profiler_logs */
typedef enum profiler_log {
	PRF_PPS_CLEAR
} profiler_log_t;

/* Struct contains information about all actions during one hardpps call. */
typedef struct profiler_log_entry {
	unsigned long id; //unic id of this entry
	struct timespec64 phase_ts; //timestamps from hardpps parameters
	struct timespec64 raw_ts;
} profiler_log_entry;

typedef struct profiler_vars_shot {
	unsigned long 	  shot_tick_usec;
	unsigned long 	  shot_tick_nsec;
	u64 			  shot_tick_length;
	u64 			  shot_tick_length_base;
	int				  shot_time_state;
	int 			  shot_time_status;
	s64 			  shot_time_offset;
	long 			  shot_time_constant;
	long 			  shot_time_maxerror;
	long 			  shot_time_esterror;
	s64 			  shot_time_freq;
	long 			  shot_time_reftime;
	long 			  shot_time_adjust;
	s64				  shot_ntp_tick_adj;
	int 			  shot_pps_valid;
	long 			  shot_pps_tf[PPS_FILTER_SIZE];
	unsigned 		  shot_pps_tf_pos;
	long 			  shot_pps_jitter;
	struct timespec64 shot_pps_fbase;
	int 			  shot_pps_shift;
	int 			  shot_pps_intcnt;
	s64 			  shot_pps_freq;
	long 			  shot_pps_stabil;
	long 			  shot_pps_calcnt;
	long 			  shot_pps_jitcnt;
	long 			  shot_pps_stbcnt;
	long 			  shot_pps_errcnt;
} profiler_vars_shot;

extern void set_vars_from_dump(profiler_vars_shot *shot);

#endif

#endif