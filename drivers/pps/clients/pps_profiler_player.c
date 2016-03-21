/*
 * pps_profiler_player.c -- player for pps-profiler logs
 *
 *
 * Copyright (C) 2015   Vladislav Shpilevoy <vshpilevoi@mail.ru>
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifdef CONFIG_PPS_PROFILER
#include <linux/module.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/pps_kernel.h>
#include <linux/string.h>
#include <asm/uaccess.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>

//#define PPS_PLAYER_DEBUG 1

#include "linux/pps-profiler-utils.h"

/*
 * Global variables
 */

static struct pps_device *pps = NULL;
static struct timer_list ktimer;


struct proc_dir_entry *log_entry = NULL;
profiler_log_entry player_ring[PROFILER_RING_SIZE];
char buffer[1024];
int buffer_it = 0;
int ring_iter = 0;
profiler_vars_shot vars_shot;
char vars_shot_inited = 0;

char ring_is_played = 1;


/*
 * The kernel timer
 */

static void pps_ktimer_event(unsigned long ptr)
{
	struct pps_event_time ts;
	profiler_log_entry *cur;
#ifdef PPS_PLAYER_DEBUG
	printk("pps_ktimer_event(), ring_is_played = %d\n", (int)ring_is_played);
#endif

	if (ring_is_played) {
		ts.ts_raw = ns_to_timespec64(0);
		ts.ts_real = ns_to_timespec64(0);
	} else {
		if (ring_iter >= PROFILER_RING_SIZE) {
			printk(KERN_ALERT "Playing ring stoped\n");
			ring_is_played = 1;
			ts.ts_raw = ns_to_timespec64(0);
			ts.ts_real = ns_to_timespec64(0);
		} else {
			cur = player_ring + ring_iter++;
			ts.ts_raw = cur->raw_ts;
			ts.ts_real = cur->phase_ts;
		}
	}
#ifdef PPS_PLAYER_DEBUG
	printk(KERN_ALERT "pps_player_event(): ts.ts_real: %llu, ts.ts_raw: %llu\n", timespec_to_ns(&(ts.ts_real)), timespec_to_ns(&(ts.ts_raw)));
#endif
	pps_event(pps, &ts, PPS_CAPTUREASSERT, NULL);

	mod_timer(&ktimer, jiffies + HZ);
}

/*
 * The PPS info struct
 */

static struct pps_source_info pps_ktimer_info = {
	.name		= "ktimer",
	.path		= "",
	.mode		= PPS_CAPTUREASSERT | PPS_OFFSETASSERT |
			  PPS_ECHOASSERT |
			  PPS_CANWAIT | PPS_TSFMT_TSPEC,
	.owner		= THIS_MODULE,
};

/*
 * Module staff
 */

void move_cursor_while_not(size_t *cursor, size_t size, const char *str, char sym) {
	while((*cursor < size) && (str[*cursor] != sym)) ++(*cursor);
}
void move_cursor_while_is(size_t *cursor, size_t size, const char *str, char sym) {
	while((*cursor < size) && (str[*cursor] == sym)) ++(*cursor);
}

int profiler_log_entry_from_string(profiler_log_entry *ob, const char *str, size_t size) {
	size_t cursor;
	char *end;
	unsigned long long first_ts;
	unsigned long long second_ts;
	int id;

	cursor = 0;
	end = NULL;
	move_cursor_while_not(&cursor, size, str, '#');
	if ((cursor >= size) || (str[cursor] != '#')) {
		printk(KERN_ALERT "Not found first #\n");
		return -1;
	}
	++cursor;
	id = simple_strtol(str, NULL, 0);

	move_cursor_while_is(&cursor, size, str, ' ');
	if ((cursor >= size) || (str[cursor] == ' ')) {
		printk(KERN_ALERT "Not found first ts\n");
		return -1;
	}
	first_ts = simple_strtoull(str + cursor, NULL, 0);
	if (first_ts == 0) {
		printk(KERN_ALERT "Incorrect first ts\n");
		return -1;
	}
	move_cursor_while_not(&cursor, size, str, ' ');
	move_cursor_while_is(&cursor, size, str, ' ');
	if ((cursor >= size) || (str[cursor] == ' ')) {
		printk(KERN_ALERT "Not found second ts\n");
		return -1;
	}
	second_ts = simple_strtoull(str + cursor, NULL, 0);
	if (second_ts == 0) {
		printk(KERN_ALERT "Incorrect second ts\n");
		return -1;
	}
	ob->phase_ts = ns_to_timespec64(first_ts);
	ob->raw_ts = ns_to_timespec64(second_ts);
	ob->id = id;
	move_cursor_while_not(&cursor, size, str, '#');
	if ((cursor >= size) || (str[cursor] != '#')) {
		return 0;
	}
	++cursor;
	return 0;
}

static void __exit pps_player_exit(void)
{
	remove_proc_entry("pps_player", NULL);
	dev_info(pps->dev, "ktimer PPS source unregistered\n");

	del_timer_sync(&ktimer);
	pps_unregister_source(pps);
}

int pps_player_release(struct inode *inodep, struct file *filp) {
	ring_iter = 0;
#ifdef PPS_PLAYER_DEBUG
	int i, j;
#endif

	#ifdef PPS_PLAYER_DEBUG
		printk(KERN_ALERT "pps_player_release()\n");
	#endif

	if (pps == NULL) {
		pr_err("cannot register PPS source\n");
		return -1;
	}

#ifdef PPS_PLAYER_DEBUG
	for (i = 0; i < PROFILER_RING_SIZE; ++i) {
		printk("profiler_log_entry %d: id = %d, ts_raw = %llu, ts_real = %llu\n", i, (int)(player_ring[i].id),
			timespec_to_ns(&(player_ring[i].raw_ts)), timespec_to_ns(&(player_ring[i].phase_ts)));
	}
#endif
	ring_is_played = 0;

	set_vars_from_dump(&vars_shot);

	return 0;
}

#define __set_field_with_func(field, format) vars_shot.shot_##field = simple_strto##format(buffer + rc, NULL, 0); \
		move_cursor_while_not(&rc, buffer_it, buffer, ' '); \
		++rc; \

ssize_t pps_player_write(struct file *filp, const char *buf, size_t count, loff_t *f_pos) {
	int i, prev_i, rc;
	profiler_log_entry tmp;
	int j;

	i = 0;
	prev_i = 0;


	if (count == 0) return 0;
	if (ring_iter >= PROFILER_RING_SIZE) return 0;
	if (buffer_it >= 1024) return 0;
#ifdef PPS_PLAYER_DEBUG
	printk(KERN_ALERT "pps_player_write(): count = %d\n", (int)count);
#endif

	while (i < count) {
		prev_i = i;
		while ((i < count) && (buf[i] != '\n')) ++i;
		if (i >= count) {
			if (copy_from_user(buffer + buffer_it, buf + prev_i, i - prev_i)) {
				printk(KERN_ALERT "copy_from_user error1\n");
				return -EFAULT;
			}
			buffer_it += i - prev_i;
			*f_pos += count;
			return count;
		}
		if (copy_from_user(buffer + buffer_it, buf + prev_i, i - prev_i)) {
			printk(KERN_ALERT "copy_from_user error2\n");
			return -EFAULT;
		}

		buffer_it += i - prev_i;
#ifdef PPS_PLAYER_DEBUG
		printk(KERN_ALERT "pps_player_write(): buffer_it = %d\n", (int)buffer_it);
#endif
		if (vars_shot_inited == 0) {
			rc = 0;
			__set_field_with_func(tick_usec, ul);
			__set_field_with_func(tick_nsec, ul);
			__set_field_with_func(tick_length, ull);
			__set_field_with_func(tick_length_base, ull);
			__set_field_with_func(time_state, l);
			__set_field_with_func(time_status, l);
			__set_field_with_func(time_offset, ll);
			__set_field_with_func(time_constant, l);
			__set_field_with_func(time_maxerror, l);
			__set_field_with_func(time_esterror, l);
			__set_field_with_func(time_freq, ll);
			__set_field_with_func(time_reftime, l);
			__set_field_with_func(time_adjust, l);
			__set_field_with_func(ntp_tick_adj, ll);
			__set_field_with_func(pps_valid, l);
			for (j = 0; j < PPS_FILTER_SIZE; ++j) {
				vars_shot.shot_pps_tf[j] = simple_strtol(buffer + rc, NULL, 0);
				move_cursor_while_not(&rc, buffer_it, buffer, ' ');
				++rc;
			}
			__set_field_with_func(pps_tf_pos, ul);
			__set_field_with_func(pps_jitter, l);
			vars_shot.shot_pps_fbase = ns_to_timespec64(simple_strtoll(buffer + rc, NULL, 0));
			move_cursor_while_not(&rc, buffer_it, buffer, ' ');
			++rc;
			__set_field_with_func(pps_shift, l);
			__set_field_with_func(pps_intcnt, l);
			__set_field_with_func(pps_freq, ll);
			__set_field_with_func(pps_stabil, l);
			__set_field_with_func(pps_calcnt, l);
			__set_field_with_func(pps_jitcnt, l);
			__set_field_with_func(pps_stbcnt, l);
			__set_field_with_func(pps_errcnt, l);
#ifdef PPS_PLAYER_DEBUG
			printk(KERN_ALERT "In vars shot: %lu %lu %llu %llu %d %d %lld %ld %ld %ld %lld %ld %ld ", vars_shot.shot_tick_usec,
				vars_shot.shot_tick_nsec, vars_shot.shot_tick_length, vars_shot.shot_tick_length_base,
				vars_shot.shot_time_state, vars_shot.shot_time_status, vars_shot.shot_time_offset,
				vars_shot.shot_time_constant, vars_shot.shot_time_maxerror, vars_shot.shot_time_esterror,
				vars_shot.shot_time_freq, vars_shot.shot_time_reftime, vars_shot.shot_time_adjust);
			printk(KERN_ALERT "%lld %d ", vars_shot.shot_ntp_tick_adj, vars_shot.shot_pps_valid);
			for (j = 0; j < PPS_FILTER_SIZE; ++j) {
				printk(KERN_ALERT "%ld ", vars_shot.shot_pps_tf[j]);
			}
			printk(KERN_ALERT "%u %ld %lld %d %d %lld %ld %ld %ld %ld %ld\n", vars_shot.shot_pps_tf_pos,
				vars_shot.shot_pps_jitter, timespec_to_ns(&(vars_shot.shot_pps_fbase)), vars_shot.shot_pps_shift,
				vars_shot.shot_pps_intcnt, vars_shot.shot_pps_freq, vars_shot.shot_pps_stabil,
				vars_shot.shot_pps_calcnt, vars_shot.shot_pps_jitcnt, vars_shot.shot_pps_stbcnt, vars_shot.shot_pps_errcnt);
#endif
			vars_shot_inited = 1;
		} else {
#ifdef PPS_PLAYER_DEBUG
			printk(KERN_ALERT "pps_player_write(): buffer for converting: %s\n", buffer);
#endif
			rc = profiler_log_entry_from_string(&tmp, buffer, buffer_it);
			if (rc < 0) return -EFAULT;
			player_ring[ring_iter++] = tmp;
		}
		buffer_it = 0;
		memset(buffer, 0, 1024);
		++i;
	}
	*f_pos += count;
	return count;
}

#undef __set_field_with_func

static int pps_player_open(struct inode *inode, struct file *file) {
#ifdef PPS_PLAYER_DEBUG
	printk(KERN_ALERT "pps_player_open():\n");
#endif
	buffer_it = 0;
	ring_iter = 0;
	ring_is_played = 1;
	memset(buffer, 0, 1024);
	vars_shot_inited = 0;
	return 0;
}

static struct file_operations pps_proc_ops = {
	.owner = THIS_MODULE,
	.open = pps_player_open,
	.write = pps_player_write,
	.release = pps_player_release,
};

static int __init pps_player_init(void)
{
	log_entry = proc_create("pps_player", 0, NULL, &pps_proc_ops);

	pps = pps_register_source(&pps_ktimer_info,
				PPS_CAPTUREASSERT | PPS_OFFSETASSERT);
	if (pps == NULL) {
		pr_err("cannot register PPS source\n");
		return -ENOMEM;
	}

	setup_timer(&ktimer, pps_ktimer_event, 0);
	mod_timer(&ktimer, jiffies + HZ);

	dev_info(pps->dev, "ktimer PPS source registered\n");
	return 0;
}

module_init(pps_player_init);
module_exit(pps_player_exit);

MODULE_AUTHOR("Vladislav Shpilevoi <vshpilevoi@mail.ru>");
MODULE_DESCRIPTION("Module for playing pps dumps maked by pps profiler");
MODULE_LICENSE("GPL");

#endif
