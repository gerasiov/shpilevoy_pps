#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cstdio>
#include <ctime>
#include <cstring>
#define NSEC_PER_SEC 100000000L

#define BUFFER_SIZE 1024

#define MAX_LOG_ENTRY_SIZE 10

//#define PPS_LOG_READER_DEBUG 1

/* Pring instructions for this program */
void print_help() {
	std::cout << "Vladislav Shpilevoi 2015 pps_log_reader\nUsage: pps_log_reader [-f <filename>]\n<filename>" \
		" - default is 'proc/pps_profiler'" << std::endl;
	exit(0);
}

/* Class for closing FILE *, when it is over its visibility area */
struct FileKeeper {
private:
	FILE *ob;

public:
	const FILE *get() const;
	FILE *get();
	FileKeeper(FILE *ob_);
	~FileKeeper();
};

//---------------- P R O F I L E R   L O G ----------------

/* Analogue of kernel ns_to_timespec */
struct timespec ns_to_timespec(const long long nsec)
{
	struct timespec ts;
	int rem;
	if (!nsec) return (struct timespec) {0, 0};
	ts.tv_sec = nsec / NSEC_PER_SEC;
	ts.tv_nsec = nsec % NSEC_PER_SEC;
	return ts;
}

/* Copy of kernel pps_profiler structures */
typedef enum profiler_log {
	PRF_PPS_CLEAR
} profiler_log_t;

typedef struct profiler_log_unit {
	profiler_log_t type;
} profiler_log_unit;

typedef struct profiler_log_entry {
	unsigned int units_cnt;
	profiler_log_unit units[MAX_LOG_ENTRY_SIZE];
	unsigned long id;
	struct timespec phase_ts;
	struct timespec raw_ts;
} profiler_log_entry;

/* Functions for iterating over string */
void move_cursor_while_not(size_t *cursor, size_t size, const char *str, char sym) {
	while((*cursor < size) && (str[*cursor] != sym)) ++(*cursor);
}
void move_cursor_while_is(size_t *cursor, size_t size, const char *str, char sym) {
	while((*cursor < size) && (str[*cursor] == sym)) ++(*cursor);
}

/* Function for parsing string to profiler_log_entry */
int profiler_log_entry_from_string(profiler_log_entry *ob, const char *str, size_t size) {
	profiler_log_entry res;
	profiler_log_unit unit;
	size_t cursor;
	char *end;
	unsigned long long first_ts;
	unsigned long long second_ts;
	int id;
	int i;

	cursor = 0;
	end = NULL;
	ob->units_cnt = 0;

	move_cursor_while_not(&cursor, size, str, '#');
	if ((cursor >= size) || (str[cursor] != '#')) {
		std::cout << "Not found first #" << std::endl;
		return -1;
	}
	++cursor;
	id = atoi(str);

	move_cursor_while_is(&cursor, size, str, ' ');
	if ((cursor >= size) || (str[cursor] == ' ')) {
		std::cout << "Not found first ts" << std::endl;
		return -1;
	}
	std::cout << "id = " << id << ", cursor = " << cursor << std::endl;
	first_ts = strtoull(str + cursor, NULL, 0);
	std::cout << "first_ts: " << first_ts << std::endl;
	if (first_ts == 0) {
		std::cout << "Incorrect first ts" << std::endl;
		return -1;
	}
	move_cursor_while_not(&cursor, size, str, ' ');
	move_cursor_while_is(&cursor, size, str, ' ');
	if ((cursor >= size) || (str[cursor] == ' ')) {
		std::cout << "Not found second ts" << std::endl;
		return -1;
	}
	second_ts = strtoull(str + cursor, NULL, 0);
	std::cout << "second_ts: " << second_ts << std::endl;
	if (second_ts == 0) {
		std::cout << "Incorrect second ts" << std::endl;
		return -1;
	}
	ob->id = id;
	move_cursor_while_not(&cursor, size, str, '#');
	if ((cursor >= size) || (str[cursor] != '#')) {
		return 0;
	}
	++cursor;
	std::cout << "cursor = " << cursor << ", size = " << size << std::endl;

	while (cursor < size) {
		move_cursor_while_is(&cursor, size, str, ' ');
		if ((cursor >= size) || (str[cursor] == ' ')) {
			std::cout << "Error1 while reading unit" << std::endl;
			return -1;
		}
		if (str[cursor] == '\n') return 0;
		id = atoi(str + cursor);
		unit.type = (profiler_log_t)id;
		if (ob->units_cnt >= MAX_LOG_ENTRY_SIZE) {
			std::cout << "Overflow units in entry" << std::endl;
			return -1;
		}
		ob->units[ob->units_cnt++] = unit;
		move_cursor_while_not(&cursor, size, str, '#');
		++cursor;
	}
	return 0;
}

/* Is must be same value that is in ntp_internal.h */
#define RING_SIZE 20

profiler_log_entry ring[RING_SIZE];
char buffer[1024];
int buffer_it = 0;
int ring_it = 0;

/* Function for parsing multiline text to ring */
void fill_ring(const char *buf, int count) {
	if (ring_it >= RING_SIZE) {
		std::cout << "error 1" << std::endl;
		return;
	}
	if (buffer_it >= 1024) {
		std::cout << "error 2" << std::endl;
		return;
	}
	int i = 0, prev_i = 0;
	while (i < count) {
		std::cout << "iteration, i = " << i << std::endl;
		prev_i = i;
		while ((i < count) && (buf[i] != '\n')) ++i;
		if (i >= count) {
			std::cout << "all in buffer" << std::endl;
			memcpy(buffer + buffer_it, buf + prev_i, i - prev_i);
			buffer_it += i - prev_i;
			std::cout << buffer << std::endl;
			return;
		}
		std::cout << "i = " << i << std::endl;
		memcpy(buffer + buffer_it, buf + prev_i, i - prev_i);
		buffer_it += i - prev_i;
		profiler_log_entry tmp;
		std::cout << std::string(buffer, buffer_it + 1) << ";" << std::endl;
		int rc = profiler_log_entry_from_string(&tmp, buffer, buffer_it);
		buffer_it = 0;
		memset(buffer, 0, 1024);
		std::cout << "new string: res = " << rc << std::endl;
		if (rc < 0) {
			std::cout << "error 3" << std::endl;
			return;
		}
		ring[ring_it++] = tmp;
		++i;
	}
	std::cout << "success" << std::endl;
}

/* Methods of FileKeeper */
const FILE *FileKeeper::get() const { return ob; }
FILE *FileKeeper::get() { return ob; }
FileKeeper::FileKeeper(FILE *ob_) : ob(ob_) { }
FileKeeper::~FileKeeper() { std::fclose(ob); }

int main(int argc, char **argv) {
#ifdef PPS_LOG_READER_DEBUG
	const char *t = "1 # 12345 6789 # 0 #\n2 # 1011 1213 # 0 #\n3 # 1415 1617 #";
	fill_ring(t, strlen(t));
	t = "\n";
	fill_ring(t, strlen(t));
	for (int i = 0; i < ring_it; ++i) {
		std::cout << "id = " << ring[i].id << ", units_cnt = " << ring[i].units_cnt << std::endl;
		for (int j = 0; j < ring[i].units_cnt; ++j) {
			std::cout << j << ": type = " << ring[i].units[j].type << std::endl;
		}
	}
	return 0;
#else
	std::string proc_file("/proc/pps_profiler");
	std::string res_file("pps_profiler.txt");
	if (argc > 2) print_help();
	if (argc == 2) {
		proc_file = std::string(argv[1]);
	} 
	FILE *f = std::fopen(proc_file.c_str(), "r");
	if (f == NULL) {
		std::cout << "Error while opening file: " << proc_file << std::endl;
		return 0;
	}
	FileKeeper f_keeper(f);
	FILE *resf = std::fopen(res_file.c_str(), "w");
	if (resf == NULL) {
		std::cout << "Error while opening file: " << res_file << std::endl;
		return 0;
	}
	FileKeeper resf_keeper(resf);

	char buffer[BUFFER_SIZE];
	while (std::fgets(buffer, BUFFER_SIZE, f) != NULL) {
		if (std::fputs(buffer, resf) < 0) {
			std::cout << "Error while writing in file: " << res_file << std::endl;
			return 0;
		}
	}
	if (std::feof(f) == 0) {
		std::cout << "Error while reading is occured" << std::endl;
	}
	std::cout << "Log is ready" << std::endl;

	return 0;
#endif
}