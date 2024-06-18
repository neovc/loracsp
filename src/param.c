#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "common.h"

#define MAX_PARAM_NAME_LEN          14

/**
  * List of supported parameter types.
  */
typedef enum __attribute__((__packed__)) {
	PARAM_UINT8,
	PARAM_UINT16,
	PARAM_UINT32,
	PARAM_UINT64,
	PARAM_INT8,
	PARAM_INT16,
	PARAM_INT32,
	PARAM_INT64,
	PARAM_X8,
	PARAM_X16,
	PARAM_X32,
	PARAM_X64,
	PARAM_DOUBLE,
	PARAM_FLOAT,
	PARAM_STRING,
	PARAM_DATA,
	PARAM_BOOL,
} param_type_t;

enum param_flags {
	PARAM_F_READONLY = (1 << 1),
	PARAM_F_PERSIST = (1 << 0),
};

struct g_param_table {
	uint16_t addr;
	param_type_t type;
	uint8_t size;
	uint8_t flags;
	void *p;
	char name[MAX_PARAM_NAME_LEN];
};

extern const char status[2][8];
extern uint32_t crc32_memory(uint8_t *, uint32_t);
extern int flash_write_data(uint8_t *, uint32_t, int len);
extern int flash_read_data(uint8_t *, uint32_t, int len);
extern uint8_t csp_node, boot_cause;
extern uint32_t flash_eccc_count, flash_eccd_count;

uint8_t g_params_buffer[256];

#define PARAM_OFFSET (FLASH_BASEADDR + 256000) /* 250K Offset */

const struct g_param_table lora_params[] = {
	{.addr = 0x00, .type = PARAM_UINT8,  .size = 1,  .p = &(csp_node),            .name = "csp_node",      .flags = PARAM_F_PERSIST},
	{.addr = 0x01, .type = PARAM_UINT8,  .size = 1,  .p = &(boot_cause),          .name = "boot_cause",    .flags = 0},
	{.addr = 0x04, .type = PARAM_UINT32, .size = 4,  .p = &(flash_eccc_count),    .name = "eccc_cnt",      .flags = PARAM_F_PERSIST},
	{.addr = 0x08, .type = PARAM_UINT32, .size = 4,  .p = &(flash_eccd_count),    .name = "eccd_cnt",      .flags = PARAM_F_PERSIST},

	{.addr = 0xFB, .type = PARAM_INT8,   .size = 1,  .p = NULL,                   .name = "eof",           .flags = 0},
	/* 4B for CRC32 */
	/* 0xFF */
};

#define PARAM_SIZE sizeof(lora_params) / sizeof(lora_params[0])

const char param_type_name[17][4] = {
	{"U8"}, {"U16"}, {"U32"}, {"U64"},
	{"I8"}, {"I16"}, {"I32"}, {"I64"},
	{"X8"}, {"X16"}, {"X32"}, {"X64"},
	{"DBL"}, {"FLT"}, {"STR"}, {"DAT"},
	{"BL"}
};

const char *
get_param_type(param_type_t type)
{
	if (type > PARAM_BOOL)
		return "ERR";
	return &(param_type_name[type][0]);
}

void
print_hex(uint8_t *src, int len)
{
	int i;

	if (len <= 0 || src == NULL)
		return;
	mini_printf("0x");
	for (i = 0; i < len; i ++)
		mini_printf("%02X", src[i]);
	mini_printf("\n");
}

/* return 0 if OK
 * return 1 if failed
 */
int
read_params_config(void)
{
	int i, len, r;
	uint32_t crc2, crc = 0;

	memset(g_params_buffer, 0, sizeof(g_params_buffer));
	len = lora_params[PARAM_SIZE - 1].addr + lora_params[PARAM_SIZE - 1].size + sizeof(crc);
	r = flash_read_data(g_params_buffer, PARAM_OFFSET, len);
	if (r != len)
		return 1;

	crc = crc32_memory(g_params_buffer, len - 4);
	memcpy(&crc2, g_params_buffer + len - 4, sizeof(crc2));

	if (crc != crc2) /* CRC mismatched */
		return 1;

	for (i = 0; i < PARAM_SIZE; i ++) {
		if ((lora_params[i].p != NULL) && (lora_params[i].size > 0) &&
			(lora_params[i].flags & PARAM_F_PERSIST)) {
			memcpy(lora_params[i].p, g_params_buffer + lora_params[i].addr, lora_params[i].size);
		}
	}

	return 0;
}

/* return 0 if OK
 * return 1 if failed
 */
int
save_params_config(void)
{
	int i, r = 0, len;
	uint32_t crc;

	memset(g_params_buffer, 0, sizeof(g_params_buffer));
	len = lora_params[PARAM_SIZE - 1].addr + lora_params[PARAM_SIZE - 1].size;
	for (i = 0; i < PARAM_SIZE; i ++) {
		if ((lora_params[i].p != NULL) && (lora_params[i].size > 0) &&
			(lora_params[i].flags & PARAM_F_PERSIST)) {
			memcpy(g_params_buffer + lora_params[i].addr, lora_params[i].p, lora_params[i].size);
		}
	}
	crc = crc32_memory(g_params_buffer, len);
	memcpy(g_params_buffer + len, &crc, sizeof(crc));
	len += sizeof(crc);

	r = flash_write_data(g_params_buffer, PARAM_OFFSET, len);
	return (r != len);
}

void
list_params_config(int start, int end, uint8_t asterisk_flag)
{
	int r, i, j;
	char tmp[64], c, *name;
	float f = 0.0;
	void *src = NULL;

	for (i = start; i < end; i ++) {
		r = 0;
		j = 0;
		c = (lora_params[i].flags & asterisk_flag)?'*':' ';

		src = lora_params[i].p;

		name = (char *) get_param_type(lora_params[i].type);
		mini_printf("  0x%03X %c %-14s %-4s ", lora_params[i].addr, c, lora_params[i].name, name);
		switch(lora_params[i].type) {
		case PARAM_X8:
			r = *((uint8_t *) src);
			mini_printf("0x%02x\n", r);
			break;
		case PARAM_X16:
			r = *((uint16_t *) src);
			mini_printf("0x%04x\n", r);
			break;
		case PARAM_X32:
			r = *((uint32_t *) src);
			mini_printf("0x%08x\n", r);
			break;
		case PARAM_UINT8:
			r = *((uint8_t *) src);
			j = 1;
			break;
		case PARAM_UINT16:
			r = *((uint16_t *) src);
			j = 1;
			break;
		case PARAM_UINT32:
			r = *((uint32_t *) src);
			j = 1;
			break;
		case PARAM_INT8:
			r = *((int8_t *) src);
			j = 2;
			break;
		case PARAM_INT16:
			r = *((int16_t *) src);
			j = 2;
			break;
		case PARAM_INT32:
			r = *((int32_t *) src);
			j = 2;
			break;
		case PARAM_DOUBLE:
			j = 3;
			f = *((double *) src);
			break;
		case PARAM_FLOAT:
			j = 3;
			f = *((float *) src);
			break;
		case PARAM_STRING:
			memcpy(tmp, src, lora_params[i].size);
			tmp[lora_params[i].size] = '\0';
			mini_printf("\"%s\"\n", tmp);
			break;
		case PARAM_DATA:
			memcpy(tmp, src, lora_params[i].size);
			tmp[lora_params[i].size] = '\0';
			print_hex((uint8_t *)tmp, lora_params[i].size);
			break;
		case PARAM_BOOL:
			r = *((uint8_t *) src);
			mini_printf("%s\n", r?"true":"false");
			break;
		default:
			break;
		}

		if (j == 1) {
			/* unsigned int */
			mini_printf("%u\n", r);
		} else if (j == 2) {
			/* signed int */
			mini_printf("%d\n", r);
		} else if (j == 3) {
			/* float */
			mini_printf("%.2f\n", f);
		}
	}
}

void
param_set_data(char *src, int len, char *dst, int max_len)
{
	int i, j, x, val;
	char c;

	if (src == NULL || len <= 0 || dst == NULL || max_len <= 0)
		return;

	for (i = 0, j = 0, x = 0; i < len && j < max_len; i ++) {
		c = src[i];

		if ('0' <= c && c <= '9') {
			val = c - '0';
		} else if ('a' <= c && c <= 'f') {
			val = 10 + c - 'a';
		} else if ('A' <= c && c <= 'F') {
			val = 10 + c - 'A';
		} else {
			continue;
		}

		if (x == 0) {
			dst[j] = val;
			x = 1;
		} else {
			dst[j] = dst[j] << 4 | val;
			x = 0;
			j ++;
		}
	}
}

/* return 0 if OK
 * return 1 if failed
 */
int
set_params_value(char *name, char *value)
{
	int i, j;
	double d;
	float f;
	uint8_t t8;
	uint16_t t16;

	if (name == NULL || value == NULL || name[0] == '\0' || value[0] == '\0')
		return 1;

	for (i = 0; i < PARAM_SIZE; i ++) {
		if (strcasecmp(lora_params[i].name, name) == 0)
			break;
	}

	if (i == PARAM_SIZE)
		return 1;

	switch (lora_params[i].type) {
	case PARAM_UINT8:
	case PARAM_INT8:
	case PARAM_X8:
		t8 = atoi(value) & 0xff;
		memcpy(lora_params[i].p, &t8, lora_params[i].size);
		break;
	case PARAM_UINT16:
	case PARAM_INT16:
	case PARAM_X16:
		t16 = atoi(value) & 0xffff;
		memcpy(lora_params[i].p, &t16, lora_params[i].size);
		break;
	case PARAM_UINT32:
	case PARAM_INT32:
	case PARAM_X32:
		j = atoi(value) & 0xffffffff;
		memcpy(lora_params[i].p, &j, lora_params[i].size);
		break;
	case PARAM_FLOAT:
		f = strtof(value, NULL);
		memcpy(lora_params[i].p, &f, lora_params[i].size);
		break;
	case PARAM_DOUBLE:
		d = strtod(value, NULL);
		memcpy(lora_params[i].p, &d, lora_params[i].size);
		break;
	case PARAM_STRING:
		j = strlen(value);
		memset(lora_params[i].p, 0, lora_params[i].size);
		memcpy(lora_params[i].p, value, j);
		break;
	case PARAM_DATA:
		/* convert hex string to hex data */
		memset(lora_params[i].p, 0, lora_params[i].size);
		param_set_data(value, strlen(value), lora_params[i].p, lora_params[i].size);
		break;
	case PARAM_BOOL:
		if (strcasecmp(value, "true") == 0)
			t8 = 1;
		else
			t8 = 0;
		memcpy(lora_params[i].p, &t8, lora_params[i].size);
		break;
	default:
		break;
	}

	return 0;
}

int
param_cmd(int argc, char **argv)
{
	int r;

	if (argc < 2)
		return 1;

	if (strcasecmp(argv[1], "list") == 0) {
		list_params_config(0, PARAM_SIZE, PARAM_F_PERSIST);
	} else if (strcasecmp(argv[1], "set") == 0 && argc >= 4) {
		if (set_params_value(argv[2], argv[3])) {
			mini_printf("can't find '%s' parameter\n", argv[2]);
		} else {
			mini_printf("set %s -> %s\n", argv[2], argv[3]);
		}
	} else if (strcasecmp(argv[1], "save") == 0) {
		r = save_params_config();
		mini_printf("save parameter -> %s\n", status[r]);
	} else {
		return 1;
	}
	return 0;
}

