/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* This file implements the GDB Remote Serial Debugging protocol as
 * described in "Debugging with GDB" build from GDB source.
 *
 * Originally written for GDB 6.8, updated and tested with GDB 7.2.
 */

#include "general.h"
#include "ctype.h"
#include "hex_utils.h"
#include "gdb_if.h"
#include "gdb_packet.h"
#include "gdb_main.h"
#include "gdb_hostio.h"
#include "target.h"
#include "command.h"
#include "crc32.h"
#include "morse.h"

enum gdb_signal {
	GDB_SIGINT = 2,
	GDB_SIGTRAP = 5,
	GDB_SIGSEGV = 11,
	GDB_SIGLOST = 29,
};

#define BUF_SIZE	1024

#define ERROR_IF_NO_TARGET()	\
	if(!cur_target) { gdb_putpacketz("EFF"); break; }

typedef struct
{
	const char *cmd_prefix;
	void (*func)(const char *packet, int len);
} cmd_executer;

static char pbuf[BUF_SIZE + 1];

static target *cur_target;
static target *last_target;

static void handle_q_packet(char *packet, int len);
static void handle_v_packet(char *packet, int len);
static void handle_z_packet(char *packet, int len);

static void gdb_target_destroy_callback(struct target_controller *tc, target *t)
{
	(void)tc;
	if (cur_target == t)
		cur_target = NULL;

	if (last_target == t)
		last_target = NULL;
}

static void gdb_target_printf(struct target_controller *tc,
                              const char *fmt, va_list ap)
{
	(void)tc;
	gdb_voutf(fmt, ap);
}

static struct target_controller gdb_controller = {
	.destroy_callback = gdb_target_destroy_callback,
	.printf = gdb_target_printf,

	.open = hostio_open,
	.close = hostio_close,
	.read = hostio_read,
	.write = hostio_write,
	.lseek = hostio_lseek,
	.rename = hostio_rename,
	.unlink = hostio_unlink,
	.stat = hostio_stat,
	.fstat = hostio_fstat,
	.gettimeofday = hostio_gettimeofday,
	.isatty = hostio_isatty,
	.system = hostio_system,
};

int gdb_main_loop(struct target_controller *tc, bool in_syscall)
{
	int size;
	bool single_step = false;

	/* GDB protocol main loop */
	while(1) {
		SET_IDLE_STATE(1);
		size = gdb_getpacket(pbuf, BUF_SIZE);
		SET_IDLE_STATE(0);
		switch(pbuf[0]) {
		/* Implementation of these is mandatory! */
		case 'g': { /* 'g': Read general registers */
			ERROR_IF_NO_TARGET();
			uint8_t arm_regs[target_regs_size(cur_target)];
			target_regs_read(cur_target, arm_regs);
			gdb_putpacket(hexify(pbuf, arm_regs, sizeof(arm_regs)),
			              sizeof(arm_regs) * 2);
			break;
		}
		case 'm': {	/* 'm addr,len': Read len bytes from addr */
			uint32_t addr, len;
			ERROR_IF_NO_TARGET();
			sscanf(pbuf, "m%" SCNx32 ",%" SCNx32, &addr, &len);
			if (len > sizeof(pbuf) / 2) {
				gdb_putpacketz("E02");
				break;
			}
			DEBUG_GDB("m packet: addr = %" PRIx32 ", len = %" PRIx32 "\n",
					  addr, len);
			uint8_t mem[len];
			if (target_mem_read(cur_target, mem, addr, len))
				gdb_putpacketz("E01");
			else
				gdb_putpacket(hexify(pbuf, mem, len), len * 2);
			break;
		}
		case 'G': {	/* 'G XX': Write general registers */
			ERROR_IF_NO_TARGET();
			uint8_t arm_regs[target_regs_size(cur_target)];
			unhexify(arm_regs, &pbuf[1], sizeof(arm_regs));
			target_regs_write(cur_target, arm_regs);
			gdb_putpacketz("OK");
			break;
		}
		case 'M': { /* 'M addr,len:XX': Write len bytes to addr */
			uint32_t addr, len;
			int hex;
			ERROR_IF_NO_TARGET();
			sscanf(pbuf, "M%" SCNx32 ",%" SCNx32 ":%n", &addr, &len, &hex);
			if (len > (unsigned)(size - hex) / 2) {
				gdb_putpacketz("E02");
				break;
			}
			DEBUG_GDB("M packet: addr = %" PRIx32 ", len = %" PRIx32 "\n",
					  addr, len);
			uint8_t mem[len];
			unhexify(mem, pbuf + hex, len);
			if (target_mem_write(cur_target, addr, mem, len))
				gdb_putpacketz("E01");
			else
				gdb_putpacketz("OK");
			break;
		}
		case 's':	/* 's [addr]': Single step [start at addr] */
			single_step = true;
			/* fall through */
		case 'c':	/* 'c [addr]': Continue [at addr] */
			if(!cur_target) {
				gdb_putpacketz("X1D");
				break;
			}

			target_halt_resume(cur_target, single_step);
			SET_RUN_STATE(1);
			single_step = false;
			/* fall through */
		case '?': {	/* '?': Request reason for target halt */
			/* This packet isn't documented as being mandatory,
			 * but GDB doesn't work without it. */
			target_addr watch;
			enum target_halt_reason reason;

			if(!cur_target) {
				/* Report "target exited" if no target */
				gdb_putpacketz("W00");
				break;
			}

			/* Wait for target halt */
			while(!(reason = target_halt_poll(cur_target, &watch))) {
				unsigned char c = gdb_if_getchar_to(0);
				if((c == '\x03') || (c == '\x04')) {
					target_halt_request(cur_target);
				}
			}
			SET_RUN_STATE(0);

			/* Translate reason to GDB signal */
			switch (reason) {
			case TARGET_HALT_ERROR:
				gdb_putpacket_f("X%02X", GDB_SIGLOST);
				morse("TARGET LOST.", true);
				break;
			case TARGET_HALT_REQUEST:
				gdb_putpacket_f("T%02X", GDB_SIGINT);
				break;
			case TARGET_HALT_WATCHPOINT:
				gdb_putpacket_f("T%02Xwatch:%08X;", GDB_SIGTRAP, watch);
				break;
			case TARGET_HALT_FAULT:
				gdb_putpacket_f("T%02X", GDB_SIGSEGV);
				break;
			default:
				gdb_putpacket_f("T%02X", GDB_SIGTRAP);
			}
			break;
		}

		/* Optional GDB packet support */
		case 'p': { /* Read single register */
			ERROR_IF_NO_TARGET();
			uint32_t reg;
			sscanf(pbuf, "p%" SCNx32, &reg);
			uint8_t val[8];
			size_t s = target_reg_read(cur_target, reg, val, sizeof(val));
			if (s > 0)
				gdb_putpacket(hexify(pbuf, val, s), s * 2);
			else
				gdb_putpacketz("EFF");
			break;
		}
		case 'P': { /* Write single register */
			ERROR_IF_NO_TARGET();
			uint32_t reg;
			int n;
			sscanf(pbuf, "P%" SCNx32 "=%n", &reg, &n);
			// TODO: FIXME, VLAs considered harmful.
			uint8_t val[strlen(&pbuf[n]) / 2];
			unhexify(val, pbuf + n, sizeof(val));
			if (target_reg_write(cur_target, reg, val, sizeof(val)) > 0)
				gdb_putpacketz("OK");
			else
				gdb_putpacketz("EFF");
			break;
		}

		case 'F':	/* Semihosting call finished */
			if (in_syscall)
				return hostio_reply(tc, pbuf, size);
			else {
				DEBUG_GDB("*** F packet when not in syscall! '%s'\n", pbuf);
				gdb_putpacketz("");
			}
			break;

		case '!':	/* Enable Extended GDB Protocol. */
			/* This doesn't do anything, we support the extended
			 * protocol anyway, but GDB will never send us a 'R'
			 * packet unless we answer 'OK' here.
			 */
			gdb_putpacketz("OK");
			break;

		case 0x04:
		case 'D':	/* GDB 'detach' command. */
			if(cur_target) {
				SET_RUN_STATE(1);
				target_detach(cur_target);
			}
			last_target = cur_target;
			cur_target = NULL;
			gdb_putpacketz("OK");
			break;

		case 'k':	/* Kill the target */
			if(cur_target) {
				target_reset(cur_target);
				target_detach(cur_target);
				last_target = cur_target;
				cur_target = NULL;
			}
			break;

		case 'r':	/* Reset the target system */
		case 'R':	/* Restart the target program */
			if(cur_target)
				target_reset(cur_target);
			else if(last_target) {
				cur_target = target_attach(last_target,
						           &gdb_controller);
				if(cur_target)
					morse(NULL, false);
				target_reset(cur_target);
			}
			break;

		case 'X': { /* 'X addr,len:XX': Write binary data to addr */
			uint32_t addr, len;
			int bin;
			ERROR_IF_NO_TARGET();
			sscanf(pbuf, "X%" SCNx32 ",%" SCNx32 ":%n", &addr, &len, &bin);
			if (len > (unsigned)(size - bin)) {
				gdb_putpacketz("E02");
				break;
			}
			DEBUG_GDB("X packet: addr = %" PRIx32 ", len = %" PRIx32 "\n",
					  addr, len);
			if (target_mem_write(cur_target, addr, pbuf + bin, len))
				gdb_putpacketz("E01");
			else
				gdb_putpacketz("OK");
			break;
		}

		case 'q':	/* General query packet */
			handle_q_packet(pbuf, size);
			break;

		case 'v':	/* General query packet */
			handle_v_packet(pbuf, size);
			break;

		/* These packet implement hardware break-/watchpoints */
		case 'Z':	/* Z type,addr,len: Set breakpoint packet */
		case 'z':	/* z type,addr,len: Clear breakpoint packet */
			ERROR_IF_NO_TARGET();
			handle_z_packet(pbuf, size);
			break;

		default: 	/* Packet not implemented */
			DEBUG_GDB("*** Unsupported packet: %s\n", pbuf);
			gdb_putpacketz("");
		}
	}
}

static bool exec_command(char *packet, int len, const cmd_executer *exec)
{
	while (exec->cmd_prefix) {
		const int l = strlen(exec->cmd_prefix);
		if (!strncmp(packet, exec->cmd_prefix, l)) {
			exec->func(packet + l, len - l);
			return true;
		}
		++exec;
	}
	return false;
}

static void exec_q_rcmd(const char *packet,int len)
{
	char *data;
	int datalen;

	/* calculate size and allocate buffer for command */
	datalen = len / 2;
	data = alloca(datalen + 1);
	/* dehexify command */
	unhexify(data, packet, datalen);
	data[datalen] = 0;	/* add terminating null */

	int c = command_process(cur_target, data);
	if (c < 0)
		gdb_putpacketz("");
	else if (c == 0)
		gdb_putpacketz("OK");
	else
		gdb_putpacket(hexify(pbuf, "Failed\n", strlen("Failed\n")),
				2 * strlen("Failed\n"));
}

static void
handle_q_string_reply(const char *str, const char *param)
{
	unsigned long addr, len;
	const size_t str_len = strlen(str);

	if (sscanf(param, "%08lx,%08lx", &addr, &len) != 2) {
		gdb_putpacketz("E01");
		return;
	}
	else if (addr > str_len) {
		gdb_putpacketz("E01");
		return;
	}
	else if (addr == str_len) {
		gdb_putpacketz("l");
		return;
	}
	unsigned long output_len = str_len - addr;
	if (output_len > len)
		output_len = len;
	gdb_putpacket2("m", 1, str + addr, output_len);
}

static void exec_q_supported(const char *packet, int len)
{
	(void)packet;
	(void)len;
	gdb_putpacket_f("PacketSize=%X;qXfer:memory-map:read+;qXfer:features:read+", BUF_SIZE);
}

static void exec_q_memory_map(const char *packet, int len)
{
	(void)packet;
	(void)len;
	/* Read target XML memory map */
	if ((!cur_target) && last_target) {
		/* Attach to last target if detached. */
		cur_target = target_attach(last_target,
				   &gdb_controller);
	}
	if (!cur_target) {
		gdb_putpacketz("E01");
		return;
	}
	char buf[1024];
	target_mem_map(cur_target, buf, sizeof(buf)); /* Fixme: Check size!*/
	handle_q_string_reply(buf, packet);
}

static void exec_q_feature_read(const char *packet, int len)
{
	(void)len;
	/* Read target description */
	if ((!cur_target) && last_target) {
	  /* Attach to last target if detached. */
	  cur_target = target_attach(last_target, &gdb_controller);
	}
	if (!cur_target) {
	  gdb_putpacketz("E01");
	  return;
	}
	handle_q_string_reply(target_tdesc(cur_target), packet);
}

static void exec_q_crc(const char *packet, int len)
{
	(void)len;
	uint32_t addr, alen;
	if (sscanf(packet, "%" PRIx32 ",%" PRIx32, &addr, &alen) == 2) {
		if (!cur_target) {
			gdb_putpacketz("E01");
			return;
		}
		uint32_t crc;
		int res = generic_crc32(cur_target, &crc, addr, alen);
		if (res)
			gdb_putpacketz("E03");
		else
			gdb_putpacket_f("C%lx", crc);
	}
}

static const cmd_executer q_commands[]=
{
	{"qRcmd,",                         exec_q_rcmd},
	{"qSupported",                     exec_q_supported},
	{"qXfer:memory-map:read::",        exec_q_memory_map},
	{"qXfer:features:read:target.xml:",exec_q_feature_read},
	{"qCRC:",                          exec_q_crc},
	{NULL, NULL},
};

static void
handle_q_packet(char *packet, int len)
{
	if (exec_command(packet, len, q_commands))
		return;
	DEBUG_GDB("*** Unsupported packet: %s\n", packet);
	gdb_putpacket("", 0);
}

static void
handle_v_packet(char *packet, int plen)
{
	unsigned long addr, len;
	int bin;
	static uint8_t flash_mode = 0;

	if (sscanf(packet, "vAttach;%08lx", &addr) == 1) {
		/* Attach to remote target processor */
		cur_target = target_attach_n(addr, &gdb_controller);
		if(cur_target) {
			morse(NULL, false);
			gdb_putpacketz("T05");
		} else
			gdb_putpacketz("E01");

	} else if (!strncmp(packet, "vRun", 4)) {
		/* Parse command line for get_cmdline semihosting call */
		char cmdline[83];
		char *pcmdline = cmdline;
		char *tok = packet + 4;
		if (*tok == ';')
			++tok;
		cmdline[0] = '\0';
		while(*tok != '\0') {
			if (strlen(cmdline) + 3 >= sizeof(cmdline))
				break;
			if (*tok == ';') {
				*pcmdline++ = ' ';
				pcmdline[0] = '\0';
				tok++;
				continue;
			}
			if (isxdigit(tok[0]) && isxdigit(tok[1])) {
				unhexify(pcmdline, tok, 2);
				if ((*pcmdline == ' ') || (*pcmdline == '\\')) {
					pcmdline[1] = *pcmdline;
					*pcmdline++ = '\\';
				}
				pcmdline++;
				tok += 2;
				pcmdline[0] = '\0';
				continue;
			}
			break;
		}
		/* Run target program. For us (embedded) this means reset. */
		if (cur_target) {
			target_set_cmdline(cur_target, cmdline);
			target_reset(cur_target);
			gdb_putpacketz("T05");
		} else if (last_target) {
			cur_target = target_attach(last_target,
						   &gdb_controller);

			/* If we were able to attach to the target again */
			if (cur_target) {
				target_set_cmdline(cur_target, cmdline);
				target_reset(cur_target);
				morse(NULL, false);
				gdb_putpacketz("T05");
			} else
				gdb_putpacketz("E01");

		} else
			gdb_putpacketz("E01");

	} else if (sscanf(packet, "vFlashErase:%08lx,%08lx", &addr, &len) == 2) {
		/* Erase Flash Memory */
		DEBUG_GDB("Flash Erase %08lX %08lX\n", addr, len);
		if (!cur_target) {
			gdb_putpacketz("EFF");
			return;
		}

		if (!flash_mode) {
			/* Reset target if first flash command! */
			/* This saves us if we're interrupted in IRQ context */
			target_reset(cur_target);
			flash_mode = 1;
		}
		if (target_flash_erase(cur_target, addr, len) == 0)
			gdb_putpacketz("OK");
		else {
			flash_mode = 0;
			gdb_putpacketz("EFF");
		}

	} else if (sscanf(packet, "vFlashWrite:%08lx:%n", &addr, &bin) == 1) {
		/* Write Flash Memory */
		len = plen - bin;
		DEBUG_GDB("Flash Write %08lX %08lX\n", addr, len);
		if (cur_target && target_flash_write(cur_target, addr, (void*)packet + bin, len) == 0)
			gdb_putpacketz("OK");
		else {
			flash_mode = 0;
			gdb_putpacketz("EFF");
		}

	} else if (!strcmp(packet, "vFlashDone")) {
		/* Commit flash operations. */
		gdb_putpacketz(target_flash_done(cur_target) ? "EFF" : "OK");
		flash_mode = 0;

	} else {
		DEBUG_GDB("*** Unsupported packet: %s\n", packet);
		gdb_putpacket("", 0);
	}
}

static void
handle_z_packet(char *packet, int plen)
{
	(void)plen;

	uint8_t set = (packet[0] == 'Z') ? 1 : 0;
	int type, len;
	uint32_t addr;
	int ret;

	sscanf(packet, "%*[zZ]%d,%08" PRIX32 ",%d", &type, &addr, &len);
	if(set)
		ret = target_breakwatch_set(cur_target, type, addr, len);
	else
		ret = target_breakwatch_clear(cur_target, type, addr, len);

	if (ret < 0)
		gdb_putpacketz("E01");
	else if (ret > 0)
		gdb_putpacketz("");
	else
		gdb_putpacketz("OK");
}

void gdb_main(void)
{
	gdb_main_loop(&gdb_controller, false);
}
