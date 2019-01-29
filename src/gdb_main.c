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
#include "hex_utils.h"
#include "gdb_if.h"
#include "gdb_packet.h"
#include "gdb_main.h"
#include "gdb_hostio.h"
#include "target.h"
#include "command.h"
#include "crc32.h"
#include "morse.h"
#include "engine.h"
#include "sf-arch.h"

enum gdb_signal {
	GDB_SIGINT = 2,
	GDB_SIGTRAP = 5,
	GDB_SIGSEGV = 11,
	GDB_SIGLOST = 29,
};

#define BUF_SIZE	(1024 * 4)

#define ERROR_IF_NO_TARGET()	\
	if(!cur_target) { gdb_putpacketz("EFF"); break; }

static char pbuf[BUF_SIZE+1];

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

int sfgetc(void)
{
	return gdb_if_getchar_single();
}

int sfputc(int c)
{
	gdb_if_putchar_single(c);
	return 0;
}

int sfsync(void)
{
	gdb_if_flush();
	return 0;
}

int gdb_main_loop(struct target_controller *tc, bool in_syscall)
{
	int size;
	bool single_step = false;

	if(is_sforth_mode_active())while(1)sf_reset(),do_quit();

	DEBUG("Entering GDB protocol main loop\n");
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
			DEBUG("m packet: addr = %" PRIx32 ", len = %" PRIx32 "\n", addr, len);
			uint8_t mem[len];
			if (target_mem_read(cur_target, mem, addr, len))
				gdb_putpacketz("E01");
			else
				gdb_putpacket(hexify(pbuf, mem, len), len*2);
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
			DEBUG("M packet: addr = %" PRIx32 ", len = %" PRIx32 "\n", addr, len);
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
		case 'F':	/* Semihosting call finished */
			if (in_syscall) {
				return hostio_reply(tc, pbuf, size);
			} else {
				DEBUG("*** F packet when not in syscall! '%s'\n", pbuf);
				gdb_putpacketz("");
			}
			break;

		/* Optional GDB packet support */
		case '!':	/* Enable Extended GDB Protocol. */
			/* This doesn't do anything, we support the extended
			 * protocol anyway, but GDB will never send us a 'R'
			 * packet unless we answer 'OK' here.
			 */
			gdb_putpacketz("OK");
			break;

		case 0x04:
		case 'D':	/* GDB 'detach' command. */
			if(cur_target)
				target_detach(cur_target);
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
			DEBUG("X packet: addr = %" PRIx32 ", len = %" PRIx32 "\n", addr, len);
			memmove(pbuf, pbuf + bin, len);
			if (target_mem_write(cur_target, addr, pbuf, len))
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
			DEBUG("*** Unsupported packet: %s\n", pbuf);
			gdb_putpacketz("");
		}
	}
}

static void
handle_q_string_reply(const char *str, const char *param)
{
	unsigned long addr, len;

	if (sscanf(param, "%08lx,%08lx", &addr, &len) != 2) {
		gdb_putpacketz("E01");
		return;
	}
	if (addr < strlen (str)) {
		char reply[len+2];
		reply[0] = 'm';
		strncpy (reply + 1, &str[addr], len);
		if(len > strlen(&str[addr]))
			len = strlen(&str[addr]);
		gdb_putpacket(reply, len + 1);
	} else if (addr == strlen (str)) {
		gdb_putpacketz("l");
	} else
		gdb_putpacketz("E01");
}

static void
handle_q_packet(char *packet, int len)
{
	uint32_t addr, alen;

	if(!strncmp(packet, "qRcmd,", 6)) {
		char *data;
		int datalen;

		/* calculate size and allocate buffer for command */
		datalen = (len - 6) / 2;
		data = alloca(datalen+1);
		/* dehexify command */
		unhexify(data, packet+6, datalen);
		data[datalen] = 0;	/* add terminating null */

		int c = command_process(cur_target, data);
		if(c < 0)
			gdb_putpacketz("");
		else if(c == 0)
			gdb_putpacketz("OK");
		else
			gdb_putpacketz("E");

	} else if (!strncmp (packet, "qSupported", 10)) {
		/* Query supported protocol features */
		gdb_putpacket_f("PacketSize=%X;qXfer:memory-map:read+;qXfer:features:read+", BUF_SIZE);

	} else if (strncmp (packet, "qXfer:memory-map:read::", 23) == 0) {
		/* Read target XML memory map */
		if((!cur_target) && last_target) {
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
		handle_q_string_reply(buf, packet + 23);

	} else if (strncmp (packet, "qXfer:features:read:target.xml:", 31) == 0) {
		/* Read target description */
		if((!cur_target) && last_target) {
			/* Attach to last target if detached. */
			cur_target = target_attach(last_target,
						   &gdb_controller);
		}
		if (!cur_target) {
			gdb_putpacketz("E01");
			return;
		}
		handle_q_string_reply(target_tdesc(cur_target), packet + 31);
	} else if (sscanf(packet, "qCRC:%" PRIx32 ",%" PRIx32, &addr, &alen) == 2) {
		if(!cur_target) {
			gdb_putpacketz("E01");
			return;
		}
		gdb_putpacket_f("C%lx", generic_crc32(cur_target, addr, alen));

	} else {
		DEBUG("*** Unsupported packet: %s\n", packet);
		gdb_putpacket("", 0);
	}
}

static void
handle_v_packet(char *packet, int plen)
{
static uint8_t flash_mode = 0;
	unsigned long addr, len;
	int bin;

	if (sscanf(packet, "vAttach;%08lx", &addr) == 1) {
		/* Attach to remote target processor */
		cur_target = target_attach_n(addr, &gdb_controller);
		if(cur_target)
			gdb_putpacketz("T05");
		else
			gdb_putpacketz("E01");

	} else if (!strcmp(packet, "vRun;")) {
		/* Run target program. For us (embedded) this means reset. */
		if(cur_target) {
			target_reset(cur_target);
			gdb_putpacketz("T05");
		} else if(last_target) {
			cur_target = target_attach(last_target,
						   &gdb_controller);

                        /* If we were able to attach to the target again */
                        if (cur_target) {
                        	target_reset(cur_target);
                        	gdb_putpacketz("T05");
                        } else	gdb_putpacketz("E01");

		} else	gdb_putpacketz("E01");

	} else if (sscanf(packet, "vFlashErase:%08lx,%08lx", &addr, &len) == 2) {
		/* Erase Flash Memory */
		DEBUG("Flash Erase %08lX %08lX\n", addr, len);
		if(!cur_target) { gdb_putpacketz("EFF"); return; }

		if(!flash_mode) {
			/* Reset target if first flash command! */
			/* This saves us if we're interrupted in IRQ context */
			target_reset(cur_target);
			flash_mode = 1;
		}
		if(target_flash_erase(cur_target, addr, len) == 0)
			gdb_putpacketz("OK");
		else
			gdb_putpacketz("EFF");

	} else if (sscanf(packet, "vFlashWrite:%08lx:%n", &addr, &bin) == 1) {
		/* Write Flash Memory */
		len = plen - bin;
		DEBUG("Flash Write %08lX %08lX\n", addr, len);
		if(cur_target && target_flash_write(cur_target, addr, (void*)packet + bin, len) == 0)
			gdb_putpacketz("OK");
		else
			gdb_putpacketz("EFF");

	} else if (!strcmp(packet, "vFlashDone")) {
		/* Commit flash operations. */
		gdb_putpacketz(target_flash_done(cur_target) ? "EFF" : "OK");
		flash_mode = 0;

	} else {
		DEBUG("*** Unsupported packet: %s\n", packet);
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

	/* I have no idea why this doesn't work. Seems to work
	 * with real sscanf() though... */
	//sscanf(packet, "%*[zZ]%hhd,%08lX,%hhd", &type, &addr, &len);
	type = packet[1] - '0';
	sscanf(packet + 2, ",%" PRIx32 ",%d", &addr, &len);
	if(set)
		ret = target_breakwatch_set(cur_target, type, addr, len);
	else
		ret = target_breakwatch_clear(cur_target, type, addr, len);

	if (ret < 0) {
		gdb_putpacketz("E01");
	} else if (ret > 0) {
		gdb_putpacketz("");
	} else {
		gdb_putpacketz("OK");
	}
}

void gdb_main(void)
{
	gdb_main_loop(&gdb_controller, false);
}

/* sforth extension words */
#include "engine.h"
#include "sf-word-wizard.h"

static void wait_target_halted(void)
{

	target_addr watch;
	enum target_halt_reason reason;

	if(!cur_target) {
		/* Report "target exited" if no target */
		print_str("target not connected");
		return;
	}

	/* Wait for target halt */
	while(!(reason = target_halt_poll(cur_target, &watch))) {
		unsigned char c = gdb_if_poll_char();
		if((c == '\x03') || (c == '\x04')) {
			target_halt_request(cur_target);
		}
	}
	SET_RUN_STATE(0);

	/* Translate reason to GDB signal */
	switch (reason) {
	case TARGET_HALT_ERROR:
		print_str("target-lost\n");
		break;
	case TARGET_HALT_REQUEST:
		print_str("target-halted\n");
		break;
	case TARGET_HALT_WATCHPOINT:
		print_str("target-halted-watchpoint\n");
		break;
	case TARGET_HALT_FAULT:
		print_str("target-halted-fault\n");
		break;
	default:
		print_str("target-halted-breakpoint\n");
	}
}


static void do_swdp_scan(void) { sf_push(command_process(cur_target, "swdp_scan")); }
static void do_gdb_attach(void) { sf_push((cur_target = target_attach_n(1, &gdb_controller)) != 0); }
static void do_read_registers(void)
{
	if (cur_target)
	{
		int i, n;
		uint32_t arm_regs[n = (target_regs_size(cur_target) / sizeof(uint32_t))];
		target_regs_read(cur_target, arm_regs);
		sf_eval("base @ hex");
		for (i = 0; i < n; sf_push(arm_regs[i ++]), do_u_dot());
		sf_eval("cr base !");
	}
	else
	{
		print_str("target not connected\n");
	}
}
static void do_target_fetch(void)
{
uint32_t address, x;
	address = sf_pop();
	if (!cur_target)
	{
		print_str("target not connected\n");
		return;
	}
	if (!target_mem_read(cur_target, &x, address, sizeof x))
		sf_push(x);
	else
		sf_push(address), sf_eval(".( failed to read memory at address ) base @ swap hex . cr base !"), sf_push(-1);
}
static void do_target_single_step(void)
{
target_addr watch;
enum target_halt_reason reason;

	if (cur_target)
	{
		target_halt_resume(cur_target, true);
		SET_RUN_STATE(1);
		wait_target_halted();
	}
	else
		print_str("target not connected\n");
}
static void do_target_resume(void)
{
	if (cur_target)
	{
		target_halt_resume(cur_target, false);
		SET_RUN_STATE(1);
		wait_target_halted();
	}
	else
		print_str("target not connected\n");
}
static void do_quesstion_target_run_state(void)
{
target_addr watch;
	if (cur_target)
	{
		sf_push(target_halt_poll(cur_target, &watch));
		if (!sf_top())
			SET_RUN_STATE(0);
	}
	else
		print_str("target not connected\n");
}
static void do_target_request_halt(void)
{
	if (cur_target)
	{
		target_halt_request(cur_target);
		//print_str("requested target halt\n");
		wait_target_halted();
	}
	else
		print_str("target not connected\n");
}
static void do_target_memory_dump(void)
{
uint32_t len = sf_pop(), address = sf_pop();
	if (!cur_target)
	{
		print_str("target not connected\n");
		return;
	}
	else
	{
		union
		{
			uint32_t	idata[15];
			uint8_t		data[0];
		} buf;
		int i, x;
		while (len)
		{
			x = ((len > sizeof buf) ? sizeof buf : len);
			/*! \todo	handle errors here */
			target_mem_read(cur_target, buf.idata, address, x);
			for (i = 0; i < x; gdb_if_putchar_single(buf.data[i ++]));
			len -= x;
			address += x;
		}
	}
}
static void do_flash_erase(void)
{
uint32_t len = sf_pop(), addr = sf_pop();
	if(!cur_target)
	{
		print_str("target not connected\n");
		return;
	}

	/* Reset target if first flash command! */
	/* This saves us if we're interrupted in IRQ context */
	target_reset(cur_target);

	if(target_flash_erase(cur_target, addr, len) == 0 && target_flash_done(cur_target) == 0)
		print_str("flash erased successfully");
	else
		print_str("error erasing flash");
}
static void do_flash_write(void)
{
uint32_t len = sf_pop(), addr = sf_pop(), x;
uint8_t buf[256];
int result = 0, i;
	if(!cur_target)
	{
		print_str("target not connected\n");
		return;
	}
	while (len)
	{
		x = (len > sizeof buf) ? sizeof buf : len;
		for (i = 0; i < x; buf[i ++] = gdb_if_getchar_single());
		result |= target_flash_write(cur_target, addr, buf, x);
		addr += x;
		len -= x;
	}
	result |= target_flash_done(cur_target);
	if (!result)
		print_str("flash written successfully");
	else
		print_str("error writing flash");
}

static void do_question_target_mem_map(void)
{
	if(!cur_target)
	{
		print_str("target not connected\n");
		return;
	}
	print_str(target_mem_map(cur_target));
	print_str("\n");
}
static void do_target_reset(void)
{
	if(cur_target)
		target_reset(cur_target);
	else if(last_target) {
		cur_target = target_attach(last_target,
		                           &gdb_controller);
		target_reset(cur_target);
	}
}

static void do_breakpoint_set(void)
{
int result, length = sf_pop();
	/* ( address length --) */
	result = target_breakwatch_set(cur_target, TARGET_BREAK_HARD, sf_pop(), length);
	print_str(result ? "breakpoint-error\n" : "breakpoint-ok\n");
}
static void do_breakpoint_clear(void)
{
int result, length = sf_pop();
	/* ( address length --) */
	result = target_breakwatch_clear(cur_target, TARGET_BREAK_HARD, sf_pop(), length);
	print_str(result ? "breakpoint-error\n" : "breakpoint-ok\n");
}

static struct word dict_base_dummy_word[1] = { MKWORD(0, 0, "", 0), };
static const struct word custom_dict[] = {
	/* override the sforth supplied engine reset */
	MKWORD(dict_base_dummy_word,	0,		"swdp-scan",	do_swdp_scan),
	MKWORD(custom_dict,		__COUNTER__,	"gdb-attach",	do_gdb_attach),
	MKWORD(custom_dict,		__COUNTER__,	"?regs",	do_read_registers),
	MKWORD(custom_dict,		__COUNTER__,	"t@",		do_target_fetch),
	MKWORD(custom_dict,		__COUNTER__,	"step",		do_target_single_step),
	MKWORD(custom_dict,		__COUNTER__,	"target-resume",		do_target_resume),
	MKWORD(custom_dict,		__COUNTER__,	"?target-run-state",		do_quesstion_target_run_state),
	MKWORD(custom_dict,		__COUNTER__,	"target-request-halt",		do_target_request_halt),
	MKWORD(custom_dict,		__COUNTER__,	"?target-mem-map",		do_question_target_mem_map),
	MKWORD(custom_dict,		__COUNTER__,	"target-dump",		do_target_memory_dump),
	MKWORD(custom_dict,		__COUNTER__,	"target-reset",		do_target_reset),
	MKWORD(custom_dict,		__COUNTER__,	"flash-erase",		do_flash_erase),
	MKWORD(custom_dict,		__COUNTER__,	"flash-write",		do_flash_write),
	MKWORD(custom_dict,		__COUNTER__,	"breakpoint-set",	do_breakpoint_set),
	MKWORD(custom_dict,		__COUNTER__,	"breakpoint-clear",	do_breakpoint_clear),

}, * custom_dict_start = custom_dict + __COUNTER__;

static void sf_gdb_main_init(void) __attribute__((constructor));
static void sf_gdb_main_init(void)
{
	sf_merge_custom_dictionary(dict_base_dummy_word, custom_dict_start);
}
