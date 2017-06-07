/*
Copyright (c) 2014-2016 stoyan shopov

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/
#include "engine.h"
#include "sf-word-wizard.h"
#include "inline-test.h"

#include "target/adiv5.h"

void do_f_struct_short(void)
{
	f_struct_short((struct short_struct) { .a = 2, .b = 6, });
}
void do_f_struct_longer(void)
{
struct longer_struct x = { .a = 2, .b = 6, };
	//f_struct_longer((struct longer_struct) { .a = 2, .b = 6, });
	f_struct_longer(x);
}
void do_f_struct_long(void)
{
	f_struct_long((struct long_struct) { .a = 2, .b = 6, .c = 10, .d = 20, .e = 30, .f = 40,});
}

static void do_squared(void) { cell x = sf_pop(); sf_push(x * x); }
static void do_gcd(void)
/* implements the FORTH word:
: gcd ( n1 n2 | n) begin over mod swap over 0= until nip ;
*/
{ do { do_over(); do_mod(); do_swap(); do_over(); do_zero_not_equals(); } while (sf_pop()); do_nip(); }

void invoke_slow_mem_write(void);

void stm32lx_nvm_lock(int, int);

void call_nvm_lock(void)
{
	//stm32lx_nvm_lock(0x1120, /*0x1234*/ sf_pop());
	stm32lx_nvm_lock(0x1120, 0x1234);
}


uint32_t adiv5_swdp_low_access(void *dp, uint8_t RnW,
				      uint16_t addr, uint32_t value);

void do_swdp_low_1(void)
{
	adiv5_swdp_low_access(0x300, 1, 0x200, 0xabc);
}
void do_swdp_low_2(void)
{
	adiv5_swdp_low_access(0x400, 0, 0x101, 0xdef);
}
uint32_t low_address = 0x401;
static void * void_ = 0x300;
int rnw = 12, value_ = 450;
void do_swdp_low_3(void)
{
	adiv5_swdp_low_access(void_, rnw, low_address, value_);
}

ADIv5_DP_t dpx = { .low_access = do_swdp_low_1, };

void do_adiv5_dp_write(void)
{
	do_adiv5_dp_write_proxy((& dpx) - 5, 225, 312);
}

extern void get_fib();
int do_param_test(void);
static struct word dict_base_dummy_word[1] = { MKWORD(0, 0, "", 0), };
static const struct word custom_dict[] = {
	/* override the sforth supplied engine reset */
	MKWORD(dict_base_dummy_word,	0,		"squared",	do_squared),
	MKWORD(custom_dict,		__COUNTER__,	"gcd",	do_gcd),
	MKWORD(custom_dict,		__COUNTER__,	"fib",	get_fib),
	MKWORD(custom_dict,		__COUNTER__,	"slow-mem-write",	invoke_slow_mem_write),
	MKWORD(custom_dict,		__COUNTER__,	"nvm-lock",	call_nvm_lock),
	MKWORD(custom_dict,		__COUNTER__,	"short-struct",	do_f_struct_short),
	MKWORD(custom_dict,		__COUNTER__,	"longer-struct",	do_f_struct_longer),
	MKWORD(custom_dict,		__COUNTER__,	"long-struct",	do_f_struct_long),
	MKWORD(custom_dict,		__COUNTER__,	"swdp-low-1",	do_swdp_low_1),
	MKWORD(custom_dict,		__COUNTER__,	"swdp-low-2",	do_swdp_low_2),
	MKWORD(custom_dict,		__COUNTER__,	"swdp-low-3",	do_swdp_low_3),
	MKWORD(custom_dict,		__COUNTER__,	"adiv5-dp-write",	do_adiv5_dp_write),
	MKWORD(custom_dict,		__COUNTER__,	"param-test",	do_param_test),

}, * custom_dict_start = custom_dict + __COUNTER__;

static void sf_opt_sample_init(void) __attribute__((constructor));
static void sf_opt_sample_init(void)
{
	sf_merge_custom_dictionary(dict_base_dummy_word, custom_dict_start);
}


int do_param_test_1(int a, int b, int c, int d, int e)
{
	return a + b + c + d + e;
}
int do_param_test(void)
{
	param_test(10, 20, 30, 40, 50);
}
