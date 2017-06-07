#include <stddef.h>
#include <stdint.h>
#include "inline-test.h"
#include "target/adiv5.h"

volatile int x = 20, y = 5;

static int fib(int n)
{
	if (n < 2)
		return n;
	return fib(n - 1) 
		+ fib(n - 2);
}


int get_fib(int src, int len)
{
	adiv5_mem_read(0, 0, 0, 0);
	return fib(x);
}


void f_struct_short(struct short_struct s){}
void f_struct_longer(struct longer_struct s){}
void f_struct_long(struct long_struct s){}

void do_adiv5_dp_write_proxy(ADIv5_DP_t *dp, uint16_t addr, uint32_t value)
{
	value ++;
	addr += addr << 12;
	dp += 5;
	adiv5_dp_write(dp, addr, value);
}

int param_test(int a, int b, int c, int d, int e)
{
	return do_param_test_1(e, d, c, b, a);
}

