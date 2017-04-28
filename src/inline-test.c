
volatile int x = 20, y = 5;

static int fib(int n)
{
	if (n < 2)
		return n;
	return fib(n - 1) 
		+ fib(n - 2);
}


void adiv5_mem_read(void *ap, void *dest, int src, int len);

int get_fib(int src, int len)
{
	adiv5_mem_read(0, 0, 0, 0);
	return fib(x);
}
