
volatile int x = 20, y = 5;

static int fib(int n)
{
	if (n < 2)
		return n;
	return fib(n - 1) 
		+ fib(n - 2);
}

int get_fib(void)
{
	return fib(x);
}
