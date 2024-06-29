#include "stm32l4xx.h"

void _exit (int status)
{
	UNUSED(status);
	__BKPT(0);
	while (1);
}