/* /src/kernel.c */
#include "io.h"

void main()
{
    // uart_init();
    // uart_writeText("Hello world!\n");

    // Spin into infinite loop (this kmain() shouldnt return, since its the OS kernel (this function) that controls every event in the hardware)
    while (1)
        ;
}