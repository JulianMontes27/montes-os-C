#include "io.h"

typedef unsigned short int16;
typedef unsigned int int32;
typedef unsigned long int64;
typedef unsigned char int8;

// CPU Core<--> Memory Bus<--> BCM2711 SoC<--> GPIO Controller<--> Physical Pins<--> UART Controller

/**
 * CPU works with 8 bits at once (a byte), but a serial wire can only send 1 bit at a time. UART is the translator between these two worlds.
 * The GPIO registers are mapped to specific physical memory addresses
 * For RPi 4 (BCM2711), the GPIO base address is typically 0xFE200000 (this changed from 0x20200000 in earlier Pi models)
 * The ARM Cortex-A72 CPU is 64-bit and can process 64-bit memory operations.
 * The peripheral bus (where GPIO registers are mapped) is a separate hardware block that retains 32-bit registers for:
 * Compatibility: Older Raspberry Pi OS/drivers assume 32-bit MMIO.
 * Simplicity: 32 bits are sufficient for control registers (e.g., setting a GPIO pin).
 * How 32-bit Registers Work in a 64-bit System
 * The RPi4’s MMIO (Memory-Mapped I/O) region is hardwired for 32-bit accesses.
 * When you write to 0xFE200000 (GPIO base), the CPU:
 * Issues a 32-bit write (even if you use *(volatile uint64_t*)addr).
 * The peripheral bus ignores the upper 32 bits.
 * Misaligned/64-bit writes may fault or behave unpredictably.
 */

// Why these specific addresses ?
// Broadcom designed the BCM2711 chip with peripherals mapped to these addresses 0xFE000000 is the base where all peripherals start
// Each peripheral gets its own offset from this base
// This is documented in the BCM2711 datasheet

// BCM2711 peripheral base address (mapped to physical memory addresses)
#define PERIPHERAL_BASE 0xFE000000              // This is where peripherals start in the BCM2711
#define GPIO_BASE (PERIPHERAL_BASE + 0x200000)  // 0xFE200000
#define UART0_BASE (PERIPHERAL_BASE + 0x201000) // 0xFE201000

// GPIO Registers
#define GPFSEL1 (GPIO_BASE + 0x04)
#define GPPUD (GPIO_BASE + 0x94)
#define GPPUDCLK0 (GPIO_BASE + 0x98)

// UART0 (PL011) registers
#define UART0_DR (UART0_BASE + 0x00)   // Data Register
#define UART0_FR (UART0_BASE + 0x18)   // Flag Register
#define UART0_IBRD (UART0_BASE + 0x24) // Integer Baud Rate Divisor
#define UART0_FBRD (UART0_BASE + 0x28) // Fractional Baud Rate Divisor
#define UART0_LCRH (UART0_BASE + 0x2C) // Line Control Register
#define UART0_CR (UART0_BASE + 0x30)   // Control Register
#define UART0_ICR (UART0_BASE + 0x44)  // Interrupt Clear Register

/* --- Memory Access Functions --- */
/**
 * For mmio on the RPi4, the best practice is to use 32-bit accesses (even if the CPU is 64-bit)
 * This function writes a 32-bit value to a memory-mapped I/O (MMIO) address (this is how interaction with hardware on the RPi4).
 * addr: The physical memory address of the hardware register to write to
 * value: The 32-bit value you want to write
 * 'volatile' tells the GCC compiler to NOT optimize away this memory access (as hardwarw regs. can change outside of program flow); and that the write has side effects (changing hardware state)
 */
static inline void mmio_write(int64 addr, int32 value)
{
    // Type cast the addr from an int to a volatile int32 pointer type
    // Perform memory write by dereferencing the addr pointer and assigning it to value
    // The compiler geenrates a 32-bit STORE instr
    *(volatile int32 *)addr = value;
}

static inline int32 mmio_read(int64 addr)
{
    // Cast the int64 addr number into a int32.
    // Addr is now a pointer that points to addr in memory
    // Dereference the addr pointer to get the value stored in memory
    return *(volatile int32 *)addr;
}

void delay(int32 count)
{
    for (volatile int32 i = 0; i < count; i++)
        ;
}

/* GPIO Configuration */
/**
 * Controls what each GPIO pin does (input, output, or special function)
 */
void gpi_set_function(int8 pin, int8 function)
{
    int32 reg = pin / 10; // Which GPFSEL register?
    int32 shift = (pin % 10) * 3; // Which bits in that register?
    int64 addr = GPIO_BASE + (reg * 4);

    int32 value = mmio_read(addr);
}

// Initialize the UART
void uart_init(void)
{
    // Disable UART0
    mmio_write(UART0_CR, 0);

    // Configure GPIO pins 14 and 15 for UART
    gpio_set_function(14, 4); // GPIO 14 -> ALT0 (UART0_TXD)
    gpio_set_function(15, 4); // GPIO 15 -> ALT0 (UART0_RXD)

    // Disable pull up/down for pins 14 and 15
    gpio_set_pull(14, 0);
    gpio_set_pull(15, 0);

    // Clear pending interrupts
    mmio_write(UART0_ICR, 0x7FF);

    // Set baud rate to 115200
    // UART clock = 48MHz
    // Baud rate divisor = 48000000 / (16 * 115200) = 26.041666...
    mmio_write(UART0_IBRD, 26); // Integer part
    mmio_write(UART0_FBRD, 3);  // Fractional part (0.041666 * 64 = 2.67 ≈ 3)

    // Enable FIFO, 8-bit data transmission, 1 stop bit, no parity
    mmio_write(UART0_LCRH, 0x70);

    // Enable UART0, receive, and transmit
    mmio_write(UART0_CR, 0x301);
}

void uart_putc(char c)
{
    mmio_write(UART0_DR, c); // Uses mmio_write to send a byte
}

char uart_getc()
{
    return mmio_read(UART0_DR); // Reads a byte (needs status checks!)
}