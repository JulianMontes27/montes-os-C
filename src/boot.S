; /src/boot.s
; AArch64 assembly boot code for Raspberry Pi 4
; This code runs immediately after the GPU bootloader transfers control

.section ".text.boot"  ; Linker directive: place this section at start of kernel image
                       ; The linker script will position this at 0x80000 (default kernel load address)

.global _start         ; Export _start symbol for linker - this becomes the entry point in kernel8.img

_start:
    ; === MULTI-CORE INITIALIZATION ===
    ; Pi 4 has 4 ARM Cortex-A72 cores that all start executing simultaneously
    ; We need to put 3 cores to sleep and let only core 0 continue
    
    mrs     x1, mpidr_el1      ; Move System Register: read Multiprocessor Affinity Register
                               ; mpidr_el1[1:0] contains the core ID (0, 1, 2, or 3)
                               ; EL1 = Exception Level 1 (kernel/supervisor mode)
    
    and     x1, x1, #3         ; Bitwise AND with 3 (0b11) to isolate bottom 2 bits
                               ; This extracts just the core ID: 0=main core, 1-3=secondary cores
    
    cbz     x1, 2f             ; Compare and Branch if Zero: if core ID == 0, jump to label 2 (main core path)
                               ; 'f' means forward reference to label 2

    ; === SECONDARY CORE SLEEP LOOP ===
    ; Cores 1, 2, 3 execute this infinite loop
1:  wfe                        ; Wait For Event: puts core into low-power state until woken by SEV instruction
                               ; This is more power-efficient than a busy loop
    b       1b                 ; Branch to label 1 (backward reference): infinite loop
                               ; Secondary cores stay here until main core sends an event

2:  ; === MAIN CORE CONTINUES HERE ===
    ; Only core 0 reaches this point

    ; === STACK POINTER INITIALIZATION ===
    ; AArch64 uses SP (Stack Pointer) register for function calls and local variables
    ldr     x1, =_start        ; Load address of _start symbol into x1
                               ; This is typically 0x80000 (512KB) on Pi 4
    mov     sp, x1             ; Move address to Stack Pointer register
                               ; Stack grows DOWNWARD from this address
                               ; WARNING: This places stack in same area as code - dangerous for larger kernels!

    ; === BSS SECTION INITIALIZATION ===
    ; BSS = Block Starting Symbol (uninitialized global/static variables in C)
    ; C standard requires these to be zero-initialized before main() is called
    
    ldr     x1, =__bss_start   ; Load start address of BSS section (defined in linker script)
    ldr     w2, =__bss_size    ; Load size of BSS section in bytes (w2 = 32-bit portion of x2)
                               ; Note: using w2 instead of x2 assumes BSS < 4GB

3:  cbz     w2, 4f             ; Compare and Branch if Zero: if size == 0, jump to label 4 (done)
    
    str     xzr, [x1], #8      ; Store Zero Register to memory at [x1], then increment x1 by 8
                               ; xzr = 64-bit zero register (always contains 0)
                               ; Post-increment addressing: store 8 bytes of zero, advance pointer
    
    sub     w2, w2, #1         ; Subtract 1 from remaining size counter
                               ; NOTE: This is incorrect! Should be 'sub w2, w2, #8' since we store 8 bytes
    
    cbnz    w2, 3b             ; Compare and Branch if Not Zero: if size != 0, loop back to label 3

    ; === TRANSFER CONTROL TO C CODE ===
4:  bl      main               ; Branch with Link: call main() function
                               ; Saves return address in x30 (Link Register)
                               ; At this point: stack set up, BSS cleared, single core running
    
    ; === HALT ON RETURN ===
    ; main() should never return in a kernel, but if it does:
    b       1b                 ; Jump back to the WFE sleep loop
                               ; This prevents undefined behavior if main() exits

; === REGISTER USAGE SUMMARY ===
; x0: Available for main() (by ARM calling convention, contains first argument)
; x1: Used for temporary addresses and calculations
; x2/w2: Used for BSS size counter  
; x30 (LR): Contains return address from 'bl main'
; sp: Stack pointer, set to _start address
; Other registers: Undefined state, main() should initialize as needed

; === MEMORY LAYOUT AT THIS POINT ===
; 0x00000000 - 0x3F000000: GPU memory and peripherals
; 0x80000 (typical): Kernel code starts here (_start)
; Stack: Grows down from 0x80000 (overlaps with code - fix this!)
; BSS: Somewhere after code, zeroed out
; 0x3F000000+: Peripheral registers (GPIO, UART, etc.)