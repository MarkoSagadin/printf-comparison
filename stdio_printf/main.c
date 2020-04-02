#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/systick.h>

static void clock_setup()
{
    // First, let's ensure that our clock is running off the high-speed internal
    // oscillator (HSI) at 48MHz.
    rcc_clock_setup_hsi(&rcc_3v3[RCC_CLOCK_3V3_48MHZ]);

    //Enable periphial clocks that we will need
    rcc_periph_clock_enable(RCC_USART3);
    rcc_periph_clock_enable(RCC_GPIOD);
    rcc_periph_clock_enable(RCC_GPIOB);
}

/*
 * This is where we setup our systick
 */ 
static void systick_setup()
{
    // Set the systick clock source to our main clock
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    // Clear the Current Value Register so that we start at 0
    STK_CVR = 0;
    // In order to trigger an interrupt every millisecond, we can set the reload
    // value to be the speed of the processor / 1000 -1
    systick_set_reload(rcc_ahb_frequency / 1000 - 1);
    // Enable interrupts from the system tick clock
    systick_interrupt_enable();
    // Enable the system tick counter
    systick_counter_enable();
}

/*
 * Storage for our monotonic system clock.
 */ 
static volatile uint64_t _millis = 0;

uint64_t millis()
{
    return _millis;
}

/*
 * This is our interrupt handler for the systick reload interrupt.
 */ 
void sys_tick_handler(void)
{
    // Increment our monotonic clock
    _millis++;
}

/*
 * Delay for a real number of milliseconds
 */
void delay(uint64_t duration)
{
    const uint64_t until = millis() + duration;
    while (millis() < until);
}


/*
 * Setup of uart
 */
static void usart_setup(void)
{
	/* Setup GPIO pins for USART3 transmit. */
	gpio_mode_setup(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8);
	gpio_set_af(GPIOD, GPIO_AF7, GPIO8);

	/* Setup USART3 parameters. */
	usart_set_baudrate(USART3, 115200);
	usart_set_databits(USART3, 8);
	usart_set_stopbits(USART3, USART_STOPBITS_1);
	usart_set_mode(USART3, USART_MODE_TX);
	usart_set_parity(USART3, USART_PARITY_NONE);
	usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);

	/* Finally enable the USART. */
	usart_enable(USART3);
}


static void gpio_setup()
{
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO4);
}

ssize_t _write(int file, const char *ptr, ssize_t len);

int _write(int file, const char *ptr, ssize_t len) {
    // If the target file isn't stdout/stderr, then return an error
    // since we don't _actually_ support file handles
    if (file != STDOUT_FILENO && file != STDERR_FILENO) {
        // Set the errno code (requires errno.h)
        errno = EIO;
        return -1;
    }

    // Keep i defined outside the loop so we can return it
    int i;
    for (i = 0; i < len; i++) {
        // If we get a newline character, also be sure to send the carriage
        // return character first, otherwise the serial console may not
        // actually return to the left.
        if (ptr[i] == '\n') {
            usart_send_blocking(USART3, '\r');
        }

        // Write the character to send to the USART1 transmit buffer, and block
        // until it has been sent.
        usart_send_blocking(USART3, ptr[i]);
    }

    // Return the number of bytes we sent
    return i;
}

int main() 
{
    clock_setup();
    systick_setup();
    gpio_setup();
    usart_setup();


    float f_num = 3.14;
    float e_num = 0.00034;
    long long int longi = 4;

    int a = 1;
    int b = 2;
    int* pa = &a;
    int* pb = &b;
    ptrdiff_t diff = pb - pa;

    // Toggle the LED on and off forever
    while (1) 
    {
        gpio_set(GPIOB, GPIO4);
        printf("This is a float number: %f\n", f_num);
        printf("This is a float in exp notation: %e\n", e_num);
        printf("This is long long int: %lli\n", longi);
        printf("This is ptrdiff_t: %td\n", diff);
        gpio_clear(GPIOB, GPIO4);
        delay(30);
    }

    return 0;
}

