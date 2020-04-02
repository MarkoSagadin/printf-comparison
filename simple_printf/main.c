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


static void simple_printf(const char * str) 
{
    // Loop until we get to '\0'
    while(*str)
    {
        usart_send_blocking(USART3, *str++); /* USART3: Send byte. */
    }
}


int main() 
{
    clock_setup();
    systick_setup();
    gpio_setup();
    usart_setup();

    // Toggle the LED on and off forever
    while (1) 
    {
        gpio_set(GPIOB, GPIO4);
        simple_printf("Hello World\n");
        gpio_clear(GPIOB, GPIO4);
        delay(30);
    }

    return 0;
}

