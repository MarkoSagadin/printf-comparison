# Printf comparison
Example project where we can see comparison between different implementations of printf function. Project is prepared to compile for stm32f767zi, but it can be easily changed to some other microcontroller by setting macro TARGET in projects Makefile. Openocd.cfg should also be changed accordingly.

## Initial setup
 1. git clone --recurse-submodules https://github.com/MarkoSagadin/printf_comparison.git
 2. make -C libopencm3 # (Only needed once)

## Usual develop, flash cycle
Move into project.
```
make        # compile
openocd     # flash
minicom     # open serial port
```

Credits
[libopencm3](https://github.com/libopencm3)
[mpaland/printf](https://github.com/mpaland/printf)
[useful tutorial about printf](https://rhye.org/post/stm32-with-opencm3-1-usart-and-printf/)
