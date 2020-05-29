
Instructions for building and burning the bootloader with PlatformIO.

Provided by msquirogac, see https://github.com/msquirogac/avr-debugger-bootloader

# avr-debugger bootloader
How to use it?


1) Customize the `platformio.ini` file to match the programmer and board you have. For example, if you have an **usbasp** programmer and an **Arduino Nano**, then change it like this:

~~~
[env:loader]
upload_protocol = usbasp
board = nanoatmega328new
~~~

What you put into [board](https://docs.platformio.org/en/latest/boards/index.html) will always depend on the hardware you have, or if you have a personalized one you can also use the generic definition. For example [328p16m](https://docs.platformio.org/en/latest/boards/atmelavr/328p16m.html) means an `atmega328p` IC with a 16MHz clock. Besides that, you can and should also configure the `fuses`.

So if you have a bare `atmega328p` in a breadboard without external crystal then you use `328p8m` and change the `board_fuses.lfuse` to `0xE2`. What those magic numbers means?
Easy, just use a calculator like [this](https://eleccelerator.com/fusecalc/fusecalc.php?chip=atmega328p&LOW=E2&HIGH=D2&EXTENDED=FD&LOCKBIT=3F) one. Also remember to configure de baudrate, the default value is 115200, nonetheless low MHz crystals or the internal one aren't suitable for that baudrate, heceforth it should be lowered to 57600 at least.

2) Connect your programmer to the board.
3) Finally, compile and program the bootloader with:
~~~
pio run -t fuses && pio run -t program
~~~

Note: You can run the above command from VS Code with PlatformIO - click the PlatformIO: New Terminal button in PIO toolbar to open terminal, then switch to the optoboot folder (...\avr_debug\bootloader\optiboot) and then run the command.

Done!! Now your board has the **avr-debugger** bootloader installed and is able to use the **Flash breakpoint mode**. The major difference with the regular one comes when you use the avr-debugger library, which means that you don't need to revert to the regular one even if you don't need this library anymore.

