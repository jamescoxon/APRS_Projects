Arduino ARPS example project
============================

The code is organized as follows:
 * aprs/   - The source code of the application. Have a look at main.c for the entry
             point of the program.
 * images/ - Precompiled binaries of the application. You can flash .bin or .hex
             images to your board to instantly try the application.
 * obj/    - Compiled object files
 * bertos/ - Complete BeRTOS sources and documentation.
 * aprs.workspace - BeRTOS IDE (CodeLite) workspace file. You can still use any other
                    editor of your choice.

Compilation
===========
Compilation of this source distribution may not work out of the box on your machine.
This is not a problem because you can always regenerate this example project
on your machine using the Wizard.
Check the tutorial here:
http://www.bertos.org/use/tutorial-front-page/wizard-user-guide/

If you still want to try compiling in your box without using the wizard, read on.
You need to change the toolchain path by editing the file
    aprs/aprs.mk
and changing the following variables:
    aprs_PREFIX = "your/path/goes/here/avr-"
    aprs_SUFFIX = ".extension-if-any"

PREFIX should contain the path to your avr toolchain. The "avr-" in the filename
is necessary.
SUFFIX should contain the extension of the compiler (if any). Most probably it
should be ".exe" on Windows and "" on Mac and Linux.
