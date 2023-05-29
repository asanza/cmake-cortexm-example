STM32 CMake Example
===================

This is an example project to show how to use cmake to build a simple
firmware for an STM32F103 nucleo board. 

The firmware is based on FreeRTOS and just blink the nucleo's green led.

Getting Started
===============

1. Install the arm-none-eabi toolchain on your computer and add it to the system path
2. Install the ninja build system. Add it to the system path
3. Install CMake. Add it to the system path.
4. Install OpenOCD. Add it to the system path.
5. go to the build folder, do: 

```
    $> cmake -GNinja ..
    $> ninja
```

This should build the project. To flash it to the nucleo board, connect the board and on the 
terminal write:

```
 $> ninja flash
```

To run the unit tests:

```
 $> ninja test
```

How it works
============

The firmware is made of following parts:

1. The main application, which is located in the src/app folder
2. The system libraries, which are located in the src/platform folder
3. The unit tests, which are located in the test folder

The main application is a simple blinky example. It blinks the green led of the nucleo board
at a single frequency. We use Freertos to spawn a single task which does this.

The cubeMX was used to generate the low driver code. We compile this into a library and use from
the application board.

The unit tests use the Unity unit test framework and qemu to run.

