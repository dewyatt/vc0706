vc0706
======

Library to interface with a VC0706 security camera via a serial port.

Specifically, I tested the one from adafruit: http://www.adafruit.com/products/397

I used a RS-232 level shifter from sparkfun: https://www.sparkfun.com/products/449

Usage
=====

See test/test.cpp. It's simple.

Test
====

```
$ cd vc0706/test
$ make
$ ./test
Usage: ./test <port> <baud> <command>
        port - /dev/ttyS0, etc
        baud - baud rate (try 38400)
        command - tv, motion, picture
Example:
        ./test /dev/ttyS0 38400 motion
```
