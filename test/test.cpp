/**
* The MIT License (MIT)
*
* Copyright (c) 2013 Daniel Wyatt
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
**/
#include "vc0706.hpp"
#include <cstdio>
#include <fstream>

VC0706 cam;

void sleepms(unsigned int ms)
{
    usleep(ms * 1000);
}

void cmd_tv() {
    cam.tv_off();
    printf("TV output OFF\n");
    sleepms(2000);
    cam.tv_on();
    printf("TV output ON\n");
    sleepms(2000);
    cam.freeze_picture();
    printf("Picture frozen...\n");
    sleepms(2000);
    for (int i = 0; i < 3; i++) {
        cam.step_picture();
        printf("Step frame...\n");
        sleepms(1000);
    }
    cam.resume_picture();
    printf("Picture resumed...\n");
}

void cmd_motion() {
    cam.motion_detect_on();
    printf("Kill with ctrl+c...\n");
    while (true) {
        sleepms(100);
        if (cam.motion_detected())
            putc('+', stdout);
        else
            putc('.', stdout);

        fflush(stdout);
    }
}

void cmd_picture() {
    cam.freeze_picture();
    printf("Saving picture (%d bytes)...", cam.get_picture_size());
    fflush(stdout);
    const VC0706::DataBuffer& data = cam.read_picture();
    cam.resume_picture();

    std::ofstream outf("picture.jpg", std::ios::binary);
    outf.write((char*)&data[0], data.size());
    outf.close();
}

int main(int argc, char *argv[])
{
    if ( argc != 4 ) {
        printf("Usage: %s <port> <baud> <command>\n", argv[0]);
        printf("\tport - /dev/ttyS0, etc\n");
        printf("\tbaud - baud rate (try 38400)\n");
        printf("\tcommand - tv, motion, picture\n");
        printf("Example:\n");
        printf("\t%s /dev/ttyS0 38400 motion\n", argv[0]);
        return 1;
    }
    cam.open(argv[1], atoi(argv[2]));
    printf("Firmware version: %s\n", cam.get_version().c_str());

    if ( strcmp(argv[3], "tv") == 0 )
        cmd_tv();
    else if ( strcmp(argv[3], "motion") == 0 )
        cmd_motion();
    else if ( strcmp(argv[3], "picture") == 0 )
        cmd_picture();
    else {
        printf("Invalid command: %s\n", argv[1]);
        return 1;
    }
    printf("Done\n");
    return 0;
}

