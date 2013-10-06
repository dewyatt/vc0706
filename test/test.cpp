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

void save_picture(const char *filename)
{
    cam.freeze_picture();
    const VC0706::DataBuffer& data = cam.read_picture();
    cam.resume_picture();

    std::ofstream outf(filename, std::ios::binary);
    outf.write((char*)&data[0], data.size());
    outf.close();
}

int main(int argc, char *argv[])
{
    if ( argc != 3 ) {
        printf("Usage: %s <port> <baud>\n\n", argv[0]);
        printf("Example:\n");
        printf("\t%s /dev/ttyS0 38400\n", argv[0]);
        return 1;
    }
    cam.open(argv[1], atoi(argv[2]));
    printf("Version: %s\n", cam.get_version().c_str());
    printf ("Compression: %d\n", cam.get_compression());

    cam.tv_on();
    cam.motion_detect_on();
    while (true) {
        sleepms(100);
        if (cam.motion_detected()) {
            printf("Motion detected\n");
            cam.motion_detect_off();
            save_picture("capture.jpg");
            cam.motion_detect_on();
        }
    }
}

