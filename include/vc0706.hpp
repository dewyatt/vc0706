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
#include <boost/asio/serial_port.hpp>
#include <vector>

//This is the default baud rate at startup, stored in EEPROM
const unsigned int DefaultBaudRate = 38400;

class VC0706
{
public:
    enum ImageSize {
        Image_640x480 = 0x00,
        Image_320x240 = 0x11,
        Image_160x120 = 0x22
    };

    typedef std::vector<uint8_t> DataBuffer;

    VC0706();
    //Construct and open()
    VC0706(const std::string& device, unsigned int rate = DefaultBaudRate);

    void open(const std::string& device, unsigned int rate = DefaultBaudRate);
    void close();

    //Modify the baud rate. The rate will reset to the default (in EEPROM) after system reset.
    void set_baud_rate(unsigned int rate);

    //Get the firmware version
    std::string get_version();

    //Reset the device
    void system_reset();

    //Toggle TV output
    void tv_on();
    void tv_off();

    //Freeze, step, and resume the framebuffer.
    void freeze_picture();
    void step_picture();
    void resume_picture();
    //Read the frame (must call freeze_picture first)
    const DataBuffer& read_picture();

    ImageSize get_image_size();
    void set_image_size(ImageSize size);

    uint8_t get_compression();
    void set_compression(uint8_t compression);

    //enable motion detection, must poll motion_detected() after
    void motion_detect_on();
    //disable motion detection
    void motion_detect_off();
    //check for motion detection
    bool motion_detected();

private:
    struct Command {
        uint8_t sign;
        uint8_t serial_number;
        uint8_t command;
        uint8_t data_length;
        uint8_t data[16];
    };
    
    struct Response {
        uint8_t sign;
        uint8_t serial_number;
        uint8_t command;
        uint8_t status;
        uint8_t data_length;
        uint8_t data[16];
    };
    
    std::string my_device;
    boost::asio::io_service my_io_service;
    boost::asio::serial_port my_port;
    Command my_command;
    Response my_response;
    DataBuffer my_image_buffer;

    void send_command(uint8_t command, uint8_t args[] = 0, uint8_t arg_count = 0);
    bool read_response(uint8_t command, unsigned int ms = 50, bool timeout_allowed = false);
    bool read_timeout(uint8_t *buffer, unsigned int length, unsigned int ms);
    bool write_timeout(uint8_t *buffer, unsigned int length, unsigned int ms);
    uint32_t get_fbuf_len();
    void motion_ctrl(uint8_t attribute, uint8_t interface, uint8_t enable);
};

