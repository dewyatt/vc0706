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
#include <boost/asio/read.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/asio/write.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/optional.hpp>
#include <boost/bind.hpp>
#include <cstdio>
#include <iostream>

/* 
   Command format:
    ProtocolSign(0x56) SerialNumber(1byte) Command(1byte) DataLength(1byte) Data(0~16bytes)
   
   Response format:
    ProtocolSign(0x76) SerialNumber(1byte) Command(1byte) Status(1byte) DataLengths(1byte) Data(0~16bytes)
*/

using boost::asio::serial_port;
using boost::asio::async_read;
using boost::asio::read;
using boost::asio::write;
using boost::asio::buffer;

const uint8_t StatusOk = 0;

const uint8_t CommandSign = 0x56;
const uint8_t ResponseSign = 0x76;

const uint8_t SerialNumber = 0x00;

const uint8_t CommandSystemReset = 0x26;
const uint8_t CommandGetVersion = 0x11;
const uint8_t CommandTVOutCtrl = 0x44;
const uint8_t CommandFBufCtrl = 0x36;
const uint8_t CommandSetPort = 0x24;
const uint8_t CommandGetFBufLen = 0x34;
const uint8_t CommandReadFBuf = 0x32;
const uint8_t CommandReadData = 0x30;
const uint8_t CommandWriteData = 0x31;
const uint8_t CommandMotionCtrl = 0x42;
const uint8_t CommandCommMotionCtrl = 0x37;

const uint8_t TVOutStart = 0x01;
const uint8_t TVOutStop = 0x00;

const uint8_t FBufStopCurrentFrame = 0x00;
const uint8_t FBufStopNextFrame = 0x01;
//These two are swapped in the datasheet
const uint8_t FBufStepFrame = 0x02;
const uint8_t FBufResumeFrame = 0x03;

const uint8_t CurrentFrame = 0x00;
const uint8_t NextFrame = 0x01;

VC0706::VC0706() : my_port(my_io_service)
{

}

VC0706::VC0706(const std::string& device, unsigned int rate) : my_port(my_io_service)
{
    open(device, rate);
}

void VC0706::open(const std::string& device, unsigned int rate)
{
    close();

    my_port.open(device);
    my_port.set_option(serial_port::baud_rate(rate));
    my_port.set_option(serial_port::flow_control(serial_port::flow_control::none));
    my_port.set_option(serial_port::parity(serial_port::parity::none));
    my_port.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    my_port.set_option(serial_port::character_size(8));
    my_device = device;
}

void VC0706::close()
{
    my_port.close();
}

void VC0706::set_baud_rate(unsigned int rate)
{
    uint8_t args[] = {0x01, 0x00, 0x00};
    switch (rate) {
        case 9600:
            args[1] = 0xAE;
            args[2] = 0xC8;
            break;
        case 19200:
            args[1] = 0x56;
            args[2] = 0xE4;
            break;
        case 38400:
            args[1] = 0x2A;
            args[2] = 0xF2;
            break;
        case 57600:
            args[1] = 0x1C;
            args[2] = 0x1C;
            break;
        case 115200:
            args[1] = 0x0D;
            args[2] = 0xA6;
            break;
        default:
            throw std::runtime_error("Invalid baud rate");
            break;
    }
    send_command(CommandSetPort, args, sizeof(args));
    my_port.set_option(serial_port::baud_rate(rate));
}

std::string VC0706::get_version()
{
    send_command(CommandGetVersion);
    return std::string((char*)&my_response.data[0], my_response.data_length);
}

void VC0706::system_reset()
{
    send_command(CommandSystemReset);
    /*
    Something like the following is sent after a system reset:
    VC0703 1.00
    Ctrl infr exist
    User-defined sensor
    525
    Init end
    */
    boost::asio::streambuf sbuf;
    boost::asio::read_until(my_port, sbuf, "Init end\r\n");
}

void VC0706::tv_on()
{
    uint8_t args[] = {TVOutStart};
    send_command(CommandTVOutCtrl, args, sizeof(args));
}

void VC0706::tv_off()
{
    uint8_t args[] = {TVOutStop};
    send_command(CommandTVOutCtrl, args, sizeof(args));
}

void VC0706::freeze_picture()
{
    uint8_t args[] = {FBufStopCurrentFrame};
    send_command(CommandFBufCtrl, args, sizeof(args));
}

uint32_t VC0706::get_picture_size()
{
    return get_fbuf_len();
}

void VC0706::step_picture()
{
    uint8_t args[] = {FBufStepFrame};
    send_command(CommandFBufCtrl, args, sizeof(args));
}

void VC0706::resume_picture()
{
    uint8_t args[] = {FBufResumeFrame};
    send_command(CommandFBufCtrl, args, sizeof(args));
}

const VC0706::DataBuffer& VC0706::read_picture()
{
    uint32_t length = get_fbuf_len();
    my_image_buffer.resize(length);
    uint8_t args[] = {CurrentFrame,             //FBUF type
                      0x0A,                     //Control mode
                      0x00, 0x00, 0x00, 0x00,   //Starting address
                      0x00, 0x00, 0x00, 0x00,   //Data-length (filled in later)
                      0x01, 0x00};              //Delay (in multiples of .01ms)
    args[6] = my_response.data[0];
    args[7] = my_response.data[1];
    args[8] = my_response.data[2];
    args[9] = my_response.data[3];

    send_command(CommandReadFBuf, args, sizeof(args));

    read(my_port, buffer(&my_image_buffer[0], length));
    read_response(CommandReadFBuf);
    return my_image_buffer;
}

VC0706::ImageSize VC0706::get_image_size()
{
    uint8_t args[] = {0x04, 0x01, 0x00, 0x19};
    send_command(CommandReadData, args, sizeof(args));
    return (ImageSize)my_response.data[0];
}

void VC0706::set_image_size(ImageSize size)
{
    uint8_t args[] = {0x04, 0x01, 0x00, 0x19, (uint8_t)size};
    send_command(CommandWriteData, args, sizeof(args));
    /*
    NOTE: Seems necessary to do a system restart for this to take effect.
    */
}

uint8_t VC0706::get_compression()
{
    uint8_t args[] = {0x01, 0x01, 0x12, 0x04};
    send_command(CommandReadData, args, sizeof(args));
    return my_response.data[0];
}

void VC0706::set_compression(uint8_t compression)
{
    uint8_t args[] = {0x01, 0x01, 0x12, 0x04, compression};
    send_command(CommandWriteData, args, sizeof(args));
}

void VC0706::motion_detect_on()
{
    motion_ctrl(0, 1, 1);
    uint8_t args[] = {0x01};
    send_command(CommandCommMotionCtrl, args, sizeof(args));
}

void VC0706::motion_detect_off()
{
    motion_ctrl(0, 1, 1);
    uint8_t args[] = {0x00};
    send_command(CommandCommMotionCtrl, args, sizeof(args));
}

bool VC0706::motion_detected()
{
    /* wait up to 1ms */
    if (read_response(0x39, 1, true, false))
        return true;

    return false;
}

void VC0706::send_command(uint8_t command, uint8_t args[], uint8_t arg_count)
{
    my_command.sign = 0x56;
    my_command.serial_number = SerialNumber;
    my_command.command = command;
    my_command.data_length = arg_count;
    for (uint8_t i = 0; i < arg_count; ++i) {
        my_command.data[i] = args[i];
    }
    if (!write_timeout((uint8_t*)&my_command, 4 + arg_count, 10))
        throw std::runtime_error ("Write timeout");

    read_response(command);
}

bool VC0706::read_response(uint8_t command, unsigned int ms, bool timeout_allowed, bool ignore_motion)
{
    while (true) {
        if (!read_timeout((uint8_t *)&my_response, 5, ms)) {
            if (timeout_allowed)
                return false;
            else
                throw std::runtime_error("Read timeout");
        }
        uint8_t motion_detected[] = {0x76, SerialNumber, 0x39, 0x00};
        //We may receive motion detection responses at any time.
        //We ignore them here unless instructed not to.
        if (memcmp(&my_response, motion_detected, 4) == 0 && ignore_motion)
            continue;

        uint8_t expected[] = {0x76, SerialNumber, command, StatusOk};
        if (memcmp(&my_response, expected, 4) != 0) {
            uint8_t *p = (uint8_t *)&my_response;
            for (int i = 0; i < 5; ++i) {
                printf ("%02X ", *p++);
            }
            printf("\n");
            p = expected;
            printf("expected:\n");
            for (int i = 0; i < 5; ++i) {
                printf ("%02X ", *p++);
            }
            printf("\n");
            throw std::runtime_error("Unexpected response");
        }
        break;
    }
    if (my_response.data_length > 20)
        throw std::runtime_error("Data length exceeded");

    if (!read_timeout((uint8_t *)&my_response.data[0], my_response.data_length, ms)) {
        if (timeout_allowed)
            return false;
        else
            throw std::runtime_error("Read timeout");
    }
    return true;
}

void handle_timer(bool *timeout, boost::system::error_code ec) {
    if (ec != boost::asio::error::operation_aborted)
        *timeout = true;
}

void handle_rw(bool *done, boost::system::error_code *e, boost::system::error_code ec) {
    *done = true;
    *e = ec;
}

bool VC0706::read_io(uint8_t *buff, unsigned int length, unsigned int ms, bool read) {
    bool timeout = false;
    bool rw_done = false;
    boost::system::error_code ec;

    boost::asio::deadline_timer deadline(my_io_service);
    deadline.expires_from_now(boost::posix_time::milliseconds(ms));
    deadline.async_wait(boost::bind(&handle_timer, &timeout, _1));

    if (read)
        async_read(my_port, buffer(buff, length), boost::bind(&handle_rw, &rw_done, &ec, _1));
    else
        async_write(my_port, buffer(buff, length), boost::bind(&handle_rw, &rw_done, &ec, _1));

    my_io_service.reset();
    while (my_io_service.run_one()) {
        if (rw_done) {
            deadline.cancel();
        } else if (timeout) {
            my_port.cancel();
        }
    }
    return !timeout && !ec;
}

bool VC0706::read_timeout(uint8_t *buff, unsigned int length, unsigned int ms)
{
    return read_io(buff, length, ms, true);
}

bool VC0706::write_timeout(uint8_t *buff, unsigned int length, unsigned int ms)
{
    return read_io(buff, length, ms, false);
}

uint32_t VC0706::get_fbuf_len()
{
    uint8_t args[] = {CurrentFrame};
    send_command(CommandGetFBufLen, args, sizeof(args));
    uint32_t length = my_response.data[0];
    length <<= 8;
    length |= my_response.data[1];
    length <<= 8;
    length |= my_response.data[2];
    length <<= 8;
    length |= my_response.data[3];
    return length;
}

void VC0706::motion_ctrl(uint8_t attribute, uint8_t interface, uint8_t enable)
{
    uint8_t args[] = {attribute, interface, enable};
    send_command(CommandMotionCtrl, args, sizeof(args));
}

