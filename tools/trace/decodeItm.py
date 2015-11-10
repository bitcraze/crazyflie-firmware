#!/usr/bin/env python3
# Decodes ITM bit flow and print it on the console
import sys
import struct

class EOFException(Exception):
    pass

def read_u8(file):
    data = file.read(1)
    if len(data) < 1:
        raise EOFException()
    return struct.unpack("<B", data)[0]


def read_u16(file):
    data = file.read(2)
    if len(data) < 2:
        raise EOFException()
    return struct.unpack("<H", data)[0]


def read_u32(file):
    data = file.read(4)
    if len(data) < 4:
        raise EOFException()
    return struct.unpack("<L", data)[0]

if len(sys.argv) < 2:
    print("Usage: {} <raw trace>".format(sys.argv[0]))
    sys.exit(1)

trace = open(sys.argv[1], "rb")

# OS messages
OS_MASK = 0xFF00
OS_MESSAGES = {
    0x0100: "ITM_QUEUE_SEND",
    0x0200: "QUEUE_FAILED",
    0x0300: "BLOCKING_ON_QUEUE_RECEIVE",
    0x0400: "BLOCKING_ON_QUEUE_SEND"
}

# Decoding...
ctn = 0
try:
    while True:
        b = read_u8(trace)
        # sys.stdout.write("0x{:04x} ".format(ctn))
        ctn += 1

        if b == 0 or b == 0x80:
            pass
        elif b == 0b01110000:
            print("OVF")
        elif (b & 0x0f) == 0:
            print ("LTS")
            while (b & 0x7F) != 0:
                b = read_u8(trace)
                ctn += 1
        elif (b & 0b00001011) == 0b00001000:
            print("EXT")
            while (b & 0x7F) != 0:
                b = read_u8(trace)
                ctn += 1
        elif (b & 0b11011111) == 0b10010100:
            print("GTS")
            while (b & 0x7F) != 0:
                b = read_u8(trace)
                ctn += 1
        elif (b & 0x03) != 0:    # Source packet
            s = b & 0x03
            a = b >> 3
            data = 0
            data_str = ""
            info = ""
            if s == 1:
                data = read_u8(trace)
                data_str = "0x{:02x}".format(data)
                ctn += 1
            elif s == 2:
                data = read_u16(trace)
                data_str = "0x{:02x}".format(data)
                ctn += 2
            elif s == 3:
                data = read_u32(trace)
                data_str = "0x{:02x}".format(data)
                ctn += 4
            if (b & 0x04) == 0:  # ITM/SW
                info = ""
                if a == 1:
                    info = "\t# Task: " + struct.pack("<L", data).decode('utf8')
                if a == 2:
                    info = "\t# Systick: 0x{:04x}".format(data)
                if a == 3:
                    info = "\t\t# OS {} {}".format(OS_MESSAGES[data&OS_MASK], data & (~OS_MASK))
                print("ITM {} {} {}".format(a, data_str, info))
            else:                # DWT/HW
                if a == 1:
                    info = "\t# "
                    if data_str[2] == "1":
                        info += "Enter "
                    elif data_str[2] == "2":
                        info += "Exit "
                    else:
                        info += "returned to "
                    info += "IRQ 0x" + data_str[4:]
                print("DWT {} {} {}".format(a, data_str, info))

        else:
            pass
except EOFException:
    pass
