# -*- coding: utf-8 -*-
"""
decode: decodes binary logged sensor data from crazyflie2 with uSD-Card-Deck
createConfig: create config file which has to placed on µSD-Card
@author: jsschell
"""
from zlib import crc32
import struct
import numpy as np
import os

# lookup dictionary to determine size of data types
fmtChars = {'c': 1, 'b': 1, 'b': 1, 'B': 1, '?': 1, 'h': 2, 'H': 2,
            'i': 4, 'I': 4, 'l': 4, 'L': 4, 'q': 8, 'Q': 8, 'f': 4, 'd': 8} 

def decode(filName):
    # read file as binary
    filObj = open(filName, 'rb')
    filCon = filObj.read()
    filObj.close()
    
    # get file size to forecast output array
    statinfo = os.stat(filName)
    
    # process file header
    setWidth = struct.unpack('B', filCon[:1])
    setNames = []
    for ii in range(0, setWidth[0]):
        setNames.append(filCon[ii*5+1:ii*5+6])
    print("[CRC] of file header:", end="")
    crcVal = crc32(filCon[0:setWidth[0]*5+1+4]) & 0xffffffff
    crcErrors = 0
    if ( crcVal == 0xffffffff):
        print("\tOK\t["+hex(crcVal)+"]")
    else:
        print("\tERROR\t["+hex(crcVal)+"]")
        crcErrors += 1
    offset = setWidth[0]*5+5
    
    # process data sets
    setCon = np.zeros(statinfo.st_size // 4)
    idx = 0
    fmtStr = ""
    setBytes = 0
    for setName in setNames:
        fmtStr += chr(setName[0])
        setBytes += fmtChars[chr(setName[0])]
    while(offset < len(filCon)):
        setNumber = struct.unpack('B', filCon[offset:offset+1])
        offset += 1
        for ii in range(setNumber[0]):
            setCon[idx:idx+setWidth[0]] = np.array(struct.unpack(fmtStr, filCon[offset:setBytes+offset]))
            offset += setBytes
            idx += setWidth[0]
        crcVal = crc32(filCon[offset-setBytes*setNumber[0]-1:offset+4]) & 0xffffffff
        print("[CRC] of data set:", end="")
        if ( crcVal == 0xffffffff):
            print("\tOK\t["+hex(crcVal)+"]")
        else:
            print("\tERROR\t["+hex(crcVal)+"]")
            crcErrors += 1
        offset += 4
    if (not crcErrors):
        print("[CRC] no errors occurred:\tOK")
    else:
        print("[CRC] {0} errors occurred:\tERROR".format(crcErrors))
    
    # remove not required elements and reshape as matrix
    setCon = np.reshape(setCon[0:idx], (setWidth[0], idx//setWidth[0]), 'f')
    
    # create output dictionary
    output = {}
    for ii in range(setWidth[0]):
        output[setNames[ii][1:].decode("utf-8").strip()] = setCon[ii]
    return output

def createConfig():
    import re

    temp = 0
    
    print("Which data should be logged?")
    inStr = input(" * Acceleration ([Y]es / [n]o): ")
    if ((re.search('^[Yy]', inStr)) or (inStr == '')):
        temp += 1
        
    inStr = input(" * Gyroscope ([Y]es / [n]o): ")
    if ((re.search('^[Yy]', inStr)) or (inStr == '')):
        temp += 2
        
    inStr = input(" * Barometer ([Y]es / [n]o): ")
    if ((re.search('^[Yy]', inStr)) or (inStr == '')):
        temp += 4
        
    inStr = input(" * Magnetometer ([Y]es / [n]o): ")
    if ((re.search('^[Yy]', inStr)) or (inStr == '')):
        temp += 8
        
    inStr = input(" * Stabilizer ([Y]es / [n]o): ")
    if ((re.search('^[Yy]', inStr)) or (inStr == '')):
        temp += 16
        
    inStr = input(" * Control ([Y]es / [n]o): ")
    if ((re.search('^[Yy]', inStr)) or (inStr == '')):
        temp += 32
        
    inStr = input(" * Z-Range ([Y]es / [n]o): ")
    if ((re.search('^[Yy]', inStr)) or (inStr == '')):
        temp += 64
        
    config = temp.to_bytes(1, byteorder='big', signed=False)
    
    config += int(input("\nEnter the log frequency [1-1000]: ")).to_bytes(2, byteorder='big', signed=False)
    
    config += int(input("Enter buffer size [0-255]: ")).to_bytes(1, byteorder='big', signed=False)
    
    config += bytes(input("Filename [max. 10 letters]: ").encode('ascii'))
    
    # write config to file
    filObj = open("config", 'wb')
    filObj.write(config)
    filObj.close()
    
if __name__=='__main__':
    createConfig()