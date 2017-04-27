# -*- coding: utf-8 -*-
"""
Created on Tue Mar 07 14:23:16 2017
decode: decodes binary logged sensor data from crazyflie2 with uSD-Card-Deck
@author: SJA7SI
"""
from __future__ import print_function
from zlib import crc32
import struct
import numpy as np
import os

# lookup dictionary to determine size of data types
fmtChars = {'c': 1, 'b': 1, 'b': 1, 'B': 1, '?': 1, 'h': 2, 'H': 2,
            'i': 4, 'I': 4, 'l': 4, 'L': 4, 'q': 8, 'Q': 8, 'f': 4, 'd': 8} 

def decode(filName, conCut):
    # read file as binary
    filObj = open(filName, 'rb')
    filCon = filObj.read()
    filObj.close()
    
    # get file size to forecast output array
    statinfo = os.stat(filName)
    
    # process file header
    setWidth = struct.unpack('B', filCon[0])
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
        fmtStr += setName[0]
        setBytes += fmtChars[setName[0]]
    while(offset < len(filCon)):
        setNumber = struct.unpack('B', filCon[offset])
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
    # remove all ticks (and according data) lower/equal conCut
    setCon = setCon[:, (setCon[0]>conCut)]
    
    # create output dictionary
    output = {}
    for ii in range(setWidth[0]):
        output[setNames[ii][1:].strip()] = setCon[ii]
    return output

def createConfig():
    import re
    
    config = ''
    temp = 0
    
    print("Which data should be logged?")
    inStr = raw_input(" * Acceleration ([Y]es / [n]o): ")
    if ((re.search(inStr, '^[Yy]')) or (inStr == '')):
        temp += 1
        
    inStr = raw_input(" * Gyroscope ([Y]es / [n]o): ")
    if ((re.search(inStr, '^[Yy]')) or (inStr == '')):
        temp += 2
        
    inStr = raw_input(" * Barometer ([Y]es / [n]o): ")
    if ((re.search(inStr, '^[Yy]')) or (inStr == '')):
        temp += 4
        
    inStr = raw_input(" * Magnetometer ([Y]es / [n]o): ")
    if ((re.search(inStr, '^[Yy]')) or (inStr == '')):
        temp += 8
        
    inStr = raw_input(" * Stabilizer ([Y]es / [n]o): ")
    if ((re.search(inStr, '^[Yy]')) or (inStr == '')):
        temp += 16
        
    inStr = raw_input(" * Control ([Y]es / [n]o): ")
    if ((re.search(inStr, '^[Yy]')) or (inStr == '')):
        temp += 32
        
    inStr = raw_input(" * Z-Range ([Y]es / [n]o): ")
    if ((re.search(inStr, '^[Yy]')) or (inStr == '')):
        temp += 64
        
    config += chr(temp)
    
    temp = int(raw_input("\nEnter the log frequency [1-1000]: "))
    config += chr(temp // 256)
    config += chr(temp % 256)
    
    config += chr(int(raw_input("Enter buffer size [0-255]: ")))
    
    config += raw_input("Filename: ")
    #config += chr(int(raw_input("Enter write frequency for µSD-Card [0-255]: ")))
    
    # write config to file
    filObj = open(path+"config", 'w')
    filObj.write(config)
    filObj.close()
    
if __name__=='__main__':
    createConfig()