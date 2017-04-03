#!/usr/bin/env python
import os, time
import serial
from ctypes import c_long, c_ulong, c_ushort, c_short


def set_tmode2_fixed(lat,lon,alt,acc):
    #from: u-blox 8 / u-blox M8 Receiver Description Including Protocol Specification p 144
    #accuracy in mm!
    syncchars = r'\xB5\x62'
    clss = r'\x06'# CFG
    ID = r'\x3D' # TMODE2    
    length = int_to_byte_stream(28,0)
    mode = r'\x02' #fixed mode
    reserved = r'\x00'
    flags = int_to_byte_stream(1,0) #lat/lon/alt;alt valid
    #lon/lat accuracy: 7 decimal places
    lat = long_to_byte_stream(int(lat*1e7), 1)
    lon = long_to_byte_stream(int(lon*1e7), 1)
    # altitude in cm
    alt = long_to_byte_stream(int(alt*1e2), 1)
    #accuracy in mm
    acc = long_to_byte_stream(int(acc*1000), 0)
    smindur = long_to_byte_stream(0, 0)
    sminacc = long_to_byte_stream(0, 0)
    stream_center = clss + ID + length +  mode + reserved + flags + lat + lon + alt + acc + smindur + sminacc
    # calculate Fletcher16 checksum
    CK_A = 0
    CK_B = 0
    for byte in stream_center.split(r'\x')[1:]: 
         CK_A = ((CK_A + int(byte,16))%256)
         CK_B = ((CK_B + CK_A)%256)
    #CK_0 = 255-((CK_B+CK_A)%255)
    #CK_1 = 255-((CK_0+CK_A)%255)
    bytestream =  syncchars + stream_center + r'\x%002x' % (c_ushort(CK_A).value) + r'\x%002x' % (c_ushort(CK_B).value)
    return bytestream

    
def long_to_byte_stream(integer,signed):
    if signed:
        hexstring = '0x%008x' % (c_long(integer).value)
    else:
        hexstring = '0x%008x' % (c_ulong(integer).value)
    #lsb to msb:
    hexstring = hexstring[2:]
    hexlist = [hexstring[i:i+2] for i in range(len(hexstring)-2,-2, -2)]
    #Add empty string to list to set prefix in front of first byte!
    bytestream = r'\x'.join(['']+hexlist)
    return bytestream
    
def int_to_byte_stream(integer,signed):
    if signed:
        hexstring = '0x%004x' % (c_short(integer).value)
    else:
        hexstring = '0x%004x' % (c_ushort(integer).value)
    #cut prefix
    hexstring = hexstring[2:]
    #lsb to msb:
    hexlist = [hexstring[i:i+2] for i in range(len(hexstring)-2,-2, -2)]
    #Add empty string to list to append prefix to first byte!
    bytestream = r'\x'.join(['']+hexlist)
    return bytestream
 
def set_ublox_coordinates_fixed(lat, lon, alt, accuracy):
    # check if m8f is connected! (in receiver) 
    bytestream = set_tmode2_fixed(lat,lon,alt,accuracy)
    bytestream.replace('\\x',' 0x')
    try:
        print "Send serial command through USB"
        ser = serial.Serial('/dev/ttyACM0',9600)
        ser.write(bytestream.decode('string_escape'))
        print "Wait.."
        time.sleep(10)
        print 'Done'
        # close connection and destroy serial instance
        ser.close()
        ser.__exit__
    except:
        print "Configuring u-blox LEA-M8F not possible. Check connection."