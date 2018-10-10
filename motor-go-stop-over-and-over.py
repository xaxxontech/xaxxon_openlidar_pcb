#!/usr/bin/env python

import time, sys, usbdiscover

usbdiscover.removelockfiles()
ser = usbdiscover.usbdiscover("<id::xaxxonlidar>")

while 1:
	time.sleep(5)
	ser.write("g\n")
	time.sleep(5)
	ser.write("p\n")
	
