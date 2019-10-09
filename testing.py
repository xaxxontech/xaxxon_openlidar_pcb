# write input from keyboard over serial in format: [char] [int{0-255}] [int 0-255]

import time, sys, usbdiscover

# ser = serial.Serial('/dev/ttyUSB0', 115200)
# time.sleep(2)

# while ser.inWaiting() > 0:
	# line = ser.readline().strip() # clear buffer
	# print(line)

# ser.write("x\n") # get product
# line = ""
# time.sleep(0.1)
# while ser.inWaiting() > 0:
	# line = ser.readline().strip()
	# print(line)

# if not line == "<id::xaxxonlidar>":
	# print("incorrect board id")
	# sys.exit(0)
	
ser = usbdiscover.usbdiscover("<id::xaxxonopenlidar>")

while 1:
	
	# print any output from device
	while ser.inWaiting() > 0:
		line = ser.readline().strip()
		print(line)

	# wait for keyboard input (blocking)
	s = raw_input('> ').split()
	if len(s) > 0:
		
		# 1st character is single byte command
		cmd = s[0]
		
		# next comes either 0,1 or 2 numbers
		val1 = None
		val2 = None
		if len(s) > 1:
			val1 = s[1]
		if len(s) > 2:
			val2 = s[2]

		ser.write(cmd)
		
		if not val1 == None:
		
			# if 1st number is > 255, send as 2-byte integer using val 1 & val2
			if val1 > 255:
				n = int(val1)
				val1 = n & 0xFF
				val2 = (n >>8) & 0xFF
				ser.write(chr(val1));
				ser.write(chr(val2));
				print "sending: "+str(int((val2 << 8) | (val1 & 0xFF)))
			else:
				ser.write(chr(int(val1)))

				if not val2 == None:
					ser.write(chr(int(val2)))

		ser.write("\n")
	
	time.sleep(0.1)
	

	
	
