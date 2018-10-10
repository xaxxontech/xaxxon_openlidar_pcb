# oculusprimesocket.py module
# make tcp socket connection with Oculus Prime robot, relay commands and messages 

"""make tcp socket connection with Oculus Prime Server Application
provide functions for relay of commands and messages"""

import serial, os, time

def removelockfiles():
	os.system("rm -f /tmp/dev_ttyUSB*")

def checkBoardId(idstring, ser):
	ser.reset_input_buffer()
	ser.write("x\n") # check board id
	line = ""
	time.sleep(0.1)
	while ser.inWaiting() > 0:
		line = ser.readline().strip()

	if not line == idstring:
		print("incorrect board id: "+line)
		return False
	
	print("connected to: "+line)
	return True
	
	
def usbdiscover(idstring):
	global lockfilepath
	
	portnum = 0;

	while portnum <= 6:
		port = '/dev/ttyUSB'+str(portnum)

		print("trying port: "+port)
		
		lockfilepath = "/tmp/dev_ttyUSB"+str(portnum)
		tries = 0
		while tries < 5:
			if os.path.exists(lockfilepath):
				print(" port busy: "+port)
				time.sleep(1)
				tries += 1
			else:
				break 

		if tries == 5:
			print("giving up on port: "+port)
			portnum += 1
			continue
			
		open(lockfilepath, 'w') # creates lockfile
		
		try:
			ser = serial.Serial(port, 115200, timeout=5)
		except serial.SerialException: 
			print("port exception: "+port)
			os.remove(lockfilepath)
			portnum += 1
			time.sleep(1)
			continue
			
		time.sleep(2)

		if checkBoardId(idstring, ser):
			break
			
		ser.close()
		if os.path.exists(lockfilepath):
			os.remove(lockfilepath)
		time.sleep(1)
		portnum += 1
		
	if not ser.is_open:
		print("device not found")
		sys.exit(0)
	
	return ser
	ser.write("x\n") # check board id
	line = ""
	time.sleep(0.1)
	while ser.inWaiting() > 0:
		line = ser.readline().strip()

	if not line == idstring:
		print("incorrect board id: "+line)
		return False
	
	print("connected to: "+line)
	return True
	
	
def usbdiscover(idstring):
	global lockfilepath
	
	portnum = 0;

	while portnum <= 6:
		port = '/dev/ttyUSB'+str(portnum)

		print("trying port: "+port)
		
		lockfilepath = "/tmp/dev_ttyUSB"+str(portnum)
		tries = 0
		while tries < 5:
			if os.path.exists(lockfilepath):
				print("port busy: "+port)
				time.sleep(1)
				tries += 1
			else:
				break 

		if tries == 5:
			print("giving up on port: "+port)
			portnum += 1
			continue
			
		open(lockfilepath, 'w') # creates lockfile
		
		try:
			ser = serial.Serial(port, 115200, timeout=5)
		except serial.SerialException: 
			print("port exception: "+port)
			os.remove(lockfilepath)
			portnum += 1
			time.sleep(1)
			continue
			
		time.sleep(2)

		if checkBoardId(idstring, ser):
			break
			
		ser.close()
		if os.path.exists(lockfilepath):
			os.remove(lockfilepath)
		time.sleep(1)
		portnum += 1
		
	if not ser.is_open:
		print("device not found")
		sys.exit(0)
	
	return ser
