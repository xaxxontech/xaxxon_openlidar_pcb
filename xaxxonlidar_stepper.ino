// photo interrupter GP1A57HRJ00F and garmin lidar lite 3 test
// using MALG

/*  Commands:

g - motor go
s - motor stop 
p - motor stop facing forward
r,a - set motor rpm (0-255, not 10 or 13)
d,a - set motor direction (1,0)
1 - lidar enable pin HIGH
0 - lidar enable pin LOW
b - lidar broadcast start
n - lidar broadcast stop
x - get ID
y - get version 
e - enable host hearbeat check
h - host heartbeat
v - toggle verbose
a - all on (1, b, g)
f - all off (p, n, 0)
 
*/

#include <Wire.h>
#include <LIDARLite.h>

LIDARLite myLidarLite;

// input pins
const int photoPin  = 2; 
const int lidarEnablePin = A3;

// A4988 stepper driver pins 
const int STEPPIN = 8; 
const int DIRPIN = 4; 
const int M0 = 7; 
const int M1 = 3; 
const int SLEEP = 9; // low = disable 

// command byte buffer 
int buffer[32];
int commandSize = 0;

// rpm tracking
volatile boolean photoblocked = false;
volatile long timeToPhotoUnblockedCheck = 0;
const long photoUnblockedCheckInterval = 150000; // approx 1/2 turn minimum should be OK
long waitingForStop = 0;
boolean motorspinning = false;
const long WAITFORSTOPDELAY = 1500000;
int rev = 0; // used by verbose/debug only 

// motor speed
int rpm = 180; // default
const int MICROSTEPS = 32;
const int STEPSPERREV = 48;

// distance output
boolean lidarBroadcast = false;
volatile unsigned int count = 0;
int distancecm = 0;
volatile boolean delayedOutputScanHeader = false;

// distance reading 
byte isBusy = 0;
int loopCount;
boolean distanceMeasureStarted = false;
// timed distance readi OUTPUT); 
	// digitalWrite(ng
unsigned long lidarReadyTime = 0;
const long LIDARDELAY = 1400; // read frequency microseconds
const int BIASCORRECTIONCOUNT = 200;
boolean lidarenabled = true;

// host heartbeat
const boolean HEARTBEATENABLED = true;
unsigned long lasthostresponse = 0;
const unsigned long HOSTTIMEOUT = 10000000; // 10 sec
int timedout = 0;
const int MAXTIMEOUTS = 10;

volatile unsigned long time = 0;
boolean verbose = false;


void setup() { 

	pinMode(STEPPIN,OUTPUT); 
	pinMode(DIRPIN,OUTPUT);
	pinMode(SLEEP, OUTPUT);

	// pinMode(lidarEnablePin, OUTPUT);
	pinMode(photoPin, INPUT_PULLUP); // interrupt

	
	// set step mode
	pinMode(M0, OUTPUT); 
	digitalWrite(M0, HIGH); 
	
	pinMode(M1,OUTPUT);
	digitalWrite(M1, HIGH);

	digitalWrite(DIRPIN, HIGH); // default direction 
	digitalWrite(SLEEP, LOW); // stepper sleeping

	// from https://github.com/garmin/LIDARLite_v3_Arduino_Library/blob/master/examples/ShortRangeHighSpeed/ShortRangeHighSpeed.ino
	myLidarLite.begin(0, true); // Set configuration to default and I2C to 400 kHz
	myLidarLite.write(0x02, 0x0d); // Maximum acquisition count of 0x0d. (default is 0x80)
	// myLidarLite.write(0x04, 0b00000100); // Use non-default reference acquisition count
	// myLidarLite.write(0x12, 0x03); // Reference acquisition count of 3 (default is 5)

	Serial.begin(115200);
	Serial.println("<reset>");
	
	lasthostresponse = micros();
}
	
void loop(){
	
	time = micros();
	
	if (lidarBroadcast) { 
		if (!distanceMeasureStarted && !isBusy && time >= lidarReadyTime) { 
			lidarReadyTime = time + LIDARDELAY;	
			if (count==BIASCORRECTIONCOUNT)  distanceMeasureStart(true); // recommended periodically
			else distanceMeasureStart(false);
			distanceMeasureStarted = true;
		}
		else if (distanceMeasureStarted && isBusy) {
			distanceWait();
		}
		else if (distanceMeasureStarted && !isBusy) {			
			distanceGet();  			
			outputDistance();
			distanceMeasureStarted = false;
			if (delayedOutputScanHeader) {
				outputScanHeader();
				delayedOutputScanHeader = false;
			}
		}
	}

	if (photoblocked) {
		if (time >= timeToPhotoUnblockedCheck && motorspinning) {
			if (digitalRead(photoPin) == HIGH) { photoblocked = false; }
			enableInterrupts();
		}
	
		if (waitingForStop > 0 && waitingForStop <= time)  {
			waitingForStop = 0;
			hardStop();
		}
	}
	
	// stop motor if ignored by host
	if (HEARTBEATENABLED) { // TODO: buggy?
		if ((motorspinning || lidarenabled || lidarBroadcast) && time - lasthostresponse > HOSTTIMEOUT) {
			lasthostresponse = time;
			allOff();
		}
	}
	
	if( Serial.available() > 0) {
		manageCommand();
	}
	
}


// buffer and/or execute commands from host controller 
void manageCommand() {
	int input = Serial.read();
	if((input == 13) || (input == 10)){
		if(commandSize > 0){
			  parseCommand();
			  commandSize = 0; 
		}
	} 
	else {
		buffer[commandSize++] = input;
	}
	
	lasthostresponse = time;
}

void parseCommand(){
  
	if (buffer[0] == 's'){ // stop motor
		noTone(STEPPIN);
		digitalWrite(SLEEP, LOW); 
		motorspinning = false;
	}
	else if (buffer[0] == 'g') goMotor(); 
	
	else if(buffer[0] == 'p') stopMotorFacingForward();
	else if(buffer[0] == 'r') rpm = buffer[1];
	else if (buffer[0] == 'd') digitalWrite(DIRPIN, buffer[1]); // 0 or 1
	
	else if (buffer[0] == '1') { lidarEnable(); }
	else if (buffer[0] == '0') { lidarDisable(); }

	else if(buffer[0] == 'x') Serial.println("<id::xaxxonlidar>");
	else if(buffer[0] == 'y') version();
		
	else if(buffer[0] == 'b') lidarBroadcast = true;
	else if(buffer[0] == 'n') lidarBroadcast = false;
	
	
	else if(buffer[0] == 'h') lasthostresponse = time;
	else if (buffer[0] == 'v') toggleVerbose();
	else if (buffer[0] == 'f') allOff();
	else if (buffer[0] == 'a') allOn();

}

void outputDistance() {
											
	unsigned int cm = distancecm; 
	cm +=3; // from front of sensor to ctr of rotation
	uint8_t d[2];
	d[0] = cm & 0xff;
	d[1] = (cm >> 8); 
	
	count ++;
	
	if (!verbose) Serial.write(d, 2);
	else {
	// Serial.print(" ");
	// Serial.print(cmlow);
	// Serial.print(" ");
	Serial.println(cm);
	// Serial.print("distance: ");
	// Serial.println(distancecm);
	}
	
}

void outputScanHeader() {
	
	if (distanceMeasureStarted && !isBusy) { // in the middle of data point ouput
		delayedOutputScanHeader = true;
		return;
	}
	
	uint8_t header[6];
	
	/* send header code */
	header[0]=(0xFF);
	header[1]=(0xFF);
	header[2]=(0xFF);
	header[3]=(0xFF);
	
	/* send count */
	header[4] = count & 0xff;
	header[5] = (count >> 8);

	if (!verbose) Serial.write(header, 6);
	else Serial.println(count);
	
	// Serial.print("count: ");
	// Serial.println(count);
	// Serial.print("cycle: ");
	// Serial.println(cycle);
	// Serial.println(testtime);
		
	count = 0;

}
	
void trackRPM() {	
	
	disableInterrupts();

	// if (photoblocked) return; // just in case	
		
	photoblocked = true;
	if (lidarBroadcast) outputScanHeader();
	timeToPhotoUnblockedCheck = time + photoUnblockedCheckInterval;
	
	if (verbose) {
		Serial.print("rev: ");
		Serial.println(rev);
		rev ++;
	}
}

void goMotor() {
	motorsEnable();
	unsigned int frequency = frequencyFromRPM(rpm);
	
	// int f = frequency*0.5;
	// tone(STEPPIN, f);
	// delay(250);

	// f = frequency*0.75;
	// tone(STEPPIN, f);
	// delay(250);
	
	int acceltime = 1000; // ms
	int steps = 20;
	int startfreq = frequencyFromRPM(30);
	int inc = (frequency - startfreq)/steps;
	for (int f=startfreq; f<frequency; f+=inc) {
		tone (STEPPIN, f);
		delay(acceltime/steps);
	}

	tone(STEPPIN, frequency);
	enableInterrupts();
	motorspinning = true;
	waitingForStop = 0;
}

void stopMotorFacingForward() {
	unsigned int frequency = frequencyFromRPM(60);
	tone(STEPPIN, frequency); 
	waitingForStop = time + WAITFORSTOPDELAY;
}

void motorsEnable() { 	
	digitalWrite(SLEEP, HIGH);
	// delay(100);
}

void hardStop() {
	noTone(STEPPIN);
	delay(500);
	digitalWrite(SLEEP, LOW); 
	motorspinning = false;
}

void lidarEnable() {
	digitalWrite(lidarEnablePin, HIGH);
	// delay(100);
	lidarenabled = true;
}

void lidarDisable() {
	digitalWrite(lidarEnablePin, LOW);
	lidarenabled = false;
}

void enableInterrupts() {
	attachInterrupt(digitalPinToInterrupt(photoPin), trackRPM, LOW);
}

void disableInterrupts() {
	detachInterrupt(digitalPinToInterrupt(photoPin));
}

void distanceMeasureStart(bool biasCorrection) {
	
	// Send measurement command
	Wire.beginTransmission(LIDARLITE_ADDR_DEFAULT);
	Wire.write(0X00); // Prepare write to register 0x00
	if(biasCorrection == true)	{
		Wire.write(0X04); // Perform measurement with receiver bias correction
	}
	else {
		Wire.write(0X03); // Perform measurement without receiver bias correction
	}
	Wire.endTransmission();
	
	isBusy = 1;
	loopCount = 0;
}

void distanceWait() {
	// Read status register
	Wire.beginTransmission(LIDARLITE_ADDR_DEFAULT);
	Wire.write(0x01);
	Wire.endTransmission();
	Wire.requestFrom(LIDARLITE_ADDR_DEFAULT, 1);
	isBusy = readIfAvailable();
	isBusy = bitRead(isBusy,0); // Take LSB of status register, busy bit

	loopCount++; // Increment loop counter
	// Stop status register polling if stuck in loop
	if(loopCount > 9999) {
		Serial.println("loopCount > 9999");
		isBusy = 0;
	}
}

int distanceGet() {

	// Immediately read previous distance measurement data. This is valid until the next measurement finishes.
	// The I2C transaction finishes before new distance measurement data is acquired.
	// Prepare 2 byte read from registers 0x0f and 0x10
	Wire.beginTransmission(LIDARLITE_ADDR_DEFAULT);
	Wire.write(0x8f);
	Wire.endTransmission();

	// below tested to take 110 micros for both biasCorrection true and false
	// Perform the read and repack the 2 bytes into 16-bit word
	Wire.requestFrom(LIDARLITE_ADDR_DEFAULT, 2);
	distancecm = readIfAvailable();
	distancecm <<= 8;
	distancecm |= readIfAvailable();
	
}

// TODO: unused, may cause noisier scans/affects gmapping
byte readIfAvailable() {
	byte b = Wire.read(); 
	return b;
	
	unsigned long m = 0;
	while (m < 1000000) { // 1 second timeout
		if (Wire.available()) {
			byte b = Wire.read();
			return b;
		}
		delayMicroseconds(1);
		m ++;
	}
	// timed out
	
	// if (timedout < MAXTIMEOUTS) {
		// lidarBroadcast = false;
		// lidarDisable();
		// delay(100);
		// lidarEnable();
		// lidarBroadcast = true;
		// timedout ++;
	// }
	// else 
	
	Serial.println("sensor read timed out");
	allOff();	
	return 0;
}

void allOn() {
	lidarEnable();
	lidarBroadcast = true;
	goMotor();
}

void allOff() {
	stopMotorFacingForward();
	lidarBroadcast = false;
	lidarDisable();
	timedout = 0;
}

unsigned int frequencyFromRPM(int rpm) {
	unsigned int freq = ((float) rpm/60.0) * MICROSTEPS * STEPSPERREV;
	return freq;
}

void toggleVerbose() {
	if (verbose) {
		Serial.println("verbose off");
		verbose = false;
	}
	else {
		Serial.println("verbose on");
		verbose = true;
	}
}

void version() {
	Serial.println("<version:0.897>"); 
}
