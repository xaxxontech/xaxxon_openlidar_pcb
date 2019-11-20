// photo interrupter GP1A57HRJ00F and garmin lidar lite 3

/*  Commands:

g - motor go
s - motor stop 
p - motor stop facing forward
r,a - set motor rpm (0-255, not 10 or 13)
d,a - set motor direction (1,0) default=1 CW/RHR+ if motor on bottom
1 - lidar enable 
b - lidar broadcast start
n - lidar broadcast stop
a - all on (1, b, g)
f - all off (p, n)
m - lidar read single // TODO: IMPLEMENT
x - get ID
y - get version 
e - enable host heartbeat check (default)
3 - disable host heartbeat check
h - host heartbeat
v - toggle verbose output
k,a,b - set header offset from photo sensor -- 2 byte int units: deci-degrees (1/10 degree)	
i - print header offset ratio
q,a,b - set park position offset from photo sensor -- 2 byte int units: deci-degrees (1/10 degree)	
t,a,b - set read interval -- 2 byte int microseconds 
z - get i2c_error count
 
*/


/* 
 * using non- standard Wire lib 
 * from PR #107 https://github.com/arduino/ArduinoCore-avr/pull/107
 */
#include "src/Wire.h" 

#define LIDARLITE_ADDR_DEFAULT 0x62

// pins
#define PHOTOPIN	2 
const int LIDARENABLEPIN = A3;
#define LIDARMOSFET 6

// A4988 stepper driver pins 
#define STEPPIN		8
#define DIRPIN		4
#define M0			7
#define M1			3
#define SLEEP		9 // low = disable 

// command byte buffer 
int buffer[32];
int commandSize = 0;

// rpm tracking
volatile boolean photoblocked = false;
volatile long timeToPhotoUnblockedCheck = 0;
long photoUnblockedCheckInterval = 0; // approx 1/2 turn minimum should be OK
boolean motorspinning = false;
const long WAITFORSTOPDELAY = 1500000;
int rev = 0; // used by verbose/debug only 
volatile unsigned long lastrev = 0;
// volatile unsigned long delayreadtime = 0;
volatile long lastcycle = 0;
int maxcount = 0;
volatile unsigned long lastLidarRead = 0;
long parkTime = 0;
unsigned long stopTime = 0;
unsigned long safetyStopTime = 0; // in case photo sensor failure, stop motor eventually
float parkRatio = 0.5; // smt PCB, prime accessory frame
float headerOffsetRatio = 0.525; //  photo sensor rotational offset from X (straight forward) smt PCB, prime accessory frame
long revno = 0; // verbose-debug use only 

// motor speed
int rpm = 180; // default
const int MICROSTEPS = 32;
const int STEPSPERREV = 48;
long cycle = 0;
const int PARKINGSPEED = 120; //rpm

// distance output
boolean lidarBroadcast = false;
volatile unsigned int count = 0;
int distancecm = 0;
// volatile boolean delayedOutputScanHeader = false;
volatile unsigned long outputScanHeaderTime = 0; 
unsigned long scanHeaderTimeOffset = 0;
int i2c_errors = 0;

// distance reading 
byte isBusy = 0;
int loopCount;
boolean distanceMeasureStarted = false;
unsigned long lidarReadyTime = 0;
long readInterval = 1400; // read period microseconds
const boolean DOBIASCORRECTION = true; // perform read with biascorrection once per rev
// const int SKIPLAST = 6; // should be lowest possible yet still yeild unchanging count 6 if bias correction
const long HEADERGAP = 8400; // microseconds - lowest possible yet still yeild unchanging count (incl bias correction)
boolean lidarenabled = false;
int timedout = 0;
const int MAXTIMEOUTS = 10;

// host heartbeat
boolean heartbeatenabled = true;
unsigned long lasthostresponse = 0;
const unsigned long HOSTTIMEOUT = 10000000; // 10 sec

volatile unsigned long time = 0;
boolean verbose = false;


void setup() { 

	// photo sensor
	pinMode(PHOTOPIN, INPUT_PULLUP); // interrupt


	// stepper driver
	pinMode(STEPPIN,OUTPUT); 
	pinMode(DIRPIN,OUTPUT);
	pinMode(SLEEP, OUTPUT);

	// set step mode
	pinMode(M0, OUTPUT); 
	digitalWrite(M0, HIGH); 
	
	pinMode(M1,OUTPUT);
	digitalWrite(M1, HIGH);

	digitalWrite(DIRPIN, HIGH); // default direction  dir=1 CW/RHR+ if motor on bottom
	digitalWrite(SLEEP, LOW); // stepper sleeping
	
	// garmin lidar
	pinMode(LIDARMOSFET, OUTPUT);
	pinMode(LIDARENABLEPIN, OUTPUT);
	digitalWrite(LIDARENABLEPIN, LOW);
	digitalWrite(LIDARMOSFET, LOW);

	Serial.begin(115200);
	
	setRPM(rpm); // defines cycle
	
	lasthostresponse = micros();

	Serial.println("<reset>");

}
	
void loop(){
	time = micros();
	
	if (lidarBroadcast) lidarReadDistance();
	
	if (outputScanHeaderTime !=0 && time >= outputScanHeaderTime && lidarBroadcast) {
		outputScanHeaderTime = 0;
		lastcycle = time - lastrev;
		if (verbose)  Serial.println(lastcycle);
		lastrev = time;
		outputScanHeader(time);
	}

	if (photoblocked) {
		if (time >= timeToPhotoUnblockedCheck && motorspinning) {
			if (digitalRead(PHOTOPIN) == HIGH) { photoblocked = false; }
			enableInterrupts();
		}
	}
	
	if (stopTime > 0) {
		if (time >= stopTime) stop();
	}
	
	if (time > safetyStopTime && safetyStopTime != 0)  stop();
	
	// stop motor if ignored by host
	if (heartbeatenabled) { 
		if ((motorspinning || lidarenabled || lidarBroadcast) && time - lasthostresponse > HOSTTIMEOUT) {
			lasthostresponse = time;
			allOff();
			stop();
		}
	}
	
	if( Serial.available() > 0) {
		manageCommand();
	}
	
}

// buffer and/or execute commands from host controller 
void manageCommand() {
	int input = Serial.read();
	buffer[commandSize++] = input;
	
	if((input == 13) || (input == 10) && commandSize > 0 && buffer[0] != 'k' && buffer[0] != 'q' && buffer[0] != 't') {
		commandSize--; // ignore newline
		parseCommand();
		commandSize = 0; 
	} 
	else if ((buffer[0] == 'k' || buffer[0]=='q' ||  buffer[0]=='t') && commandSize == 3) { // handle commands with 2 byte parameters
		parseCommand();
		commandSize = 0; 
	}
	else if ((input == 13 || input == 10) && commandSize == 1)  commandSize = 0; // ignore solitary newlines
	
	lasthostresponse = time;
}

void parseCommand(){
  
	if (buffer[0] == 's') stop(); // stop motor
	else if (buffer[0] == 'g') goMotor(); 
	else if(buffer[0] == 'p') stopMotorFacingForward();
	else if(buffer[0] == 'r') setRPM(buffer[1]);
	else if (buffer[0] == 'd') digitalWrite(DIRPIN, buffer[1]); // 0 or 1 (default 1) 1=CW/RHR+ if motor on bottom
	
	else if (buffer[0] == '1') { lidarEnable(); }
	// else if (buffer[0] == '0') { lidarDisable(); }

	else if(buffer[0] == 'x') boardID();
	else if(buffer[0] == 'y') version();
		
	else if(buffer[0] == 'b') lidarBroadcast = true;
	else if(buffer[0] == 'n') lidarBroadcast = false;
	else if(buffer[0] == 'm') {
		if (!lidarenabled) lidarEnable();
		// TODO: output single reading
	}
	
	else if(buffer[0] == 'h') lasthostresponse = time;
	else if (buffer[0] == 'v') toggleVerbose();
	else if (buffer[0] == 'f') allOff();
	else if (buffer[0] == 'a') allOn();

	else if (buffer[0] == 'e') heartbeatenabled = true;
	else if (buffer[0] == '3') heartbeatenabled = false;
	else if (buffer[0] == 'k') updateHeaderOffset();
	else if (buffer[0] == 'q') updateParkRatio();
	else if (buffer[0] == 'i') { 
		Serial.print("<");
		Serial.print(headerOffsetRatio,3);
		Serial.println(">");
	}
	else if (buffer[0] == 't') updateReadInterval();
	else if (buffer[0] == 'z') i2cerrorreport();
}



void outputDistance() {
											
	unsigned int cm = distancecm; 
	cm +=3; // from front of sensor to ctr of rotation
	uint8_t d[2];
	d[0] = cm & 0xff;
	d[1] = (cm >> 8); 
	
	count ++;
	
	if (!verbose) Serial.write(d, 2);
	// else {
		// Serial.print(cm);
		// Serial.print(" ");
	// }
	
}

void outputScanHeader(unsigned long now) {
	
	uint8_t header[10];
	
	/* send header code */
	header[0]=(0xFF);
	header[1]=(0xFF);
	header[2]=(0xFF);
	header[3]=(0xFF);
	
	/* send count */
	header[4] = count & 0xff;
	header[5] = (count >> 8);
	
	/* send last cycle microseconds */
	header[6] = lastcycle & 0xff;
	header[7] = (lastcycle >> 8);
	header[8] = (lastcycle >> 16);
	header[9] = (lastcycle >> 24);
	
	// /* send offset to last distance */
	// unsigned int lastDistanceOffset = now - lastLidarRead;
	// header[10] = lastDistanceOffset & 0xff;
	// header[11] = (lastDistanceOffset >> 8);

	if (!verbose)   Serial.write(header, 10);
	
	else { 
		Serial.println(lastcycle);
		Serial.print("count: ");
		Serial.println(count);
		Serial.print("cycle: ");
		Serial.println(cycle);
		// Serial.println(testtime);
	}
		
	count = 0;

}
	
void trackRPM() {	
	
	disableInterrupts();

	unsigned long now = micros();

	revno ++;
	if (verbose) {
		Serial.print("rev: ");
		Serial.println(revno);
	}
	
	photoblocked = true;
	timeToPhotoUnblockedCheck = now + photoUnblockedCheckInterval;
	
	outputScanHeaderTime = now + scanHeaderTimeOffset;

	if (stopTime == -1)  
		stopTime = now + parkTime;
	else if (stopTime == -2) stopTime = -1;
	
}

void goMotor() {
	
	motorsEnable();
	unsigned int frequency = frequencyFromRPM(rpm);

	/* Accelerate */
	int acceltime = 2000; // ms
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
	stopTime = 0;
}

void changeSpeed() {
	if (!motorspinning) return;
	
	unsigned int frequency = frequencyFromRPM(rpm);
	tone(STEPPIN, frequency);
}

void stopMotorFacingForward() {
	
	// slow down 
	unsigned int frequency = frequencyFromRPM(PARKINGSPEED);
	tone(STEPPIN, frequency); 

	stopTime = -2;
	
	safetyStopTime = time + 10000000; // ten seconds
}

void motorsEnable() { 	
	digitalWrite(SLEEP, HIGH);
	// delay(100);
}

void stop() {
	noTone(STEPPIN);
	delay(500);
	digitalWrite(SLEEP, LOW); 
	motorspinning = false;
	safetyStopTime = 0;
}

void enableInterrupts() {
	attachInterrupt(digitalPinToInterrupt(PHOTOPIN), trackRPM, LOW);
}

void disableInterrupts() {
	detachInterrupt(digitalPinToInterrupt(PHOTOPIN));
}

void lidarEnable() {
	if (lidarenabled) return;
	
	digitalWrite(LIDARENABLEPIN, HIGH); // must be 1st! (didn't matter before due to mosfet speed?)
	digitalWrite(LIDARMOSFET, HIGH);
	delay(500);
	
	// from https://github.com/garmin/LIDARLite_v3_Arduino_Library/blob/master/examples/ShortRangeHighSpeed/ShortRangeHighSpeed.ino
	Wire.begin();
	Wire.setClock(400000UL);
	Wire.setTimeoutInMicros(1000);
	writelidar(0x02, 0x0d); // Maximum acquisition count of 0x0d. (default is 0x80)
	delay(1);
	writelidar(0x04,0x08); // Default
	delay(1);
	writelidar(0x1c,0x00); // Default

	delay(500); // increased from 200, helps? manual says should only take 22ms...
	if (verbose) Serial.println("lidar enabled");

	lidarenabled = true;
}

void writelidar(int address) {
	writelidar(address, -1);
}

boolean writelidar(int address, int value)
{
	Wire.beginTransmission(LIDARLITE_ADDR_DEFAULT);
	Wire.write(address); // Set register for write
	if (value != -1)  Wire.write(value); // Write myValue to register

	if (Wire.endTransmission() != 0) {
		if (verbose) Serial.println("NACK");
		i2c_errors ++;
		return false;
	}
	
	return true;
	
	// if (value != -1) 
		// delayMicroseconds(50); // testing
}

void lidarReadDistance() {
		
	// start: request lidar data
	if (!distanceMeasureStarted && !isBusy && time >= lidarReadyTime) { 
		
		if (count > maxcount && motorspinning) return; // near complete rev, stop reading 
		
		boolean biascorrection = false;
		if (count == maxcount && DOBIASCORRECTION) biascorrection=true; 
		
		lidarReadyTime = time + readInterval;	
		if (distanceMeasureStart(biascorrection))
			distanceMeasureStarted = true;
	}
	
	// wait
	else if (distanceMeasureStarted && isBusy) {
		distanceWait();
	}
	
	// read lidar data
	else if (distanceMeasureStarted && !isBusy) {			
		distanceGet();  			
		outputDistance();
		distanceMeasureStarted = false;
		
		lastLidarRead = time;
	}
	
}	

boolean distanceMeasureStart(bool biasCorrection) {
	
	boolean started = false;
	
	// Send measurement command
	if(biasCorrection == true)	{    
		if (writelidar(0x00, 0x04))  // Perform measurement with receiver bias correction
			started = true;
	}
	else {
		if (writelidar(0x00, 0x03)) // Perform measurement without receiver bias correction
			started = true;
	}
	
	if (started) {
		isBusy = 1;
		loopCount = 0;
	}
	
	return started;
}

void distanceWait() {
	// Read status register
	// Wire.beginTransmission(LIDARLITE_ADDR_DEFAULT);
	// Wire.write(0x01);
	// if (Wire.endTransmission() != 0) nack();
	
	writelidar(0x01);
	
	Wire.requestFrom(LIDARLITE_ADDR_DEFAULT, 1);
	isBusy = Wire.read(); 
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
	// Wire.beginTransmission(LIDARLITE_ADDR_DEFAULT);
	// Wire.write(0x8f);
	// if (Wire.endTransmission() != 0) nack();
	
	writelidar(0x8f);

	// below tested to take 110 micros for both biasCorrection true and false
	// Perform the read and repack the 2 bytes into 16-bit word
	Wire.requestFrom(LIDARLITE_ADDR_DEFAULT, 2);
	distancecm = Wire.read(); 
	distancecm <<= 8;
	distancecm |= Wire.read(); 
	
}


void allOn() {
	lidarEnable();
	lidarBroadcast = true;
	goMotor();
}

void allOff() {
	stopMotorFacingForward();
	lidarBroadcast = false;
	// lidarDisable();
	// timedout = 0;
}

unsigned int frequencyFromRPM(int rpm) {
	unsigned int freq = ((float) rpm/60.0) * MICROSTEPS * STEPSPERREV;
	return freq;
}

void setRPM(int n) {
	rpm = n;
	cycle = (60.0/(float) rpm) * 1000000;
	// maxcount = cycle/LIDARDELAY - (cycle/LIDARDELAY * 0.022); // constant is fraction of full rev: was 0.014 -- should be highest that yeilds unchanging count
	// maxcount = cycle/LIDARDELAY - SKIPLAST;
	maxcount = (cycle-HEADERGAP)/readInterval;
	photoUnblockedCheckInterval = cycle/2;
	parkTime = (float)rpm/PARKINGSPEED * cycle * parkRatio;
	scanHeaderTimeOffset = cycle * headerOffsetRatio;
	
	if (motorspinning) changeSpeed();
	// Serial.println(maxcount);
}

void updateHeaderOffset() {
	int a = buffer[1];
	int b = buffer[2]; 
	int c = 0;
    c = (c << 8) + b;
    c = (c << 8) + a;
    float d = c/10.0;
    headerOffsetRatio = d/360.0; 
    setRPM(rpm);
}

void updateParkRatio() {
	int a = buffer[1];
	int b = buffer[2]; 
	int c = 0;
    c = (c << 8) + b;
    c = (c << 8) + a;
    float d = c/10.0;
    parkRatio = d/360.0; 
    setRPM(rpm);
}

void updateReadInterval() {
	int a = buffer[1];
	int b = buffer[2]; 
	readInterval = 0;
    readInterval = (readInterval << 8) + b;
    readInterval = (readInterval << 8) + a;
    setRPM(rpm);
}

void toggleVerbose() {
	if (verbose) {
		Serial.println("verbose off");
		verbose = false;
	}
	else {
		Serial.println("verbose on");
		verbose = true;
		heartbeatenabled = false;
	}
}

void i2cerrorreport() {
	Serial.print("<i2c_errors: ");
	Serial.print(i2c_errors);
	Serial.println(">");
}

void boardID() {
	Serial.println("<id::xaxxonopenlidar>");
}

void version() {
	Serial.println("<version:1.185>"); 
}
