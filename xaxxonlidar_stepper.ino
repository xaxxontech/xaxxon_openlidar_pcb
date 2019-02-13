// photo interrupter GP1A57HRJ00F and garmin lidar lite 3 test
// using MALG

/*  Commands:

g - motor go
s - motor stop 
p - motor stop facing forward
r,a - set motor rpm (0-255, not 10 or 13)
d,a - set motor direction (1,0)
1 - lidar enable 
0 - lidar disable
b - lidar broadcast start
n - lidar broadcast stop
m - lidar read single
x - get ID
y - get version 
e - enable host heartbeat check
3 - disable host heartbeat check
h - host heartbeat
v - toggle verbose
a - all on (1, b, g)
f - all off (p, n, 0)
 
*/

#include <Wire.h>
#include <LIDARLite.h>

LIDARLite myLidarLite;

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
// const float PARKRATIO = 0.75; // PROTO 2
// const float HEADEROFFSETRATIO = 0.783; //  photo sensor rotational offset from X (straight forward) PROTO 2
const float PARKRATIO = 0.5; // smt PCB, prime accessory frame
const float HEADEROFFSETRATIO = 0.52; //  photo sensor rotational offset from X (straight forward) smt PCB, prime accessory frame

// motor speed
int rpm = 180; // default
const int MICROSTEPS = 32;
const int STEPSPERREV = 48;
long cycle = 0;

// distance output
boolean lidarBroadcast = false;
volatile unsigned int count = 0;
int distancecm = 0;
// volatile boolean delayedOutputScanHeader = false;
volatile unsigned long outputScanHeaderTime = 0; // was 'volatile' but caused occasional scanheader output at interrupt
unsigned long scanHeaderTimeOffset = 0;

// distance reading 
byte isBusy = 0;
int loopCount;
boolean distanceMeasureStarted = false;
// timed distance readi OUTPUT); 

unsigned long lidarReadyTime = 0;
const long LIDARDELAY = 1400; // read period microseconds
// const int SKIPLAST = 5;
// const int BIASCORRECTIONCOUNT = 200;
boolean lidarenabled = true;

// host heartbeat
boolean heartbeatenabled = true;
unsigned long lasthostresponse = 0;
const unsigned long HOSTTIMEOUT = 10000000; // 10 sec
// int timedout = 0;
// const int MAXTIMEOUTS = 10;

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

	digitalWrite(DIRPIN, HIGH); // default direction 
	digitalWrite(SLEEP, LOW); // stepper sleeping
	
	// garmin lidar
	pinMode(LIDARMOSFET, OUTPUT);
	pinMode(LIDARENABLEPIN, OUTPUT);
	lidarEnable();
	
	// from https://github.com/garmin/LIDARLite_v3_Arduino_Library/blob/master/examples/ShortRangeHighSpeed/ShortRangeHighSpeed.ino
	myLidarLite.begin(0, true); // Set configuration to default and I2C to 400 kHz
	myLidarLite.write(0x02, 0x0d); // Maximum acquisition count of 0x0d. (default is 0x80)
	// myLidarLite.write(0x04, 0b00000100); // Use non-default reference acquisition count
	// myLidarLite.write(0x12, 0x03); // Reference acquisition count of 3 (default is 5)
	// delay(100);
	// lidarDisable();


	Serial.begin(115200);

	Serial.println("<reset>");
	
	lasthostresponse = micros();
	
	setRPM(rpm); // defines cycle
	
}
	
void loop(){
	time = micros();
	
	if (lidarBroadcast) readLidar();
	
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
		if (time >= stopTime) hardStop();
	}
	
	// stop motor if ignored by host
	if (heartbeatenabled) { // TODO: buggy?
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
	else if(buffer[0] == 'r') setRPM(buffer[1]);
	else if (buffer[0] == 'd') digitalWrite(DIRPIN, buffer[1]); // 0 or 1
	
	else if (buffer[0] == '1') { lidarEnable(); }
	else if (buffer[0] == '0') { lidarDisable(); }

	else if(buffer[0] == 'x') Serial.println("<id::xaxxonlidar>");
	else if(buffer[0] == 'y') version();
		
	else if(buffer[0] == 'b') lidarBroadcast = true;
	else if(buffer[0] == 'n') lidarBroadcast = false;
	else if(buffer[0] == 'm') Serial.println(myLidarLite.distance());
	
	
	else if(buffer[0] == 'h') lasthostresponse = time;
	else if (buffer[0] == 'v') toggleVerbose();
	else if (buffer[0] == 'f') allOff();
	else if (buffer[0] == 'a') allOn();

	else if (buffer[0] == 'e') heartbeatenabled = true;
	else if (buffer[0] == '3') heartbeatenabled = false;
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
	// Serial.println(cm);
	}
	
}

void outputScanHeader(unsigned long now) {
	
	/* avoids output/read +/-1 count mismatch */
	// if (distanceMeasureStarted && !isBusy) { // in the middle of data point ouput
		// delayedOutputScanHeader = true;
		// return;
	// }
	
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

	// if (photoblocked) return; // just in case

	unsigned long now = micros();

	// lastcycle = now - lastrev;
	// if (verbose)  Serial.println(lastcycle);
	// lastrev = now;
	
	photoblocked = true;
	timeToPhotoUnblockedCheck = now + photoUnblockedCheckInterval;
	
	// if (lidarBroadcast)  outputScanHeader(now);
	// if (outputScanHeaderTime != 0) { // TODO: TESTING
		// Serial.println("outputScanHeaderTime != 0");
		// allOff();
		// return;
	// }
	outputScanHeaderTime = now + scanHeaderTimeOffset;

	if (stopTime == -1)  
		stopTime = now + parkTime;
	else if (stopTime == -2) stopTime = -1;
	
}

void readLidar() {
	
	// start: request lidar data
	if (!distanceMeasureStarted && !isBusy && time >= lidarReadyTime) { 
		
		if (count > maxcount) return; // near complete rev, stop reading 
		
		boolean biascorrection = false;
		// if (time > delayreadtime & motorspinning) return;
		if (count == maxcount) biascorrection=true; 
		
		lidarReadyTime = time + LIDARDELAY;	
		// if (count==BIASCORRECTIONCOUNT)  distanceMeasureStart(true); // recommended periodically
		// if (count==0)  distanceMeasureStart(true); // recommended periodically CAUSES SERIOUS LAG
		// else 
		distanceMeasureStart(biascorrection);
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
		
		// if (delayedOutputScanHeader) {
			// outputScanHeader(time);
			// delayedOutputScanHeader = false;
		// }
	}
	
}	

void lidarWarmUp() {
	Serial.println(myLidarLite.distance());
	for (int i=0; i<100; i++)   Serial.println(myLidarLite.distance(false));
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

void stopMotorFacingForward() {
	
	//disableInterrupts();

	unsigned int frequency = frequencyFromRPM(60);
	tone(STEPPIN, frequency); 

	stopTime = -2;
	// if (time - lastrev < (float) cycle*0.75) stopTime = -2;
	
	//long now = micros();
	//long timetonextrev = lastrev + 2*cycle - now;
	
	//long n = cycle*0.75;

	//if (timetonextrev < n) timetonextrev += cycle;
	//timetonextrev = timetonextrev * 0.80 * rpm/60.0;	
	//timetonextrev = timetonextrev/1000.0;
	
	//delay(timetonextrev);
	//hardStop();
	
	//Serial.println(timetonextrev);
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
	digitalWrite(LIDARENABLEPIN, HIGH); // must be 1st! (didn't matter before due to mosfet speed?)
	digitalWrite(LIDARMOSFET, HIGH);
	delay(200);
	lidarenabled = true;
}

void lidarDisable() {
	// digitalWrite(LIDARENABLEPIN, LOW);
	lidarenabled = false;
}

void enableInterrupts() {
	attachInterrupt(digitalPinToInterrupt(PHOTOPIN), trackRPM, LOW);
}

void disableInterrupts() {
	detachInterrupt(digitalPinToInterrupt(PHOTOPIN));
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

// TODO: use top 2 lines if unused, may cause noisier scans/affects gmapping
byte readIfAvailable() {
	// byte b = Wire.read(); 
	// return b;
	
	unsigned long m = 0;
	while (m < 10000) { // 0.1 second timeout
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
	// timedout = 0;
}

unsigned int frequencyFromRPM(int rpm) {
	unsigned int freq = ((float) rpm/60.0) * MICROSTEPS * STEPSPERREV;
	return freq;
}

void setRPM(int n) {
	rpm = n;
	cycle = (60.0/(float) rpm) * 1000000;
	maxcount = cycle/LIDARDELAY - (cycle/LIDARDELAY * 0.014);
	photoUnblockedCheckInterval = cycle/2;
	parkTime = (float)rpm/60.0 * cycle * PARKRATIO;
	scanHeaderTimeOffset = cycle * HEADEROFFSETRATIO;
	
	// Serial.println(maxcount);
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

void version() {
	Serial.println("<version:0.948>"); 
}
