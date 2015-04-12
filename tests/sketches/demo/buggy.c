#include <pin.h>
#include <stdio.h>
#include <ardutime.h>
#include <arduAdvIO.h>
#include <arduthread.h>

const int leftmotorpin1 = 8; //signal output of Dc motor driven plate
const int leftmotorpin2 = 9;
const int rightmotorpin1 = 6;
const int rightmotorpin2 = 7;

const int trigPin = 11;
const int echoPin = 2;

void setup() {
  // put your setup code here, to run once:
  pinMode(leftmotorpin1, OUTPUT);
  pinMode(leftmotorpin2, OUTPUT);
  pinMode(rightmotorpin1, OUTPUT);
  pinMode(rightmotorpin2, OUTPUT);
  //pinMode(13, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, FAST_INPUT);
}

#if 0
void loop(1, 70, 100) {
  delay(200);
  digitalWrite(13, HIGH);
  delay(200);
  digitalWrite(13, LOW);
}
#endif

long distance = 0;

void nodanger() {
 digitalWrite(leftmotorpin1, HIGH);
 digitalWrite(leftmotorpin2, LOW);
 digitalWrite(rightmotorpin1, HIGH);
 digitalWrite(rightmotorpin2, LOW);
}

void backup() {
 digitalWrite(leftmotorpin1, LOW);
 digitalWrite(leftmotorpin2, HIGH);
 digitalWrite(rightmotorpin1, LOW);
 digitalWrite(rightmotorpin2, HIGH);
 delay(600);
}

//turn left
void body_lturn() {
 digitalWrite(leftmotorpin1, LOW);
 digitalWrite(leftmotorpin2, HIGH);
 digitalWrite(rightmotorpin1, HIGH);
 digitalWrite(rightmotorpin2, LOW);
 delay(5000);
 //totalhalt();
}
//turn right
void body_rturn() {
 digitalWrite(leftmotorpin1, HIGH);
 digitalWrite(leftmotorpin2, LOW);
 digitalWrite(rightmotorpin1, LOW);
 digitalWrite(rightmotorpin2, HIGH);
 delay(1000);
 //totalhalt();
}

void loop(1, 20, 100){
  // put your main code here, to run repeatedly: 
  //digitalWrite(leftmotorpin1, HIGH);
  //digitalWrite(leftmotorpin2, LOW);
  //digitalWrite(rightmotorpin1, HIGH);
  //digitalWrite(rightmotorpin2, LOW);
	//delay(100);
  //delay(200);
  //digitalWrite(13, HIGH);
  //delay(200);
  //digitalWrite(13, LOW);
	if (distance > 100 || distance == 0)
		nodanger();
	else {
		backup();
		body_rturn();
		backup();
		body_rturn();
	}
	delay(100);
}

void loop(2, 70, 100) {
	long duration;// distance;
	digitalWrite(trigPin, LOW);
	delayMicroseconds(2);
	digitalWrite(trigPin, HIGH);
	delayMicroseconds(10);
	digitalWrite(trigPin, LOW);
	duration = pulseIn(echoPin, HIGH);
	distance = (duration / 2) / 29.1;
	printf("%ld cm\n", distance);
	delay(200);
	/*
	unsigned long start, end, dur;
	start = micros();
	digitalWrite(13, HIGH);
	end = micros();
	dur = end - start;
	printf("%ld\n", dur);
	*/
}


