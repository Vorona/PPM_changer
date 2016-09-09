/*This program puts the servo values into an array,
 reagrdless of channel number, polarity, ppm frame length, etc...
 You can even change these while scanning!*/

#include <EEPROM.h>
 
#define PPM_Pin 3  //this must be 2 or 3

#define button_pin 9
#define led_pin 8

#define chanel_number 8  //set the number of chanels

#define default_servo_value 0  //set the default servo value
#define PPM_FrLen 22500  //set the PPM frame length in microseconds (1ms = 1000�s)
#define PPM_PulseLen 300  //set the pulse length
#define onState 1  //set polarity of the pulses: 1 is positive, 0 is negative
#define sigPin 10  //set PPM signal output pin on the arduino

#define EEP_COPTER 0 //EEPROM address of copter flight mode switch mode

int ppm[16];  //array for storing up to 16 servo signals

int *ppm_out[chanel_number];

void setup()
{
byte COPTER_MODE = 0x00;

  
  //initiallize default ppm values
  for(int i=0; i<chanel_number; i++){
    ppm[i]= default_servo_value;
    ppm_out[i] = &ppm[i];
  }
  ppm_out[0] = &ppm[1];
  ppm_out[1] = &ppm[0];
  
  EEPROM.get(EEP_COPTER, COPTER_MODE);
  
  pinMode(button_pin, INPUT_PULLUP);
  pinMode(led_pin, OUTPUT);
  digitalWrite(led_pin, HIGH);
  if (digitalRead(button_pin) == LOW){
	  COPTER_MODE = (COPTER_MODE == 0x00)?0x01:0x00;
	  EEPROM.update(EEP_COPTER, COPTER_MODE);
  }
  
  if (COPTER_MODE)
  {
    digitalWrite(led_pin, LOW);
    ppm_out[4] = &ppm[7];
    ppm_out[7] = &ppm[4];
    ppm_out[5] = &ppm[6];
    ppm_out[6] = &ppm[5];
  }
  
  pinMode(sigPin, OUTPUT);
  digitalWrite(sigPin, !onState);  //set the PPM signal pin to the default state (off)

  pinMode(PPM_Pin, INPUT);

  cli();
  
  attachInterrupt(PPM_Pin - 2, read_ppm, CHANGE);  
  
  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;
  
  OCR1A = 100;  // compare match register, change this
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  
  TCCR3A = 0;  //reset timer1
  TCCR3B = 0;
  TCCR3B |= (1 << CS11);  //set timer1 to increment every 0,5 us
  sei();
//  Serial.begin(9600);
}

void loop()
{
//  int count;
//  //You can delete everithing inside loop() and put your own code here
//  int count;
//
//  while(ppm[count] != 0){  //print out the servo values
//    Serial.print(ppm[count]);
//    Serial.print("  ");
//    count++;
//  }
//  Serial.println("");
//  delay(100);  //you can even use delays!!!
}



void read_ppm(){  //leave this alone
  static unsigned int pulse;
  static unsigned long counter;
  static byte channel;
  int ppmtmp;

  counter = TCNT3;
  TCNT3 = 0;

  if(counter < 1020){  //must be a pulse if less than 510us
    pulse = counter;
  }
  else if(counter > 3820){  //sync pulses over 1910us
    channel = 0;
  }
  else{  //servo values between 510us and 2420us will end up here
    ppmtmp = (counter + pulse)/2;
    ppm[channel] = (abs(ppm[channel]-ppmtmp) > 5)?ppmtmp:ppm[channel];
    channel++;
  }
}

ISR(TIMER1_COMPA_vect){  //leave this alone
  static boolean state = true;
  
  TCNT1 = 0;
  
  if(state) {  //start pulse
    digitalWrite(sigPin, onState);
    OCR1A = PPM_PulseLen * 2;
    state = false;
  }
  else{  //end pulse and calculate when to start the next pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;
  
    digitalWrite(sigPin, !onState);
    state = true;

    if(cur_chan_numb >= chanel_number){
      cur_chan_numb = 0;
      calc_rest = calc_rest + PPM_PulseLen;// 
      OCR1A = (PPM_FrLen - calc_rest) * 2;
      calc_rest = 0;
    }
    else{
      OCR1A = (*ppm_out[cur_chan_numb] - PPM_PulseLen) * 2;
      calc_rest = calc_rest + *ppm_out[cur_chan_numb];
      cur_chan_numb++;
    }     
  }
}
