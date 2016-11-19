#include "a7105.h"

//////////////////////CONFIGURATION///////////////////////////////
//	#define proto_flysky true	// commenter pour HUBSAN

#define chanel_number 6  //set the number of chanels (max 4 anaolg + 2 digital
int secuNeutreON=1;
#define LED 13
#define Red_LED_ON  PORTB |= _BV(5);
#define Red_LED_OFF  PORTB &= ~_BV(5);

#define arriere true
#define motor_bridage 50 // pourcentage

	#define DEBUG


#define Servo_OUT 8 //Servo3(B0)
	// Motor PWM = pin 11
#define sensAA 16
#define sensAB 17

#define option_A_binaire 14	// feu nav
#define option_B_binaire 15	// feu pousseur
#define seuilB 70
#define seuilH (255-seuilB)

//########## HUBSAN ###############
#define AUX1_FLAG   0x04 // LED (trim bas gaz long)
#define AUX2_FLAG   0x08 // Flip (mode expert + click gaz)

#define id_servo 4	// direction	2 gaz		4 aileron		6 prof		8 direction

//##############################
//######### Partie fixe ##############
//##############################
#define Servo_OUT_HIGH PORTB |= _BV(0) //Servo3(B0) 8
#define Servo_OUT_LOW PORTB &= _BV(0) //Servo3(B0) 8

#define LEDetat 1
#define PPM_FrLen 22500  //set the PPM frame length in microseconds (1ms = 1000�s)
#define servo_default 1500  //set the default servo value
#define option_default 1000  //set the default option value
#define option_active 2000  //set the active option value
#define default_gaz 1000  //set the default servo value
#define bridage_Max servo_default+(5*motor_bridage)
#define bridage_Min servo_default-(5*motor_bridage)

#define accusV 4
#define accusA 370

//########## Variables #################
static uint32_t id;

static uint16_t Servo_data[10] = {servo_default,servo_default,1000,servo_default,servo_default,servo_default,servo_default,servo_default};
static uint16_t failsafeCnt=0;
volatile byte scale=2;
	static byte cur_chan_numb=0;
	static unsigned int calc_rest=0;
static uint16_t total_servo_time=0;
int failsafeON=0;
int x=0, y=0;
int tmpSensA, tmpSensB, tmpSensM=0, tmpSensN=0, tmpOptA=0, tmpOptB=0;
int tmpPous=option_default;
int accusValue=0; boolean brideDef;

static uint8_t packet[16], channel, counter;
static uint8_t txid[5];
static unsigned long timeout_timer;


void setup() {
	setupA7105();
	
	pinMode(Servo_OUT, OUTPUT); //Servo
	pinMode(11, OUTPUT); //Motor A
		pinMode(sensAA, OUTPUT); //Motor A sens 1
		pinMode(sensAB, OUTPUT); //Motor A sens 2
		
	for (int n=0;n<chanel_number;n++){	Servo_data[n]=servo_default;	}
	
	analogReference(INTERNAL);
	accusValue = map(analogRead(accusV), 0, 1023, 0, 1012);
	brideDef = (accusValue > 600) ? true : false;
	
	#ifdef DEBUG
		Serial.begin(9600);
		Serial.println(" ");
		Serial.println("RC MOTOR"); Serial.println(" ");
		Serial.print(brideDef); Serial.print("\t");		Serial.print(accusValue); Serial.println(" ");
	#endif
	
	delay(10);//wait 10ms for A7105 wakeup
	#ifdef proto_flysky
		uint8_t if_calibration1;
		uint8_t vco_calibration0;
		uint8_t vco_calibration1;
		
		delay(10);//wait 10ms for A7105 wakeup
		_spi_write_adress(0x00,0x00);//reset A7105
		A7105_WriteID(0x5475c52A);//A7105 id
		for (i = 0; i < 0x33; i++) {	if(A7105_regs[i] != 0xff) { _spi_write_adress(i, A7105_regs[i]);	} }
		_spi_strobe(0xA0);//stand-by
		_spi_write_adress(0x02,0x01);
		while(1){
			if_calibration1=_spi_read_adress(0x02);
			if(if_calibration1==0) { break; }
		}
		_spi_read_adress(0x22);
		
		_spi_write_adress(0x24,0x13);
		_spi_write_adress(0x25,0x09);
		_spi_strobe(0xA0);//stand-by
		
		//END A7105 init
		while(1){
			Red_LED_ON;
			delay(500);
			Red_LED_OFF;
			delay(500);
			bind_Flysky();
			Red_LED_ON;
		}
		
		id=(txid[1] | ((uint32_t)txid[2]<<8) | ((uint32_t)txid[3]<<16) | ((uint32_t)txid[4]<<24));
		
		chanrow=id%16;
		chanoffset=(id & 0xff) / 16;
		chancol=0;
		if(chanoffset > 9) chanoffset = 9;//from sloped soarer findings, bug in flysky protocol
	#else
		init_hubsan();
		hubsan_bind();
		A7105_Strobe(A7105_RX);
	#endif
	
	// init timer servo
	#if F_CPU == 16000000// thanks to goebish for this nice idea.
		scale = 2;
	#elif F_CPU == 8000000
		scale = 1;
	#else
		#error // 8 or 16MHz only !
	#endif
	
	// gestion servo
	OCR1A = 50*scale;
	cli();
	TCCR1A = 0; // set entire TCCR1 register to 0
	TCCR1B = 0;
	TCCR1B |= (1 << WGM12);  // turn on CTC mode
	TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
	#if defined(__AVR_ATmega8__)
		// your m8 timer register code
		TIMSK |= (1 << OCIE1A); // enable timer compare interrupt
	#elif defined (__AVR_ATmega328P__)||(__AVR_ATmega88__)||(__AVR_ATmega168__)
		TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
		// your m328 timer register code
	#else
		#error // only m8 & m328 are supported !
	#endif 
	sei();
	
	//gestion motors
	cli();
	TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
	TCCR2B = _BV(CS22);
	OCR2A = 0;
	OCR2B = 0;
	sei();
}
void loop() {
	uint8_t n;
	if(failsafeCnt >700){	  //fs delay 1575ms
		failsafeCnt=0;
		// enter your fs code here
		for (n=0;n<chanel_number;n++){	Servo_data[n]=servo_default;	}
		motor();
		#ifdef DEBUG
			Serial.println("failsafe!");
		#endif
		failsafeON=1;
	}
	if(secuNeutreON) {
		if(micros() % 100000 > 25000) { digitalWrite(LED, !LEDetat); } else { digitalWrite(LED, LEDetat); }	//clignote led
	} else {
		if(failsafeON && micros() % 50000 > 25000) { digitalWrite(LED, !LEDetat); } else { digitalWrite(LED, LEDetat); }	//clignote led
	}
	
	if( (micros()-timeout_timer) > 14000) {		timeout_timer = micros();	A7105_Strobe(A7105_RX);	}
	// nothing received
	if(A7105_ReadRegister(A7105_00_MODE) & A7105_MODE_TRER_MASK) {
		if(micros()%100 == 0) { failsafeCnt++; }
		return; 
	}
	
	A7105_ReadPayload((uint8_t*)&packet, sizeof(packet)); 
	if(!((packet[11]==txid[0])&&(packet[12]==txid[1])&&(packet[13]==txid[2])&&(packet[14]==txid[3]))) { return; } // not our TX !
	if(!hubsan_check_integrity()) { return;}	// bad checksum
	timeout_timer = micros();
	A7105_Strobe(A7105_RST_RDPTR);
	A7105_Strobe(A7105_RX);
	if(packet[0]==32) {
		failsafeCnt=0; failsafeON=0;
		if(secuNeutreON) {
			#if arriere
				if(packet[2]==127) { secuNeutreON=0; } 
			#else
				if(packet[2]==0) { secuNeutreON=0; } 
			#endif
			#ifdef DEBUG
				Serial.print("\tSecu neutre motor\t");
				Serial.println(packet[2]);
			#endif
		}
		else {
			// "LEDs" channel, AUX1 	Bridage motor
			if((packet[9] & AUX1_FLAG) && brideDef) {
				Servo_data[0]=map(packet[2],0,255,bridage_Min,bridage_Max);	//gaz
			} else {
				Servo_data[0]=map(packet[2],0,255,1000,2000);	//gaz
			}
			Servo_data[1]=map(packet[id_servo],255,0,1000,2000);	//aileron
			Servo_data[2]=servo_default;	//prof
			Servo_data[3]=servo_default;	//direction
			// "Flip" channel, AUX2 (only on H107L, H107C, H107D and Deviation TXs, high by default)
			if(packet[6]>seuilH) { tmpPous=option_default; }
			else if(packet[6]<seuilB) { tmpPous=option_active; }
			Servo_data[4]=(packet[9] & AUX2_FLAG) ? option_default : option_active;
			Servo_data[5]=(packet[9] & AUX2_FLAG) ? option_default : tmpPous;
			
			//Protection accus => avertisseur moteur
			if(accusValue<accusA && micros() % 100000 > 25000) {
				#if arriere
					Servo_data[0]= 126;
				#else
					Servo_data[0]= 0;
				#endif
			}
			motor();
		}
	}
	
	accusValue = map(analogRead(accusV), 0, 1023, 0, 1012);
	
	#ifdef DEBUG
		for(int i=0; i<chanel_number; i++) { Serial.print(i); Serial.print(":"); Serial.print(Servo_data[i]); Serial.print("\t"); }
//		for(int i=0; i<chanel_number; i++) { Serial.print(packet[i*2]); Serial.print("\t"); }
		Serial.print(digitalRead(option_A_binaire)); Serial.print("\t");
		Serial.print(digitalRead(option_B_binaire)); Serial.print("\t");
		Serial.print(brideDef); Serial.print("\t");
		Serial.print(accusValue); Serial.print("\t");
		Serial.println("");
	#endif
}

ISR(TIMER1_COMPA_vect) {
	TCNT1 = 0;
	Servo_OUT_LOW;
	if(cur_chan_numb < chanel_number) {
		total_servo_time +=Servo_data[cur_chan_numb]*scale;
		OCR1A=Servo_data[cur_chan_numb]*scale;
	}
	else {
		OCR1A=PPM_FrLen*scale-total_servo_time;
		cur_chan_numb = 0xff;
		total_servo_time=0;
	}
	
	switch (cur_chan_numb) {
		case 0:				break;	// motor
		case 1:		digitalWrite(Servo_OUT, HIGH);		break;	// direction
		case 2:		digitalWrite(Servo_OUT, LOW);		break;	// vide
		case 3:				break;	// vide
		case 4:
			if(tmpOptA != Servo_data[cur_chan_numb]) {
				digitalWrite(option_A_binaire, (Servo_data[cur_chan_numb] > servo_default) ? HIGH : LOW);
				tmpOptA = Servo_data[cur_chan_numb];
			}
			break;
		case 5:	
			if(tmpOptB != Servo_data[cur_chan_numb]) {
				digitalWrite(option_B_binaire, (Servo_data[cur_chan_numb] > servo_default) ? HIGH : LOW);
				tmpOptB = Servo_data[cur_chan_numb];
			}
			break;
	}
	cur_chan_numb++;//next servo
}
void motor(void) {
	tmpSensA = map(Servo_data[0], 1000, 2000, 0, 255);
	int tmpX=0, tmpY=0;
	#if arriere
		if(tmpSensA>126) {
			if(tmpSensM!=2) {	digitalWrite(sensAA, HIGH);		digitalWrite(sensAB, LOW);		tmpSensM=2;	}
			tmpX = constrain((tmpSensA-127)*2, 0, 255);
			OCR2A = tmpX;
		} else {
			if(tmpSensM!=1) {	digitalWrite(sensAB, HIGH);		digitalWrite(sensAA, LOW);		tmpSensM=1;	}
			OCR2A=constrain((127-tmpSensA)*2, 0, 255);
		} 
		#ifdef DEBUG
			Serial.print(OCR2A); Serial.print(" ");
			Serial.print(digitalRead(sensAA)); Serial.print("-");
			Serial.print(digitalRead(sensAB)); Serial.print("...\t");
		#endif
	#else
		OCR2A=tmpSensA;
	#endif
}
