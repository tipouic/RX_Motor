#include "a7105.h"
//########## A7105 ###############
#define SDI_pin 5 //SDIO-D5 
#define SCLK_pin 4 //SCK-D4
#define CS_pin 2//CS-D2

#define  CS_on PORTD |= 0x04 //D2
#define  CS_off PORTD &= 0xFB //D2
#define  SCK_on PORTD |= 0x10//D4
#define  SCK_off PORTD &= 0xEF//D4
#define  SDI_on PORTD |= 0x20 //D5
#define  SDI_off PORTD &= 0xDF //D5
#define  SDI_1 (PIND & 0x20) == 0x20 //D5
#define  SDI_0 (PIND & 0x20) == 0x00 //D5
#define  GIO_1 (PIND & 0x40) == 0x40 //D6 input
#define  GIO_0 (PIND & 0x40) == 0x00 //D6
#define NOP() __asm__ __volatile__("nop")
//#####################################
static const uint8_t allowed_ch[] = {0x14, 0x1E, 0x28, 0x32, 0x3C, 0x46, 0x50, 0x5A, 0x64, 0x6E, 0x78, 0x82};
	static boolean state = true;


void A7105_WriteID(uint32_t ida) {
    CS_off;
    _spi_write(A7105_06_ID_DATA);//ex id=0x5475c52a ;txid3txid2txid1txid0
    _spi_write((ida>>24)&0xff);//53 
    _spi_write((ida>>16)&0xff);//75
    _spi_write((ida>>8)&0xff);//c5
    _spi_write((ida>>0)&0xff);//2a
    CS_on;
}
// read 4 bytes ID
void A7105_ReadID(uint8_t *_aid) {
    uint8_t i;
    CS_off;
    _spi_write(0x46);
    for(i=0;i<4;i++){ _aid[i]=_spi_read(); }
    CS_on;
}

void Read_Packet() {
	uint8_t i;
	CS_off;
	_spi_write(0x45);
	for (i=0;i<21;i++) {	packet[i]=_spi_read();	}
	CS_on;
}
void Write_Packet(uint8_t *_packet, uint8_t len) {
	uint8_t i;
	CS_off;
	_spi_write(A7105_RST_WRPTR);
	_spi_write(0x05);
	for (i=0;i<len;i++) { _spi_write(_packet[i]); }
	CS_on;
}

void A7105_WritePayload(uint8_t *_packet, uint8_t len) {
    uint8_t i;
    CS_off;
    _spi_write(A7105_RST_WRPTR);
    _spi_write(0x05);
    for (i=0;i<len;i++) { _spi_write(_packet[i]); }
    CS_on;
}
void A7105_ReadPayload(uint8_t *_packet, uint8_t len) {
    uint8_t i;
    CS_off;
    _spi_write(0x45);
    for (i=0;i<len;i++) { _packet[i]=_spi_read(); }
    CS_on;
}

void A7105_Reset(void) {	A7105_WriteRegister(A7105_00_MODE,0x00); }

uint8_t A7105_ReadRegister(uint8_t address) { 
    uint8_t result;
    CS_off;
    address |=0x40; 
    _spi_write(address);
    result = _spi_read();  
    CS_on;
    return(result); 
} 
void A7105_WriteRegister(uint8_t address, uint8_t data) {
    CS_off;
    _spi_write(address); 
    _spi_write(data);  
    CS_on;
} 

void A7105_Strobe(uint8_t command) {
    CS_off;
    _spi_write(command);
    CS_on;
}

void A7105_SetPower(int power) {
    /*
    Power amp is ~+16dBm so:
    TXPOWER_100uW  = -23dBm == PAC=0 TBG=0
    TXPOWER_300uW  = -20dBm == PAC=0 TBG=1
    TXPOWER_1mW    = -16dBm == PAC=0 TBG=2
    TXPOWER_3mW    = -11dBm == PAC=0 TBG=4
    TXPOWER_10mW   = -6dBm  == PAC=1 TBG=5
    TXPOWER_30mW   = 0dBm   == PAC=2 TBG=7
    TXPOWER_100mW  = 1dBm   == PAC=3 TBG=7
    TXPOWER_150mW  = 1dBm   == PAC=3 TBG=7
    */
    uint8_t pac, tbg;
    switch(power) {
        case 0: pac = 0; tbg = 0; break;
        case 1: pac = 0; tbg = 1; break;
        case 2: pac = 0; tbg = 2; break;
        case 3: pac = 0; tbg = 4; break;
        case 4: pac = 1; tbg = 5; break;
        case 5: pac = 2; tbg = 7; break;
        case 6: pac = 3; tbg = 7; break;
        case 7: pac = 3; tbg = 7; break;
        default: pac = 0; tbg = 0; break;
    };
    A7105_WriteRegister(0x28, (pac << 3) | tbg);
}

void setupA7105() {
	//RF module pins
	pinMode(SDI_pin, OUTPUT);//SDI   SDIO 
	pinMode(SCLK_pin, OUTPUT);//SCLK SCK 
	pinMode(CS_pin, OUTPUT);//CS output
	pinMode(LED, OUTPUT);//LED
}

//------------------SPI--------------------------
void _spi_write(uint8_t command) {  
	uint8_t n=8; 
	SCK_off;//SCK starts low
	SDI_off;
	while(n--) {
		if(command&0x80)
			SDI_on;
		else 
			SDI_off;
		SCK_on;
		NOP();
		SCK_off;
		command = command << 1;
	}
	SDI_on;
}  
void _spi_write_adress(uint8_t address, uint8_t data) {
	CS_off;
	_spi_write(address); 
	NOP();
	_spi_write(data);  
	CS_on;
} 
uint8_t _spi_read(void) {
	uint8_t result;
	uint8_t i;
	result=0;
	pinMode(SDI_pin,INPUT);//make SDIO pin input
	//SDI_on;
	for(i=0;i<8;i++) {                    
		if(SDI_1)  //if SDIO ==1 
			result=(result<<1)|0x01;
		else
			result=result<<1;
		SCK_on;
		NOP();
		SCK_off;
		NOP();
	}
	pinMode(SDI_pin,OUTPUT);//make SDIO pin output again
	return result;
}
uint8_t _spi_read_adress(uint8_t address) { 
	uint8_t result;
	CS_off;
	address |=0x40;
	_spi_write(address);
	result = _spi_read();  
	CS_on;
	return(result); 
} 

//------------------HUBSAN--------------------------
void init_hubsan(void) {
	A7105_Reset();
	A7105_SetPower(7);
	A7105_WriteID(0x55201041); 
	A7105_WriteRegister(A7105_01_MODE_CONTROL, 0x63);
	A7105_WriteRegister(A7105_03_FIFOI, 0x0f);
	A7105_WriteRegister(A7105_0D_CLOCK, 0x05);
	A7105_WriteRegister(A7105_0E_DATA_RATE, 0x04);
	A7105_WriteRegister(A7105_15_TX_II, 0x2b);
	A7105_WriteRegister(A7105_18_RX, 0x62);
	A7105_WriteRegister(A7105_19_RX_GAIN_I, 0x80);
	A7105_WriteRegister(A7105_1C_RX_GAIN_IV, 0x0A);
	A7105_WriteRegister(A7105_1F_CODE_I, 0x07);
	A7105_WriteRegister(A7105_20_CODE_II, 0x17);
	A7105_WriteRegister(A7105_29_RX_DEM_TEST_I, 0x47);
	A7105_Strobe(A7105_STANDBY);
	A7105_WriteRegister(A7105_02_CALC,0x01);
	A7105_WriteRegister(A7105_0F_PLL_I,0x00);
	A7105_WriteRegister(A7105_02_CALC,0x02);
	A7105_WriteRegister(A7105_0F_PLL_I,0xA0);
	A7105_WriteRegister(A7105_02_CALC,0x02);
	A7105_Strobe(A7105_STANDBY);
}
void hubsan_build_bind_packet(uint8_t bindstate) {
	packet[0] = bindstate;
	packet[1] = (bindstate!=0x0a)? channel : counter;
	packet[6] = 0x08;
	packet[7] = 0xe4;
	packet[8] = 0xea;	
	packet[9] = 0x9e;
	packet[10] = 0x50;
	
	int sum = 0;
	for(int i = 0; i < 15; i++) { sum += packet[i]; }
	packet[15] = (256 - (sum % 256)) & 0xff;
}
void strobeTXRX(void) {
	A7105_WriteRegister(A7105_0F_PLL_I, channel);
	A7105_Strobe(A7105_TX);
	waitTRXCompletion();
	A7105_Strobe(A7105_RX);
	waitTRXCompletion();
	A7105_Strobe(A7105_RST_RDPTR);
}
void waitTRXCompletion(void) {
	while(( A7105_ReadRegister(A7105_00_MODE) & A7105_MODE_TRER_MASK)) 
		;
}
bool hubsan_check_integrity(void) {
	int sum = 0;
	for(int i = 0; i < 15; i++) { sum += packet[i]; }
	return packet[15] == ((256 - (sum % 256)) & 0xff);
}
void hubsan_bind() {
	uint8_t chan=0;
	#ifdef DEBUG
		Serial.println("Binding ...");
	#endif
	
	while(1) {
		if( micros() % 500000 > 250000) { digitalWrite(LED, !LEDetat);  } else { digitalWrite(LED, LEDetat); } //clignote led
		
		A7105_Strobe(A7105_STANDBY);
		channel=allowed_ch[chan];
		if(chan==11) { chan=0; }
		A7105_WriteRegister(A7105_0F_PLL_I, channel);
		A7105_Strobe(A7105_RX);
		unsigned long timer=micros();
		while(1) {
			if((micros()-timer) > 8000) {	chan++; break; }
			if(A7105_ReadRegister(A7105_00_MODE) & A7105_MODE_TRER_MASK){ continue; }
			else {
				A7105_ReadPayload((uint8_t*)&packet, sizeof(packet)); 
				A7105_Strobe(A7105_RST_RDPTR);
				if (packet[0]==1){ break; }	
			}
		}	
		if (packet[0]==1){ break; }
	}
	channel = packet[1];
	
	while(1) {
		hubsan_build_bind_packet(2);
		A7105_Strobe(A7105_STANDBY);
		A7105_WritePayload((uint8_t*)&packet, sizeof(packet));
		strobeTXRX();
		A7105_ReadPayload((uint8_t*)&packet, sizeof(packet));
		if (packet[0]==3){ break; }
	}
	
	hubsan_build_bind_packet(4);
	A7105_Strobe(A7105_STANDBY);
	A7105_WritePayload((uint8_t*)&packet, sizeof(packet));
	A7105_WriteRegister(A7105_0F_PLL_I, channel);
	A7105_Strobe(A7105_TX);
	waitTRXCompletion();
	
	A7105_WriteID(((uint32_t)packet[2] << 24) | ((uint32_t)packet[3] << 16) | ((uint32_t)packet[4] << 8) | packet[5]);
	
	while(1) { // useless block ?
		A7105_Strobe(A7105_RX);
		waitTRXCompletion();
		A7105_Strobe(A7105_RST_RDPTR);
		A7105_ReadPayload((uint8_t*)&packet, sizeof(packet));
		if (packet[0]==1){ break; }
	}
	while(1){
		hubsan_build_bind_packet(2);
		A7105_Strobe(A7105_STANDBY);
		A7105_WritePayload((uint8_t*)&packet, sizeof(packet));
		strobeTXRX();
		A7105_ReadPayload((uint8_t*)&packet, sizeof(packet));
		if (packet[0]==9){ break; }
	}
	while(1){
		counter++;
		if(counter==10) { counter=0; }
		hubsan_build_bind_packet(0x0A);
		A7105_Strobe(A7105_STANDBY);
		A7105_WritePayload((uint8_t*)&packet, sizeof(packet));
		strobeTXRX();
		A7105_ReadPayload((uint8_t*)&packet, sizeof(packet));
		if (counter==9){ break; }
	}
	
	A7105_WriteRegister(A7105_1F_CODE_I,0x0F); //CRC option CRC enabled adress 0x1f data 1111(CRCS=1,IDL=4bytes,PML[1:1]=4 bytes)
	//A7105_WriteRegister(0x28, 0x1F);//set Power to "1" dbm max value.
	A7105_Strobe(A7105_STANDBY);
	for(int i=0;i<4;i++){ txid[i]=packet[i+11]; }
	digitalWrite(LED, HIGH);
}

void bind_Flysky() {
	static byte counter1=255;
	A7105_Strobe(0xA0);
	A7105_Strobe(0xF0);
	_spi_write_adress(0x0F,0x00);//binding listen on channel 0
	A7105_Strobe(0xC0);
	while(counter1){//
		delay(10);//wait 10ms
		if (bitRead(counter1,2)==1){	Red_LED_ON;	}
		if(bitRead(counter1,2)==0){		Red_LED_OFF;	}
		if (GIO_0){
			uint8_t x;
			x=_spi_read_adress(0x00);
			if ((bitRead(x,5)==0)){//test CRC&CRF bits
				Read_Packet();
				uint8_t i;
				uint8_t adr=10;
				for(i=0;i<5;i++){
//					EEPROM.write(adr+i,packet[i]);
					txid[i]=packet[i];
				}
				break;
			}
			else{
				A7105_Strobe(0xA0);
				A7105_Strobe(0xF0);
				_spi_write_adress(0x0F,0x00);//binding listen on channel 0
				A7105_Strobe(0xC0);//try again
				continue;
			}
		}
		else{
			--counter1;
			if (counter1==0){	counter1=255;	}
		}
	}
}