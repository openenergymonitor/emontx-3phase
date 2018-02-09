/*
Interface for the RFM69CW Radio Module
*/

#ifdef RFM69CW

#include <avr/sleep.h>	
#define REG_FIFO            0x00	
#define REG_OPMODE          0x01
#define MODE_TRANSMITTER    0x0C
#define REG_DIOMAPPING1     0x25	
#define REG_IRQFLAGS2       0x28
#define IRQ2_FIFOFULL       0x80
#define IRQ2_FIFONOTEMPTY   0x40
#define IRQ2_PACKETSENT     0x08
#define IRQ2_FIFOOVERRUN    0x10



void rfm_init(void)
{	
	// Set up to drive the Radio Module
	digitalWrite(RFMSELPIN, HIGH);
	pinMode(RFMSELPIN, OUTPUT);
	SPI.begin();
	SPI.setBitOrder(MSBFIRST);
	SPI.setDataMode(0);
	SPI.setClockDivider(SPI_CLOCK_DIV4); // decided to slow down from DIV2 after SPI stalling in some instances, especially visible on mega1284p when RFM69 and FLASH chip both present
	
	// Initialise RFM69CW
	do 
		writeReg(0x2F, 0xAA); // RegSyncValue1
	while (readReg(0x2F) != 0xAA) ;
	do
	  writeReg(0x2F, 0x55); 
	while (readReg(0x2F) != 0x55);
	
	writeReg(0x01, 0x04); // RegOpMode: RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY
	writeReg(0x02, 0x00); // RegDataModul: RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00 = no shaping
	writeReg(0x03, 0x02); // RegBitrateMsb  ~49.23k BPS
	writeReg(0x04, 0x8A); // RegBitrateLsb
	writeReg(0x05, 0x05); // RegFdevMsb: ~90 kHz 
	writeReg(0x06, 0xC3); // RegFdevLsb
	#ifdef RF12_868MHZ
		writeReg(0x07, 0xD9); // RegFrfMsb: Frf = Rf Freq / 61.03515625 Hz = 0xD90000 = 868.00 MHz as used JeeLib  
		writeReg(0x08, 0x00); // RegFrfMid
		writeReg(0x09, 0x00); // RegFrfLsb
	#elif defined RF12_915MHZ // JeeLib uses 912.00 MHz	
		writeReg(0x07, 0xE4); // RegFrfMsb: Frf = Rf Freq / 61.03515625 Hz = 0xE40000 = 912.00 MHz as used JeeLib 
		writeReg(0x08, 0x00); // RegFrfMid
		writeReg(0x09, 0x00); // RegFrfLsb
	#else // default to 433 MHz band
		writeReg(0x07, 0x6C); // RegFrfMsb: Frf = Rf Freq / 61.03515625 Hz = 0x6C8000 = 434.00 MHz as used JeeLib 
		writeReg(0x08, 0x80); // RegFrfMid
		writeReg(0x09, 0x00); // RegFrfLsb
	#endif

//	writeReg(0x0B, 0x20); // RegAfcCtrl:
	writeReg(0x11, RFPWR); // RegPaLevel = 0x9F = PA0 on, +13 dBm  -- RFM12B equivalent: 0x99 | 0x88 (-10dBm) appears to be the max before the AC power supply fails @ 230 V mains. Min value is 0x80 (-18 dBm)
	writeReg(0x1E, 0x2C); //
	writeReg(0x25, 0x80); // RegDioMapping1: DIO0 is used as IRQ 
	writeReg(0x26, 0x03); // RegDioMapping2: ClkOut off
	writeReg(0x28, 0x00); // RegIrqFlags2: FifoOverrun

	// RegPreamble (0x2c, 0x2d): default 0x0003
	writeReg(0x2E, 0x88); // RegSyncConfig: SyncOn | FifoFillCondition | SyncSize = 2 bytes | SyncTol = 0
	writeReg(0x2F, 0x2D); // RegSyncValue1: Same as JeeLib
	writeReg(0x30, networkGroup); // RegSyncValue2
	writeReg(0x37, 0x00); // RegPacketConfig1: PacketFormat=fixed | !DcFree | !CrcOn | !CrcAutoClearOff | !AddressFiltering >> 0x00
}


// transmit data via the RFM69CW
void rfm_send(const byte *data, const byte size, const byte group, const byte node)      // *SEND RF DATA*
{
	while (readReg(REG_IRQFLAGS2) & (IRQ2_FIFONOTEMPTY | IRQ2_FIFOOVERRUN))		// Flush FIFO
        readReg(REG_FIFO);

    writeReg(REG_DIOMAPPING1, 0x00); 											// PacketSent
		
	volatile uint8_t txstate = 0;
	byte i = 0;
	uint16_t crc = _crc16_update(~0, group);	

	while(txstate < 7)
	{
		if ((readReg(REG_IRQFLAGS2) & IRQ2_FIFOFULL) == 0)			// FIFO !full
		{
			uint8_t next;
			switch(txstate)
			{
			  case 0: next=node & 0x1F; txstate++; break;    		// Bits: CTL, DST, ACK, Node ID(5)
			  case 1: next=size; txstate++; break;				   	// No. of payload bytes
			  case 2: next=data[i++]; if(i==size) txstate++; break;
			  case 3: next=(byte)crc; txstate++; break;
			  case 4: next=(byte)(crc>>8); txstate++; break;
			  case 5:
			  case 6: next=0xAA; txstate++; break; 					// dummy bytes (if < 2, locks up)
			}
			if(txstate<4) crc = _crc16_update(crc, next);
			writeReg(REG_FIFO, next);								// RegFifo(next);
		}
	}
    //transmit buffer is now filled, transmit, flag that to the ISR
	writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | MODE_TRANSMITTER);		// Transmit mode - 56 Bytes max payload
   
    rfm_sleep();
}

void rfm_sleep(void)
{   
    // Put into sleep mode when buffer is empty
	while (!(readReg(REG_IRQFLAGS2) & IRQ2_PACKETSENT))				// wait for transmission to complete (not present in JeeLib) 
	    delay(1);													//   

	writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | 0x01); 		// Standby Mode
	set_sleep_mode(SLEEP_MODE_IDLE);  								// SLEEP_MODE_STANDBY
	sleep_mode();	
	
} 



void writeReg(uint8_t addr, uint8_t value)
{
	select();
	SPI.transfer(addr | 0x80);
	SPI.transfer(value);
	unselect();
}

uint8_t readReg(uint8_t addr)
{
	select();
	SPI.transfer(addr & 0x7F);
	uint8_t regval = SPI.transfer(0);
	unselect();
	return regval;
}

// select the transceiver
void select() {
	noInterrupts();
	SPI.setDataMode(SPI_MODE0);
	SPI.setBitOrder(MSBFIRST);
	SPI.setClockDivider(SPI_CLOCK_DIV4); // decided to slow down from DIV2 after SPI stalling in some instances, especially visible on mega1284p when RFM69 and FLASH chip both present
	digitalWrite(RFMSELPIN, LOW);
}

// UNselect the transceiver chip
void unselect() {
	digitalWrite(RFMSELPIN, HIGH);
	interrupts();
}
#endif
