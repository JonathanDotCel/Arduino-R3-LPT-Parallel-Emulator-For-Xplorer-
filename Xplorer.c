//
// Xplorer/Xploder connectivity via an Arduino R3
// 2015, Jonathan Cel
// See LICENSE file for the terms of the GPL license.
//
// Hardware Note:
// You'll need an arduino with the 5 extra dual analogue/digital mode pins to pull this off.
// 
// 
// Credits
//
// Based on code from http://www.savaitgalioprojektai.lt/arduino-projektai/arduino-lpt-spauzdintuvas/
// Information from here:
// http://retired.beyondlogic.org/spp/parallel.htm
// And various XPlorer/Caetla disassemblies/sources
// 

/*
//
// Thhis is the ack/nack table in full
// Normally this makes more sense, but since it takes a chunk of
// the Arduino's limited space, I've taken the CatFlap approach below
//
byte lookups[]={
	0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,
	0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,
	0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,
	0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,
	0x0c,0x0c,0x0c,0x0c,0x0c,0x0c,0x0c,0x0c,0x0c,0x0c,0x0c,0x0c,0x0c,0x0c,0x0c,0x0c,
	0x0d,0x0d,0x0d,0x0d,0x0d,0x0d,0x0d,0x0d,0x0d,0x0d,0x0d,0x0d,0x0d,0x0d,0x0d,0x0d,
	0x0e,0x0e,0x0e,0x0e,0x0e,0x0e,0x0e,0x0e,0x0e,0x0e,0x0e,0x0e,0x0e,0x0e,0x0e,0x0e,
	0x0f,0x0f,0x0f,0x0f,0x0f,0x0f,0x0f,0x0f,0x0f,0x0f,0x0f,0x0f,0x0f,0x0f,0x0f,0x0f,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
	0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,
	0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,
	0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,
	0x09,0x09,0x09,0x09,0x09,0x09,0x09,0x09,0x09,0x09,0x09,0x09,0x09,0x09,0x09,0x09,
	0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,
	0x0b,0x0b,0x0b,0x0b,0x0b,0x0b,0x0b,0x0b,0x0b,0x0b,0x0b,0x0b,0x0b,0x0b,0x0b,0x0b,
};
*/

// Slower than the precalc table below, but saves a chunk of space on the arduino.
// Thanks to the Catflap folks for this one.
const byte aps[] = { 0x04, 0x05, 0x06, 0x07, 0x0c, 0x0d, 0x0e, 0x0f, 0x00, 0x01, 0x02, 0x03, 0x08, 0x09, 0x0a, 0x0b };


// DATA PORT - which arduino pin is each on?
// Skipping 0/1 as they can be used for Serial IO at the same time.

const int DATA_0 = 2;      //LPT2
const int DATA_1 = 3;      //LPT3
const int DATA_2 = 4;      //LPT4
const int DATA_3 = 5;      //LPT5
const int DATA_4 = 6;      //LPT6
const int DATA_5 = 7;      //LPT7
const int DATA_6 = 8;      //LPT8
const int DATA_7 = 9;      //LPT9

						   //"i" Denotes active low, i.e. this line is normally HIGH. E.g. on the error pin "HIGH" would 
						   //mean all is well, "LOW" means the shit hit the fan.
						   //"h" means it's hardware inverted, such as the busy line, where if +5V is applied, it'll return 0.
						   //Some are both.

// STATUS PORT
// pins 10/11 (busy/ack are in reverse order compared to their bit values)
const int inACKi = 14;       //LPT10 
const int inBUSYh = 15;      //LPT11
const int inPAPEROUT = 16;   //LPT12
const int inSELECT = 17;     //LPT13
const int inERRORi = 18;     //LPT15

// CONTROL PORT
const int outPSELECTih = 10;    //LPT17
const int outRESETINITi = 11;	//LPT16
const int outLINEFEEDh = 12;    //LPT14
const int outSTROBEih = 13;     //LPT1    (not 0 indexed here, 1 is actually pin 1)


// Various vars.
int nVal = 1;
int dbg = 2;
int tickCounter = 0;
int returnByte = 0;
bool gotFirstByte = false;
byte firstByte = 0;

int ackWaitCount = 5;
int ackWaitTime = 1;
int currentByte = 0;

void setup() {

	// We're no longer using the lookup table.
	//        createLookup();

	// To the PC
	Serial.begin( 9600 );

	//DATA PORT
	pinMode( DATA_0, OUTPUT );
	pinMode( DATA_1, OUTPUT );
	pinMode( DATA_2, OUTPUT );
	pinMode( DATA_3, OUTPUT );
	pinMode( DATA_4, OUTPUT );
	pinMode( DATA_5, OUTPUT );
	pinMode( DATA_6, OUTPUT );
	pinMode( DATA_7, OUTPUT );

	setAllBits( 0 );

	//STATUS PORT
	pinMode( inACKi, INPUT_PULLUP );
	pinMode( inBUSYh, INPUT_PULLUP );
	pinMode( inPAPEROUT, INPUT_PULLUP );
	pinMode( inSELECT, INPUT_PULLUP );
	pinMode( inERRORi, INPUT_PULLUP );

	//CONTROL PORT
	pinMode( outPSELECTih, OUTPUT );
	pinMode( outRESETINITi, OUTPUT );
	pinMode( outLINEFEEDh, OUTPUT );
	pinMode( outSTROBEih, OUTPUT );

	Serial.flush();

}//======================

void loop(){

	while ( !Serial.available() ){

		if ( dbg >= 0 ){
			delay( 1000 );
		}

		if ( dbg >= 2 ) Serial.println( "|ACK: " + String( digitalRead( inACKi ), HEX ) + " PSELECT= " + String( digitalRead( outPSELECTih ), HEX ) );
		if ( dbg >= 2 ) Serial.println( "|CT " + String( tickCounter, HEX ) );

		tickCounter++;
	}

	//Clear the read buffer (flush() only clears the write buffer);
	while ( Serial.available() ){

		if ( !gotFirstByte ){
			firstByte = Serial.read();
			gotFirstByte = true;
		} else{
			Serial.read();
		}

	}

	//1 = request, 2= resume, 3 is reset, 8 is status
	if ( dbg >= 1 ){

		if ( dbg >= 2 ) printInputs();

		// Set everything to a known state
		setAllBits( 0 );

		// Send command 0x57, 0x53
		// ( Upload to memory )
		doByte( 0x57 );
		doByte( 0x53 );

		// The addr we'll upload bytes to
		// e.g. 0x80024444 (little endian)
		doByte( 0x80 );
		doByte( 0x02 );
		doByte( 0x44 );
		doByte( 0x44 );

		// There will be 14h
		// e.g. 20d or 5 32-bit words
		doByte( 0x00 );
		doByte( 0x00 );
		doByte( 0x00 );
		doByte( 0x14 );

		// xor t0,t0
		// instruction zeros out the t0 register
		doByte( 0x26 );
		doByte( 0x40 );
		doByte( 0x08 );
		doByte( 0x01 );

		// lui t0,$BFC0
		// Loads BFC0 into the upper part of t0 (BFC0****)
		doByte( 0xC0 );
		doByte( 0xBF );
		doByte( 0x08 );
		doByte( 0x3C );

		// lui t0,$BFC0
		// ORs 0 to t0 (lower half = ****0000)
		// not necessary since we zeroed it, but good practice
		doByte( 0x00 );
		doByte( 0x00 );
		doByte( 0x08 );
		doByte( 0x25 );

		// Jump to the addr in t0 (reset the PSX)
		// jr t0
		doByte( 0x08 );
		doByte( 0x00 );
		doByte( 0x00 );
		doByte( 0x01 );

		//some nops, for good luck you see
		doByte( 0x00 );
		doByte( 0x00 );
		doByte( 0x00 );
		doByte( 0x00 );

		// Tell it we're done
		// These ones require ACKs
		doByte( 0x02 );
		backAck();

		doByte( 0x68 );
		backAck();

		// And a few more for good measure
		backAck();
		backAck();


		// Then jump to our new bytecode
		// 

		// 0x57, 0x0F - Jump to addr immediate
		doByte( 0x57 );
		doByte( 0x0F );

		// 0x80024444
		// Where we just put all those instructions
		doByte( 0x80 );
		doByte( 0x02 );
		doByte( 0x44 );
		doByte( 0x44 );

		// Done.
		Serial.println( "| ****************************HA1*******************" );
		Serial.println( "|*****************************HA1*******************" );
		Serial.println( "| ****************************HA1*******************" );


	} else{
		doByte( firstByte );
	}

	gotFirstByte = false;

	Serial.write( returnByte );

	Serial.flush();

}

void doByte( byte inByte ){

	if ( dbg >= 2 ) Serial.println( "| Doing byte____________________ #=" + String( nVal, HEX ) + " value = " + String( inByte, HEX ) );

	currentByte = inByte;
	returnByte = 0;

	setAllBits( inByte );
	ackStuff();

	nVal++;

}

// Ack routine, string inputs are purely for debugging.
void ackStuff(){

	//7 is low originally, but inverted here.. but is high here
	goHigh( "2" );
	XpAck1( "ack1" );

	goLow( "1" );
	XpAck2( "ack2" );

	if ( dbg >= 4 ) Serial.println( "--acked--" );

}

// The other Ack routine
void backAck(){

	int in1, in2, in3;

	in1 = XpAck1( "lh1" );
	goHigh( "lh1" );

	in2 = XpAck1_N( "lh2" );
	goLow( "lh2" );

	in3 = XpAck1( "lh3" );
	goHigh( "lh3" );

	XpAck1_N( "lh4" );
	goLow( "lh4" );

	//XpAck2_N("lh5"); 
	//Fuckit?

	// This took some time...
	returnByte = ( ( ( in1 & 0x30 ) << 2 ) | ( ( ( ~in2 ) & 0x80 ) >> 2 ) | ( ( in2 & 0x30 ) >> 1 ) | ( ( ( ~in3 ) & 0x80 ) >> 5 ) | ( ( in3 & 0x30 ) >> 4 ) );
	if ( dbg >= 1 ) Serial.println( "| Final Byte: " + String( returnByte, HEX ) + " = " + returnByte );

}

void doReset(){

	// Command 0x57, 0x0F
	doByte( 0x57 );
	doByte( 0x0F );

	doByte( 0xBF );
	doByte( 0xC0 );
	doByte( 0x00 );
	doByte( 0x00 );

}

// E.g. as the old DOS software did to mess around
// with the GPU.
void doFreeze(){

	doByte( 0x57 );
	doByte( 0x4C ); //L

}
void doUnfreeze(){

	doByte( 0x57 ); //R
	doByte( 0x52 );

}


void printInputs(){

	Serial.println( "|||| IN:= " );
	Serial.println( "|||| BUSY=" + String( digitalRead( inBUSYh ) ) + "  ACK=" + String( digitalRead( inACKi ) ) + "  PAPEROUT=" + String( digitalRead( inPAPEROUT ) ) );
	Serial.println( "|||| PAPEROUT " + String( digitalRead( inSELECT ) ) + "   ERROR=" + String( digitalRead( inERRORi ) ) );
	Serial.println( "|--- OUT:= " );
	Serial.println( "|--- SELECT=" + String( digitalRead( outPSELECTih ) ) + " INIT(RESET)= " + String( digitalRead( outRESETINITi ) ) );
	Serial.println( "|--- LINEF=" + String( digitalRead( outLINEFEEDh ) ) + " STROBE= " + String( digitalRead( outSTROBEih ) ) );

}


// Set the lines high/low/wait, etc
void goHigh( String inString ){

	if ( dbg >= 3 ) Serial.println( "|" + inString + " setting Strobe HI" );
	digitalWrite( outPSELECTih, HIGH );

}

void waitHigh( String inString ){

	while ( digitalRead( inACKi ) < 1 ){
		if ( dbg >= 1 ) Serial.println( "|" + inString + " waiting for ACK HIGH" );
		if ( dbg >= 2 ) printInputs();
		delay( ackWaitTime );
	}

}

void goLow( String inString ){

	if ( dbg >= 3 ) Serial.println( "|" + inString + " setting Strobe HI" );
	digitalWrite( outPSELECTih, LOW );

}

void waitLow( String inString ){

	while ( digitalRead( inACKi ) > 1 ){
		if ( dbg >= 1 ) Serial.println( "|" + inString + " waiting for ACK LOW" );
		if ( dbg >= 2 ) printInputs();
		delay( ackWaitTime );
	}

}


// Just a convenience function
void setAllBits( byte inByte ){

	byte xByte = inByte;

	int b0 = bitRead( xByte, 0 );
	int b1 = bitRead( xByte, 1 );
	int b2 = bitRead( xByte, 2 );
	int b3 = bitRead( xByte, 3 );
	int b4 = bitRead( xByte, 4 );
	int b5 = bitRead( xByte, 5 );
	int b6 = bitRead( xByte, 6 );
	int b7 = bitRead( xByte, 7 );

	digitalWrite( DATA_0, b0 );
	digitalWrite( DATA_1, b1 );
	digitalWrite( DATA_2, b2 );
	digitalWrite( DATA_3, b3 );
	digitalWrite( DATA_4, b4 );
	digitalWrite( DATA_5, b5 );
	digitalWrite( DATA_6, b6 );
	digitalWrite( DATA_7, b7 );

}

// Get a status byte back from the cart.
// Now you see why this is so slow without hardware support!
byte getStatusVal(){

	//Bit7 "BUSY" is automatically hardware inverted. Bastards.
	//Bits 6,3 ACK/ERROR are HIGH by default

	byte returnVal = 0;
	returnVal |= ( ( digitalRead( inBUSYh ) ^ 1 ) << 7 );  //used for calc
	returnVal |= ( ( digitalRead( inACKi ) ) << 6 );   //used for timing
	returnVal |= ( ( digitalRead( inPAPEROUT ) ) << 5 ); //used for calc
	returnVal |= ( ( digitalRead( inSELECT ) ) << 4 );  //used for calc
	returnVal |= ( ( digitalRead( inERRORi ) ^ 1 ) << 3 );

	//pin 2 on a standard port (IRQ) is also active low, but the handshake table strips anything below #16.
	//pins 1 & 0 are also be reserved.
	return returnVal;

}


// The XP ack sequences are different for Caetla/XPlorer, but involve
// comparing the 5th bit to a value from the ack table (above)
// you can legitimately skip these, but then the (already poor)
// timing suffers further.

byte XpAck1( String inVal ){

	if ( dbg >= 4 ) Serial.println( "| (XpAck1)  iv " + inVal );
	int inA, inB, inCx;

	//Serial.println("| inB is " + String( inB, HEX) ) ;
	while ( true ){

		inA = getStatusVal();
		inB = aps[ inA >> 4 ];

		//Now we have to hope that inA is of value 4x,5x,6x,7x,ex,fx, since those values in the table have the 8th bit.
		//Serial.println("| A B C " + String( inA, HEX) + " " + String( inB, HEX) + " " + String( inC, HEX) );

		if ( ( inB & 0x08 ) != 0 ){

			inCx = getStatusVal();
			if ( inCx == inA ){
				if ( dbg >= 4 ) Serial.println( "| XpAck1 returning!" );
				return( inCx );
			}

		} else{
			if ( dbg >= 2 ) Serial.println( "| XpAck1 WAITING!" );
		}

		// Depending on the speed of your Arduino?
		//delayMicroseconds(MSD); 

	}

}

// See XP Ack 1 description
byte XpAck1_N( String inVal ){

	if ( dbg >= 4 ) Serial.println( "| XpAck1_N iv " + inVal );

	int inA, inB, inCx;

	while ( true ){

		inA = getStatusVal();
		inB = aps[ inA >> 4 ];
		//Serial.println("| A B C " + String( inA, HEX) + " " + String( inB, HEX) + " " + String( inC, HEX) ); 

		if ( ( inB & 0x08 ) == 0 ){

			//Serial.println("| not 0 yet...");
			inCx = getStatusVal();
			if ( inCx == inA ){
				if ( dbg >= 4 ) Serial.println( "| (XpAck1_N) returning!" );
				return( inCx );
			}
		}

		// Depending on the speed of your Arduino?
		//delayMicroseconds(MSD); 

	}

}

// See XP Ack 1 description
byte XpAck2( String inVal ){

	if ( dbg >= 4 ) Serial.println( "| (XpAck2)" + inVal );

	int inA, inB, inC;
	while ( true ){

		inA = getStatusVal();
		inB = aps[ inA >> 4 ];

		//Serial.println("| A B C " + String( inA, HEX) + " " + String( inB, HEX) + " " + String( inC, HEX) );          
		if ( inB != 0x08 ){

			inC = getStatusVal();

			if ( inC == inA ){
				if ( dbg >= 4 ) Serial.println( "| XpAck2 returning!" );
				return( inC );
			}

		}

		//delayMicroseconds(MSD); 

	}

}

// See XP Ack 1 description
byte XpAck2_N( String inVal ){

	if ( dbg >= 4 ) Serial.println( "| (XpAck2_N) iv " + inVal );

	int inA, inB, inC;
	inA = getStatusVal();
	inB = aps[ inA >> 4 ];

getit:
	if ( ( inB & 0x08 ) != 0 ){

		//delayMicroseconds(MSD);
		goto getit;

	}

	inC = getStatusVal();
	if ( inC != inA ){

		//delayMicroseconds(MSD);
		goto getit;

	}

	if ( dbg >= 4 ) Serial.println( "| XpAck2_N returning!" );
	return inC;

}
