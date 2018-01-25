/*
  UBLOX8.cpp
  Copyright (C) 2017 Curtis L. Olson curtolson@flightgear.org
*/

#include "Arduino.h"
#include "UBLOX8.h"

/* uBlox object, input the serial bus */
UBLOX8::UBLOX8(HardwareSerial* port){
    _port = port; // serial port
}

/* starts the serial communication */
void UBLOX8::begin(int baud){
    // begin the serial port for uBlox
    _port->begin(baud);
}


bool UBLOX8::read_ublox8() {
    static int state = 0;
    static int msg_class = 0, msg_id = 0;
    static int length_lo = 0, length_hi = 0, payload_length = 0;
    static int counter = 0;
    static uint8_t cksum_A = 0, cksum_B = 0, cksum_lo = 0, cksum_hi = 0;
    int len;
    uint8_t input;
    static uint8_t payload[500];

    // printf("read ublox8, entry state = %d\n", state);

    bool new_data = false;

    if ( state == 0 ) {
	counter = 0;
	cksum_A = cksum_B = 0;
	while ( _port->available() ) {
	    input = _port->read();
	    // fprintf( stderr, "state0: len = %d val = %2X\n", len, input );
            if ( input == 0xB5 ) {\
                state = 1;
                break;
            }
	}
    }
    if ( state == 1 ) {
        if ( _port->available() >= 1 ) {
            input = _port->read();
	    if ( input == 0x62 ) {
		// fprintf( stderr, "read 0x62\n");
		state = 2;
	    } else if ( input == 0xB5 ) {
		// fprintf( stderr, "read 0xB5\n");
	    } else {
                // oops
		state = 0;
	    }
	}
    }
    if ( state == 2 ) {
        if ( _port->available() >= 1 ) {
	    msg_class = _port->read();
	    cksum_A += msg_class;
	    cksum_B += cksum_A;
	    // fprintf( stderr, "msg class = %d\n", msg_class );
	    state = 3;
	}
    }
    if ( state == 3 ) {
        if ( _port->available() >= 1 ) {
	    msg_id = _port->read();
	    cksum_A += msg_id;
	    cksum_B += cksum_A;
            // Serial.print("msg id = "); Serial.println(msg_id);
	    state = 4;
	}
    }
    if ( state == 4 ) {
        if ( _port->available() >= 1 ) {
	    length_lo = _port->read();
	    cksum_A += length_lo;
	    cksum_B += cksum_A;
	    state = 5;
	}
    }
    if ( state == 5 ) {
        if ( _port->available() >= 1 ) {
	    length_hi = _port->read();
	    cksum_A += length_hi;
	    cksum_B += cksum_A;
	    payload_length = length_hi*256 + length_lo;
	    // Serial.print("payload len = "); Serial.println(payload_length);
	    if ( payload_length > 400 ) {
		state = 0;
	    } else {
		state = 6;
	    }
	}
    }
    if ( state == 6 ) {
	while ( _port->available() ) {
            uint8_t val = _port->read();
	    payload[counter++] = val;
	    //fprintf( stderr, "%02X ", input );
	    cksum_A += val;
	    cksum_B += cksum_A;
	    if ( counter >= payload_length ) {
		break;
	    }
	}

	if ( counter >= payload_length ) {
	    state = 7;
	    //fprintf( stderr, "\n" );
	}
    }
    if ( state == 7 ) {
        if ( _port->available() ) {
	    cksum_lo = _port->read();
	    state = 8;
	}
    }
    if ( state == 8 ) {
        if ( _port->available() ) {
	    cksum_hi = _port->read();
	    if ( cksum_A == cksum_lo && cksum_B == cksum_hi ) {
                // Serial.print("gps checksum passes. class: "); Serial.print(msg_class);
                // Serial.print(" id = "); Serial.println(msg_id);
		new_data = parse_msg( msg_class, msg_id,
                                      payload_length, payload );
	    } else {
                // Serial.println("gps checksum fail");
                // printf("checksum failed %d %d (computed) != %d %d (message)\n",
                //        cksum_A, cksum_B, cksum_lo, cksum_hi );
	    }
	    // this is the end of a record, reset state to 0 to start
	    // looking for next record
	    state = 0;
	}
    }

    return new_data;
}


bool UBLOX8::parse_msg( uint8_t msg_class, uint8_t msg_id,
                       uint16_t payload_length, uint8_t *payload )
{
    bool new_data = false;
    static bool set_system_time = false;

    if ( msg_class == 0x01 && msg_id == 0x02 ) {
	// NAV-POSLLH: Please refer to the ublox6 driver (here or in the
	// code history) for a nav-posllh parser
    } else if ( msg_class == 0x01 && msg_id == 0x06 ) {
	// NAV-SOL: Please refer to the ublox6 driver (here or in the
	// code history) for a nav-sol parser that transforms eced
	// pos/vel to lla pos/ned vel.
    } else if ( msg_class == 0x01 && msg_id == 0x07 ) {
	// NAV-PVT
	uint8_t *p = payload;
	data.iTOW = *((uint32_t *)p+0);
	data.year = *((uint16_t *)(p+4));
	data.month = p[6];
	data.day = p[7];
	data.hour = p[8];
	data.min = p[9];
	data.sec = p[10];
	data.valid = p[11];
	data.tAcc = *((uint32_t *)(p+12));
	data.nano = *((int32_t *)(p+16));
	data.fixType = p[20];
	data.flags = p[21];
	data.numSV = p[23];
	data.lon = *((int32_t *)(p+24));
	data.lat = *((int32_t *)(p+28));
	data.height = *((int32_t *)(p+32));
	data.hMSL = *((int32_t *)(p+36));
	data.hAcc = *((uint32_t *)(p+40));
	data.vAcc = *((uint32_t *)(p+44));
	data.velN = *((int32_t *)(p+48));
	data.velE = *((int32_t *)(p+52));
	data.velD = *((int32_t *)(p+56));
	data.gSpeed = *((uint32_t *)(p+60));
	data.heading = *((int32_t *)(p+64));
	data.sAcc = *((uint32_t *)(p+68));
	data.headingAcc = *((uint32_t *)(p+72));
	data.pDOP = *((uint16_t *)(p+76));
        new_data = true;
        //Serial.print("fixType: ");
        //Serial.println(data.fixType);
    } else if ( msg_class == 0x01 && msg_id == 0x12 ) {
	// NAV-VELNED: Please refer to the ublox6 driver (here or in the
	// code history) for a nav-velned parser
    } else if ( msg_class == 0x01 && msg_id == 0x21 ) {
	// NAV-TIMEUTC: Please refer to the ublox6 driver (here or in the
	// code history) for a nav-timeutc parser
    } else if ( msg_class == 0x01 && msg_id == 0x30 ) {
	// NAV-SVINFO (partial parse)
	uint8_t *p = payload;
	// uint32_t iTOW = *((uint32_t *)(p+0));
	uint8_t numCh = p[4];
	// uint8_t globalFlags = p[5];
	int satUsed = 0;
	for ( int i = 0; i < numCh; i++ ) {
	    // uint8_t satid = p[9 + 12*i];
	    // uint8_t flags = p[10 + 12*i];
	    uint8_t quality = p[11 + 12*i];
	    // printf(" chn=%d satid=%d flags=%d quality=%d\n", i, satid, flags, quality);
	    if ( quality > 3 ) {
		satUsed++;
	    }
	}
    } else {
	if ( false ) {
            Serial.print("ublox8 msg class: ");
            Serial.print(msg_class);
            Serial.print(" msg id: ");
            Serial.print(msg_id);
	}
    }

    return new_data;
}
