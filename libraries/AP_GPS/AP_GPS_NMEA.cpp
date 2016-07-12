// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//
// NMEA parser, adapted by Michael Smith from TinyGPS v9:
//
// TinyGPS - a small GPS library for Arduino providing basic NMEA parsing
// Copyright (C) 2008-9 Mikal Hart
// All rights reserved.
//

/// @file	AP_GPS_NMEA.cpp
/// @brief	NMEA protocol parser
///
/// This is a lightweight NMEA parser, derived originally from the
/// TinyGPS parser by Mikal Hart.
///

#include <AP_Common.h>

#include <AP_Progmem.h>
#include <ctype.h>
#include <stdint.h>
#include <stdlib.h>

#include "AP_GPS_NMEA.h"

extern const AP_HAL::HAL& hal;

// SiRF init messages //////////////////////////////////////////////////////////
//
// Note that we will only see a SiRF in NMEA mode if we are explicitly configured
// for NMEA.  GPS_AUTO will try to set any SiRF unit to binary mode as part of
// the autodetection process.
//
#define SIRF_INIT_MSG \
        "$PSRF103,0,0,1,1*25\r\n"   /* GGA @ 1Hz */ \
        "$PSRF103,1,0,0,1*25\r\n"   /* GLL off */   \
        "$PSRF103,2,0,0,1*26\r\n"   /* GSA off */   \
        "$PSRF103,3,0,0,1*27\r\n"   /* GSV off */   \
        "$PSRF103,4,0,1,1*20\r\n"   /* RMC off */   \
        "$PSRF103,5,0,1,1*20\r\n"   /* VTG @ 1Hz */ \
        "$PSRF103,6,0,0,1*22\r\n"   /* MSS off */   \
        "$PSRF103,8,0,0,1*2C\r\n"   /* ZDA off */   \
        "$PSRF151,1*3F\r\n"         /* WAAS on (not always supported) */ \
        "$PSRF106,21*0F\r\n"        /* datum = WGS84 */

// MediaTek init messages //////////////////////////////////////////////////////
//
// Note that we may see a MediaTek in NMEA mode if we are connected to a non-DIYDrones
// MediaTek-based GPS.
//
#define MTK_INIT_MSG \
    "$PMTK314,0,5,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*2D\r\n" /* RMC GGA & VTG once every fix */ \
    "$PMTK330,0*2E\r\n"                                 /* datum = WGS84 */ \
	"$PMTK397,0*23\r\n"									/* Set Nav Threshold (the minimum speed the GPS must be moving to update the position) to 0 m/s*/ \
    "$PMTK313,1*2E\r\n"                                 /* SBAS on */ \
    "$PMTK301,2*2E\r\n"                                 /* use SBAS data for DGPS */ \
    "$PMTK251,115200*1F\r\n"									\
    "$PMTK311,15*19\r\n"                                 /* 15 deg min elevation for globaltop new fw */ \
	"$PMTK300,100,0,0,0,0*2C\r\n"   /*  100ms 10hz */ \
    "$PMTK220,100*2F"


// ublox init messages /////////////////////////////////////////////////////////
//
// Note that we will only see a ublox in NMEA mode if we are explicitly configured
// for NMEA.  GPS_AUTO will try to set any ublox unit to binary mode as part of
// the autodetection process.
//
// We don't attempt to send $PUBX,41 as the unit must already be talking NMEA
// and we don't know the baudrate.
//
#define UBLOX_INIT_MSG \
	"$PUBX,41,1,0007,0002,115200,0*19\r\n"   /*  115200 nmea */ \
    "$PUBX,40,gga,0,1,0,0,0,0*7B\r\n"   /* GGA on at one per fix */ \
    "$PUBX,40,vtg,0,1,0,0,0,0*7F\r\n"   /* VTG on at one per fix */ \
    "$PUBX,40,rmc,0,0,0,0,0,0*67\r\n"   /* RMC off (XXX suppress other message types?) */ 

//    "\181\098\006\008\006\000\200\000\001\000\001\000\222\106\181\098\006\008\000\000\014\048"   /*  only 5HZ */

#define UBLOX_INIT_RATE "\xB5\x62\x06\x08\x006\x00\xC8\x00\x01\x00\x01\x00\xDE\x6A\xB5\x62\x06\x08\x00\x00\x0E\x30"

//setDataRate[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, settingsArrayPointer[1], settingsArrayPointer[2], 0x01, 0x00, 0x01, 0x00, 0x00, 0x00

const prog_char AP_GPS_NMEA::_initialisation_blob[] PROGMEM = SIRF_INIT_MSG MTK_INIT_MSG UBLOX_INIT_MSG UBLOX_INIT_RATE;
//const prog_char AP_GPS_NMEA::_initialisation_blob[] PROGMEM = UBLOX_INIT_MSG;


// NMEA message identifiers ////////////////////////////////////////////////////
// GP GN ignore
const char AP_GPS_NMEA::_gprmc_string[] PROGMEM = "RMC"; 
const char AP_GPS_NMEA::_gpgga_string[] PROGMEM = "GGA";
const char AP_GPS_NMEA::_gpvtg_string[] PROGMEM = "VTG";

// Convenience macros //////////////////////////////////////////////////////////
//
#define DIGIT_TO_VAL(_x)        (_x - '0')

AP_GPS_NMEA::AP_GPS_NMEA(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port) :
    AP_GPS_Backend(_gps, _state, _port),
    _parity(0),
    _is_checksum_term(false),
    _sentence_type(0),
    _term_number(0),
    _term_offset(0),
    _gps_data_good(false)
{
    gps.send_blob_start(state.instance, _initialisation_blob, sizeof(_initialisation_blob));

//	PEsize =constrain((uint8_t) gps._filterpe, 3, LocHistorySize - 3);
//	if (gps._filterpe == 0)
//				PEsize = 0; //disable filter
//
//	kcnt = constrain((int32_t) gps._filterspd , 3 , LocHistorySize / 2 -1 );
}

bool AP_GPS_NMEA::read(void)
{
    int16_t numc;
    bool parsed = false;

    numc = port->available();
    while (numc--) {
        if (_decode(port->read())) {
            parsed = true;
        }
    }
    return parsed;
}

bool AP_GPS_NMEA::_decode(char c)
{
    bool valid_sentence = false;

    switch (c) {
    case ',': // term terminators
        _parity ^= c;
    case '\r':
    case '\n':
    case '*':
        if (_term_offset < sizeof(_term)) {
            _term[_term_offset] = 0;
            valid_sentence = _term_complete();
        }
        ++_term_number;
        _term_offset = 0;
        _is_checksum_term = c == '*';
        return valid_sentence;

    case '$': // sentence begin
        _term_number = _term_offset = 0;
        _parity = 0;
        _sentence_type = _GPS_SENTENCE_OTHER;
        _is_checksum_term = false;
        _gps_data_good = false;
        return valid_sentence;
    }

    // ordinary characters
    if (_term_offset < sizeof(_term) - 1)
        _term[_term_offset++] = c;
    if (!_is_checksum_term)
        _parity ^= c;

    return valid_sentence;
}

//
// internal utilities
//
int16_t AP_GPS_NMEA::_from_hex(char a)
{
    if (a >= 'A' && a <= 'F')
        return a - 'A' + 10;
    else if (a >= 'a' && a <= 'f')
        return a - 'a' + 10;
    else
        return a - '0';
}

uint32_t AP_GPS_NMEA::_parse_decimal_100()
{
    char *p = _term;
    uint32_t ret = 100UL * atol(p);
    while (isdigit(*p))
        ++p;
    if (*p == '.') {
        if (isdigit(p[1])) {
            ret += 10 * (p[1] - '0');
            if (isdigit(p[2]))
                ret += p[2] - '0';
        }
    }
    return ret;
}

/*
  parse a NMEA latitude/longitude degree value. The result is in degrees*1e7
 */
uint32_t AP_GPS_NMEA::_parse_degrees()
{
    char *p, *q;
    uint8_t deg = 0, min = 0;
    float frac_min = 0;
    int32_t ret = 0;

    // scan for decimal point or end of field
    for (p = _term; isdigit(*p); p++)
        ;
    q = _term;

    // convert degrees
    while ((p - q) > 2) {
        if (deg)
            deg *= 10;
        deg += DIGIT_TO_VAL(*q++);
    }

    // convert minutes
    while (p > q) {
        if (min)
            min *= 10;
        min += DIGIT_TO_VAL(*q++);
    }

    // convert fractional minutes
    if (*p == '.') {
        q = p + 1;
        float frac_scale = 0.1f;
        while (isdigit(*q)) {
            frac_min += (*q++ - '0') * frac_scale;
            frac_scale *= 0.1f;
        }
    }
    ret = (deg * (int32_t)10000000UL);
    ret += (min * (int32_t)10000000UL / 60);
    ret += (int32_t) (frac_min * (1.0e7f / 60.0f));
    return ret;
}

// Processes a just-completed term
// Returns true if new sentence has just passed checksum test and is validated
bool AP_GPS_NMEA::_term_complete()
{
    // handle the last term in a message
    if (_is_checksum_term) {
        uint8_t checksum = 16 * _from_hex(_term[0]) + _from_hex(_term[1]);
        if (checksum == _parity) {
            if (_gps_data_good) {
                switch (_sentence_type) {
                case _GPS_SENTENCE_GPRMC:
                    //time                        = _new_time;
                    //date                        = _new_date;
  
 
					
					state.ground_speed     = _new_speed*0.01f;
                    state.ground_course_cd = _new_course;
                    make_gps_time(_new_date, _new_time * 10);
                    state.last_gps_time_ms = hal.scheduler->millis();

					state.location.lat     = _new_latitude;
                    state.location.lng     = _new_longitude;
  	//				runEstmation(); 
   
					// To-Do: add support for proper reporting of 2D and 3D fix
                    state.status           = AP_GPS::GPS_OK_FIX_3D;
                    fill_3d_velocity();
                    break;
                case _GPS_SENTENCE_GPGGA:
                    state.location.alt  = _new_altitude;
     
					state.location.lat  = _new_latitude;
                    state.location.lng  = _new_longitude;

					state.num_sats      = _new_satellite_count;
                    state.hdop          = _new_hdop;

                    make_gps_time(_new_date, _new_time * 10);
		//			runEstmation();

					
					// To-Do: add support for proper reporting of 2D and 3D fix
                    //state.status        = AP_GPS::GPS_OK_FIX_3D;
                    break;
                case _GPS_SENTENCE_GPVTG:
                    state.ground_speed     = _new_speed*0.01f;
                    state.ground_course_cd = _new_course;
                    // VTG has no fix indicator, can't change fix status
                    break;
                }
            } else {
                switch (_sentence_type) {
                case _GPS_SENTENCE_GPRMC:
                case _GPS_SENTENCE_GPGGA:
                    // Only these sentences give us information about
                    // fix status.
                    state.status = AP_GPS::NO_FIX;
                }
            }
            // we got a good message
            return true;
        }
        // we got a bad message, ignore it
        return false;
    }

    // the first term determines the sentence type
    if (_term_number == 0) {
        if (!strcmp_P(_term+2, _gprmc_string)) {
            _sentence_type = _GPS_SENTENCE_GPRMC;
        } else if (!strcmp_P(_term+2, _gpgga_string)) {
            _sentence_type = _GPS_SENTENCE_GPGGA;
        } else if (!strcmp_P(_term+2, _gpvtg_string)) {
            _sentence_type = _GPS_SENTENCE_GPVTG;
            // VTG may not contain a data qualifier, presume the solution is good
            // unless it tells us otherwise.
            _gps_data_good = true;
        } else {
            _sentence_type = _GPS_SENTENCE_OTHER;
        }
        return false;
    }

    // 32 = RMC, 64 = GGA, 96 = VTG
    if (_sentence_type != _GPS_SENTENCE_OTHER && _term[0]) {
        switch (_sentence_type + _term_number) {
        // operational status
        //
        case _GPS_SENTENCE_GPRMC + 2: // validity (RMC)
            _gps_data_good = _term[0] == 'A';
            break;
        case _GPS_SENTENCE_GPGGA + 6: // Fix data (GGA)
            _gps_data_good = _term[0] > '0';
            break;
        case _GPS_SENTENCE_GPVTG + 9: // validity (VTG) (we may not see this field)
            _gps_data_good = _term[0] != 'N';
            break;
        case _GPS_SENTENCE_GPGGA + 7: // satellite count (GGA)
            _new_satellite_count = atol(_term);
            break;
        case _GPS_SENTENCE_GPGGA + 8: // HDOP (GGA)
            _new_hdop = _parse_decimal_100();
            break;

        // time and date
        //
        case _GPS_SENTENCE_GPRMC + 1: // Time (RMC)
        case _GPS_SENTENCE_GPGGA + 1: // Time (GGA)
            _new_time = _parse_decimal_100();
            break;
        case _GPS_SENTENCE_GPRMC + 9: // Date (GPRMC)
            _new_date = atol(_term);
            break;

        // location
        //
        case _GPS_SENTENCE_GPRMC + 3: // Latitude
        case _GPS_SENTENCE_GPGGA + 2:
            _new_latitude = _parse_degrees();
            break;
        case _GPS_SENTENCE_GPRMC + 4: // N/S
        case _GPS_SENTENCE_GPGGA + 3:
            if (_term[0] == 'S')
                _new_latitude = -_new_latitude;
            break;
        case _GPS_SENTENCE_GPRMC + 5: // Longitude
        case _GPS_SENTENCE_GPGGA + 4:
            _new_longitude = _parse_degrees();
            break;
        case _GPS_SENTENCE_GPRMC + 6: // E/W
        case _GPS_SENTENCE_GPGGA + 5:
            if (_term[0] == 'W')
                _new_longitude = -_new_longitude;
            break;
        case _GPS_SENTENCE_GPGGA + 9: // Altitude (GPGGA)
            _new_altitude = _parse_decimal_100();
            break;

        // course and speed
        //
        case _GPS_SENTENCE_GPRMC + 7: // Speed (GPRMC)
        case _GPS_SENTENCE_GPVTG + 5: // Speed (VTG)
            _new_speed = (_parse_decimal_100() * 514) / 1000;       // knots-> m/sec, approximiates * 0.514
            break;
        case _GPS_SENTENCE_GPRMC + 8: // Course (GPRMC)
        case _GPS_SENTENCE_GPVTG + 1: // Course (VTG)
            _new_course = _parse_decimal_100();
            break;
        }
    }

    return false;
}

#define hexdigit(x) ((x)>9?'A'+(x):'0'+(x))

/*
  detect a NMEA GPS. Adds one byte, and returns true if the stream
  matches a NMEA string
 */
bool
AP_GPS_NMEA::_detect(struct NMEA_detect_state &state, uint8_t data)
{
	switch (state.step) {
	case 0:
		state.ck = 0;
		if ('$' == data) {
			state.step++;
		}
		break;
	case 1:
		if ('*' == data) {
			state.step++;
		} else {
			state.ck ^= data;
		}
		break;
	case 2:
		if (hexdigit(state.ck>>4) == data) {
			state.step++;
		} else {
			state.step = 0;
		}
		break;
	case 3:
		if (hexdigit(state.ck&0xF) == data) {
			return true;
		}
		state.step = 0;
		break;
    }
    return false;
}




/*#if KPEMF


void AP_GPS_NMEA::runEstmation()
        {
         
                if (state.time_week_ms == last_history_time_week_ms  || state.time_week_ms == 0) //prevent put location marked same time but received both rmc  gga
                   {
                   return;
                   }
               

				if (PEsize==0) //disable filter flag
				{
					state.location.lat = _new_latitude;
					state.location.lng = _new_longitude;
					state.status        = AP_GPS::GPS_OK_FIX_3D;
					return;
				}
          
                LocHistoryPush(_new_latitude , _new_longitude);
                LocHistory[0].DateTime = state.time_week_ms;
                last_history_time_week_ms = LocHistory[0].DateTime;
                
                if (!LocHistoryIsFull()) { 
					
				   // copy raw to estmate ?
				  state.location.lat = _new_latitude;
				  state.location.lng = _new_longitude;
                  //state.status        = AP_GPS::GPS_OK_FIX_3D;
					return; } //fill buffer before estmation
                
                CalcSpeedByHistory(); // calc avgspeedLat lng
                for (int j = 1; j <= PEsize; j++) { estItemHist(j); } //fill parallelEstmations arrays lat lng
                bubbleSortLat();
                state.location.lat = parallelEstmationslat[(PEsize + 1) / 2]; //median of PEsize values
                bubbleSortLng();
                state.location.lng = parallelEstmationslng[(PEsize + 1) / 2]; //median of PEsize values

 
                LocHistory[0].lat = state.location.lat; //replace actual raw valie with estmated  
                LocHistory[0].lng = state.location.lng;

				state.status        = AP_GPS::GPS_OK_FIX_3D;
                
                
                
                                
 
 
              //estmatedata[i].lat = estmatedata[i].lat + avgspeedLatErr; //acc compensation  in speed mode

        }



void
AP_GPS_NMEA::LocHistoryPush(    int32_t rawlat, int32_t rawlng)
        {

            //todo compare datetime. clear buffer if data is old. put only DATA with new DateTime


            for (int i = LocHistorySize - 1; i >= 1; i--)
            {
                LocHistory[i] = LocHistory[i - 1];
            }
            LocHistory[0].lat = rawlat;
            LocHistory[0].lng = rawlng;          
            if (LocHistoryLen < LocHistorySize) { LocHistoryLen += 1; }
        }

bool
AP_GPS_NMEA::LocHistoryIsFull()
        {
            return LocHistoryLen == LocHistorySize;
        }
        
void
AP_GPS_NMEA::CalcSpeedByHistory()
         // indian style code for kcnt =4 
         //                avgspeedLat = ((estmatedata[i - 1].lat + estmatedata[i - 2].lat + estmatedata[i - 3].lat + estmatedata[i - 4].lat) / 4.0f - (estmatedata[i - 5].lat + estmatedata[i - 6].lat + estmatedata[i - 7].lat + estmatedata[i - 8].lat) / 4.0f)
         //    ((estmatedata[i - 1].DateTime + estmatedata[i - 2].DateTime + estmatedata[i - 3].DateTime + estmatedata[i - 4].DateTime) / 4.0f - (estmatedata[i - 5].DateTime + estmatedata[i - 6].DateTime + estmatedata[i - 7].DateTime + estmatedata[i - 8].DateTime) / 4.0f);
        {
            if (!LocHistoryIsFull()) 
            {
                avgspeedLat = 0;
                return;
            }
//            const int32_t kcnt = 2; //count points in avg speed mode. 2 low latency with fast accelerations, 5 better avg 
            int32_t lat1 = 0; //avg position new point
            int32_t lat2 = 0; //avg position old point
            int32_t lng1 = 0; //avg position new point
            int32_t lng2 = 0; //avg position old point
            int32_t t1 = 0; //avg time new point
            int32_t t2 = 0; //avg time old point
            for (int k = 1; k <= kcnt; k++)
            {
                lat1 += LocHistory[k].lat / kcnt ;
                lat2 += LocHistory[ k + kcnt].lat / kcnt;

                lng1 += LocHistory[k].lng / kcnt;
                lng2 += LocHistory[ k + kcnt].lng / kcnt;

                t1 += LocHistory[k].DateTime / kcnt / 100L;
                t2 += LocHistory[k + kcnt].DateTime / kcnt / 100L;
            }

            if (t1 - t2 != 0)
            {
                  avgspeedLat = (lat1 - lat2)  / (t1 - t2) ;
               avgspeedLng = (lng1 - lng2) /  (t1 - t2) ;
              //   Serial.print("lat1 ");  Serial.print(lat1); Serial.print(" lat2 ");  Serial.print(lat2); Serial.print(" (lat1 - lat2) ");  Serial.println((lat1 - lat2)); Serial.print(" (t1 - t2) ");  Serial.println((t1 - t2));//todo remove

            }
            else
             {   avgspeedLat = 0; 
                 avgspeedLng = 0;
                // Serial.println("zerotime!"); //todo remove
             }    
  
  }


void
AP_GPS_NMEA::estItemHist( int level)  //fill parallelEstmations[level]
        {
            //           double avgdatalat      =  (rawdata[dataindex -0].lat + rawdata[dataindex -1].lat) / 2.0; 
            //           double avgdatatime  =  (rawdata[dataindex -0].p1 + rawdata[dataindex -1].p1) / 2.0;  

            int32_t avgdatalat = 0; // avg latitude
            int32_t avgdatalng = 0;
            int32_t avgdatatime = 0; //Latency of estmation
            for (int p = 0; p <= level; p++)
            {
                avgdatalat = avgdatalat + LocHistory[p].lat / (int32_t) (level + 1.0); // level=1  (d[0] + d[1]) /2
                avgdatalng = avgdatalng + LocHistory[p].lng / (int32_t) (level + 1.0);
                avgdatatime = avgdatatime + LocHistory[p].DateTime / (int32_t)(level + 1.0);
            }


            uint32_t addDistancelat = avgspeedLat * ((state.time_week_ms - avgdatatime) / 100L) ;  //correct lanency of filer += speed *time to actual
            uint32_t addDistancelng = avgspeedLng * ((state.time_week_ms - avgdatatime) / 100L);  
            
           parallelEstmationslat[level] = avgdatalat + addDistancelat; 
           parallelEstmationslng[level] = avgdatalng + addDistancelng;          
        }
        
void
AP_GPS_NMEA::bubbleSortLat() {
              int i = 0;
              int o= 0;
              uint32_t swapper = 0.0;

              for (o = 0; o < PEsize; o++)
              {  // outer loop
                  for (i = o; i < (PEsize - 1); i++)
                  {  // inner loop
                  if( parallelEstmationslat[i] > parallelEstmationslat[i+1] ) {   // out of order?
                    // swap them:
                    swapper = parallelEstmationslat[i];
                    parallelEstmationslat [i] = parallelEstmationslat[i+1];
                    parallelEstmationslat[i+1] = swapper;
                  }
                }
              }
            }    
 

void
AP_GPS_NMEA::bubbleSortLng() {
              int i = 0;
              int o= 0;
              uint32_t swapper = 0.0;

              for (o = 0; o < PEsize; o++)
              {  // outer loop
                  for (i = o; i < (PEsize - 1); i++)
                  {  // inner loop
                  if( parallelEstmationslng[i] > parallelEstmationslng[i+1] ) {   // out of order?
                    // swap them:
                    swapper = parallelEstmationslng[i];
                    parallelEstmationslng [i] = parallelEstmationslng[i+1];
                    parallelEstmationslng[i+1] = swapper;
                  }
                }
              }
            }


*/