#ifndef HAS_INCLUDED_GPS_H
#define HAS_INCLUDED_GPS_H

#include <stdlib.h>
#include <stdio.h>
#define _CRT_SECURE_CPP_OVERLOAD_STANDARD_NAMES 1
#include <string.h>
#include "serial.h"
#include <ctype.h>
#include <math.h>

#include <nmea.h>
#include <nmea/gpgll.h>
#include <nmea/gngga.h>
#include <nmea/gpgga.h>

#define degToRad(angleInDegrees) ((angleInDegrees) * M_PI / 180.0)
#define radToDeg(angleInRadians) ((angleInRadians) * 180.0 / M_PI)


#define SEMI_MAJOR_AXIS 6378137.0
#define SEMI_MINOR_AXIS 6356752.314245

int strlen_safe(const char* str, size_t maxsize, size_t* result) {
    if (!str || !result) return 1;
    *result = 0;
    while (*str && maxsize--) {
        (*result)++;
        str++;
    }
    return (maxsize == 0 && *str) ? 1 : 0;
}

typedef struct {
  double latitude_degrees;
  double longitude_degrees;
  double altitude;
  char altitude_valid;
  struct timespec time;
} raw_gnss_t;

nmea_s* read_data(const int port) {
  char* raw_sentence = read_serial_port(port);
  int sentence_size;
  int err = strlen_safe(raw_sentence, 150, &sentence_size);
  // printf("Err: %d\n", err);
  if(raw_sentence == NULL || err != 0 || sentence_size == 0) {
    return NULL;
  }
  // printf("Err: %d, Sentence: %s, Len: %d\n", err, raw_sentence, sentence_size);

  int is_valid;
  do {
    char* last_char = raw_sentence + strlen(raw_sentence) - 1;
    is_valid = isalnum(*last_char) || *last_char == ','; 
    if(!is_valid) {
      *last_char = '\0';
    }
  } while(!is_valid && strlen(raw_sentence));

  if(!is_valid) {
    return NULL;
  }

  char sentence[NMEA_MAX_LENGTH + 3];
  memcpy(sentence, raw_sentence, strlen(raw_sentence));

  sentence[strlen(raw_sentence)] = '\r';
  sentence[strlen(raw_sentence) + 1] = '\n';
  sentence[strlen(raw_sentence) + 2] = '\0';

  int length;
  err = strlen_safe(sentence, 150, &length);
  if(err != 0) return NULL;

  nmea_s* result;
  result = nmea_parse(sentence, strlen(sentence), 0);

  // free(raw_sentence);
  return result;
}

int read_nav_data(const int port, raw_gnss_t* result) {

  nmea_s* data = read_data(port);
  if(data == NULL) {
    return -1;
  }

  double lat_deg, lat_min, lon_deg, lon_min, alt;
  char lon_card, lat_card;

  nmea_gngga_s* gngga;
  nmea_gpgga_s* gpgga;
  nmea_gpgll_s* gpgll;

  struct timespec time;

  switch(data->type){
    case NMEA_GNGGA:
    case NMEA_GPGGA:
      gpgga = (nmea_gpgga_s *) data;
      if(gpgga->position_fix <= 0) {
	      return -1;
      }
      lon_deg = gpgga->longitude.degrees;
      lon_min = gpgga->longitude.minutes;
      lon_card = gpgga->longitude.cardinal;

      lat_deg = gpgga->latitude.degrees;
      lat_min = gpgga->latitude.minutes;
      lat_card = gpgga->latitude.cardinal;

      alt = gpgga->altitude;
      time = gpgga->time;
      break;
    case NMEA_GPGLL:
      return -1;
      // printf("GPGLL- ");
      gpgll = (nmea_gpgll_s*) data;
      lon_deg = gpgll->longitude.degrees;
      lon_min = gpgll->longitude.minutes;
      lon_card = gpgll->longitude.cardinal;

      lat_deg = gpgll->latitude.degrees;
      lat_min = gpgll->latitude.minutes;
      lat_card = gpgll->latitude.cardinal;
      time = gpgll->time;
      break;
    default:
      return -1;
  }

  // printf("======\nLat: [%f, %f, %c]\nLon: [%f,%f,%c]\nAlt:%f\n======\n",
	// 	  lat_deg, lat_min, lat_card,
	// 	  lon_deg, lon_min, lon_card,
	// 	  alt);

  double lon_dec = lon_deg + (lon_min / 60.0);
  if (lon_card == 'W') {
    lon_dec *= -1;
  }

  double lat_dec = lat_deg + (lat_min / 60.0);
  if (lat_card == 'S') {
    lat_dec *= -1;  
  }


  result->altitude_valid = 1; //altitude_valid;
  result->longitude_degrees = lon_dec;
  result->latitude_degrees = lat_dec;
  result->altitude = alt;

  return 1;
}


double computeN(double lat) {
    double eSq = 1 - (SEMI_MINOR_AXIS * SEMI_MINOR_AXIS) / (SEMI_MAJOR_AXIS * SEMI_MAJOR_AXIS);
    return SEMI_MAJOR_AXIS / sqrt(1 - eSq * pow(sin(lat), 2));
}

void compute_ecef(double lat, double lon, double h, double result[3]) {
    double eSq = 1 - (SEMI_MINOR_AXIS * SEMI_MINOR_AXIS) / (SEMI_MAJOR_AXIS * SEMI_MAJOR_AXIS);

    // Convert lat and lon from degrees to radians
    lat = lat * M_PI / 180.0;
    lon = lon * M_PI / 180.0;

    double N = computeN(lat);

    // Calculate ECEF coordinates
    result[0] = (N + h) * cos(lat) * cos(lon); // X in meters
    result[1] = (N + h) * cos(lat) * sin(lon); // Y in meters
    result[2] = ((1 - eSq) * N + h) * sin(lat); // Z in meters
}

void compute_reference(const double lat_ref, 
                       const double lon_ref,
                       const double alt_ref,
                       double r_ref[9], 
                       double p_ref[3])
{
  const double p = lat_ref;
  const double l = lon_ref;

  const double R00 = -sin(p)*cos(l);
  const double R01 = -sin(p)*sin(l);
  const double R02 = cos(p);

  const double R10 = -sin(l);
  const double R11 = cos(l);
  const double R12 = 0;

  const double R20 = -cos(p)*cos(l);
  const double R21 = -cos(p)*sin(l);
  const double R22 = -sin(p);

  r_ref[0] = R00;
  r_ref[1] = R01;
  r_ref[2] = R02;
  r_ref[3] = R10;
  r_ref[4] = R11;
  r_ref[5] = R12;
  r_ref[6] = R20;
  r_ref[7] = R21;
  r_ref[8] = R22;
  // printf("here!");

  compute_ecef(lat_ref, lon_ref, alt_ref, p_ref);
}

int convert_data_to_ned(const raw_gnss_t data, 
                        const double R_ref[9],
                        const double p_ref[9],
                        double result[3])
{
  double lat_rad = degToRad(data.latitude_degrees);
  double lon_rad = degToRad(data.longitude_degrees);


  const double R00 = R_ref[0];
  const double R01 = R_ref[1];
  const double R02 = R_ref[2];

  const double R10 = R_ref[3];
  const double R11 = R_ref[4];
  const double R12 = R_ref[5];

  const double R20 = R_ref[6];
  const double R21 = R_ref[7];
  const double R22 = R_ref[8];


  double p_curr[3];
  compute_ecef(data.latitude_degrees, data.longitude_degrees, data.altitude, p_curr);

  // Compute p_diff = p_curr - p_ref
  double p_diff[3];
  p_diff[0] = p_curr[0] - p_ref[0];
  p_diff[1] = p_curr[1] - p_ref[1];
  p_diff[2] = p_curr[2] - p_ref[2];

  result[0] = R00 * p_diff[0] + R01 * p_diff[1] + R02 * p_diff[2];
  result[1] = R10 * p_diff[0] + R11 * p_diff[1] + R12 * p_diff[2];
  result[2] = R20 * p_diff[0] + R21 * p_diff[1] + R22 * p_diff[2];

  return 1;
}

int read_and_convert_data(const int port, 
                          const double R_ref[9],
                          const double p_ref[3],
                          double result[3])
{

  raw_gnss_t data;
  while(read_nav_data(port, &data) <= 0){
    continue;
  }
  // printf("Raw Data: [%f, %f, %f]\n", data.latitude_degrees, data.longitude_degrees, data.altitude);

  if(convert_data_to_ned(data, R_ref, p_ref, result) <= 0) {
    return -1;
  }

  return 1;
}

#endif

