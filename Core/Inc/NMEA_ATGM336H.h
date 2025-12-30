//
// Created by gleam on 2025/11/27.
//

#ifndef GPS_ATGM336H_NMEA_ATGM336H_H
#define GPS_ATGM336H_NMEA_ATGM336H_H

#include "main.h"

// GPS数据结构体
typedef struct {
    char latitude[16];    // 纬度
    char longitude[16];   // 经度
    char ns_indicator;    // 南北半球指示器 (N/S)
    char ew_indicator;    // 东西半球指示器 (E/W)
    char status;          // 状态 (A=有效, V=无效)
    uint8_t is_valid;      // 数据是否有效
} GPS_Data_t;

// 函数声明
void GPS_Parser_Init(void);
void GPS_Parse_NMEA(const char* nmea_data);
void GPS_Get_Data(GPS_Data_t* gps_data);
uint8_t GPS_Check_Checksum(const char* nmea_data);
#endif //GPS_ATGM336H_NMEA_ATGM336H_H