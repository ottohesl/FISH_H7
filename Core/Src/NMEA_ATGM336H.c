#include "NMEA_ATGM336H.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
// 全局GPS数据结构
static GPS_Data_t g_gps_data = {0};

// 初始化GPS解析器
void GPS_Parser_Init(void)
{
    memset(&g_gps_data, 0, sizeof(GPS_Data_t));
    g_gps_data.is_valid = 0;
    g_gps_data.status = 'V';  // 默认无效
}

uint8_t GPS_Check_Checksum(const char* nmea_data)
{
    if (nmea_data == NULL || strlen(nmea_data) < 7) {
        return 0;
    }

    if (nmea_data[0] != '$') {
        return 0;
    }

    uint8_t calculated_checksum = 0;
    uint8_t received_checksum = 0;
    const char* ptr = nmea_data + 1;

    // 计算校验和
    while (*ptr && *ptr != '*' && (ptr - nmea_data) < 100) {
        calculated_checksum ^= *ptr;
        ptr++;
    }

    // 获取接收到的校验和
    if (*ptr == '*' && *(ptr+1) && *(ptr+2)) {
        char hex[3] = {*(ptr+1), *(ptr+2), '\0'};
        received_checksum = (uint8_t)strtol(hex, NULL, 16);
        return (calculated_checksum == received_checksum);
    }

    return 0;
}
// 解析GGA语句
void GPS_Parse_GGA(const char* nmea_data)
{
    char copy[256];
    char* tokens[20];
    uint8_t token_count = 0;

    strncpy(copy, nmea_data, sizeof(copy)-1);
    copy[sizeof(copy)-1] = '\0';

    // 分割字符串
    char* token = strtok(copy, ",");
    while (token != NULL && token_count < 20) {
        tokens[token_count++] = token;
        token = strtok(NULL, ",");
    }
    // 只有满足条件时才设置有效
    if (token_count >= 10) {
        // 纬度
        if (strlen(tokens[2]) > 0) {
            strncpy(g_gps_data.latitude, tokens[2], sizeof(g_gps_data.latitude)-1);
            g_gps_data.latitude[sizeof(g_gps_data.latitude)-1] = '\0';
            g_gps_data.ns_indicator = (tokens[3][0] == 'N' || tokens[3][0] == 'S') ? tokens[3][0] : 'N';
        }

        // 经度
        if (strlen(tokens[4]) > 0) {
            strncpy(g_gps_data.longitude, tokens[4], sizeof(g_gps_data.longitude)-1);
            g_gps_data.longitude[sizeof(g_gps_data.longitude)-1] = '\0';
            g_gps_data.ew_indicator = (tokens[5][0] == 'E' || tokens[5][0] == 'W') ? tokens[5][0] : 'E';
        }

        // 状态 (GGA语句中第6个字段是定位状态)
        g_gps_data.status = (token_count > 6 && tokens[6][0] == '1') ? 'A' : 'V';

        // 只有状态为A时才认为数据有效
        g_gps_data.is_valid = (g_gps_data.status == 'A');

        printf("  GGA Latitude: %s %c\n", g_gps_data.latitude, g_gps_data.ns_indicator);
        printf("  GGA Longitude: %s %c\n", g_gps_data.longitude, g_gps_data.ew_indicator);
    } else {
        g_gps_data.is_valid = 0;

    }
}
// 解析RMC语句
void GPS_Parse_RMC(const char* nmea_data)
{

    g_gps_data.is_valid = 0;
    char copy[256];
    char* tokens[20];
    uint8_t token_count = 0;

    strncpy(copy, nmea_data, sizeof(copy)-1);
    copy[sizeof(copy)-1] = '\0';

    char* token = strtok(copy, ",");
    while (token != NULL && token_count < 20) {
        tokens[token_count++] = token;
        token = strtok(NULL, ",");
    }



    if (token_count >= 12) {  // RMC语句通常有12个字段
       // printf("RMC has enough tokens\n");

        // 状态 (RMC语句中第2个字段是状态)
        g_gps_data.status = tokens[2][0];

        // 纬度 (第3个字段是纬度，第4个是N/S指示器)
        if (strlen(tokens[3]) > 0) {
            strncpy(g_gps_data.latitude, tokens[3], sizeof(g_gps_data.latitude)-1);
            g_gps_data.latitude[sizeof(g_gps_data.latitude)-1] = '\0';
            g_gps_data.ns_indicator = tokens[4][0];
        }

        // 经度 (第5个字段是经度，第6个是E/W指示器)
        if (strlen(tokens[5]) > 0) {
            strncpy(g_gps_data.longitude, tokens[5], sizeof(g_gps_data.longitude)-1);
            g_gps_data.longitude[sizeof(g_gps_data.longitude)-1] = '\0';
            g_gps_data.ew_indicator = tokens[6][0];
        }

        g_gps_data.is_valid = (g_gps_data.status == 'A');

        // 打印RMC解析结果
        printf("  RMC Latitude: %s %c\n", g_gps_data.latitude, g_gps_data.ns_indicator);
        printf("  RMC Longitude: %s %c\n", g_gps_data.longitude, g_gps_data.ew_indicator);

    } else {
        g_gps_data.is_valid = 0;

    }
}
void GPS_Parse_NMEA(const char* nmea_data)
{
    g_gps_data.is_valid = 0;

    if (nmea_data == NULL || strlen(nmea_data) < 7 || nmea_data[0] != '$') {
        return;
    }

    if (!GPS_Check_Checksum(nmea_data)) {
        return;
    }

    if (strstr(nmea_data, "$GPGGA") || strstr(nmea_data, "$GNGGA")) {
        GPS_Parse_GGA(nmea_data);
    }
    else if (strstr(nmea_data, "$GPRMC") || strstr(nmea_data, "$GNRMC")) {
        GPS_Parse_RMC(nmea_data);
    }
}

// 获取GPS数据
void GPS_Get_Data(GPS_Data_t* gps_data)
{
    if (gps_data != NULL) {
        memcpy(gps_data, &g_gps_data, sizeof(GPS_Data_t));
    }
}