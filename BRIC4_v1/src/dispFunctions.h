/*
 * dispFunctions.h
 *
 * Created: 8/22/2020 5:56:37 PM
 *  Author: Kris Fausnight
 */ 


#ifndef DISPFUNCTIONS_H_
#define DISPFUNCTIONS_H_

#include <asf.h>

void print_data_screen(void);
void print_Buff_to_Box(char *, uint32_t, uint8_t, uint8_t, uint8_t, uint8_t);
uint8_t getCursor(enum INPUT, uint8_t, uint8_t);
void draw_BLE_symbol(uint8_t, uint8_t);
float getDispX(float [3], uint8_t , uint8_t, bool);
void drawSoftKeys(const char *, const char *, const char *, const char *);
void draw2LineSoftKey(char *, char *, uint8_t);
void disp_report(uint8_t);
void genTimestampString(char  *, struct TIME *, uint8_t );
uint16_t incDecData(uint16_t, int8_t, uint16_t, uint16_t);

#endif /* DISPFUNCTIONS_H_ */