/*
 * dispFunctions.c
 *
 * Created: 8/22/2020 5:56:03 PM
 *  Author: Kris Fausnight
 */ 
#include <dispFunctions.h>

//  For Y-axis Indexes for soft-Keys
const uint8_t yLine[] = {1, 16, 32, 48, 64};

#define num_lines 5
#define x1 0  //  Starting X position for Ref data
#define x2 28 //  Starting X position for Distance data
#define x3 63 //  Starting X position for Azimuth data
#define x4 98 //  Starting X position for Inclination data
#define y1 0  //  Starting Y position for header
#define y2 10 //  Starting Y position for data


void genTimestampString(char  *timeString, struct TIME *timePtr, uint8_t style){
	switch (style){
		case 1:
			sprintf(timeString,"%04d.%02d.%02d@%02d:%02d:%02d",
				timePtr->year,timePtr->month, timePtr->day,
				timePtr->hours, timePtr->minutes, timePtr->seconds);
			break;
		case 2:
			sprintf(timeString,"%04d%02d%02d_%02d%02d%02d",
				timePtr->year,timePtr->month, timePtr->day,
				timePtr->hours, timePtr->minutes, timePtr->seconds);
			break;
		case 3:
			sprintf(timeString,"%04d%02d%02d",
				timePtr->year,timePtr->month, timePtr->day);
			break;
		
	}
	
	
}

void print_line(uint8_t y_line, struct MEASUREMENT *measPtr){
	if((measPtr->refIndex==0)&&(measPtr->meas_type!=measQuick)){
		//  Don't print if there is no reference index
		return;
	}
	
	//Azimuth
	sprintf(display_str, "%.1f", measPtr->azimuth);
	glcd_draw_string_xy(x3, y_line, display_str);
	
	//Inclination
	sprintf(display_str, "%.1f", measPtr->inclination);
	glcd_draw_string_xy(x4, y_line, display_str);
	
	if(measPtr->meas_type!=measQuick){
		//  Reference number
		sprintf(display_str, "%d", measPtr->refIndex);//reference
		glcd_draw_string_xy(x1, y_line, display_str);
		
		//  Distance
		if (options.current_unit_dist==meters){
			sprintf(display_str, "%.2f", measPtr->distMeters);//distance
		}else{
			sprintf(display_str, "%.1f", MT2FT*(measPtr->distMeters));//distance
		}
		glcd_draw_string_xy(x2, y_line, display_str);
		
		
		//  Add Error message if necessary
		if (measPtr->errCode[0]!=0){
			glcd_draw_string_xy(x1+18, y_line, "E");
		}
		
	}
		

	
}



void print_data_screen(void){
	static bool percFlipper;
	uint8_t battery_level;
	

	glcd_clear_buffer();
	
	//Print Data Headers
	sprintf(display_str,"REF");
	glcd_draw_string_xy(x1,y1, display_str);	
	sprintf(display_str,"DIST");
	glcd_draw_string_xy(x2,y1,display_str);	
	sprintf(display_str,"AZM");
	glcd_draw_string_xy(x3, y1, display_str);
	glcd_draw_circle(x3+21, y1+2, 2, BLACK);
	sprintf(display_str,"INCL");
	glcd_draw_string_xy(x4, y1, display_str);
	glcd_draw_circle(x4+26, y1+2, 2, BLACK);

	//Print Grid Lines
	glcd_draw_line(0, y1+8, 128, y1+8, BLACK);
	glcd_draw_line(0, y2+8, 128, y2+8, BLACK);
	glcd_draw_line(x2-2, 0, x2-2, 53, BLACK);
	glcd_draw_line(x3-2, 0, x3-2, 53, BLACK);
	glcd_draw_line(x4-2, 0, x4-2, 53, BLACK);
	
	
	
	//Print Data
	uint16_t line;
	uint16_t y_line;
	uint32_t dispInd;
	//  Temporary reading stored at buffer index		
	dispInd = measBufInd;
	if (!isLaserOn()){
		//  If not displaying temporary reading, move back 1 to first measurement
		circBuffDec(&dispInd, N_MEASBUF);	
 	}	
	//  Print data in buffer
	for (line=0;line<num_lines;line++){
		if(line==0){
			y_line = y2;
		}else{			
			y_line = line*9+1+y2;
		}
		print_line(y_line, &measBuf[dispInd]);
		circBuffDec(&dispInd, N_MEASBUF);	
	}//  Loop for each line
	
	//  Add extra information on bottom	
	
	//  Temperature
	if (options.current_unit_temp == celsius){
		sprintf(display_str,"T:%4.1fC", currentTempC);
	}else{
		sprintf(display_str,"T:%0.1fF", celsius2fahrenheit(currentTempC));
	}
	glcd_tiny_draw_string(81,7,display_str);
	
	//  Draw Current Time
	get_time();
	sprintf(display_str,"%02d:%02d:%02d", current_time.hours, current_time.minutes, current_time.seconds);
	glcd_tiny_draw_string(0,7,display_str);

	//  Draw Charge Status	
	isCharging = getChargerStatus();
	if (isCharging){
		percFlipper = !percFlipper; //  Toggle % sign 
		//  Draw box around battery
		glcd_draw_line(48, 64, 48, 54, BLACK);
		glcd_draw_line(48, 54, 79, 54, BLACK);
		glcd_draw_line(79, 64, 79, 54, BLACK);
	
	}else{
		percFlipper = true;
	}
	//  Format Battery String
	battery_level = getBatteryLevel();
	if (battery_level>=100){
		//  Only room for 3 digits, remove ":"
		sprintf(display_str,"B%02d", battery_level);
	}else{
		sprintf(display_str,"B:%02d",battery_level);
	}
	if (percFlipper){
		//  Blink % sign on/off
		strcat(display_str,"%");
	}
	glcd_tiny_draw_string(50,7,display_str);
	
	//  Draw Bluetooth Connection Status
	if(isBleConnected()){
		draw_BLE_symbol(123, 55);
	}
	
	
	
	
	
	glcd_write();
}


void draw_BLE_symbol(uint8_t horiz, uint8_t vert){
	//  vertical line
	glcd_draw_line(horiz+2, vert, horiz+2, vert+8, BLACK);
	//  Crosses
	glcd_draw_line(horiz, vert+2, horiz+4, vert+6, BLACK);
	glcd_draw_line(horiz, vert+6, horiz+4, vert+2, BLACK);
	glcd_draw_line(horiz+2, vert, horiz+4, vert+2, BLACK);
	glcd_draw_line(horiz+2, vert+8, horiz+4,vert+6, BLACK );
	
	
}



void print_Buff_to_Box(char strBuff[], uint32_t currInd, uint8_t xMin, uint8_t yMin, uint8_t xCharMax, uint8_t yCharMax){
	int32_t bI;
	
	uint32_t totalChars = xCharMax*yCharMax;
	
	if(totalChars>currInd){
		//  Enough room to display everything, start at beginning
		bI = 0;
	}else{
		bI = currInd-totalChars+1;
	}
	uint8_t i,j;
	char dispChar;
	for (j=0;j<yCharMax;j++){
		for(i = 0;i<xCharMax;i++){
			switch (strBuff[bI]){
				case 0x0a:
					dispChar = 'N';
					break;
				case 0x0d:
					dispChar = 'C';
					break;
				default:
					dispChar = strBuff[bI];
			};
			glcd_tiny_draw_char_xy(i*5+xMin, j*8+yMin, dispChar);	
			bI++;
			
		}
	}
	
	
	
}


uint8_t getCursor(enum INPUT input_temp, uint8_t cursor, uint8_t cursorN){

	switch (input_temp){
		case input_button2:
			if (cursor>0){
				cursor--;
			}
			break;
		case input_button3:
			if ((cursor+1)<cursorN){
				cursor++;
			}
			break;
		default:
			break;
	};
	return cursor;
}

float getDispX(float XYZ[3], uint8_t boxMin, uint8_t boxWidth, bool invert){
	//  Calculate the normalized X position for display of a 3-D vector
	//  X is displacement of vector along X axis
	//  Returned X dimension normalized to box width and offset by box min
	float magnitude, xPos;
	
	//  Total magnitude of vector
	magnitude = sqrt(pow(XYZ[0],2)+pow(XYZ[1],2)+pow(XYZ[2],2));
	// Normalize X dimension by magnitude
	// Scale -1:1
	xPos = XYZ[0]/magnitude;
	if (invert){
		xPos = -1*xPos;
	}
	//  Normalize to requested range
	xPos = ((xPos+1)/2) ;
	xPos = xPos*boxWidth+boxMin;
	
	return xPos;
	
}

void drawBox(uint8_t xLine, uint8_t button){
	//  Draw Box
	glcd_draw_line(xLine, yLine[button-1],   128, yLine[button-1],   BLACK);//  Horizontal Top
	glcd_draw_line(xLine, yLine[button], 128, yLine[button], BLACK);//  Horizontal Bottom
	glcd_draw_line(xLine, yLine[button-1], xLine, yLine[button], BLACK); //  Vertical
	
}


void drawSoftKeys(const char *str1, const char *str2, const char *str3, const char *str4){
	uint8_t xLine, i;
	
	
	uint8_t strLength[4];
	char *strPtr[4];
	
	strLength[0] = strlen(str1);
	strLength[1] = strlen(str2);
	strLength[2] = strlen(str3);
	strLength[3] = strlen(str4);
	
	strPtr[0] = str1;
	strPtr[1] = str2;
	strPtr[2] = str3;
	strPtr[3] = str4;
	
	for (i=0;i<4;i++){
		if (strLength[i]>0){
			//  Draw Box
			xLine = 128-(strLength[i]*6)-1;
			
			//  Draw Text
			if (!strcmp(strPtr[i],"<")){
				//  Arrow Up
				xLine = 117;
				glcd_draw_line(119, yLine[i]+10, 123, yLine[i]+6, BLACK);
				glcd_draw_line(123, yLine[i]+6, 127, yLine[i]+10, BLACK);
			}else if (!strcmp(strPtr[i],">")){
				//  Arrow Down
				xLine = 117;
				glcd_draw_line(119, yLine[i]+6, 123, yLine[i]+10, BLACK);
				glcd_draw_line(123, yLine[i]+10, 127, yLine[i]+6, BLACK);
			}else{
				glcd_draw_string_xy(xLine+2,yLine[i]+4,strPtr[i]);
			}
			
			//  Draw Box
			drawBox(xLine, i+1);// "i" index 0-3, button 1-4
			//glcd_draw_line(xLine, yLine[i],   128, yLine[i],   BLACK);//  Horizontal Top
			//glcd_draw_line(xLine, yLine[i+1], 128, yLine[i+1], BLACK);//  Horizontal Bottom
			//glcd_draw_line(xLine, yLine[i], xLine, yLine[i+1], BLACK); //  Vertical
			
			
		}
	}
	
}


void draw2LineSoftKey(char *str1, char *str2, uint8_t button){
	uint8_t strLength;
	uint8_t xLine;
	
	if (strlen(str1)>strlen(str2)){
		strLength = strlen(str1);
	}else{
		strLength = strlen(str2);
	}
	
	xLine = 128-(strLength*6)-1;	
	
	drawBox(xLine, button);
	
	glcd_draw_string_xy(xLine+2,yLine[button-1]+1,str1);
	glcd_draw_string_xy(xLine+2,yLine[button-1]+8,str2);
}


void disp_report(uint8_t pageView){
	switch(pageView){
		///////////////////////// AZM and INC Report
		case 1:
			//// Page 1			
			sprintf(display_str, "Inclination & Azimuth");
			glcd_tiny_draw_string(0,1,display_str);
			genTimestampString(display_str, &cal_report.time_inc_azm,  1);
			glcd_tiny_draw_string(0,2,display_str);
			sprintf(display_str,"4-Point Groups: %d", cal_report.groups);
			glcd_tiny_draw_string(0,3,display_str);
			sprintf(display_str,"Azm Stdev: %.3f", cal_report.azm_angle_err);
			glcd_tiny_draw_string(0,5,display_str);
			glcd_draw_circle(98, 41, 1, BLACK);// Draw degree symbol
			sprintf(display_str,"Inc Stdev: %.3f", cal_report.inc_angle_err);
			glcd_tiny_draw_string(0,6,display_str);
			glcd_draw_circle(98, 49, 1, BLACK);// Draw degree symbol
			if (options.current_unit_temp==fahrenheit){
				sprintf(display_str,"Temp: %0.1f F", celsius2fahrenheit( cal_report.tempC_inc_azm));
			}else{
				sprintf(display_str,"Temp: %0.1f C", cal_report.tempC_inc_azm);
			}
			glcd_tiny_draw_string(0,7,display_str);
			break;
		case 2:
			//// Page 2
			sprintf(display_str, "Inclination:");
			glcd_tiny_draw_string(0,1,display_str);
			genTimestampString(display_str, &cal_report.time_inc_azm, 1);
			glcd_tiny_draw_string(0,2,display_str);
			// Sensor Disparity
			sprintf(display_str,"A1-A2 Delta X,Y,Z %%");
			glcd_tiny_draw_string(0,3,display_str);
			sprintf(display_str,"%.3f, %.3f, %.3f",
				cal_report.disp_stdev_acc[0]*100, cal_report.disp_stdev_acc[1]*100, cal_report.disp_stdev_acc[2]*100);
			glcd_tiny_draw_string(0,4,display_str);
			//  Magnitude Error			
			sprintf(display_str,"Magnitude Error %%");
			glcd_tiny_draw_string(0,5,display_str);
			sprintf(display_str,"A1:%.3f A2:%.3f", cal_report.mag_stdev_a1*100, cal_report.mag_stdev_a2*100);
			glcd_tiny_draw_string(0,6,display_str);
			break;
		case 3:
			//// Page 3
			sprintf(display_str, "Azimuth");
			glcd_tiny_draw_string(0,1,display_str);
			genTimestampString(display_str, &cal_report.time_quick_azm,  1);
			glcd_tiny_draw_string(0,2,display_str);
			// Sensor Disparity
			sprintf(display_str,"M1-M2 Delta X,Y,Z %%");
			glcd_tiny_draw_string(0,3,display_str);
			sprintf(display_str,"%.3f, %.3f, %.3f",
			cal_report.disp_stdev_comp[0]*100, cal_report.disp_stdev_comp[1]*100, cal_report.disp_stdev_comp[2]*100);
			glcd_tiny_draw_string(0,4,display_str);
			//  Magnitude Error
			sprintf(display_str,"Magnitude Error %%");
			glcd_tiny_draw_string(0,5,display_str);
			sprintf(display_str,"M1:%.3f M2:%.3f", cal_report.mag_stdev_m1*100, cal_report.mag_stdev_m2*100);
			glcd_tiny_draw_string(0,6,display_str);
			break;
		//////////////////////// Distance Report
		case 4:
			sprintf(display_str, "Distance");
			glcd_tiny_draw_string(0,1,display_str);
			genTimestampString(display_str, &cal_report.time_rangeFinder,  1);
			glcd_tiny_draw_string(0,2,display_str);
			sprintf(display_str,"Rangefinder Offset:");
			glcd_tiny_draw_string(0,4,display_str);
			sprintf(display_str,"  %.4f meters", dist_calst.dist_offset);
			glcd_tiny_draw_string(0,5,display_str);
			sprintf(display_str,"  %.4f feet", dist_calst.dist_offset*MT2FT);
			glcd_tiny_draw_string(0,6,display_str);
			
		break;
	}
}




uint16_t incDecData(uint16_t intData, int8_t increment, uint16_t dataMin, uint16_t dataMax){
	//  Increments "intData" by increment, assuming increment is either +1 or -1;
	
	//  Subtract 1
	if(increment<0){
		if(intData>dataMin){
			return(intData-1);
		}else{
			return (dataMax);
		}
	}
	
	//  Add 1
	if(increment>0){
		if(intData<dataMax){
			return(intData+1);
		}else{
			return(dataMin);
		}
	}
	
	// For increment == 0
	return intData;
}



