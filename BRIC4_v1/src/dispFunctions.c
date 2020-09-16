/*
 * dispFunctions.c
 *
 * Created: 8/22/2020 5:56:03 PM
 *  Author: Kris Fausnight
 */ 
#include <dispFunctions.h>

const uint8_t yLine[] = {1, 16, 32, 48, 64};




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


void drawSoftKeys(char *str1, char *str2, char *str3, char *str4){
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

