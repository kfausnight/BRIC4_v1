/*
 * batteryManagement.c
 *
 * Created: 4/14/2020 6:51:29 PM
 *  Author: Kris Fausnight
 */ 

#include <batteryManagement.h>


//  Battery Sensor Values
//capacity is in 5 uV-hours/Rsense.
//Battery is 1800 mAH, Rsense=10mOhms.  Capacity = 3600d=0x0E10
uint16_t DesignCap = 0x0E10;//1800mAH@3.7V lithium polymer
//Per ADP5062, default termination current is 52.5mA
// 156.25 uA/LSB, 52.5mA = 336d=0x0150
uint16_t IchgTerm = 0x0150;
//Per ADP5062, dead battery is 2.6V, lockout at 2.5V
//0.078125 mV/LSB, 2.6V = 33280d=0x8200
uint16_t VEmpty = 0x8200;
//Per ADP5062, default termination voltage is 4.2V
//.078125mV/LSB, 4.2V = 53760d=0xD200
uint16_t VFull=0xD200;
uint16_t HibCFG;
float battVoltage;




void setup_batt(void){
	uint16_t data1;
	
	//Read POR (powerup reset) status
	max17055_reg_read_write(readp, 0x00, &data1);
	data1=data1 & 0x0002;//bit two is POR
	if (data1){
		config_batt();
	}
	max17055_reg_read_write(readp, 0x00, &data1);
	data1=data1 & 0xFFFD;//clear POR bit
	max17055_reg_read_write(writep ,0x00, &data1);
	
}

void config_batt(void){
	uint16_t data1;
	max17055_reg_read_write(readp, 0x3D, &data1);//check DNR bit 0x3D bit 1
	while(data1 & 0x0001){
		delay_ms(10);
		max17055_reg_read_write(readp, 0x3D, &data1);
	}
	max17055_reg_read_write(writep, 0x18, &DesignCap);//Write design capacity register 0x18
	data1=DesignCap/32;
	max17055_reg_read_write(writep, 0x45, &data1);//write dQacc register 0x45
	max17055_reg_read_write(writep, 0x1E, &IchgTerm);//Write termination charge register 0x1E
	max17055_reg_read_write(writep, 0x3A, &VEmpty);//Write empty voltage, register 0x3A
	max17055_reg_read_write(readp, 0xBA, &HibCFG);//Save hibernation configuration
	data1=0x90;
	max17055_reg_read_write(writep, 0x60, &data1);//Exit hibernate mode step 1
	data1=0x00;
	max17055_reg_read_write(writep,0xBA, &data1);//exit hibernate mode step 2
	data1=0x00;
	max17055_reg_read_write(writep, 0x60, &data1);//exit hibernate mode step 3
	data1=DesignCap/32;
	data1=data1*44138;
	data1=data1/DesignCap;
	max17055_reg_read_write(writep, 0x46, &data1);//write dPAcc register 0x46
	data1=0x8000;//model 0
	max17055_reg_read_write(writep, 0xD8, &data1);//write model configuration at register D8
	//wait for model to refresh
	data1=0x0000;
	while(!data1){
		delay_ms(10);
		max17055_reg_read_write(readp,0xD8,&data1);
		data1=data1 & 0x8000;
	}
	max17055_reg_read_write(writep, 0xBA, &HibCFG);//restore hiberation configuration at register 0xBA
}

uint16_t getBatteryLevel(void){
	//  Reads back battery state of charge in %, 0-100
	uint16_t batt_SOC;
	max17055_reg_read_write(readp, 0x06, &batt_SOC);
	batt_SOC=batt_SOC>>8;
	//  Add margin to show topped-off battery
	batt_SOC = batt_SOC*1.03;  
	if (batt_SOC>100){
		batt_SOC = 100;
	}
	
	return batt_SOC;
}


void setChargeCurrent(uint32_t chargeCurrent){
	// ILIM is lower 3 bits of addr 0x02
	uint8_t data;
	if (chargeCurrent==500){
		data = 0x06;
	}else{
		data = 0x00;
	}
	adp5062_reg_read_write(writep, 0x02, &data);
	
	
	
}

uint8_t getChargerStatus(void){
	uint8_t status;
	// Read Charge Status 1 (add 0x0B)
	status = getChargerRegister(0x0B);
	
	status = status & 0x07;
	
	return status;
}


uint8_t getChargerRegister(uint8_t address){
	//  Read data from address on Charger
	uint8_t registerData;
	
	adp5062_reg_read_write(readp, address, &registerData); 
	
	return registerData;
}



void  bin2str(uint8_t data, char *strPtr){
	uint8_t i;
	uint8_t shifter = 128;
	
	for (i=0;i<8;i++){
		if (data&shifter){
			strPtr[i] = '1';
		}else{
			strPtr[i] = '0';
		}
		shifter = shifter>>1;
	}
	strPtr[8] = '\0';
	
}



