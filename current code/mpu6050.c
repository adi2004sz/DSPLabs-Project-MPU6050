#include <lpc214x.h>
#include <stdint.h>
#include <string.h>
#include "serial.h"

uint8_t gyroRange=0 , accRange=0;

volatile int cntOperation = 0, cntWrite = 0, cntRead = 0, cntIndex = 0;
volatile int doneInit = 0;
uint8_t lenWrite = 13, lenRead = 13;
uint8_t measurements[14];
double results[7];

void gyroConfig(int selectGyro){
	// adresa 1B
	switch (selectGyro) {
  case 0:
			gyroRange = 0x00;
    break;
  case 1:
			gyroRange = 0x01;
		break;
	case 2:
			gyroRange = 0x02;
    break;
	case 3:
			gyroRange = 0x03;
    break;
  default:
    gyroRange = 0x00;
	}
	//nu vreau sa las userul sa aiba acces la variabile globale, vreau ca el sa lucreze prin functii
}

void accConfig(int selectAcc){
	//adresa 1C
	switch (selectAcc) {
  case 0:
			accRange = 0x00;
    break;
  case 1:
			accRange = 0x01;
		break;
	case 2:
			accRange = 0x02;
    break;
	case 3:
			accRange = 0x03;
    break;
  default:
    accRange = 0x00;
	}
}
	
void resetI2C(void)
{
	I2C0CONCLR = 0x000000FF;
	I2C0CONSET = 0x00000040;
	I2C0CONSET = 0x00000020;
}

void pollingWrite(uint8_t adresa, uint8_t valoare)
{
	uint8_t stat;
	stat = I2C0STAT;
	switch(stat)
	{
		case 0x08: // start
		{
			cntOperation = 0;
			I2C0DAT = 0xD0;
			I2C0CONCLR = 0x20;
			I2C0CONSET = 0x04;
			break;
		}
		case 0x18: // enter data ACK received
		{
			cntOperation++;
			I2C0DAT = adresa;
			break;
		}
		case 0x20: // enter data NACK received
		{
			I2C0CONSET = 0x14;
			break;
		}
		case 0x28: // data sent with ACK
		{
			cntOperation++;
			I2C0DAT = valoare;
			if(cntOperation == 3)
			{
				I2C0CONSET = 0x20;
				if(cntWrite < lenWrite)
				{
					cntWrite++;
				}
				else
				{
					doneInit = 1;
				}
			}		
			break;
		}
		case 0x30: // data sent with NACK
		{
			I2C0CONSET = 0x14;
			break;
		}
		default:
		{
			break;
		}
	}
	if (stat != 0xF8)
	{
		I2C0CONCLR = 0x08;
	}
}

void pollingRead(uint8_t adresa)
{
	uint8_t stat;
	stat = I2C0STAT;
	switch(stat)
	{
		case 0x08: // start
		{
			I2C0DAT = 0xD0;
			I2C0CONCLR = 0x20;
			I2C0CONSET = 0x04;
			break;
		}
		case 0x10: // repeated start
		{
			I2C0DAT = 0xD1;
			I2C0CONCLR = 0x20;
			I2C0CONSET = 0x04;
			break;
		}
		case 0x18: // enter data ACK received
		{
			I2C0DAT = adresa;
			break;
		}
		case 0x20: // enter data NACK received
		{
			I2C0CONSET = 0x14;
			break;
		}
		case 0x28: // data sent with ACK
		{
			I2C0CONSET = 0x20;
			break;
		}
		case 0x30: // data sent with NACK
		{
			I2C0CONSET = 0x14;
			break;
		}
		case 0x40: // address received with ACK
		{
			I2C0CONSET = 0x04; // genereaza data + ACK
			break;
		}
		case 0x48: //  address received with NACK
		{
			I2C0CONSET = 0x20;
			break;
		}
		case 0x50: // data received with ACK
		{
			measurements[cntIndex++] = I2C0DAT;
			cntRead++;
			if(cntRead == lenRead)
			{
				I2C0CONCLR = 0x3C; // genereaza data + NACK
			}
			break; 
		}
		case 0x58: // data received with NACK
		{
			measurements[cntIndex++] = I2C0DAT;
			I2C0CONSET = 0x14;
			cntRead++;
			cntIndex = 0;
			break;
		}
		default:
		{
			break;
		}
	}
	if (stat != 0xF8)
	{
		I2C0CONCLR = 0x08;
	}
}

void processMeasurements(uint8_t *measurements, double *results)
{
	int16_t ax, ay, az, t, gx, gy, gz;
	double AX, AY, AZ, T, GX, GY, GZ;
	double RangeDividerGyro = 0,RangeDividerAcc = 0;
	
	switch (gyroRange) {
  case 0x00:
			RangeDividerGyro = 131.0;
    break;
  case 0x01:
			RangeDividerGyro = 65.5;
		break;
	case 0x02:
			RangeDividerGyro = 32.8;
    break;
	case 0x03:
			RangeDividerGyro = 16.4;
    break;
  default:
    RangeDividerGyro = 131.0;
	}
	
	switch (accRange) {
  case 0x00:
			RangeDividerAcc = 16384.0;
    break;
  case 0x01:
			RangeDividerAcc = 8192.0;
		break;
	case 0x02:
			RangeDividerAcc = 4096.0;
    break;
	case 0x03:
			RangeDividerAcc = 2048.0;
    break;
  default:
    RangeDividerAcc = 16384.0;
	}
	
	
	ax = ((int16_t)(measurements[0]) << 8) | ((int16_t)(measurements[1]));
	AX = (double)ax / RangeDividerAcc;
	ay = ((int16_t)(measurements[2]) << 8) | ((int16_t)(measurements[3]));
	AY = (double)ay / RangeDividerAcc;
	az = ((int16_t)(measurements[4]) << 8) | ((int16_t)(measurements[5]));
	AZ = (double)az / RangeDividerAcc;
	
	t = ((int16_t)(measurements[6]) << 8) | ((int16_t)(measurements[7]));
	T = ((double)t / 340.0) + 36.53;	
	
	gx = ((int16_t)(measurements[8]) << 8) | ((int16_t)(measurements[9]));
	GX = (double)gx / RangeDividerGyro;
	gy = ((int16_t)(measurements[10]) << 8) | ((int16_t)(measurements[11]));
	GY = (double)gy / RangeDividerGyro;
	gz = ((int16_t)(measurements[12]) << 8) | ((int16_t)(measurements[13]));
	GZ = (double)gz / RangeDividerGyro;
	results[0] = AX;
	results[1] = AY;
	results[2] = AZ;
	results[3] = T;
	results[4] = GX;
	results[5] = GY;
	results[6] = GZ;
}

void startSensor(void)
{
	resetI2C();
}
