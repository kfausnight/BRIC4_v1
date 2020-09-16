/*
 * mathBRIC.c
 *
 * Created: 8/29/2020 5:32:14 PM
 *  Author: Kris Fausnight
 */ 

#include <mathBRIC.h>


void calc_orientation(struct MEASUREMENT *meas_inst){
	uint8_t i;
	float aXYZ[3], cXYZ[3];
	
	//  Take average reading from both sensors for each axis
	for (i=0;i<3;i++){
		aXYZ[i] = 0.5*(meas_inst->a1Cal[i]+meas_inst->a2Cal[i]);
		cXYZ[i] = 0.5*(meas_inst->m1Cal[i]+meas_inst->m2Cal[i]);
	}
	
	//  Calculate Aximuth, Inclination, and Roll
	calc_azm_inc_roll_dec(aXYZ, cXYZ,
	&meas_inst->azimuth,		&meas_inst->inclination,
	&meas_inst->roll,		&meas_inst->declination);
	
	
}





void calc_azm_inc_roll_dec(float aXYZ[3], float cXYZ[3], float *azimuthP, float *inclinationP, float *rollP, float *declinationP){
	
	float crotXYZ[3];
	float thetaX, thetaY, crxy;
	
	//  Calculate Inclination and Roll
	//  thetaX and thetaY are rotations of instrument relative to NED
	calc_theta_XY( aXYZ , &thetaX, &thetaY);
	*inclinationP = -1*thetaY;
	*rollP = thetaX;
	//  Roll is -180 to 180; Convert to 0-360;
	if ((*rollP)<0){
		*rollP = *rollP+360;
	}
	
	//  Calculate Azimuth
	rotvec_theta_XY(cXYZ, crotXYZ, thetaX, thetaY);
	*azimuthP = RAD2DEG*atan2(crotXYZ[1], crotXYZ[0]);
	//  Result is -180 to +180; Compass must be 0-360
	if ((*azimuthP)<0){
		*azimuthP = *azimuthP+360;
	}
	
	//  Calculate declination
	crxy= sqrt(pow(crotXYZ[0],2)+pow(crotXYZ[1],2));
	*declinationP = RAD2DEG*atan2(crotXYZ[2], crxy);
	
	
}

float calc_mag_stdev(float XYZ[NBUFF][3]){
	float err_mag[NBUFF];
	uint32_t p;
	float temp1;
	
	for (p=0;p<nPoints;p++){
		temp1 = sqrt(pow(XYZ[p][0],2)+pow(XYZ[p][1],2)+pow(XYZ[p][2],2));
		err_mag[p] = temp1-1;
	}
	temp1 = stdev(err_mag, nPoints);
	
	return temp1;
}




float calc_disp_stdev(float XYZ1[NBUFF][3], float XYZ2[NBUFF][3], uint8_t axis){
	float err_disp[NBUFF];
	uint32_t p;
	for (p=0;p<nPoints;p++){
		err_disp[p] =  XYZ1[p][axis]-XYZ2[p][axis];
		
	}
	
	return stdev(err_disp, nPoints);
	
}

void rotvec_theta_ZY(float XYZ[3], float rotXYZ[3], float thetaZ, float thetaY){
	float rotM[3][3];
	float rthetaZ;
	float rthetaY;
	
	rthetaZ = thetaZ*DEG2RAD;
	rthetaY = thetaY*DEG2RAD;
	
	//  Rotation Matrix about Z Axis
	rotM[0][0] = cos(rthetaZ);
	rotM[0][1] = -1*sin(rthetaZ);
	rotM[0][2] = 0;
	rotM[1][0] = sin(rthetaZ);
	rotM[1][1] = cos(rthetaZ);
	rotM[1][2] = 0;
	rotM[2][0] = 0;
	rotM[2][1] = 0;
	rotM[2][2] = 1;
	mat_mult_33_31(rotM, XYZ, rotXYZ);
	// Rotation Matrix about Y Axis
	rotM[0][0] = cos(rthetaY);
	rotM[0][1] = 0;
	rotM[0][2] = sin(rthetaY);
	rotM[1][0] = 0;
	rotM[1][1] = 1;
	rotM[1][2] = 0;
	rotM[2][0] = -1*sin(rthetaY);
	rotM[2][1] = 0;
	rotM[2][2] = cos(rthetaY);
	mat_mult_33_31(rotM, rotXYZ, rotXYZ);
	
	
}

void rotvec_theta_XY(float XYZ[3], float rotXYZ[3], float thetaX, float thetaY){
	float rotM[3][3];
	float rthetaX;
	float rthetaY;
	
	rthetaX = thetaX*DEG2RAD;
	rthetaY = thetaY*DEG2RAD;
	
	// Product of two rotation matrixes, R(thetaX)*R(thetaY)
	//  Rotate around X axis
	rotM[0][0] = 1;
	rotM[0][1] = 0;
	rotM[0][2] = 0;
	rotM[1][0] = 0;
	rotM[1][1] = cos(rthetaX);
	rotM[1][2] = -1*sin(rthetaX);
	rotM[2][0] = 0;
	rotM[2][1] = sin(rthetaX);
	rotM[2][2] = cos(rthetaX);
	mat_mult_33_31(rotM, XYZ, rotXYZ);
	
	//  Rotate about Y axis
	rotM[0][0] = cos(rthetaY);
	rotM[0][1] = 0;
	rotM[0][2] = sin(rthetaY);
	rotM[1][0] = 0;
	rotM[1][1] = 1;
	rotM[1][2] = 0;
	rotM[2][0] = -1*sin(rthetaY);
	rotM[2][1] = 0;
	rotM[2][2] = cos(rthetaY);
	mat_mult_33_31(rotM, rotXYZ, rotXYZ);


}


void mat_mult_33_31(float mat33[3][3], float mat3[3], float ret3[3]){
	uint8_t i, j;
	float temp[3];
	
	//  Multiply 3x3 mat33 matrix by 3x1 mat3 matrix
	for(i=0;i<3;i++){
		temp[i] = 0;
		for (j=0;j<3;j++){
			temp[i] = temp[i]+mat33[i][j]*mat3[j];
		}

	}
	//  Copy temp matrix back into ret3 matrix
	for(i=0;i<3;i++){
		ret3[i] = temp[i];
	}
	
}

void calc_theta_XY(float XYZ[3], float *thetaX, float *thetaY){
	float ryz;
	ryz = sqrt(pow(XYZ[1],2) + pow(XYZ[2],2));
	
	*thetaX = RAD2DEG*atan2(XYZ[1], XYZ[2]);
	*thetaY = -1*RAD2DEG*atan2(XYZ[0], ryz);
	
}


float stdev(float data[], uint32_t n_meas){
	uint8_t i;
	double mean, sumsq;
	
	mean = meanArr(data, n_meas);
	
	sumsq = 0;
	for (i=0;i<n_meas;i++){
		sumsq = sumsq+pow((data[i]-mean),2);
	}
	sumsq = sumsq/(n_meas-1);
	sumsq = sqrt(sumsq);
	
	return sumsq;
}

float meanArr(float data[], uint32_t n_meas){
	uint8_t i;
	float mean;
	
	
	mean = 0;
	for (i=0;i<n_meas;i++){
		mean = mean+data[i];
	}
	mean = mean/n_meas;
	return mean;
	
}


void inverse(float source[6][6], float dest[6][6], uint8_t f)
{
	float b[6][6], fac[6][6];
	uint8_t p, q, m, n, i, j;
	//f = 6;
	for (q = 0;q < f; q++)
	{
		for (p = 0;p < f; p++)
		{
			m = 0;
			n = 0;
			for (i = 0;i < f; i++)
			{
				for (j = 0;j < f; j++)
				{
					if (i != q && j != p)
					{
						b[m][n] = source[i][j];
						if (n < (f - 2))
						n++;
						else
						{
							n = 0;
							m++;
						}
					}
				}
			}
			fac[q][p] = pow(-1, q + p) * determinant(b, f - 1);
		}
	}
	transpose(source, dest, fac, f);
}
/*Finding transpose of matrix*/
void transpose(float source[6][6], float dest[6][6], float fac[6][6], uint8_t r)
{
	uint8_t i, j;
	//r = 6;
	float b[6][6], d;
	
	for (i = 0;i < r; i++)
	{
		for (j = 0;j < r; j++)
		{
			b[i][j] = fac[j][i];
		}
	}

	
	d = determinant(source, r);
	for (i = 0;i < r; i++)
	{
		for (j = 0;j < r; j++)
		{
			dest[i][j] = b[i][j] / d;
		}
	}
	
}


float determinant(float a[6][6], uint8_t k)
{
	float s = 1, det = 0, b[6][6];
	uint8_t i, j, m, n, c;
	if (k == 1)
	{
		return (a[0][0]);
	}
	else
	{
		det = 0;
		for (c = 0; c < k; c++)
		{
			m = 0;
			n = 0;
			for (i = 0;i < k; i++)
			{
				for (j = 0 ;j < k; j++)
				{
					b[i][j] = 0;
					if (i != 0 && j != c)
					{
						b[m][n] = a[i][j];
						if (n < (k - 2))
						n++;
						else
						{
							n = 0;
							m++;
						}
					}
				}
			}
			det = det + s * (a[0][c] * determinant(b, k - 1));
			s = -1 * s;
		}
	}
	
	return (det);
}





