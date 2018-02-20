#include <math.h>
#include <string.h>
#include "my_mat_math_5.h"

void mmMakeIdentityMatrix(float * m16)
{
	m16[0] = 1.0f;
	m16[1] = 0.0f;
	m16[2] = 0.0f;
	m16[3] = 0.0f;
	
	m16[4] = 0.0f;
	m16[5] = 1.0f;
	m16[6] = 0.0f;
	m16[7] = 0.0f;
	
	m16[8] = 0.0f;
	m16[9] = 0.0f;
	m16[10] = 1.0f;
	m16[11] = 0.0f;
	
	m16[12] = 0.0f;
	m16[13] = 0.0f;
	m16[14] = 0.0f;
	m16[15] = 1.0f;	
}

void mmTranslateMatrix(float * m16, float x, float y, float z)
{
	mmMakeIdentityMatrix(m16);
	m16[12] = x;
	m16[13] = y;
	m16[14] = z;
}

//misnomer rads is actually degrees
void mmRotateAboutY(float * m16, float degs)
{
	float sine;
	float cosine;
	
	degs = RATIO_DEGTORAD*degs;
	sine = sinf(degs);
	cosine = cosf(degs);
	mmMakeIdentityMatrix(m16);
	m16[0] = cosine;
	m16[8] = sine;
	m16[2] = -sine;
	m16[10] = cosine;	
}

void mmRotateAboutYRad(float * m16, float rads)
{
	float sine;
	float cosine;

	sine = sinf(rads);
	cosine = cosf(rads);

	mmMakeIdentityMatrix(m16);
	m16[0] = cosine;
	m16[8] = sine;
	m16[2] = -sine;
	m16[10] = cosine;
}

void mmRotateAboutZ(float * m16, float rads)
{
	float sine;
	float cosine;
	
	rads = RATIO_DEGTORAD*rads;
	sine = sinf(rads);
	cosine = cosf(rads);
	mmMakeIdentityMatrix(m16);
	
	m16[0] = cosine;
	m16[1] = -sine;
	m16[4] = sine;
	m16[5] = cosine;
}

void mmRotateAboutX(float * m16, float degs)
{
	float sine;
	float cosine;
	
	degs = RATIO_DEGTORAD*degs;
	sine = sinf(degs);
	cosine = cosf(degs);
	mmMakeIdentityMatrix(m16);
	
	m16[5] = cosine;
	m16[6] = sine;
	m16[9] = -sine;
	m16[10] = cosine;
}

void mmRotateAboutXRad(float * m16, float rads)
{
	float sine;
	float cosine;

	sine = sinf(rads);
	cosine = cosf(rads);
	mmMakeIdentityMatrix(m16);

	m16[5] = cosine;
	m16[6] = sine;
	m16[9] = -sine;
	m16[10] = cosine;
}

void mmConvertMat4toMat3(float * m16, float * m9)
{
	m9[0] = m16[0];
	m9[1] = m16[1];
	m9[2] = m16[2];
	
	m9[3] = m16[4];
	m9[4] = m16[5];
	m9[5] = m16[6];
	
	m9[6] = m16[8];
	m9[7] = m16[9];
	m9[8] = m16[10];
}

/*
returns r16
*/
void mmMultiplyMatrix4x4(float * m, float * n, float * r)
{
	float t[16]; //temp matrix
	t[0] = (m[0]*n[0]) + (m[4]*n[1]) + (m[8]*n[2]) + (m[12]*n[3]);
	t[1] = (m[1]*n[0]) + (m[5]*n[1]) + (m[9]*n[2]) + (m[13]*n[3]);
	t[2] = (m[2]*n[0]) + (m[6]*n[1]) + (m[10]*n[2]) + (m[14]*n[3]);
	t[3] = (m[3]*n[0]) + (m[7]*n[1]) + (m[11]*n[2]) + (m[15]*n[3]);
	
	t[4] = (m[0]*n[4]) + (m[4]*n[5]) + (m[8]*n[6]) + (m[12]*n[7]);
	t[5] = (m[1]*n[4]) + (m[5]*n[5]) + (m[9]*n[6]) + (m[13]*n[7]);
	t[6] = (m[2]*n[4]) + (m[6]*n[5]) + (m[10]*n[6]) + (m[14]*n[7]);
	t[7] = (m[3]*n[4]) + (m[7]*n[5]) + (m[11]*n[6]) + (m[15]*n[7]);
	
	t[8] = (m[0]*n[8]) + (m[4]*n[9]) + (m[8]*n[10]) + (m[12]*n[11]);
	t[9] = (m[1]*n[8]) + (m[5]*n[9]) + (m[9]*n[10]) + (m[13]*n[11]);
	t[10] = (m[2]*n[8]) + (m[6]*n[9]) + (m[10]*n[10]) + (m[14]*n[11]);
	t[11] = (m[3]*n[8]) + (m[7]*n[9]) + (m[11]*n[10]) + (m[15]*n[11]);
	
	t[12] = (m[0]*n[12]) + (m[4]*n[13]) + (m[8]*n[14]) + (m[12]*n[15]);
	t[13] = (m[1]*n[12]) + (m[5]*n[13]) + (m[9]*n[14]) + (m[13]*n[15]);
	t[14] = (m[2]*n[12]) + (m[6]*n[13]) + (m[10]*n[14]) + (m[14]*n[15]);
	t[15] = (m[3]*n[12]) + (m[7]*n[13]) + (m[11]*n[14]) + (m[15]*n[15]);
	
	memcpy(r, t, (sizeof(float)*16));
}

//Assumes: m is a 4x4 matrix, and v is a 4x1 matrix (aka vector)
void mmTransformVec(float * m, float * v)
{
	float r[4];
	
	r[0] = (m[0]*v[0]) + (m[4]*v[1]) + (m[8]*v[2]) + (m[12]*v[3]);
	r[1] = (m[1]*v[0]) + (m[5]*v[1]) + (m[9]*v[2]) + (m[13]*v[3]);
	r[2] = (m[2]*v[0]) + (m[6]*v[1]) + (m[10]*v[2]) + (m[14]*v[3]);
	r[3] = (m[3]*v[0]) + (m[7]*v[1]) + (m[11]*v[2]) + (m[15]*v[3]);
	
	memcpy(v, r, (sizeof(float)*4));
}

/*
Transform v, a 3x1 matrix, with m, a 3x3 matrix.
*/
void mmTransformVec3(float * m, float * v)
{
	float r[3];
	
	r[0] = (m[0]*v[0]) + (m[3]*v[1]) + (m[6]*v[2]);
	r[1] = (m[1]*v[0]) + (m[4]*v[1]) + (m[7]*v[2]);
	r[2] = (m[2]*v[0]) + (m[5]*v[1]) + (m[8]*v[2]);
	
	memcpy(v, r, 3*sizeof(float));
}

float mmDeterminant4x4(float * m)
{
	float r[4];
	r[0] = (m[5]*(m[10]*m[15]-m[11]*m[14])) - (m[9]*(m[6]*m[15]-m[7]*m[14])) + (m[13]*(m[6]*m[11]-m[10]*m[7]));
	r[0] = r[0] * m[0];
	
	r[1] = (m[1]*(m[10]*m[15]-m[11]*m[14])) - (m[9]*(m[2]*m[15]-m[14]*m[3])) + (m[13]*(m[2]*m[11]-m[10]*m[3])); 
	r[1] = r[1] * m[4];
	
	r[2] = (m[1]*(m[6]*m[15]-m[14]*m[7])) - (m[5]*(m[2]*m[15]-m[14]*m[3])) + (m[13]*(m[2]*m[7]-m[6]*m[3]));
	r[2] = r[2] * m[8];
	
	r[3] = (m[1]*(m[6]*m[11]-m[10]*m[7])) - (m[5]*(m[2]*m[11]-m[10]*m[3])) + (m[9]*(m[2]*m[7]-m[6]*m[3]));
	r[3] = r[3] * m[12];
	
	return (r[0] - r[1] + r[2] - r[3]);
}

/*
calculate an inverse matrix using the adjugate method
*/
void mmInverse4x4(float * m)
{
	float n[16];
	float det;
	float idet;
	int i;
	
	det = mmDeterminant4x4(m);
	idet = 1.0f / det;
	
	//Calculate the "Matrix of Minors"
	n[0] = m[5]*(m[10]*m[15]-m[11]*m[14]) - m[9]*(m[6]*m[15]-m[14]*m[7]) + m[13]*(m[6]*m[11]-m[10]*m[7]);	
	n[1] = m[4]*(m[10]*m[15]-m[14]*m[11]) - m[8]*(m[6]*m[15]-m[14]*m[7]) + m[12]*(m[6]*m[11]-m[10]*m[7]);
	n[2] = m[4]*(m[9]*m[15]-m[13]*m[11]) - m[8]*(m[5]*m[15]-m[13]*m[7]) + m[12]*(m[5]*m[11]-m[7]*m[9]);
	n[3] = m[4]*(m[9]*m[14]-m[13]*m[10]) - m[8]*(m[5]*m[14]-m[13]*m[6]) + m[12]*(m[5]*m[10]-m[6]*m[9]);
	
	n[4] = m[1]*(m[10]*m[15]-m[11]*m[14]) - m[9]*(m[2]*m[15]-m[3]*m[14]) + m[13]*(m[2]*m[11]-m[3]*m[10]);
	n[5] = m[0]*(m[10]*m[15]-m[11]*m[14]) - m[8]*(m[2]*m[15]-m[3]*m[14]) + m[12]*(m[2]*m[11]-m[3]*m[10]);
	n[6] = m[0]*(m[9]*m[15]-m[13]*m[11]) - m[8]*(m[1]*m[15]-m[13]*m[3]) + m[12]*(m[1]*m[11]-m[9]*m[3]);
	n[7] = m[0]*(m[9]*m[14]-m[13]*m[10]) - m[8]*(m[1]*m[14]-m[13]*m[2]) + m[12]*(m[1]*m[10]-m[9]*m[2]);
	
	n[8] = m[1]*(m[6]*m[15]-m[14]*m[7]) - m[5]*(m[2]*m[15]-m[14]*m[3]) + m[13]*(m[2]*m[7]-m[6]*m[3]);
	n[9] = m[0]*(m[6]*m[15]-m[14]*m[7]) - m[4]*(m[2]*m[15]-m[14]*m[3]) + m[12]*(m[2]*m[7]-m[6]*m[3]);
	n[10] = m[0]*(m[5]*m[15]-m[13]*m[7]) - m[4]*(m[1]*m[15]-m[13]*m[3]) + m[12]*(m[1]*m[7]-m[5]*m[3]);
	n[11] = m[0]*(m[5]*m[14]-m[13]*m[6]) - m[4]*(m[1]*m[14]-m[13]*m[2]) + m[12]*(m[1]*m[6]-m[5]*m[2]);
	
	n[12] = m[1]*(m[6]*m[11]-m[10]*m[7]) - m[5]*(m[2]*m[11]-m[10]*m[3]) + m[9]*(m[2]*m[7]-m[6]*m[3]);
	n[13] = m[0]*(m[6]*m[11]-m[10]*m[7]) - m[4]*(m[2]*m[11]-m[10]*m[3]) + m[8]*(m[2]*m[7]-m[6]*m[3]);
	n[14] = m[0]*(m[5]*m[11]-m[9]*m[7]) - m[4]*(m[1]*m[11]-m[9]*m[3]) + m[8]*(m[1]*m[7]-m[5]*m[3]);
	n[15] = m[0]*(m[5]*m[10]-m[9]*m[6]) - m[4]*(m[1]*m[10]-m[9]*m[2]) + m[8]*(m[1]*m[6]-m[5]*m[2]);
	
	//Make the matrix-of-minors into the matrix-of-cofactors
	n[1] = -1.0f*n[1];
	n[3] = -1.0f*n[3];
	n[4] = -1.0f*n[4];
	n[6] = -1.0f*n[6];
	n[9] = -1.0f*n[9];
	n[11] = -1.0f*n[11];
	n[12] = -1.0f*n[12];
	n[14] = -1.0f*n[14];
	
	//transpose the matrix
	m[0] = n[0]; //copy the diagonol
	m[5] = n[5];
	m[10] = n[10];
	m[15] = n[15];
	m[1] = n[4]; //swap the non-diagonal elements
	m[4] = n[1];
	m[2] = n[8];
	m[8] = n[2];
	m[3] = n[12];
	m[12] = n[3];
	m[6] = n[9];
	m[9] = n[6];
	m[7] = n[13];
	m[13] = n[7];
	m[11] = n[14];
	m[14] = n[11];
	
	//multiply by 1/det
	for(i = 0; i < 16; i++)
	{
		m[i] = m[i] * idet;
	}	
}

float mmDeterminant3x3(float * m)
{
	float r[3];
	
	r[0] = m[0]*((m[4]*m[8]) - (m[7]*m[5]));
	r[1] = m[3]*((m[1]*m[8]) - (m[7]*m[2]));
	r[2] = m[6]*((m[1]*m[5]) - (m[4]*m[2]));
	return (r[0] - r[1] + r[2]);
}

/*
Calculate the inverse matrix of a 3x3 matrix using the adjugate method
*/
void mmInverse3x3(float * m)
{
	float n[9];
	float idet;
	int i;
	
	idet = 1.0f / mmDeterminant3x3(m);
	
	//Calculate the matrix of minors
	n[0] = (m[4]*m[8]) - (m[7]*m[5]);
	n[1] = (m[3]*m[8]) - (m[6]*m[5]);
	n[2] = (m[3]*m[7]) - (m[6]*m[4]);
	n[3] = (m[1]*m[8]) - (m[7]*m[2]);
	n[4] = (m[0]*m[8]) - (m[6]*m[2]);
	n[5] = (m[0]*m[7]) - (m[6]*m[1]);
	n[6] = (m[1]*m[5]) - (m[4]*m[2]);
	n[7] = (m[0]*m[5]) - (m[3]*m[2]);
	n[8] = (m[0]*m[4]) - (m[3]*m[1]);
	
	//get the matrix of cofactors
	n[1] *= -1.0f;
	n[3] *= -1.0f;
	n[5] *= -1.0f;
	n[7] *= -1.0f;
	
	//Adjugate and overwrite m
	m[0] = n[0];
	m[4] = n[4];
	m[8] = n[8];
	m[1] = n[3];
	m[2] = n[6];
	m[5] = n[7];
	m[3] = n[1];
	m[6] = n[2];
	m[7] = n[5];
	
	//Divide by the determinant
	for(i = 0; i < 9; i++)
	{
		m[i] /= idet;
	}
}

float vMagnitude(float * v3)
{
	return ( sqrtf((v3[0]*v3[0]) + (v3[1]*v3[1]) + (v3[2]*v3[2])) );
}

void vNormalize(float * v3)
{
	float mag;
	
	mag = vMagnitude(v3);
	v3[0] = v3[0]/mag;
	v3[1] = v3[1]/mag;
	v3[2] = v3[2]/mag;
}

void vCrossProduct(float * result, float * u, float * v)
{
	result[0] = (u[1]*v[2]) - (u[2]*v[1]);
	result[1] = (u[2]*v[0]) - (u[0]*v[2]);
	result[2] = (u[0]*v[1]) - (u[1]*v[0]);
}

float vDotProduct(float * x3, float * y3)
{
	return ((x3[0]*y3[0]) + (x3[1]*y3[1]) + (x3[2]*y3[2]));
}

void vSubtract(float * result, float * a, float * b)
{
	result[0] = a[0] - b[0];
	result[1] = a[1] - b[1];
	result[2] = a[2] - b[2];
}

void vAdd(float * result, float * a, float * b)
{
	result[0] = a[0] + b[0];
	result[1] = a[1] + b[1];
	result[2] = a[2] + b[2];
}

/*
Returns the normal unit vector of the plane made by the positions u and v.
expects n to be an array of 3 floats.
*/
void vGetPlaneNormal(float * u3, float * v3, float * w3, float * n3)
{
	float v_in_plane_a[3];
	float v_in_plane_b[3];
	float mag;
	
	//Using three positions that are in a plane, calculate two vectors that are inline with the plane
	vSubtract(v_in_plane_a, v3, u3);
	vSubtract(v_in_plane_b, w3, u3);
	
	//Now take the crossproduct to get a vector normal to the plane
	vCrossProduct(n3, v_in_plane_a, v_in_plane_b);
	
	//Make the unit normal vector
	mag = vMagnitude(n3);	
	n3[0] = n3[0]/mag;
	n3[1] = n3[1]/mag;
	n3[2] = n3[2]/mag;
}

/*
return m9
*/
void mmMultiplyMatrix3x3(float * m, float * n, float * r)
{
	float t[9];
	
	t[0] = (m[0]*n[0]) + (m[3]*n[1]) + (m[6]*n[2]);
	t[1] = (m[1]*n[0]) + (m[4]*n[1]) + (m[7]*n[2]);
	t[2] = (m[2]*n[0]) + (m[5]*n[1]) + (m[8]*n[2]);
	
	t[3] = (m[0]*n[3]) + (m[3]*n[4]) + (m[6]*n[5]);
	t[4] = (m[1]*n[3]) + (m[4]*n[4]) + (m[7]*n[5]);
	t[5] = (m[2]*n[3]) + (m[5]*n[4]) + (m[8]*n[5]);
	
	t[6] = (m[0]*n[6]) + (m[3]*n[7]) + (m[6]*n[8]);
	t[7] = (m[1]*n[6]) + (m[4]*n[7]) + (m[7]*n[8]);
	t[8] = (m[2]*n[6]) + (m[5]*n[7]) + (m[8]*n[8]);
	
	memcpy(r, t, (9*sizeof(float)));
}

/*
return m9
*/
void mmAddMatrix3x3(float * m, float * n, float * r)
{
	r[0] = m[0] + n[0];
	r[1] = m[1] + n[1];
	r[2] = m[2] + n[2];
	
	r[3] = m[3] + n[3];
	r[4] = m[4] + n[4];
	r[5] = m[5] + n[5];
	
	r[6] = m[6] + n[6];
	r[7] = m[7] + n[7];
	r[8] = m[8] + n[8];
}

void mmTranspose3x3(float * m)
{
	float r[9];
	
	r[0] = m[0];
	r[4] = m[4];
	r[8] = m[8];
	r[1] = m[3];
	r[2] = m[6];
	r[5] = m[7];
	r[3] = m[1];
	r[6] = m[2];
	r[7] = m[5];
	
	memcpy(m, r, (9*sizeof(float)));
}

void mmTransposeMat4(float * m)
{
	float r[16];

	r[0] = m[0];
	r[5] = m[5];
	r[10] = m[10];
	r[15] = m[15];
	r[1] = m[4];
	r[4] = m[1];
	r[2] = m[8];
	r[8] = m[2];
	r[6] = m[9];
	r[9] = m[6];
	r[3] = m[12];
	r[12] = m[3];
	r[7] = m[13];
	r[13] = m[7];
	r[11] = m[14];
	r[14] = m[11];

	memcpy(m, r, (16*sizeof(float)));
}

int vIsZero(float * v3)
{
	if(v3[0] == 0.0f && v3[1] == 0.0f && v3[2] == 0.0f)
		return 1;
	else
		return 0;
}

/*
Orthonormalize the orientation matrix using Gram-Schmidt orthonormalization.
There is a better way with regard to numerical errors of performing Gram-Schmidt
*/
void Orthonormalize(float * mat9)
{
	float mag[3];
	float a1[3];
	float a2[3];
	float a3[3];
	float b1[3];
	float b2[3];
	float b3[3];
	float temp[3];
	float temp_dot;
	
	a1[0] = mat9[0];
	a1[1] = mat9[3];
	a1[2] = mat9[6];
	
	a2[0] = mat9[1];
	a2[1] = mat9[4];
	a2[2] = mat9[7];
	
	a3[0] = mat9[2];
	a3[1] = mat9[5];
	a3[2] = mat9[8];
	
	b1[0] = a1[0];
	b1[1] = a1[1];
	b1[2] = a1[2];
	
	//calculate b1
	vNormalize(b1);
	
	//calculate b2
	b2[0] = a2[0];
	b2[1] = a2[1];
	b2[2] = a2[2];
	temp_dot = vDotProduct(b1,a2);
	temp[0] = b1[0] * temp_dot;
	temp[1] = b1[1] * temp_dot;
	temp[2] = b1[2] * temp_dot;
	vSubtract(b2, b2, temp);
	vNormalize(b2);
	
	//calculate b3
	b3[0] = a3[0];
	b3[1] = a3[1];
	b3[2] = a3[2];

	temp_dot = vDotProduct(b1,a3);
	temp[0] = b1[0] * temp_dot;
	temp[1] = b1[1] * temp_dot;
	temp[2] = b1[2] * temp_dot;
	vSubtract(b3, b3, temp);

	temp_dot = vDotProduct(b2,a3);
	temp[0] = b2[0] * temp_dot;
	temp[1] = b2[1] * temp_dot;
	temp[2] = b2[2] * temp_dot;
	vSubtract(b3, b3, temp);
	vNormalize(b3);
	
	//put the orthonormalized vectors back in the matrix
	mat9[0] = b1[0];
	mat9[1] = b2[0];
	mat9[2] = b3[0];
	mat9[3] = b1[1];
	mat9[4] = b2[1];
	mat9[5] = b3[1];
	mat9[6] = b1[2];
	mat9[7] = b2[2];
	mat9[8] = b3[2];
}

float vDotProduct2(float * x2, float * y2)
{
	return ((x2[0]*y2[0]) + (x2[1]*y2[1]));
}

void vNormalize2(float * v2)
{
	float mag;
	mag = sqrtf((v2[0]*v2[0]) + (v2[1]*v2[1]));
	v2[0] /= mag;
	v2[1] /= mag;
}

void qConvertToMat3(float * q, float * m9)
{
	m9[0] = 1 - (2*q[1]*q[1]) - (2*q[2]*q[2]);
	m9[1] = (2*q[0]*q[1]) + (2*q[3]*q[2]);
	m9[2] = (2*q[0]*q[2]) - (2*q[3]*q[1]);

	m9[3] = (2*q[0]*q[1]) - (2*q[3]*q[2]);
	m9[4] = 1 - (2*q[0]*q[0]) - (2*q[2]*q[2]);
	m9[5] = (2*q[1]*q[2]) + (2*q[3]*q[0]);

	m9[6] = (2*q[0]*q[2]) + (2*q[3]*q[1]);
	m9[7] = (2*q[1]*q[2]) - (2*q[3]*q[0]);
	m9[8] = 1 - (2*q[0]*q[0]) - (2*q[1]*q[1]);
}

void qMultiply(float *r, float * q0, float * q1)
{
	float temp[4];

	//scalar part
	temp[3] = (q0[3]*q1[3]) - vDotProduct(q0, q1);

	//vector part
	//(w0*v1) + (w1*v0) + (v0 <cross> v1)
	temp[0] = (q0[3]*q1[0]) + (q1[3]*q0[0]) + ((q0[1]*q1[2])-(q0[2]*q1[1]));
	temp[1] = (q0[3]*q1[1]) + (q1[3]*q0[1]) + ((q0[2]*q1[0]) - (q0[0]*q1[2]));
	temp[2] = (q0[3]*q1[2]) + (q1[3]*q0[2]) + ((q0[0]*q1[1]) - (q0[1]*q1[0]));

	memcpy(r, temp, (4*sizeof(float)));
}

void qAdd(float * r, float * q0, float * q1)
{
	r[0] = q0[0] + q1[0];
	r[1] = q0[1] + q1[1];
	r[2] = q0[2] + q1[2];
	r[3] = q0[3] + q1[3];
}

float qMagnitude(float * q)
{
	return(sqrtf((q[3]*q[3])+(q[0]*q[0])+(q[1]*q[1])+(q[2]*q[2])));
}

void qNormalize(float * q)
{
	float mag;

	mag = qMagnitude(q);
	if(mag != 0.0f);
	{
		q[0] /= mag;
		q[1] /= mag;
		q[2] /= mag;
		q[3] /= mag;
	}
}

void qInvert(float * q)
{
	float scalar;

	scalar = (q[3]*q[3]) + (q[2]*q[2]) + (q[1]*q[1]) + (q[0]*q[0]);
	q[3] = q[3]/scalar;

	scalar *= -1.0f;
	q[0] /= scalar;
	q[1] /= scalar;
	q[2] /= scalar;
}

//v is a vec4
void qRotate(float * q, float * v4)
{
	float inverseQ[4];

	memcpy(inverseQ, q, 4*sizeof(float));
	qInvert(inverseQ);
	qMultiply(v4, q, v4);
	qMultiply(v4, v4, inverseQ);
}

void qCreate(float * q, float * v3, float deg_angle)
{
	float rads;
	float sine_f;

	rads = RATIO_DEGTORAD*deg_angle;
	q[0] = v3[0];
	q[1] = v3[1];
	q[2] = v3[2];
	
	q[3] = cosf(rads*0.5f);
	
	sine_f = sinf(rads*0.5f);
	q[0] *= sine_f;
	q[1] *= sine_f;
	q[2] *= sine_f;
}
