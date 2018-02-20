#ifndef MY_MAT_MATH
#define MY_MAT_MATH

#define RATIO_DEGTORAD 0.017453293f
#define RATIO_RADTODEG 57.295779513f

/*Global matrix functions*/
void mmTranslateMatrix(float * m16, float x, float y, float z);
void mmRotateAboutX(float * m16, float degs);
void mmRotateAboutXRad(float * m16, float rads);
void mmRotateAboutY(float * m16, float degs);
void mmRotateAboutYRad(float * m16, float rads);
void mmRotateAboutZ(float * m16, float rads);
void mmMakeIdentityMatrix(float * m16);
void mmConvertMat4toMat3(float * m16, float * m9);
void mmMultiplyMatrix4x4(float * m, float * n, float * r);
void mmInverse4x4(float * m);
float mmDeterminant4x4(float * m);
void mmTransformVec(float * m, float * v); //This is the transform for vec4 
void mmTransformVec3(float * m, float * v);
void mmMultiplyMatrix3x3(float * m, float * n, float * r);
void mmAddMatrix3x3(float * m, float * n, float * r);
float mmDeterminant3x3(float * m);
void mmInverse3x3(float * m);
void mmTranspose3x3(float * m);
void mmTransposeMat4(float * m);
void Orthonormalize(float * mat9);

/*Global Vec3 functions*/
float vMagnitude(float * v3);
void vNormalize(float * v3);
void vCrossProduct(float * result, float * u, float * v);
float vDotProduct(float * x3, float * y3);
void vSubtract(float * result, float * a, float * b);
void vAdd(float * result, float * a, float * b);
void vGetPlaneNormal(float * u3, float * v3, float * w3, float * n3);
int vIsZero(float * v3);

/*Vec2 functions*/
float vDotProduct2(float * x2, float * y2);
void vNormalize2(float * v2);

/*
quaternion functions
-where quaternions stored as (x,y,z,w) in memory
*/
void qConvertToMat3(float * q, float * m9);
void qMultiply(float * r, float * q0, float * q1);
void qAdd(float * r, float * q0, float * q1);
float qMagnitude(float * q);
void qNormalize(float * q);
void qInvert(float * q);
void qRotate(float * q, float * v4);
void qCreate(float * q, float * v3, float deg_angle);

#endif
