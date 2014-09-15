/**** Decompose.h - Basic declarations ****/
#pragma once

typedef struct {float x, y, z, w;} DecomposeQuat; /* Quaternion */
enum QuatPart {X, Y, Z, W};
typedef DecomposeQuat HVect; /* Homogeneous 3D vector */
typedef float HMatrix[4][4]; /* Right-handed, for column vectors */
typedef struct {
    HVect t;	/* Translation components */
    DecomposeQuat  q;	/* Essential rotation	(quaternion form)  */
    HMatrix q_mat;  /* Essential rotation (matrix form)  */
    DecomposeQuat  u;	/* Stretch rotation	  */
    HVect k;	/* Stretch factors	  */
    float f;	/* Sign of determinant	  */
} AffineParts;

float polar_decomp(HMatrix M, HMatrix Q, HMatrix S);
HVect spect_decomp(HMatrix S, HMatrix U);
DecomposeQuat snuggle(DecomposeQuat q, HVect *k);
void decomp_affine(HMatrix A, AffineParts *parts);
void invert_affine(AffineParts *parts, AffineParts *inverse);
