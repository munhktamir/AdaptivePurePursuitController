#ifndef GEOMETRY_H
#define GEOMETRY_H

typedef struct translation2d {
    double x_in;
    double y_in;
} translation2d_t;

typedef struct rotation2d {
    double sinTheta_rad;
    double cosTheta_rad;
} rotation2d_t;

typedef struct transform2d {
    translation2d_t translation;
    rotation2d_t rotation;
} transform2d_t;

typedef struct twist2d {
    double dx_in;
    double dy_in;
    double dtheta_rad;
} twist2d_t;

double TranslationNormal (translation2d_t *trans);
double TranslationCross (translation2d_t *transA, translation2d_t *transB);
double TranslationDot (translation2d_t *transA, translation2d_t *transB);
translation2d_t TranslationScale (translation2d_t *trans, double scale);
translation2d_t TranslationDelta (translation2d_t *start, translation2d_t *end);
translation2d_t TranslateAbyB (translation2d_t *transA, translation2d_t *transB);
translation2d_t TranslationRotate (translation2d_t *trans, rotation2d_t *rot);
double TranslationGetAngle (translation2d_t *transA, translation2d_t *transB);
translation2d_t TranslationInverse (translation2d_t *trans);
translation2d_t TranslationInterpolate (translation2d_t *start, translation2d_t *end, double scale);
rotation2d_t TranslationDirection (translation2d_t *trans);
rotation2d_t RotationNormalize (rotation2d_t *rot);
rotation2d_t RotationNormal (rotation2d_t *rot);
translation2d_t RotationToTranslation (rotation2d_t *rot);
rotation2d_t RotateAbyB (rotation2d_t *rotA, rotation2d_t *rotB);
rotation2d_t RotationInverse (rotation2d_t *rot);
int IsParallel (rotation2d_t *rotA, rotation2d_t *rotB);
double Tan (rotation2d_t *rot);
translation2d_t Intersection (transform2d_t *tfrmA, transform2d_t *tfrmB);
transform2d_t Exp (twist2d_t *delta);
twist2d_t Log (transform2d_t *tfrm);
transform2d_t TranformAByB (transform2d_t *tfrmA, transform2d_t *tfrmB);
transform2d_t TransformInverse (transform2d_t *tfrm);
int IsColinear (transform2d_t *tfrmA, transform2d_t *tfrmB);
transform2d_t TransformNormal (transform2d_t *tfrm);



#endif