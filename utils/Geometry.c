#include <math.h>
#include "Geometry.h"
#include "Utils.h"

/******************************************************************************************************************************** 
**
********************************************************************************************************************************/
double TranslationNormal (translation2d_t *trans) {
    double rv;
    
    rv = sqrt( pow( trans->x_in, 2 ) + pow( trans->y_in, 2 ) );
    return rv;
}

double TranslationCross (translation2d_t *transA, translation2d_t *transB) {
    double rv;
    
    rv = transA->x_in * transB->y_in - transA->y_in * transB->x_in;
    return rv;
}

double TranslationDot (translation2d_t *transA, translation2d_t *transB) {
    double rv;
    
    rv = transA->x_in * transB->x_in + transA->y_in * transB->y_in;
    return rv;
}

translation2d_t TranslationScale (translation2d_t *trans, double scale) {
    translation2d_t rv;

    rv.x_in = trans->x_in * scale;
    rv.y_in = trans->y_in * scale;
    return rv;
}

translation2d_t TranslationDelta (translation2d_t *start, translation2d_t *end) {
    translation2d_t rv;

    rv.x_in = end->x_in - start->x_in;
    rv.y_in = end->y_in - start->y_in;
    return rv;
}

translation2d_t TranslateAbyB (translation2d_t *transA, translation2d_t *transB) {
    translation2d_t rv;

    rv.x_in = transA->x_in + transB->x_in;
    rv.y_in = transA->y_in + transB->y_in;
    return rv;
}

translation2d_t TranslationRotate (translation2d_t *trans, rotation2d_t *rot) {
    translation2d_t rv;

    rv.x_in = trans->x_in * rot->cosTheta_rad - trans->y_in * rot->sinTheta_rad; 
    rv.y_in = trans->x_in * rot->sinTheta_rad + rot->cosTheta_rad;
    return rv;
  }

double TranslationGetAngle (translation2d_t *transA, translation2d_t *transB) {
    double cosAngle_rad;
    
    cosAngle_rad = TranslationDot( transA, transB ) / ( TranslationNormal( transA ) * TranslationNormal( transB ) );
    if( isnan( cosAngle_rad ) ) {
        return 0.0;
    }
    return acos( fmin( 1.0, fmax( cosAngle_rad, -1.0 ) ) );
  }

translation2d_t TranslationInverse (translation2d_t *trans) {
    translation2d_t rv;

    rv.x_in = -trans->x_in;
    rv.y_in = -trans->y_in;
    return rv;
}

translation2d_t TranslationInterpolate (translation2d_t *start, translation2d_t *end, double scale) {
    translation2d_t rv;

    rv = *start;
    if ( scale >= 1.0 ) {
        rv = *end;    
    } else if ( scale >= 0 ) {
        rv.x_in = scale * (end->x_in - start->x_in);
        rv.y_in = scale * (end->y_in - start->y_in);
    }
    return rv;
}

rotation2d_t TranslationDirection (translation2d_t *trans) {
    rotation2d_t rv;
    
    rv.cosTheta_rad = trans->x_in;
    rv.sinTheta_rad = trans->y_in;
    rv = RotationNormalize( &rv );
    return rv;
}


/******************************************************************************************************************************** 
**
********************************************************************************************************************************/
rotation2d_t RotationNormalize (rotation2d_t *rot) {
    rotation2d_t rv;
    double mag;

    mag = sqrt( pow( rot->cosTheta_rad, 2 ) + pow( rot->sinTheta_rad, 2 ) );
    if ( mag > 1e-9 ) {
        rv.cosTheta_rad = rot->cosTheta_rad / mag;
        rv.sinTheta_rad = rot->sinTheta_rad / mag;
    } else {
        rv.cosTheta_rad = 1.0;
        rv.sinTheta_rad = 0.0;
    }
    return rv;
}


rotation2d_t RotationNormal (rotation2d_t *rot) {
    rotation2d_t rv;

    rv.cosTheta_rad = -rot->sinTheta_rad;
    rv.sinTheta_rad = rot->cosTheta_rad;
    return rv;
}


translation2d_t RotationToTranslation (rotation2d_t *rot) {
    translation2d_t rv;

    rv.x_in = rot->cosTheta_rad;
    rv.y_in = rot->sinTheta_rad;
    return rv;
}  


rotation2d_t RotateAbyB (rotation2d_t *rotA, rotation2d_t *rotB) {
    rotation2d_t rv;

    rv.cosTheta_rad = rotA->cosTheta_rad * rotB->cosTheta_rad - rotA->sinTheta_rad * rotB->sinTheta_rad;
    rv.sinTheta_rad = rotA->cosTheta_rad * rotB->sinTheta_rad + rotA->sinTheta_rad * rotB->cosTheta_rad;
    rv = RotationNormalize( &rv );
    return rv;
}


rotation2d_t RotationInverse (rotation2d_t *rot) {
    rotation2d_t rv;

    rv.cosTheta_rad = rot->cosTheta_rad;
    rv.sinTheta_rad = -rot->sinTheta_rad;
    return rv;    
}


int IsParallel(rotation2d_t *rotA, rotation2d_t *rotB) {    
    translation2d_t transA, transB;
    int rv;

    transA = RotationToTranslation( rotA );
    transB = RotationToTranslation( rotB );
    rv = EpsilonEquals( TranslationCross( &transA, &transB ), 0.0, 1e-9);
    return rv;
}


double Tan (rotation2d_t *rot) {
    double rv;
    
    if ( fabs( rot->cosTheta_rad ) < 1e-9 ) {
        if ( rot->sinTheta_rad >= 0.0 ) {
            rv = INFINITY;
        } else {
            rv = -INFINITY;
        }
        return rv;
    }
    rv = rot->sinTheta_rad / rot->cosTheta_rad;
    return rv;
}
  

/******************************************************************************************************************************** 
**
********************************************************************************************************************************/
translation2d_t Intersection (transform2d_t *tfrmA, transform2d_t *tfrmB) {
    translation2d_t rv;
    double scale, tangent;

    if ( IsParallel( &tfrmA->rotation, &tfrmB->rotation ) ) {
        rv.x_in = INFINITY;
        rv.y_in = INFINITY;    

    } else if ( fabs( tfrmA->rotation.cosTheta_rad ) < fabs( tfrmB->rotation.cosTheta_rad ) ) {
        tangent = Tan( &tfrmB->rotation );
        scale = ( ( tfrmA->translation.x_in - tfrmB->translation.x_in ) * tangent + tfrmB->translation.y_in - tfrmA->translation.y_in ) / ( tfrmA->rotation.sinTheta_rad - tfrmA->rotation.cosTheta_rad * tangent );
        rv = RotationToTranslation( &tfrmA->rotation );
        rv = TranslationScale( &rv, scale );
        rv = TranslateAbyB( &tfrmA->translation, &rv );

    } else {
        tangent = Tan( &tfrmA->rotation );
        scale = ( ( tfrmB->translation.x_in - tfrmA->translation.x_in ) * tangent + tfrmA->translation.y_in - tfrmB->translation.y_in ) / ( tfrmB->rotation.sinTheta_rad - tfrmB->rotation.cosTheta_rad * tangent );
        rv = RotationToTranslation( &tfrmB->rotation );
        rv = TranslationScale( &rv, scale );
        rv = TranslateAbyB( &tfrmB->translation, &rv );
    }
    return rv;
}



//  Obtain a new RigidTransform2d from a (constant curvature) velocity. See:
//  https://github.com/strasdat/Sophus/blob/master/sophus/se2.hpp
transform2d_t Exp (twist2d_t *delta) {
    transform2d_t rv;
    rotation2d_t rot;
    translation2d_t trans;
    double sinTheta, cosTheta, c, s;
    
    sinTheta = sin( delta->dtheta_rad );
    cosTheta = cos( delta->dtheta_rad );
    if ( fabs( delta->dtheta_rad ) < 1E-9 ) {
        s = 1.0 - 1.0 / 6.0 * delta->dtheta_rad * delta->dtheta_rad;
        c = .5 * delta->dtheta_rad;

    } else {
        s = sinTheta / delta->dtheta_rad;
        c = (1.0 - cosTheta) / delta->dtheta_rad;

    }
    rot.cosTheta_rad = cosTheta;
    rot.sinTheta_rad = sinTheta;
    trans.x_in = delta->dx_in * s - delta->dy_in * c;
    trans.y_in = delta->dx_in * c + delta->dy_in * s;
    rv.translation = trans;
    rv.rotation = rot;

    return rv;
 }

//  Logical inverse of the above.
twist2d_t Log (transform2d_t *tfrm) {
    twist2d_t rv;
    rotation2d_t rot;
    translation2d_t trans;
    double dTheta, halfdTheta, cosMinusOne, halfThetaByTanOfHalfdTheta;
    
    dTheta = atan2( tfrm->rotation.sinTheta_rad, tfrm->rotation.cosTheta_rad );
    halfdTheta = 0.5 * dTheta;
    cosMinusOne = tfrm->rotation.cosTheta_rad - 1.0;
    if ( fabs( cosMinusOne ) < 1E-9 ) {
        halfThetaByTanOfHalfdTheta = 1.0 - 1.0 / 12.0 * dTheta * dTheta;
    } else {
        halfThetaByTanOfHalfdTheta = -( halfdTheta * tfrm->rotation.sinTheta_rad ) / cosMinusOne;
    }
    rot.cosTheta_rad = halfThetaByTanOfHalfdTheta;
    rot.sinTheta_rad = -halfdTheta;
    trans = TranslationRotate( &tfrm->translation, &rot );
    rv.dx_in = trans.x_in;
    rv.dy_in = trans.y_in;
    rv.dtheta_rad = dTheta;
    
    return rv;
}


//  Transforming means first translating a by b translation and then rotating a by b rotation
transform2d_t TranformAByB (transform2d_t *tfrmA, transform2d_t *tfrmB) {
    transform2d_t rv;
    translation2d_t trans;
    rotation2d_t rot;
   
    trans = TranslationRotate( &tfrmB->translation, &tfrmA->rotation );
    trans = TranslateAbyB( &tfrmA->translation, &trans );
    rot = RotateAbyB( &tfrmA->rotation, &tfrmB->rotation );
    rv.translation = trans;
    rv.rotation = rot;

    return rv;
}


transform2d_t TransformInverse (transform2d_t *tfrm) {
    transform2d_t rv;
    rotation2d_t invertedRot;
    translation2d_t invertedTrans;

    invertedRot = RotationInverse( &tfrm->rotation );
    invertedTrans = TranslationInverse( &tfrm->translation );
    invertedTrans = TranslationRotate( &invertedTrans, &invertedRot );
    rv.translation = invertedTrans;
    rv.rotation = invertedRot;
    return rv;
}


int IsColinear (transform2d_t *tfrmA, transform2d_t *tfrmB) {
    int rv;
    transform2d_t invertedTfrm;
    twist2d_t twist;
    
    invertedTfrm = TransformInverse( tfrmA );
    invertedTfrm = TranformAByB( &invertedTfrm, tfrmB);
    twist = Log( &invertedTfrm );    
    rv = ( EpsilonEquals(twist.dx_in, 0.0, 1e-9 ) && EpsilonEquals( twist.dtheta_rad, 0.0, 1e-9 ) );
    return rv;
  }

transform2d_t TransformNormal (transform2d_t *tfrm) {
    transform2d_t rv;

    rv.translation = tfrm->translation;
    rv.rotation = RotationNormal( &tfrm->rotation );
    return rv;
}

