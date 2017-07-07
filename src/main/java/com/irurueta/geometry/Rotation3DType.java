/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.Rotation3DType
 * 
 * @auhtor Alberto Irurueta (alberto@irurueta.com)
 * @date September 11, 2012
 */
package com.irurueta.geometry;

/**
 *  Enumeration defining type of rotation 3D.
 */
public enum Rotation3DType {
    /**
     * Rotation based on axis and angle of rotation.
     */
    AXIS_ROTATION3D,
    
    /**
     * Rotation based on a 3x3 orthonormal matrix.
     */
    MATRIX_ROTATION3D,
    
    /**
     * Rotation based on quaternions.
     */
    QUATERNION
}
