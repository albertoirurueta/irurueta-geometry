/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.sfm.Point3DTriangulatorType
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 29, 2015
 */
package com.irurueta.geometry.sfm;

/**
 * Type of 3D point triangulator.
 * Homogeneous methods are suitable for any case
 * Inhomogeneous ones are suitable only for cases where finite points and
 * cameras are being used. If points or cameras are located very far or at
 * infinity, triangulation will fail when using inhomogeneous methods
 */
public enum Point3DTriangulatorType {
    /**
     * Triangulator using homogeneous method and an LMSE (Least Mean Square 
     * Error) solution
     */
    LMSE_HOMOGENEOUS_TRIANGULATOR,
    
    /**
     * Triangulator using inhomogeneous method and an LMSE (Least Mean Square
     * Error) solution
     */
    LMSE_INHOMOGENEOUS_TRIANGULATOR,
    
    /**
     * Triangulator using homogeneous method and a weighted solution
     */
    WEIGHTED_HOMOGENEOUS_TRIANGULATOR,
    
    /**
     * Triangulator using inhomogeneous method and a weighted solution
     */
    WEIGHTED_INHOMOGENEOUS_TRIANGULATOR
}
