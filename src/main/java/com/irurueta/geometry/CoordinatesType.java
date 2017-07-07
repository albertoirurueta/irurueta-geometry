/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.CoordinatesType
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date June 24, 2012
 */
package com.irurueta.geometry;

/**
 * Enumerator that indicates the type of coordinates used to represent a point
 */
public enum CoordinatesType {
    
    /**
     * Homogeneous coordinates: coordinates x, y and w must be given.
     */
    HOMOGENEOUS_COORDINATES,
    
    /**
     * Inhomogeneous coordinates: coordinates x and y (w is assumed to be 1).
     */
    INHOMOGENEOUS_COORDINATES
}
