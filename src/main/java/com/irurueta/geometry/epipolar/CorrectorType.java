/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.epipolar.CorrectorType
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 27, 2015
 */
package com.irurueta.geometry.epipolar;

/**
 * Type of corrector to fix point matches under a given epipolar geometry
 */
public enum CorrectorType {
    /**
     * Corrector that uses Sampson approximation
     */
    SAMPSON_CORRECTOR,
    
    /**
     * Corrector that uses Gold Standard method
     */
    GOLD_STANDARD
}
