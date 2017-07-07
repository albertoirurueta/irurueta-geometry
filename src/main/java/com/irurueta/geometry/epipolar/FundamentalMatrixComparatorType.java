/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.epipolar.FundamentalMatrixComparatorType
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 28, 2015
 */
package com.irurueta.geometry.epipolar;

/**
 * Indicates method used to compare fundamental matrices
 */
public enum FundamentalMatrixComparatorType {
    /**
     * Comparator based on pure algebraic comparison.
     * Has no geometric meaning
     */
    ALGEBRAIC_COMPARATOR,
    
    /**
     * Comparator based on distances to epipolar lines assuming a certain
     * image size.
     * Has geometric meaning
     */
    EPIPOLAR_DISTANCE_COMPARATOR
}
