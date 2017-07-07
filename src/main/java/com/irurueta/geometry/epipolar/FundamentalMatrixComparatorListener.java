/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.epipolar.FundamentalMatrixComparatorListener
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 28, 2015
 */
package com.irurueta.geometry.epipolar;

/**
 * Handles events produced by a FundamentalMatrixComparator
 */
public interface FundamentalMatrixComparatorListener {
    
    /**
     * Called when comparison starts
     * @param comparator instance that raised the event
     */
    public void onCompareStart(FundamentalMatrixComparator comparator);
    
    /**
     * Called when comparison finishes
     * @param comparator instance that raised the event
     */
    public void onCompareEnd(FundamentalMatrixComparator comparator);
    
    /**
     * Called when progress of comparison significantly changes
     * @param comparator instance that riased the event
     * @param progress progress of comparison expressed as a value between 0.0
     * and 1.0
     */
    public void onCompareProgressChange(FundamentalMatrixComparator comparator, 
            float progress);
}
