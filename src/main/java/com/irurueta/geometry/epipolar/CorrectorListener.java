/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.epipolar.CorrectorListener
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 28, 2015
 */
package com.irurueta.geometry.epipolar;

/**
 * Interface to handle events generated by Corrector instances
 */
public interface CorrectorListener {
    
    /**
     * Called when correction of points starts
     * @param corrector instance that generated the event
     */
    public void onCorrectStart(Corrector corrector);
    
    /**
     * Called when correction of points finishes
     * @param corrector instance that generated the event
     */
    public void onCorrectEnd(Corrector corrector);
    
    /**
     * Called when progress of correction of points changes significantly
     * @param corrector instance that generated the event
     * @param progress progress expressed as a value between 0 and 1
     */
    public void onCorrectProgressChange(Corrector corrector, float progress);
}