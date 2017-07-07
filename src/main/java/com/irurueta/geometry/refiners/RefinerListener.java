/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.refiners.RefinerListener
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date March 29, 2017.
 */
package com.irurueta.geometry.refiners;

/**
 * Listener for a refiner.
 * @param <T> type of instance to be refined.
 */
public interface RefinerListener<T> {
    
    /**
     * Called when refinement starts.
     * @param refiner refiner that raised the event.
     * @param initialEstimation initial estimation before refinement.
     */
    void onRefineStart(Refiner<T> refiner, T initialEstimation);
    
    /**
     * Called when refinement finishes successfully.
     * @param refiner refiner that raised the event.
     * @param initialEstimation initial estimation before refinement.
     * @param result refined instance.
     * @param errorDecreased true if error decreased after refinemnt, false
     * otherwise.
     */
    void onRefineEnd(Refiner<T> refiner, T initialEstimation, T result, 
            boolean errorDecreased);
}
