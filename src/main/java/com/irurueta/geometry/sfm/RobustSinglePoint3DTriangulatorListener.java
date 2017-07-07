/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.sfm.RobustSinglePoint3DTriangulatorListener
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 30, 2015
 */
package com.irurueta.geometry.sfm;

/**
 * Listener to be notified of events such as when triangulation starts, ends or
 * when progress changes
 */
public interface RobustSinglePoint3DTriangulatorListener {
    
    /**
     * Called when triangulation starts
     * @param triangulator reference to robust triangulator
     */
    public void onTriangulateStart(
            RobustSinglePoint3DTriangulator triangulator);
    
    /**
     * Called when triangulation ends
     * @param triangulator reference to robust triangulator
     */
    public void onTriangulateEnd(RobustSinglePoint3DTriangulator triangulator);
    
    /**
     * Called when robust triangulator iterates to refine a possible solution
     * @param triangulator reference to robust triangulator
     * @param iteration current iteration
     */
    public void onTriangulateNextIteration(
            RobustSinglePoint3DTriangulator triangulator, int iteration);
    
    /**
     * Called when estimation progress changes significantly
     * @param triangulator reference to robust triangulator
     * @param progress progress of estimation expressed as a value between 0.0
     * and 1.0
     */
    public void onTriangulateProgressChange(
            RobustSinglePoint3DTriangulator triangulator, float progress);
}
