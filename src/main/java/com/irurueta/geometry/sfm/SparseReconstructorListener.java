/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.sfm.SparseReconstructorListener
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date December 26, 2016.
 */
package com.irurueta.geometry.sfm;

import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.epipolar.FundamentalMatrix;
import java.util.List;

/**
 * Listener to retrieve and store required data to compute a 3D reconstruction
 * from sparse image point correspondences.
 */
public interface SparseReconstructorListener {
    
    /**
     * Indicates whether there are more views available to add to the 
     * reconstruction.
     * @param reconstructor reconstructor raising this event.
     * @return true if there are more views available, false otherwise.
     */
    boolean hasMoreViewsAvailable(SparseReconstructor reconstructor);
    
    /**
     * Called when samples containing points of interest for current view must 
     * be attempted.
     * @param reconstructor reconstructor raising this event.
     * @param viewId id of view where points will be used.
     * @param samples samples containing points of interest for current view to 
     * test.
     * @return true if provided points can be used for reconstruction, false
     * if points are in a degenerate configuration and cannot be used.
     */
    boolean onAttemptSamplesForCurrentView(SparseReconstructor reconstructor, 
            int viewId, List<Sample2D> samples);
    
    /**
     * Called when a fundamental matrix relating two views has been estimated.
     * This event can be used to store estimated fundamental matrix relating
     * two views.
     * @param reconstructor reconstructor raising this event.
     * @param viewId1 id of first view.
     * @param viewId2 id of second view.
     * @param fundamentalMatrix estimated fundamental matrix.
     */
    void onFundamentalMatrixEstimated(SparseReconstructor reconstructor,
            int viewId1, int viewId2, FundamentalMatrix fundamentalMatrix);
    
    /**
     * Notifies when provided 2D points on a given view are found to be of
     * interest. This event can be used to store points associated to such view.
     * @param reconstructor reconstructor raising this event.
     * @param points points to be stored.
     * @param viewId id of view for which points must be associated to.
     */
    void onValidImagePointsFound(SparseReconstructor reconstructor, 
            List<Point2D> points, int viewId);
    
    /**
     * Notifies when a camera for a given view has been estimated. This event
     * can be used to store the estimated camera associated to such view.
     * @param reconstructor reconstructor raising this event.
     * @param viewId id of view where the camera belongs to.
     * @param camera estimated camera.
     */
    void onCameraEstimated(SparseReconstructor reconstructor, int viewId, 
            PinholeCamera camera);
    
    /**
     * Called when matches for provided samples on provided view must be found
     * on other views.
     * @param reconstructor reconstructor raising this event.
     * @param samples samples for which matches on other views must be found.
     * @param viewId id of view where samples belong to.
     * @param matches instance where found matches must be provided.
     */
    void onRequestMatches(SparseReconstructor reconstructor, 
            List<Sample2D> samples, int viewId, List<MatchedSamples> matches);
        
    /**
     * Called when reconstructed points have been estimated from a series of
     * 2D matches. This event can be used to store reconstructed points and
     * their associated data.
     * @param reconstructor reconstructor raising this event.
     * @param matches 2D matches associated to estimated reconstructed 
     * points.
     * @param points reconstructed 3D points.
     */
    void onReconstructedPointsEstimated(SparseReconstructor reconstructor, 
            List<MatchedSamples> matches, List<ReconstructedPoint3D> points);
    
    /**
     * Called when reconstruction starts.
     * @param reconstructor reconstructor raising this event.
     */
    void onStart(SparseReconstructor reconstructor);
    
    /**
     * Called when reconstruction stops.
     * @param reconstructor reconstructor raising this event.
     */
    void onFinish(SparseReconstructor reconstructor);
    
    /**
     * Called when reconstruction fails.
     * @param reconstructor reconstructor raising this event.
     */
    void onFail(SparseReconstructor reconstructor);
    
    /**
     * Called when a new color data structure needs to be provided.
     * @param reconstructor reconstructor raising this event.
     * @return a new color data structure.
     */
    PointColorData onCreateColorData(SparseReconstructor reconstructor);
}
