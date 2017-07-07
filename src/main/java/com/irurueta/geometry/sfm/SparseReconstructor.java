/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.sfm.SparseReconstructor
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date March 12, 2017.
 */
package com.irurueta.geometry.sfm;

import java.util.ArrayList;
import java.util.List;

/**
 * Class in charge of estimating cameras and 3D reconstruction points from
 * sparse image point correspondences.
 */
public class SparseReconstructor {
    
    /**
     * Listener in charge of handling events such as when reconstruction starts,
     * ends, when certain data is needed or when estimation of data has been 
     * computed so that estimated data can be stored.
     */
    private SparseReconstructorListener mListener;
    
    /**
     * Indicates whether reconstruction is running or not.
     */
    private volatile boolean mRunning;

    /**
     * Indicates whether reconstruction has been cancelled or not.
     */
    private volatile boolean mCancelled;        
    
    /**
     * Indicates whether reconstruction has failed or not.
     */
    private volatile boolean mFailed;
    
    /**
     * Counter of number of processed views.
     */
    private int mViewCount;
    
    /**
     * Current fundamental matrix estimation.
     */
    private EstimatedFundamentalMatrix mCurrentEstimatedFundamentalMatrix;
    
    /**
     * Constructor.
     * @param listener listener in charge of handling events.
     * @throws NullPointerException if listener is not provided.
     */
    public SparseReconstructor(SparseReconstructorListener listener) 
            throws NullPointerException {
        mListener = listener;
    }
    
    /**
     * Indicates whether reconstruction is running or not.
     * @return true if reconstruction is running, false if reconstruction has 
     * stopped for any reason.
     */
    public synchronized boolean isRunning() {
        return mRunning;
    }

    /**
     * Indicates whether reconstruction has been cancelled or not.
     * @return true if reconstruction has been cancelled, false otherwise.
     */
    public synchronized boolean isCancelled() {
        return mCancelled;
    }
    
    /**
     * Indicates whether reconstruction has failed or not.
     * @return true if reconstruction has failed, false otherwise.
     */
    public synchronized boolean hasFailed() {
        return mFailed;
    }    
    
    /**
     * Gets counter of number of processed views.
     * @return counter of number of processed views.
     */
    public int getViewCount() {
        return mViewCount;
    }
    
    /**
     * Starts reconstruction.
     * If reconstruction has already started and is running, calling this method
     * has no effect.
     * @throws FailedReconstructionException if reconstruction fails for some 
     * reason.
     * @throws CancelledReconstructionException if reconstruction is cancelled.
     */
    public void start() throws FailedReconstructionException, 
            CancelledReconstructionException{
        synchronized(this) {
            if (mRunning) {
                //already started
                return;
            }
            
            mCancelled = mFailed = false;
            mViewCount = 0;
            mRunning = true;
        }
        
        List<Sample2D> tmp;
        List<Sample2D> previousViewSamples = new ArrayList<Sample2D>();
        List<Sample2D> currentViewSamples = new ArrayList<Sample2D>();
        while (mListener.hasMoreViewsAvailable(this)) {
            
            mCurrentEstimatedFundamentalMatrix = null;
            currentViewSamples.clear();
            mListener.onAttemptSamplesForCurrentView(this, mViewCount, 
                    currentViewSamples);
            
            if (mViewCount == 0) {
                //on first view skip remaining processing
                tmp = currentViewSamples;
                currentViewSamples = previousViewSamples;
                previousViewSamples = tmp;
                continue;
            }
            
            //determine fundamental matrix with previous samples
            estimateFundamentalMatrix(previousViewSamples, 
                            currentViewSamples);
            //InitialCamerasEstimator
            mViewCount++;
        }
    }
    
    /**
     * Cancels reconstruction.
     * If reconstruction has already been cancelled, calling this method has no 
     * effect.
     */
    public synchronized void cancel() {
        if (mCancelled) {
            //already cancelled
            return;
        }
        
        mCancelled = true;
    }
    
    /**
     * Gets listener in charge of handling events such as when reconstruction 
     * starts, ends, when certain data is needed or when estimation of data has
     * been computed so that estimated data can be stored.
     * @return listener in charge of handling events.
     */
    protected SparseReconstructorListener getListener() {
        return mListener;
    }    
    
    private void estimateFundamentalMatrix(List<Sample2D> previousSamples,
            List<Sample2D> currentSamples) {
        //mCurrentEstimatedFundamentalMatrix
    }
}
