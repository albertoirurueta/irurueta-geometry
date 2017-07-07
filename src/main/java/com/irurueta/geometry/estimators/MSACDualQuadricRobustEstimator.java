/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.estimators.MSACDualQuadricRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date February 24, 2015
 */
package com.irurueta.geometry.estimators;

import com.irurueta.geometry.CoincidentPlanesException;
import com.irurueta.geometry.DualQuadric;
import com.irurueta.geometry.Plane;
import com.irurueta.numerical.robust.MSACRobustEstimator;
import com.irurueta.numerical.robust.MSACRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import java.util.List;

/**
 * Finds the best dual quadric for provided collection of 3D planes using MSAC
 * algorithm
 */
public class MSACDualQuadricRobustEstimator extends DualQuadricRobustEstimator{
    /**
     * Constant defining default threshold to determine whether planes are 
     * inliers or not.
     * Threshold is defined by the equation abs(trans(P) * dQ * P) &lt; t, where
     * trans is the transposition, P is a plane, dQ is a dual quadric and t is a 
     * threshold.
     * This equation determines the planes P belonging to the locus of a dual 
     * quadric dQ up to a certain threshold.
     */
    public static final double DEFAULT_THRESHOLD = 1e-7;
        
    /**
     * Minimum value that can be set as threshold.
     * Threshold must be strictly greater than 0.0
     */
    public static final double MIN_THRESHOLD = 0.0;
    
    /**
     * Threshold to determine whether planes are inliers or not when testing
     * possible estimation solutions.
     * The threshold refers to the amount of algebraic error a possible 
     * solution has on a given line
     */
    private double mThreshold;   
    
    /**
     * Constructor
     */
    public MSACDualQuadricRobustEstimator(){
        super();
        mThreshold = DEFAULT_THRESHOLD;
    }

    /**
     * Constructor with points
     * @param planes 3D planes to estimate a dual quadric
     * @throws IllegalArgumentException if provided list of planes don't have 
     * a size greater or equal than MINIMUM_SIZE
     */
    public MSACDualQuadricRobustEstimator(List<Plane> planes) 
            throws IllegalArgumentException{
        super(planes);
        mThreshold = DEFAULT_THRESHOLD;
    }
    
    /**
     * Constructor
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     */
    public MSACDualQuadricRobustEstimator(
            DualQuadricRobustEstimatorListener listener){
        super(listener);
        mThreshold = DEFAULT_THRESHOLD;
    }
    
    
    /**
     * Constructor
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     * @param planes 3D planes to estimate a dual quadric
     * @throws IllegalArgumentException if provided list of planes don't have 
     * a size greater or equal than MINIMUM_SIZE
     */
    public MSACDualQuadricRobustEstimator(
            DualQuadricRobustEstimatorListener listener,
            List<Plane> planes) throws IllegalArgumentException{
        super(listener, planes);
        mThreshold = DEFAULT_THRESHOLD;
    }
    
    /**
     * Returns threshold to determine whether planes are inliers or not when 
     * testing possible estimation solutions.
     * The threshold refers to the amount of error a possible solution has on a 
     * given plane
     * @return threshold to determine whether planes are inliers or not when 
     * testing possible estimation solutions
     */
    public double getThreshold(){
        return mThreshold;
    }
    
    /**
     * Sets threshold to determine whether planes are inliers or not when 
     * testing possible estimation solutions.
     * Thre threshold refers to the amount of algebrauc error a possible 
     * solution has on a given plane
     * @param threshold threshold to be set
     * @throws IllegalArgumentException if provided value is equal or less than 
     * zero
     * @throws LockedException if robust estimator is locked because an 
     * estimation is already in progress
     */
    public void setThreshold(double threshold) throws IllegalArgumentException, 
            LockedException{
        if(isLocked()) throw new LockedException();
        if(threshold <= MIN_THRESHOLD) throw new IllegalArgumentException();
        mThreshold = threshold;
    }
    
            
    /**
     * Estimates a dual quadric using a robust estimator and the best set of 3D 
     * planes that fit into the locus of the estimated dual quadric found using 
     * the robust estimator
     * @return a dual quadric
     * @throws LockedException if robust estimator is locked because an 
     * estimation is already in progress
     * @throws NotReadyException if provided input data is not enough to start
     * the estimation
     * @throws RobustEstimatorException if estimation fails for any reason
     * (i.e. numerical instability, no solution available, etc)
     */    
    @Override
    public DualQuadric estimate() throws LockedException, NotReadyException, 
            RobustEstimatorException {
        if(isLocked()) throw new LockedException();
        if(!isReady()) throw new NotReadyException();
        
        MSACRobustEstimator<DualQuadric> innerEstimator =
                new MSACRobustEstimator<DualQuadric>(
                        new MSACRobustEstimatorListener<DualQuadric>(){

            @Override
            public double getThreshold() {
                return mThreshold;
            }

            @Override
            public int getTotalSamples() {
                return mPlanes.size();
            }

            @Override
            public int getSubsetSize() {
                return DualQuadricRobustEstimator.MINIMUM_SIZE;
            }

            @Override
            public void estimatePreliminarSolutions(int[] samplesIndices, 
                    List<DualQuadric> solutions) {
                Plane plane1 = mPlanes.get(samplesIndices[0]);
                Plane plane2 = mPlanes.get(samplesIndices[1]);
                Plane plane3 = mPlanes.get(samplesIndices[2]);
                Plane plane4 = mPlanes.get(samplesIndices[3]);
                Plane plane5 = mPlanes.get(samplesIndices[4]);
                Plane plane6 = mPlanes.get(samplesIndices[5]);
                Plane plane7 = mPlanes.get(samplesIndices[6]);
                Plane plane8 = mPlanes.get(samplesIndices[7]);
                Plane plane9 = mPlanes.get(samplesIndices[8]);
                
                try{
                    DualQuadric dualQuadric = new DualQuadric(plane1, plane2, 
                            plane3, plane4, plane5, plane6, plane7, plane8, 
                            plane9);
                    solutions.add(dualQuadric);
                }catch(CoincidentPlanesException e){
                    //if points are coincident, no solution is added
                }
            }

            @Override
            public double computeResidual(DualQuadric currentEstimation, 
                    int i) {
                return residual(currentEstimation, mPlanes.get(i));
            }

            @Override
            public boolean isReady() {
                return MSACDualQuadricRobustEstimator.this.isReady();
            }

            @Override
            public void onEstimateStart(RobustEstimator<DualQuadric> estimator) {
                if(mListener != null){
                    mListener.onEstimateStart(
                            MSACDualQuadricRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateEnd(RobustEstimator<DualQuadric> estimator) {
                if(mListener != null){
                    mListener.onEstimateEnd(
                            MSACDualQuadricRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateNextIteration(
                    RobustEstimator<DualQuadric> estimator, int iteration) {
                if(mListener != null){
                    mListener.onEstimateNextIteration(
                            MSACDualQuadricRobustEstimator.this, iteration);
                }
            }

            @Override
            public void onEstimateProgressChange(
                    RobustEstimator<DualQuadric> estimator, float progress) {
                if(mListener != null){
                    mListener.onEstimateProgressChange(
                            MSACDualQuadricRobustEstimator.this, progress);
                }
            }
        });
        
        try{
            mLocked = true;
            innerEstimator.setConfidence(mConfidence);
            innerEstimator.setMaxIterations(mMaxIterations);
            innerEstimator.setProgressDelta(mProgressDelta);
            return innerEstimator.estimate();
        }catch(com.irurueta.numerical.LockedException e){
            throw new LockedException(e);
        }catch(com.irurueta.numerical.NotReadyException e){
            throw new NotReadyException(e);
        }finally{
            mLocked = false;
        }
    }

    /**
     * Returns method being used for robust estimation
     * @return method being used for robust estimation
     */    
    @Override
    public RobustEstimatorMethod getMethod() {
        return RobustEstimatorMethod.MSAC;
    }                
}
