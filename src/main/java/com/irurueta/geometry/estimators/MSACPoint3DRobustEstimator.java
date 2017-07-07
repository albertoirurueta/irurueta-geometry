/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.estimators.MSACPoint3DRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date March 2, 2015
 */
package com.irurueta.geometry.estimators;

import com.irurueta.geometry.NoIntersectionException;
import com.irurueta.geometry.Plane;
import com.irurueta.geometry.Point3D;
import com.irurueta.numerical.robust.MSACRobustEstimator;
import com.irurueta.numerical.robust.MSACRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import java.util.List;

/**
 * Finds the best 3D point for provided collection of 3D planes using MSAC
 * algorithm
 */
public class MSACPoint3DRobustEstimator extends Point3DRobustEstimator{
    
    /**
     * Constant defining default threshold to determine whether points are 
     * inliers or not.
     * Because typical resolution for points is 1 voxel, then default threshold 
     * is defined as 1.
     */
    public static final double DEFAULT_THRESHOLD = 1;
        
    /**
     * Minimum value that can be set as threshold.
     * Threshold must be strictly greater than 0.0
     */
    public static final double MIN_THRESHOLD = 0.0;
    
    /**
     * Threshold to determine whether lines are inliers or not when testing
     * possible estimation solutions.
     * The threshold refers to the amount of error (i.e. distance) a possible 
     * solution has on a sampled line
     */
    private double mThreshold;     
    
    /**
     * Constructor
     */
    public MSACPoint3DRobustEstimator(){
        super();
        mThreshold = DEFAULT_THRESHOLD;
    }

    /**
     * Constructor with planes
     * @param planes 3D planes to estimate a 3D point
     * @throws IllegalArgumentException if provided list of planes doesn't have 
     * a size greater or equal than MINIMUM_SIZE
     */
    public MSACPoint3DRobustEstimator(List<Plane> planes) 
            throws IllegalArgumentException{
        super(planes);
        mThreshold = DEFAULT_THRESHOLD;
    }
    
    /**
     * Constructor
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     */
    public MSACPoint3DRobustEstimator(Point3DRobustEstimatorListener listener){
        super(listener);
        mThreshold = DEFAULT_THRESHOLD;
    }
    
    
    /**
     * Constructor
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     * @param planes 3D planes to estimate a 3D point
     * @throws IllegalArgumentException if provided list of planes doesn't have 
     * a size greater or equal than MINIMUM_SIZE
     */
    public MSACPoint3DRobustEstimator(Point3DRobustEstimatorListener listener,
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
     * Thre threshold refers to the amount of error a possible solution has on 
     * a given plane
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
     * Estimates a 3D point using a robust estimator and the best set of 3D 
     * planes that intersect into the estimated 3D point
     * @return a 3D point
     * @throws LockedException if robust estimator is locked because an 
     * estimation is already in progress
     * @throws NotReadyException if provided input data is not enough to start
     * the estimation
     * @throws RobustEstimatorException if estimation fails for any reason
     * (i.e. numerical instability, no solution available, etc)
     */
    @Override
    public Point3D estimate() throws LockedException, NotReadyException, 
            RobustEstimatorException {
        if(isLocked()) throw new LockedException();
        if(!isReady()) throw new NotReadyException();
        
        MSACRobustEstimator<Point3D> innerEstimator =
                new MSACRobustEstimator<Point3D>(
                        new MSACRobustEstimatorListener<Point3D>(){

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
                return Point3DRobustEstimator.MINIMUM_SIZE;
            }

            @Override
            public void estimatePreliminarSolutions(int[] samplesIndices, 
                    List<Point3D> solutions) {
                Plane plane1 = mPlanes.get(samplesIndices[0]);
                Plane plane2 = mPlanes.get(samplesIndices[1]);
                Plane plane3 = mPlanes.get(samplesIndices[2]);
                
                try{
                    Point3D point = plane1.getIntersection(plane2, plane3);
                    solutions.add(point);
                }catch(NoIntersectionException e){
                    //if points are coincident, no solution is added
                }
            }

            @Override
            public double computeResidual(Point3D currentEstimation, int i) {
                return residual(currentEstimation, mPlanes.get(i));
            }

            @Override
            public boolean isReady() {
                return MSACPoint3DRobustEstimator.this.isReady();
            }

            @Override
            public void onEstimateStart(RobustEstimator<Point3D> estimator) {
                if(mListener != null){
                    mListener.onEstimateStart(MSACPoint3DRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateEnd(RobustEstimator<Point3D> estimator) {
                if(mListener != null){
                    mListener.onEstimateEnd(MSACPoint3DRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateNextIteration(
                    RobustEstimator<Point3D> estimator, int iteration) {
                if(mListener != null){
                    mListener.onEstimateNextIteration(
                            MSACPoint3DRobustEstimator.this, iteration);
                }
            }

            @Override
            public void onEstimateProgressChange(
                    RobustEstimator<Point3D> estimator, float progress) {
                if(mListener != null){
                    mListener.onEstimateProgressChange(
                            MSACPoint3DRobustEstimator.this, progress);
                }
            }
        });
        
        try {
            mLocked = true;
            mInliersData = null;
            innerEstimator.setConfidence(mConfidence);
            innerEstimator.setMaxIterations(mMaxIterations);
            innerEstimator.setProgressDelta(mProgressDelta);
            Point3D result = innerEstimator.estimate();
            mInliersData = innerEstimator.getInliersData();
            return attemptRefine(result);
        } catch (com.irurueta.numerical.LockedException e) {
            throw new LockedException(e);
        } catch(com.irurueta.numerical.NotReadyException e) {
            throw new NotReadyException(e);
        } finally {
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
    
    /**
     * Gets standard deviation used for Levenberg-Marquardt fitting during 
     * refinement.
     * Returned value gives an indication of how much variance each residual
     * has.
     * Typically this value is related to the threshold used on each robust 
     * estimation, since residuals of found inliers are within the range of 
     * such threshold.
     * @return standard deviation used for refinement.
     */
    @Override
    protected double getRefinementStandardDeviation() {
        return mThreshold;
    }        
}
