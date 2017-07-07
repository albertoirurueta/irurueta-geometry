/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.estimators.MSACCircleRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date February 28, 2015
 */
package com.irurueta.geometry.estimators;

import com.irurueta.geometry.Circle;
import com.irurueta.geometry.ColinearPointsException;
import com.irurueta.geometry.Point2D;
import com.irurueta.numerical.robust.MSACRobustEstimator;
import com.irurueta.numerical.robust.MSACRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import java.util.List;

/**
 * Finds the best circle for provided collection of 2D points using MSAC 
 * algorithm
 */
public class MSACCircleRobustEstimator extends CircleRobustEstimator{
    /**
     * Constant defining default threshold to determine whether points are 
     * inliers or not.
     * Because typical resolution for points is 1 pixel, then default threshold 
     * is defined as 1.
     */
    public static final double DEFAULT_THRESHOLD = 1;
        
    /**
     * Minimum value that can be set as threshold.
     * Threshold must be strictly greater than 0.0
     */
    public static final double MIN_THRESHOLD = 0.0;
    
    /**
     * Threshold to determine whether points are inliers or not when testing
     * possible estimation solutions.
     * The threshold refers to the amount of error (i.e. distance) a possible 
     * solution has on a matched pair of points
     */
    private double mThreshold;     
    
    /**
     * Constructor
     */
    public MSACCircleRobustEstimator(){
        super();
        mThreshold = DEFAULT_THRESHOLD;
    }

    /**
     * Constructor with points
     * @param points 2D points to estimate a circle
     * @throws IllegalArgumentException if provided list of points don't have 
     * a size greater or equal than MINIMUM_SIZE
     */
    public MSACCircleRobustEstimator(List<Point2D> points) 
            throws IllegalArgumentException{
        super(points);
        mThreshold = DEFAULT_THRESHOLD;
    }
    
    /**
     * Constructor
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     */
    public MSACCircleRobustEstimator(CircleRobustEstimatorListener listener){
        super(listener);
        mThreshold = DEFAULT_THRESHOLD;
    }
    
    
    /**
     * Constructor
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     * @param points 2D points to estimate a circle
     * @throws IllegalArgumentException if provided list of points don't have 
     * a size greater or equal than MINIMUM_SIZE
     */
    public MSACCircleRobustEstimator(CircleRobustEstimatorListener listener,
            List<Point2D> points) throws IllegalArgumentException{
        super(listener, points);
        mThreshold = DEFAULT_THRESHOLD;
    }
    
    /**
     * Returns threshold to determine whether points are inliers or not when 
     * testing possible estimation solutions.
     * The threshold refers to the amount of error a possible solution has on a 
     * given point
     * @return threshold to determine whether points are inliers or not when 
     * testing possible estimation solutions
     */
    public double getThreshold(){
        return mThreshold;
    }
    
    /**
     * Sets threshold to determine whether points are inliers or not when 
     * testing possible estimation solutions.
     * Thre threshold refers to the amount of error a possible solution has on 
     * a given point
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
     * Estimates a circle using a robust estimator and the best set of 2D points 
     * that fit into the locus of the estimated circle found using the robust 
     * estimator
     * @return a circle
     * @throws LockedException if robust estimator is locked because an 
     * estimation is already in progress
     * @throws NotReadyException if provided input data is not enough to start
     * the estimation
     * @throws RobustEstimatorException if estimation fails for any reason
     * (i.e. numerical instability, no solution available, etc)
     */    
    @Override
    public Circle estimate() throws LockedException, NotReadyException, 
            RobustEstimatorException {
        if(isLocked()) throw new LockedException();
        if(!isReady()) throw new NotReadyException();
        
        MSACRobustEstimator<Circle> innerEstimator =
                new MSACRobustEstimator<Circle>(
                        new MSACRobustEstimatorListener<Circle>(){

            @Override
            public double getThreshold() {
                return mThreshold;
            }

            @Override
            public int getTotalSamples() {
                return mPoints.size();
            }

            @Override
            public int getSubsetSize() {
                return CircleRobustEstimator.MINIMUM_SIZE;
            }

            @Override
            public void estimatePreliminarSolutions(int[] samplesIndices, 
                    List<Circle> solutions) {
                Point2D point1 = mPoints.get(samplesIndices[0]);
                Point2D point2 = mPoints.get(samplesIndices[1]);
                Point2D point3 = mPoints.get(samplesIndices[2]);
                
                try{
                    Circle circle = new Circle(point1, point2, point3);
                    solutions.add(circle);
                }catch(ColinearPointsException e){
                    //if points are coincident, no solution is added
                }
            }

            @Override
            public double computeResidual(Circle currentEstimation, int i) {
                return residual(currentEstimation, mPoints.get(i));
            }

            @Override
            public boolean isReady() {
                return MSACCircleRobustEstimator.this.isReady();
            }

            @Override
            public void onEstimateStart(RobustEstimator<Circle> estimator) {
                if(mListener != null){
                    mListener.onEstimateStart(MSACCircleRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateEnd(RobustEstimator<Circle> estimator) {
                if(mListener != null){
                    mListener.onEstimateEnd(MSACCircleRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateNextIteration(
                    RobustEstimator<Circle> estimator, int iteration) {
                if(mListener != null){
                    mListener.onEstimateNextIteration(
                            MSACCircleRobustEstimator.this, iteration);
                }
            }

            @Override
            public void onEstimateProgressChange(
                    RobustEstimator<Circle> estimator, float progress) {
                if(mListener != null){
                    mListener.onEstimateProgressChange(
                            MSACCircleRobustEstimator.this, progress);
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
