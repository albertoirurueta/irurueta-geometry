/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.estimators.MSACPoint2DRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date March 1, 2015
 */
package com.irurueta.geometry.estimators;

import com.irurueta.geometry.Line2D;
import com.irurueta.geometry.NoIntersectionException;
import com.irurueta.geometry.Point2D;
import com.irurueta.numerical.robust.MSACRobustEstimator;
import com.irurueta.numerical.robust.MSACRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import java.util.List;

/**
 * Finds the best 2D point for provided collection of 2D lines using MSAC
 * algorithm
 */
public class MSACPoint2DRobustEstimator extends Point2DRobustEstimator{
    
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
     * Threshold to determine whether lines are inliers or not when testing
     * possible estimation solutions.
     * The threshold refers to the amount of error (i.e. distance) a possible 
     * solution has on a sampled line
     */
    private double mThreshold;     
    
    /**
     * Constructor
     */
    public MSACPoint2DRobustEstimator(){
        super();
        mThreshold = DEFAULT_THRESHOLD;
    }

    /**
     * Constructor with lines
     * @param lines 2D lines to estimate a 2D point
     * @throws IllegalArgumentException if provided list of lines don't have 
     * a size greater or equal than MINIMUM_SIZE
     */
    public MSACPoint2DRobustEstimator(List<Line2D> lines) 
            throws IllegalArgumentException{
        super(lines);
        mThreshold = DEFAULT_THRESHOLD;
    }
    
    /**
     * Constructor
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     */
    public MSACPoint2DRobustEstimator(Point2DRobustEstimatorListener listener){
        super(listener);
        mThreshold = DEFAULT_THRESHOLD;
    }
    
    
    /**
     * Constructor
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     * @param lines 2D lines to estimate a 2D point
     * @throws IllegalArgumentException if provided list of lines don't have 
     * a size greater or equal than MINIMUM_SIZE
     */
    public MSACPoint2DRobustEstimator(Point2DRobustEstimatorListener listener,
            List<Line2D> lines) throws IllegalArgumentException{
        super(listener, lines);
        mThreshold = DEFAULT_THRESHOLD;
    }
    
    /**
     * Returns threshold to determine whether lines are inliers or not when 
     * testing possible estimation solutions.
     * The threshold refers to the amount of error a possible solution has on a 
     * given line
     * @return threshold to determine whether lines are inliers or not when 
     * testing possible estimation solutions
     */
    public double getThreshold(){
        return mThreshold;
    }
    
    /**
     * Sets threshold to determine whether lines are inliers or not when 
     * testing possible estimation solutions.
     * Thre threshold refers to the amount of error a possible solution has on 
     * a given line
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
     * Estimates a 2D point using a robust estimator and the best set of 2D 
     * lines that intersect into the estimated 2D point
     * @return a 2D point
     * @throws LockedException if robust estimator is locked because an 
     * estimation is already in progress
     * @throws NotReadyException if provided input data is not enough to start
     * the estimation
     * @throws RobustEstimatorException if estimation fails for any reason
     * (i.e. numerical instability, no solution available, etc)
     */
    @Override
    public Point2D estimate() throws LockedException, NotReadyException, 
            RobustEstimatorException {
        if(isLocked()) throw new LockedException();
        if(!isReady()) throw new NotReadyException();
        
        MSACRobustEstimator<Point2D> innerEstimator =
                new MSACRobustEstimator<Point2D>(
                        new MSACRobustEstimatorListener<Point2D>(){

            @Override
            public double getThreshold() {
                return mThreshold;
            }

            @Override
            public int getTotalSamples() {
                return mLines.size();
            }

            @Override
            public int getSubsetSize() {
                return Point2DRobustEstimator.MINIMUM_SIZE;
            }

            @Override
            public void estimatePreliminarSolutions(int[] samplesIndices, 
                    List<Point2D> solutions) {
                Line2D line1 = mLines.get(samplesIndices[0]);
                Line2D line2 = mLines.get(samplesIndices[1]);
                
                try{
                    Point2D point = line1.getIntersection(line2);
                    solutions.add(point);
                }catch(NoIntersectionException e){
                    //if points are coincident, no solution is added
                }
            }

            @Override
            public double computeResidual(Point2D currentEstimation, int i) {
                return residual(currentEstimation, mLines.get(i));
            }

            @Override
            public boolean isReady() {
                return MSACPoint2DRobustEstimator.this.isReady();
            }

            @Override
            public void onEstimateStart(RobustEstimator<Point2D> estimator) {
                if(mListener != null){
                    mListener.onEstimateStart(MSACPoint2DRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateEnd(RobustEstimator<Point2D> estimator) {
                if(mListener != null){
                    mListener.onEstimateEnd(MSACPoint2DRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateNextIteration(
                    RobustEstimator<Point2D> estimator, int iteration) {
                if(mListener != null){
                    mListener.onEstimateNextIteration(
                            MSACPoint2DRobustEstimator.this, iteration);
                }
            }

            @Override
            public void onEstimateProgressChange(
                    RobustEstimator<Point2D> estimator, float progress) {
                if(mListener != null){
                    mListener.onEstimateProgressChange(
                            MSACPoint2DRobustEstimator.this, progress);
                }
            }
        });
        
        try{
            mLocked = true;
            mInliersData = null;
            innerEstimator.setConfidence(mConfidence);
            innerEstimator.setMaxIterations(mMaxIterations);
            innerEstimator.setProgressDelta(mProgressDelta);
            Point2D result = innerEstimator.estimate();
            mInliersData = innerEstimator.getInliersData();
            return attemptRefine(result);
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
