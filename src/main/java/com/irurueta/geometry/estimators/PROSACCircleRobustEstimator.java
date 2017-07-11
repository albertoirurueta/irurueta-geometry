/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.estimators.PROSACCircleRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date February 28, 2015
 */
package com.irurueta.geometry.estimators;

import com.irurueta.geometry.Circle;
import com.irurueta.geometry.ColinearPointsException;
import com.irurueta.geometry.Point2D;
import com.irurueta.numerical.robust.PROSACRobustEstimator;
import com.irurueta.numerical.robust.PROSACRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import java.util.List;

/**
 * Finds the best circle for provided collection of 2D points using PROSAC
 * algorithm
 */
public class PROSACCircleRobustEstimator extends CircleRobustEstimator{
    /**
     * Constant defining default threshold to determine whether points are 
     * inliers or not.
     * Because typical resolution for points is 1 pixel, then default threshold 
     * is defined as 1.
     */
    public static final double DEFAULT_THRESHOLD = 1.0;
        
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
     * Quality scores corresponding to each provided point.
     * The larger the score value the betther the quality of the sample
     */
    private double[] mQualityScores;    
    
    /**
     * Constructor
     */
    public PROSACCircleRobustEstimator(){
        super();
        mThreshold = DEFAULT_THRESHOLD;
    }

    /**
     * Constructor with points
     * @param points 2D points to estimate a circle
     * @throws IllegalArgumentException if provided list of points don't have 
     * a size greater or equal than MINIMUM_SIZE
     */
    public PROSACCircleRobustEstimator(List<Point2D> points) 
            throws IllegalArgumentException{
        super(points);
        mThreshold = DEFAULT_THRESHOLD;
    }
    
    /**
     * Constructor
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     */
    public PROSACCircleRobustEstimator(CircleRobustEstimatorListener listener){
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
    public PROSACCircleRobustEstimator(CircleRobustEstimatorListener listener,
            List<Point2D> points) throws IllegalArgumentException{
        super(listener, points);
        mThreshold = DEFAULT_THRESHOLD;
    }
    
    /**
     * Constructor
     * @param qualityScores quality scores corresponding to each provided point
     * @throws IllegalArgumentException if provided quality scores length is
     * smaller than MINIMUM_SIZE (i.e. 3 points)
     */
    public PROSACCircleRobustEstimator(double[] qualityScores) 
            throws IllegalArgumentException{
        super();
        mThreshold = DEFAULT_THRESHOLD;
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor with points
     * @param points 2D points to estimate a circle
     * @param qualityScores quality scores corresponding to each provided point
     * @throws IllegalArgumentException if provided list of points don't have 
     * the same size as the list of provided quality scores, or it their size 
     * is not greater or equal than MINIMUM_SIZE
     */
    public PROSACCircleRobustEstimator(List<Point2D> points,
            double[] qualityScores) throws IllegalArgumentException{
        super(points);
        
        if(qualityScores.length != points.size()) 
            throw new IllegalArgumentException();
        
        mThreshold = DEFAULT_THRESHOLD;
        internalSetQualityScores(qualityScores);
    }
    
    /**
     * Constructor
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     * @param qualityScores quality scores corresponding to each provided point
     * @throws IllegalArgumentException if provided quality scores length is
     * smaller than MINIMUM_SIZE (i.e. 3 points)
     */
    public PROSACCircleRobustEstimator(CircleRobustEstimatorListener listener,
            double[] qualityScores) throws IllegalArgumentException{
        super(listener);
        mThreshold = DEFAULT_THRESHOLD;
        internalSetQualityScores(qualityScores);
    }
    
    
    /**
     * Constructor
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     * @param points 2D points to estimate a circle
     * @param qualityScores quality scores corresponding to each provided point
     * @throws IllegalArgumentException if provided list of points don't have 
     * the same size as the list of provided quality scores, or it their size 
     * is not greater or equal than MINIMUM_SIZE
     */
    public PROSACCircleRobustEstimator(CircleRobustEstimatorListener listener,
            List<Point2D> points, double[] qualityScores) 
            throws IllegalArgumentException{
        super(listener, points);
        
        if(qualityScores.length != points.size()) 
            throw new IllegalArgumentException();
        
        mThreshold = DEFAULT_THRESHOLD;
        internalSetQualityScores(qualityScores);
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
     * Returns quality scores corresponding to each provided point.
     * The larger the score value the betther the quality of the sampled point
     * @return quality scores corresponding to each point
     */
    @Override
    public double[] getQualityScores(){
        return mQualityScores;
    }
    
    /**
     * Sets quality scores corresponding to each provided point.
     * The larger the score value the better the quality of the sampled point
     * @param qualityScores quality scores corresponding to each point
     * @throws LockedException if robust estimator is locked because an 
     * estimation is already in progress
     * @throws IllegalArgumentException if provided quality scores length is 
     * smaller than MINIMUM_SIZE (i.e. 3 samples)
     */
    @Override
    public void setQualityScores(double[] qualityScores) throws LockedException,
            IllegalArgumentException{
        if(isLocked()) throw new LockedException();
        internalSetQualityScores(qualityScores);
    }
    
    /**
     * Indicates if eatimator is ready to start the conic estimation.
     * This is true when input data (i.e. 2D points and quality scores) are 
     * provided and a minimum of MINIMUM_SIZE points are available
     * @return true if estimator is ready, false otherwise
     */
    @Override
    public boolean isReady(){
        return super.isReady() && mQualityScores != null && 
                mQualityScores.length == mPoints.size();
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
        
        PROSACRobustEstimator<Circle> innerEstimator =
                new PROSACRobustEstimator<Circle>(
                        new PROSACRobustEstimatorListener<Circle>(){

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
                return PROSACCircleRobustEstimator.this.isReady();
            }

            @Override
            public void onEstimateStart(RobustEstimator<Circle> estimator) {
                if(mListener != null){
                    mListener.onEstimateStart(PROSACCircleRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateEnd(RobustEstimator<Circle> estimator) {
                if(mListener != null){
                    mListener.onEstimateEnd(PROSACCircleRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateNextIteration(
                    RobustEstimator<Circle> estimator, int iteration) {
                if(mListener != null){
                    mListener.onEstimateNextIteration(
                            PROSACCircleRobustEstimator.this, iteration);
                }
            }

            @Override
            public void onEstimateProgressChange(
                    RobustEstimator<Circle> estimator, float progress) {
                if(mListener != null){
                    mListener.onEstimateProgressChange(
                            PROSACCircleRobustEstimator.this, progress);
                }
            }
            
            @Override
            public double[] getQualityScores() {
                return mQualityScores;
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
        return RobustEstimatorMethod.PROSAC;
    }  
    
    /**
     * Sets quality scores corresponding to each provided point.
     * This method is used internally and does not check whether instance is
     * locked or not
     * @param qualityScores quality scores to be set
     * @throws IllegalArgumentException if provided quality scores length is
     * smaller than MINIMUM_SIZE
     */
    private void internalSetQualityScores(double[] qualityScores) 
            throws IllegalArgumentException{
        if(qualityScores.length < MINIMUM_SIZE) 
            throw new IllegalArgumentException();
        
        mQualityScores = qualityScores;        
    }     
}