/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.estimators.LMedSSphereRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date February 28, 2015
 */
package com.irurueta.geometry.estimators;

import com.irurueta.geometry.CoplanarPointsException;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.Sphere;
import com.irurueta.numerical.robust.LMedSRobustEstimator;
import com.irurueta.numerical.robust.LMedSRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import java.util.List;

/**
 * Finds the best sphere for provided collection of 3D points using LMedS
 * algorithm
 */
public class LMedSSphereRobustEstimator extends SphereRobustEstimator{
    /**
     * Default value to be used for stop threshold. Stop threshold can be used 
     * to keep the algorithm iterating in case that best estimated threshold 
     * using median of residuals is not small enough. Once a solution is found 
     * that generates a threshold below this value, the algorithm will stop.
     * The stop threshold can be used to prevent the LMedS algorithm iterating
     * too many times in cases where samples have a very similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close 
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would 
     * iterate for a long time trying to find the best solution when indeed 
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in RANSAC, and yet the algorithm could
     * still produce even smaller thresholds in estimated results
     */
    public static final double DEFAULT_STOP_THRESHOLD = 1e-3;
    
    /**
     * Minimum allowed stop threshold value
     */
    public static final double MIN_STOP_THRESHOLD = 0.0;

    /**
     * Threshold to be used to keep the algorithm iterating in case that best 
     * estimated threshold using median of residuals is not small enough. Once 
     * a solution is found that generates a threshold below this value, the 
     * algorithm will stop.
     * The stop threshold can be used to prevent the LMedS algorithm iterating
     * too many times in cases where samples have a very similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close 
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would 
     * iterate for a long time trying to find the best solution when indeed 
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in RANSAC, and yet the algorithm could
     * still produce even smaller thresholds in estimated results
     */
    private double mStopThreshold;      
    
    /**
     * Constructor
     */
    public LMedSSphereRobustEstimator(){
        super();
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }

    /**
     * Constructor with points
     * @param points 3D points to estimate a sphere
     * @throws IllegalArgumentException if provided list of points don't have 
     * a size greater or equal than MINIMUM_SIZE
     */
    public LMedSSphereRobustEstimator(List<Point3D> points) 
            throws IllegalArgumentException{
        super(points);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }
    
    /**
     * Constructor
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     */
    public LMedSSphereRobustEstimator(SphereRobustEstimatorListener listener){
        super(listener);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }
    
    
    /**
     * Constructor
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     * @param points 3D points to estimate a sphere
     * @throws IllegalArgumentException if provided list of points don't have 
     * a size greater or equal than MINIMUM_SIZE
     */
    public LMedSSphereRobustEstimator(SphereRobustEstimatorListener listener,
            List<Point3D> points) throws IllegalArgumentException{
        super(listener, points);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }
    
    /**
     * Returns threshold to be used to keep the algorithm iterating in case that 
     * best estimated threshold using median of residuals is not small enough. 
     * Once a solution is found that generates a threshold below this value, the 
     * algorithm will stop.
     * The stop threshold can be used to prevent the LMedS algorithm iterating
     * too many times in cases where samples have a very similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close 
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would 
     * iterate for a long time trying to find the best solution when indeed 
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in RANSAC, and yet the algorithm could
     * still produce even smaller thresholds in estimated results
     * @return stop threshold to stop the algorithm prematurely when a certain
     * accuracy has been reached
     */
    public double getStopThreshold(){
        return mStopThreshold;
    }
    
    /**
     * Sets threshold to be used to keep the algorithm iterating in case that
     * best estimated threshold using median of residuals is not small enough. 
     * Once a solution is found that generates a threshold below this value, the 
     * algorithm will stop.
     * The stop threshold can be used to prevent the LMedS algorithm iterating
     * too many times in cases where samples have a very similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close 
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would 
     * iterate for a long time trying to find the best solution when indeed 
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in RANSAC, and yet the algorithm could
     * still produce even smaller thresholds in estimated results
     * @param stopThreshold stop threshold to stop the algorithm prematurely 
     * when a certain accuracy has been reached
     * @throws IllegalArgumentException if provided value is zero or negative
     * @throws LockedException if robust estimator is locked because an 
     * estimation is already in progress
     */
    public void setStopThreshold(double stopThreshold) 
            throws IllegalArgumentException, LockedException{
        if(isLocked()) throw new LockedException();
        if(stopThreshold <= MIN_STOP_THRESHOLD) 
            throw new IllegalArgumentException();
        
        mStopThreshold = stopThreshold;
    }    
            
    /**
     * Estimates a sphere using a robust estimator and the best set of 3D points 
     * that fit into the locus of the estimated sphere found using the robust 
     * estimator
     * @return a sphere
     * @throws LockedException if robust estimator is locked because an 
     * estimation is already in progress
     * @throws NotReadyException if provided input data is not enough to start
     * the estimation
     * @throws RobustEstimatorException if estimation fails for any reason
     * (i.e. numerical instability, no solution available, etc)
     */    
    @Override
    public Sphere estimate() throws LockedException, NotReadyException, 
            RobustEstimatorException {
        if(isLocked()) throw new LockedException();
        if(!isReady()) throw new NotReadyException();
        
        LMedSRobustEstimator<Sphere> innerEstimator =
                new LMedSRobustEstimator<Sphere>(
                        new LMedSRobustEstimatorListener<Sphere>(){

            @Override
            public int getTotalSamples() {
                return mPoints.size();
            }

            @Override
            public int getSubsetSize() {
                return SphereRobustEstimator.MINIMUM_SIZE;
            }

            @Override
            public void estimatePreliminarSolutions(int[] samplesIndices, 
                    List<Sphere> solutions) {
                Point3D point1 = mPoints.get(samplesIndices[0]);
                Point3D point2 = mPoints.get(samplesIndices[1]);
                Point3D point3 = mPoints.get(samplesIndices[2]);
                Point3D point4 = mPoints.get(samplesIndices[3]);
                
                try{
                    Sphere sphere = new Sphere(point1, point2, point3, point4);
                    solutions.add(sphere);
                }catch(CoplanarPointsException e){
                    //if points are coincident, no solution is added
                }
            }

            @Override
            public double computeResidual(Sphere currentEstimation, int i) {
                return residual(currentEstimation, mPoints.get(i));
            }

            @Override
            public boolean isReady() {
                return LMedSSphereRobustEstimator.this.isReady();
            }

            @Override
            public void onEstimateStart(RobustEstimator<Sphere> estimator) {
                if(mListener != null){
                    mListener.onEstimateStart(LMedSSphereRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateEnd(RobustEstimator<Sphere> estimator) {
                if(mListener != null){
                    mListener.onEstimateEnd(LMedSSphereRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateNextIteration(
                    RobustEstimator<Sphere> estimator, int iteration) {
                if(mListener != null){
                    mListener.onEstimateNextIteration(
                            LMedSSphereRobustEstimator.this, iteration);
                }
            }

            @Override
            public void onEstimateProgressChange(
                    RobustEstimator<Sphere> estimator, float progress) {
                if(mListener != null){
                    mListener.onEstimateProgressChange(
                            LMedSSphereRobustEstimator.this, progress);
                }
            }
        });
        
        try{
            mLocked = true;
            innerEstimator.setConfidence(mConfidence);
            innerEstimator.setMaxIterations(mMaxIterations);
            innerEstimator.setProgressDelta(mProgressDelta);
            innerEstimator.setStopThreshold(mStopThreshold);
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
        return RobustEstimatorMethod.LMedS;
    }        
}