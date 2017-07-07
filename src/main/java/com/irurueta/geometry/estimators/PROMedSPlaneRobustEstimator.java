/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.estimators.PROMedSPlaneRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date March 2, 2015
 */
package com.irurueta.geometry.estimators;

import com.irurueta.geometry.ColinearPointsException;
import com.irurueta.geometry.Plane;
import com.irurueta.geometry.Point3D;
import com.irurueta.numerical.robust.PROMedSRobustEstimator;
import com.irurueta.numerical.robust.PROMedSRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import java.util.List;

/**
 * Finds the best 2D plane for provided collection of 3D points using PROMedS
 * algorithm
 */
public class PROMedSPlaneRobustEstimator extends PlaneRobustEstimator{
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
     * Quality scores corresponding to each provided point.
     * The larger the score value the betther the quality of the sample
     */
    private double[] mQualityScores;      
    
    /**
     * Constructor
     */
    public PROMedSPlaneRobustEstimator(){
        super();
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }

    /**
     * Constructor with points
     * @param points 3D points to estimate a 3D plane
     * @throws IllegalArgumentException if provided list of points doesn't have 
     * a size greater or equal than MINIMUM_SIZE
     */
    public PROMedSPlaneRobustEstimator(List<Point3D> points) 
            throws IllegalArgumentException{
        super(points);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }
    
    /**
     * Constructor
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     */
    public PROMedSPlaneRobustEstimator(PlaneRobustEstimatorListener listener){
        super(listener);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }
    
    
    /**
     * Constructor
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     * @param points 3D points to estimate a 3D plane
     * @throws IllegalArgumentException if provided list of points doesn't have 
     * a size greater or equal than MINIMUM_SIZE
     */
    public PROMedSPlaneRobustEstimator(PlaneRobustEstimatorListener listener,
            List<Point3D> points) throws IllegalArgumentException{
        super(listener, points);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }
    
    /**
     * Constructor
     * @param qualityScores quality scores corresponding to each provided point
     * @throws IllegalArgumentException if provided quality scores length is
     * smaller than MINIMUM_SIZE (i.e. 3 points)
     */
    public PROMedSPlaneRobustEstimator(double[] qualityScores) 
            throws IllegalArgumentException{
        super();
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor with points
     * @param points 3D points to estimate a 3D plane
     * @param qualityScores quality scores corresponding to each provided point
     * @throws IllegalArgumentException if provided list of points don't have 
     * the same size as the list of provided quality scores, or it their size 
     * is not greater or equal than MINIMUM_SIZE
     */
    public PROMedSPlaneRobustEstimator(List<Point3D> points,
            double[] qualityScores) 
            throws IllegalArgumentException{
        super(points);
        
        if(qualityScores.length != points.size()) 
            throw new IllegalArgumentException();
        
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
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
    public PROMedSPlaneRobustEstimator(PlaneRobustEstimatorListener listener,
            double[] qualityScores) throws IllegalArgumentException{
        super(listener);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
        internalSetQualityScores(qualityScores);
    }
    
    
    /**
     * Constructor
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     * @param points 3D points to estimate a 3D plane
     * @param qualityScores quality scores corresponding to each provided point
     * @throws IllegalArgumentException if provided list of points don't have 
     * the same size as the list of provided quality scores, or it their size 
     * is not greater or equal than MINIMUM_SIZE
     */
    public PROMedSPlaneRobustEstimator(PlaneRobustEstimatorListener listener,
            List<Point3D> points, double[] qualityScores) 
            throws IllegalArgumentException{
        super(listener, points);
        
        if(qualityScores.length != points.size()) 
            throw new IllegalArgumentException();
        
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
        internalSetQualityScores(qualityScores);
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
     * Indicates if eatimator is ready to start the 2D line estimation.
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
     * Estimates a 3D plane using a robust estimator and the best set of 3D 
     * points that pass through the estimated 3D plane (i.e. belong to its 
     * locus)
     * @return a 3D plane
     * @throws LockedException if robust estimator is locked because an 
     * estimation is already in progress
     * @throws NotReadyException if provided input data is not enough to start
     * the estimation
     * @throws RobustEstimatorException if estimation fails for any reason
     * (i.e. numerical instability, no solution available, etc)
     */
    @Override
    public Plane estimate() throws LockedException, NotReadyException, 
            RobustEstimatorException {
        if(isLocked()) throw new LockedException();
        if(!isReady()) throw new NotReadyException();
        
        PROMedSRobustEstimator<Plane> innerEstimator =
                new PROMedSRobustEstimator<Plane>(
                        new PROMedSRobustEstimatorListener<Plane>(){

            @Override
            public double getThreshold() {
                return mStopThreshold;
            }

            @Override
            public int getTotalSamples() {
                return mPoints.size();
            }

            @Override
            public int getSubsetSize() {
                return PlaneRobustEstimator.MINIMUM_SIZE;
            }

            @Override
            public void estimatePreliminarSolutions(int[] samplesIndices, 
                    List<Plane> solutions) {
                Point3D point1 = mPoints.get(samplesIndices[0]);
                Point3D point2 = mPoints.get(samplesIndices[1]);
                Point3D point3 = mPoints.get(samplesIndices[2]);
                
                try{
                    Plane plane = new Plane(point1, point2, point3);
                    solutions.add(plane);
                }catch(ColinearPointsException e){
                    //if points are coincident, no solution is added
                }
            }

            @Override
            public double computeResidual(Plane currentEstimation, int i) {
                return residual(currentEstimation, mPoints.get(i));
            }

            @Override
            public boolean isReady() {
                return PROMedSPlaneRobustEstimator.this.isReady();
            }

            @Override
            public void onEstimateStart(RobustEstimator<Plane> estimator) {
                if(mListener != null){
                    mListener.onEstimateStart(PROMedSPlaneRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateEnd(RobustEstimator<Plane> estimator) {
                if(mListener != null){
                    mListener.onEstimateEnd(PROMedSPlaneRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateNextIteration(
                    RobustEstimator<Plane> estimator, int iteration) {
                if(mListener != null){
                    mListener.onEstimateNextIteration(
                            PROMedSPlaneRobustEstimator.this, iteration);
                }
            }

            @Override
            public void onEstimateProgressChange(
                    RobustEstimator<Plane> estimator, float progress) {
                if(mListener != null){
                    mListener.onEstimateProgressChange(
                            PROMedSPlaneRobustEstimator.this, progress);
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
        return RobustEstimatorMethod.PROMedS;
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
