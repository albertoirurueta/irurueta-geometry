/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.estimators.PROMedSPoint3DRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date March 3, 2015
 */
package com.irurueta.geometry.estimators;

import com.irurueta.geometry.NoIntersectionException;
import com.irurueta.geometry.Plane;
import com.irurueta.geometry.Point3D;
import com.irurueta.numerical.robust.PROMedSRobustEstimator;
import com.irurueta.numerical.robust.PROMedSRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import java.util.List;

/**
 * Finds the best 3D point for provided collection of 3D points using PROMedS 
 * algorithm
 */
public class PROMedSPoint3DRobustEstimator extends Point3DRobustEstimator{
    
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
    public PROMedSPoint3DRobustEstimator(){
        super();
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }

    /**
     * Constructor with planes
     * @param planes 3D planes to estimate a 3D point
     * @throws IllegalArgumentException if provided list of planes doesn't have 
     * a size greater or equal than MINIMUM_SIZE
     */
    public PROMedSPoint3DRobustEstimator(List<Plane> planes) 
            throws IllegalArgumentException{
        super(planes);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }
    
    /**
     * Constructor
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     */
    public PROMedSPoint3DRobustEstimator(
            Point3DRobustEstimatorListener listener){
        super(listener);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }
    
    
    /**
     * Constructor
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     * @param planes 3D lines to estimate a 3D point
     * @throws IllegalArgumentException if provided list of planes doesn't have 
     * a size greater or equal than MINIMUM_SIZE
     */
    public PROMedSPoint3DRobustEstimator(
            Point3DRobustEstimatorListener listener,
            List<Plane> planes) throws IllegalArgumentException{
        super(listener, planes);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }
    
    /**
     * Constructor
     * @param qualityScores quality scores corresponding to each provided plane
     * @throws IllegalArgumentException if provided quality scores length is
     * smaller than MINIMUM_SIZE (i.e. 3 planes)
     */
    public PROMedSPoint3DRobustEstimator(double[] qualityScores) 
            throws IllegalArgumentException{
        super();
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor with planes
     * @param planes 3D planes to estimate a 3D point
     * @param qualityScores quality scores corresponding to each provided plane
     * @throws IllegalArgumentException if provided list of planes doesn't have 
     * the same size as the list of provided quality scores, or it their size 
     * is not greater or equal than MINIMUM_SIZE
     */
    public PROMedSPoint3DRobustEstimator(List<Plane> planes,
            double[] qualityScores) throws IllegalArgumentException{
        super(planes);
        
        if(qualityScores.length != planes.size()) 
            throw new IllegalArgumentException();
        
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
        internalSetQualityScores(qualityScores);
    }
    
    /**
     * Constructor
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     * @param qualityScores quality scores corresponding to each provided plane
     * @throws IllegalArgumentException if provided quality scores length is
     * smaller than MINIMUM_SIZE (i.e. 3 planes)
     */
    public PROMedSPoint3DRobustEstimator(
            Point3DRobustEstimatorListener listener,
            double[] qualityScores) throws IllegalArgumentException{
        super(listener);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
        internalSetQualityScores(qualityScores);
    }
    
    
    /**
     * Constructor
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     * @param planes 3D planes to estimate a 3D point
     * @param qualityScores quality scores corresponding to each provided plane
     * @throws IllegalArgumentException if provided list of planes doesn't have 
     * the same size as the list of provided quality scores, or it their size 
     * is not greater or equal than MINIMUM_SIZE
     */
    public PROMedSPoint3DRobustEstimator(
            Point3DRobustEstimatorListener listener,
            List<Plane> planes, double[] qualityScores) 
            throws IllegalArgumentException{
        super(listener, planes);
        
        if(qualityScores.length != planes.size()) 
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
     * Indicates if eatimator is ready to start the 3D point estimation.
     * This is true when input data (i.e. 2D points and quality scores) are 
     * provided and a minimum of MINIMUM_SIZE points are available
     * @return true if estimator is ready, false otherwise
     */
    @Override
    public boolean isReady(){
        return super.isReady() && mQualityScores != null && 
                mQualityScores.length == mPlanes.size();
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
        
        PROMedSRobustEstimator<Point3D> innerEstimator =
                new PROMedSRobustEstimator<Point3D>(
                        new PROMedSRobustEstimatorListener<Point3D>(){

            @Override
            public double getThreshold() {
                return mStopThreshold;
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
                return PROMedSPoint3DRobustEstimator.this.isReady();
            }

            @Override
            public void onEstimateStart(RobustEstimator<Point3D> estimator) {
                if(mListener != null){
                    mListener.onEstimateStart(
                            PROMedSPoint3DRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateEnd(RobustEstimator<Point3D> estimator) {
                if(mListener != null){
                    mListener.onEstimateEnd(PROMedSPoint3DRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateNextIteration(
                    RobustEstimator<Point3D> estimator, int iteration) {
                if(mListener != null){
                    mListener.onEstimateNextIteration(
                            PROMedSPoint3DRobustEstimator.this, iteration);
                }
            }

            @Override
            public void onEstimateProgressChange(
                    RobustEstimator<Point3D> estimator, float progress) {
                if(mListener != null){
                    mListener.onEstimateProgressChange(
                            PROMedSPoint3DRobustEstimator.this, progress);
                }
            }
            
            @Override
            public double[] getQualityScores() {
                return mQualityScores;
            }            
        });
        
        try{
            mLocked = true;
            mInliersData = null;
            innerEstimator.setConfidence(mConfidence);
            innerEstimator.setMaxIterations(mMaxIterations);
            innerEstimator.setProgressDelta(mProgressDelta);
            Point3D result = innerEstimator.estimate();
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
        return RobustEstimatorMethod.PROMedS;
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
        PROMedSRobustEstimator.PROMedSInliersData inliersData =
                (PROMedSRobustEstimator.PROMedSInliersData)getInliersData();
        return inliersData.getEstimatedThreshold();
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