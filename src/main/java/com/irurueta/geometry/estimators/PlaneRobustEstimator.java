/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.estimators.PlaceRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date March 1, 2015
 */
package com.irurueta.geometry.estimators;

import com.irurueta.geometry.Plane;
import com.irurueta.geometry.Point3D;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import java.util.List;

/**
 * This is an abstract class for algorithms to robustly find the best 3D plane
 * that passes through a collection of 3D points.
 * Implementations of this class should be able to detect and discard outliers 
 * in order to find the best solution
 */
public abstract class PlaneRobustEstimator {
    /**
     * Minimum number of 3D points required to estimate a plane
     */
    public static final int MINIMUM_SIZE = 3;
    
    /**
     * Default amount of progress variation before notifying a change in
     * estimation progress. By default this is set to 5%
     */
    public static final float DEFAULT_PROGRESS_DELTA = 0.05f;
    
    /**
     * Minimum allowed value for progress delta
     */
    public static final float MIN_PROGRESS_DELTA = 0.0f;
    
   /**
     * Maximum allowed value for progress delta
     */
    public static final float MAX_PROGRESS_DELTA = 1.0f;
    
    /**
     * Constant defining default confidence of the estimated result, which is
     * 99%. This means that with a probability of 99% estimation will be
     * accurate because chosen subsamples will be inliers.
     */
    public static final double DEFAULT_CONFIDENCE = 0.99;
    
    /**
     * Default maximum allowed number of iterations
     */
    public static final int DEFAULT_MAX_ITERATIONS = 5000;
    
    /**
     * Minimum allowed confidence value
     */
    public static final double MIN_CONFIDENCE = 0.0;
    
    /**
     * Maximum allowed confidence value
     */
    public static final double MAX_CONFIDENCE = 1.0;
    
    /**
     * Minimum allowed number of iterations
     */
    public static final int MIN_ITERATIONS = 1;
    
    /**
     * Default robust estimator method when none is provided
     */
    public static final RobustEstimatorMethod DEFAULT_ROBUST_METHOD =
            RobustEstimatorMethod.PROMedS;
    
    /**
     * Listener to be notified of events such as when estimation starts, ends
     * or its progress significantly changes
     */
    protected PlaneRobustEstimatorListener mListener;
    
    /**
     * Indicates if this estimator is locked because an estimation is being
     * computed
     */
    protected volatile boolean mLocked;
    
    /**
     * Amount of progress variation before notifying a progress change during
     * estimation
     */
    protected float mProgressDelta;
    
    /**
     * Amount of confidence expressed as a value between 0.0 and 1.0 (which is
     * equivalent to 100%). The amount of confidence indicates the probability
     * that the estimated result is correct. Usually this value will be close
     * to 1.0, but not exactly 1.0
     */
    protected double mConfidence;
    
    /**
     * Maximum allowed number of iterations. When the maximum number of
     * iterations is exceeded, result will not be available, however an
     * approximate result will be available for retrieval
     */
    protected int mMaxIterations;   
    
    /**
     * List of points to be used to estimate a 3D plane. Provided list must have
     * a size greater or equal than MINIMUM_SIZE
     */
    protected List<Point3D> mPoints;
    
    /**
     * Constructor
     */
    public PlaneRobustEstimator(){
        mProgressDelta = DEFAULT_PROGRESS_DELTA;
        mConfidence = DEFAULT_CONFIDENCE;
        mMaxIterations = DEFAULT_MAX_ITERATIONS;                
    }
    
    /**
     * Constructor
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     */
    public PlaneRobustEstimator(PlaneRobustEstimatorListener listener){
        mListener = listener;
        mProgressDelta = DEFAULT_PROGRESS_DELTA;
        mConfidence = DEFAULT_CONFIDENCE;
        mMaxIterations = DEFAULT_MAX_ITERATIONS;        
    }
    
    /**
     * Constructor with points
     * @param points 3D points to estimate a 3D plane
     * @throws IllegalArgumentException if provided list of points doesn't have 
     * a size greater or equal than MINIMUM_SIZE
     */
    public PlaneRobustEstimator(List<Point3D> points) 
            throws IllegalArgumentException{
        mProgressDelta = DEFAULT_PROGRESS_DELTA;
        mConfidence = DEFAULT_CONFIDENCE;
        mMaxIterations = DEFAULT_MAX_ITERATIONS; 
        internalSetPoints(points);
    }
    
    /**
     * Constructor
     * @param points 3D points to estimate a 3D plane
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     * @throws IllegalArgumentException if provided list of points doesn't have 
     * a size greater or equal than MINIMUM_SIZE
     */
    public PlaneRobustEstimator(PlaneRobustEstimatorListener listener,
            List<Point3D> points) throws IllegalArgumentException{
        mListener = listener;
        mProgressDelta = DEFAULT_PROGRESS_DELTA;
        mConfidence = DEFAULT_CONFIDENCE;
        mMaxIterations = DEFAULT_MAX_ITERATIONS;
        internalSetPoints(points);
    }
    
    
    /**
     * Returns reference to listener to be notified of events such as when 
     * estimation starts, ends or its progress significantly changes
     * @return listener to be notified of events
     */
    public PlaneRobustEstimatorListener getListener(){
        return mListener;
    }
    
    /**
     * Sets listener to be notified of events such as when estimation starts,
     * ends or its progress significantly changes
     * @param listener listener to be notified of events
     * @throws LockedException if robust estimator is locked
     */
    public void setListener(PlaneRobustEstimatorListener listener) 
            throws LockedException{
        if(isLocked()) throw new LockedException();
        mListener = listener;
    }
    
    /**
     * Indicates whether listener has been provided and is available for 
     * retrieval
     * @return true if available, false otherwise
     */
    public boolean isListenerAvailable(){
        return mListener != null;
    }
    
    /**
     * Indicates if this instance is locked because estimation is being computed
     * @return true if locked, false otherwise
     */
    public boolean isLocked(){
        return mLocked;
    }

    /**
     * Returns amount of progress variation before notifying a progress change 
     * during estimation
     * @return amount of progress variation before notifying a progress change
     * during estimation
     */
    public float getProgressDelta(){
        return mProgressDelta;
    }
    
    /**
     * Sets amount of progress variation before notifying a progress change 
     * during estimation
     * @param progressDelta amount of progress variation before notifying a 
     * progress change during estimation
     * @throws IllegalArgumentException if progress delta is less than zero or
     * greater than 1
     * @throws LockedException if this estimator is locked because an estimation
     * is being computed
     */
    public void setProgressDelta(float progressDelta) 
            throws IllegalArgumentException, LockedException{
        if(isLocked()) throw new LockedException();
        if(progressDelta < MIN_PROGRESS_DELTA || 
                progressDelta > MAX_PROGRESS_DELTA) 
            throw new IllegalArgumentException();
        mProgressDelta = progressDelta;
    }
    
    /**
     * Returns amount of confidence expressed as a value between 0.0 and 1.0
     * (which is equivalent to 100%). The amount of confidence indicates the
     * probability that the estimated result is correct. Usually this value will
     * be close to 1.0, but not exactly 1.0
     * @return amount of confidence as a value between 0.0 and 1.0
     */
    public double getConfidence(){
        return mConfidence;
    }
    
    /**
     * Sets amount of confidence expressed as a value between 0.0 and 1.0 (which
     * is equivalent to 100%). The amount of confidence indicates the 
     * probability that the estimated result is correct. Usually this value will
     * be close to 1.0, but not exactly 1.0
     * @param confidence confidence to be set as a value between 0.0 and 1.0
     * @throws IllegalArgumentException if provided value is not between 0.0 and 
     * 1.0
     * @throws LockedException if this estimator is locked because an estimator 
     * is being computed
     */
    public void setConfidence(double confidence)
            throws IllegalArgumentException, LockedException{
        if(isLocked()) throw new LockedException();
        if(confidence < MIN_CONFIDENCE || confidence > MAX_CONFIDENCE)
            throw new IllegalArgumentException();
        mConfidence = confidence;
    }
    
    /**
     * Returns maximum allowed number of iterations. If maximum allowed number
     * of iterations is achieved without converging to a result when calling 
     * estimate(), a RobustEstimatorException will be raised
     * @return maximum allowed number of iterations
     */
    public int getMaxIterations(){
        return mMaxIterations;
    }
    
    /**
     * Sets maximum allowed number of iterations. When the maximum number of
     * iterations is exceeded, result will not be available, however an 
     * approximate result will be available for retrieval
     * @param maxIterations maximum allowed number of iterations to be set
     * @throws IllegalArgumentException if provided value is less than 1
     * @throws LockedException if this estimator is locked because an estimation
     * is being computed
     */
    public void setMaxIterations(int maxIterations) 
            throws IllegalArgumentException, LockedException{
        if(isLocked()) throw new LockedException();
        if(maxIterations < MIN_ITERATIONS) throw new IllegalArgumentException();
        mMaxIterations = maxIterations;
    }  
    
    /**
     * Returns list of points to be used to estimate a 3D plane.
     * Provided list must have a size greater or equal than MINIMUM_SIZE
     * @return list of points to be used to estimate a 3D plane
     */
    public List<Point3D> getPoints(){
        return mPoints;
    }
    
    /**
     * Sets list of points to be used to estimate a 3D plane.
     * Provided list must have a size greater or equal than MINIMUM_SIZE
     * @param points list of points to be used to estimate a 3D plane
     * @throws IllegalArgumentException if provided list of points doesn't have 
     * a size greater or equal than MINIMUM_SIZE
     * @throws LockedException if estimator is locked because a computation is
     * already in progress
     */
    public void setPoints(List<Point3D> points) throws IllegalArgumentException,
            LockedException{
        if(isLocked()) throw new LockedException();
        internalSetPoints(points);
    }
    
    /**
     * Indicates if estimator is ready to start the 3D plane estimation.
     * This is true when a minimum of MINIMUM_SIZE points are available
     * @return true if estimator is ready, false otherwise
     */
    public boolean isReady(){
        return mPoints != null && mPoints.size() >= MINIMUM_SIZE;
    }
    
    /**
     * Returns quality scores corresponding to each point.
     * The larger the score value the better the quality of the point measure.
     * This implementation always returns null.
     * Subclasses using quality scores must implement proper behaviour
     * @return quality scores corresponding to each point
     */
    public double[] getQualityScores(){
        return null;
    }
    
    /**
     * Sets quality scores corresponding to each point.
     * The larger the score value the better the quality of the point measure.
     * This implementation makes no action.
     * Subclasses using quality scores must implement proper behaviour.
     * @param qualityScores quality scores corresponding to each sampled point
     * @throws LockedException if robust estimator is locked because an 
     * estimation is already in progress
     * @throws IllegalArgumentException if provided quality scores length is
     * smaller than MINIMUM_SIZE (i.e. 2 samples)
     */
    public void setQualityScores(double[] qualityScores) throws LockedException,
            IllegalArgumentException{}
    
    /**
     * Creates a 3D plane robust estimator based on 3D point samples and using
     * provided robust estimator method
     * @param method method of a robust estimator algorithm to estimate best
     * 3D plane
     * @return an instance of a 3D plane robust estimator
     */
    public static PlaneRobustEstimator create(RobustEstimatorMethod method){
        switch(method){
            case LMedS:
                return new LMedSPlaneRobustEstimator();
            case MSAC:
                return new MSACPlaneRobustEstimator();
            case PROSAC:
                return new PROSACPlaneRobustEstimator();
            case PROMedS:
                return new PROMedSPlaneRobustEstimator();
            case RANSAC:
            default:
                return new RANSACPlaneRobustEstimator();
        }
    }
    
    /**
     * Creates a 3D plane robust estimator based on 3D point samples and using
     * provided points and robust estimator method
     * @param points 3D points to estimate a 3D plane
     * @param method method of a robust estimator algorithm to estimate best
     * 3D plane
     * @return an instance of a 3D plane robust estimator
     * @throws IllegalArgumentException if provided list of points doesn't have 
     * a size greater or equal than MINIMUM_SIZE
     */
    public static PlaneRobustEstimator create(List<Point3D> points, 
            RobustEstimatorMethod method) throws IllegalArgumentException{
        switch(method){
            case LMedS:
                return new LMedSPlaneRobustEstimator(points);
            case MSAC:
                return new MSACPlaneRobustEstimator(points);
            case PROSAC:
                return new PROSACPlaneRobustEstimator(points);
            case PROMedS:
                return new PROMedSPlaneRobustEstimator(points);
            case RANSAC:
            default:
                return new RANSACPlaneRobustEstimator(points);
        }
    }
    
    /**
     * Creates a 3D plane robust estimator based on 3D point samples and using
     * provided listener
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     * @param method method of a robust estimator algorithm to estimate best
     * 3D plane
     * @return an instance of a 3D plane robust estimator
     */
    public static PlaneRobustEstimator create(
            PlaneRobustEstimatorListener listener, 
            RobustEstimatorMethod method){
        switch(method){
            case LMedS:
                return new LMedSPlaneRobustEstimator(listener);
            case MSAC:
                return new MSACPlaneRobustEstimator(listener);
            case PROSAC:
                return new PROSACPlaneRobustEstimator(listener);
            case PROMedS:
                return new PROMedSPlaneRobustEstimator(listener);
            case RANSAC:
            default:
                return new RANSACPlaneRobustEstimator(listener);
        }
    }
    
    /**
     * Creates a 3D plane robust estimator based on 3D point samples and using
     * provided listener and points
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     * @param points 3D points to estimate a 3D plane
     * @param method method of a robust estimator algorithm to estimate best
     * 3D plane
     * @return an instance of a 3D plane robust estimator
     * @throws IllegalArgumentException if provided list of points doesn't have 
     * a size greater or equal than MINIMUM_SIZE
     */
    public static PlaneRobustEstimator create(
            PlaneRobustEstimatorListener listener, List<Point3D> points,
            RobustEstimatorMethod method) throws IllegalArgumentException{
        switch(method){
            case LMedS:
                return new LMedSPlaneRobustEstimator(listener, points);
            case MSAC:
                return new MSACPlaneRobustEstimator(listener, points);
            case PROSAC:
                return new PROSACPlaneRobustEstimator(listener, points);
            case PROMedS:
                return new PROMedSPlaneRobustEstimator(listener, points);
            case RANSAC:
            default:
                return new RANSACPlaneRobustEstimator(listener, points);
        }
    }
    
    /**
     * Creates a 3D plane robust estimator based on 3D point samples and using
     * provided robust estimator method
     * @param qualityScores quality scores corresponding to each provided point
     * @param method method of a robust estimator algorithm to estimate best
     * 3D plane
     * @return an instance of a 3D plane robust estimator
     * @throws IllegalArgumentException if provided quality scores length is
     * smaller than MINIMUM_SIZE (i.e. 3 points)
     */
    public static PlaneRobustEstimator create(double[] qualityScores,
            RobustEstimatorMethod method) throws IllegalArgumentException{
        switch(method){
            case LMedS:
                return new LMedSPlaneRobustEstimator();
            case MSAC:
                return new MSACPlaneRobustEstimator();
            case PROSAC:
                return new PROSACPlaneRobustEstimator(qualityScores);
            case PROMedS:
                return new PROMedSPlaneRobustEstimator(qualityScores);
            case RANSAC:
            default:
                return new RANSACPlaneRobustEstimator();
        }
    }
    
    /**
     * Creates a 3D plane robust estimator based on 3D point samples and using
     * provided points and robust estimator method
     * @param points 3D points to estimate a 3D plane
     * @param qualityScores quality scores corresponding to each provided point
     * @param method method of a robust estimator algorithm to estimate best
     * 3D plane
     * @return an instance of a 3D plane robust estimator
     * @throws IllegalArgumentException if provided list of points doesn't have 
     * the same size as the list of provided quality scores, or it their size 
     * is not greater or equal than MINIMUM_SIZE
     */
    public static PlaneRobustEstimator create(List<Point3D> points, 
            double[] qualityScores, RobustEstimatorMethod method) 
            throws IllegalArgumentException{
        switch(method){
            case LMedS:
                return new LMedSPlaneRobustEstimator(points);
            case MSAC:
                return new MSACPlaneRobustEstimator(points);
            case PROSAC:
                return new PROSACPlaneRobustEstimator(points, qualityScores);
            case PROMedS:
                return new PROMedSPlaneRobustEstimator(points, qualityScores);
            case RANSAC:
            default:
                return new RANSACPlaneRobustEstimator(points);
        }
    }
    
    /**
     * Creates a 3D plane robust estimator based on 3D point samples and using
     * provided listener
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     * @param qualityScores quality scores corresponding to each provided point
     * @param method method of a robust estimator algorithm to estimate best
     * 3D plane
     * @return an instance of a 3D plane robust estimator
     * @throws IllegalArgumentException if provided quality scores length is
     * smaller than MINIMUM_SIZE (i.e. 3 points)
     */
    public static PlaneRobustEstimator create(
            PlaneRobustEstimatorListener listener, double[] qualityScores,
            RobustEstimatorMethod method) throws IllegalArgumentException{
        switch(method){
            case LMedS:
                return new LMedSPlaneRobustEstimator(listener);
            case MSAC:
                return new MSACPlaneRobustEstimator(listener);
            case PROSAC:
                return new PROSACPlaneRobustEstimator(listener, qualityScores);
            case PROMedS:
                return new PROMedSPlaneRobustEstimator(listener, qualityScores);
            case RANSAC:
            default:
                return new RANSACPlaneRobustEstimator(listener);
        }
    }
    
    /**
     * Creates a 3D plane robust estimator based on 3D point samples and using
     * provided listener and points
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     * @param points 3D points to estimate a 3D plane
     * @param qualityScores quality scores corresponding to each provided point
     * @param method method of a robust estimator algorithm to estimate best
     * 3D plane
     * @return an instance of a 3D plane robust estimator
     * @throws IllegalArgumentException if provided list of points doesn't have 
     * the same size as the list of provided quality scores, or it their size 
     * is not greater or equal than MINIMUM_SIZE
     */
    public static PlaneRobustEstimator create(
            PlaneRobustEstimatorListener listener, List<Point3D> points,
            double[] qualityScores, RobustEstimatorMethod method) 
            throws IllegalArgumentException{
        switch(method){
            case LMedS:
                return new LMedSPlaneRobustEstimator(listener, points);
            case MSAC:
                return new MSACPlaneRobustEstimator(listener, points);
            case PROSAC:
                return new PROSACPlaneRobustEstimator(listener, points, 
                        qualityScores);
            case PROMedS:
                return new PROMedSPlaneRobustEstimator(listener, points, 
                        qualityScores);
            case RANSAC:
            default:
                return new RANSACPlaneRobustEstimator(listener, points);
        }
    }    
    
    /**
     * Creates a 3D line robust estimator based on 3D point samples and using
     * default robust estimator method
     * @return an instance of a 3D line robust estimator
     */
    public static PlaneRobustEstimator create(){
        return create(DEFAULT_ROBUST_METHOD);
    }
    
    /**
     * Creates a 3D plane robust estimator based on 3D point samples and using
     * provided points and default robust estimator method
     * @param points 3D points to estimate a 3D plane
     * @return an instance of a 3D plane robust estimator
     * @throws IllegalArgumentException if provided list of points doesn't have 
     * a size greater or equal than MINIMUM_SIZE
     */
    public static PlaneRobustEstimator create(List<Point3D> points) 
            throws IllegalArgumentException{
        return create(points, DEFAULT_ROBUST_METHOD);
    }
    
    /**
     * Creates a 3D plane robust estimator based on 3D point samples and using
     * provided listener and default robust estimator method
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     * @return an instance of a 3D plane robust estimator
     */
    public static PlaneRobustEstimator create(
            PlaneRobustEstimatorListener listener){
        return create(listener, DEFAULT_ROBUST_METHOD);
    }
    
    /**
     * Creates a 3D plane robust estimator based on 3D point samples and using
     * provided listener and lines and default robust estimator method
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     * @param points 3D points to estimate a plane
     * @return an instance of a 3D plane robust estimator
     * @throws IllegalArgumentException if provided list of points doesn't have 
     * a size greater or equal than MINIMUM_SIZE
     */
    public static PlaneRobustEstimator create(
            PlaneRobustEstimatorListener listener, List<Point3D> points) 
            throws IllegalArgumentException{
        return create(listener, points, DEFAULT_ROBUST_METHOD);
    }
    
    /**
     * Creates a 3D plane robust estimator based on 3D point samples and using
     * default robust estimator method
     * @param qualityScores quality scores corresponding to each provided point
     * @return an instance of a 3D point robust estimator
     * @throws IllegalArgumentException if provided quality scores length is
     * smaller than MINIMUM_SIZE (i.e. 3 points)
     */
    public static PlaneRobustEstimator create(double[] qualityScores) 
            throws IllegalArgumentException{
        return create(qualityScores, DEFAULT_ROBUST_METHOD);
    }
    
    /**
     * Creates a 3D plane robust estimator based on 3D point samples and using
     * provided points and default estimator method
     * @param points 3D points to estimate a 3D plane
     * @param qualityScores quality scores corresponding to each provided point
     * @return an instance of a 3D plane robust estimator
     * @throws IllegalArgumentException if provided list of points don't have 
     * the same size as the list of provided quality scores, or if their size 
     * is not greater or equal than MINIMUM_SIZE
     */
    public static PlaneRobustEstimator create(List<Point3D> points, 
            double[] qualityScores) throws IllegalArgumentException{
        return create(points, qualityScores, DEFAULT_ROBUST_METHOD);
    }
    
    /**
     * Creates a 3D plane robust estimator based on 3D point samples and using
     * provided listener and default estimator method
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     * @param qualityScores quality scores corresponding to each provided point
     * @return an instance of a circle robust estimator
     * @throws IllegalArgumentException if provided quality scores length is
     * smaller than MINIMUM_SIZE (i.e. 3 points)
     */
    public static PlaneRobustEstimator create(
            PlaneRobustEstimatorListener listener, double[] qualityScores)
            throws IllegalArgumentException{
        return create(listener, qualityScores, DEFAULT_ROBUST_METHOD);
    }
    
    /**
     * Creates a 3D plane robust estimator based on 3D point samples and using
     * provided listener and points and default estimator method
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     * @param points 3D points to estimate a 3D plane
     * @param qualityScores quality scores corresponding to each provided point
     * @return an instance of a 3D plane robust estimator
     * @throws IllegalArgumentException if provided list of points don't have 
     * the same size as the list of provided quality scores, or if their size 
     * is not greater or equal than MINIMUM_SIZE
     */
    public static PlaneRobustEstimator create(
            PlaneRobustEstimatorListener listener, List<Point3D> points,
            double[] qualityScores) throws IllegalArgumentException{
        return create(listener, points, qualityScores, DEFAULT_ROBUST_METHOD);
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
    public abstract Plane estimate() throws LockedException, 
            NotReadyException, RobustEstimatorException;
        
    /**
     * Returns method being used for robust estimation
     * @return method being used for robust estimation
     */
    public abstract RobustEstimatorMethod getMethod();   
    
    /**
     * Internal method to set list of 3D points to be used to estimate a 3D 
     * plane.
     * This method does not check whether estimator is locked or not
     * @param points list of points to be used to estimate a 3D plane
     * @throws IllegalArgumentException if provided list of points doesn't have
     * a size greater or equal than MINIMUM_SIZE
     */
    private void internalSetPoints(List<Point3D> points) 
            throws IllegalArgumentException{
        if(points.size() < MINIMUM_SIZE) throw new IllegalArgumentException();
        mPoints = points;
    }
    
    /**
     * Computes the residual between a 3D point and a plane
     * @param plane a 3D plane
     * @param point a 3D point
     * @return residual
     */
    protected double residual(Plane plane, Point3D point){
        plane.normalize();
        point.normalize();
        
        return Math.abs(plane.signedDistance(point));
    }            
}
