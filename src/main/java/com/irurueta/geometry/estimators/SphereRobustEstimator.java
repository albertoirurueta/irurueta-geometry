/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.estimators.SphereRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date February 28, 2015
 */
package com.irurueta.geometry.estimators;

import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.Sphere;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import java.util.List;

/**
 * This is an abstract class for algorithms to robustly find the best sphere 
 * that fits in a collection of 3D points.
 * Implementations of this class should be able to detect and discard outliers
 * in order to find the best solution
 */
public abstract class SphereRobustEstimator {
    /**
     * Minimum number of 3D points required to estimate a sphere
     */
    public static final int MINIMUM_SIZE = 4;
    
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
    protected SphereRobustEstimatorListener mListener;
    
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
     * List of points to be used to estimate a sphere. Provided list must have
     * a size greater or equal than MINIMUM_SIZE
     */
    protected List<Point3D> mPoints;    
    
    /**
     * Constructor
     */
    public SphereRobustEstimator(){
        mProgressDelta = DEFAULT_PROGRESS_DELTA;
        mConfidence = DEFAULT_CONFIDENCE;
        mMaxIterations = DEFAULT_MAX_ITERATIONS;                
    }
    
    /**
     * Constructor
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     */
    public SphereRobustEstimator(SphereRobustEstimatorListener listener){
        mListener = listener;
        mProgressDelta = DEFAULT_PROGRESS_DELTA;
        mConfidence = DEFAULT_CONFIDENCE;
        mMaxIterations = DEFAULT_MAX_ITERATIONS;        
    }
    
    /**
     * Constructor with points
     * @param points 3D points to estimate a sphere
     * @throws IllegalArgumentException if provided list of points don't have 
     * a size greater or equal than MINIMUM_SIZE
     */
    public SphereRobustEstimator(List<Point3D> points) 
            throws IllegalArgumentException{
        mProgressDelta = DEFAULT_PROGRESS_DELTA;
        mConfidence = DEFAULT_CONFIDENCE;
        mMaxIterations = DEFAULT_MAX_ITERATIONS; 
        internalSetPoints(points);
    }
    
    /**
     * Constructor
     * @param points 3D points to estimate a sphere
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     * @throws IllegalArgumentException if provided list of points don't have 
     * a size greater or equal than MINIMUM_SIZE
     */
    public SphereRobustEstimator(SphereRobustEstimatorListener listener,
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
    public SphereRobustEstimatorListener getListener(){
        return mListener;
    }
    
    /**
     * Sets listener to be notified of events such as when estimation starts,
     * ends or its progress significantly changes
     * @param listener listener to be notified of events
     * @throws LockedException if robust estimator is locked
     */
    public void setListener(SphereRobustEstimatorListener listener) 
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
     * Returns list of points to be used to estimate a sphere.
     * Provided list must have a size greater or equal than MINIMUM_SIZE
     * @return list of points to be used to estimate a sphere
     */
    public List<Point3D> getPoints(){
        return mPoints;
    }
    
    /**
     * Sets list of points to be used to estimate a sphere.
     * Provided list must have a size greater or equal than MINIMUM_SIZE
     * @param points list of points to be used to estimate a sphere
     * @throws IllegalArgumentException if provided list of points don't have 
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
     * Indicates if estimator is ready to start the sphere estimation.
     * This is true when a minimum if MINIMUM_SIZE points are available
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
     * The larger the score value the better the quality of the matching.
     * This implementation makes no action.
     * Subclasses using quality scores must implement proper behaviour.
     * @param qualityScores quality scores corresponding to each pair of matched
     * points
     * @throws LockedException if robust estimator is locked because an 
     * estimation is already in progress
     * @throws IllegalArgumentException if provided quality scores length is
     * smaller than MINIMUM_SIZE (i.e. 4 samples)
     */
    public void setQualityScores(double[] qualityScores) throws LockedException,
            IllegalArgumentException{}
    
    /**
     * Creates a sphere robust estimator based on 3D point samples and using
     * provided robust estimator method
     * @param method method of a robust estimator algorithm to estimate best
     * sphere
     * @return an instance of a sphere robust estimator
     */
    public static SphereRobustEstimator create(RobustEstimatorMethod method){
        switch(method){
            case LMedS:
                return new LMedSSphereRobustEstimator();
            case MSAC:
                return new MSACSphereRobustEstimator();
            case PROSAC:
                return new PROSACSphereRobustEstimator();
            case PROMedS:
                return new PROMedSSphereRobustEstimator();
            case RANSAC:
            default:
                return new RANSACSphereRobustEstimator();
        }
    }
    
    /**
     * Creates a sphere robust estimator based on 3D point samples and using
     * provided points and robust estimator method
     * @param points 3D points to estimate a sphere
     * @param method method of a robust estimator algorithm to estimate best
     * sphere
     * @return an instance of a sphere robust estimator
     * @throws IllegalArgumentException if provided list of points don't have a
     * size greater or equal than MINIMUM_SIZE
     */
    public static SphereRobustEstimator create(List<Point3D> points, 
            RobustEstimatorMethod method) throws IllegalArgumentException{
        switch(method){
            case LMedS:
                return new LMedSSphereRobustEstimator(points);
            case MSAC:
                return new MSACSphereRobustEstimator(points);
            case PROSAC:
                return new PROSACSphereRobustEstimator(points);
            case PROMedS:
                return new PROMedSSphereRobustEstimator(points);
            case RANSAC:
            default:
                return new RANSACSphereRobustEstimator(points);
        }
    }
    
    /**
     * Creates a sphere robust estimator based on 3D point samples and using
     * provided listener
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     * @param method method of a robust estimator algorithm to estimate best
     * sphere
     * @return an instance of a sphere robust estimator
     */
    public static SphereRobustEstimator create(
            SphereRobustEstimatorListener listener, 
            RobustEstimatorMethod method){
        switch(method){
            case LMedS:
                return new LMedSSphereRobustEstimator(listener);
            case MSAC:
                return new MSACSphereRobustEstimator(listener);
            case PROSAC:
                return new PROSACSphereRobustEstimator(listener);
            case PROMedS:
                return new PROMedSSphereRobustEstimator(listener);
            case RANSAC:
            default:
                return new RANSACSphereRobustEstimator(listener);
        }
    }
    
    /**
     * Creates a sphere robust estimator based on 3D point samples and using
     * provided listener and points
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     * @param points 3D points to estimate a sphere
     * @param method method of a robust estimator algorithm to estimate best
     * sphere
     * @return an instance of a sphere robust estimator
     * @throws IllegalArgumentException if provided list of points don't have a
     * size greater or equal than MINIMUM_SIZE
     */
    public static SphereRobustEstimator create(
            SphereRobustEstimatorListener listener, List<Point3D> points,
            RobustEstimatorMethod method) throws IllegalArgumentException{
        switch(method){
            case LMedS:
                return new LMedSSphereRobustEstimator(listener, points);
            case MSAC:
                return new MSACSphereRobustEstimator(listener, points);
            case PROSAC:
                return new PROSACSphereRobustEstimator(listener, points);
            case PROMedS:
                return new PROMedSSphereRobustEstimator(listener, points);
            case RANSAC:
            default:
                return new RANSACSphereRobustEstimator(listener, points);
        }
    }
    
    /**
     * Creates a sphere robust estimator based on 3D point samples and using
     * provided robust estimator method
     * @param qualityScores quality scores corresponding to each provided point
     * @param method method of a robust estimator algorithm to estimate best
     * sphere
     * @return an instance of a sphere robust estimator
     * @throws IllegalArgumentException if provided quality scores length is
     * smaller than MINIMUM_SIZE (i.e. 4 points)
     */
    public static SphereRobustEstimator create(double[] qualityScores,
            RobustEstimatorMethod method) throws IllegalArgumentException{
        switch(method){
            case LMedS:
                return new LMedSSphereRobustEstimator();
            case MSAC:
                return new MSACSphereRobustEstimator();
            case PROSAC:
                return new PROSACSphereRobustEstimator(qualityScores);
            case PROMedS:
                return new PROMedSSphereRobustEstimator(qualityScores);
            case RANSAC:
            default:
                return new RANSACSphereRobustEstimator();
        }
    }
    
    /**
     * Creates a sphere robust estimator based on 3D point samples and using
     * provided points and robust estimator method
     * @param points 3D points to estimate a sphere
     * @param qualityScores quality scores corresponding to each provided point
     * @param method method of a robust estimator algorithm to estimate best
     * sphere
     * @return an instance of a sphere robust estimator
     * @throws IllegalArgumentException if provided list of points don't have 
     * the same size as the list of provided quality scores, or it their size 
     * is not greater or equal than MINIMUM_SIZE
     */
    public static SphereRobustEstimator create(List<Point3D> points, 
            double[] qualityScores, RobustEstimatorMethod method) 
            throws IllegalArgumentException{
        switch(method){
            case LMedS:
                return new LMedSSphereRobustEstimator(points);
            case MSAC:
                return new MSACSphereRobustEstimator(points);
            case PROSAC:
                return new PROSACSphereRobustEstimator(points, qualityScores);
            case PROMedS:
                return new PROMedSSphereRobustEstimator(points, qualityScores);
            case RANSAC:
            default:
                return new RANSACSphereRobustEstimator(points);
        }
    }
    
    /**
     * Creates a sphere robust estimator based on 3D point samples and using
     * provided listener
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     * @param qualityScores quality scores corresponding to each provided point
     * @param method method of a robust estimator algorithm to estimate best
     * sphere
     * @return an instance of a sphere robust estimator
     * @throws IllegalArgumentException if provided quality scores length is
     * smaller than MINIMUM_SIZE (i.e. 4 points)
     */
    public static SphereRobustEstimator create(
            SphereRobustEstimatorListener listener, double[] qualityScores,
            RobustEstimatorMethod method) throws IllegalArgumentException{
        switch(method){
            case LMedS:
                return new LMedSSphereRobustEstimator(listener);
            case MSAC:
                return new MSACSphereRobustEstimator(listener);
            case PROSAC:
                return new PROSACSphereRobustEstimator(listener, qualityScores);
            case PROMedS:
                return new PROMedSSphereRobustEstimator(listener, qualityScores);
            case RANSAC:
            default:
                return new RANSACSphereRobustEstimator(listener);
        }
    }
    
    /**
     * Creates a sphere robust estimator based on 3D point samples and using
     * provided listener and points
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     * @param points 3D points to estimate a sphere
     * @param qualityScores quality scores corresponding to each provided point
     * @param method method of a robust estimator algorithm to estimate best
     * sphere
     * @return an instance of a sphere robust estimator
     * @throws IllegalArgumentException if provided list of points don't have 
     * the same size as the list of provided quality scores, or it their size 
     * is not greater or equal than MINIMUM_SIZE
     */
    public static SphereRobustEstimator create(
            SphereRobustEstimatorListener listener, List<Point3D> points,
            double[] qualityScores, RobustEstimatorMethod method) 
            throws IllegalArgumentException{
        switch(method){
            case LMedS:
                return new LMedSSphereRobustEstimator(listener, points);
            case MSAC:
                return new MSACSphereRobustEstimator(listener, points);
            case PROSAC:
                return new PROSACSphereRobustEstimator(listener, points, 
                        qualityScores);
            case PROMedS:
                return new PROMedSSphereRobustEstimator(listener, points, 
                        qualityScores);
            case RANSAC:
            default:
                return new RANSACSphereRobustEstimator(listener, points);
        }
    }    
    
    /**
     * Creates a sphere robust estimator based on 3D point samples and using
     * default robust estimator method
     * @return an instance of a sphere robust estimator
     */
    public static SphereRobustEstimator create(){
        return create(DEFAULT_ROBUST_METHOD);
    }
    
    /**
     * Creates a sphere robust estimator based on 3D point samples and using
     * provided points and default robust estimator method
     * @param points 3D points to estimate a sphere
     * @return an instance of a sphere robust estimator
     * @throws IllegalArgumentException if provided list of points don't have a
     * size greater or equal than MINIMUM_SIZE
     */
    public static SphereRobustEstimator create(List<Point3D> points) 
            throws IllegalArgumentException{
        return create(points, DEFAULT_ROBUST_METHOD);
    }
    
    /**
     * Creates a sphere robust estimator based on 3D point samples and using
     * provided listener and default robust estimator method
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     * @return an instance of a sphere robust estimator
     */
    public static SphereRobustEstimator create(
            SphereRobustEstimatorListener listener){
        return create(listener, DEFAULT_ROBUST_METHOD);
    }
    
    /**
     * Creates a sphere robust estimator based on 3D point samples and using
     * provided listener and points and default robust estimator method
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     * @param points 3D points to estimate a sphere
     * @return an instance of a sphere robust estimator
     * @throws IllegalArgumentException if provided list of points don't have a
     * size greater or equal than MINIMUM_SIZE
     */
    public static SphereRobustEstimator create(
            SphereRobustEstimatorListener listener, List<Point3D> points) 
            throws IllegalArgumentException{
        return create(listener, points, DEFAULT_ROBUST_METHOD);
    }
    
    /**
     * Creates a sphere robust estimator based on 3D point samples and using
     * default robust estimator method
     * @param qualityScores quality scores corresponding to each provided point
     * @return an instance of a sphere robust estimator
     * @throws IllegalArgumentException if provided quality scores length is
     * smaller than MINIMUM_SIZE (i.e. 4 points)
     */
    public static SphereRobustEstimator create(double[] qualityScores) 
            throws IllegalArgumentException{
        return create(qualityScores, DEFAULT_ROBUST_METHOD);
    }
    
    /**
     * Creates a sphere robust estimator based on 3D point samples and using
     * provided points and default estimator method
     * @param points 3D points to estimate a sphere
     * @param qualityScores quality scores corresponding to each provided point
     * @return an instance of a sphere robust estimator
     * @throws IllegalArgumentException if provided list of points don't have 
     * the same size as the list of provided quality scores, or if their size 
     * is not greater or equal than MINIMUM_SIZE
     */
    public static SphereRobustEstimator create(List<Point3D> points, 
            double[] qualityScores) throws IllegalArgumentException{
        return create(points, qualityScores, DEFAULT_ROBUST_METHOD);
    }
    
    /**
     * Creates a sphere robust estimator based on 3D point samples and using
     * provided listener and default estimator method
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     * @param qualityScores quality scores corresponding to each provided point
     * @return an instance of a sphere robust estimator
     * @throws IllegalArgumentException if provided quality scores length is
     * smaller than MINIMUM_SIZE (i.e. 4 points)
     */
    public static SphereRobustEstimator create(
            SphereRobustEstimatorListener listener, double[] qualityScores)
            throws IllegalArgumentException{
        return create(listener, qualityScores, DEFAULT_ROBUST_METHOD);
    }
    
    /**
     * Creates a sphere robust estimator based on 3D point samples and using
     * provided listener and points and default estimator method
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     * @param points 3D points to estimate a sphere
     * @param qualityScores quality scores corresponding to each provided point
     * @return an instance of a sphere robust estimator
     * @throws IllegalArgumentException if provided list of points don't have 
     * the same size as the list of provided quality scores, or if their size 
     * is not greater or equal than MINIMUM_SIZE
     */
    public static SphereRobustEstimator create(
            SphereRobustEstimatorListener listener, List<Point3D> points,
            double[] qualityScores) throws IllegalArgumentException{
        return create(listener, points, qualityScores, DEFAULT_ROBUST_METHOD);
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
    public abstract Sphere estimate() throws LockedException, 
            NotReadyException, RobustEstimatorException;
        
    /**
     * Returns method being used for robust estimation
     * @return method being used for robust estimation
     */
    public abstract RobustEstimatorMethod getMethod();   
    
    /**
     * Internal method to set lists of points to be used to estimate a sphere.
     * This method does not check whether estimator is locked or not
     * @param points list of points to be used to estimate a sphere
     * @throws IllegalArgumentException if provided list of points doesn't have
     * a size greater or equal than MINIMUM_SIZE
     */
    private void internalSetPoints(List<Point3D> points) 
            throws IllegalArgumentException{
        if(points.size() < MINIMUM_SIZE) throw new IllegalArgumentException();
        mPoints = points;
    }
    
    /**
     * Computes the residual between a sphere and a point
     * @param c a sphere
     * @param point a 3D point
     * @return residual
     */
    protected double residual(Sphere c, Point3D point){
        point.normalize();
        
        return c.getDistance(point);
    }    
}