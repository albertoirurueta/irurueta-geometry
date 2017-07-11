/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.estimators.PointCorrespondenceAffineTransformation3DRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date February 14, 2015
 */
package com.irurueta.geometry.estimators;

import com.irurueta.geometry.AffineTransformation3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.refiners.PointCorrespondenceAffineTransformation3DRefiner;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import java.util.List;

/**
 * This is an abstract class for algorithms to robustly find the best affine
 * 3D transformation for collections of matching 3D points.
 * Implementations of this class should be able to detect and discard outliers
 * in order to find the best solution.
 */
public abstract class PointCorrespondenceAffineTransformation3DRobustEstimator 
        extends AffineTransformation3DRobustEstimator {

    /**
     * Default robust estimator method when none is provided.
     */
    public static final RobustEstimatorMethod DEFAULT_ROBUST_METHOD = 
            RobustEstimatorMethod.PROMedS; 
    
    /**
     * List of points to be used to estimate an affine 3D transformation.
     * Each point in the list of input points must be matched with the 
     * corresponding point in the list of output points located at the same 
     * position. Hence, both input points and output points must have the same 
     * size, and their size must be greater or equal than MINIMUM_SIZE.
     */
    protected List<Point3D> mInputPoints;
    
    /**
     * List of points to be used to estimate an affine 3D transformation.
     * Each point in the list of output points must be matched with the 
     * corresponding point in the list of input points located at the same 
     * position. Hence, both input points and output points must have the same 
     * size, and their size must be greater or equal than MINIMUM_SIZE.
     */
    protected List<Point3D> mOutputPoints;
        
    /**
     * Constructor.
     */
    public PointCorrespondenceAffineTransformation3DRobustEstimator() {
        super();
    }
    
    /**
     * Constructor with lists of points to be used to estimate an affine 3D 
     * transformation.
     * Points in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MINIMUM_SIZE.
     * @param inputPoints list of input points to be used to estimate an 
     * affine 3D transformation.
     * @param outputPoints list of output points to be used to estimate an 
     * affine 3D transformation.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than MINIMUM_SIZE.
     */
    public PointCorrespondenceAffineTransformation3DRobustEstimator(
            List<Point3D> inputPoints, List<Point3D> outputPoints) 
            throws IllegalArgumentException {
        super();
        internalSetPoints(inputPoints, outputPoints);
    }
    
    /**
     * Constructor.
     * @param listener listener to be notified of events such as when estimation
     * stars, ends or its progress significantly changes.
     */
    public PointCorrespondenceAffineTransformation3DRobustEstimator(
            AffineTransformation3DRobustEstimatorListener listener) {
        super(listener);
    }
    
    /**
     * Constructor with listener and lists of points to be used to estimate an
     * affine 3D transformation.
     * Points in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MINIMUM_SIZE.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param inputPoints list of input points to be used to estimate an 
     * affine 3D transformation.
     * @param outputPoints list of output points to be used to estimate an 
     * affine 3D transformation.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than MINIMUM_SIZE.
     */
    public PointCorrespondenceAffineTransformation3DRobustEstimator(
            AffineTransformation3DRobustEstimatorListener listener,
            List<Point3D> inputPoints, List<Point3D> outputPoints) 
            throws IllegalArgumentException {
        super(listener);
        internalSetPoints(inputPoints, outputPoints);
    }
    
    /**
     * Returns list of input points to be used to estimate an affine 3D 
     * transformation.
     * Each point in the list of input points must be matched with the 
     * corresponding point in the list of output points located at the same 
     * position. Hence, both input points and output points must have the same 
     * size, and their size must be greater or equal than MINIMUM_SIZE.
     * @return list of input points to be used to estimate an affine 3D 
     * transformation.
     */
    public List<Point3D> getInputPoints() {
        return mInputPoints;
    }    
    
    /**
     * Returns list of output points to be used to estimate an affine 3D 
     * transformation.
     * Each point in the list of output points must be matched with the 
     * corresponding point in the list of input points located at the same 
     * position. Hence, both input points and output points must have the same 
     * size, and their size must be greater or equal than MINIMUM_SIZE.
     * @return list of output points to be used to estimate an affine 2D 
     * transformation.
     */
    public List<Point3D> getOutputPoints() {
        return mOutputPoints;
    }
    
    /**
     * Sets lists of points to be used to estimate an affine 3D transformation.
     * Points in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MINIMUM_SIZE.
     * @param inputPoints list of input points to be used to estimate an 
     * affine 3D transformation.
     * @param outputPoints list of output points to be used to estimate an 
     * affine 3D transformation.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than MINIMUM_SIZE.
     * @throws LockedException if estimator is locked because a computation is
     * already in progress.
     */
    public final void setPoints(List<Point3D> inputPoints, 
            List<Point3D> outputPoints) throws IllegalArgumentException, 
            LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetPoints(inputPoints, outputPoints);
    }
        
    /**
     * Indicates if estimator is ready to start the affine 3D transformation
     * estimation.
     * This is true when input data (i.e. lists of matched points) are provided
     * and a minimum of MINIMUM_SIZE points are available.
     * @return true if estimator is ready, false otherwise.
     */
    public boolean isReady() {
        return mInputPoints != null && mOutputPoints != null && 
                mInputPoints.size() == mOutputPoints.size() &&
                mInputPoints.size() >= MINIMUM_SIZE;
    }
    
    /**
     * Returns quality scores corresponding to each pair of matched points.
     * The larger the score value the betther the quality of the matching.
     * This implementation always returns null.
     * Subclasses using quality scores must implement proper behaviour.
     * @return quality scores corresponding to each pair of matched points.
     */
    public double[] getQualityScores() {
        return null;
    }    
    
    /**
     * Sets quality scores corresponding to each pair of matched points.
     * The larger the score value the better the quality of the matching.
     * This implementation makes no action.
     * Subclasses using quality scores must implement proper behaviour.
     * @param qualityScores quality scores corresponding to each pair of matched
     * points.
     * @throws LockedException if robust estimator is locked because an 
     * estimation is already in progress.
     * @throws IllegalArgumentException if provided quality scores length is 
     * smaller than MINIMUM_SIZE (i.e. 3 samples).
     */
    public void setQualityScores(double[] qualityScores) throws LockedException,
            IllegalArgumentException { }    
    
    /**
     * Creates an affine 3D transformation estimator based on 2D point 
     * correspondences and using provided robust estimator method.
     * @param method method of a robust estimator algorithm to estimate
     * best affine 3D transformation.
     * @return an instance of affine 3D transformation estimator.
     */
    public static PointCorrespondenceAffineTransformation3DRobustEstimator 
            create(RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSPointCorrespondenceAffineTransformation3DRobustEstimator();
            case MSAC:
                return new MSACPointCorrespondenceAffineTransformation3DRobustEstimator();
            case PROSAC:
                return new PROSACPointCorrespondenceAffineTransformation3DRobustEstimator();
            case PROMedS:
                return new PROMedSPointCorrespondenceAffineTransformation3DRobustEstimator();
            case RANSAC:
            default:
                return new RANSACPointCorrespondenceAffineTransformation3DRobustEstimator();
        }
    }
            
    /**
     * Creates an affine 3D transformation estimator based on 3D point 
     * correspondences and using provided robust estimator method.
     * @param inputPoints list of input points to be used to estimate an 
     * affine 3D transformation.
     * @param outputPoints list of output points to be used to estimate an 
     * affine 3D transformation.
     * @param method method of a robust estimator algorithm to estimate
     * best affine 3D transformation.
     * @return an instance of affine 3D transformation estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static PointCorrespondenceAffineTransformation3DRobustEstimator
            create(List<Point3D> inputPoints, List<Point3D> outputPoints, 
            RobustEstimatorMethod method) throws IllegalArgumentException {
        switch(method) {
            case LMedS:
                return new LMedSPointCorrespondenceAffineTransformation3DRobustEstimator(
                        inputPoints, outputPoints);
            case MSAC:
                return new MSACPointCorrespondenceAffineTransformation3DRobustEstimator(
                        inputPoints, outputPoints);
            case PROSAC:
                return new PROSACPointCorrespondenceAffineTransformation3DRobustEstimator(
                        inputPoints, outputPoints);
            case PROMedS:
                return new PROMedSPointCorrespondenceAffineTransformation3DRobustEstimator(
                        inputPoints, outputPoints);
            case RANSAC:
            default:
                return new RANSACPointCorrespondenceAffineTransformation3DRobustEstimator(
                        inputPoints, outputPoints);
        }
    }            
            
    /**
     * Creates an affine 3D transformation estimator based on 3D point 
     * correspondences and using provided robust estimator method.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param method method of a robust estimator algorithm to estimate
     * best affine 3D transformation.
     * @return an instance of affine 3D transformation estimator.
     */
    public static PointCorrespondenceAffineTransformation3DRobustEstimator 
            create(AffineTransformation3DRobustEstimatorListener listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSPointCorrespondenceAffineTransformation3DRobustEstimator(
                        listener);
            case MSAC:
                return new MSACPointCorrespondenceAffineTransformation3DRobustEstimator(
                        listener);
            case PROSAC:
                return new PROSACPointCorrespondenceAffineTransformation3DRobustEstimator(
                        listener);
            case PROMedS:
                return new PROMedSPointCorrespondenceAffineTransformation3DRobustEstimator(
                        listener);
            case RANSAC:
            default:
                return new RANSACPointCorrespondenceAffineTransformation3DRobustEstimator(
                        listener);
        }
    }
            
    /**
     * Creates an affine 3D transformation estimator based on 3D point 
     * correspondences and using provided robust estimator method.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param inputPoints list of input points to be used to estimate an 
     * affine 3D transformation.
     * @param outputPoints list of output points to be used to estimate an 
     * affine 3D transformation.
     * @param method method of a robust estimator algorithm to estimate
     * best affine 3D transformation.
     * @return an instance of affine 3D transformation estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static PointCorrespondenceAffineTransformation3DRobustEstimator
            create(AffineTransformation3DRobustEstimatorListener listener, 
            List<Point3D> inputPoints, List<Point3D> outputPoints, 
            RobustEstimatorMethod method) throws IllegalArgumentException {
        switch (method) {
            case LMedS:
                return new LMedSPointCorrespondenceAffineTransformation3DRobustEstimator(
                        listener, inputPoints, outputPoints);
            case MSAC:
                return new MSACPointCorrespondenceAffineTransformation3DRobustEstimator(
                        listener, inputPoints, outputPoints);
            case PROSAC:
                return new PROSACPointCorrespondenceAffineTransformation3DRobustEstimator(
                        listener, inputPoints, outputPoints);
            case PROMedS:
                return new PROMedSPointCorrespondenceAffineTransformation3DRobustEstimator(
                        listener, inputPoints, outputPoints);
            case RANSAC:
            default:
                return new RANSACPointCorrespondenceAffineTransformation3DRobustEstimator(
                        listener, inputPoints, outputPoints);
        }
    }            

    /**
     * Creates an affine 3D transformation estimator based on 3D point 
     * correspondences and using provided robust estimator method.
     * @param qualityScores quality scores corresponding to each pair of matched 
     * points.
     * @param method method of a robust estimator algorithm to estimate
     * best affine 3D transformation.
     * @return an instance of affine 3D transformation estimator.
     */
    public static PointCorrespondenceAffineTransformation3DRobustEstimator 
            create(double[] qualityScores, RobustEstimatorMethod method) {
        switch (method) { 
            case LMedS:
                return new LMedSPointCorrespondenceAffineTransformation3DRobustEstimator();
            case MSAC:
                return new MSACPointCorrespondenceAffineTransformation3DRobustEstimator();
            case PROSAC:
                return new PROSACPointCorrespondenceAffineTransformation3DRobustEstimator(
                        qualityScores);
            case PROMedS:
                return new PROMedSPointCorrespondenceAffineTransformation3DRobustEstimator(
                        qualityScores);
            case RANSAC:
            default:
                return new RANSACPointCorrespondenceAffineTransformation3DRobustEstimator();
        }
    }
            
    /**
     * Creates an affine 3D transformation estimator based on 3D point 
     * correspondences and using provided robust estimator method.
     * @param inputPoints list of input points to be used to estimate an 
     * affine 3D transformation.
     * @param outputPoints list of output points to be used to estimate an 
     * affine 3D transformation.
     * @param qualityScores quality scores corresponding to each pair of matched 
     * points.
     * @param method method of a robust estimator algorithm to estimate
     * best affine 3D transformation.
     * @return an instance of affine 3D transformation estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static PointCorrespondenceAffineTransformation3DRobustEstimator
            create(List<Point3D> inputPoints, List<Point3D> outputPoints, 
            double[] qualityScores, RobustEstimatorMethod method) 
            throws IllegalArgumentException {
        switch (method) {
            case LMedS:
                return new LMedSPointCorrespondenceAffineTransformation3DRobustEstimator(
                        inputPoints, outputPoints);
            case MSAC:
                return new MSACPointCorrespondenceAffineTransformation3DRobustEstimator(
                        inputPoints, outputPoints);
            case PROSAC:
                return new PROSACPointCorrespondenceAffineTransformation3DRobustEstimator(
                        inputPoints, outputPoints, qualityScores);
            case PROMedS:
                return new PROMedSPointCorrespondenceAffineTransformation3DRobustEstimator(
                        inputPoints, outputPoints, qualityScores);
            case RANSAC:
            default:
                return new RANSACPointCorrespondenceAffineTransformation3DRobustEstimator(
                        inputPoints, outputPoints);
        }
    }            
            
    /**
     * Creates an affine 3D transformation estimator based on 3D point 
     * correspondences and using provided robust estimator method.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param qualityScores quality scores corresponding to each pair of matched 
     * points.
     * @param method method of a robust estimator algorithm to estimate
     * best affine 3D transformation.
     * @return an instance of affine 3D transformation estimator.
     */
    public static PointCorrespondenceAffineTransformation3DRobustEstimator 
            create(AffineTransformation3DRobustEstimatorListener listener,
            double[] qualityScores, RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSPointCorrespondenceAffineTransformation3DRobustEstimator(
                        listener);
            case MSAC:
                return new MSACPointCorrespondenceAffineTransformation3DRobustEstimator(
                        listener);
            case PROSAC:
                return new PROSACPointCorrespondenceAffineTransformation3DRobustEstimator(
                        listener, qualityScores);
            case PROMedS:
                return new PROMedSPointCorrespondenceAffineTransformation3DRobustEstimator(
                        listener, qualityScores);
            case RANSAC:
            default:
                return new RANSACPointCorrespondenceAffineTransformation3DRobustEstimator(
                        listener);
        }
    }
            
    /**
     * Creates an affine 3D transformation estimator based on 3D point 
     * correspondences and using provided robust estimator method.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param inputPoints list of input points to be used to estimate an 
     * affine 3D transformation.
     * @param outputPoints list of output points to be used to estimate an 
     * affine 3D transformation.
     * @param qualityScores quality scores corresponding to each pair of matched 
     * points.
     * @param method method of a robust estimator algorithm to estimate
     * best affine 3D transformation.
     * @return an instance of affine 3D transformation estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static PointCorrespondenceAffineTransformation3DRobustEstimator
            create(AffineTransformation3DRobustEstimatorListener listener, 
            List<Point3D> inputPoints, List<Point3D> outputPoints, 
            double[] qualityScores, RobustEstimatorMethod method) 
            throws IllegalArgumentException {
        switch (method) {
            case LMedS:
                return new LMedSPointCorrespondenceAffineTransformation3DRobustEstimator(
                        listener, inputPoints, outputPoints);
            case MSAC:
                return new MSACPointCorrespondenceAffineTransformation3DRobustEstimator(
                        listener, inputPoints, outputPoints);
            case PROSAC:
                return new PROSACPointCorrespondenceAffineTransformation3DRobustEstimator(
                        listener, inputPoints, outputPoints, qualityScores);
            case PROMedS:
                return new PROMedSPointCorrespondenceAffineTransformation3DRobustEstimator(
                        listener, inputPoints, outputPoints, qualityScores);
            case RANSAC:
            default:
                return new RANSACPointCorrespondenceAffineTransformation3DRobustEstimator(
                        listener, inputPoints, outputPoints);
        }
    }            
            
    /**
     * Creates an affine 3D transformation estimator based on 3D point 
     * correspondences and using default robust estimator method.
     * @return an instance of affine 3D transformation estimator.
     */
    public static PointCorrespondenceAffineTransformation3DRobustEstimator 
            create() {
        return create(DEFAULT_ROBUST_METHOD);
    }
            
    /**
     * Creates an affine 3D transformation estimator based on 3D point 
     * correspondences and using default robust estimator method.
     * @param inputPoints list of input points to be used to estimate an 
     * affine 3D transformation.
     * @param outputPoints list of output points to be used to estimate an 
     * affine 3D transformation.
     * @return an instance of affine 3D transformation estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static PointCorrespondenceAffineTransformation3DRobustEstimator
            create(List<Point3D> inputPoints, List<Point3D> outputPoints) 
            throws IllegalArgumentException {
        return create(inputPoints, outputPoints, DEFAULT_ROBUST_METHOD);
    }            
            
    /**
     * Creates an affine 3D transformation estimator based on 3D point 
     * correspondences and using default robust estimator method.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @return an instance of affine 3D transformation estimator.
     */
    public static PointCorrespondenceAffineTransformation3DRobustEstimator 
            create(AffineTransformation3DRobustEstimatorListener listener) {
        return create(listener, DEFAULT_ROBUST_METHOD);
    }
            
    /**
     * Creates an affine 3D transformation estimator based on 3D point 
     * correspondences and using default robust estimator method.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param inputPoints list of input points to be used to estimate an 
     * affine 3D transformation.
     * @param outputPoints list of output points to be used to estimate an 
     * affine 3D transformation.
     * @return an instance of affine 3D transformation estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static PointCorrespondenceAffineTransformation3DRobustEstimator
            create(AffineTransformation3DRobustEstimatorListener listener, 
            List<Point3D> inputPoints, List<Point3D> outputPoints) 
            throws IllegalArgumentException {
        return create(listener, inputPoints, outputPoints, 
                DEFAULT_ROBUST_METHOD);
    }            

    /**
     * Creates an affine 3D transformation estimator based on 3D point 
     * correspondences and using default robust estimator method.
     * @param qualityScores quality scores corresponding to each pair of matched 
     * points.
     * @return an instance of affine 3D transformation estimator.
     */
    public static PointCorrespondenceAffineTransformation3DRobustEstimator 
            create(double[] qualityScores) {
        return create(qualityScores, DEFAULT_ROBUST_METHOD);
    }
            
    /**
     * Creates an affine 3D transformation estimator based on 3D point 
     * correspondences and using default robust estimator method.
     * @param inputPoints list of input points to be used to estimate an 
     * affine 3D transformation.
     * @param outputPoints list of output points to be used to estimate an 
     * affine 3D transformation.
     * @param qualityScores quality scores corresponding to each pair of matched 
     * points.
     * @return an instance of affine 3D transformation estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static PointCorrespondenceAffineTransformation3DRobustEstimator
            create(List<Point3D> inputPoints, List<Point3D> outputPoints, 
            double[] qualityScores) throws IllegalArgumentException {
        return create(inputPoints, outputPoints, qualityScores, 
                DEFAULT_ROBUST_METHOD);
    }            
            
    /**
     * Creates an affine 3D transformation estimator based on 3D point 
     * correspondences and using default robust estimator method.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param qualityScores quality scores corresponding to each pair of matched 
     * points.
     * @return an instance of affine 3D transformation estimator.
     */
    public static PointCorrespondenceAffineTransformation3DRobustEstimator 
            create(AffineTransformation3DRobustEstimatorListener listener,
            double[] qualityScores) {
        return create(listener, qualityScores, DEFAULT_ROBUST_METHOD);
    }
            
    /**
     * Creates an affine 3D transformation estimator based on 3D point 
     * correspondences and using default robust estimator method.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param inputPoints list of input points to be used to estimate an 
     * affine 3D transformation.
     * @param outputPoints list of output points to be used to estimate an 
     * affine 3D transformation.
     * @param qualityScores quality scores corresponding to each pair of matched 
     * points.
     * @return an instance of affine 3D transformation estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static PointCorrespondenceAffineTransformation3DRobustEstimator
            create(AffineTransformation3DRobustEstimatorListener listener, 
            List<Point3D> inputPoints, List<Point3D> outputPoints, 
            double[] qualityScores) throws IllegalArgumentException {
        return create(listener, inputPoints, outputPoints, qualityScores,
                DEFAULT_ROBUST_METHOD);
    }    
         
    /**
     * Attempts to refine provided solution if refinement is requested.
     * This method returns a refined solution of the same provided solution
     * if refinement is not requested or has failed.
     * If refinement is enabled and it is requested to keep covariance, this
     * method will also keep covariance of refined transformation.
     * @param transformation transformation estimated by a robust estimator 
     * without refinement.
     * @return solution after refinement (if requested) or the provided 
     * non-refined solution if not requested or refinement failed.
     */            
    protected AffineTransformation3D attemptRefine(
            AffineTransformation3D transformation) {
        if (mRefineResult) {
            PointCorrespondenceAffineTransformation3DRefiner refiner =
                    new PointCorrespondenceAffineTransformation3DRefiner(
                    transformation, mKeepCovariance, getInliersData(), 
                    mInputPoints, mOutputPoints, 
                    getRefinementStandardDeviation());
            
            try {
                AffineTransformation3D result = new AffineTransformation3D();
                boolean improved = refiner.refine(result);
                
                if (mKeepCovariance) {
                    //keep covariance
                    mCovariance = refiner.getCovariance();
                }
                
                return improved ? result : transformation;
            } catch (Exception e) {
                //refinement failed, so we return input value
                return transformation;
            }
        } else {
            return transformation;
        }        
    }
            
    /**
     * Internal method to set lists of points to be used to estimate an affine 
     * 3D transformation.
     * This method does not check whether estimator is locked or not.
     * @param inputPoints list of input points to be used to estimate an 
     * affine 3D transformation.
     * @param outputPoints list of output points to be used to estimate an 
     * affine 3D transformation.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than MINIMUM_SIZE.
     */
    private void internalSetPoints(List<Point3D> inputPoints, 
            List<Point3D> outputPoints) throws IllegalArgumentException {
        if (inputPoints.size() < MINIMUM_SIZE) {
            throw new IllegalArgumentException();
        }
        if (inputPoints.size() != outputPoints.size()) {
            throw new IllegalArgumentException();
        }
        mInputPoints = inputPoints;
        mOutputPoints = outputPoints;        
    }                
}