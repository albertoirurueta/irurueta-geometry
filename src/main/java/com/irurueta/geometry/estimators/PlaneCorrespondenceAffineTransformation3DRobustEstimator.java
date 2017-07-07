/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.estimators.PlaneCorrespondenceAffineTransformation3DRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date February 14, 2015
 */
package com.irurueta.geometry.estimators;

import com.irurueta.geometry.AffineTransformation3D;
import com.irurueta.geometry.Plane;
import com.irurueta.geometry.refiners.PlaneCorrespondenceAffineTransformation3DRefiner;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import java.util.List;

/**
 * This is an abstract class for algorithms to robustly find the best affine
 * 3D transformation for collections of matching planes.
 * Implementations of this class should be able to detect and discard outliers
 * in order to find the best solution
 */
public abstract class PlaneCorrespondenceAffineTransformation3DRobustEstimator 
        extends AffineTransformation3DRobustEstimator{
    
    /**
     * Default robust estimator method when none is provided
     */
    public static final RobustEstimatorMethod DEFAULT_ROBUST_METHOD =
            RobustEstimatorMethod.PROMedS;
    
    /**
     * List of planes to be used to estimate an affine 3D transformation.
     * Each line in the list of input lines must be matched with the
     * corresponding line in the list of output lines located at the same
     * position. Hence, both input lines and output lines must have the
     * same size, and their size must be greater or equal than MINIMUM_SIZE
     */
    protected List<Plane> mInputPlanes;
    
    /**
     * List of planes to be used to estimate an affine 3D transformation.
     * Each point in the list of output lines must be matched with the
     * corresponding line in the list of input lines located at the same
     * position. Hence, both input lines and output lines must have the
     * same size, and their size must be greater or equal than MINIMUM_SIZE
     */
    protected List<Plane> mOutputPlanes;
    
    /**
     * Constructor
     */
    public PlaneCorrespondenceAffineTransformation3DRobustEstimator(){
        super();
    }
    
    /**
     * Constructor with lists of planes to be used to estimate an affine 3D
     * transformation.
     * Planes in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MINIMUM_SIZE
     * @param inputPlanes list of input planes to be used to estimate an affine 
     * 3D transformation
     * @param outputPlanes list of output planes ot be used to estimate an 
     * affine 3D transformation
     * @throws IllegalArgumentException if provided lists of planes don't have
     * the same size or their size is smaller than MINIMUM_SIZE
     */
    public PlaneCorrespondenceAffineTransformation3DRobustEstimator(
            List<Plane> inputPlanes, List<Plane> outputPlanes) 
            throws IllegalArgumentException{
        super();
        internalSetPlanes(inputPlanes, outputPlanes);
    }
    
    /**
     * Constructor.
     * @param listener listener to be notified of events such as when estimation
     * stars, ends or its progress significantly changes
     */
    public PlaneCorrespondenceAffineTransformation3DRobustEstimator(
            AffineTransformation3DRobustEstimatorListener listener){
        super(listener);
    }
    
    /**
     * Constructor with listener and lists of planes to be used to estimate
     * affine 3D tranformation.
     * Planes in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MINIMUM_SIZE
     * @param listener listener to be notified of events such as when estimation 
     * starts, ends or its progress significantly changes
     * @param inputPlanes list of input planes to be used to estimate an affine
     * 3D transformation
     * @param outputPlanes list of output planes to be used to estimate an 
     * affine 3D transformation
     * @throws IllegalArgumentException if provided lists of planes don't have
     * the same size or their size is smaller than MINIMUM_SIZE
     */
    public PlaneCorrespondenceAffineTransformation3DRobustEstimator(
            AffineTransformation3DRobustEstimatorListener listener,
            List<Plane> inputPlanes, List<Plane> outputPlanes)
            throws IllegalArgumentException{
        super(listener);
        internalSetPlanes(inputPlanes, outputPlanes);
    }
    
    /**
     * Returns list of input planes to be used to estimate an affine 3D
     * transformation.
     * Each plane in the list of input planes must be matched with the
     * corresponding planes in the list of output planes located at the same
     * position. Hence, both input planes and output planes must have the same
     * size, and their size must be greater or equal than MINIMUM_SIZE
     * @return list of input planes to be used to estimate an affine 3D
     * transformation
     */
    public List<Plane> getInputPlanes(){
        return mInputPlanes;
    }
    
    /**
     * Returns list of output planes to be used to estimate an affine 3D
     * transformation.
     * Each plane in the list of output planes must be matched with the
     * corresponding plane in the list of input planes located at the same
     * position. Hence, both input planes and output planes must have the same
     * size, and their size must be greater or equal than MINIMUM_SIZE
     * @return list of output planes to be used to estimate an affine 3D
     * transformation
     */
    public List<Plane> getOutputPlanes(){
        return mOutputPlanes;
    }
    
    /**
     * Sets lists of planes to be used to estimate an affine 3D transformation.
     * Planes in the list located at the same position are considered to be 
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MINIMUM_SIZE
     * @param inputPlanes list of input planes to be used to estimate an affine
     * 3D transformation
     * @param outputPlanes list of output planes to be used to estimate an 
     * affine 3D transformation
     * @throws IllegalArgumentException if provided lists of planes don't have
     * the same size or their size is smaller than MINIMUM_SIZE
     * @throws LockedException if estimator is locked because a computation is
     * already in progress
     */
    public final void setPlanes(List<Plane> inputPlanes, 
            List<Plane> outputPlanes) throws IllegalArgumentException,
            LockedException{
        if(isLocked()) throw new LockedException();
        internalSetPlanes(inputPlanes, outputPlanes);
    }
    
    /**
     * Indicates if estimator is ready to start the affine 3D transformation
     * estimation.
     * This is true when input data (i.e. lists of matched planes) are provided
     * and a minimum of MINIMUM_SIZE lines are available
     * @return true if estimator is ready, false otherwise
     */
    public boolean isReady(){
        return mInputPlanes != null && mOutputPlanes != null &&
                mInputPlanes.size() == mOutputPlanes.size() &&
                mInputPlanes.size() >= MINIMUM_SIZE;
    }
    
    /**
     * Returns quality scores corresponding to each pair of matched planes.
     * The larger the score value the betther the quality of the matching.
     * This implementation always returns null.
     * Subclasses using quality scores must implement proper behaviour.
     * @return quality scores corresponding to each pair of matched points
     */
    public double[] getQualityScores(){
        return null;
    }    
    
    /**
     * Sets quality scores corresponding to each pair of matched planes.
     * The larger the score value the better the quality of the matching.
     * This implementation makes no action.
     * Subclasses using quality scores must implement proper behaviour.
     * @param qualityScores quality scores corresponding to each pair of matched
     * points
     * @throws LockedException if robust estimator is locked because an 
     * estimation is already in progress
     * @throws IllegalArgumentException if provided quality scores length is 
     * smaller than MINIMUM_SIZE (i.e. 3 samples)
     */
    public void setQualityScores(double[] qualityScores) throws LockedException,
            IllegalArgumentException{}        
    
    /**
     * Creates an affine 3D transformation estimator based on 3D plane
     * correspondences an using provided robust estimator method
     * @param method method of a robust estimator algorithm to estimate
     * best affine 3D transformation
     * @return an instance of affine 3D transformation estimator
     */
    public static PlaneCorrespondenceAffineTransformation3DRobustEstimator
            create(RobustEstimatorMethod method){
        switch(method){
            case LMedS:
                return new LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator();
            case MSAC:
                return new MSACPlaneCorrespondenceAffineTransformation3DRobustEstimator();
            case PROSAC:
                return new PROSACPlaneCorrespondenceAffineTransformation3DRobustEstimator();
            case PROMedS:
                return new PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator();
            case RANSAC:
            default:
                return new RANSACPlaneCorrespondenceAffineTransformation3DRobustEstimator();
        }
    }
            
    /**
     * Creates an affine 3D transformation estimator based on 3D plane
     * correspondences and using provided robust estimator method
     * @param inputPlanes list of input planes to be used to estimate an
     * affine 3D transformation
     * @param outputPlanes list of output planes to be used to estimate an
     * affine 3D transformation
     * @param method method of a robust estimator algorithm to estimate
     * best affine 3D transformation
     * @return an instance of affine 3D transformation estimator
     * @throws IllegalArgumentException if provided lists of lines don't have
     * the same size or their size is smaller than MINIMUM_SIZE
     */
    public static PlaneCorrespondenceAffineTransformation3DRobustEstimator
            create(List<Plane> inputPlanes, List<Plane> outputPlanes,
            RobustEstimatorMethod method) throws IllegalArgumentException{
        switch(method){
            case LMedS:
                return new LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                        inputPlanes, outputPlanes);
            case MSAC:
                return new MSACPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                        inputPlanes, outputPlanes);
            case PROSAC:
                return new PROSACPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                        inputPlanes, outputPlanes);
            case PROMedS:
                return new PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                        inputPlanes, outputPlanes);
            case RANSAC:
            default:
                return new RANSACPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                        inputPlanes, outputPlanes);
        }
    }
            
    /**
     * Creates an affine 3D transformation estimator based on 3D plane
     * correspondences and using provided robust estimator method
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     * @param method method of a robust estimator algorithm to estimate best
     * affine 3D transformation
     * @return an instance of affine 3D transformation estimator
     */
    public static PlaneCorrespondenceAffineTransformation3DRobustEstimator
            create(AffineTransformation3DRobustEstimatorListener listener,
            RobustEstimatorMethod method){
        switch(method){
            case LMedS:
                return new LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                        listener);
            case MSAC:
                return new MSACPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                        listener);
            case PROSAC:
                return new PROSACPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                        listener);
            case PROMedS:
                return new PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                        listener);
            case RANSAC:
            default:
                return new RANSACPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                        listener);
        }
    }
            
    /**
     * Creates an affine 3D transformation estimator based on 3D line
     * correspondences and using provided robust estimator method
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     * @param inputPlanes list of input lines to be used to estimate an affine
     * 3D transformation
     * @param outputPlanes list of output lines to be used to estimate an affine
     * 3D transformation
     * @param method method of a robust estimator algorithm to estimate best
     * affine 3D transformation
     * @return an instance of affine 3D transformation estimator
     * @throws IllegalArgumentException if provided lists of lines don't have
     * the same size or their size is smaller than MINIMUM_SIZE
     */
    public static PlaneCorrespondenceAffineTransformation3DRobustEstimator
            create(AffineTransformation3DRobustEstimatorListener listener,
            List<Plane> inputPlanes, List<Plane> outputPlanes,
            RobustEstimatorMethod method) throws IllegalArgumentException{
        switch(method){
            case LMedS:
                return new LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                        listener, inputPlanes, outputPlanes);
            case MSAC:
                return new MSACPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                        listener, inputPlanes, outputPlanes);
            case PROSAC:
                return new PROSACPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                        listener, inputPlanes, outputPlanes);
            case PROMedS:
                return new PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                        listener, inputPlanes, outputPlanes);
            case RANSAC:
            default:
                return new RANSACPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                        listener, inputPlanes, outputPlanes);
        }
    }
            
    /**
     * Creates an affine 3D transformation estimator based on 3D plane
     * correspondences and using provided robust estimator method
     * @param qualityScores quality scores corresponding to each pair of matched
     planes.
     * @param method method of a robust estimator algorithm to estimate best
     * affine 3D transformation
     * @return an instance of affine 3D transformation estimator
     */
    public static PlaneCorrespondenceAffineTransformation3DRobustEstimator
            create(double[] qualityScores, RobustEstimatorMethod method){
        switch(method){
            case LMedS:
                return new LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator();
            case MSAC:
                return new MSACPlaneCorrespondenceAffineTransformation3DRobustEstimator();
            case PROSAC:
                return new PROSACPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                        qualityScores);
            case PROMedS:
                return new PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                        qualityScores);
            case RANSAC:
            default:
                return new RANSACPlaneCorrespondenceAffineTransformation3DRobustEstimator();
        }
    }
            
    /**
     * Creates an affine 3D transformation estimator based on plane
     * correspondences and using provided robust estimator method
     * @param inputPlanes list of input planes to be used to estimate an
     * affine 3D transformation
     * @param outputPlanes list of output planes to be used to estimate an
     * affine 3D transformation
     * @param qualityScores quality scores corresponding to each pair of matched
     * planes.
     * @param method method of a robust estimator algorithm to estimate best
     * affine 3D transformation
     * @return an instance of affine 3D transformation estimator
     * @throws IllegalArgumentException if provided lists of lines don't have
     * the same size or their size is smaller than MINIMUM_SIZE
     */
    public static PlaneCorrespondenceAffineTransformation3DRobustEstimator
            create(List<Plane> inputPlanes, List<Plane> outputPlanes,
            double[] qualityScores, RobustEstimatorMethod method)
            throws IllegalArgumentException{
        switch(method){
            case LMedS:
                return new LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                        inputPlanes, outputPlanes);
            case MSAC:
                return new MSACPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                        inputPlanes, outputPlanes);
            case PROSAC:
                return new PROSACPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                        inputPlanes, outputPlanes, qualityScores);
            case PROMedS:
                return new PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                        inputPlanes, outputPlanes, qualityScores);
            case RANSAC:
            default:
                return new RANSACPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                        inputPlanes, outputPlanes);
        }
    }
            
    /**
     * Creates an affine 3D transformation estimator based on plane
     * correspondences and using provided robust estimator method
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     * @param qualityScores quality scores corresponding to each pair of matched
     * lines.
     * @param method method of a robust estimator algorithm to estimate best
     * affine 3D transformation
     * @return an instance of affine 3D transformation estimator
     */
    public static PlaneCorrespondenceAffineTransformation3DRobustEstimator
            create(AffineTransformation3DRobustEstimatorListener listener,
            double[] qualityScores, RobustEstimatorMethod method){
        switch(method){
            case LMedS:
                return new LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                        listener);
            case MSAC:
                return new MSACPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                        listener);
            case PROSAC:
                return new PROSACPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                        listener, qualityScores);
            case PROMedS:
                return new PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                        listener, qualityScores);
            case RANSAC:
            default:
                return new RANSACPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                        listener);
        }
    }
            
    /**
     * Creates an affine 3D transformation estimator based on plane
     * correspondences and using provided robust estimator method
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     * @param inputPlanes list of input planes to be used to estimate an affine
     * 3D transformation
     * @param outputPlanes list of output planes to be used to estimate an affine
     * 3D transformation
     * @param qualityScores quality scores corresponding to each pair of matched
     * planes.
     * @param method method of a robust estimator algorithm to estimate best
     * affine 3D transformation
     * @return an instance of affine 3D transformation estimator
     * @throws IllegalArgumentException if provided lists of lines don't have
     * the same size or their size is smaller than MINIMUM_SIZE
     */
    public static PlaneCorrespondenceAffineTransformation3DRobustEstimator
            create(AffineTransformation3DRobustEstimatorListener listener,
            List<Plane> inputPlanes, List<Plane> outputPlanes,
            double[] qualityScores, RobustEstimatorMethod method)
            throws IllegalArgumentException{
        switch(method){
            case LMedS:
                return new LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                        listener, inputPlanes, outputPlanes);
            case MSAC:
                return new MSACPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                        listener, inputPlanes, outputPlanes);
            case PROSAC:
                return new PROSACPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                        listener, inputPlanes, outputPlanes, qualityScores);
            case PROMedS:
                return new PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                        listener, inputPlanes, outputPlanes, qualityScores);
            case RANSAC:
            default:
                return new RANSACPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                        listener, inputPlanes, outputPlanes);
        }
    }
    
    /**
     * Creates an affine 3D transformation estimator based on plane
     * correspondences and using default robust estimator method
     * @return an instance of affine 3D transformation estimator
     */
    public static PlaneCorrespondenceAffineTransformation3DRobustEstimator
            create(){
        return create(DEFAULT_ROBUST_METHOD);
    }
            
    /**
     * Creates an affine 3D transformation estimator based on plane
     * correspondences and using default robust estimator method
     * @param inputPlanes list of input planes to be used to estimate an
     * affine 3D transformation
     * @param outputPlanes list of output planes to be used to estimate an
     * affine 3D transformation
     * @return an instance of affine 3D transformation estimator
     * @throws IllegalArgumentException if provided lists of planes don't have
     * the same size or their size is smaller than MINIMUM_SIZE
     */
    public static PlaneCorrespondenceAffineTransformation3DRobustEstimator
            create(List<Plane> inputPlanes, List<Plane> outputPlanes)
            throws IllegalArgumentException{
        return create(inputPlanes, outputPlanes, DEFAULT_ROBUST_METHOD);
    }
            
    /**
     * Creates an affine 3D transformation estimator based on plane
     * correspondences and using default robust estimator method
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     * @return an instance of affine 3D transformation estimator
     */
    public static PlaneCorrespondenceAffineTransformation3DRobustEstimator
            create(AffineTransformation3DRobustEstimatorListener listener){
        return create(listener, DEFAULT_ROBUST_METHOD);
    }
            
    /**
     * Creates an affine 3D transformation estimator based on plane
     * correspondences and using default robust estimator method
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     * @param inputPlanes list of input planes to be used to estimate an affine
     * 3D transformation
     * @param outputPlanes list of output planes to be used to estimate an 
     * affine 3D transformation
     * @return an instance of affine 3D transformation estimator
     * @throws IllegalArgumentException if provided lists of lines don't have
     * the same size or their size is smaller than MINIMUM_SIZE
     */
    public static PlaneCorrespondenceAffineTransformation3DRobustEstimator
            create(AffineTransformation3DRobustEstimatorListener listener,
            List<Plane> inputPlanes, List<Plane> outputPlanes)
            throws IllegalArgumentException{
        return create(listener, inputPlanes, outputPlanes, 
                DEFAULT_ROBUST_METHOD);
    }
            
    /**
     * Creates an affine 3D transformation estimator based on plane
     * correspondences and using default robust estimator method
     * @param qualityScores quality scores corresponding to each pair of matched
     * planes.
     * @return an instance of affine 3D transformation estimator 
     */
    public static PlaneCorrespondenceAffineTransformation3DRobustEstimator
            create(double[] qualityScores){
        return create(qualityScores, DEFAULT_ROBUST_METHOD);
    }
            
    /**
     * Creates an affine 3D transformation estimator based on 3D line
     * correspondences and using default robust estimator method
     * @param inputPlanes list of input planes to be used to estimate an affine
     * 3D transformation
     * @param outputPlanes list of output planes to be used to estimate an affine
     * 3D transformation
     * @param qualityScores quality scores corresponding to each pair of matched
     * planes.
     * @return an instance of affine 3D transformation estimator
     * @throws IllegalArgumentException if provided lists of lines don't have 
     * the same size or their size is smaller than MINIMUM_SIZE
     */
    public static PlaneCorrespondenceAffineTransformation3DRobustEstimator
            create(List<Plane> inputPlanes, List<Plane> outputPlanes,
            double[] qualityScores) throws IllegalArgumentException{
        return create(inputPlanes, outputPlanes, qualityScores, 
                DEFAULT_ROBUST_METHOD);
    }
            
    /**
     * Creates an affine 3D transformation estimator based on 3D line
     * correspondences and using default robust estimator method
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     * @param qualityScores quality scores corresponding to each pair of matched
     * points.
     * @return an instance of affine 3D transformation estimator
     */
    public static PlaneCorrespondenceAffineTransformation3DRobustEstimator
            create(AffineTransformation3DRobustEstimatorListener listener,
            double[] qualityScores){
        return create(listener, qualityScores, DEFAULT_ROBUST_METHOD);
    }
            
    /**
     * Creates an affine 3D transformation estimator based on plane
     * correspondences and using default robust estimator method
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     * @param inputPlanes list of input planes to be used to estimate an affine
     * 3D transformation
     * @param outputPlanes list of output planes to be used to estimate an 
     * affine 3D transformation
     * @param qualityScores quality scores corresponding to each pair of matched
     * lines.
     * @return an instance of affine 3D transformation estimator
     * @throws IllegalArgumentException if provided lists of lines don't have
     * the same size or their size is smaller than MINIMUM_SIZE
     */
    public static PlaneCorrespondenceAffineTransformation3DRobustEstimator
            create(AffineTransformation3DRobustEstimatorListener listener,
            List<Plane> inputPlanes, List<Plane> outputPlanes,
            double[] qualityScores) throws IllegalArgumentException{
        return create(listener, inputPlanes, outputPlanes, qualityScores,
                DEFAULT_ROBUST_METHOD);
    }
            
    /**
     * Internal method to set lists of planes to be used to estimate an affine
     * 3D transformation.
     * This method does not check whether estimator is locked or not
     * @param inputPlanes list of input planes to be used to estimate an affine
     * 3D transformation
     * @param outputPlanes list of output planes to be used to estimate an 
     * affine 3D transformation
     * @throws IllegalArgumentException if provided lists of lines don't have 
     * the same size or their size is smaller than MINIMUM_SIZE
     */
    private void internalSetPlanes(List<Plane> inputPlanes, 
            List<Plane> outputPlanes) throws IllegalArgumentException{
        if(inputPlanes.size() < MINIMUM_SIZE) 
            throw new IllegalArgumentException();
        if(inputPlanes.size() != outputPlanes.size())
            throw new IllegalArgumentException();
        mInputPlanes = inputPlanes;
        mOutputPlanes = outputPlanes;
    }
    
    /**
     * Computes residual by comparing two lines algebraically by doing the
     * dot product of their parameters.
     * A residual of 0 indicates that dot product was 1 or -1 and lines were 
     * equal.
     * A residual of 1 indicates that dot product was 0 and lines were 
     * orthogonal.
     * If dot product was -1, then although their director vectors are opposed,
     * lines are considered equal, since sign changes are not taken into account
     * @param plane originally sampled output plane
     * @param transformedPlane estimated output plane obtained after using 
     * estimated transformation
     * @return computed residual
     */
    protected static double getResidual(Plane plane, Plane transformedPlane){
        plane.normalize();
        transformedPlane.normalize();
        
        double dotProduct = Math.abs(plane.getA() * transformedPlane.getA() +
                plane.getB() * transformedPlane.getB() +
                plane.getC() * transformedPlane.getC() +
                plane.getD() * transformedPlane.getD());
        return 1.0 - dotProduct;
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
            PlaneCorrespondenceAffineTransformation3DRefiner refiner =
                    new PlaneCorrespondenceAffineTransformation3DRefiner(
                    transformation, mKeepCovariance, getInliersData(),
                    mInputPlanes, mOutputPlanes, 
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
}
