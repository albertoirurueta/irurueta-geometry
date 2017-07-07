/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.estimators.EPnPPointCorrespondencePinholeCameraRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date February 20, 2017.
 */
package com.irurueta.geometry.estimators;

import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.Point3D;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import java.util.List;

/**
 * Base abstract class for algorithms to robustly find the best pinhole camera 
 * for collections of matched 3D/2D points using EPnP (Efficient 
 * Perspective-n-Point) algorithm.
 * Implementations of this class should be able to detect and discard outliers 
 * in order to find the best solution.
 */
public abstract class EPnPPointCorrespondencePinholeCameraRobustEstimator 
        extends PointCorrespondencePinholeCameraRobustEstimator {
    
    /**
     * Intrinsic parameters of camera to be estimated.
     */
    protected PinholeCameraIntrinsicParameters mIntrinsic;
    
    /**
     * Indicates whether planar configuration is checked to determine whether
     * point correspondences are in such configuration and find a specific
     * solution for such case.
     */
    protected boolean mPlanarConfigurationAllowed = 
            EPnPPointCorrespondencePinholeCameraEstimator.
            DEFAULT_PLANAR_CONFIGURATION_ALLOWED;
    
    /**
     * Inicates whether the case where a dimension 2 nullspace is allowed.
     * When allowed, additional constraints are taken into account to ensure
     * equality of scales so that less point correspondences are required.
     * Enabling this parameter is usually ok.
     */
    protected boolean mNullspaceDimension2Allowed = 
            EPnPPointCorrespondencePinholeCameraEstimator.
            DEFAULT_NULLSPACE_DIMENSION2_ALLOWED;
    
    /**
     * Indicates whether the case where a dimension 3 nullspace is allowed.
     * When allowed, additional constraints are taken into account to ensure
     * equality of scales so that less point correspondences are required.
     * Enabling this parameter is usually ok although less precise than
     * when a nullspace of dimension 2 is used.
     */
    protected boolean mNullspaceDimension3Allowed = 
            EPnPPointCorrespondencePinholeCameraEstimator.
            DEFAULT_NULLSPACE_DIMENSION3_ALLOWED;    
    
    /**
     * Threshold to determine whether 3D matched points are in a planar 
     * configuration.
     * Points are considered to be laying in a plane when the smallest singular
     * value of their covariance matrix has a value much smaller than the 
     * largest one as many times as this value. 
     */
    protected double mPlanarThreshold = 
            EPnPPointCorrespondencePinholeCameraEstimator.
            DEFAULT_PLANAR_THRESHOLD;
    
    /**
     * Constructor.
     */
    public EPnPPointCorrespondencePinholeCameraRobustEstimator() {
        super();
    }
    
    /**
     * Constructor with listener.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     */
    public EPnPPointCorrespondencePinholeCameraRobustEstimator(
            PinholeCameraRobustEstimatorListener listener) {
        super(listener);
    }
    
    /**
     * Constructor with lists of points to be used to estimate a pinhole camera.
     * Points in the lists located at the same position are cnsidered to be 
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MIN_NUMBER_OF_POINT_CORRESPONDENCES (6 points).
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to
     * estimate a pinhole camera.
     * @throws IllegalArgumentException if provided lists of points don't have 
     * the same size or their size is smaller than required minimum size (6 
     * correspondences).
     */
    public EPnPPointCorrespondencePinholeCameraRobustEstimator(
            List<Point3D> points3D, List<Point2D> points2D) 
            throws IllegalArgumentException {
        super(points3D, points2D);
    }
        
    /**
     * Constructor with listener and lists of points to be used to estimate a
     * pinhole camera.
     * Points in the lists located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MIN_NUMBER_OF_POINT_CORRESPONDENCES (6 points).
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to
     * estimate a pinhole camera.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than required minimum size (6
     * correspondences).
     */
    public EPnPPointCorrespondencePinholeCameraRobustEstimator(
            PinholeCameraRobustEstimatorListener listener, 
            List<Point3D> points3D, List<Point2D> points2D)
            throws IllegalArgumentException {
        super(listener, points3D, points2D);
    }
    
    /**
     * Constructor with intrinsic parameters.
     * @param intrinsic intrinsic parameters of camera to be estimated.
     */
    public EPnPPointCorrespondencePinholeCameraRobustEstimator(
            PinholeCameraIntrinsicParameters intrinsic) {
        this();
        mIntrinsic = intrinsic;
    }
    
    /**
     * Constructor with intrinsic parameters and listener.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param intrinsic intrinsic parameters of camera to be estimated.
     */
    public EPnPPointCorrespondencePinholeCameraRobustEstimator(
            PinholeCameraRobustEstimatorListener listener,
            PinholeCameraIntrinsicParameters intrinsic) {
        this(listener);
        mIntrinsic = intrinsic;
    }
    
    /**
     * Constructor with lists of points to be used to estimate a pinhole camera
     * and intrinsic parameters.
     * Points in the lists located at the same position are cnsidered to be 
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MIN_NUMBER_OF_POINT_CORRESPONDENCES (6 points).
     * @param intrinsic intrinsic parameters of camera to be estimated.
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to
     * estimate a pinhole camera.
     * @throws IllegalArgumentException if provided lists of points don't have 
     * the same size or their size is smaller than required minimum size (6 
     * correspondences).
     */
    public EPnPPointCorrespondencePinholeCameraRobustEstimator(
            PinholeCameraIntrinsicParameters intrinsic, List<Point3D> points3D, 
            List<Point2D> points2D) throws IllegalArgumentException {
        this(points3D, points2D);
        mIntrinsic = intrinsic;
    }
        
    /**
     * Constructor with listener and lists of points to be used to estimate a
     * pinhole camera and intrinsic parameters.
     * Points in the lists located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MIN_NUMBER_OF_POINT_CORRESPONDENCES (6 points).
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param intrinsic intrinsic parameters of camera to be estimated.
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to
     * estimate a pinhole camera.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than required minimum size (6
     * correspondences).
     */
    public EPnPPointCorrespondencePinholeCameraRobustEstimator(
            PinholeCameraRobustEstimatorListener listener, 
            PinholeCameraIntrinsicParameters intrinsic, List<Point3D> points3D, 
            List<Point2D> points2D) throws IllegalArgumentException {
        this(listener, points3D, points2D);
        mIntrinsic = intrinsic;
    }
        
    /**
     * Indicates whether planar configuration is checked to determine whether
     * point correspondences are in such configuration and find a specific
     * solution for such case.
     * @return true to allow specific solutions for planar configurations,
     * false to always find a solution assuming the general case.
     */
    public boolean isPlanarConfigurationAllowed() {
        return mPlanarConfigurationAllowed;
    }
    
    /**
     * Specifies whether planar configuration is checked to determine whether
     * point correspondences are in such configuration and find a specific
     * solution for such case.
     * @param planarConfigurationAllowed true to allow specific solutions for
     * planar configurations, false to always find a solution assuming the 
     * general case.
     * @throws LockedException if estimator is locked.
     */
    public void setPlanarConfigurationAllowed(
            boolean planarConfigurationAllowed) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mPlanarConfigurationAllowed = planarConfigurationAllowed;
    }
    
    /**
     * Indicates whether the case where a dimension 2 nullspace is allowed.
     * When allowed, additional constraints are taken into account to ensure
     * equality of scales so that less point correspondences are required.
     * Enabling this parameter is usually ok.
     * @return true to allow 2 dimensional nullspace, false otherwise.
     */
    public boolean isNullspaceDimension2Allowed() {
        return mNullspaceDimension2Allowed;
    }
    
    /**
     * Specifies whether the case where a dimension 2 nullspace is allowed.
     * When allowed, additional constraints are taken into account to ensure
     * equality of scales so that less point correspondences are required.
     * Enabling this parameter is usually ok.
     * @param nullspaceDimension2Allowed true to allow 2 dimensional nullspace,
     * false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setNullspaceDimension2Allowed(
            boolean nullspaceDimension2Allowed) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mNullspaceDimension2Allowed = nullspaceDimension2Allowed;
    }
    
    /**
     * Indicates whether the case where a dimension 3 nullspace is allowed.
     * When allowed, additional constraints are taken into account to ensure 
     * equality of scales so that less point correspondences are required.
     * Enabling this parameter is usually ok although less precise than
     * when a nullspace of dimension 2 is used.
     * @return true to allow 3 dimensional nullspace, false otherwise.
     */
    public boolean isNullspaceDimension3Allowed() {
        return mNullspaceDimension3Allowed;
    }
    
    /**
     * Specifies whether the case where a dimension 3 nullspace is allowed.
     * When allowed, additional constraints are taken into account to ensure
     * equality of scales so that less point correspondences are required.
     * Enabling this parameter is usually ok although less precise than
     * when a nullspace of dimension 2 is used.
     * @param nullspaceDimension3Allowed true to allow 3 dimensional nullspace,
     * false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setNullspaceDimension3Allowed(
            boolean nullspaceDimension3Allowed) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mNullspaceDimension3Allowed = nullspaceDimension3Allowed;
    } 
    
    /**
     * Gets threshold to determine whether 3D matched points are in a planar
     * configuration.
     * Points are considered to be laying in a plane when the smallest singular
     * value of their covariance matrix has a value much smaller than the
     * largest one as many times as this value.
     * @return threshold to determine whether 3D matched points are in a planar
     * configuration.
     */
    public double getPlanarThreshold() {
        return mPlanarThreshold;
    }
    
    /**
     * Sets threshold to determine whether 3D matched points are in a planar
     * configuration.
     * Points are considered to be laying in a plane when the smallest singular
     * value of their covariance matrix has a value much smaller than the
     * largest one as many times as this value.
     * @param planarThreshold threshold to determine whether 3D matched points 
     * are in a planar configuration.
     * @throws IllegalArgumentException if provided threshold is negative.
     * @throws LockedException if estimator is locked.
     */
    public void setPlanarThreshold(double planarThreshold) 
            throws IllegalArgumentException, LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (planarThreshold < 0.0) {
            throw new IllegalArgumentException();
        }
        mPlanarThreshold = planarThreshold;
    }   
    
    /**
     * Gets intrinsic parameters of camera to be estimated.
     * @return intrinsic parameters of camera to be estimated.
     */
    public PinholeCameraIntrinsicParameters getIntrinsic() {
        return mIntrinsic;
    }
    
    /**
     * Sets intrinsic parameters of camera to be estimated.
     * @param intrinsic intrinsic parameters of camera to be estimated.
     * @throws LockedException if estimator is locked.
     */
    public void setIntrinsic(PinholeCameraIntrinsicParameters intrinsic)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mIntrinsic = intrinsic;
    }

    /**
     * Indicates if estimator is ready to start the pinhole camera estimation.
     * This is true when input data (i.e. lists of 2D/3D matched points) are
     * provided and a minimum of MIN_NUMBER_OF_POINT_CORRESPONDENCES are 
     * available.
     * @return true if estimator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return super.isReady() && mIntrinsic != null;
    }    
    
   /**
     * Returns value indicating if each picked subset point correspondences are
     * normalized to increase the accuracy of the estimation.
     * @return true if each picked subset point correspondences are normalized,
     * false otherwise.
     */
    @Override
    public boolean isNormalizeSubsetPointCorrespondences() {
        return false;
    }
    
    /**
     * Sets value indicating if each picked subset point correspondences are
     * normalized to increase the accuracy of the estimation.
     * @param normalizeSubsetPointCorrespondences true if each picked subset 
     * point correspondences are normalized, false otherwise.
     * @throws LockedException if robust estimator is locked because an 
     * estimation is already in progress.
     */
    @Override
    public void setNormalizeSubsetPointCorrespondences(
            boolean normalizeSubsetPointCorrespondences) 
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
    }    

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided robust estimator method.
     * @param method method of a robust estimator algorithm to estimate best
     * pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     */
    public static EPnPPointCorrespondencePinholeCameraRobustEstimator create(
            RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator();
            case MSAC:
                return new MSACEPnPPointCorrespondencePinholeCameraRobustEstimator();
            case PROSAC:
                return new PROSACEPnPPointCorrespondencePinholeCameraRobustEstimator();
            case PROMedS:
                return new PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator();
            case RANSAC:
            default:
                return new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator();
        }
    }
    
    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided 2D/3D points and robust estimator method.
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to 
     * estimate a pinhole camera.
     * @param method method of a robust estimator algorithm to estimate best 
     * pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than required minimum size (6
     * correspondences).
     */
    public static EPnPPointCorrespondencePinholeCameraRobustEstimator create(
            List<Point3D> points3D, List<Point2D> points2D, 
            RobustEstimatorMethod method) throws IllegalArgumentException {
        switch (method) {
            case LMedS:
                return new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        points3D, points2D);
            case MSAC:
                return new MSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        points3D, points2D);
            case PROSAC:
                return new PROSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        points3D, points2D);
            case PROMedS:
                return new PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        points3D, points2D);
            case RANSAC:
            default:
                return new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        points3D, points2D);
        }
    }
    
    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided listener.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param method method of a robust estimator algorithm to estimate best
     * pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     */
    public static EPnPPointCorrespondencePinholeCameraRobustEstimator create(
            PinholeCameraRobustEstimatorListener listener, 
            RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener);
            case MSAC:
                return new MSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener);
            case PROSAC:
                return new PROSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener);
            case PROMedS:
                return new PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener);
            case RANSAC:
            default:
                return new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener);
        }
    }
    
    /**
     * Creates a pinhole camera robust estimator based on point correspondences 
     * and using provided listener, 2D/3D points and robust estimator method.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to 
     * estimate a pinhole camera.
     * @param method method of a robust estimator algorithm to estimate best
     * pinhole camera.
     * @return an instance of a pinhole camera robust esitmator.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than required minimum size (6
     * correspondences).
     */
    public static EPnPPointCorrespondencePinholeCameraRobustEstimator create(
            PinholeCameraRobustEstimatorListener listener,
            List<Point3D> points3D, List<Point2D> points2D,
            RobustEstimatorMethod method) throws IllegalArgumentException {
        switch (method) {
            case LMedS:
                return new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener, points3D, points2D);
            case MSAC:
                return new MSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener, points3D, points2D);
            case PROSAC:
                return new PROSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener, points3D, points2D);
            case PROMedS:
                return new PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener, points3D, points2D);
            case RANSAC:
            default:
                return new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener, points3D, points2D);
        }
    }
    
    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided quality scores and robust estimator method.
     * @param qualityScores quality scores corresponding to each pair of matched 
     * points.
     * @param method method of a robust estimator algorithm to estimate best
     * pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided quality scores length is 
     * smaller than required minimum size (6 samples).
     */
    public static EPnPPointCorrespondencePinholeCameraRobustEstimator create(
            double[] qualityScores, RobustEstimatorMethod method) 
            throws IllegalArgumentException {
        switch (method) {
            case LMedS:
                return new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator();
            case MSAC:
                return new MSACEPnPPointCorrespondencePinholeCameraRobustEstimator();
            case PROSAC:
                return new PROSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        qualityScores);
            case PROMedS:
                return new PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        qualityScores);
            case RANSAC:
            default:
                return new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator();
        }
    }
    
    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided 2D/3D points, quality scores and robust estimator 
     * method.
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to 
     * estimate a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of matched 
     * points.
     * @param method method of a robust estimator algorithm to estimate best
     * pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points and quality
     * scores don't have the same size or their size is smaller than required
     * minimum size (6 correspondences).
     */
    public static EPnPPointCorrespondencePinholeCameraRobustEstimator create(
            List<Point3D> points3D, List<Point2D> points2D, 
            double[] qualityScores, RobustEstimatorMethod method)
            throws IllegalArgumentException {
        switch (method) {
            case LMedS:
                return new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        points3D, points2D);
            case MSAC:
                return new MSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        points3D, points2D);
            case PROSAC:
                return new PROSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        points3D, points2D, qualityScores);
            case PROMedS:
                return new PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        points3D, points2D, qualityScores);
            case RANSAC:
            default:
                return new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        points3D, points2D);
        }
    }
    
    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided listener and quality scores.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param qualityScores quality scores corresponding to each pair of matched
     * points.
     * @param method method of a robust estimator algorithm to estimate best
     * pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided quality scores don't have
     * the required minimum size (6 correspondences).
     */
    public static EPnPPointCorrespondencePinholeCameraRobustEstimator create(
            PinholeCameraRobustEstimatorListener listener, 
            double[] qualityScores, RobustEstimatorMethod method)
            throws IllegalArgumentException {
        switch (method) {
            case LMedS:
                return new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener);
            case MSAC:
                return new MSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener);
            case PROSAC:
                return new PROSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener, qualityScores);
            case PROMedS:
                return new PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener, qualityScores);
            case RANSAC:
            default:
                return new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener);
        }
    }
    
    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided listener, 2D/3D points, quality scores and robust
     * estimator method.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to 
     * estimate a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of matched 
     * points.
     * @param method method of a robust estimator algorithm to estimate best
     * pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points and quality 
     * scores don't have the same size or their size is smaller than required 
     * minimum size (6 correspondences).
     */
    public static EPnPPointCorrespondencePinholeCameraRobustEstimator create(
            PinholeCameraRobustEstimatorListener listener,
            List<Point3D> points3D, List<Point2D> points2D, 
            double[] qualityScores, RobustEstimatorMethod method)
            throws IllegalArgumentException {
        switch (method) {
            case LMedS:
                return new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener, points3D, points2D);
            case MSAC:
                return new MSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener, points3D, points2D);
            case PROSAC:
                return new PROSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener, points3D, points2D, qualityScores);
            case PROMedS:
                return new PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener, points3D, points2D, qualityScores);
            case RANSAC:
            default:
                return new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener, points3D, points2D);
        }
    }
    
    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided robust estimator method.
     * @param intrinsic intrinsic parameters of camera to be estimated.
     * @param method method of a robust estimator algorithm to estimate best
     * pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     */
    public static EPnPPointCorrespondencePinholeCameraRobustEstimator create(
            PinholeCameraIntrinsicParameters intrinsic, 
            RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        intrinsic);
            case MSAC:
                return new MSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        intrinsic);
            case PROSAC:
                return new PROSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        intrinsic);
            case PROMedS:
                return new PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        intrinsic);
            case RANSAC:
            default:
                return new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        intrinsic);
        }
    }
    
    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided 2D/3D points and robust estimator method.
     * @param intrinsic intrinsic parameters of camera to be estimated.
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to 
     * estimate a pinhole camera.
     * @param method method of a robust estimator algorithm to estimate best 
     * pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than required minimum size (6
     * correspondences).
     */
    public static EPnPPointCorrespondencePinholeCameraRobustEstimator create(
            PinholeCameraIntrinsicParameters intrinsic, 
            List<Point3D> points3D, List<Point2D> points2D, 
            RobustEstimatorMethod method) throws IllegalArgumentException {
        switch (method) {
            case LMedS:
                return new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        intrinsic, points3D, points2D);
            case MSAC:
                return new MSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        intrinsic, points3D, points2D);
            case PROSAC:
                return new PROSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        intrinsic, points3D, points2D);
            case PROMedS:
                return new PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        intrinsic, points3D, points2D);
            case RANSAC:
            default:
                return new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        intrinsic, points3D, points2D);
        }
    }
    
    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided listener.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param intrinsic intrinsic parameters of camera to be estimated.
     * @param method method of a robust estimator algorithm to estimate best
     * pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     */
    public static EPnPPointCorrespondencePinholeCameraRobustEstimator create(
            PinholeCameraRobustEstimatorListener listener, 
            PinholeCameraIntrinsicParameters intrinsic,
            RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener, intrinsic);
            case MSAC:
                return new MSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener, intrinsic);
            case PROSAC:
                return new PROSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener, intrinsic);
            case PROMedS:
                return new PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener, intrinsic);
            case RANSAC:
            default:
                return new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener, intrinsic);
        }
    }
    
    /**
     * Creates a pinhole camera robust estimator based on point correspondences 
     * and using provided listener, 2D/3D points and robust estimator method.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param intrinsic intrinsic parameters of camera to be estimated.
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to 
     * estimate a pinhole camera.
     * @param method method of a robust estimator algorithm to estimate best
     * pinhole camera.
     * @return an instance of a pinhole camera robust esitmator.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than required minimum size (6
     * correspondences).
     */
    public static EPnPPointCorrespondencePinholeCameraRobustEstimator create(
            PinholeCameraRobustEstimatorListener listener,
            PinholeCameraIntrinsicParameters intrinsic, 
            List<Point3D> points3D, List<Point2D> points2D,
            RobustEstimatorMethod method) throws IllegalArgumentException {
        switch (method) {
            case LMedS:
                return new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener, intrinsic, points3D, points2D);
            case MSAC:
                return new MSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener, intrinsic, points3D, points2D);
            case PROSAC:
                return new PROSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener, intrinsic, points3D, points2D);
            case PROMedS:
                return new PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener, intrinsic, points3D, points2D);
            case RANSAC:
            default:
                return new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener, intrinsic, points3D, points2D);
        }
    }
    
    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided quality scores and robust estimator method.
     * @param intrinsic intrinsic parameters of camera to be estimated.
     * @param qualityScores quality scores corresponding to each pair of matched 
     * points.
     * @param method method of a robust estimator algorithm to estimate best
     * pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided quality scores length is 
     * smaller than required minimum size (6 samples).
     */
    public static EPnPPointCorrespondencePinholeCameraRobustEstimator create(
            PinholeCameraIntrinsicParameters intrinsic,
            double[] qualityScores, RobustEstimatorMethod method) 
            throws IllegalArgumentException {
        switch (method) {
            case LMedS:
                return new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        intrinsic);
            case MSAC:
                return new MSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        intrinsic);
            case PROSAC:
                return new PROSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        intrinsic, qualityScores);
            case PROMedS:
                return new PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        intrinsic, qualityScores);
            case RANSAC:
            default:
                return new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        intrinsic);
        }
    }
    
    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided 2D/3D points, quality scores and robust estimator 
     * method.
     * @param intrinsic intrinsic parameters of camera to be estimated.
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to 
     * estimate a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of matched 
     * points.
     * @param method method of a robust estimator algorithm to estimate best
     * pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points and quality
     * scores don't have the same size or their size is smaller than required
     * minimum size (6 correspondences).
     */
    public static EPnPPointCorrespondencePinholeCameraRobustEstimator create(
            PinholeCameraIntrinsicParameters intrinsic,
            List<Point3D> points3D, List<Point2D> points2D, 
            double[] qualityScores, RobustEstimatorMethod method)
            throws IllegalArgumentException {
        switch (method) {
            case LMedS:
                return new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        intrinsic, points3D, points2D);
            case MSAC:
                return new MSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        intrinsic, points3D, points2D);
            case PROSAC:
                return new PROSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        intrinsic, points3D, points2D, qualityScores);
            case PROMedS:
                return new PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        intrinsic, points3D, points2D, qualityScores);
            case RANSAC:
            default:
                return new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        intrinsic, points3D, points2D);
        }
    }
    
    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided listener and quality scores.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param intrinsic intrinsic parameters of camera to be estimated.
     * @param qualityScores quality scores corresponding to each pair of matched
     * points.
     * @param method method of a robust estimator algorithm to estimate best
     * pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided quality scores don't have
     * the required minimum size (6 correspondences).
     */
    public static EPnPPointCorrespondencePinholeCameraRobustEstimator create(            
            PinholeCameraRobustEstimatorListener listener, 
            PinholeCameraIntrinsicParameters intrinsic,
            double[] qualityScores, RobustEstimatorMethod method)
            throws IllegalArgumentException {
        switch (method) {
            case LMedS:
                return new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener, intrinsic);
            case MSAC:
                return new MSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener, intrinsic);
            case PROSAC:
                return new PROSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener, intrinsic, qualityScores);
            case PROMedS:
                return new PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener, intrinsic, qualityScores);
            case RANSAC:
            default:
                return new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener, intrinsic);
        }
    }
    
    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided listener, 2D/3D points, quality scores and robust
     * estimator method.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param intrinsic intrinsic parameters of camera to be estimated.
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to 
     * estimate a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of matched 
     * points.
     * @param method method of a robust estimator algorithm to estimate best
     * pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points and quality 
     * scores don't have the same size or their size is smaller than required 
     * minimum size (6 correspondences).
     */
    public static EPnPPointCorrespondencePinholeCameraRobustEstimator create(
            PinholeCameraRobustEstimatorListener listener,
            PinholeCameraIntrinsicParameters intrinsic,
            List<Point3D> points3D, List<Point2D> points2D, 
            double[] qualityScores, RobustEstimatorMethod method)
            throws IllegalArgumentException {
        switch (method) {
            case LMedS:
                return new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener, intrinsic, points3D, points2D);
            case MSAC:
                return new MSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener, intrinsic, points3D, points2D);
            case PROSAC:
                return new PROSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener, intrinsic, points3D, points2D, qualityScores);
            case PROMedS:
                return new PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener, intrinsic, points3D, points2D, qualityScores);
            case RANSAC:
            default:
                return new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener, intrinsic, points3D, points2D);
        }
    }
        
    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using default robust estimator method.
     * @return an instance of a pinhole camera robust estimator.
     */
    public static EPnPPointCorrespondencePinholeCameraRobustEstimator create() {
        return create(DEFAULT_ROBUST_METHOD);
    }
        
    /**
     * Creates a pinhole camera robust estimator based on point correspondences 
     * and using provided 2D/3D points and default robust estimator method.
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to 
     * estimate a pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than required minimum size 
     * (6 correspondences).
     */
    public static EPnPPointCorrespondencePinholeCameraRobustEstimator create(
            List<Point3D> points3D, List<Point2D> points2D) 
            throws IllegalArgumentException {
        return create(points3D, points2D, DEFAULT_ROBUST_METHOD);
    }
    
    /**
     * Creates a pinhole camera robust estimator based on point 
     * correspondences and using provided listener and default robust estimator
     * method.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @return an instance of a pinhole camera robust estimator.
     */
    public static EPnPPointCorrespondencePinholeCameraRobustEstimator create(
            PinholeCameraRobustEstimatorListener listener) {
        return create(listener, DEFAULT_ROBUST_METHOD);
    }
    
    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided listener, 2D/3D points and default robust estimator 
     * method.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to 
     * estimate a pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than required minimum size 
     * (6 correspondences).
     */
    public static EPnPPointCorrespondencePinholeCameraRobustEstimator create(
            PinholeCameraRobustEstimatorListener listener,
            List<Point3D> points3D, List<Point2D> points2D) 
            throws IllegalArgumentException {
        return create(listener, points3D, points2D, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided quality scores and default robust estimator method.
     * @param qualityScores quality scores corresponding to each pair of matched 
     * points.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided quality scores length is
     * smalelr than required minimum size (6 samples).
     */    
    public static EPnPPointCorrespondencePinholeCameraRobustEstimator create(
            double[] qualityScores) throws IllegalArgumentException {
        return create(qualityScores, DEFAULT_ROBUST_METHOD);
    }
    
    /**
     * Creates a pinhole camera robust estimator based on point correspondences 
     * and using provided 2D/3D points, quality scores and default robust 
     * estimator method.
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to 
     * estimate a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of matched 
     * points.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points and quality 
     * scores don't have the same size or their size is smaller than required 
     * minimum size (6 correspondences).
     */
    public static EPnPPointCorrespondencePinholeCameraRobustEstimator create(
            List<Point3D> points3D, List<Point2D> points2D, 
            double[] qualityScores) throws IllegalArgumentException {
        return create(points3D, points2D, qualityScores, DEFAULT_ROBUST_METHOD);
    }
    
    /**
     * Creates a pinhole camera robust estimator based on point 
     * correspondences and using provided listener, quality scores and default
     * robust estimator method.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param qualityScores quality scores corresponding to each pair of matched 
     * points.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided quality scores don't have 
     * the required minimum size (6 correspondences).
     */
    public static EPnPPointCorrespondencePinholeCameraRobustEstimator create(
            PinholeCameraRobustEstimatorListener listener, 
            double[] qualityScores) throws IllegalArgumentException {
        return create(listener, qualityScores, DEFAULT_ROBUST_METHOD);
    }
    
    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided listener, 2D/3D points, quality scores and default 
     * robust estimator method.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to 
     * estimate a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of matched 
     * points.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points and quality 
     * scores don't have the same size or their size is smaller than required 
     * minimum size (6 correspondences).
     */
    public static EPnPPointCorrespondencePinholeCameraRobustEstimator create(
            PinholeCameraRobustEstimatorListener listener,
            List<Point3D> points3D, List<Point2D> points2D,
            double[] qualityScores) throws IllegalArgumentException {
        return create(listener, points3D, points2D, qualityScores, 
                DEFAULT_ROBUST_METHOD);
    }            
    
    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using default robust estimator method.
     * @param intrinsic intrinsic parameters of camera to be estimated.
     * @return an instance of a pinhole camera robust estimator.
     */
    public static EPnPPointCorrespondencePinholeCameraRobustEstimator create(
            PinholeCameraIntrinsicParameters intrinsic) {
        return create(intrinsic, DEFAULT_ROBUST_METHOD);
    }
        
    /**
     * Creates a pinhole camera robust estimator based on point correspondences 
     * and using provided 2D/3D points and default robust estimator method.
     * @param intrinsic intrinsic parameters of camera to be estimated.
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to 
     * estimate a pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than required minimum size 
     * (6 correspondences).
     */
    public static EPnPPointCorrespondencePinholeCameraRobustEstimator create(
            PinholeCameraIntrinsicParameters intrinsic,
            List<Point3D> points3D, List<Point2D> points2D) 
            throws IllegalArgumentException {
        return create(intrinsic, points3D, points2D, DEFAULT_ROBUST_METHOD);
    }
    
    /**
     * Creates a pinhole camera robust estimator based on point 
     * correspondences and using provided listener and default robust estimator
     * method.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param intrinsic intrinsic parameters of camera to be estimated.
     * @return an instance of a pinhole camera robust estimator.
     */
    public static EPnPPointCorrespondencePinholeCameraRobustEstimator create(
            PinholeCameraRobustEstimatorListener listener,
            PinholeCameraIntrinsicParameters intrinsic) {
        return create(listener, intrinsic, DEFAULT_ROBUST_METHOD);
    }
    
    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided listener, 2D/3D points and default robust estimator 
     * method.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param intrinsic intrinsic parameters of camera to be estimated.
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to 
     * estimate a pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than required minimum size 
     * (6 correspondences).
     */
    public static EPnPPointCorrespondencePinholeCameraRobustEstimator create(
            PinholeCameraRobustEstimatorListener listener,
            PinholeCameraIntrinsicParameters intrinsic,
            List<Point3D> points3D, List<Point2D> points2D) 
            throws IllegalArgumentException {
        return create(listener, intrinsic, points3D, points2D, 
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided quality scores and default robust estimator method.
     * @param intrinsic intrinsic parameters of camera to be estimated.
     * @param qualityScores quality scores corresponding to each pair of matched 
     * points.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided quality scores length is
     * smalelr than required minimum size (6 samples).
     */    
    public static EPnPPointCorrespondencePinholeCameraRobustEstimator create(
            PinholeCameraIntrinsicParameters intrinsic,
            double[] qualityScores) throws IllegalArgumentException {
        return create(intrinsic, qualityScores, DEFAULT_ROBUST_METHOD);
    }
    
    /**
     * Creates a pinhole camera robust estimator based on point correspondences 
     * and using provided 2D/3D points, quality scores and default robust 
     * estimator method.
     * @param intrinsic intrinsic parameters of camera to be estimated.
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to 
     * estimate a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of matched 
     * points.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points and quality 
     * scores don't have the same size or their size is smaller than required 
     * minimum size (6 correspondences).
     */
    public static EPnPPointCorrespondencePinholeCameraRobustEstimator create(
            PinholeCameraIntrinsicParameters intrinsic,
            List<Point3D> points3D, List<Point2D> points2D, 
            double[] qualityScores) throws IllegalArgumentException {
        return create(intrinsic, points3D, points2D, qualityScores, 
                DEFAULT_ROBUST_METHOD);
    }
    
    /**
     * Creates a pinhole camera robust estimator based on point 
     * correspondences and using provided listener, quality scores and default
     * robust estimator method.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param intrinsic intrinsic parameters of camera to be estimated.
     * @param qualityScores quality scores corresponding to each pair of matched 
     * points.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided quality scores don't have 
     * the required minimum size (6 correspondences).
     */
    public static EPnPPointCorrespondencePinholeCameraRobustEstimator create(
            PinholeCameraRobustEstimatorListener listener, 
            PinholeCameraIntrinsicParameters intrinsic,
            double[] qualityScores) throws IllegalArgumentException {
        return create(listener, intrinsic, qualityScores, 
                DEFAULT_ROBUST_METHOD);
    }
    
    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided listener, 2D/3D points, quality scores and default 
     * robust estimator method.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param intrinsic intrinsic parameters of camera to be estimated.
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to 
     * estimate a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of matched 
     * points.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points and quality 
     * scores don't have the same size or their size is smaller than required 
     * minimum size (6 correspondences).
     */
    public static EPnPPointCorrespondencePinholeCameraRobustEstimator create(
            PinholeCameraRobustEstimatorListener listener,
            PinholeCameraIntrinsicParameters intrinsic,
            List<Point3D> points3D, List<Point2D> points2D,
            double[] qualityScores) throws IllegalArgumentException {
        return create(listener, intrinsic, points3D, points2D, qualityScores, 
                DEFAULT_ROBUST_METHOD);
    }    
}
