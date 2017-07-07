/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.estimators.DLTPointCorrespondencePinholeCameraRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date February 18, 2017.
 */
package com.irurueta.geometry.estimators;

import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.Point3D;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import java.util.List;

/**
 * Base abstract class for algorithms to robustly find the best pihole camera
 * for collections of matched 3D/2D points using DLT (Direct Linear Transform) 
 * algorithm.
 * Implementations of this class should be able to detect and discard outliers
 * in order to find the best solution.
 */
public abstract class DLTPointCorrespondencePinholeCameraRobustEstimator
        extends PointCorrespondencePinholeCameraRobustEstimator {
    
    /**
     * Constructor.
     */
    public DLTPointCorrespondencePinholeCameraRobustEstimator() {
        super();
    }
    
    /**
     * Constructor with lists of points to be used to estimate a pinhole camera.
     * Points in the lists located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MIN_NUMBER_OF_POINT_CORRESPONDENCES (6 points).
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to 
     * estimate a pinhole camera.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than required minimum size (6
     * correspondences).
     */
    public DLTPointCorrespondencePinholeCameraRobustEstimator(
            List<Point3D> points3D, List<Point2D> points2D)
            throws IllegalArgumentException {
        super(points3D, points2D);
    }
    
    /**
     * Constructor with listener.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     */
    public DLTPointCorrespondencePinholeCameraRobustEstimator(
            PinholeCameraRobustEstimatorListener listener) {
        super(listener);
    }
    
    /**
     * Constructor with listener and lists of points to be used to estimate a
     * pinhole camera.
     * Points in the lists located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MIN_NUMBER_OF_POINT_CORRESPONDENCES (6 points).
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param points3D lists of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to
     * estimate a pinhole camera.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than required minimum size
     * (6 correspondences).
     */
    public DLTPointCorrespondencePinholeCameraRobustEstimator(
            PinholeCameraRobustEstimatorListener listener,
            List<Point3D> points3D, List<Point2D> points2D)
            throws IllegalArgumentException {
        super(listener, points3D, points2D);
    }
    
    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided robust estimator method.
     * @param method method of a robust estimator algorithm to estimate best
     * pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     */    
    public static DLTPointCorrespondencePinholeCameraRobustEstimator create(
            RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSDLTPointCorrespondencePinholeCameraRobustEstimator();
            case MSAC:
                return new MSACDLTPointCorrespondencePinholeCameraRobustEstimator();
            case PROSAC:
                return new PROSACDLTPointCorrespondencePinholeCameraRobustEstimator();
            case PROMedS:
                return new PROMedSDLTPointCorrespondencePinholeCameraRobustEstimator();
            case RANSAC:
            default:
                return new RANSACDLTPointCorrespondencePinholeCameraRobustEstimator();
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
     * the same size or their size is smaller than required minimum size 
     * (6 correspondences).
     */
    public static DLTPointCorrespondencePinholeCameraRobustEstimator create(
            List<Point3D> points3D, List<Point2D> points2D, 
            RobustEstimatorMethod method) throws IllegalArgumentException {
        switch (method) {
            case LMedS:
                return new LMedSDLTPointCorrespondencePinholeCameraRobustEstimator(
                        points3D, points2D);
            case MSAC:
                return new MSACDLTPointCorrespondencePinholeCameraRobustEstimator(
                        points3D, points2D);
            case PROSAC:
                return new PROSACDLTPointCorrespondencePinholeCameraRobustEstimator(
                        points3D, points2D);
            case PROMedS:
                return new PROMedSDLTPointCorrespondencePinholeCameraRobustEstimator(
                        points3D, points2D);
            case RANSAC:
            default:
                return new RANSACDLTPointCorrespondencePinholeCameraRobustEstimator(
                        points3D, points2D);
        }
    }
    
    /**
     * Creates a pinhole camera robust estimator based on point 
     * correspondences and using provided listener.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param method method of a robust estimator algorithm to estimate best
     * pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     */
    public static DLTPointCorrespondencePinholeCameraRobustEstimator create(
            PinholeCameraRobustEstimatorListener listener, 
            RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSDLTPointCorrespondencePinholeCameraRobustEstimator(
                        listener);
            case MSAC:
                return new MSACDLTPointCorrespondencePinholeCameraRobustEstimator(
                        listener);
            case PROSAC:
                return new PROSACDLTPointCorrespondencePinholeCameraRobustEstimator(
                        listener);
            case PROMedS:
                return new PROMedSDLTPointCorrespondencePinholeCameraRobustEstimator(
                        listener);
            case RANSAC:
            default:
                return new RANSACDLTPointCorrespondencePinholeCameraRobustEstimator(
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
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than required minimum size 
     * (6 correspondences).
     */
    public static DLTPointCorrespondencePinholeCameraRobustEstimator create(
            PinholeCameraRobustEstimatorListener listener,
            List<Point3D> points3D, List<Point2D> points2D,
            RobustEstimatorMethod method) throws IllegalArgumentException {
        switch (method) {
            case LMedS:
                return new LMedSDLTPointCorrespondencePinholeCameraRobustEstimator(
                        listener, points3D, points2D);
            case MSAC:
                return new MSACDLTPointCorrespondencePinholeCameraRobustEstimator(
                        listener, points3D, points2D);
            case PROSAC:
                return new PROSACDLTPointCorrespondencePinholeCameraRobustEstimator(
                        listener, points3D, points2D);
            case PROMedS:
                return new PROMedSDLTPointCorrespondencePinholeCameraRobustEstimator(
                        listener, points3D, points2D);
            case RANSAC:
            default:
                return new RANSACDLTPointCorrespondencePinholeCameraRobustEstimator(
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
    public static DLTPointCorrespondencePinholeCameraRobustEstimator create(
            double[] qualityScores, RobustEstimatorMethod method)
            throws IllegalArgumentException {
        switch (method) {
            case LMedS:
                return new LMedSDLTPointCorrespondencePinholeCameraRobustEstimator();
            case MSAC:
                return new MSACDLTPointCorrespondencePinholeCameraRobustEstimator();
            case PROSAC:
                return new PROSACDLTPointCorrespondencePinholeCameraRobustEstimator(
                        qualityScores);
            case PROMedS:
                return new PROMedSDLTPointCorrespondencePinholeCameraRobustEstimator(
                        qualityScores);
            case RANSAC:
            default:
                return new RANSACDLTPointCorrespondencePinholeCameraRobustEstimator();
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
    public static DLTPointCorrespondencePinholeCameraRobustEstimator create(
            List<Point3D> points3D, List<Point2D> points2D, 
            double[] qualityScores, RobustEstimatorMethod method) 
            throws IllegalArgumentException {
        switch (method) {
            case LMedS:
                return new LMedSDLTPointCorrespondencePinholeCameraRobustEstimator(
                        points3D, points2D);
            case MSAC:
                return new MSACDLTPointCorrespondencePinholeCameraRobustEstimator(
                        points3D, points2D);
            case PROSAC:
                return new PROSACDLTPointCorrespondencePinholeCameraRobustEstimator(
                        points3D, points2D, qualityScores);
            case PROMedS:
                return new PROMedSDLTPointCorrespondencePinholeCameraRobustEstimator(
                        points3D, points2D, qualityScores);
            case RANSAC:
            default:
                return new RANSACDLTPointCorrespondencePinholeCameraRobustEstimator(
                        points3D, points2D);
        }
    }
    
    /**
     * Creates a pinhole camera robust estimator based on point 
     * correspondences and using provided listener and quality scores.
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
    public static DLTPointCorrespondencePinholeCameraRobustEstimator create(
            PinholeCameraRobustEstimatorListener listener, 
            double[] qualityScores, RobustEstimatorMethod method) 
            throws IllegalArgumentException {
        switch (method) {
            case LMedS:
                return new LMedSDLTPointCorrespondencePinholeCameraRobustEstimator(
                        listener);
            case MSAC:
                return new MSACDLTPointCorrespondencePinholeCameraRobustEstimator(
                        listener);
            case PROSAC:
                return new PROSACDLTPointCorrespondencePinholeCameraRobustEstimator(
                        listener, qualityScores);
            case PROMedS:
                return new PROMedSDLTPointCorrespondencePinholeCameraRobustEstimator(
                        listener, qualityScores);
            case RANSAC:
            default:
                return new RANSACDLTPointCorrespondencePinholeCameraRobustEstimator(
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
    public static DLTPointCorrespondencePinholeCameraRobustEstimator create(
            PinholeCameraRobustEstimatorListener listener,
            List<Point3D> points3D, List<Point2D> points2D,
            double[] qualityScores, RobustEstimatorMethod method) 
            throws IllegalArgumentException {
        switch (method) {
            case LMedS:
                return new LMedSDLTPointCorrespondencePinholeCameraRobustEstimator(
                        listener, points3D, points2D);
            case MSAC:
                return new MSACDLTPointCorrespondencePinholeCameraRobustEstimator(
                        listener, points3D, points2D);
            case PROSAC:
                return new PROSACDLTPointCorrespondencePinholeCameraRobustEstimator(
                        listener, points3D, points2D, qualityScores);
            case PROMedS:
                return new PROMedSDLTPointCorrespondencePinholeCameraRobustEstimator(
                        listener, points3D, points2D, qualityScores);
            case RANSAC:
            default:
                return new RANSACDLTPointCorrespondencePinholeCameraRobustEstimator(
                        listener, points3D, points2D);
        }
    }  
    
    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using default robust estimator method.
     * @return an instance of a pinhole camera robust estimator.
     */
    public static DLTPointCorrespondencePinholeCameraRobustEstimator create() {
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
    public static DLTPointCorrespondencePinholeCameraRobustEstimator create(
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
    public static DLTPointCorrespondencePinholeCameraRobustEstimator create(
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
    public static DLTPointCorrespondencePinholeCameraRobustEstimator create(
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
    public static DLTPointCorrespondencePinholeCameraRobustEstimator create(
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
    public static DLTPointCorrespondencePinholeCameraRobustEstimator create(
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
    public static DLTPointCorrespondencePinholeCameraRobustEstimator create(
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
    public static DLTPointCorrespondencePinholeCameraRobustEstimator create(
            PinholeCameraRobustEstimatorListener listener,
            List<Point3D> points3D, List<Point2D> points2D,
            double[] qualityScores) throws IllegalArgumentException {
        return create(listener, points3D, points2D, qualityScores, 
                DEFAULT_ROBUST_METHOD);
    }        
}
