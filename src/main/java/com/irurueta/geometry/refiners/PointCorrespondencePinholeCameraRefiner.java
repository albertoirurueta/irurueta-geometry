/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.refiners.PointCorrespondencePinholeCameraRefiner
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date March 29, 2017.
 */
package com.irurueta.geometry.refiners;

import com.irurueta.geometry.CoordinatesType;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.Point3D;
import com.irurueta.numerical.robust.InliersData;
import java.util.BitSet;
import java.util.List;

/**
 * Base class for a pinhole camera refiner using point correspondences.
 * Implementations of this class refine a pinhole camera by taking into account
 * an initial estimation, inlier point matches and their residuals.
 * This class can be used to find a solution that minimizes error of inliers in
 * LMSE terms.
 * Typically a refiner is used by a robust estimator, however it can also be 
 * useful in some other situations.
 */
public abstract class PointCorrespondencePinholeCameraRefiner extends 
        PinholeCameraRefiner<Point3D, Point2D> {
    
    /**
     * Point to be reused when computing residuals.
     */
    protected Point2D mResidualTestPoint = Point2D.create(
            CoordinatesType.HOMOGENEOUS_COORDINATES);     
    
    /**
     * Constructor.
     */
    public PointCorrespondencePinholeCameraRefiner() { }
    
    /**
     * Constructor.
     * @param initialEstimation initial estimation to be set.
     * @param keepCovariance true if covariance of estimation must be kept after
     * refinement, false otherwise.
     * @param inliers set indicating which of the provided matches are inliers.
     * @param residuals residuals for matched samples.
     * @param numInliers number of inliers on initial estimation.
     * @param samples1 1st set of paired samples.
     * @param samples2 2nd set of paired samples.
     * @param refinementStandardDeviation standard deviation used for 
     * Levenberg-Marquardt fitting.
     */
    public PointCorrespondencePinholeCameraRefiner(
            PinholeCamera initialEstimation, boolean keepCovariance, 
            BitSet inliers, double[] residuals, int numInliers, 
            List<Point3D> samples1, List<Point2D> samples2, 
            double refinementStandardDeviation) {
        super(initialEstimation, keepCovariance, inliers, residuals, numInliers, 
                samples1, samples2, refinementStandardDeviation);
    }
    
    /**
     * Constructor.
     * @param initialEstimation initial estimation to be set.
     * @param keepCovariance true if covariance of estimation must be kept after
     * refinement, false otherwise.
     * @param inliersData inlier data, typically obtained from a robust 
     * estimator.
     * @param samples1 1st set of paired samples.
     * @param samples2 2nd set of paired samples.
     * @param refinementStandardDeviation standard deviation used for 
     * Levenberg-Marquardt fitting.
     */
    public PointCorrespondencePinholeCameraRefiner(
            PinholeCamera initialEstimation, boolean keepCovariance,
            InliersData inliersData, List<Point3D> samples1, 
            List<Point2D> samples2, double refinementStandardDeviation) {
        super(initialEstimation, keepCovariance, inliersData, samples1, 
                samples2, refinementStandardDeviation);
    }
    
    /**
     * Total residual to be used during Powell refinement.
     * Powell refinement uses Powell algorithm to minimize a cost function
     * consisting on the sum of squared projection residuals plus the
     * suggestion residual for any suggested terms.
     * @param pinholeCamera camera to be checked.
     * @param params camera parameters. In the following order: 
     * skewness, horizontal focal length, vertical focal length, 
     * horizontal principal point, vertical principal point, quaternion A,
     * quaternion B, quaternion C, quaternion D, center x, center y, center z.
     * @param weight weight for suggestion residual.
     * @return total residual during Powell refinement.
     */
    protected double residualPowell(PinholeCamera pinholeCamera, 
            double[] params, double weight) {
        return projectionResidual(pinholeCamera) + 
                suggestionResidual(params, weight);
    }
    
    /**
     * Computes total point projection residual for provided camera.
     * This method computes the sum of the squared residuals for all inlier 
     * projected points.
     * @param pinholeCamera camera to compute residual for.
     * @return total projection residual.
     */
    private double projectionResidual(PinholeCamera pinholeCamera) {
        pinholeCamera.normalize();
        
        //project inlier 3D points into test point
        int nSamples = mInliers.length();
        Point3D point3D;
        Point2D point2D, projectedPoint2D = Point2D.create();
        double residual = 0.0;
        for (int i = 0; i < nSamples; i++) {
            if (mInliers.get(i)) {
                point3D = mSamples1.get(i);
                point2D = mSamples2.get(i);
                
                point3D.normalize();
                point2D.normalize();
                
                pinholeCamera.project(point3D, projectedPoint2D);
                
                projectedPoint2D.normalize();
                
                residual += Math.pow(projectedPoint2D.distanceTo(point2D), 2.0);
            }                        
        }
        
        return residual;        
    }   

    /**
     * Computes total residual to be used during Levenberg/Marquardt covariance
     * estimation.
     * @param pinholeCamera camera to estimate covariance for.
     * @param point3D 3D point to be projected with provided pinhole camera.
     * @param point2D 2D point to be compared with projected point.
     * @param params camera parameters. In the following order: 
     * skewness, horizontal focal length, vertical focal length, 
     * horizontal principal point, vertical principal point, quaternion A,
     * quaternion B, quaternion C, quaternion D, center x, center y, center z.
     * @param weight weight for suggestion residual.
     * @return total residual.
     */
    protected double residualLevenbergMarquardt(PinholeCamera pinholeCamera, 
            Point3D point3D, Point2D point2D, double[] params, double weight) {
        double residual = singleProjectionResidual(pinholeCamera, point3D, 
                point2D);
        if (hasSuggestions()) {
            residual += suggestionResidual(params, weight);
        }
        return residual;                
    }     
    
    /**
     * Projection residual/error for a single point using provided camera.
     * @param pinholeCamera camera to be checked.
     * @param point3D point to be projected.
     * @param point2D point to check against.
     * @return distance between projected point and 2D point.
     */
    private double singleProjectionResidual(PinholeCamera pinholeCamera, 
            Point3D point3D, Point2D point2D) {
        //project point3D into test point
        pinholeCamera.project(point3D, mResidualTestPoint);
                
        //compare test point and 2D point
        return mResidualTestPoint.distanceTo(point2D);                
    }    
}
