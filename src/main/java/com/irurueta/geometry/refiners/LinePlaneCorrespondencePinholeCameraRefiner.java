/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.refiners.LinePlaneCorrespondencePinholeCameraRefiner
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 2, 2017.
 */
package com.irurueta.geometry.refiners;

import com.irurueta.geometry.Line2D;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.Plane;
import com.irurueta.numerical.robust.InliersData;
import java.util.BitSet;
import java.util.List;

/**
 * Base class for a pinhole camera refiner using line/plane correspondences.
 * Implementations of this class refine a pinhole camera by taking into account
 * an initial estimation, inlier line/plane matches and their residuals.
 * This class can be used to find a solution that minimizes error of inliers in
 * LMSE terms.
 * Typically a refiner is used by a robust estimator, however it can also be
 * useful in some other situations.
 */
public abstract class LinePlaneCorrespondencePinholeCameraRefiner extends 
        PinholeCameraRefiner<Plane, Line2D>{

    /**
     * Plane to be reused when computing residuals.
     */
    private Plane mResidualTestPlane = new Plane();
    
    /**
     * Constructor.
     */
    public LinePlaneCorrespondencePinholeCameraRefiner() { }
    
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
    public LinePlaneCorrespondencePinholeCameraRefiner(
            PinholeCamera initialEstimation, boolean keepCovariance,
            BitSet inliers, double[] residuals, int numInliers,
            List<Plane> samples1, List<Line2D> samples2,
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
    public LinePlaneCorrespondencePinholeCameraRefiner(
            PinholeCamera initialEstimation, boolean keepCovariance,
            InliersData inliersData, List<Plane> samples1, 
            List<Line2D> samples2, double refinementStandardDeviation) {
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
        return backprojectionResidual(pinholeCamera) + 
                suggestionResidual(params, weight);
    }
    
    /**
     * Computes total line backprojection residual for provided camera.
     * This method computes the sum of the squared residuals for all inlier
     * backprojected lines.
     * @param pinholeCamera camera to compute residual for.
     * @return total backprojection residual.
     */
    private double backprojectionResidual(PinholeCamera pinholeCamera) {
        pinholeCamera.normalize();
        
        //backprojection inlier lines into test plane
        int nSamples = mInliers.length();
        Line2D line;
        Plane plane;
        double residual = 0.0;
        for (int i = 0; i < nSamples; i++) {
            if (mInliers.get(i)) {
                line = mSamples2.get(i);
                plane = mSamples1.get(i);
                
                line.normalize();
                plane.normalize();
                
                residual += Math.pow(singleBackprojectionResidual(
                        pinholeCamera, line, plane), 2.0);
            }
        }
        
        return residual;
    }  
    
    /**
     * Computes total residual to be used during Levenberg/Marquard covariance
     * estimation.
     * @param pinholeCamera camera to estimate covariance for.
     * @param line 2D line to be backprojected with provided pinhole camera.
     * @param plane plane to be compared with backprojected line.
     * @param params camera parameters. In the following order:
     * skewness, horizontal focal length, vertical focal length, 
     * horizontal principal point, vertical principal point, quaternion A,
     * quaternion B, quaternion C, quaternion D, center x, center y, center z.
     * @param weight weight for suggestion residual.
     * @return total residual.
     */
    protected double residualLevenbergMarquardt(PinholeCamera pinholeCamera, 
            Line2D line, Plane plane, double[] params, double weight) {
        double residual = singleBackprojectionResidual(pinholeCamera, line, 
                plane);
        if (hasSuggestions()) {
            residual += suggestionResidual(params, weight);
        }
        return residual;
    }
    
    /**
     * Backprojection residual/error for a single line using provided camera.
     * @param pinholeCamera camera ot be checked.
     * @param line line to be backprojected.
     * @param plane plane to check against.
     * @return dot product distance between backprojected line and plane.
     */
    protected double singleBackprojectionResidual(PinholeCamera pinholeCamera, 
            Line2D line, Plane plane) {
        //backproject line into test plane
        pinholeCamera.backProject(line, mResidualTestPlane);
        mResidualTestPlane.normalize();
        
        double dotProduct = Math.abs(plane.getA() * mResidualTestPlane.getA() +
                plane.getB() * mResidualTestPlane.getB() +
                plane.getC() * mResidualTestPlane.getC() + 
                plane.getD() * mResidualTestPlane.getD());
        return 1.0 - dotProduct;
    }        
}