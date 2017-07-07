/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.refiners.Point2DRefiner
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April  8, 2017.
 */
package com.irurueta.geometry.refiners;

import com.irurueta.geometry.Line2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.numerical.robust.InliersData;
import java.util.BitSet;
import java.util.List;

/**
 * Refines a 2D point by taking into account an initial estimation, inlier
 * samples and their residuals.
 * This class can be used to find a solution that minimizes error of inliers in
 * LMSE terms.
 * Typically a refiner is used by a robust estimator, however it can also be 
 * useful in some other situations.
 * @param <T> an implementation of a 2D point.
 */
public abstract class Point2DRefiner<T extends Point2D> extends 
        SamplesAndInliersDataRefiner<T, Line2D> {
    
    /**
     * Standard deviation used for Levenberg-Marquardt fitting during 
     * refinement.
     * Returned value gives an indication of how much variance each residual 
     * has.
     * Typically this value is related to the threshold used on each robust
     * estimation, since residuals of found inliers are within the range of
     * such threshold.
     */
    protected double mRefinementStandardDeviation;
    
    /**
     * Constructor.
     */
    public Point2DRefiner() { }
    
    /**
     * Constructor.
     * @param initialEstimation initial estimation to be set.
     * @param keepCovariance true if covariance of estimation must be kept after
     * refinement, false otherwise.
     * @param inliers set indicating which of the provided matches are inliers.
     * @param residuals residuals for matched samples.
     * @param numInliers number of inliers on initial estimation.
     * @param samples collection of samples.
     * @param refinementStandardDeviation standard deviation used for
     * Levenberg-Marquardt fitting.
     */
    public Point2DRefiner(T initialEstimation,
            boolean keepCovariance, BitSet inliers, double[] residuals,
            int numInliers, List<Line2D> samples,
            double refinementStandardDeviation) {
        super(initialEstimation, keepCovariance, inliers, residuals, numInliers,
                samples);
        mRefinementStandardDeviation = refinementStandardDeviation;
    }
    
    /**
     * Constructor.
     * @param initialEstimation initial estimation to be set.
     * @param keepCovariance true if covariance of estimation must be kept after
     * refinement, false otherwise.
     * @param inliersData inlier data, typically obtained from a robust estimator.
     * @param samples collection of samples.
     * @param refinementStandardDeviation standard deviation used for
     * Levenberg-Marquardt fitting.
     */
    public Point2DRefiner(T initialEstimation, boolean keepCovariance,
            InliersData inliersData, List<Line2D> samples,
            double refinementStandardDeviation) {
        super(initialEstimation, keepCovariance, inliersData, samples);
        mRefinementStandardDeviation = refinementStandardDeviation;
    }
    
    /**
     * Gets standard deviation used for Levenberg-Marquardt fitting during
     * refinement.
     * Returned value gives an indication of how much variance each residual 
     * has.
     * Typically this value is related to the threshold used on each robust
     * estimation, since residuals of found inliers are within the range of such
     * threshold.
     * @return standard deviation used for refinement.
     */
    public double getRefinementStandardDeviation() {
        return mRefinementStandardDeviation;
    }
    
    /**
     * Sets standard deviation used for Levenberg-Marquardt fittin during
     * refinement.
     * Returned value gives an indication of how much variance each residual
     * has.
     * Typically this value is related to the threshold used on each robust
     * estimation, since residuals of found inliers are within the range of such
     * threshold.
     * @param refinementStandardDeviation standard deviation used for
     * refinement.
     * @throws LockedException if estimator is locked.
     */
    public void setRefinementStandardDeviation(
            double refinementStandardDeviation) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mRefinementStandardDeviation = refinementStandardDeviation;
    }
    
    /**
     * Computes the residual between a point and a line as their distance.
     * @param point a point
     * @param line a line.
     * @return residual (distance between provided point and line).
     */
    protected double residual(Point2D point, Line2D line) {
        point.normalize();
        line.normalize();
        return Math.abs(line.signedDistance(point));
    }
    
    /**
     * Computes total residual among all provided inlier samples.
     * @param point a point.
     * @return total residual.
     */
    protected double totalResidual(Point2D point) {
        double result = 0.0;
        
        int nSamples = mInliers.length();
        Line2D line;
        for (int i = 0; i < nSamples; i++) {
            if (mInliers.get(i)) {
                //sample is inlier
                line = mSamples.get(i);
                result += residual(point, line);
            }
        }
        
        return result;
    }
}
