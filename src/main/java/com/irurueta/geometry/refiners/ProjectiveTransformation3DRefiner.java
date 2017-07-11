/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.refiners.ProjectiveTransformation3DRefiner
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date May 3, 2017.
 */
package com.irurueta.geometry.refiners;

import com.irurueta.geometry.ProjectiveTransformation3D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.numerical.robust.InliersData;
import java.util.BitSet;
import java.util.List;

/**
 * Base class for ProjectiveTransformation3D refiner.
 * Implementations of this class refine a 3D projective transformation by taking
 * into account an initial estimaton, inlier point or plane matches and their
 * residuals.
 * This class can be used to find a solution that minimizes error of inliers in
 * LMSE terms.
 * Typically a refiner is used by a robust estimator, however it can also be
 * useful in some other situations.
 * @param <S1> type of matched samples in 1st set.
 * @param <S2> type of matched samples in 2nd set.
 */
public abstract class ProjectiveTransformation3DRefiner<S1, S2> extends 
        PairMatchesAndInliersDataRefiner<ProjectiveTransformation3D, S1, S2> {

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
    public ProjectiveTransformation3DRefiner() { }
    
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
    public ProjectiveTransformation3DRefiner(
            ProjectiveTransformation3D initialEstimation, 
            boolean keepCovariance, BitSet inliers, double[] residuals, 
            int numInliers, List<S1> samples1, List<S2> samples2, 
            double refinementStandardDeviation) {
        super(initialEstimation, keepCovariance, inliers, residuals, numInliers,
                samples1, samples2);
        mRefinementStandardDeviation = refinementStandardDeviation;
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
    public ProjectiveTransformation3DRefiner(
            ProjectiveTransformation3D initialEstimation, 
            boolean keepCovariance, InliersData inliersData, List<S1> samples1, 
            List<S2> samples2, double refinementStandardDeviation) {
        super(initialEstimation, keepCovariance, inliersData, samples1, 
                samples2);
        mRefinementStandardDeviation = refinementStandardDeviation;
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
    public double getRefinementStandardDeviation() {
        return mRefinementStandardDeviation;
    }
    
    /**
     * Sets standard deviation used for Levenberg-Marquardt fitting during 
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
     * Refines provided initial estimation.
     * @return refines estimation.
     * @throws NotReadyException if not enough input data has been provided.
     * @throws LockedException if estimator is locked because refinement is 
     * already in progress.
     * @throws RefinerException if refinement fails for some reason (e.g. unable
     * to converge to a result).
     */    
    @Override
    public ProjectiveTransformation3D refine() throws NotReadyException, LockedException, RefinerException {
        ProjectiveTransformation3D result = new ProjectiveTransformation3D();
        refine(result);
        return result;
    }    
}