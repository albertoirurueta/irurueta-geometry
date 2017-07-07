/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.epipolar.refiners.HomogeneousRightEpipoleRefiner
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 26, 2017.
 */
package com.irurueta.geometry.epipolar.refiners;

import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.CoordinatesType;
import com.irurueta.geometry.HomogeneousPoint2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.Transformation2D;
import com.irurueta.geometry.epipolar.FundamentalMatrix;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.geometry.refiners.RefinerException;
import com.irurueta.numerical.GradientEstimator;
import com.irurueta.numerical.MultiDimensionFunctionEvaluatorListener;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiDimensionFitter;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiDimensionFunctionEvaluator;
import com.irurueta.numerical.robust.InliersData;
import java.util.BitSet;
import java.util.List;

/**
 * Refines the epipole of a fundamental matrix formed by an initial epipole 
 * estimation and an estimated homography.
 * Any fundamental matrix can be expressed as F = [e']x*H, where
 * e' is the epipole on the right view, []x is the skew matrix, and H is a non 
 * degenerate homography.
 * This class refines an initial epipole so that residuals from provided point
 * correspondences generating fundamental matrix F are reduced.
 * This class is especially useful in cases where geometry of the scene is 
 * degenerate (e.g. planar scene) and provided point correspondences would
 * generate an inaccurate fundamental matrix.
 * This implementation uses Levenberg-Marquardt algorithm for a fast cost 
 * optimization and uses homogeneous points for epipole refinement.
 */
public class HomogeneousRightEpipoleRefiner extends RightEpipoleRefiner {
    
    /**
     * Maximum allowed number of refinement iterations.
     */
    public static int MAX_ITERS = 10;   
    
    /**
     * Constructor.
     */
    public HomogeneousRightEpipoleRefiner() { }
    
    /**
     * Constructor.
     * @param initialEpipoleEstimation initial right epipole estimation to be 
     * set and refined.
     * @param keepCovariance true if covariance of estimation must be kept after
     * refinement, false otherwise.
     * @param inliers set indicating which of the provided matches are inliers.
     * @param residuals residuals for matched samples.
     * @param numInliers number of inliers on initial estimation.
     * @param samples1 1st set of paired samples.
     * @param samples2 2nd set of paired samples.
     * @param refinementStandardDeviation standard deviation used for 
     * Levenberg-Marquardt fitting.
     * @param homography homography relating samples in two views, which is
     * used to generate a fundamental matrix and its corresponding epipolar 
     * geometry.
     */
    public HomogeneousRightEpipoleRefiner(Point2D initialEpipoleEstimation, 
            boolean keepCovariance, BitSet inliers, double[] residuals, 
            int numInliers, List<Point2D> samples1, List<Point2D> samples2,
            double refinementStandardDeviation, Transformation2D homography) {
        super(initialEpipoleEstimation, keepCovariance, inliers, residuals, 
                numInliers, samples1, samples2, refinementStandardDeviation, 
                homography);
    }
    
    /**
     * Constructor.
     * @param initialEpipoleEstimation initial right epipole estimation to be
     * set and refined.
     * @param keepCovariance true if covariance of estimation must be kept after
     * refinement, false otherwise.
     * @param inliersData inlier data, typically obtained from a robust 
     * estimator.
     * @param samples1 1st set of paired samples.
     * @param samples2 2nd set of paired samples.
     * @param refinementStandardDeviation standard deviation used for 
     * Levenberg-Marquardt fitting.
     * @param homography homography relating samples in two views, which is used
     * to generate a fundamental matrix and its corresponding epipolar geometry.
     */
    public HomogeneousRightEpipoleRefiner(Point2D initialEpipoleEstimation,
            boolean keepCovariance, InliersData inliersData,
            List<Point2D> samples1, List<Point2D> samples2, 
            double refinementStandardDeviation, Transformation2D homography) {
        super(initialEpipoleEstimation, keepCovariance, inliersData, samples1, 
                samples2, refinementStandardDeviation, homography);
    }
    
    /**
     * Refines provided initial right epipole estimation.
     * This method always sets a value into provided result instance regardless
     * of the fact that error has actually improved in LMSE terms or not.
     * @param result instance where refined estimation will be stored.
     * @return true if result improves (decreases) in LMSE terms respect to 
     * initial estimation, false if no improvement has been achieved.
     * @throws NotReadyException if not enough input data has been provided.
     * @throws LockedException if estimator is locked because refinement is 
     * already in progress.
     * @throws RefinerException if refinement fails for some reason (e.g. unable
     * to converge to a result).
     */
    @Override
    public boolean refine(Point2D result) throws NotReadyException, 
            LockedException, RefinerException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }
        
        mLocked = true;
        
        if (mListener != null) {
            mListener.onRefineStart(this, mInitialEstimation);
        }
        
        try {                        
            HomogeneousPoint2D epipole = new HomogeneousPoint2D(
                    mInitialEstimation);
            boolean errorDecreased = false, iterErrorDecreased;
            int numIter = 0;
            do {            
                FundamentalMatrix fundamentalMatrix = new FundamentalMatrix();
                computeFundamentalMatrix(mHomography, epipole, 
                        fundamentalMatrix);
                double initialTotalResidual = totalResidual(fundamentalMatrix);
                final double[] initParams = epipole.asArray();
        
                //output values to be fitted/optimized will contain residuals
                double[] y = new double[mNumInliers];
                //input values will contain 2 points to compute residuals
                final int nDims = 
                        2*Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH;
                Matrix x = new Matrix(mNumInliers, nDims);
                int nSamples = mInliers.length();
                int pos = 0;
                Point2D leftPoint, rightPoint;
                for (int i = 0; i < nSamples; i++) {
                    if (mInliers.get(i)) {
                        //sample is inlier
                        leftPoint = mSamples1.get(i);
                        rightPoint = mSamples2.get(i);
                        leftPoint.normalize();
                        rightPoint.normalize();
                        x.setElementAt(pos, 0, leftPoint.getHomX());
                        x.setElementAt(pos, 1, leftPoint.getHomY());
                        x.setElementAt(pos, 2, leftPoint.getHomW());
                        x.setElementAt(pos, 3, rightPoint.getHomX());
                        x.setElementAt(pos, 4, rightPoint.getHomY());
                        x.setElementAt(pos, 5, rightPoint.getHomW());
                        
                        y[pos] = mResiduals[i];
                        pos++;                    
                    }
                }
            
                LevenbergMarquardtMultiDimensionFunctionEvaluator evaluator =
                        new LevenbergMarquardtMultiDimensionFunctionEvaluator() {
                
                    private Point2D mLeftPoint = Point2D.create(
                            CoordinatesType.HOMOGENEOUS_COORDINATES);
                    
                    private Point2D mRightPoint = Point2D.create(
                            CoordinatesType.HOMOGENEOUS_COORDINATES);
                        
                    private HomogeneousPoint2D mEpipole = 
                            new HomogeneousPoint2D();
                
                    private FundamentalMatrix mFundMatrix = 
                            new FundamentalMatrix();
                
                    private GradientEstimator mGradientEstimator =
                            new GradientEstimator(
                                    new MultiDimensionFunctionEvaluatorListener() {
                        @Override
                        public double evaluate(double[] point) 
                                throws Throwable {
                        
                            mEpipole.setCoordinates(point);
                            computeFundamentalMatrix(mHomography, mEpipole, 
                                    mFundMatrix);
                            return residual(mFundMatrix, mLeftPoint, 
                                    mRightPoint);
                        }
                    });
                        
                    @Override
                    public int getNumberOfDimensions() {
                        return nDims;
                    }

                    @Override
                    public double[] createInitialParametersArray() {
                        return initParams;
                    }

                    @Override
                    public double evaluate(int i, double[] point, 
                            double[] params, double[] derivatives) 
                            throws Throwable {
                        mLeftPoint.setHomogeneousCoordinates(point[0], point[1],
                                point[2]);
                        mRightPoint.setHomogeneousCoordinates(point[3], 
                                point[4], point[5]);

                        mEpipole.setCoordinates(params);
                        computeFundamentalMatrix(mHomography, mEpipole, 
                                mFundMatrix);
                    
                        double y = residual(mFundMatrix, mLeftPoint, 
                                mRightPoint);
                        mGradientEstimator.gradient(params, derivatives);
                    
                        return y;
                    }
                };
        
                LevenbergMarquardtMultiDimensionFitter fitter =
                        new LevenbergMarquardtMultiDimensionFitter(evaluator, x,
                                y, getRefinementStandardDeviation());
                
                fitter.fit();
                
                //obtain estimated params
                double[] params = fitter.getA();

                //update initial epipole for next iteration
                epipole.setHomogeneousCoordinates(params[0], params[1], 
                        params[2]);
            
                computeFundamentalMatrix(mHomography, epipole, 
                        fundamentalMatrix);
            
                double finalTotalResidual = totalResidual(fundamentalMatrix);
                iterErrorDecreased = finalTotalResidual < initialTotalResidual;                
                if (iterErrorDecreased || !errorDecreased) {
                    errorDecreased = true;
                    //update final result
                    result.setHomogeneousCoordinates(params[0], params[1], 
                            params[2]);
                    
                    if (mKeepCovariance) {
                        //keep covariance
                        mCovariance = fitter.getCovar();
                    }                    
                }
                numIter++;
            } while(iterErrorDecreased && numIter < MAX_ITERS);                
            
            if (mListener != null) {
                mListener.onRefineEnd(this, mInitialEstimation, result, 
                        errorDecreased);
            }
                
            return errorDecreased;

        } catch (Exception e) {
            throw new RefinerException(e);
        } finally {
            mLocked = false;
        }
    }        
}
