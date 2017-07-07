/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.refiners.NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 2, 2017.
 */
package com.irurueta.geometry.refiners;

import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.Line2D;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.Plane;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.numerical.GradientEstimator;
import com.irurueta.numerical.MultiDimensionFunctionEvaluatorListener;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiDimensionFitter;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiDimensionFunctionEvaluator;
import com.irurueta.numerical.robust.InliersData;
import java.util.BitSet;
import java.util.List;

/**
 * A pinhole camera refiner using line/plane correspondences and the
 * Levenberg-Marquardt algorithm to try to decrease overall error in LMSE terms
 * among inlier samples by taking the pinhole camera matrix as a whole without
 * decomposition.
 * Typically this refiner is used by a robust estimator, however it can also be
 * useful in some other situations.
 */
public class NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner extends
        LinePlaneCorrespondencePinholeCameraRefiner {

    /**
     * Default value for the weight applied to errors related to suggested 
     * camera parameters during computation of projection residuals.
     */
    public static final double DEFAULT_SUGGESTION_ERROR_WEIGHT = 2.0;
        
    /**
     * Dimensions for refinement.
     */
    private static final int REFINE_DIMS = 12;

    /**
     * Suggestion error weight. This weight is applied to errors related to 
     * suggested camera parameters during computation of projection residuals.
     */
    private double mSuggestionErrorWeight = DEFAULT_SUGGESTION_ERROR_WEIGHT;
    
    /**
     * Constructor.
     */
    public NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner() { }
    
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
    public NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner(
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
    public NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner(
            PinholeCamera initialEstimation, boolean keepCovariance,
            InliersData inliersData, List<Plane> samples1, 
            List<Line2D> samples2, double refinementStandardDeviation) {
        super(initialEstimation, keepCovariance, inliersData, samples1, 
                samples2, refinementStandardDeviation);
    }
    
    /**
     * Gets suggestion error weight. This weight is applied to errors related to
     * suggested camera parameters during computation of projection residuals.
     * @return suggestion error weight.
     */
    public double getSuggestionErrorWeight() {
        return mSuggestionErrorWeight;
    }
    
    /**
     * Sets suggestion error weight. This weight is applied to errors related to
     * suggested camera parameters during computation of projection residuals.
     * @param suggestionErrorWeight suggestion error weight.
     * @throws LockedException if estimator is locked.
     */
    public void setSuggestionErrorWeight(double suggestionErrorWeight)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mSuggestionErrorWeight = suggestionErrorWeight;
    }    
    
    /**
     * Refines provided initial estimation.
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
    public boolean refine(PinholeCamera result) throws NotReadyException, 
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
            mInitialEstimation.normalize();
            
            //output values to be fitted/optimized will contain residuals
            double[] y = new double[mNumInliers];
            //input values will contain line and plane to compute reiduals
            final int nDims = Line2D.LINE_NUMBER_PARAMS +
                    Plane.PLANE_NUMBER_PARAMS;
            Matrix x = new Matrix(mNumInliers, nDims);
            int nSamples = mInliers.length();
            int pos = 0;
            Line2D line;
            Plane plane;
            final double[] initParams = new double[REFINE_DIMS];
            cameraToParameters(mInitialEstimation, initParams);

            double initResidual = residualPowell(mInitialEstimation, initParams, 
                        mSuggestionErrorWeight);
            
            double suggestionResidual = hasSuggestions() ?
                    suggestionResidual(initParams, mSuggestionErrorWeight) : 
                    0.0;
            for (int i = 0; i < nSamples; i++) {
                if (mInliers.get(i)) {
                    //sample is inlier
                    line = mSamples2.get(i);
                    plane = mSamples1.get(i);
                    line.normalize();
                    plane.normalize();
                    x.setElementAt(pos, 0, line.getA());
                    x.setElementAt(pos, 1, line.getB());
                    x.setElementAt(pos, 2, line.getC());
                    x.setElementAt(pos, 3, plane.getA());
                    x.setElementAt(pos, 4, plane.getB());
                    x.setElementAt(pos, 5, plane.getC());
                    x.setElementAt(pos, 6, plane.getD());
                        
                    y[pos] = Math.pow(mResiduals[i], 2.0) + suggestionResidual;
                    pos++;
                }
            }
            
            LevenbergMarquardtMultiDimensionFunctionEvaluator evaluator =
                    new LevenbergMarquardtMultiDimensionFunctionEvaluator() {
                        
                private Line2D mLine = new Line2D();
                    
                private Plane mPlane = new Plane();
                
                private PinholeCamera mPinholeCamera = new PinholeCamera();
                
                private GradientEstimator mGradientEstimator =
                        new GradientEstimator(
                                new MultiDimensionFunctionEvaluatorListener() {
                    @Override
                    public double evaluate(double[] params) throws Throwable {
                        
                        parametersToCamera(params, mPinholeCamera);
                        return residualLevenbergMarquardt(mPinholeCamera, 
                                mLine, mPlane, params, mSuggestionErrorWeight);
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
                public double evaluate(int i, double[] point, double[] params, 
                        double[] derivatives) throws Throwable {
                    mLine.setParameters(point[0], point[1], point[2]);
                    mPlane.setParameters(point[3], point[4], point[5], 
                            point[6]);
                        
                    mLine.normalize();
                    mPlane.normalize();

                    parametersToCamera(params, mPinholeCamera);
                    double y = residualLevenbergMarquardt(mPinholeCamera, 
                            mLine, mPlane, params, mSuggestionErrorWeight);                      
                    mGradientEstimator.gradient(params, derivatives);
                        
                    return y;
                }
            };
            
            LevenbergMarquardtMultiDimensionFitter fitter = 
                    new LevenbergMarquardtMultiDimensionFitter(evaluator, 
                        x, y, mRefinementStandardDeviation);
                
            fitter.fit();

            double[] finalParams = fitter.getA();

            parametersToCamera(finalParams, result);
            
            if (mKeepCovariance) {
                mCovariance = fitter.getCovar();
            }
            
            double finalResidual = residualPowell(result, finalParams, 
                        mSuggestionErrorWeight);
            boolean errorDecreased = finalResidual < initResidual;
            
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
