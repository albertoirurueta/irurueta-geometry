/*
 * Copyright (C) 2017 Alberto Irurueta Carro (alberto@irurueta.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *         http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package com.irurueta.geometry.refiners;

import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.CoordinatesType;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.numerical.EvaluationException;
import com.irurueta.numerical.GradientEstimator;
import com.irurueta.numerical.MultiDimensionFunctionEvaluatorListener;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiDimensionFitter;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiDimensionFunctionEvaluator;
import com.irurueta.numerical.optimization.PowellMultiOptimizer;
import com.irurueta.numerical.robust.InliersData;

import java.util.BitSet;
import java.util.List;

/**
 * A pinhole camera refiner using point correspondences and the
 * Powell algorithm to try to decrease overall error in LMSE terms among
 * inlier samples by taking the decomposed parameters of a pinhole camera.
 * Typically this refiner is used by a robust estimator, however it can also be
 * useful in some other situations.
 */
@SuppressWarnings("WeakerAccess")
public class DecomposedPointCorrespondencePinholeCameraRefiner extends 
        PointCorrespondencePinholeCameraRefiner {

    /**
     * Default value for minimum suggestion weight. This weight is used to 
     * slowly draw original camera parameters into desired suggested values.
     * Suggestion weight slowly increases each time Levenberg-Marquardt is used
     * to find a solution so that the algorithm can converge into desired value.
     * The faster the weights are increased the less likely that suggested 
     * values can be converged if they differ too much from the original ones.
     */
    public static final double DEFAULT_MIN_SUGGESTION_WEIGHT = 0.1;
    
    /**
     * Default value for maximum suggestion weight. This weight is used to
     * slowly draw original camera parameters into desired suggested values.
     * Suggestion weight slowly increases each time Levenberg-Marquardt is used
     * to find a solution so that the algorithm can converge into desired value.
     * The faster the weights are increased the less likely that suggested
     * values can be converged if they differ too much from the original ones.
     */
    public static final double DEFAULT_MAX_SUGGESTION_WEIGHT = 2.0;
        
    /**
     * Default value for the step to increase suggestion weight. This weight is 
     * used to slowly draw original camera parameters into desired suggested 
     * values. Suggestion weight slowly increases each time Levenberg-Marquardt 
     * is used to find a solution so that the algorithm can converge into 
     * desired value. The faster the weights are increased the less likely that 
     * suggested values can be converged if they differ too much from the 
     * original ones.
     */
    public static final double DEFAULT_SUGGESTION_WEIGHT_STEP = 0.475;
    
    /**
     * Dimensions for refinement.
     */
    private static final int REFINE_DIMS = 12;    
    
    /**
     * Minimum suggestion weight. This weight is used to slowly draw original
     * camera parameters into desired suggested values.
     * Suggestion weight slowly increases each time Levenberg-Marquardt is used
     * to find a solution so that the algorithm can converge into desired value.
     * The faster the weights are increased the less likely that suggested 
     * values can be converged if they differ too much from the original ones.
     */
    protected double mMinSuggestionWeight = DEFAULT_MIN_SUGGESTION_WEIGHT;
    
    /**
     * Maximum suggestion weight. This weight is used to slowly draw original
     * camera parameters into desired suggested values.
     * Suggestion weight slowly increases each time Levenberg-Marquardt is used
     * to find a solution so that the algorithm can converge into desired value.
     * The faster the weights are increased the less likely that suggested
     * values can be converged if they differ too much from the original ones.
     */
    protected double mMaxSuggestionWeight = DEFAULT_MAX_SUGGESTION_WEIGHT;
    
    /**
     * Step to increase suggestion weight. This weight is used to slowly draw
     * original camera parameters into desired suggested values. Suggestion 
     * weight slowly increases each time Levenberg-Marquardt is used to find a
     * solution so that the algorithm can converge into desired value. The 
     * faster the weights are increased the less likely that suggested values
     * can be converged if they differ too much from the original ones.
     */
    protected double mSuggestionWeightStep = DEFAULT_SUGGESTION_WEIGHT_STEP;    
    
    /**
     * Instance of a pinhole camera to be reused during refinement.
     */
    private PinholeCamera mRefineCamera;
    
    /**
     * Current weight during refinement.
     */
    protected double mCurrentWeight;
    
    /**
     * Constructor.
     */
    public DecomposedPointCorrespondencePinholeCameraRefiner() { }
    
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
    public DecomposedPointCorrespondencePinholeCameraRefiner(
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
    public DecomposedPointCorrespondencePinholeCameraRefiner(
            PinholeCamera initialEstimation, boolean keepCovariance,
            InliersData inliersData, List<Point3D> samples1, 
            List<Point2D> samples2, double refinementStandardDeviation) {
        super(initialEstimation, keepCovariance, inliersData, samples1, 
                samples2, refinementStandardDeviation);
    }
    
    /**
     * Gets minimum suggestion weight. This weight is used to slowly draw 
     * original camera parameters into desired suggested values.
     * Suggestion weight slowly increases each time Levenberg-Marquardt is used
     * to find a solution so that the algorithm can converge into desired value.
     * The faster the weights are increased the less likely that suggested 
     * values can be converged if they differ too much from the original ones.
     * @return minimum suggestion weight.
     */
    public double getMinSuggestionWeight() {
        return mMinSuggestionWeight;
    }
    
    /**
     * Sets minimum suggestion weight. This weight is used to slowly draw
     * original camera parameters into desired suggested values.
     * Suggestion weight slowly increases each time Levenberg-Marquardt is used
     * to find a solution so that the algorithm can converge into desired value.
     * The faster the weights are increased the less likely that suggested
     * values can be converged if they differ too much from the original ones.
     * @param minSuggestionWeight minimum suggestion weight.
     * @throws LockedException if estimator is locked.
     */
    public void setMinSuggestionWeight(double minSuggestionWeight) 
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mMinSuggestionWeight = minSuggestionWeight;
    }
    
    /**
     * Gets maximum suggestion weight. This weight is used to slowly draw 
     * original camera parameters into desired suggested values.
     * Suggestion weight slowly increases each time Levenberg-Marquardt is used
     * to find a solution so that the algorithm can converge into desired value.
     * The faster the weights are increased the less likely that suggested
     * values can be converged if they differ too much from the original ones.
     * @return maximum suggestion weight.
     */
    public double getMaxSuggestionWeight() {
        return mMaxSuggestionWeight;
    }
    
    /**
     * Sets maximum suggestion weight. This weight is used to slowly draw
     * original camera parameters into desired suggested values.
     * Suggestion weight slowly increases each time Levenberg-Marquardt is used
     * to find a solution so that the algorithm can converge into desired value.
     * The faster the weights are increased the less likely that suggested
     * values can be converged if they differ too much from the original ones.
     * @param maxSuggestionWeight maximum suggestion weight.
     * @throws LockedException if estimator is locked.
     */
    public void setMaxSuggestionWeight(double maxSuggestionWeight)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mMaxSuggestionWeight = maxSuggestionWeight;
    }
    
    /**
     * Sets minimum and maximum suggestion weights. Suggestion weight is used to
     * slowly draw original camera parameters into desired suggested values.
     * Suggestion weight slowly increases each time Levenberg-Marquardt is used
     * to find a solution so that the algorithm can converge into desired value.
     * The faster the weights are increased the less likely that suggested
     * values can be converged if they differ too much from the original ones.
     * @param minSuggestionWeight minimum suggestion weight.
     * @param maxSuggestionWeight maximum suggestion weight.
     * @throws LockedException if estimator is locked.
     * @throws IllegalArgumentException if minimum suggestion weight is greater 
     * or equal than maximum value.
     */
    public void setMinMaxSuggestionWeight(double minSuggestionWeight, 
            double maxSuggestionWeight) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (minSuggestionWeight >= maxSuggestionWeight) {
            throw new IllegalArgumentException();
        }
        
        mMinSuggestionWeight = minSuggestionWeight;
        mMaxSuggestionWeight = maxSuggestionWeight;
    }
    
    /**
     * Gets step to increase suggestion weight. This weight is used to slowly
     * draw original camera parameters into desired suggested values. Suggestion
     * weight slowly increases each time Levenberg-Marquardt is used to find a
     * solution so that the algorithm can converge into desired value. The 
     * faster the weights are increased the less likely that suggested values
     * can be converged if they differ too much from the original ones.
     * @return step to increase suggestion weight.
     */
    public double getSuggestionWeightStep() {
        return mSuggestionWeightStep;
    }
    
    /**
     * Sets step to increase suggestion weight. This weight is used to slowly
     * draw original camera parameters into desired suggested values. Suggestion
     * weight slowly increases each time Levenberg-Marquardt is used to find a
     * solution so that the algorithm can converge into desired value. The 
     * faster the weights are increased the less likely that suggested values
     * can be converged if they differ too much from the original ones.
     * @param suggestionWeightStep step to increase suggestion weight.
     * @throws LockedException if estimator is locked.
     * @throws IllegalArgumentException if provided step is negative or zero.
     */
    public void setSuggestionWeightStep(double suggestionWeightStep) 
            throws LockedException, IllegalArgumentException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (suggestionWeightStep <= 0.0) {
            throw new IllegalArgumentException();
        }
        
        mSuggestionWeightStep = suggestionWeightStep;
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
        
        boolean improved = refinePowell(result);
        
        if (mKeepCovariance) {
            mCovariance = estimateCovarianceLevenbergMarquardt(
                    improved ? result : mInitialEstimation, 
                    mCurrentWeight);
        }
        
        if (mListener != null) {
            mListener.onRefineEnd(this, mInitialEstimation, result, improved);
        }
        
        mLocked = false;
        
        return improved;
    } 
    
    /**
     * Estimates covariance matrix for provided estimated and refined camera
     * @param pinholeCamera pinhole camera to estimate covariance for.
     * @param weight weight for suggestion residual.
     * @return estimated covariance or null if anything fails.
     */
    private Matrix estimateCovarianceLevenbergMarquardt(
            PinholeCamera pinholeCamera, final double weight) {
        try {
            pinholeCamera.normalize();
            
            //output values to be fitted/optimized will contain residuals
            double[] y = new double[mNumInliers];
            //input values will contain 3D point and 2D point to compute 
            //residuals
            final int nDims = Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH +
                    Point3D.POINT3D_HOMOGENEOUS_COORDINATES_LENGTH;
            Matrix x = new Matrix(mNumInliers, nDims);
            int nSamples = mInliers.length();
            int pos = 0;
            Point2D point2D;
            Point3D point3D;
            final double[] initParams = new double[REFINE_DIMS];
            cameraToParameters(pinholeCamera, initParams);
            
            double suggestionResidual = hasSuggestions() ?
                    suggestionResidual(initParams, weight) : 0.0;
            for (int i = 0; i < nSamples; i++) {
                if (mInliers.get(i)) {
                    //sample is inlier
                    point2D = mSamples2.get(i);
                    point3D = mSamples1.get(i);
                    point2D.normalize();
                    point3D.normalize();
                    x.setElementAt(pos, 0, point2D.getHomX());
                    x.setElementAt(pos, 1, point2D.getHomY());
                    x.setElementAt(pos, 2, point2D.getHomW());
                    x.setElementAt(pos, 3, point3D.getHomX());
                    x.setElementAt(pos, 4, point3D.getHomY());
                    x.setElementAt(pos, 5, point3D.getHomZ());
                    x.setElementAt(pos, 6, point3D.getHomW());
                        
                    y[pos] = Math.pow(mResiduals[i], 2.0) + suggestionResidual;
                    pos++;
                }
            }
            
            LevenbergMarquardtMultiDimensionFunctionEvaluator evaluator =
                    new LevenbergMarquardtMultiDimensionFunctionEvaluator() {
                        
                private Point2D mPoint2D = Point2D.create(
                        CoordinatesType.HOMOGENEOUS_COORDINATES);
                
                private Point3D mPoint3D = Point3D.create(
                        CoordinatesType.HOMOGENEOUS_COORDINATES);
                
                private PinholeCamera mPinholeCamera = new PinholeCamera();
                
                private GradientEstimator mGradientEstimator =
                        new GradientEstimator(
                                new MultiDimensionFunctionEvaluatorListener() {
                    @Override
                    public double evaluate(double[] params) {
                        
                        parametersToCamera(params, mPinholeCamera);
                        return residualLevenbergMarquardt(mPinholeCamera, 
                                mPoint3D, mPoint2D, params, weight);
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
                        double[] derivatives) throws EvaluationException {
                    mPoint2D.setHomogeneousCoordinates(point[0], point[1], 
                            point[2]);
                    mPoint3D.setHomogeneousCoordinates(point[3], point[4], 
                            point[5], point[6]);

                    parametersToCamera(params, mPinholeCamera);
                    double y = residualLevenbergMarquardt(mPinholeCamera, 
                            mPoint3D, mPoint2D, params, weight);                      
                    mGradientEstimator.gradient(params, derivatives);
                        
                    return y;
                }
            };
            
            LevenbergMarquardtMultiDimensionFitter fitter =
                    new LevenbergMarquardtMultiDimensionFitter(evaluator, 
                    x, y, getRefinementStandardDeviation());
                
            fitter.fit();

            //obtain covariance
            return fitter.getCovar();
            
        } catch (Exception e) {
            //estimation failed, so we return null
            return null;
        }
    }
        
    /**
     * Refines camera using Powell optimization to minimize a cost function
     * consisting on the sum of squared projection residuals plus the
     * suggestion residual for any suggested terms.
     * @param result instance where refined estimation will be stored.
     * @return true if result improves (decreases) in LMSE terms respect to
     * initial estimation, false if no improvement has been achieved.
     */    
    private boolean refinePowell(PinholeCamera result) {
        boolean improvedAtLeastOnce = false;
        mCurrentWeight = mMinSuggestionWeight;    
        
        if (hasSuggestions()) {
            try {
                //copy camera into a new instance
                mRefineCamera = new PinholeCamera(
                        mInitialEstimation.getInternalMatrix().clone());
                mRefineCamera.normalize();
                
                double[] startPoint = new double[REFINE_DIMS];
                RefinementMultiDimensionFunctionEvaluatorListener listener =
                        new RefinementMultiDimensionFunctionEvaluatorListener();
                PowellMultiOptimizer optimizer = new PowellMultiOptimizer(
                        listener,
                        PowellMultiOptimizer.DEFAULT_TOLERANCE);
                
                boolean improved;
                do {           
                    improved = refinementStepPowell(optimizer, listener, 
                            startPoint, mCurrentWeight);
                    
                    if(improved) {
                        //update result
                        result.setInternalMatrix(
                                mRefineCamera.getInternalMatrix().clone());
                        improvedAtLeastOnce = true;
                    }
                    
                    mCurrentWeight += mSuggestionWeightStep;
                } while (mCurrentWeight < mMaxSuggestionWeight && improved);
                                                                                
                return improvedAtLeastOnce;
            } catch (Exception e) {
                //refinement failed, so we return input value
                return improvedAtLeastOnce;
            }
        }
        return false;
    }
    
    /**
     * Computes one refinement step using Powell optimizer for a given weight
     * on suggestion terms.
     * @param optimizer Powell optimizer to be reused.
     * @param listener Powell optimizer listener to be reused.
     * @param startPoint starting point for powell optimization. This array is
     * passed only for reuse purposes.
     * @param weight suggestion terms weight.
     * @return true if this refinement step decreased projection error in LMSE
     * terms, false otherwise.
     * @throws Exception if something failed.
     */
    private boolean refinementStepPowell(PowellMultiOptimizer optimizer, 
            RefinementMultiDimensionFunctionEvaluatorListener listener,
            double[] startPoint, double weight) throws Exception {
        
        listener.weight = weight;
        cameraToParameters(mRefineCamera, startPoint);
        double initResidual = residualPowell(mRefineCamera, startPoint, weight);
                    
        optimizer.setStartPoint(startPoint);                
        optimizer.minimize();
                    
        double[] resultParams = optimizer.getResult();
        parametersToCamera(resultParams, mRefineCamera);
        
        double finalResidual = residualPowell(mRefineCamera, resultParams, 
                weight);
        
        return finalResidual < initResidual;
    }

    /**
     * Listener for powell optimizer to minimize cost function during 
     * refinement.
     * A weight can be provided so that required parameters are slowly drawn
     * to suggested values.
     */
    private class RefinementMultiDimensionFunctionEvaluatorListener 
            implements MultiDimensionFunctionEvaluatorListener {
        /**
         * Weight to slowly draw parameters to suggested values.
         */
        public double weight;
        
        /**
         * Evaluates cost function
         * @param point parameters to evaluate cost function.
         * @return cost value.
         * @throws EvaluationException if anything fails.
         */
        @Override
        public double evaluate(double[] point) throws EvaluationException {
            parametersToCamera(point, mRefineCamera);
            return residualPowell(mRefineCamera, point, weight);
        }        
    }
}
