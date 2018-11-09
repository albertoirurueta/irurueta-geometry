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

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.Line2D;
import com.irurueta.geometry.ProjectiveTransformation2D;
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
 * A 2D projective transformation refiner using line correspondences.
 * This class takes into account an initial estimation, inlier line matches
 * and their residuals to find a solution that minimizes error of inliers in
 * LMSE terms.
 * Typically a refiner is used by a robust estimator, however it can also be
 * useful in some other situations.
 */
public class LineCorrespondenceProjectiveTransformation2DRefiner extends
        ProjectiveTransformation2DRefiner<Line2D, Line2D> {

    /**
     * Line to be reused when computing residuals.
     */
    private Line2D mResidualTestLine = new Line2D();
        
    /**
     * Constructor.
     */
    public LineCorrespondenceProjectiveTransformation2DRefiner() { }
    
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
    public LineCorrespondenceProjectiveTransformation2DRefiner(
            ProjectiveTransformation2D initialEstimation, 
            boolean keepCovariance, BitSet inliers, double[] residuals, 
            int numInliers, List<Line2D> samples1, List<Line2D> samples2,
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
    public LineCorrespondenceProjectiveTransformation2DRefiner(
            ProjectiveTransformation2D initialEstimation, 
            boolean keepCovariance, InliersData inliersData, 
            List<Line2D> samples1, List<Line2D> samples2, 
            double refinementStandardDeviation) {
        super(initialEstimation, keepCovariance, inliersData, samples1, 
                samples2, refinementStandardDeviation);
    }
    
    /**
     * Refines provided initial estimation.
     * This method always sets a value into provided result instance regardless
     * of the fact that error has actually improved in LMSE terms or not.
     * @param result instance where refined estimation will be stored.
     * @return true if result improves (error decreases) in LMSE terms respect 
     * to initial estimation, false if no improvement has been achieved.
     * @throws NotReadyException if not enough input data has been provided.
     * @throws LockedException if estimator is locked because refinement is 
     * already in progress.
     * @throws RefinerException if refinement fails for some reason (e.g. unable
     * to converge to a result).
     */    
    @Override
    public boolean refine(ProjectiveTransformation2D result) 
            throws NotReadyException, LockedException, RefinerException {
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
        
        mInitialEstimation.normalize();
        
        double initialTotalResidual = totalResidual(mInitialEstimation);
        
        try {
            final double[] initParams = new double[
                    ProjectiveTransformation2D.HOM_COORDS * 
                    ProjectiveTransformation2D.HOM_COORDS];
            //copy values
            System.arraycopy(mInitialEstimation.getT().getBuffer(), 0, 
                    initParams, 0, initParams.length);

            //output values to be fitted/optimized will contain residuals
            double[] y = new double[mNumInliers];
            //input values will contain 2 sets of 2D lines to compute residuals
            final int nDims = 2 * Line2D.LINE_NUMBER_PARAMS;
            Matrix x = new Matrix(mNumInliers, nDims);
            int nSamples = mInliers.length();
            int pos = 0;
            Line2D inputLine, outputLine;
            for (int i = 0; i < nSamples; i++) {
                if (mInliers.get(i)) {
                    //sample is inlier
                    inputLine = mSamples1.get(i);
                    outputLine = mSamples2.get(i);
                    inputLine.normalize();
                    outputLine.normalize();
                    x.setElementAt(pos, 0, inputLine.getA());
                    x.setElementAt(pos, 1, inputLine.getB());
                    x.setElementAt(pos, 2, inputLine.getC());
                    x.setElementAt(pos, 3, outputLine.getA());
                    x.setElementAt(pos, 4, outputLine.getB());
                    x.setElementAt(pos, 5, outputLine.getC());
                    
                    y[pos] = mResiduals[i];
                    pos++;                    
                }
            }
            
            LevenbergMarquardtMultiDimensionFunctionEvaluator evaluator =
                    new LevenbergMarquardtMultiDimensionFunctionEvaluator() {
                        
                private Line2D mInputLine = new Line2D();
                
                private Line2D mOutputLine = new Line2D();
                
                private ProjectiveTransformation2D mTransformation =
                        new ProjectiveTransformation2D();
                
                private GradientEstimator mGradientEstimator =
                        new GradientEstimator(
                                new MultiDimensionFunctionEvaluatorListener() {
                    @Override
                    public double evaluate(double[] params) {
                        //copy values
                        System.arraycopy(params, 0, 
                                mTransformation.getT().getBuffer(), 0, 
                                params.length);
                        
                        return residual(mTransformation, mInputLine, 
                                mOutputLine);
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
                    mInputLine.setParameters(point[0], point[1], point[2]);
                    mOutputLine.setParameters(point[3], point[4], point[5]);
                    
                    //copy values
                    System.arraycopy(params, 0, 
                            mTransformation.getT().getBuffer(), 0, 
                            params.length);
                    
                    double y = residual(mTransformation, mInputLine, 
                            mOutputLine);
                    mGradientEstimator.gradient(params, derivatives);
                    
                    return y;

                }
            };
            
            LevenbergMarquardtMultiDimensionFitter fitter =
                    new LevenbergMarquardtMultiDimensionFitter(evaluator, x, y,
                    getRefinementStandardDeviation());
            
            fitter.fit();
            
            //obtain estimated params
            double[] params = fitter.getA();
            
            //update transformation
                    
            //copy values 
            System.arraycopy(params, 0, result.getT().getBuffer(), 0, 
                    params.length);
            
            if (mKeepCovariance) {
                //keep covariance
                mCovariance = fitter.getCovar();
            }
            
            double finalTotalResidual = totalResidual(result);
            boolean errorDecreased = finalTotalResidual < initialTotalResidual;
            
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
    
    /**
     * Computes the residual between the affine transformation and a pair of 
     * matched lines.
     * @param transformation a transformation.
     * @param inputLine input 2D line.
     * @param outputLine output 2D line.
     * @return residual.
     */
    private double residual(ProjectiveTransformation2D transformation, 
            Line2D inputLine, Line2D outputLine) {
        try {
            inputLine.normalize();
            outputLine.normalize();
        
            transformation.transform(inputLine, mResidualTestLine);        
            return 1.0 - Math.abs(outputLine.dotProduct(mResidualTestLine));
        } catch (AlgebraException e) {
            return 1.0;
        }
    }
    
    /**
     * Computes total residual among all provided inlier samples.
     * @param transformation a transformation.
     * @return total residual.
     */
    private double totalResidual(ProjectiveTransformation2D transformation) {
        double result = 0.0;
        
        int nSamples = mInliers.length();
        Line2D inputLine, outputLine;
        for (int i = 0; i < nSamples; i++) {
            if (mInliers.get(i)) {
                //sample is inlier
                inputLine = mSamples1.get(i);
                outputLine = mSamples2.get(i);
                inputLine.normalize();
                outputLine.normalize();
                result += residual(transformation, inputLine, outputLine);
            }
        }
        
        return result;
    }    
}
