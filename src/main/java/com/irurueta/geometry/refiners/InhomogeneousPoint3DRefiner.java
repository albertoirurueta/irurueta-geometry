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
import com.irurueta.geometry.InhomogeneousPoint3D;
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
 * Refines an inhomogeneous 3D point by taking into account an initial 
 * estimation, inlier samples and their residuals.
 * This class can be used to find a solution that minimizes error of inliers in
 * LMSE terms.
 * Typically a refiner is used by a robust estimator, however it can also be
 * useful in some other situations.
 */
public class InhomogeneousPoint3DRefiner extends 
        Point3DRefiner<InhomogeneousPoint3D> {
    
    /**
     * Constructor.
     */
    public InhomogeneousPoint3DRefiner() { }
    
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
    public InhomogeneousPoint3DRefiner(InhomogeneousPoint3D initialEstimation,
            boolean keepCovariance, BitSet inliers, double[] residuals,
            int numInliers, List<Plane> samples, 
            double refinementStandardDeviation) {
        super(initialEstimation, keepCovariance, inliers, residuals, numInliers,
                samples, refinementStandardDeviation);
    }
    
    /**
     * Constructor.
     * @param initialEstimation initial estimation to be set.
     * @param keepCovariance true if covariance of estimation must be kept after
     * refinement, false otherwise.
     * @param inliersData inlier data, typically obtained from a robust 
     * estimator.
     * @param samples collection of samples.
     * @param refinementStandardDeviation standard deviation used for
     * Levenberg-Marquardt fitting.
     */
    public InhomogeneousPoint3DRefiner(InhomogeneousPoint3D initialEstimation,
            boolean keepCovariance, InliersData inliersData, 
            List<Plane> samples, double refinementStandardDeviation) {
        super(initialEstimation, keepCovariance, inliersData, samples, 
                refinementStandardDeviation);
    }
    
    /**
     * Refines provided initial estimation.
     * @return refined estimation.
     * @throws NotReadyException if not enough input data has been provided.
     * @throws LockedException if estimator is locked because refinement is
     * already in progress.
     * @throws RefinerException if refinement fails for some reason (e.g. unable
     * to converge to a result).
     */
    @Override
    public InhomogeneousPoint3D refine() throws NotReadyException,
            LockedException, RefinerException {
        InhomogeneousPoint3D result = new InhomogeneousPoint3D();
        refine(result);
        return result;
    }
    
    /**
     * Refines provided initial estimation.
     * This method always sets a value into provided result instance regardless
     * of the fact that error has actually improved in LMSE terms or not.
     * @param result instance where refined estimation will be stored.
     * @return true if result improved (decreases) in LMSE terms respect to
     * initial estimation, false if no improvement has been achieved.
     * @throws NotReadyException if not enough input data has been provided.
     * @throws LockedException if estimator is locked because refinement is
     * already in progress.
     * @throws RefinerException if refinement fails for some reason (e.g. unable
     * to converge to a result).
     */
    @Override
    public boolean refine(InhomogeneousPoint3D result) throws NotReadyException,
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
        
        double initialTotalResidual = totalResidual(mInitialEstimation);
        
        try {
            final double[] initParams = mInitialEstimation.asArray();
            
            //output values to be fitted/optimized will contain residuals
            double[] y = new double[mNumInliers];
            //input values will contain planes to compute residuals
            final int nDims = Plane.PLANE_NUMBER_PARAMS;
            Matrix x = new Matrix(mNumInliers, nDims);
            int nSamples = mInliers.length();
            int pos = 0;
            Plane plane;
            for (int i = 0; i < nSamples; i++) {
                if (mInliers.get(i)) {
                    //sample is inlier
                    plane = mSamples.get(i);
                    plane.normalize();
                    x.setElementAt(pos, 0, plane.getA());
                    x.setElementAt(pos, 1, plane.getB());
                    x.setElementAt(pos, 2, plane.getC());
                    x.setElementAt(pos, 3, plane.getD());
                    
                    y[pos] = mResiduals[i];
                    pos++;
                }
            }
            
            LevenbergMarquardtMultiDimensionFunctionEvaluator evaluator =
                    new LevenbergMarquardtMultiDimensionFunctionEvaluator() {
                        
                private Plane mPlane = new Plane();
                
                private InhomogeneousPoint3D mPoint = 
                        new InhomogeneousPoint3D();
                
                private GradientEstimator mGradientEstimator = 
                        new GradientEstimator(
                                new MultiDimensionFunctionEvaluatorListener() {
                    @Override
                    public double evaluate(double[] point) {
                        
                        mPoint.setCoordinates(point);
                        return residual(mPoint, mPlane);
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
                    //point contains a,b,c,d values for plane
                    mPlane.setParameters(point);
                    
                    //params contains coordinates of point
                    mPoint.setCoordinates(params);
                    
                    double y = residual(mPoint, mPlane);
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
            
            //update point
            result.setCoordinates(params);
            
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
}
