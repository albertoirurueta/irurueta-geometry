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
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.ProjectiveTransformation3D;
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
 * A 3D projective transformation refiner using point correspondences.
 * This class takes into account an initial estimation, inlier point matches
 * and their residuals to find a solution that minimizes error of inliers in 
 * LMSE terms.
 * Typically a refiner is used by a robust estimator, however it can also be 
 * useful in some other situations.
 */
public class PointCorrespondenceProjectiveTransformation3DRefiner extends
        ProjectiveTransformation3DRefiner<Point3D, Point3D> {
    
    /**
     * Point to be reused when computing residuals.
     */
    private Point3D mResidualTestPoint = Point3D.create(
            CoordinatesType.HOMOGENEOUS_COORDINATES);

    /**
     * Constructor.
     */
    public PointCorrespondenceProjectiveTransformation3DRefiner() { }
    
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
    public PointCorrespondenceProjectiveTransformation3DRefiner(
            ProjectiveTransformation3D initialEstimation, 
            boolean keepCovariance, BitSet inliers, double[] residuals, 
            int numInliers, List<Point3D> samples1, List<Point3D> samples2,
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
    public PointCorrespondenceProjectiveTransformation3DRefiner(
            ProjectiveTransformation3D initialEstimation, 
            boolean keepCovariance, InliersData inliersData, 
            List<Point3D> samples1, List<Point3D> samples2, 
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
    public boolean refine(ProjectiveTransformation3D result) throws NotReadyException,
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
        
        mInitialEstimation.normalize();
        
        double initialTotalResidual = totalResidual(mInitialEstimation);

        try {
            final double[] initParams = new double[
                    ProjectiveTransformation3D.HOM_COORDS * 
                    ProjectiveTransformation3D.HOM_COORDS];
            //copy values
            System.arraycopy(mInitialEstimation.getT().getBuffer(), 0, 
                    initParams, 0, initParams.length);

            //output values to be fitted/optimized will contain residuals
            double[] y = new double[mNumInliers];
            //input values will contain 2 sets of 2D points to compute residuals
            final int nDims = 
                    2 * Point3D.POINT3D_HOMOGENEOUS_COORDINATES_LENGTH;
            Matrix x = new Matrix(mNumInliers, nDims);
            int nSamples = mInliers.length();
            int pos = 0;
            Point3D inputPoint, outputPoint;
            for (int i = 0; i < nSamples; i++) {
                if (mInliers.get(i)) {
                    //sample is inlier
                    inputPoint = mSamples1.get(i);
                    outputPoint = mSamples2.get(i);
                    inputPoint.normalize();
                    outputPoint.normalize();
                    x.setElementAt(pos, 0, inputPoint.getHomX());
                    x.setElementAt(pos, 1, inputPoint.getHomY());
                    x.setElementAt(pos, 2, inputPoint.getHomZ());
                    x.setElementAt(pos, 3, inputPoint.getHomW());
                    x.setElementAt(pos, 4, outputPoint.getHomX());
                    x.setElementAt(pos, 5, outputPoint.getHomY());
                    x.setElementAt(pos, 6, outputPoint.getHomZ());
                    x.setElementAt(pos, 7, outputPoint.getHomW());
                    
                    y[pos] = mResiduals[i];
                    pos++;
                }
            }
            
            LevenbergMarquardtMultiDimensionFunctionEvaluator evaluator =
                    new LevenbergMarquardtMultiDimensionFunctionEvaluator() {
                        
                private Point3D mInputPoint = Point3D.create(
                        CoordinatesType.HOMOGENEOUS_COORDINATES);
                
                private Point3D mOutputPoint = Point3D.create(
                        CoordinatesType.HOMOGENEOUS_COORDINATES);
                
                private ProjectiveTransformation3D mTransformation =
                        new ProjectiveTransformation3D();
                
                private GradientEstimator mGradientEstimator =
                        new GradientEstimator(
                                new MultiDimensionFunctionEvaluatorListener() {
                    @Override
                    public double evaluate(double[] params) {
                        //copy values
                        System.arraycopy(params, 0, 
                                mTransformation.getT().getBuffer(), 0, 
                                params.length);
                        
                        return residual(mTransformation, mInputPoint, 
                                mOutputPoint);
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
                    mInputPoint.setHomogeneousCoordinates(point[0], point[1], 
                            point[2], point[3]);
                    mOutputPoint.setHomogeneousCoordinates(point[4], point[5], 
                            point[6], point[7]);
                    
                    //copy values
                    System.arraycopy(params, 0, 
                            mTransformation.getT().getBuffer(), 0, 
                            params.length);
                    
                    double y = residual(mTransformation, mInputPoint, 
                            mOutputPoint);
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
     * matched points.
     * @param transformation a transformation.
     * @param inputPoint input 2D point.
     * @param outputPoint output 2D point.
     * @return residual.
     */
    private double residual(ProjectiveTransformation3D transformation, 
            Point3D inputPoint, Point3D outputPoint) {
        inputPoint.normalize();
        outputPoint.normalize();
        
        transformation.transform(inputPoint, mResidualTestPoint);        
        return mResidualTestPoint.distanceTo(outputPoint);
    }
    
    /**
     * Computes total residual among all provided inlier samples.
     * @param transformation a transformation.
     * @return total residual.
     */
    private double totalResidual(ProjectiveTransformation3D transformation) {
        double result = 0.0;
        
        int nSamples = mInliers.length();
        Point3D inputPoint, outputPoint;
        for (int i = 0; i < nSamples; i++) {
            if (mInliers.get(i)) {
                //sample is inlier
                inputPoint = mSamples1.get(i);
                outputPoint = mSamples2.get(i);
                inputPoint.normalize();
                outputPoint.normalize();
                result += residual(transformation, inputPoint, outputPoint);
            }
        }
        
        return result;
    }    
}
