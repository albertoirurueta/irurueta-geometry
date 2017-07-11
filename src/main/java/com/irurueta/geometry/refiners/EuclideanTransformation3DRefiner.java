/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.refiners.EuclideanTransformation3DRefiner
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date May 1, 2017.
 */
package com.irurueta.geometry.refiners;

import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.CoordinatesType;
import com.irurueta.geometry.EuclideanTransformation3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.Quaternion;
import com.irurueta.geometry.Rotation3DType;
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
 * Refine a 3D euclidean transformation by taking into account an initial 
 * estimation, inlier point matches and their residuals.
 * This class can be used to find a solution that minimizes error of inliers in
 * LMSE terms.
 * Typically a refiner is used by a robust estimator, however it can also be 
 * useful in some other situations.
 */
public class EuclideanTransformation3DRefiner extends 
        PairMatchesAndInliersDataRefiner<EuclideanTransformation3D, Point3D, 
        Point3D> {
    
    /**
     * Point to be reused when computing residuals.
     */    
    private Point3D mResidualTestPoint = Point3D.create(
            CoordinatesType.HOMOGENEOUS_COORDINATES);
    
    /**
     * Quaternion to be reused for refinement computations.
     */
    private Quaternion mQuaternion = new Quaternion();

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
    public EuclideanTransformation3DRefiner() { }
    
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
    public EuclideanTransformation3DRefiner(
            EuclideanTransformation3D initialEstimation, boolean keepCovariance,
            BitSet inliers, double[] residuals, int numInliers, 
            List<Point3D> samples1, List<Point3D> samples2, 
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
    public EuclideanTransformation3DRefiner(
            EuclideanTransformation3D initialEstimation, boolean keepCovariance,
            InliersData inliersData, List<Point3D> samples1, 
            List<Point3D> samples2, double refinementStandardDeviation) {
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
    public EuclideanTransformation3D refine() throws NotReadyException, 
            LockedException, RefinerException {
        EuclideanTransformation3D result = new EuclideanTransformation3D();
        refine(result);
        return result;
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
    public boolean refine(EuclideanTransformation3D result) 
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
        
        double initialTotalResidual = totalResidual(mInitialEstimation);
        
        try {
            //parameters: rotation angle + scale + translation
            final double[] initParams = new double[Quaternion.N_PARAMS + 
                    EuclideanTransformation3D.NUM_TRANSLATION_COORDS];
            //copy rotation values
            if (mInitialEstimation.getRotation().getType() == 
                    Rotation3DType.QUATERNION) {
                mQuaternion = (Quaternion)mInitialEstimation.getRotation();
            } else {
                mQuaternion = mInitialEstimation.getRotation().toQuaternion();
            }
            mQuaternion.normalize();
            
            //copy values
            initParams[0] = mQuaternion.getA();
            initParams[1] = mQuaternion.getB();
            initParams[2] = mQuaternion.getC();
            initParams[3] = mQuaternion.getD();
            
            System.arraycopy(mInitialEstimation.getTranslation(), 0, initParams,
                    Quaternion.N_PARAMS, 
                    EuclideanTransformation3D.NUM_TRANSLATION_COORDS);
            
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
                
                private EuclideanTransformation3D mTransformation = 
                        new EuclideanTransformation3D();
                
                private GradientEstimator mGradientEstimator =
                        new GradientEstimator(
                                new MultiDimensionFunctionEvaluatorListener() {
                    @Override
                    public double evaluate(double[] params) throws Throwable {
                        //copy values
                        mQuaternion.setA(params[0]);
                        mQuaternion.setB(params[1]);
                        mQuaternion.setC(params[2]);
                        mQuaternion.setD(params[3]);
                        mTransformation.setRotation(mQuaternion);
                                               
                        System.arraycopy(params, Quaternion.N_PARAMS, 
                            mTransformation.getTranslation(), 0, 
                            EuclideanTransformation3D.NUM_TRANSLATION_COORDS);

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
                    mQuaternion.setA(params[0]);
                    mQuaternion.setB(params[1]);
                    mQuaternion.setC(params[2]);
                    mQuaternion.setD(params[3]);
                    mTransformation.setRotation(mQuaternion);
                    
                    System.arraycopy(params, Quaternion.N_PARAMS, 
                        mTransformation.getTranslation(), 0, 
                        EuclideanTransformation3D.NUM_TRANSLATION_COORDS);

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
            mQuaternion.setA(params[0]);
            mQuaternion.setB(params[1]);
            mQuaternion.setC(params[2]);
            mQuaternion.setD(params[3]);
            result.setRotation(mQuaternion);
            System.arraycopy(params, Quaternion.N_PARAMS, 
                    result.getTranslation(), 0, 
                    EuclideanTransformation3D.NUM_TRANSLATION_COORDS);
            
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
     * Computes the residual between the euclidean transformation and a pair or
     * matched points.
     * @param transformation a transformation.
     * @param inputPoint input 2D point.
     * @param outputPoint output 2D point.
     * @return residual.
     */
    private double residual (EuclideanTransformation3D transformation, 
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
    private double totalResidual(EuclideanTransformation3D transformation) {
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