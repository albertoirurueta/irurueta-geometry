/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.estimators.LMedSPointCorrespondenceAffineTransformation3DRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date February 14, 2015
 */
package com.irurueta.geometry.estimators;

import com.irurueta.geometry.AffineTransformation3D;
import com.irurueta.geometry.CoincidentPointsException;
import com.irurueta.geometry.CoordinatesType;
import com.irurueta.geometry.Point3D;
import com.irurueta.numerical.robust.LMedSRobustEstimator;
import com.irurueta.numerical.robust.LMedSRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import java.util.List;

/**
 * Finds the best affine 3D transformation for provided collections of matched
 * 3D points using LMedS algorithm
 */
public class LMedSPointCorrespondenceAffineTransformation3DRobustEstimator 
        extends PointCorrespondenceAffineTransformation3DRobustEstimator{
    
    /**
     * Default value to be used for stop threshold. Stop threshold can be used 
     * to keep the algorithm iterating in case that best estimated threshold 
     * using median of residuals is not small enough. Once a solution is found 
     * that generates a threshold below this value, the algorithm will stop.
     * The stop threshold can be used to prevent the LMedS algorithm iterating
     * too many times in cases where samples have a very similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close 
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would 
     * iterate for a long time trying to find the best solution when indeed 
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in RANSAC, and yet the algorithm could
     * still produce even smaller thresholds in estimated results
     */
    public static final double DEFAULT_STOP_THRESHOLD = 1.0;
    
    /**
     * Minimum allowed stop threshold value
     */
    public static final double MIN_STOP_THRESHOLD = 0.0;

    /**
     * Threshold to be used to keep the algorithm iterating in case that best 
     * estimated threshold using median of residuals is not small enough. Once 
     * a solution is found that generates a threshold below this value, the 
     * algorithm will stop.
     * The stop threshold can be used to prevent the LMedS algorithm iterating
     * too many times in cases where samples have a very similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close 
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would 
     * iterate for a long time trying to find the best solution when indeed 
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in RANSAC, and yet the algorithm could
     * still produce even smaller thresholds in estimated results
     */
    private double mStopThreshold;
 
    /**
     * Constructor
     */
    public LMedSPointCorrespondenceAffineTransformation3DRobustEstimator(){
        super();
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }

    /**
     * Constructor with lists of points to be used to estimate an affine 3D 
     * transformation.
     * Points in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MINIMUM_SIZE
     * @param inputPoints list of input points to be used to estimate an 
     * affine 3D transformation
     * @param outputPoints list of output points to be used to estimate an 
     * affine 3D transformation
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than MINIMUM_SIZE
     */    
    public LMedSPointCorrespondenceAffineTransformation3DRobustEstimator(
            List<Point3D> inputPoints, List<Point3D> outputPoints) 
            throws IllegalArgumentException{
        super(inputPoints, outputPoints);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;        
    }
    
    /**
     * Constructor
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     */
    public LMedSPointCorrespondenceAffineTransformation3DRobustEstimator(
            AffineTransformation3DRobustEstimatorListener listener){
        super(listener);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;        
    }
    
    /**
     * Constructor with listener and lists of points to be used to estimate an
     * affine 3D transformation
     * Points in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MINIMUM_SIZE
     * @param listener listener to be notified of events such as when estimation
     * stars, ends or its progress significantly changes
     * @param inputPoints list of input points to be used to estimate an 
     * affine 3D transformation
     * @param outputPoints list of output points to be used to estimate an 
     * affine 3D transformation
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than MINIMUM_SIZE
     */    
    public LMedSPointCorrespondenceAffineTransformation3DRobustEstimator(
            AffineTransformation3DRobustEstimatorListener listener,
            List<Point3D> inputPoints, List<Point3D> outputPoints) 
            throws IllegalArgumentException{
        super(listener, inputPoints, outputPoints);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;        
    }
    
    /**
     * Returns threshold to be used to keep the algorithm iterating in case that 
     * best estimated threshold using median of residuals is not small enough. 
     * Once a solution is found that generates a threshold below this value, the 
     * algorithm will stop.
     * The stop threshold can be used to prevent the LMedS algorithm iterating
     * too many times in cases where samples have a very similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close 
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would 
     * iterate for a long time trying to find the best solution when indeed 
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in RANSAC, and yet the algorithm could
     * still produce even smaller thresholds in estimated results
     * @return stop threshold to stop the algorithm prematurely when a certain
     * accuracy has been reached
     */
    public double getStopThreshold(){
        return mStopThreshold;
    }
    
    /**
     * Sets threshold to be used to keep the algorithm iterating in case that
     * best estimated threshold using median of residuals is not small enough. 
     * Once a solution is found that generates a threshold below this value, the 
     * algorithm will stop.
     * The stop threshold can be used to prevent the LMedS algorithm iterating
     * too many times in cases where samples have a very similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close 
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would 
     * iterate for a long time trying to find the best solution when indeed 
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in RANSAC, and yet the algorithm could
     * still produce even smaller thresholds in estimated results
     * @param stopThreshold stop threshold to stop the algorithm prematurely 
     * when a certain accuracy has been reached
     * @throws IllegalArgumentException if provided value is zero or negative
     * @throws LockedException if robust estimator is locked because an 
     * estimation is already in progress
     */
    public void setStopThreshold(double stopThreshold) 
            throws IllegalArgumentException, LockedException{
        if(isLocked()) throw new LockedException();
        if(stopThreshold <= MIN_STOP_THRESHOLD) 
            throw new IllegalArgumentException();
        
        mStopThreshold = stopThreshold;
    }
    
    /**
     * Estimates an affine 3D transformation using a robust estimator and
     * the best set of matched 3D point correspondences found using the robust
     * estimator
     * @return an affine 3D transformation
     * @throws LockedException if robust estimator is locked because an 
     * estimation is already in progress
     * @throws NotReadyException if provided input data is not enough to start
     * the estimation
     * @throws RobustEstimatorException if estimation fails for any reason
     * (i.e. numerical instability, no solution available, etc)
     */        
    @Override
    public AffineTransformation3D estimate() throws LockedException, 
            NotReadyException, RobustEstimatorException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }
        
        LMedSRobustEstimator<AffineTransformation3D> innerEstimator =
                new LMedSRobustEstimator<AffineTransformation3D>(
                    new LMedSRobustEstimatorListener<AffineTransformation3D>(){
                        
            //point to be reused when computing residuals
            private Point3D mTestPoint = Point3D.create(
                    CoordinatesType.HOMOGENEOUS_COORDINATES);

            @Override
            public int getTotalSamples() {
                return mInputPoints.size();
            }

            @Override
            public int getSubsetSize() {
                return AffineTransformation3DRobustEstimator.MINIMUM_SIZE;
            }

            @Override
            public void estimatePreliminarSolutions(int[] samplesIndices, 
                    List<AffineTransformation3D> solutions) {
                Point3D inputPoint1 = mInputPoints.get(samplesIndices[0]);                
                Point3D inputPoint2 = mInputPoints.get(samplesIndices[1]);
                Point3D inputPoint3 = mInputPoints.get(samplesIndices[2]);
                Point3D inputPoint4 = mInputPoints.get(samplesIndices[3]);
                
                Point3D outputPoint1 = mOutputPoints.get(samplesIndices[0]);                
                Point3D outputPoint2 = mOutputPoints.get(samplesIndices[1]);
                Point3D outputPoint3 = mOutputPoints.get(samplesIndices[2]);
                Point3D outputPoint4 = mOutputPoints.get(samplesIndices[3]);

                try{
                    AffineTransformation3D transformation = 
                        new AffineTransformation3D(inputPoint1, inputPoint2, 
                        inputPoint3, inputPoint4, outputPoint1, outputPoint2, 
                        outputPoint3, outputPoint4);
                    solutions.add(transformation);
                }catch(CoincidentPointsException e){
                    //if points are coincident, no solution is added
                }
            }

            @Override
            public double computeResidual(
                    AffineTransformation3D currentEstimation, int i) {
                Point3D inputPoint = mInputPoints.get(i);
                Point3D outputPoint = mOutputPoints.get(i);
                
                //transform input point and store result in mTestPoint
                currentEstimation.transform(inputPoint, mTestPoint);
                
                return outputPoint.distanceTo(mTestPoint);
            }

            @Override
            public boolean isReady() {
                return LMedSPointCorrespondenceAffineTransformation3DRobustEstimator.
                        this.isReady();
            }

            @Override
            public void onEstimateStart(
                    RobustEstimator<AffineTransformation3D> estimator) {
                if(mListener != null){
                    mListener.onEstimateStart(LMedSPointCorrespondenceAffineTransformation3DRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateEnd(
                    RobustEstimator<AffineTransformation3D> estimator) {
                if(mListener != null){
                    mListener.onEstimateEnd(LMedSPointCorrespondenceAffineTransformation3DRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateNextIteration(
                    RobustEstimator<AffineTransformation3D> estimator, 
                    int iteration) {
                if(mListener != null){
                    mListener.onEstimateNextIteration(
                            LMedSPointCorrespondenceAffineTransformation3DRobustEstimator.this, 
                            iteration);
                }
            }

            @Override
            public void onEstimateProgressChange(
                    RobustEstimator<AffineTransformation3D> estimator, 
                    float progress) {
                if(mListener != null){
                    mListener.onEstimateProgressChange(
                            LMedSPointCorrespondenceAffineTransformation3DRobustEstimator.this, 
                            progress);
                }
            }
        });
        
        try{
            mLocked = true;
            mInliersData = null;
            innerEstimator.setConfidence(mConfidence);
            innerEstimator.setMaxIterations(mMaxIterations);
            innerEstimator.setProgressDelta(mProgressDelta);
            innerEstimator.setStopThreshold(mStopThreshold);
            AffineTransformation3D transformation = innerEstimator.estimate();
            mInliersData = innerEstimator.getInliersData();
            return attemptRefine(transformation);            
        }catch(com.irurueta.numerical.LockedException e){
            throw new LockedException(e);
        }catch(com.irurueta.numerical.NotReadyException e){
            throw new NotReadyException(e);
        }finally{
            mLocked = false;
        }
    }

    /**
     * Returns method being used for robust estimation
     * @return method being used for robust estimation
     */        
    @Override
    public RobustEstimatorMethod getMethod() {
        return RobustEstimatorMethod.LMedS;
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
    @Override
    protected double getRefinementStandardDeviation() {
        LMedSRobustEstimator.LMedSInliersData inliersData =
                (LMedSRobustEstimator.LMedSInliersData)getInliersData();
        return inliersData.getEstimatedThreshold();
    }    
}