/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.estimators.LMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date March 4, 2015
 */
package com.irurueta.geometry.estimators;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.geometry.CoincidentLinesException;
import com.irurueta.geometry.Line2D;
import com.irurueta.geometry.ProjectiveTransformation2D;
import com.irurueta.numerical.robust.LMedSRobustEstimator;
import com.irurueta.numerical.robust.LMedSRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import java.util.List;

/**
 * Finds the best projective 2D transformation for provided collections of 
 * matched 2D lines using LMedS algorithm
 */
public class LMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator 
        extends LineCorrespondenceProjectiveTransformation2DRobustEstimator{
        
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
    public static final double DEFAULT_STOP_THRESHOLD = 1e-6;
    
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
    public LMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator(){
        super();
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }

    /**
     * Constructor with lists of lines to be used to estimate a projective 2D
     * transformation.
     * Lines in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greter or equal than MINIMUM_SIZE
     * @param inputLines list of input lines to be used to estimate a projective
     * 2D transformation
     * @param outputLines list of output lines to be used to estimate a 
     * projective 2D transformation
     * @throws IllegalArgumentException if provided lists of lines don't have 
     * the same size or their size is smaller than MINIMUM_SIZE
     */
    public LMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator(
            List<Line2D> inputLines, List<Line2D> outputLines) 
            throws IllegalArgumentException{
        super(inputLines, outputLines);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }
    
    /**
     * Constructor
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     */
    public LMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator(
            ProjectiveTransformation2DRobustEstimatorListener listener){
        super(listener);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }
    
    /**
     * Constructor with listener and lists of lines to be used to estimate a
     * projective 2D transformation.
     * Lines in the list located at the same position are considered to be 
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MINIMUM_SIZE
     * @param listener lsitener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     * @param inputLines list of input lines to be used to estimate a projective
     * 2D transformation
     * @param outputLines list of output lines to be used to estimate a 
     * projective 2D transformation
     * @throws IllegalArgumentException if provided lists of lines don't have
     * the same size or their size is smaller than MINIMUM_SIZE
     */
    public LMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator(
            ProjectiveTransformation2DRobustEstimatorListener listener,
            List<Line2D> inputLines, List<Line2D> outputLines) 
            throws IllegalArgumentException{
        super(listener, inputLines, outputLines);
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
     * Estimates a projective 2D transformation using a robust estimator and
     * the best set of matched 2D lines correspondences found using the robust
     * estimator
     * @return a projective 2D transformation
     * @throws LockedException if robust estimator is locked because an 
     * estimation is already in progress
     * @throws NotReadyException if provided input data is not enough to start
     * the estimation
     * @throws RobustEstimatorException if estimation fails for any reason
     * (i.e. numerical instability, no solution available, etc)
     */        
    @Override
    public ProjectiveTransformation2D estimate() throws LockedException, 
            NotReadyException, RobustEstimatorException {
        if(isLocked()) throw new LockedException();
        if(!isReady()) throw new NotReadyException();
        
        LMedSRobustEstimator<ProjectiveTransformation2D> innerEstimator =
                new LMedSRobustEstimator<ProjectiveTransformation2D>(
                new LMedSRobustEstimatorListener<ProjectiveTransformation2D>(){
                    
            //line to be reused when computing residuals
            private Line2D mTestLine = new Line2D();

            @Override
            public int getTotalSamples() {
                return mInputLines.size();
            }

            @Override
            public int getSubsetSize() {
                return ProjectiveTransformation2DRobustEstimator.MINIMUM_SIZE;
            }

            @Override
            public void estimatePreliminarSolutions(int[] samplesIndices, 
                    List<ProjectiveTransformation2D> solutions) {
                Line2D inputLine1 = mInputLines.get(samplesIndices[0]);
                Line2D inputLine2 = mInputLines.get(samplesIndices[1]);
                Line2D inputLine3 = mInputLines.get(samplesIndices[2]);
                Line2D inputLine4 = mInputLines.get(samplesIndices[3]);
                
                Line2D outputLine1 = mOutputLines.get(samplesIndices[0]);
                Line2D outputLine2 = mOutputLines.get(samplesIndices[1]);
                Line2D outputLine3 = mOutputLines.get(samplesIndices[2]);
                Line2D outputLine4 = mOutputLines.get(samplesIndices[3]);
                
                try{
                    ProjectiveTransformation2D transformation =
                            new ProjectiveTransformation2D(inputLine1, 
                            inputLine2, inputLine3, inputLine4, outputLine1, 
                            outputLine2, outputLine3, outputLine4);
                    solutions.add(transformation);
                }catch(CoincidentLinesException e){
                    //if lines are coincident, no solution is added
                }
            }

            @Override
            public double computeResidual(ProjectiveTransformation2D currentEstimation, int i) {
                Line2D inputLine = mInputLines.get(i);
                Line2D outputLine = mOutputLines.get(i);
                
                //transform input line and store result in mTestLine
                try{
                    currentEstimation.transform(inputLine, mTestLine);
                    
                    return getResidual(outputLine, mTestLine);
                }catch(AlgebraException e){
                    //this happens when internal matrix of affine transformation
                    //cannot be reverse (i.e. transformation is not well defined,
                    //numerical instabilities, etc)
                    return Double.MAX_VALUE;
                }
            }

            @Override
            public boolean isReady() {
                return LMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator.
                        this.isReady();
            }

            @Override
            public void onEstimateStart(
                    RobustEstimator<ProjectiveTransformation2D> estimator) {
                if(mListener != null){
                    mListener.onEstimateStart(
                            LMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateEnd(
                    RobustEstimator<ProjectiveTransformation2D> estimator) {
                if(mListener != null){
                    mListener.onEstimateEnd(
                            LMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateNextIteration(
                    RobustEstimator<ProjectiveTransformation2D> estimator, 
                    int iteration) {
                if(mListener != null){
                    mListener.onEstimateNextIteration(
                            LMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator.this, 
                            iteration);
                }
            }

            @Override
            public void onEstimateProgressChange(
                    RobustEstimator<ProjectiveTransformation2D> estimator, 
                    float progress) {
                if(mListener != null){
                    mListener.onEstimateProgressChange(
                            LMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator.this, 
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
            ProjectiveTransformation2D transformation = 
                    innerEstimator.estimate();
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
