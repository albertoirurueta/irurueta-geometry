/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.estimators.MSACLineCorrespondenceAffineTransformation2DRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date February 12, 2015
 */
package com.irurueta.geometry.estimators;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.geometry.AffineTransformation2D;
import com.irurueta.geometry.CoincidentLinesException;
import com.irurueta.geometry.Line2D;
import com.irurueta.numerical.robust.MSACRobustEstimator;
import com.irurueta.numerical.robust.MSACRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import java.util.List;

/**
 * Finds the best affine 2D transformation for provided collections of matched
 * 2D lines using MSAC algorithm
 */
public class MSACLineCorrespondenceAffineTransformation2DRobustEstimator 
        extends LineCorrespondenceAffineTransformation2DRobustEstimator{

    /**
     * Constant defining default threshold to determine whether lines are
     * inliers or not. 
     * Residuals to determine whether lines are inliers or not are computed by
     * comparing two lines algebraically (e.g. doing the dot product of their 
     * parameters).
     * A residual of 0 indicates that dot product was 1 or -1 and lines were 
     * equal.
     * A residual of 1 indicates that dot product was 0 and lines were 
     * orthogonal.
     * If dot product between lines is -1, then although their director vectors 
     * are opposed, lines are considered equal, since sign changes are not taken 
     * into account and their residuals will be 0
     */
    public static final double DEFAULT_THRESHOLD = 1e-6;
    
    /**
     * Minimum value that can be set as threshold.
     * Threshold must be strictly greater than 0.0
     */
    public static final double MIN_THRESHOLD = 0.0;
        
    /**
     * Threshold to determine whether lines are inliers or not when testing
     * possible estimation solutions.
     * The threshold refers to the amount of error (i.e. distance and director
     * vector angle difference) a possible solution has on a matched pair of 
     * lines
     */
    private double mThreshold;   
    
    /**
     * Constructor
     */
    public MSACLineCorrespondenceAffineTransformation2DRobustEstimator(){
        super();
        mThreshold = DEFAULT_THRESHOLD;
    }

    /**
     * Constructor with lists of lines to be used to estimate an affine 2D
     * transformation.
     * Lines in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greter or equal than MINIMUM_SIZE
     * @param inputLines list of input lines to be used to estimate an affine
     * 2D transformation
     * @param outputLines list of output lines to be used to estimate an affine
     * 2D transformation
     * @throws IllegalArgumentException if provided lists of lines don't have 
     * the same size or their size is smaller than MINIMUM_SIZE
     */
    public MSACLineCorrespondenceAffineTransformation2DRobustEstimator(
            List<Line2D> inputLines, List<Line2D> outputLines) 
            throws IllegalArgumentException{
        super(inputLines, outputLines);
        mThreshold = DEFAULT_THRESHOLD;
    }
    
    /**
     * Constructor
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     */
    public MSACLineCorrespondenceAffineTransformation2DRobustEstimator(
            AffineTransformation2DRobustEstimatorListener listener){
        super(listener);
        mThreshold = DEFAULT_THRESHOLD;
    }
    
    /**
     * Constructor with listener and lists of lines to be used to estimate an
     * affine 2D transformation.
     * Lines in the list located at the same position are considered to be 
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MINIMUM_SIZE
     * @param listener lsitener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     * @param inputLines list of input lines to be used to estimate an affine
     * 2D transformation
     * @param outputLines list of output lines to be used to estimate an affine
     * 2D transformation
     * @throws IllegalArgumentException if provided lists of lines don't have
     * the same size or their size is smaller than MINIMUM_SIZE
     */
    public MSACLineCorrespondenceAffineTransformation2DRobustEstimator(
            AffineTransformation2DRobustEstimatorListener listener,
            List<Line2D> inputLines, List<Line2D> outputLines) 
            throws IllegalArgumentException{
        super(listener, inputLines, outputLines);
        mThreshold = DEFAULT_THRESHOLD;     
    }
    
    /**
     * Returns threshold to determine whether lines are inliers or not when 
     * testing possible estimation solutions.
     * Residuals to determine whether lines are inliers or not are computed by
     * comparing two lines algebraically (e.g. doing the dot product of their 
     * parameters).
     * A residual of 0 indicates that dot product was 1 or -1 and lines were 
     * equal.
     * A residual of 1 indicates that dot product was 0 and lines were 
     * orthogonal.
     * If dot product between lines is -1, then although their director vectors 
     * are opposed, lines are considered equal, since sign changes are not taken 
     * into account and their residuals will be 0
     * @return threshold to determine whether matched lines are inliers or not
     */
    public double getThreshold(){
        return mThreshold;
    }
    
    /**
     * Sets threshold to determine whether lines are inliers or not when
     * testing possible estimation solutions.
     * Residuals to determine whether lines are inliers or not are computed by
     * comparing two lines algebraically (e.g. doing the dot product of their 
     * parameters).
     * A residual of 0 indicates that dot product was 1 or -1 and lines were 
     * equal.
     * A residual of 1 indicates that dot product was 0 and lines were 
     * orthogonal.
     * If dot product between lines is -1, then although their director vectors 
     * are opposed, lines are considered equal, since sign changes are not taken 
     * into account and their residuals will be 0
     * @param threshold threshold to determine whether matched lines are inliers
     * or not
     * @throws IllegalArgumentException if provided value is equal or less than
     * zero
     * @throws LockedException if robust estimator is locked because an 
     * estimation is already in progress
     */
    public void setThreshold(double threshold) throws IllegalArgumentException, 
            LockedException{
        if(isLocked()) throw new LockedException();
        if(threshold <= MIN_THRESHOLD) throw new IllegalArgumentException();
        mThreshold = threshold;
    }    
    
    /**
     * Estimates an affine 2D transformation using a robust estimator and
     * the best set of matched 2D lines correspondences found using the robust
     * estimator
     * @return an affine 2D transformation
     * @throws LockedException if robust estimator is locked because an 
     * estimation is already in progress
     * @throws NotReadyException if provided input data is not enough to start
     * the estimation
     * @throws RobustEstimatorException if estimation fails for any reason
     * (i.e. numerical instability, no solution available, etc)
     */        
    @Override
    public AffineTransformation2D estimate() throws LockedException, 
            NotReadyException, RobustEstimatorException {
        if(isLocked()) throw new LockedException();
        if(!isReady()) throw new NotReadyException();
        
        MSACRobustEstimator<AffineTransformation2D> innerEstimator =
                new MSACRobustEstimator<AffineTransformation2D>(
                new MSACRobustEstimatorListener<AffineTransformation2D>(){
                    
            //line to be reused when computing residuals
            private Line2D mTestLine = new Line2D();

            @Override
            public double getThreshold() {
                return mThreshold;
            }

            @Override
            public int getTotalSamples() {
                return mInputLines.size();
            }

            @Override
            public int getSubsetSize() {
                return AffineTransformation2DRobustEstimator.MINIMUM_SIZE;
            }

            @Override
            public void estimatePreliminarSolutions(int[] samplesIndices, 
                    List<AffineTransformation2D> solutions) {
                Line2D inputLine1 = mInputLines.get(samplesIndices[0]);
                Line2D inputLine2 = mInputLines.get(samplesIndices[1]);
                Line2D inputLine3 = mInputLines.get(samplesIndices[2]);
                
                Line2D outputLine1 = mOutputLines.get(samplesIndices[0]);
                Line2D outputLine2 = mOutputLines.get(samplesIndices[1]);
                Line2D outputLine3 = mOutputLines.get(samplesIndices[2]);
                
                try{
                    AffineTransformation2D transformation =
                            new AffineTransformation2D(inputLine1, inputLine2, 
                            inputLine3, outputLine1, outputLine2, outputLine3);
                    solutions.add(transformation);
                }catch(CoincidentLinesException e){
                    //if lines are coincident, no solution is added
                }
            }

            @Override
            public double computeResidual(AffineTransformation2D currentEstimation, int i) {
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
                return MSACLineCorrespondenceAffineTransformation2DRobustEstimator.
                        this.isReady();
            }

            @Override
            public void onEstimateStart(
                    RobustEstimator<AffineTransformation2D> estimator) {
                if(mListener != null){
                    mListener.onEstimateStart(
                            MSACLineCorrespondenceAffineTransformation2DRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateEnd(
                    RobustEstimator<AffineTransformation2D> estimator) {
                if(mListener != null){
                    mListener.onEstimateEnd(
                            MSACLineCorrespondenceAffineTransformation2DRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateNextIteration(
                    RobustEstimator<AffineTransformation2D> estimator, 
                    int iteration) {
                if(mListener != null){
                    mListener.onEstimateNextIteration(
                            MSACLineCorrespondenceAffineTransformation2DRobustEstimator.this, 
                            iteration);
                }
            }

            @Override
            public void onEstimateProgressChange(
                    RobustEstimator<AffineTransformation2D> estimator, 
                    float progress) {
                if(mListener != null){
                    mListener.onEstimateProgressChange(
                            MSACLineCorrespondenceAffineTransformation2DRobustEstimator.this, 
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
            AffineTransformation2D transformation = innerEstimator.estimate();
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
        return RobustEstimatorMethod.MSAC;
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
        return mThreshold;
    }
}