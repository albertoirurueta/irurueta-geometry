/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.estimators.MSACPointCorrespondenceProjectiveTransformation3DRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date March 5, 2015
 */
package com.irurueta.geometry.estimators;

import com.irurueta.geometry.CoincidentPointsException;
import com.irurueta.geometry.CoordinatesType;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.ProjectiveTransformation3D;
import com.irurueta.numerical.robust.MSACRobustEstimator;
import com.irurueta.numerical.robust.MSACRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import java.util.List;

/**
 * Finds the best projective 3D transformation for provided collections of 
 * matched 2D points using MSAC algorithm
 */
public class MSACPointCorrespondenceProjectiveTransformation3DRobustEstimator 
        extends PointCorrespondenceProjectiveTransformation3DRobustEstimator{
    
    /**
     * Constant defining default threshold to determine whether points are 
     * inliers or not.
     * By defaul 1.0 is considered a good value for cases where measures are
     * done on pixels, since typically the minimum resolution is 1 pixel
     */
    public static final double DEFAULT_THRESHOLD = 1.0;
        
    /**
     * Minimum value that can be set as threshold.
     * Threshold must be strictly greater than 0.0
     */
    public static final double MIN_THRESHOLD = 0.0;
    
    /**
     * Threshold to determine whether points are inliers or not when testing
     * possible estimation solutions.
     * The threshold refers to the amount of error (i.e. distance) a possible 
     * solution has on a matched pair of points
     */
    private double mThreshold;            
    
    /**
     * Constructor
     */    
    public MSACPointCorrespondenceProjectiveTransformation3DRobustEstimator(){
        super();
        mThreshold = DEFAULT_THRESHOLD;
    }
    
    /**
     * Constructor with lists of points to be used to estimate a projective 3D 
     * transformation.
     * Points in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MINIMUM_SIZE
     * @param inputPoints list of input points to be used to estimate a 
     * projective 3D transformation
     * @param outputPoints list of output points to be used to estimate a 
     * projective 3D transformation
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than MINIMUM_SIZE
     */    
    public MSACPointCorrespondenceProjectiveTransformation3DRobustEstimator(
            List<Point3D> inputPoints, List<Point3D> outputPoints) 
            throws IllegalArgumentException{
        super(inputPoints, outputPoints);
        mThreshold = DEFAULT_THRESHOLD;
    }
    
    /**
     * Constructor.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     */    
    public MSACPointCorrespondenceProjectiveTransformation3DRobustEstimator(
            ProjectiveTransformation3DRobustEstimatorListener listener){
        super(listener);
        mThreshold = DEFAULT_THRESHOLD;
    }
    
    /**
     * Constructor with listener and lists of points to be used to estimate a
     * projective 3D transformation
     * Points in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MINIMUM_SIZE
     * @param listener listener to be notified of events such as when estimation
     * stars, ends or its progress significantly changes
     * @param inputPoints list of input points to be used to estimate a 
     * projective 3D transformation
     * @param outputPoints list of output points to be used to estimate a 
     * projective 3D transformation
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than MINIMUM_SIZE
     */    
    public MSACPointCorrespondenceProjectiveTransformation3DRobustEstimator(
            ProjectiveTransformation3DRobustEstimatorListener listener,
            List<Point3D> inputPoints, List<Point3D> outputPoints) 
            throws IllegalArgumentException{
        super(listener, inputPoints, outputPoints);
        mThreshold = DEFAULT_THRESHOLD;
    }

    /**
     * Returns threshold to determine whether points are inliers or not when 
     * testing possible estimation solutions.
     * The threshold refers to the amount of error (i.e. euclidean distance) a 
     * possible solution has on a matched pair of points
     * @return threshold to determine whether points are inliers or not when 
     * testing possible estimation solutions
     */
    public double getThreshold(){
        return mThreshold;
    }
    
    /**
     * Sets threshold to determine whether points are inliers or not when 
     * testing possible estimation solutions.
     * Thre threshold refers to the amount of error (i.e. euclidean distance) a
     * possible solution has on a matched pair of points
     * @param threshold threshold to determine whether points are inliers or not
     * @throws IllegalArgumentException if provided values is equal or less than 
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
     * Estimates a projective 3D transformation using a robust estimator and
     * the best set of matched 3D point correspondences found using the robust
     * estimator
     * @return a projective 3D transformation
     * @throws LockedException if robust estimator is locked because an 
     * estimation is already in progress
     * @throws NotReadyException if provided input data is not enough to start
     * the estimation
     * @throws RobustEstimatorException if estimation fails for any reason
     * (i.e. numerical instability, no solution available, etc)
     */    
    @Override
    public ProjectiveTransformation3D estimate() throws LockedException, 
            NotReadyException, RobustEstimatorException {
        if(isLocked()) throw new LockedException();
        if(!isReady()) throw new NotReadyException();
        
        MSACRobustEstimator<ProjectiveTransformation3D> innerEstimator = 
                new MSACRobustEstimator<ProjectiveTransformation3D>(
                    new MSACRobustEstimatorListener<ProjectiveTransformation3D>() {
                    
            //point to be reused when computing residuals
            private Point3D mTestPoint = Point3D.create(
                    CoordinatesType.HOMOGENEOUS_COORDINATES);
            
            @Override
            public double getThreshold() {
                return mThreshold;
            }

            @Override
            public int getTotalSamples() {
                return mInputPoints.size();
            }

            @Override
            public int getSubsetSize() {
                return ProjectiveTransformation3DRobustEstimator.MINIMUM_SIZE;
            }

            @Override
            public void estimatePreliminarSolutions(int[] samplesIndices, 
                    List<ProjectiveTransformation3D> solutions) {
                Point3D inputPoint1 = mInputPoints.get(samplesIndices[0]);                
                Point3D inputPoint2 = mInputPoints.get(samplesIndices[1]);
                Point3D inputPoint3 = mInputPoints.get(samplesIndices[2]);
                Point3D inputPoint4 = mInputPoints.get(samplesIndices[3]);
                Point3D inputPoint5 = mInputPoints.get(samplesIndices[4]);
                
                Point3D outputPoint1 = mOutputPoints.get(samplesIndices[0]);                
                Point3D outputPoint2 = mOutputPoints.get(samplesIndices[1]);
                Point3D outputPoint3 = mOutputPoints.get(samplesIndices[2]);
                Point3D outputPoint4 = mOutputPoints.get(samplesIndices[3]);
                Point3D outputPoint5 = mOutputPoints.get(samplesIndices[4]);

                try{
                    ProjectiveTransformation3D transformation = 
                        new ProjectiveTransformation3D(inputPoint1, inputPoint2, 
                        inputPoint3, inputPoint4, inputPoint5, outputPoint1, 
                        outputPoint2, outputPoint3, outputPoint4, outputPoint5);
                    solutions.add(transformation);
                }catch(CoincidentPointsException e){
                    //if points are coincident, no solution is added
                }
            }

            @Override
            public double computeResidual(
                    ProjectiveTransformation3D currentEstimation, int i) {
                Point3D inputPoint = mInputPoints.get(i);
                Point3D outputPoint = mOutputPoints.get(i);
                
                //transform input point and store result in mTestPoint
                currentEstimation.transform(inputPoint, mTestPoint);
                
                return outputPoint.distanceTo(mTestPoint);
            }

            @Override
            public boolean isReady() {
                return MSACPointCorrespondenceProjectiveTransformation3DRobustEstimator.
                        this.isReady();
            }

            @Override
            public void onEstimateStart(
                    RobustEstimator<ProjectiveTransformation3D> estimator) {
                if(mListener != null){
                    mListener.onEstimateStart(
                            MSACPointCorrespondenceProjectiveTransformation3DRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateEnd(
                    RobustEstimator<ProjectiveTransformation3D> estimator) {
                if(mListener != null){
                    mListener.onEstimateEnd(
                            MSACPointCorrespondenceProjectiveTransformation3DRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateNextIteration(
                    RobustEstimator<ProjectiveTransformation3D> estimator, 
                    int iteration) {
                if(mListener != null){
                    mListener.onEstimateNextIteration(
                            MSACPointCorrespondenceProjectiveTransformation3DRobustEstimator.this, 
                            iteration);
                }
            }

            @Override
            public void onEstimateProgressChange(
                    RobustEstimator<ProjectiveTransformation3D> estimator, 
                    float progress) {
                if(mListener != null){
                    mListener.onEstimateProgressChange(
                            MSACPointCorrespondenceProjectiveTransformation3DRobustEstimator.this, 
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
            ProjectiveTransformation3D transformation = 
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