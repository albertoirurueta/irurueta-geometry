/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.estimators.PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date MArch 7, 2015
 */
package com.irurueta.geometry.estimators;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.geometry.CoincidentPlanesException;
import com.irurueta.geometry.Plane;
import com.irurueta.geometry.ProjectiveTransformation3D;
import com.irurueta.numerical.robust.PROMedSRobustEstimator;
import com.irurueta.numerical.robust.PROMedSRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import java.util.List;

/**
 * Finds the best projective 3D transformation for provided collections of
 * matched 3D planes using PROMedS algorithm
 */
public class PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator 
        extends PlaneCorrespondenceProjectiveTransformation3DRobustEstimator{
    
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
     * Quality scores corresponding to each pair of matched planes.
     * The larger the score value the betther the quality of the matching
     */
    private double[] mQualityScores;    
    
    /**
     * Constructor
     */
    public PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(){
        super();
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }

    /**
     * Constructor with lists of planes to be used to estimate an projective 3D
     * transformation.
     * Planes in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greter or equal than MINIMUM_SIZE
     * @param inputPlanes list of input planes to be used to estimate a 
     * projective 3D transformation
     * @param outputPlanes list of output planes to be used to estimate a 
     * projective 3D transformation
     * @throws IllegalArgumentException if provided lists of planes don't have 
     * the same size or their size is smaller than MINIMUM_SIZE
     */
    public PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
            List<Plane> inputPlanes, List<Plane> outputPlanes) 
            throws IllegalArgumentException{
        super(inputPlanes, outputPlanes);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }
    
    /**
     * Constructor
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     */
    public PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
            ProjectiveTransformation3DRobustEstimatorListener listener){
        super(listener);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }
    
    /**
     * Constructor with listener and lists of planes to be used to estimate a
     * projective 3D transformation.
     * Planes in the list located at the same position are considered to be 
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MINIMUM_SIZE
     * @param listener lsitener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     * @param inputPlanes list of input planes to be used to estimate a 
     * projective 3D transformation
     * @param outputPlanes list of output planes to be used to estimate a 
     * projective 3D transformation
     * @throws IllegalArgumentException if provided lists of planes don't have
     * the same size or their size is smaller than MINIMUM_SIZE
     */
    public PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
            ProjectiveTransformation3DRobustEstimatorListener listener,
            List<Plane> inputPlanes, List<Plane> outputPlanes) 
            throws IllegalArgumentException{
        super(listener, inputPlanes, outputPlanes);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;     
    }
    
    /**
     * Constructor
     * @param qualityScores quality scores corresponding to each pair of matched 
     * points.
     * @throws IllegalArgumentException if provided quality scores length is 
     * smaller than MINIMUM_SIZE (i.e. 3 samples)
     */
    public PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
            double[] qualityScores) throws IllegalArgumentException{
        super();
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor with lists of planes to be used to estimate a projective 3D
     * transformation.
     * Planes in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greter or equal than MINIMUM_SIZE
     * @param inputPlanes list of input planes to be used to estimate a 
     * projective 3D transformation
     * @param outputPlanes list of output planes to be used to estimate a 
     * projective 3D transformation
     * @param qualityScores quality scores corresponding to each pair of matched 
     * planes.
     * @throws IllegalArgumentException if provided lists of planes and array
     * of quality scores don't have the same size or their size is smaller than 
     * MINIMUM_SIZE
     */
    public PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
            List<Plane> inputPlanes, List<Plane> outputPlanes, 
            double[] qualityScores) throws IllegalArgumentException{
        super(inputPlanes, outputPlanes);
        
        if(qualityScores.length != inputPlanes.size())
            throw new IllegalArgumentException();
        
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
        internalSetQualityScores(qualityScores);
    }
    
    /**
     * Constructor
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     * @param qualityScores quality scores corresponding to each pair of matched 
     * planes.
     * @throws IllegalArgumentException if provided quality scores length is 
     * smaller than MINIMUM_SIZE (i.e. 3 samples)
     */
    public PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
            ProjectiveTransformation3DRobustEstimatorListener listener,
            double[] qualityScores) throws IllegalArgumentException{
        super(listener);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
        internalSetQualityScores(qualityScores);
    }
    
    /**
     * Constructor with listener and lists of planes to be used to estimate a
     * projective 3D transformation.
     * Planes in the list located at the same position are considered to be 
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MINIMUM_SIZE
     * @param listener lsitener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes
     * @param inputPlanes list of input planes to be used to estimate a 
     * projective 3D transformation
     * @param outputPlanes list of output planes to be used to estimate a 
     * projective 3D transformation
     * @param qualityScores quality scores corresponding to each pair of matched 
     * planes.
     * @throws IllegalArgumentException if provided lists of planes don't have
     * the same size or their size is smaller than MINIMUM_SIZE
     */
    public PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
            ProjectiveTransformation3DRobustEstimatorListener listener,
            List<Plane> inputPlanes, List<Plane> outputPlanes,
            double[] qualityScores) throws IllegalArgumentException{
        super(listener, inputPlanes, outputPlanes);
        
        if(qualityScores.length != inputPlanes.size())
            throw new IllegalArgumentException();
        
        mStopThreshold = DEFAULT_STOP_THRESHOLD;     
        internalSetQualityScores(qualityScores);
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
     * Returns quality scores corresponding to each pair of matched planes.
     * The larger the score value the betther the quality of the matching
     * @return quality scores corresponding to each pair of matched planes
     */
    @Override
    public double[] getQualityScores(){
        return mQualityScores;
    }
    
    /**
     * Sets quality scores corresponding to each pair of matched planes.
     * The larger the score value the better the quality of the matching
     * @param qualityScores quality scores corresponding to each pair of matched
     * planes
     * @throws LockedException if robust estimator is locked because an 
     * estimation is already in progress
     * @throws IllegalArgumentException if provided quality scores length is 
     * smaller than MINIMUM_SIZE (i.e. 3 samples)
     */
    @Override
    public void setQualityScores(double[] qualityScores) throws LockedException,
            IllegalArgumentException{
        if(isLocked()) throw new LockedException();
        internalSetQualityScores(qualityScores);
    }    
    
    /**
     * Indicates if estimator is ready to start the projective 3D transformation
     * estimation.
     * This is true when input data (i.e. lists of matched planes and quality
     * scores) are provided and a minimum of MINIMUM_SIZE lines are available
     * @return true if estimator is ready, false otherwise
     */
    @Override
    public boolean isReady(){
        return super.isReady() && mQualityScores != null && 
                mQualityScores.length == mInputPlanes.size();
    }    
    
    /**
     * Estimates a projective 3D transformation using a robust estimator and
     * the best set of matched 3D planes correspondences found using the robust
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
        
        PROMedSRobustEstimator<ProjectiveTransformation3D> innerEstimator =
                new PROMedSRobustEstimator<ProjectiveTransformation3D>(
                new PROMedSRobustEstimatorListener<ProjectiveTransformation3D>(){
                    
            //plane to be reused when computing residuals
            private Plane mTestPlane = new Plane();

            @Override
            public double getThreshold() {
                return mStopThreshold;
            }

            @Override
            public int getTotalSamples() {
                return mInputPlanes.size();
            }

            @Override
            public int getSubsetSize() {
                return ProjectiveTransformation3DRobustEstimator.MINIMUM_SIZE;
            }

            @Override
            public void estimatePreliminarSolutions(int[] samplesIndices, 
                    List<ProjectiveTransformation3D> solutions) {
                Plane inputPlane1 = mInputPlanes.get(samplesIndices[0]);
                Plane inputPlane2 = mInputPlanes.get(samplesIndices[1]);
                Plane inputPlane3 = mInputPlanes.get(samplesIndices[2]);
                Plane inputPlane4 = mInputPlanes.get(samplesIndices[3]);
                Plane inputPlane5 = mInputPlanes.get(samplesIndices[4]);
                
                Plane outputPlane1 = mOutputPlanes.get(samplesIndices[0]);
                Plane outputPlane2 = mOutputPlanes.get(samplesIndices[1]);
                Plane outputPlane3 = mOutputPlanes.get(samplesIndices[2]);
                Plane outputPlane4 = mOutputPlanes.get(samplesIndices[3]);
                Plane outputPlane5 = mOutputPlanes.get(samplesIndices[4]);
                
                try{
                    ProjectiveTransformation3D transformation =
                            new ProjectiveTransformation3D(inputPlane1, 
                            inputPlane2, inputPlane3, inputPlane4, inputPlane5,
                            outputPlane1, outputPlane2, outputPlane3, 
                            outputPlane4, outputPlane5);
                    solutions.add(transformation);
                }catch(CoincidentPlanesException e){
                    //if lines are coincident, no solution is added
                }
            }

            @Override
            public double computeResidual(
                    ProjectiveTransformation3D currentEstimation, int i) {
                Plane inputPlane = mInputPlanes.get(i);
                Plane outputPlane = mOutputPlanes.get(i);
                
                //transform input plane and store result in mTestPlane
                try{
                    currentEstimation.transform(inputPlane, mTestPlane);
                    
                    return getResidual(outputPlane, mTestPlane);
                }catch(AlgebraException e){
                    //this happens when internal matrix of affine transformation
                    //cannot be reverse (i.e. transformation is not well defined,
                    //numerical instabilities, etc)
                    return Double.MAX_VALUE;
                }
            }

            @Override
            public boolean isReady() {
                return PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                        this.isReady();
            }

            @Override
            public void onEstimateStart(
                    RobustEstimator<ProjectiveTransformation3D> estimator) {
                if(mListener != null){
                    mListener.onEstimateStart(
                            PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateEnd(
                    RobustEstimator<ProjectiveTransformation3D> estimator) {
                if(mListener != null){
                    mListener.onEstimateEnd(
                            PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateNextIteration(
                    RobustEstimator<ProjectiveTransformation3D> estimator, 
                    int iteration) {
                if(mListener != null){
                    mListener.onEstimateNextIteration(
                            PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.this, 
                            iteration);
                }
            }

            @Override
            public void onEstimateProgressChange(
                    RobustEstimator<ProjectiveTransformation3D> estimator, 
                    float progress) {
                if(mListener != null){
                    mListener.onEstimateProgressChange(
                            PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.this, 
                            progress);
                }
            }
            
            @Override
            public double[] getQualityScores() {
                return mQualityScores;
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
        return RobustEstimatorMethod.PROMedS;
    }       
    
    /**
     * Gets standard deviation used for Levenberg-Marquardt fitting during
     * refinement.
     * Returned value gives an indication of how much variance each residual
     * has.
     * Typically this value is related to the threshold used on each robust
     * estimation, since residuals of found inliers are within the range of such
     * threshold.
     * @return standard deviation used for refinement.
     */
    @Override
    protected double getRefinementStandardDeviation() {
        PROMedSRobustEstimator.PROMedSInliersData inliersData =
                (PROMedSRobustEstimator.PROMedSInliersData)getInliersData();
        return inliersData.getEstimatedThreshold();
    }
    
    /**
     * Sets quality scores corresponding to each pair of matched lines.
     * This method is used internally and does not check whether instance is
     * locked or not
     * @param qualityScores quality scores to be set
     * @throws IllegalArgumentException if provided quality scores length is
     * smaller than MINIMUM_SIZE
     */
    private void internalSetQualityScores(double[] qualityScores) 
            throws IllegalArgumentException{
        if(qualityScores.length < MINIMUM_SIZE) 
            throw new IllegalArgumentException();
        
        mQualityScores = qualityScores;        
    }            
    
}
