/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.estimators.Point2DNormalizer
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date February 9, 2015
 */
package com.irurueta.geometry.estimators;

import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.ProjectiveTransformation2D;
import java.util.List;

/**
 * This class takes a collection of points and computes its average 
 * inhomogeneous coordinates and their scale so that a metric transformation is
 * computed to transform points and normalize them.
 * Normalized points are useful in many algorithms because due to the nature
 * of floating point quantization, accuracy of the computations increase for
 * normalized values between -1.0 and 1.0.
 * This implementation uses point inhomogeneous coordinates, hence, it should 
 * not be used for points located at or close to infininity (i.e. very large 
 * inhomogeneous values)
 */
public class Point2DNormalizer {
    
    /**
     * Minimum amount of points required to perform normalization
     */
    public static final int MIN_POINTS = 2;
    
    /**
     * Collection of points used to compute normalization
     */
    private List<Point2D> mPoints;
    
    /**
     * Flag indicating that this instance is locked because computation is
     * in progress
     */
    private boolean mLocked;
    
    /**
     * Minimum x inhomogeneous coordinate found in provided points
     */
    private double mMinInhomX;
    
    /**
     * Minimum y inhomogeneous coordinate found in provided points
     */
    private double mMinInhomY;
    
    /**
     * Maximum x inhomogeneous coordinate found in provided points
     */
    private double mMaxInhomX;
    
    /**
     * Maximum y inhomogeneous coordinate found in provided points
     */
    private double mMaxInhomY;
    
    /**
     * Computed scale on x coordinates to normalize points
     */
    private double mScaleX;
    
    /**
     * Computed scale on y coordinates to normalize points
     */
    private double mScaleY;
    
    /**
     * Computed x coordinate of centroid of points
     */
    private double mCentroidX;
    
    /**
     * Computed y coordinate of centroid of points
     */
    private double mCentroidY;
    
    /**
     * Transformation to normalize points
     */
    private ProjectiveTransformation2D mTransformation;
    
    /**
     * Transformation to denormalize points, which corresponds to the inverse
     * transformation
     */
    private ProjectiveTransformation2D mInverseTransformation;    
    
    /**
     * Constructor
     * @param points collection of points to be used to compute normalization
     * @throws IllegalArgumentException if provided collection of points does
     * not contain enough points, which is MIN_POINTS
     */
    public Point2DNormalizer(List<Point2D> points) 
            throws IllegalArgumentException{
        internalSetPoints(points);
        reset();
    }
    
    /**
     * Returns collection of points used to compute normalization
     * @return collection of points used to compute normalization
     */
    public List<Point2D> getPoints(){
        return mPoints;
    }
    
    /**
     * Sets collection of points used to compute normalization
     * @param points collection of points used to compute normalization
     * @throws LockedException if instance is locked because another computation
     * is already in progress
     * @throws IllegalArgumentException if provided collection of points does
     * not contain enough points, which is MIN_POINTS
     */
    public void setPoints(List<Point2D> points) throws LockedException,
            IllegalArgumentException{
        if(isLocked()) throw new LockedException();        
        internalSetPoints(points);
        reset();
    }    
        
    /**
     * Indicates whether this instance is ready (i.e. has enough data) to
     * start the computation
     * @return true if this instance is ready, false otherwise
     */
    public boolean isReady(){
        return mPoints != null && mPoints.size() >= MIN_POINTS;
    }
    
    /**
     * Indicates whether this instance is locked because computation is
     * in progress.
     * While an instance is in progress, no parameter can be modified and
     * no furter computations can be done until instance becomes unlocked
     * @return true if instance is locked, false otherwise
     */
    public boolean isLocked(){
        return mLocked;
    }
    
    /**
     * Returns minimum x inhomogeneous coordinate found in provided points
     * @return minimum x inhomogeneous coordinate found in provided points
     */
    public double getMinInhomX(){
        return mMinInhomX;
    }
    
    /**
     * Returns minimum y inhomogeneous coordinate found in provided points
     * @return minimum y inhomogeneous coordinate found in provided points
     */
    public double getMinInhomY(){
        return mMinInhomY;
    }
    
    /**
     * Returns maximum x inhomogeneous coordinate found in provided points
     * @return maximum x inhomogeneous coordinate found in provided points
     */
    public double getMaxInhomX(){
        return mMaxInhomX;
    }
    
    /**
     * Returns maximum y inhomogeneous coordinate found in provided points
     * @return maximum y inhomogeneous coordinate found in provided points
     */
    public double getMaxInhomY(){
        return mMaxInhomY;
    }
    
    /**
     * Returns computed scale to normalize points on x coordinate
     * @return computed scale to normalize points on x coordinate
     */
    public double getScaleX(){
        return mScaleX;
    }
    
    /**
     * Returns computed scale to normalize points on y coordinate
     * @return computed scale to normalize points on y coordinate
     */
    public double getScaleY(){
        return mScaleY;
    }
    
    /**
     * Returns computed x coordinate of centroid of points
     * @return computed x coordinate of centroid of points
     */
    public double getCentroidX(){
        return mCentroidX;
    }
    
    /**
     * Returns computed y coordinate of centroid of points
     * @return computed y coordinate of centroid of points
     */
    public double getCentroidY(){
        return mCentroidY;
    }
    
    /**
     * Returns transformation to normalize points
     * @return transformation to normalize points
     */
    public ProjectiveTransformation2D getTransformation(){
        return mTransformation;
    }
    
    /**
     * Returns transformaiton to denormalize points, which corresponds to the
     * inverse transformation
     * @return transformation to denormalize points
     */
    public ProjectiveTransformation2D getInverseTransformation(){
        return mInverseTransformation;
    }
    
    /**
     * Indicates whether result (i.e. transformation and inverse transformation)
     * are available or not
     * @return true if result is available, false otherwise
     */
    public boolean isResultAvailable(){
        return mTransformation != null;
    }
    
    /**
     * Computes normalization and denormalization transformaitons
     * @throws NotReadyException if not enough data has been provided to
     * compute normalization
     * @throws LockedException if instance is locked because another computation
     * is already in progress
     * @throws NormalizerException if normalization failed due to numerical 
     * degeneracy. This usually happens when all provided points are located too
     * close to each other, which results in a singularity when computing proper
     * normalization scale
     */
    public void compute() throws NotReadyException, LockedException,
            NormalizerException{
        if(!isReady()) throw new NotReadyException();
        if(isLocked()) throw new LockedException();
        try{
            mLocked = true;
            
            reset();
            computeLimits();
            
            //compute scale and centroids
            double width = mMaxInhomX - mMinInhomX;
            double height = mMaxInhomY - mMinInhomY;
            
            if(width < Double.MIN_VALUE || height < Double.MIN_VALUE){
                //numerical degeneracy
                throw new NormalizerException();
            }
            
            mScaleX = 1.0 / width;
            mScaleY = 1.0 / height;
            
            //centroids of points
            mCentroidX = (mMinInhomX + mMaxInhomX) / 2.0;
            mCentroidY = (mMinInhomY + mMaxInhomY) / 2.0;
            
            //transformation to normalize points            
            Matrix T = new Matrix(
                    ProjectiveTransformation2D.HOM_COORDS,
                    ProjectiveTransformation2D.HOM_COORDS);

            //X' = s * X + s * t -->
            //s * X = X' - s * t -->
            //X = 1/s*X' - t            
            T.setElementAt(0, 0, mScaleX);
            T.setElementAt(1, 1, mScaleY);
            T.setElementAt(0, 2, -mScaleX * mCentroidX);
            T.setElementAt(1, 2, -mScaleY * mCentroidY);
            T.setElementAt(2, 2, 1.0);
            
            mTransformation = new ProjectiveTransformation2D(T);
            mTransformation.normalize();
            
            //transformation to denormalize points
            Matrix invT = new Matrix(
                    ProjectiveTransformation2D.HOM_COORDS,
                    ProjectiveTransformation2D.HOM_COORDS);
            
            invT.setElementAt(0, 0, width);
            invT.setElementAt(1, 1, height);
            invT.setElementAt(0, 2, mCentroidX);
            invT.setElementAt(1, 2, mCentroidY);
            invT.setElementAt(2, 2, 1.0);
            
            mInverseTransformation = new ProjectiveTransformation2D(invT);
            mInverseTransformation.normalize();
        }catch(Exception e){
            throw new NormalizerException(e);
        }finally{
            mLocked = false;
        }
    }
    
    /**
     * Computes minimum and maximum inhomogeneous point coordinates from the
     * list of provided 2D points
     */
    private void computeLimits(){
        for(Point2D point : mPoints){
            double inhomX = point.getInhomX();
            double inhomY = point.getInhomY();
            if(inhomX < mMinInhomX){
                mMinInhomX = inhomX;
            }
            if(inhomY < mMinInhomY){
                mMinInhomY = inhomY;
            }
            
            if(inhomX > mMaxInhomX){
                mMaxInhomX = inhomX;
            }
            if(inhomY > mMaxInhomY){
                mMaxInhomY = inhomY;
            }
        }
    }
    
    /**
     * Sets list of points
     * @param points list of points to be set
     * @throws IllegalArgumentException if not enough points are provided, which
     * is MIN_POINTS
     */
    private void internalSetPoints(List<Point2D> points) 
            throws IllegalArgumentException{
        if(points.size() < MIN_POINTS) {
            throw new IllegalArgumentException();
        }
        mPoints = points;   
    }
    
    /**
     * Resets internal values
     */
    private void reset(){
        //reset result
        mTransformation = mInverseTransformation = null;
        //reset limits
        mMinInhomX = mMinInhomY = Double.MAX_VALUE;
        mMaxInhomX = mMaxInhomY = -Double.MAX_VALUE;
        mScaleX = mScaleY = 1.0;
        mCentroidX = mCentroidY = 0.0;
    }
}