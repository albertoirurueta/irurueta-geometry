/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.sfm.ReconstructedPoint3D
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date December 28, 2016.
 */
package com.irurueta.geometry.sfm;

import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.Point3D;

/**
 * Contains data of a reconstructed 3D point.
 */
public class ReconstructedPoint3D {
    
    /**
     * Default quality score value.
     */
    public static final double DEFAULT_QUALITY_SCORE = 1.0;
    
    /**
     * Id to identify this instance. This is useful in case that this data is
     * stored in some sort of database and must be set externally.
     */
    private String mId;
    
    /**
     * Coordinates of reconstructed 3D point.
     */
    private Point3D mPoint;
    
    /**
     * Indicates whether reconstructed point is an inlier or not.
     */
    private boolean mInlier;
    
    /**
     * Quality score of sampled point. The larger the value, the better the 
     * quality. This is used for robust estimators such as PROSAC or PROMedS.
     * This value is typically obtained from algorithms determining point
     * correspondences.
     */
    private double mQualityScore = DEFAULT_QUALITY_SCORE;
    
    /**
     * Covariance of reconstructed point. This can be computed during point 
     * triangularization.
     */
    private Matrix mCovariance;
    
    /**
     * Color data of reconstructed point (i.e. RGB or YUV values), if available.
     */
    private PointColorData mColorData;
        
    /**
     * Construtor.
     */
    public ReconstructedPoint3D() { }
    
    /**
     * Gets id to identify this instance. This is useful in case that this data
     * is stored in some sort of database and must be set externally.
     * @return id to identify this instance.
     */
    public String getId() {
        return mId;
    }
    
    /**
     * Sets id to identify this instance. This is useful in case that this data
     * is stored in some sort of database and must be set externally.
     * @param id id to identify this instance.
     */
    public void setId(String id) {
        mId = id;
    }
    
    /**
     * Gets coordinates of reconstructed 3D point.
     * @return coordinates of reconstructed 3D point.
     */
    public Point3D getPoint() {
        return mPoint;
    }
    
    /**
     * Sets coordinates of reconstructed 3D point.
     * @param point coordinates of reconstructed 3D point.
     */
    public void setPoint(Point3D point) {
        mPoint = point;
    }
    
    /**
     * Indicates whether reconstructed point is an inlier or not.
     * @return true if reconstructed point is an inlier, false otherwise.
     */
    public boolean isInlier() {
        return mInlier;
    }
    
    /**
     * Specifies whether reconstructed point is an inlier or not.
     * @param inlier true if reconstructed point is an inlier, false otherwise.
     */
    public void setInlier(boolean inlier) {
        mInlier = inlier;
    }    

    /**
     * Gets quality score of sampled point. The larger the value, the better the
     * quality. This is used for robust estimators such as PROSAC or PROMedS.
     * This value is typically obtained from algorithms determining point
     * correspondences.
     * @return quality score of sampled point.
     */
    public double getQualityScore() {
        return mQualityScore;
    }
    
    /**
     * Sets quality score of sampled point. The larger the value, the better the
     * quality. This is used for robust estimators such as PROSAC or PROMedS.
     * This value is typically obtained from algorithms determining point
     * correspondences.
     * @param qualityScore quality score of sampled point.
     */
    public void setQualityScore(double qualityScore) {
        mQualityScore = qualityScore;
    }
    
    /**
     * Gets covariance of reconstructed point. This is obtained from the 
     * algorithms determining points of interest or point correspondences.
     * This might be null if covariance cannot be determined.
     * @return covariance of reconstructed point.
     */
    public Matrix getCovariance() {
        return mCovariance;
    }
    
    /**
     * Sets covariance of reconstructed point. This is obtained from the
     * algorithms determining points of interest or point correspondences.
     * @param covariance covariance of reconstructed point.
     */
    public void setCovariance(Matrix covariance) {
        mCovariance = covariance;
    }
    
    /**
     * Gets color data of reconstructed point (i.e. RGB or YUV values), if 
     * available.
     * @return color data of reconstructed point or null.
     */
    public PointColorData getColorData() {
        return mColorData;
    }
    
    /**
     * Sets color data of reconstructed point (i.e. RGB or YUV values), if 
     * available.
     * @param colorData color data of reconstructed point or null.
     */
    public void setColorData(PointColorData colorData) {
        mColorData = colorData;
    }
}
