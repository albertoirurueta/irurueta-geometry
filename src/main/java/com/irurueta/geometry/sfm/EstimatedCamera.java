/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.sfm.EstimatedCamera
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date December 28, 2016.
 */
package com.irurueta.geometry.sfm;

import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.PinholeCamera;
import java.io.Serializable;

/**
 * Contains data of estimated camera.
 */
public class EstimatedCamera implements Serializable {
    
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
     * Estimated camera.
     */
    private PinholeCamera mCamera;
    
    /**
     * Quality score of estimated camera. The larger the value, the better the
     * quality. This is used for robust estimators such as PROSAC or PROMedS.
     * This value is typically obtained during camera estimation.
     */
    private double mQualityScore = DEFAULT_QUALITY_SCORE;
    
    /**
     * Covariance of estimated camera. This can be computed during camera 
     * estimation.
     */
    private Matrix mCovariance;
    
    /**
     * Constructor.
     */
    public EstimatedCamera() { }
    
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
     * Gets estimated camera.
     * @return estimated camera.
     */
    public PinholeCamera getCamera() {
        return mCamera;
    }
    
    /**
     * Sets estimated camera.
     * @param camera estimated camera.
     */
    public void setCamera(PinholeCamera camera) {
        mCamera = camera;
    }
    
    /**
     * Gets quality score of estimated camera. The larger the value, the better 
     * the quality. This is used for robust estimators such as PROSAC or 
     * PROMedS.
     * This value is typically obtained from algorithms determining point
     * correspondences.
     * @return quality score of estimated camera.
     */
    public double getQualityScore() {
        return mQualityScore;
    }
    
    /**
     * Sets quality score of estimated camera. The larger the vaule, the better
     * the quality. This is used for robust estimators such as PROSAC or 
     * PROMedS.
     * This value is typically obtained from algorithms determining point 
     * correspondences.
     * @param qualityScore quality score of estimated camera.
     */
    public void setQualityScore(double qualityScore) {
        mQualityScore = qualityScore;
    }
    
    /**
     * Gets covariance of estimated camera. This can be computed during camera
     * estimation.
     * @return covariance of estimated camera.
     */
    public Matrix getCovariance() {
        return mCovariance;
    }
    
    /**
     * Sets covariance of estimated camera. This can be computed during camera
     * estimation.
     * @param covariance covariance of estimated camera.
     */
    public void setCovariance(Matrix covariance) {
        mCovariance = covariance;
    }    
}
