/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.sfm.Sample2D
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date December 28, 2016.
 */
package com.irurueta.geometry.sfm;

import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.Point2D;
import java.io.Serializable;

/**
 * Contains data of a 2D point sample on a given view.
 */
public class Sample2D implements Serializable {
    
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
     * Id of view where 2D point has been sampled.
     */
    private int mViewId;
    
    /**
     * 2D sampled point coordinates.
     */
    private Point2D mPoint;
    
    /**
     * Quality score of sampled point. The larger the value, the
     * better the quality. This is used for robust estimators such
     * as PROSAC or PROMedS.
     * This value is typically obtained from algorithms determining quality of
     * points of interest.
     */
    private double mQualityScore = DEFAULT_QUALITY_SCORE;
        
    /**
     * Covariance of sampled points. This is obtained from the algorithms
     * determining points of interest or point correspondences.
     * If covariance cannot be determined, a typical value might be to
     * consider 1 pixel accuracy.
     * This might be null if covariance cannot be determined.
     */
    private Matrix mCovariance;
    
    /**
     * Color data of sampled point (i.e. RGB or YUV values), if available.
     */
    private PointColorData mColorData;
    
    /**
     * Constructor.
     */
    public Sample2D() { }
    
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
     * Gets id of view where 2D point has been sampled.
     * @return id of view where 2D point has been sampled.
     */
    public int getViewId() {
        return mViewId;
    }
    
    /**
     * Sets id of view where 2D point has been sampled.
     * @param viewId id of view where 2D point has been sampled.
     */
    public void setViewId(int viewId) {
        mViewId = viewId;
    }
    
    /**
     * Gets 2D sampled point coordinates.
     * @return 2D sampled point coordinates.
     */
    public Point2D getPoint() {
        return mPoint;
    }
    
    /**
     * Sets 2D sampled point coordinates.
     * @param point 2D sampled point coordinates.
     */
    public void setPoint(Point2D point) {
        mPoint = point;
    }

    /**
     * Gets quality score of sampled point. The larger the value, the
     * better the quality. This is used for robust estimators such as
     * PROSAC or PROMEdS.
     * This value is typically obtained from algorithms determining quality of 
     * points of interest.
     * @return quality score of sampled point.
     */
    public double getQualityScore() {
        return mQualityScore;
    }
    
    /**
     * Sets quality score of sampled point. The larger the value, the better
     * the quality. This is used for robust estimators such as PROSAC or 
     * PROMedS.
     * This value is typically obtained from algorithms determining quality of 
     * points of interest.
     * @param qualityScore quality score of sampled point.
     */
    public void setQualityScore(double qualityScore) {
        mQualityScore = qualityScore;
    }

    /**
     * Gets covariance of sampled points. This is obtained from the algorithms
     * determining points of interest or point correspondences.
     * If covariance cannot be determined, a typical value might be to
     * consider 1 pixel accuracy.
     * This might be null if covariance cannot be determined.
     * @return covariance of sampled points or null.
     */
    public Matrix getCovariance() {
        return mCovariance;
    }
    
    /**
     * Sets covariance of sampled points. This is obtained from the algorithms
     * determining points of interest or point correspondences.
     * If covariance cannot be determined, a typical value might be to
     * consider 1 pixel accuracy.
     * This might be null if covariance cannot be determined.
     * @param covariance covariance of sampled points.
     */
    public void setCovariance(Matrix covariance) {
        mCovariance = covariance;
    }

    /**
     * Gets color data of sampled point (i.e. RGB or YUV values), if available.
     * @return color data of sampled point or null.
     */
    public PointColorData getColorData() {
        return mColorData;
    }    
    
    /**
     * Sets color data of sampled point (i.e. RGB or YUV values), if available.
     * @param colorData color data of sampled point or null.
     */
    public void setColorData(PointColorData colorData) {
        mColorData = colorData;
    }
}
