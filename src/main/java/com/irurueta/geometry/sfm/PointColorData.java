/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.sfm.ColorData
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date December 28, 2016.
 */
package com.irurueta.geometry.sfm;

import java.io.Serializable;

/**
 * Contains color information for a given point.
 */
public abstract class PointColorData implements Serializable {
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
     * Quality score of color data. The larger the value the higher the
     * quality of color data.
     */
    private double mQualityScore = DEFAULT_QUALITY_SCORE;
    
    /**
     * Constructor.
     */
    public PointColorData() { }
    
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
     * Quality score of color data. The larger the value the higher the quality 
     * of color data.
     * @return quality score.
     */
    public double getQualityScore() {
        return mQualityScore;
    }
    
    /**
     * Sets quality score of color data. The larger the value the higher the 
     * quality of color data.
     * @param qualityScore quality score to be set.
     */
    public void setQualityScore(double qualityScore) {
        mQualityScore = qualityScore;
    }
    
    /**
     * Averages this color data with provided one and stores the result into 
     * provided instance.
     * @param other other instance to average color data with.
     * @param result instance where result of average will be stored.
     */
    public abstract void average(PointColorData other, PointColorData result);
}
