/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.estimator.PinholeCameraEstimatorType
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date February 18, 2013
 */
package com.irurueta.geometry.estimators;

/**
 * Defines types of pinhole camera estimators depending on their algorithm 
 * implementation
 */
public enum PinholeCameraEstimatorType {
    /**
     * DLT (Direct Linear Transform) method using point correspondences
     */
    DLT_POINT_PINHOLE_CAMERA_ESTIMATOR,
    
    /**
     * Weighted method using point correspondences
     */
    WEIGHTED_POINT_PINHOLE_CAMERA_ESTIMATOR,
    
    /**
     * DLT (Direct Linear Transform) method using line/plane correspondences
     */
    DLT_LINE_PLANE_PINHOLE_CAMERA_ESTIMATOR,
    
    /**
     * Weighted method using line/place correspondences
     */
    WEIGHTED_LINE_PLANE_PINHOLE_CAMERA_ESTIMATOR,
    
    /**
     * EPnP (Efficient Perspective-n-Point) method to estimate camera pose using 
     * point correspondences and given intrinsic parameters.
     */
    EPnP_PINHOLE_CAMERA_ESTIMATOR,    
    
    /**
     * UPnP (Uncalibrated Perspective-n-Point) method to estimate camera pose 
     * and focal length using point correspondences.
     */
    UPnP_PINHOLE_CAMERA_ESTIMATOR,
}
