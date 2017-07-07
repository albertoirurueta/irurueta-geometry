/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.CameraType
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date November 4, 2012
 */
package com.irurueta.geometry;

/**
 * Enumeration indicating camera type
 */
public enum CameraType {
    /**
     * Pinhole camera. A pinhole camera is a linear mapping between 3D and 2D
     * worlds. Pinhole cameras only take into account translation, rotation and
     * camera intrinsic parameters such as focal length, aspect ratio, skewness
     * and principal point.
     * Pinhole cameras perform projective mappings between 3D and 2D worlds,
     * in other words, the farther an object is, the smaller is represented or
     * parallel lines converge into vanishing points.
     * Pinhole cameras cannot be used for orthographic projections (where 
     * parallelism between lines is preserved and there are no vanishing points)
     */
    PINHOLE_CAMERA
}
