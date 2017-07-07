/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.epipolar.estimators.HomographyDecomposition
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date May 24, 2017.
 */
package com.irurueta.geometry.epipolar.estimators;

import com.irurueta.geometry.EuclideanTransformation3D;

/**
 * Contains decomposition data of an homography.
 */
public class HomographyDecomposition {
    
    /**
     * Length of plane normal vector.
     */
    public static final int PLANE_NORMAL_LENGTH = 3;
    
    /**
     * Contains rotation and translation obtained from homography decomposition.
     */
    private EuclideanTransformation3D mTransformation;
    
    /**
     * Contains computed plane normal obtained from homography decomposition.
     */
    private double[] mPlaneNormal;
    
    /**
     * Distance to computed plane.
     */
    private double mPlaneDistance;
    
    /**
     * Constructor.
     */
    public HomographyDecomposition() { }
    
    /**
     * Constructor.
     * @param transformation transformation containing rotation and translation
     * obtained from homography decomposition.
     * @param planeNormal computed plane normal obtained from homography 
     * decomposition.
     * @param planeDistance distance to computed plane.
     * @throws IllegalArgumentException if provided plane normal does not have
     * length 3.
     */
    public HomographyDecomposition(EuclideanTransformation3D transformation, 
            double[] planeNormal, double planeDistance) 
            throws IllegalArgumentException {
        setPlaneNormal(planeNormal);
        mTransformation = transformation;
        mPlaneDistance = planeDistance;
    }
    
    /**
     * Gets rotation and translation obtained from homography decomposition.
     * @return rotation and translation obtained from homography decomposition.
     */
    public EuclideanTransformation3D getTransformation() {
        return mTransformation;
    }
    
    /**
     * Sets rotation and translation obtained from homography decomposition.
     * @param transformation rotation and translation obtained from homography 
     * decomposition.
     */
    public void setTransformation(EuclideanTransformation3D transformation) {
        mTransformation = transformation;
    }
    
    /**
     * Gets computed plane normal obtained from homography decomposition.
     * @return plane normal obtained from homography decomposition.
     */
    public double[] getPlaneNormal() {
        return mPlaneNormal;
    }
    
    /**
     * Sets computed plane normal obtained from homography decomposition.
     * @param planeNormal plane normal obtained from homography decomposition.
     * @throws IllegalArgumentException if provided plane normal does not have
     * length 3.
     */
    public final void setPlaneNormal(double[] planeNormal) 
            throws IllegalArgumentException {
        if (planeNormal.length != PLANE_NORMAL_LENGTH) {
            throw new IllegalArgumentException();
        }
        mPlaneNormal = planeNormal;        
    }
    
    /**
     * Gets distance to computed plane.
     * @return distance to computed plane.
     */
    public double getPlaneDistance() {
        return mPlaneDistance;
    }
    
    /**
     * Sets distance to computed plane.
     * @param planeDistance distance to computed plane.
     */
    public void setPlaneDistance(double planeDistance) {
        mPlaneDistance = planeDistance;
    }
}
