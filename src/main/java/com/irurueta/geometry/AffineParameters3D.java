/*
 * Copyright (C) 2012 Alberto Irurueta Carro (alberto@irurueta.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *         http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package com.irurueta.geometry;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;

import java.io.Serializable;

/**
 * This class defines additional parameters that can be defined on affine
 * 3D transformations.
 */
@SuppressWarnings("WeakerAccess")
public class AffineParameters3D implements Serializable {

    /**
     * Default scale value having no effect on transformations.
     */
    public static final double DEFAULT_SCALE = 1.0;

    /**
     * Default skewness value having no effect on transformations.
     */
    public static final double DEFAULT_SKEWNESS = 0.0;

    /**
     * Number of inhomogeneous coordinates in 3D space.
     */
    public static final int INHOM_COORDS = 3;

    /**
     * Default threshold to determine whether a matrix is a valid 3x3 upper
     * triangular one.
     */
    public static final double DEFAULT_VALID_THRESHOLD = 1e-8;

    /**
     * Scale on x coordinates.
     */
    private double scaleX;
    
    /**
     * Scale on y coordinates.
     */
    private double scaleY;
    
    /**
     * Scale on z coordinates.
     */
    private double scaleZ;
    
    /**
     * Skewness factor between axes x-y. If 0.0, x and y axes remain orthogonal,
     * otherwise they get slanted.
     */
    private double skewnessXY;
    
    /**
     * Skewness factor between axes x-z. If 0.0, x and z axes remain orthogonal,
     * otherwise they get slanted.
     */
    private double skewnessXZ;
    
    /**
     * Skewness factor between axes y-z. If 0.0, y and z axes remain orthogonal,
     * otherwise they get slanted.
     */
    private double skewnessYZ;

    /**
     * Constructor.
     * Sets default scale and skewness.
     */
    public AffineParameters3D() {
        scaleX = scaleY = scaleZ = DEFAULT_SCALE;
        skewnessXY = skewnessXZ = skewnessYZ = DEFAULT_SKEWNESS;
    }
    
    /**
     * Constructor with scale.
     * Sets default skewness and provided scale.
     * @param scale Scale to be set (in x, y and z axes). A value between 0.0 
     * and 1.0 will reduce objects size, a value larger than 1.0 will enlarge 
     * objects size, and negative values will reverse objects.
     */
    public AffineParameters3D(double scale) {
        scaleX = scaleY = scaleZ = scale;
        skewnessXY = skewnessXZ = skewnessYZ = DEFAULT_SKEWNESS;
    }
    
    /**
     * Constructor with scale and skewness.
     * @param scale Scale to be set (in x, y and z axes). A value between 0.0 
     * and 1.0 will reduce objects size, a value larger than 1.0 will enlarge 
     * objects size, and negative values will reverse objects.
     * @param skewness Skewness to be set for all axes.
     */
    public AffineParameters3D(double scale, double skewness) {
        scaleX = scaleY = scaleZ = scale;
        skewnessXY = skewnessXZ = skewnessYZ = skewness;
    }
    
    /**
     * Constructor with scale and skewness on each axis.
     * @param scaleX Scale to be set on x axis. A value between 0.0 and 1.0 
     * will reduce objects size, a value larger than 1.0 will enlarge objects
     * size, and negative values will reverse objects.
     * @param scaleY Scale to be set on y axis. A value between 0.0 and 1.0
     * will reduce objects size, a value larger than 1.0 will enlarge objects.
     * @param scaleZ Scale to be set on z axis. A value between 0.0 and 1.0
     * will reduce objects size, a value larger than 1.0 will enlarge objects.
     * @param skewnessXY Skewness to be set for axes x-y.
     * @param skewnessXZ Skewness to be set for axes x-z.
     * @param skewnessYZ Skewness to be set for axes y-z.
     */
    public AffineParameters3D(double scaleX, double scaleY, double scaleZ,
            double skewnessXY, double skewnessXZ, double skewnessYZ) {
        this.scaleX = scaleX;
        this.scaleY = scaleY;
        this.scaleZ = scaleZ;
        this.skewnessXY = skewnessXY;
        this.skewnessXZ = skewnessXZ;
        this.skewnessYZ = skewnessYZ;
    }
    
    /**
     * Constructor with matrix and threshold.
     * @param m Upper triangular matrix to extract affine parameters from..
     * @param threshold Threshold to determine whether provided matrix is a
     * valid upper triangular one.
     * @throws IllegalArgumentException Raised if provided matrix is not 3x3 or
     * if it is not upper triangular, or if threshold is negative.
     */
    public AffineParameters3D(Matrix m, double threshold) {
        fromMatrix(m, threshold);
    }
    
    /**
     * Constructor with matrix.
     * @param m Upper triangular matrix to extract affine parameters from..
     * @throws IllegalArgumentException Raised if provided matrix is not 3x3 or
     * if it is not upper triangular, or if threshold is negative.
     */
    public AffineParameters3D(Matrix m) {
        fromMatrix(m);
    }
    
    /**
     * Returns scale for x axis.
     * A value between 0.0 and 1.0 will reduce objects size, a value larger than
     * 1.0 will enlarge objects size, and negative values will reverse objects.
     * @return Scale for x axis.
     */
    public double getScaleX() {
        return scaleX;
    }
    
    /**
     * Sets scale for x axis.
     * A value between 0.0 and 1.0 will reduce objects size, a value larger than
     * 1.0 will enlarge objects size, and negative values will reverse objects.
     * @param scaleX scale to be set on x axis.
     */
    public void setScaleX(double scaleX) {
        this.scaleX = scaleX;
    }
    
    /**
     * Returns scale for y axis.
     * A value between 0.0 and 1.0 will reduce objects size, a value larger than
     * 1.0 will enlarge objects size, and negative values will reverse objects.
     * @return Scale for y axis.
     */
    public double getScaleY() {
        return scaleY;
    }
    
    /**
     * Sets scale for y axis.
     * A value between 0.0 and 1.0 will reduce objects size, a value larger than
     * 1.0 will enlarge objects size, and negative values will reverse objects.
     * @param scaleY scale to be set on y axis.
     */
    public void setScaleY(double scaleY) {
        this.scaleY = scaleY;
    }

    /**
     * Returns scale for z axis.
     * A value between 0.0 and 1.0 will reduce objects size, a value larger than
     * 1.0 will enlarge objects size, and negative values will reverse objects.
     * @return Scale for z axis.
     */
    public double getScaleZ() {
        return scaleZ;
    }
    
    /**
     * Sets scale for z axis.
     * A value between 0.0 and 1.0 will reduce objects size, a value larger than
     * 1.0 will enlarge objects size, and negative values will reverse objects.
     * @param scaleZ scale to be set on z axis.
     */
    public void setScaleZ(double scaleZ) {
        this.scaleZ = scaleZ;
    }
    
    /**
     * Sets overall scale (for x, y and z axes).
     * A value between 0.0 and 1.0 will reduce objects size, a value larger than
     * 1.0 will enlarge objects size, and negative values will reverse objects.
     * @param scale Scale to be set for x, y and z axes.
     */
    public void setScale(double scale) {
        scaleX = scaleY = scaleZ = scale;
    }
    
    /**
     * Returns skewness value for x-y axes.
     * A value of 0.0 indicates that horizontal and vertical axis are 
     * orthogonal, otherwise they are slanted to each other.
     * @return Skewness value for x-y axes.
     */
    public double getSkewnessXY() {
        return skewnessXY;
    }
    
    /**
     * Sets skewness value for x-y axes.
     * A value of 0.0 indicates that horizontal and vertical axis are 
     * orthogonal, otherwise they are slanted to each other.
     * @param skewnessXY Skewness to be set for x-y axes.
     */
    public void setSkewnessXY(double skewnessXY) {
        this.skewnessXY = skewnessXY;
    }

    /**
     * Returns skewness value for x-z axes.
     * A value of 0.0 indicates that horizontal and vertical axis are 
     * orthogonal, otherwise they are slanted to each other.
     * @return Skewness value for x-z axes.
     */    
    public double getSkewnessXZ() {
        return skewnessXZ;
    }

    /**
     * Sets skewness value for x-z axes.
     * A value of 0.0 indicates that horizontal and vertical axis are 
     * orthogonal, otherwise they are slanted to each other.
     * @param skewnessXZ Skewness to be set for x-z axes.
     */    
    public void setSkewnessXZ(double skewnessXZ) {
        this.skewnessXZ = skewnessXZ;
    }
    
    /**
     * Returns skewness value for y-z axes.
     * A value of 0.0 indicates that horizontal and vertical axis are 
     * orthogonal, otherwise they are slanted to each other.
     * @return Skewness value for y-z axes.
     */    
    public double getSkewnessYZ() {
        return skewnessYZ;
    }
    
    /**
     * Sets skewness value for y-z axes.
     * A value of 0.0 indicates that horizontal and vertical axis are 
     * orthogonal, otherwise they are slanted to each other.
     * @param skewnessYZ Skewness to be set for y-z axes.
     */    
    public void setSkewnessYZ(double skewnessYZ) {
        this.skewnessYZ = skewnessYZ;
    }
    
    /**
     * Converts this affine parameters instance into matrix representation
     * @return A matrix representation of this instance.
     */
    public Matrix asMatrix() {
        Matrix m = null;
        try {
            m = new Matrix(INHOM_COORDS, INHOM_COORDS);
            asMatrix(m);
        } catch (WrongSizeException ignore) {
            //never happens
        }
        return m;
    }
    
    /**
     * Converts this affine parameters instance into matrix representation and
     * stores the result into provided matrix.
     * @param m Matrix where representation of this instance will be stored.
     * @throws IllegalArgumentException Raised if provided matrix is not 3x3.
     */
    public void asMatrix(Matrix m) {
        if (m.getRows() != INHOM_COORDS || m.getColumns() != INHOM_COORDS) {
            throw new IllegalArgumentException();
        }
        
        m.setElementAt(0, 0, scaleX);
        m.setElementAt(1, 0, 0.0);
        m.setElementAt(2, 0, 0.0);
        
        m.setElementAt(0, 1, skewnessXY);
        m.setElementAt(1, 1, scaleY);
        m.setElementAt(2, 1, 0.0);
        
        m.setElementAt(0, 2, skewnessXZ);
        m.setElementAt(1, 2, skewnessYZ);
        m.setElementAt(2, 2, scaleZ);
    }
    
    /**
     * Sets parameters of this instance from provided matrix.
     * @param m Matrix to set parameters from. Provided matrix must be 3x3 and
     * upper triangular.
     * @throws IllegalArgumentException Raised if provided matrix is not 3x3 or
     * if it is not upper triangular.
     */
    public final void fromMatrix(Matrix m) {
        fromMatrix(m, DEFAULT_VALID_THRESHOLD);
    }
    
    /**
     * Sets parameters of this instance from provided matrix.
     * @param m Matrix to set parameters from. Provided matrix must be 3x3 and
     * upper triangular up to provided threshold.
     * @param threshold Threshold to determine whether provided matrix is upper
     * triangular. Matrix will be considered upper triangular if its lower 
     * triangular elements are smaller or equal than provided threshold.
     * @throws IllegalArgumentException Raised if provided matrix is not 3x3 or
     * it it is not upper triangular or if threshold is negative.
     */
    public final void fromMatrix(Matrix m, double threshold) {
        if (!isValidMatrix(m, threshold)) {
            throw new IllegalArgumentException();
        }
        
        scaleX = m.getElementAt(0, 0);
        skewnessXY = m.getElementAt(0, 1);
        scaleY = m.getElementAt(1, 1);
        skewnessXZ = m.getElementAt(0, 2);
        skewnessYZ = m.getElementAt(1, 2);
        scaleZ = m.getElementAt(2, 2);
    }
    
    /**
     * Returns boolean indicating whether provided matrix is a valid matrix
     * to set affine parameters from.
     * Valid matrices need to be 3x3 and upper triangular.
     * @param m A matrix to determine whether it is valid to set affine 
     * parameters from.
     * @return True if matrix is valid, false otherwise.
     */
    public static boolean isValidMatrix(Matrix m) {
        return isValidMatrix(m, DEFAULT_VALID_THRESHOLD);
    }
    
    /**
     * Returns boolean indicating whether provided matrix is a valid matrix to
     * set affine parameters form.
     * Valid matrices need to be 3x3 and upper triangular up to provided 
     * threshold. In Layman terms, a valid matrix lower triangular elements need
     * to be smaller or equal than provided threshold.
     * @param m A matrix to determine whether it is valid to set affine 
     * parameters from.
     * @param threshold A threshold to determine whether provided matrix is 
     * upper triangular. Matrix will be considered upper triangular if its lower
     * triangular elements are smaller or equal than provided threshold (without
     * taking into account the sign of the elements).
     * @return True if matrix is valid, false otherwise.
     * @throws IllegalArgumentException Raised if provided threshold is negative.
     */
    public static boolean isValidMatrix(Matrix m, double threshold) {
        if (threshold < 0.0) {
            throw new IllegalArgumentException();
        }
        
        if (m.getRows() != INHOM_COORDS || m.getColumns() != INHOM_COORDS) {
            return false;
        }
        
        //check is upper triangular
        int rows = m.getRows();
        int cols = m.getColumns();
        
        for (int v = 0; v < cols; v++) {
            for (int u = 0; u < rows; u++) {
                if (u > v && Math.abs(m.getElementAt(u, v)) > threshold) {
                    return false;
                }
            }
        }
        
        return true;
    }    
}
