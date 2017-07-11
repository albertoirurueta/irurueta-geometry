/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.AffineParameters2D
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date October 28, 2012
 */
package com.irurueta.geometry;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import java.io.Serializable;

/**
 * This class defines additional parameters that can be defined on affine
 * 2D transformations
 */
public class AffineParameters2D implements Serializable{

    /**
     * Horizontal scale
     */
    private double scaleX;
    
    /**
     * Vertical scale
     */
    private double scaleY;
    
    /**
     * Skewness factor. If 0.0, horizontal and vertical axes remain orthogonal,
     * otherwise they get slanted
     */
    private double skewness;
    
    /**
     * Default scale value having no effect on transformations
     */
    public static final double DEFAULT_SCALE = 1.0;
    
    /**
     * Default skewness value having no effect on transformations
     */
    public static final double DEFAULT_SKEWNESS = 0.0;
    
    /**
     * Number of inhomogeneous coordinates in 2D space
     */
    public static final int INHOM_COORDS = 2;
    
    /**
     * Default threshold to determine whether a matrix is a valid 2x2 upper 
     * triangular one.
     */
    public static final double DEFAULT_VALID_THRESHOLD = 1e-8;
    
    /**
     * Constructor.
     * Sets default scale and skewness
     */
    public AffineParameters2D(){
        scaleX = scaleY = DEFAULT_SCALE;
        skewness = DEFAULT_SKEWNESS;
    }
    
    /**
     * Constructor with scale.
     * Sets default skewness and provided scale
     * @param scale Scale to be set (both, horizontal and vertical). A value 
     * between 0.0 and 1.0 will reduce objects size, a value larger than 1.0 
     * will enlarge objects size, and negative values will reverse objects
     */
    public AffineParameters2D(double scale){
        scaleX = scaleY = scale;
        skewness = DEFAULT_SKEWNESS;
    }
    
    /**
     * Constructor with scale and skewness
     * @param scale Scale to be set (both, horizontal and vertical). A value
     * between 0.0 and 1.0 will reduce objects size, a value larger than 1.0
     * will enlarge objects size, and negative values will reverse objects
     * @param skewness Skewness to be set
     */
    public AffineParameters2D(double scale, double skewness){
        scaleX = scaleY = scale;
        this.skewness = skewness;
    }
    
    /**
     * Constructor with horizontal scale, vertical scale and skewness
     * @param scaleX Horizontal scale to be set. A value between 0.0 and 1.0 
     * will reduce objects size, a value larger than 1.0 will enlarge objects
     * size, and negative values will reverse objects
     * @param scaleY Vertical scale to be set. A value between 0.0 and 1.0 will
     * reduce objects size, a value larger than 1.0 will enlarge objects
     * size, and negative values will reverse objects
     * @param skewness Skewness to be set
     */
    public AffineParameters2D(double scaleX, double scaleY, double skewness){
        this.scaleX = scaleX;
        this.scaleY = scaleY;
        this.skewness = skewness;
    }
    
    /**
     * Constructor with matrix and threshold
     * @param m Upper triangular matrix to extract affine parameters from.
     * @param threshold Threshold to determine whether provided matrix is a
     * valid upper triangular one
     * @throws IllegalArgumentException Raised if provided matrix is not 2x2 or
     * if it is not upper triangular, or if threshold is negative
     */
    public AffineParameters2D(Matrix m, double threshold) 
            throws IllegalArgumentException{
        fromMatrix(m, threshold);
    }
    
    /**
     * Constructor with matrix
     * @param m Upper triangular matrix to extract affine parameters from
     * @throws IllegalArgumentException Raised if provided matrix is not 2x2 or
     * if it is not upper triangular
     */
    public AffineParameters2D(Matrix m) throws IllegalArgumentException{
        fromMatrix(m);
    }
    
    /**
     * Returns horizontal scale.
     * A value between 0.0 and 1.0 will reduce objects size, a value larger than
     * 1.0 will enlarge objects size, and negative values will reverse objects
     * @return Horizontal scale
     */
    public double getScaleX(){
        return scaleX;
    }
    
    /**
     * Sets horizontal scale.
     * A value between 0.0 and 1.0 will reduce objects size, a value larger than
     * 1.0 will enlarge objects size, and negative values will reverse objects
     * @param scaleX Horizontal scale to be set
     */
    public void setScaleX(double scaleX){
        this.scaleX = scaleX;
    }
    
    /**
     * Returns vertical scale.
     * A value between 0.0 and 1.0 will reduce objects size, a value larger than
     * 1.0 will enlarge objects size, and negative values will reverse objects
     * @return Vertical scale
     */
    public double getScaleY(){
        return scaleY;
    }
    
    /**
     * Sets vertical scale.
     * A value between 0.0 and 1.0 will reduce objects size, a value larger than
     * 1.0 will enlarge objects size, and negative values will reverse objects
     * @param scaleY Vertical scale to be set
     */
    public void setScaleY(double scaleY){
        this.scaleY = scaleY;
    }
    
    /**
     * Sets overall scale (both, horizontal and vertical).
     * A value between 0.0 and 1.0 will reduce objects size, a value larger than
     * 1.0 will enlarge objects size, and negative values will reverse objects
     * @param scale Horizontal and vertical scale to be set
     */
    public void setScale(double scale){
        scaleX = scaleY = scale;
    }
    
    /**
     * Returns skewness value.
     * A value of 0.0 indicates that horizontal and vertical axis are 
     * orthogonal, otherwise they are slanted to each other
     * @return Skewness value
     */
    public double getSkewness(){
        return skewness;
    }
    
    /**
     * Sets skewness value.
     * A value of 0.0 indicates that horizontal and vertical axis are 
     * orthogonal, otherwise they are slanted to each other.
     * @param skewness Skewness to be set
     */
    public void setSkewness(double skewness){
        this.skewness = skewness;
    }
    
    /**
     * Converts this affine parameters instance into matrix representation
     * @return A matrix representation of this instance
     */
    public Matrix asMatrix(){
        Matrix m = null;
        try{
            m = new Matrix(INHOM_COORDS, INHOM_COORDS);
            asMatrix(m);
        }catch(WrongSizeException ignore){}
        return m;
    }
    
    /**
     * Converts this affine parameters instance into matrix representation and
     * stores the result into provided matrix
     * @param m Matrix where representation of this instance will be stored
     * @throws IllegalArgumentException Raised if provided matrix is not 2x2
     */
    public void asMatrix(Matrix m) throws IllegalArgumentException{
        if(m.getRows() != INHOM_COORDS || m.getColumns() != INHOM_COORDS)
            throw new IllegalArgumentException();
        
        m.setElementAt(0, 0, scaleX);
        m.setElementAt(1, 0, 0.0);
        m.setElementAt(0, 1, skewness);
        m.setElementAt(1, 1, scaleY);
    }
    
    /**
     * Sets parameters of this instance from provided matrix
     * @param m Matrix to set parameters from. Provided matrix must be 2x2 and
     * upper triangular
     * @throws IllegalArgumentException Raised if provided matrix is not 2x2 or
     * if it is not upper triangular
     */
    public final void fromMatrix(Matrix m) throws IllegalArgumentException{
        fromMatrix(m, DEFAULT_VALID_THRESHOLD);
    }
    
    /**
     * Sets parameters of this instance from provided matrix
     * @param m Matrix to set parameters from. Provided matrix must be 2x2 and
     * upper triangular up to provided threshold
     * @param threshold Threshold to determine whether provided matrix is upper
     * triangular. Matrix will be considered upper triangular if its lower 
     * triangular elements are smaller or equal than provided threshold
     * @throws IllegalArgumentException Raised if provided matrix is not 2x2 or
     * it it is not upper triangular or if threshold is negative.
     */
    public final void fromMatrix(Matrix m, double threshold) 
            throws IllegalArgumentException{
        if(!isValidMatrix(m, threshold)) throw new IllegalArgumentException();
        
        scaleX = m.getElementAt(0, 0);
        skewness = m.getElementAt(0, 1);
        scaleY = m.getElementAt(1, 1);
    }
    
    /**
     * Returns boolean indicating whether provided matrix is a valid matrix
     * to set affine parameters from.
     * Valid matrices need to be 2x2 and upper triangular
     * @param m A matrix to determine whether it is valid to set affine 
     * parameters from
     * @return True if matrix is valid, false otherwise
     */
    public static boolean isValidMatrix(Matrix m){
        return isValidMatrix(m, DEFAULT_VALID_THRESHOLD);
    }
    
    /**
     * Returns boolean indicating whether provided matrix is a valid matrix to
     * set affine parameters form.
     * Valid matrices need to be 2x2 and upper triangular up to provided 
     * threshold. In Layman terms, a valid matrix lower triangular elements need
     * to be smaller or equal than provided threshold
     * @param m A matrix to determine whether it is valid to set affine 
     * parameters from
     * @param threshold A threshold to determine whether provided matrix is 
     * upper triangular. Matrix will be considered upper triangular if its lower
     * triangular elements are smaller or equal than provided threshold (without
     * taking into account the sign of the elements)
     * @return True if matrix is valid, false otherwise
     * @throws IllegalArgumentException Raised if provided threshold is negative
     */
    public static boolean isValidMatrix(Matrix m, double threshold) 
            throws IllegalArgumentException{
        if(threshold < 0.0) throw new IllegalArgumentException();
        
        if(m.getRows() != INHOM_COORDS || m.getColumns() != INHOM_COORDS)
            return false;
        
        //check is upper triangular
        int rows = m.getRows();
        int cols = m.getColumns();
        
        for(int v = 0; v < cols; v++){
            for(int u = 0; u < rows; u++){
                if(u > v){
                    if(Math.abs(m.getElementAt(u, v)) > threshold) return false;
                }
            }
        }
        
        return true;
    }
}