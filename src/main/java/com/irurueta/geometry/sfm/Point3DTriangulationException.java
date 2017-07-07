/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.epipolar.Point3DTriangulationException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 29, 2015
 */
package com.irurueta.geometry.sfm;

/**
 * Raised if triangulation of 3D points fails for some reason (i.e. degenerate
 * geometry, numerical instabilities, etc)
 */
public class Point3DTriangulationException extends StructureFromMotionException{
    
    /**
     * Constructor
     */
    public Point3DTriangulationException(){
        super();
    }
    
    /**
     * Constructor with String containing message
     * @param message message indicating the cause of the exception
     */
    public Point3DTriangulationException(String message){
        super(message);
    }
    
    /**
     * Constructor with message and cause
     * @param message message describing the cause of the exception
     * @param cause instance containing the cause of the exception
     */
    public Point3DTriangulationException(String message, Throwable cause){
        super(message, cause);
    }
    
    /**
     * Constructor with cause
     * @param cause instance containing the cause of the exception
     */
    public Point3DTriangulationException(Throwable cause){
        super(cause);
    }
}
