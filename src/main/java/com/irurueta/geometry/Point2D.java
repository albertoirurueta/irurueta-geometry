/*
 * Copyright (C) 2017 Alberto Irurueta Carro (alberto@irurueta.com)
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

import java.io.Serializable;

/**
 * Abstract class defining the base interface that all 2D points should have.
 * 2D points describe points in a 2D space such as images. They can be 
 * implemented either as homogeneous or inhomogeneous points.
 */
public abstract class Point2D implements Serializable, Point<Point2D> {
    
    /**
     * Defines the threshold used when comparing two values.
     */
    public static final double DEFAULT_COMPARISON_THRESHOLD = 1e-10;
    
    /**
     * Constant defining minimum threshold.
     */
    public static final double MIN_THRESHOLD = 0.0;
    
    /**
     * Length of homogeneous coordinates array.
     */
    public static final int POINT2D_HOMOGENEOUS_COORDINATES_LENGTH = 3;
    
    /**
     * Length of inhomogeneous coordinates array.
     */
    public static final int POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH = 2;
    
    /**
     * Default type of coordinates.
     */
    public static final CoordinatesType DEFAULT_COORDINATES_TYPE =
            CoordinatesType.HOMOGENEOUS_COORDINATES;
    
    
    /**
     * Constructor of this class.
     */
    public Point2D() { }
    
    /**
     * Creates and returns an instance of any existing subclass of Point2D
     * specified in coordinatesType. The right size of the provided array is
     * also checked depending on the type of coordinates used.
     * @param coordinatesType Type of coordinates used.
     * @param v Array containing the coordinates of the 2d homogeneous or
     * inhomogeneous point.
     * @return Created Point2D.
     * @throws IllegalArgumentException Raised if the size of provided array is
     * not valid.
     */
    public static Point2D create(CoordinatesType coordinatesType,
            double[] v) throws IllegalArgumentException {
        switch (coordinatesType) {
            case INHOMOGENEOUS_COORDINATES:
                return new InhomogeneousPoint2D(v);
            case HOMOGENEOUS_COORDINATES:
            default:
                return new HomogeneousPoint2D(v);
        }
    }
    
    /**
     * Creates and returns an instance of any existing subclass of Point2D
     * depending on provided vector length. Size of provided vector is also
     * checked to ensure it has appropriate size to represent 2d points either
     * using inhomogeneous or homogeneous coordinates.
     * @param v Array containing the coordinates of the 2D homogeneous or
     * inhomogeneous 2d point.
     * @return Created Point2D.
     * @throws IllegalArgumentException Raised if the size of provided array
     * is not valid.
     */
    public static Point2D create(double[] v) throws IllegalArgumentException {
        return create(DEFAULT_COORDINATES_TYPE, v);
    }
    
    /**
     * Creates and returns an instance of any existing subclass of Point2D
     * specified in coordinatesType.
     * @param coordinatesType Type of coordinates used.
     * @return Created Point2D.
     */
    public static Point2D create(CoordinatesType coordinatesType) {
        switch (coordinatesType) {
            case INHOMOGENEOUS_COORDINATES:
                return new InhomogeneousPoint2D();
            case HOMOGENEOUS_COORDINATES:
            default:
                return new HomogeneousPoint2D();
        }
    }
    
    /**
     * Creates and returns an instance of an existing subclass of Point2D
     * using DEFAULT_COORDINATES_TYPE.
     * @return Create Point2D.
     */
    public static Point2D create() {
        return create(DEFAULT_COORDINATES_TYPE);
    }
    
    /**
     * Returns an array containing the coordinates of this Point2D.
     * @return Array containing coordinates of this Point2D.
     */
    public abstract double[] asArray();
    
    /**
     * Uses provided array to store the coordinates of this Point2D.
     * @param array Array where coordinates will be stored.
     * @throws IllegalArgumentException Raised if length of array is not valid.
     */
    public abstract void asArray(double[] array) 
            throws IllegalArgumentException;
    
    /**
     * Sets the coordinates of a 2d point using an array containing its
     * coordinates.
     * @param v Array containing the coordinates of the point.
     * @throws IllegalArgumentException Raised if provided array does not have
     * a valid size.
     */
    public abstract void setCoordinates(double[] v) 
            throws IllegalArgumentException;
    
    /**
     * Sets coordinates of this instance using the coordinates of provided 2D 
     * point.
     * @param point Input point.
     */
    public abstract void setCoordinates(Point2D point);
    
    /**
     * Returns X homogeneous coordinate of this 2d point.
     * @return X homogeneous coordinate.
     */
    public abstract double getHomX();
    
    /**
     * Returns Y homogeneous coordinate of this 2d point.
     * @return Y homogeneous coordinate.
     */
    public abstract double getHomY();
    
    /**
     * Returns W homogeneous coordinate of this 2d point.
     * @return W homogeneous coordinate.
     */
    public abstract double getHomW();
    
    /**
     * Sets coordinates of this 2d point instance using provided homogeneous
     * coordinates.
     * @param homX x homogeneous coordinate.
     * @param homY y homogeneous coordinate.
     * @param homW w homogeneous coordinate.
     */
    public abstract void setHomogeneousCoordinates(double homX, double homY, 
            double homW);
    
    /**
     * Returns X inhomogeneous coordinate of this 2d point.
     * @return X inhomogeneous coordinate.
     */
    public abstract double getInhomX();

    /**
     * Sets X inhomogeneous coordinate of this 2d point.
     * @param inhomX X inhomogeneous coordinate.
     */
    public abstract void setInhomX(double inhomX);
    
    /**
     * Returns Y inhomogeneous coordinate of this 2d point.
     * @return Y inhomogeneous coordinate.
     */
    public abstract double getInhomY();

    /**
     * Sets Y inhomogeneous coordinate of this 2d point.
     * @param inhomY Y inhomogeneous coordinate.
     */
    public abstract void setInhomY(double inhomY);
    
    /**
     * Sets coordinates of this 2d point instance using provided inhomogeneous
     * coordinates.
     * @param inhomX x inhomogeneous coordinate.
     * @param inhomY y inhomogeneous coordinate.
     */
    public abstract void setInhomogeneousCoordinates(double inhomX, 
            double inhomY);
    
    /**
     * Checks if the 2d point described by this class equals the input Point2d
     * (using a comparison threshold).
     * @param point Point that will be compared to.
     * @param threshold threshold used to check that the difference of the 
     * values is close to zero with an absolute error defined by threshold.
     * @return True if current point and input point are the same, false 
     * otherwise.
     * @throws IllegalArgumentException Raised if threshold is negative.
     */
    public abstract boolean equals(Point2D point, double threshold)
            throws IllegalArgumentException;
    
    /**
     * Checks if the 2d point described by this class equals the input Point2d
     * (using DEFAULT_COMPARISON_THRESHOLD).
     * @param point Point that will be compared to.
     * @return True if current point and input point are the same, false 
     * otherwise.
     */    
    public boolean equals(Point2D point) {
        return equals(point, DEFAULT_COMPARISON_THRESHOLD);
    }

    /**
     * Checks if provided object equals current 2d point.
     * @param obj Object to compare.
     * @return True if both objects are considered to be equal, false otherwise.
     */
    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof Point2D)) {
            return false;
        }
        if (obj == this) {
            return true;
        }
        
        Point2D point = (Point2D)obj;
        return equals(point);
    }
    
    /**
     * Returns hash code value. This is only defined to keep the compiler happy.
     * This method must be overridden in subclasses of this class.
     * @return Hash code.
     */
    @Override
    public int hashCode() {
        return 3;
    }
    
    /**
     * Checks whether this Point2D is at infinity or not.
     * @return True if the point is at infinity. False otherwise.
     */
    public abstract boolean isAtInfinity();
    
    /**
     * Returns the type of coordinates used to represent a Point2D.
     * @return Type of coordinates of this 2d point.
     */
    public abstract CoordinatesType getType();
    
    /**
     * Method to normalize a 2d point. This only applies to homogeneous
     * 2d points, otherwise it has no effect.
     * This method is meant to be overridden.
     */
    public void normalize() { }
    
    /**
     * Returns boolean indicating whether this point has already been 
     * normalized.
     * This method is meant to be overridden. By default it will always return
     * true, to indicate that no further normalization is possible.
     * @return True if normalized, false otherwise.
     */
    public boolean isNormalized() {
        return true;
    }

    /**
     * Returns number of dimensions of this point implementation.
     * @return number of dimensions.
     */
    @Override
    public int getDimensions() {
        return POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH;
    }

    /**
     * Gets value of inhomogeneous coordinate for provided dimension.
     * @param dim dimension to retrieve coordinate for (i.e. 0 means x, 1 means y, 2 means z, etc).
     * @return value of inhomogeneous coordinate.
     * @throws IllegalArgumentException if provided dimension value is negative or exceeds number of dimensions.
     */
    @Override
    public double getInhomogeneousCoordinate(int dim) throws IllegalArgumentException {
        if (dim < 0 || dim >= getDimensions()) {
            throw new IllegalArgumentException();
        }

        switch (dim) {
            case 0:
                return getInhomX();
            case 1:
            default:
                return getInhomY();
        }
    }

    /**
     * Sets value of inhomogeneous coordinate for provided dimension.
     * @param dim dimension to set coordinate for (i.e. 0 means x, 1 means y, 2 means z, etc).
     * @param value value to be set.
     * @throws IllegalArgumentException if provided dimension value is negative or exceeds number of dimensions.
     */
    @Override
    public void setInhomogeneousCoordinate(int dim, double value) throws IllegalArgumentException {
        if (dim < 0 || dim >= getDimensions()) {
            throw new IllegalArgumentException();
        }

        switch (dim) {
            case 0:
                setInhomX(value);
                break;
            case 1:
                setInhomY(value);
                break;
        }
    }

    /**
     * Returns euclidian distance between this point and provided point.
     * @param point Point to compare.
     * @return Euclidian distance between this point and provided point.
     */
    @Override
    public double distanceTo(Point2D point) {
        return Math.sqrt(sqrDistanceTo(point));
    }

    /**
     * Returns squared euclidean distance between this point and provided point.
     * @param point point to compare.
     * @return euclidean distance between this point and provided point.
     */
    @Override
    public double sqrDistanceTo(Point2D point) {
        double diffX = getInhomX() - point.getInhomX();
        double diffY = getInhomY() - point.getInhomY();

        return diffX * diffX + diffY * diffY;
    }

    
    /**
     * Computes the dot product between the homogeneous coordinates x, y,w of 
     * this point and the ones of provided point.
     * @param point point to compute dot product with.
     * @return dot product value.
     */    
    public double dotProduct(Point2D point) {
        
        double thisHomX = getHomX();
        double thisHomY = getHomY();
        double thisHomW = getHomW();
        double otherHomX = point.getHomX();
        double otherHomY = point.getHomY();
        double otherHomW = point.getHomW();
        
        double thisNormSqr = thisHomX * thisHomX + thisHomY * thisHomY +
                thisHomW * thisHomW;
        double otherNormSqr = otherHomX * otherHomX + otherHomY * otherHomY +
                otherHomW * otherHomW;
        double denom = Math.sqrt(thisNormSqr * otherNormSqr);
        double num = thisHomX * otherHomX + thisHomY * otherHomY +
                thisHomW * otherHomW;
        
        return num / denom;
    }    
    
    /**
     * Returns true if this point is between points point1 and point2, in other
     * words, is inside the segment formed by those 2 points.
     * @param point1 Point 1.
     * @param point2 Point 2.
     * @return True if point is between point1 and point2, false otherwise.
     */
    public boolean isBetween(Point2D point1, Point2D point2) {
        return isBetween(point1, point2, DEFAULT_COMPARISON_THRESHOLD);
    }

    /**
     * Returns true if this point is between points point1 and point2, in other
     * words, is inside the segment formed by those 2 points.
     * @param point1 Point 1.
     * @param point2 Point 2.
     * @param threshold Threshold to determine if point is between.
     * @return True if point is between point1 and point2, false otherwise.
     */    
    public boolean isBetween(Point2D point1, Point2D point2, double threshold)
            throws IllegalArgumentException {
        if (threshold < MIN_THRESHOLD) {
            throw new IllegalArgumentException();
        }
        //If this point is between point1 and point2 then,
        //dist(point1,this) + dist(point2, this) == dist(point1,point2) except
        //for some small difference due to machine precision
        return Math.abs(distanceTo(point1) + distanceTo(point2) - 
                point1.distanceTo(point2)) <= threshold;
    }
}
