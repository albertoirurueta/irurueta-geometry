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
 * Subclass of Point3D defining an homogeneous 3D point.
 * An homogeneous 3D point is defined by four coordinates: (x,y,z,w), where
 * x, y and z are the horizontal, vertical and depth coordinates, respectively, 
 * and w is a normalization (homogenization) factor. Homogeneous 3D points at 
 * infinity are expressed using w=0 (x,y,z,0) where (x,y,z) describe the 
 * direction of the 3D point towards infinity.
 * Inhomogeneous 3D points can be transformed into homogeneous 3D points by
 * setting the w coordinate to one (w=1, not at infinity) as follows:
 * Inhomogeneous 3D point (x,y,z) -&lt; Homogeneous 3D point (x,y,z,1).
 */
public class HomogeneousPoint3D extends Point3D implements Serializable {

    /**
     * Default threshold to consider a point is located at infinity.
     */
    private static final double DEFAULT_INFINITY_THRESHOLD = 1e-10;

    /**
     * Machine precision.
     */
    private static final double PRECISION = 1e-12;
    
    /**
     * Defines the X coordinate of an homogeneous 2D point.
     */
    private double mX;
    
    /**
     * Defines the Y coordinate of an homogeneous 2D point.
     */
    private double mY;
    
    /**
     * Defines the Z coordinate of an homogeneous 2D point.
     */
    private double mZ;

    /**
     * Defines the W coordinate of an homogeneous 2D point.
     */
    private double mW;
    
    /**
     * Determines whether this point is already normalized.
     */
    private boolean mNormalized;

    /**
     * Empty constructor.
     */
    public HomogeneousPoint3D() {
        super();
        mX = mY = mZ = 0.0;
        mW = 1.0;
        mNormalized = false;
    }
    
    /**
     * Constructor of this class. This constructor sets a new homogeneous
     * v array containing the coordinates X, Y, Z and W of the given point.
     * @param v Array of length 4 containing the 3D coordinates of an 
     * homogeneous point.
     * @throws IllegalArgumentException Raised when the size of the array is
     * different of 4.
     */
    public HomogeneousPoint3D(double[] v) throws IllegalArgumentException {
        super();
        setCoordinates(v);
    }
    
    /**
     * Constructor of this class. This constructor sets a new homogeneous 3D
     * point using the coordinates X, Y, Z and W of the given point.
     * @param x X coordinate of the given 3D point.
     * @param y Y coordinate of the given 3D point.
     * @param z Z coordinate of the given 3D point.
     * @param w W coordinate of the given 3D point.
     */
    public HomogeneousPoint3D(double x, double y, double z, double w) {
        mX = x;
        mY = y;
        mZ = z;
        mW = w;
        mNormalized = false;
    }
    
    /**
     * This constructor sets a new homogeneous 3D point using as initialization
     * provided Point3D instance.
     * @param point Point to initialize new instance to
     */
    public HomogeneousPoint3D(Point3D point) {
        setCoordinates(point);
    }
    
    /**
     * Returns the X coordinate of the given homogeneous 3D point instance.
     * @return X coordinate.
     */
    public double getX() {
        return mX;
    }
    
    /**
     * Sets the X coordinate of this homogeneous point.
     * @param x X coordinate.
     */
    public void setX(double x) {
        mX = x;
        mNormalized = false;
    }    
    
    /**
     * Returns the Y coordinate of the given homogeneous 3D point instance.
     * @return Y coordinate.
     */
    public double getY() {
        return mY;
    }
    
    /**
     * Sets the Y coordinate of this homogeneous point.
     * @param y Y coordinate.
     */
    public void setY(double y) {
        mY = y;
        mNormalized = false;
    }

    /**
     * Returns the Z coordinate of the given homogeneous 3D point instance.
     * @return Z coordinate.
     */
    public double getZ() {
        return mZ;
    }
    
    /**
     * Sets the Z coordinate of this homogeneous point.
     * @param z Z coordinate.
     */
    public void setZ(double z) {
        mZ = z;
        mNormalized = false;
    }
    
    /**
     * Returns the W coordinate of the given homogeneous 3D point instance.
     * @return W coordinate.
     */
    public double getW() {
        return mW;
    }
    
    /**
     * Sets the W coordinate of this homogeneous point.
     * @param w W coordinate.
     */
    public void setW(double w) {
        mW = w;
        mNormalized = false;
    }    
    
    /**
     * Sets the coordinates of this homogeneous 3D point by using provided X,
     * Y, Z and W coordinates.
     * @param x X coordinate.
     * @param y Y coordinate.
     * @param z Z coordinate.
     * @param w W coordinate.
     */
    public void setCoordinates(double x, double y, double z, double w) {
        mX = x;
        mY = y;
        mZ = z;
        mW = w;
        mNormalized = false;
    }

    /**
     * Sets the coordinates of a 3D point using an array containing its
     * coordinates.
     * @param v Array containing the coordinates of the point.
     * @throws IllegalArgumentException Raised if provided array does not have
     * a valid size.
     */    
    @Override
    public final void setCoordinates(double[] v) 
            throws IllegalArgumentException {
        if (v.length != POINT3D_HOMOGENEOUS_COORDINATES_LENGTH) {
            throw new IllegalArgumentException();
        } else {
            mX = v[0];
            mY = v[1];
            mZ = v[2];
            mW = v[3];
            mNormalized = false;
        }
    }

    /**
     * Sets coordinates of this instance using the coordinates of provided 3D 
     * point.
     * @param point Input point.
     */        
    @Override
    public final void setCoordinates(Point3D point) {
        switch (point.getType()) {
            case INHOMOGENEOUS_COORDINATES:
                InhomogeneousPoint3D inhomPoint = (InhomogeneousPoint3D)point;
                mX = inhomPoint.getX();
                mY = inhomPoint.getY();
                mZ = inhomPoint.getZ();
                mW = 1.0;
                mNormalized = false;
                break;

            case HOMOGENEOUS_COORDINATES:
            default:
                HomogeneousPoint3D homPoint = (HomogeneousPoint3D)point;
                mX = homPoint.getX();
                mY = homPoint.getY();
                mZ = homPoint.getZ();
                mW = homPoint.getW();                
                mNormalized = false;
                break;
        }
    }

    /**
     * Returns X homogeneous coordinate of this 3D point.
     * @return X homogeneous coordinate.
     */        
    @Override
    public double getHomX() {
        return mX;
    }

    /**
     * Returns Y homogeneous coordinate of this 3D point.
     * @return Y homogeneous coordinate.
     */            
    @Override
    public double getHomY() {
        return mY;
    }

    /**
     * Returns Z homogeneous coordinate of this 3D point.
     * @return Z homogeneous coordinate.
     */            
    @Override
    public double getHomZ() {
        return mZ;
    }

    /**
     * Returns W homogeneous coordinate of this 3D point.
     * @return W homogeneous coordinate.
     */            
    @Override
    public double getHomW() {
        return mW;
    }
    
    /**
     * Sets coordinates of this 3D point instance using provided homogeneous
     * coordinates.
     * @param homX x homogeneous coordinate.
     * @param homY y homogeneous coordinate.
     * @param homZ z homogeneous coordinate.
     * @param homW w homogeneous coordinate.
     */        
    @Override
    public void setHomogeneousCoordinates(double homX, double homY, double homZ, 
            double homW) {
        setCoordinates(homX, homY, homZ, homW);
    }
    
    /**
     * Returns X inhomogeneous coordinate of this 3D point.
     * @return X inhomogeneous coordinate.
     */        
    @Override
    public double getInhomX() {
        return (mX / mW);
    }

    /**
     * Sets X inhomogeneous coordinate of this 3D point.
     * @param inhomX inhomogeneous coordinate.
     */
    @Override
    public void setInhomX(double inhomX) {
        mX = inhomX * mW;
        mNormalized = false;
    }

    /**
     * Returns Y inhomogeneous coordinate of this 3D point.
     * @return Y inhomogeneous coordinate.
     */            
    @Override
    public double getInhomY() {
        return (mY / mW);
    }

    /**
     * Sets Y inhomogeneous coordinate of this 3D point.
     * @param inhomY Y inhomogeneous coordinate.
     */
    @Override
    public void setInhomY(double inhomY) {
        mY = inhomY * mW;
        mNormalized = false;
    }

    /**
     * Returns Z inhomogeneous coordinate of this 3D point.
     * @return Z inhomogeneous coordinate.
     */            
    @Override
    public double getInhomZ() {
        return (mZ / mW);
    }

    /**
     * Sets Z inhomogeneous coordinate of this 3D point.
     * @param inhomZ Z inhomogeneous coordinate.
     */
    public void setInhomZ(double inhomZ) {
        mZ = inhomZ * mW;
        mNormalized = false;
    }
    
    /**
     * Sets coordinates of this 3D point instance using provided inhomogeneous
     * coordinates.
     * @param inhomX x inhomogeneous coordinate.
     * @param inhomY y inhomogeneous coordinate.
     * @param inhomZ z inhomogeneous coordinate.
     */        
    @Override
    public void setInhomogeneousCoordinates(double inhomX, double inhomY, 
            double inhomZ) {
        mX = inhomX;
        mY = inhomY;
        mZ = inhomZ;
        mW = 1.0;
        mNormalized = false;
    }
    
    /**
     * Checks if provided object equals current 2D point.
     * @param obj Object to compare.
     * @return True if both objects are considered to be equal, false otherwise.
     */
    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof Point3D)) {
            return false;
        }
        if (obj == this) {
            return true;
        }
        
        Point3D point = (Point3D)obj;
        return equals(point);
    }

    /**
     * Returns hash code value.
     * @return Hash code value.
     */    
    @Override
    public int hashCode() {
        int hash = 7;
        hash = 67 * hash + (int) (Double.doubleToLongBits(this.mX) ^ 
                (Double.doubleToLongBits(this.mX) >>> 32));
        hash = 67 * hash + (int) (Double.doubleToLongBits(this.mY) ^ 
                (Double.doubleToLongBits(this.mY) >>> 32));
        hash = 67 * hash + (int) (Double.doubleToLongBits(this.mZ) ^ 
                (Double.doubleToLongBits(this.mZ) >>> 32));
        hash = 67 * hash + (int) (Double.doubleToLongBits(this.mW) ^ 
                (Double.doubleToLongBits(this.mW) >>> 32));
        return hash;
    }
    
    /**
     * Checks if the homogeneous 3D point described by this instance equals the 
     * input Point3D (using a comparison threshold).
     * @param point Point that will be compared to.
     * @param threshold threshold grade of tolerance to determine whether the
     * points are equal or not. It is used because due to machine precision, the
     * values might not be exactly equal (if not provided 
     * DEFAULT_COMPARISON_THRESHOLD is used).
     * @return True if current point and input point are the same, false 
     * otherwise.
     * @throws IllegalArgumentException Raised if threshold is negative.
     */        
    @Override
    public boolean equals(Point3D point, double threshold) 
            throws IllegalArgumentException {
        switch (point.getType()) {
            case INHOMOGENEOUS_COORDINATES:
                return equals((InhomogeneousPoint3D)point, threshold);
            case HOMOGENEOUS_COORDINATES:
            default:
                return equals((HomogeneousPoint3D)point, threshold);
        }
    }
    
    /**
     * Checks if the homogeneous 3D point described by this instance equals the 
     * input HomogeneousPoint2d (using a comparison threshold).
     * @param point Point that will be compared to.
     * @param threshold threshold grade of tolerance to determine whether the
     * points are equal or not. It is used because due to machine precision, the
     * values might not be exactly equal (if not provided 
     * DEFAULT_COMPARISON_THRESHOLD is used).
     * @return True if current point and input point are the same, false 
     * otherwise.
     * @throws IllegalArgumentException Raised if threshold is negative.
     */
    public boolean equals(HomogeneousPoint3D point, double threshold)
            throws IllegalArgumentException {
        
        if (threshold < MIN_THRESHOLD) {
            throw new IllegalArgumentException();
        }
        
        normalize();
        point.normalize();

        //compute sign for the case when points have different sign
        double signThis = (mW > 0.0) ? 1.0 : -1.0;
        double signPoint = (point.mW > 0.0) ? 1.0 : -1.0;
        
        double normThis = Math.sqrt(mX * mX + mY * mY + mZ * mZ + mW * mW) * 
                signThis;
        double normPoint = Math.sqrt(point.mX * point.mX + point.mY * point.mY +
                point.mZ * point.mZ + point.mW * point.mW) * signPoint;
        
        boolean validX = Math.abs(mX / normThis - point.mX / normPoint) 
                <= threshold;
        boolean validY = Math.abs(mY / normThis - point.mY / normPoint) 
                <= threshold;
        boolean validZ = Math.abs(mZ / normThis - point.mZ / normPoint) 
                <= threshold;
        boolean validW = Math.abs(mW / normThis - point.mW / normPoint) 
                <= threshold;
        
        return (validX && validY && validZ && validW);
    }

    /**
     * Checks if the homogeneous 3D point described by this instance equals the 
     * input HomogeneousPoint3D (using a comparison threshold).
     * @param point Point that will be compared to.
     * @return True if current point and input point are the same, false 
     * otherwise.
     */    
    public boolean equals(HomogeneousPoint3D point) {
       return equals(point, DEFAULT_COMPARISON_THRESHOLD);
    }
    
    /**
     * Checks if the homogeneous 3D point described by this instance equals the 
     * input InhomogeneousPoint3D (using a comparison threshold).
     * @param point Point that will be compared to.
     * @param threshold threshold grade of tolerance to determine whether the
     * points are equal or not. It is used because due to machine precision, the
     * values might not be exactly equal (if not provided 
     * DEFAULT_COMPARISON_THRESHOLD is used).
     * @return True if current point and input point are the same, false 
     * otherwise.
     * @throws IllegalArgumentException Raised if threshold is negative.
     */    
    public boolean equals(InhomogeneousPoint3D point, double threshold)
            throws IllegalArgumentException {
        
        if (threshold < MIN_THRESHOLD) {
            throw new IllegalArgumentException();
        }
        
        boolean dX = Math.abs(point.getX() - (mX / mW)) <= threshold;
        boolean dY = Math.abs(point.getY() - (mY / mW)) <= threshold;
        boolean dZ = Math.abs(point.getZ() - (mZ / mW)) <= threshold;
        return (dX && dY && dZ);
    }

    /**
     * Checks if the homogeneous 3D point described by this instance equals the 
     * input InhomogeneousPoint3D (using a comparison threshold).
     * @param point Point that will be compared to.
     * @return True if current point and input point are the same, false 
     * otherwise.
     */        
    public boolean equals(InhomogeneousPoint3D point) {
        return equals(point, DEFAULT_COMPARISON_THRESHOLD);
    }
    
    /**
     * Checks whether this Point3D is at infinity or not.
     * @return True if the point is at infinity. False otherwise.
     */    
    @Override
    public boolean isAtInfinity() {
        return isAtInfinity(DEFAULT_INFINITY_THRESHOLD);
    }
    
    /**
     * Checks whether this homogeneous 3D point is at infinity or not. An
     * homogeneous 3D point is at infinity when W coordinates are equal or close
     * to zero.
     * @param threshold Grade of tolerance to determine whether the point is at
     * infinity or not. It is used because due to machine precision, the values
     * might not be exactly equal.
     * @return True if point is at infinity, false otherwise.
     * @throws IllegalArgumentException Raised if threshold is negative.
     */
    public boolean isAtInfinity(double threshold) 
            throws IllegalArgumentException {
        if (threshold < MIN_THRESHOLD) {
            throw new IllegalArgumentException();
        }
        
        return (Math.abs(mW) <= threshold);
    }
    
    /**
     * Returns the type of coordinates used to represent a Point3D.
     * @return Type of coordinates of this 3D point.
     */    
    @Override
    public CoordinatesType getType() {
        return CoordinatesType.HOMOGENEOUS_COORDINATES;
    }
    
    /**
     * Method to normalize a 3D point by dividing all homogeneous components by
     * its norm. This only applies to homogeneous 3D points, because they are
     * defined up to scale.
     */
    @Override
    public void normalize() {
        if (!mNormalized) {
            double norm = Math.sqrt(mX * mX + mY * mY + mZ * mZ + mW * mW);
            if (norm > PRECISION) {
                mX /= norm;
                mY /= norm;
                mZ /= norm;
                mW /= norm;
                mNormalized = true;
            }
        }
    } 
    
    /**
     * Returns boolean indicating whether this point has already been normalized.
     * @return True if normalized, false otherwise.
     */
    @Override
    public boolean isNormalized() {
        return mNormalized;
    }
    
    /**
     * Converts this instance into an inhomogeneous 3D point and returns the
     * result as a new inhomogeneous 3D point instance.
     * @return Converts and returns this point as an inhomogeneous 3D point.
     */
    public InhomogeneousPoint3D toInhomogeneous() {
        return new InhomogeneousPoint3D(mX/mW, mY/mW, mZ/mW);
    }
    
    /**
     * Returns an array containing the coordinates of this Point2D.
     * @return Array containing coordinates of this Point2D.
     */    
    @Override
    public double[] asArray() {
        double[] out = new double[POINT3D_HOMOGENEOUS_COORDINATES_LENGTH];
        asArray(out);
        return out;
    } 
    
    /**
     * Uses provided array to store the coordinates of this HomogeneousPoint2D.
     * @param array Array where coordinates will be stored.
     * @throws IllegalArgumentException Raised if length of array is not 3.
     */
    @Override
    public void asArray(double[] array) throws IllegalArgumentException {
        if (array.length != POINT3D_HOMOGENEOUS_COORDINATES_LENGTH) {
            throw new IllegalArgumentException();
        }
        array[0] = mX;
        array[1] = mY;
        array[2] = mZ;
        array[3] = mW;
    }     
}
