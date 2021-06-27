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
import java.util.Objects;

/**
 * Subclass of Point3D defining an inhomogeneous 3D point.
 * An inhomogeneous 3D point is defined by three coordinates: (x, y, z), where
 * x, y and z are the horizontal, vertical and depth coordinates, respectively.
 * Homogeneous 3D points can be transformed into inhomogeneous ones by
 * normalizing x, y and z coordinates by the w factor as follows:
 * Homogeneous 3D point (x,y,z,w) -&lt; Inhomogeneous 3D point (x/w,y/w,z/w).
 * Note that if and homogeneous 3D point is at infinity (w=0), its corresponding
 * inhomogeneous point will be at infinity by setting (x=inf,y=inf,z=inf).
 * Because of this, and because of machine precision, usually Homogeneous points
 * are better suited when working at far distances or for numerical purposes,
 * and their inhomogeneous counterparts are better suited when computing
 * euclidean distances, etc.
 */
public class InhomogeneousPoint3D extends Point3D implements Serializable {

    /**
     * Defines the X coordinate of an inhomogeneous 2D point.
     */
    private double mX;

    /**
     * Defines the Y coordinate of an inhomogeneous 2D point.
     */
    private double mY;

    /**
     * Defines the Z coordinate of an inhomogeneous 2D point.
     */
    private double mZ;

    /**
     * Empty constructor.
     */
    public InhomogeneousPoint3D() {
        super();
        mX = mY = mZ = 0.0;
    }

    /**
     * Constructor of this class. This constructor sets a new inhomogeneous v
     * array containing the coordinates X, Y and Z of the given point.
     *
     * @param v Array of length 3 containing the 3D coordinates of an
     *          inhomogeneous point.
     * @throws IllegalArgumentException Raised when the size of the array is not
     *                                  2.
     */
    public InhomogeneousPoint3D(final double[] v) {
        super();
        setCoordinates(v);
    }

    /**
     * Constructor of this class. This constructor sets a new inhomogeneous 3D
     * point using the coordinates x, y and z of the given point.
     *
     * @param x X coordinate of the given 3D point.
     * @param y Y coordinate of the given 3D point.
     * @param z Z coordinate of the given 3D point.
     */
    public InhomogeneousPoint3D(final double x, final double y, final double z) {
        mX = x;
        mY = y;
        mZ = z;
    }

    /**
     * This constructor sets a new inhomogeneous 3D point using as
     * initialization provided Point3D instance.
     *
     * @param point Point to initialize new instance to.
     */
    public InhomogeneousPoint3D(final Point3D point) {
        setCoordinates(point);
    }

    /**
     * Returns the X coordinate of the given homogeneous 3D point instance.
     *
     * @return X coordinate.
     */
    public double getX() {
        return mX;
    }

    /**
     * Sets the X coordinate of this homogeneous point.
     *
     * @param x X coordinate.
     */
    public void setX(final double x) {
        mX = x;
    }

    /**
     * Returns the Y coordinate of the given homogeneous 3D point instance.
     *
     * @return Y coordinate.
     */
    public double getY() {
        return mY;
    }

    /**
     * Sets the Y coordinate of this homogeneous point.
     *
     * @param y Y coordinate.
     */
    public void setY(final double y) {
        mY = y;
    }

    /**
     * Returns the Z coordinate of the given homogeneous 3D point instance.
     *
     * @return Z coordinate.
     */
    public double getZ() {
        return mZ;
    }

    /**
     * Sets the Z coordinate of this homogeneous point.
     *
     * @param z Z coordinate.
     */
    public void setZ(final double z) {
        mZ = z;
    }

    /**
     * Sets the coordinates of this inhomogeneous 3D point by using provided X,
     * Y and Z coordinates.
     *
     * @param x X coordinate.
     * @param y Y coordinate.
     * @param z Z coordinate.
     */
    public void setCoordinates(final double x, final double y, final double z) {
        mX = x;
        mY = y;
        mZ = z;
    }

    /**
     * Sets the coordinates of a 3D point using an array containing its
     * coordinates.
     *
     * @param v Array containing the coordinates of the point.
     * @throws IllegalArgumentException Raised if provided array does not have a
     *                                  valid size.
     */
    @Override
    public final void setCoordinates(final double[] v) {
        if (v.length != POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH) {
            throw new IllegalArgumentException();
        } else {
            mX = v[0];
            mY = v[1];
            mZ = v[2];
        }
    }

    /**
     * Sets coordinates of this instance using the coordinates of provided 3D
     * point.
     *
     * @param point Input point.
     */
    @Override
    public final void setCoordinates(final Point3D point) {
        switch (point.getType()) {
            case INHOMOGENEOUS_COORDINATES:
                final InhomogeneousPoint3D inhomPoint = (InhomogeneousPoint3D) point;
                mX = inhomPoint.getX();
                mY = inhomPoint.getY();
                mZ = inhomPoint.getZ();
                break;

            case HOMOGENEOUS_COORDINATES:
            default:
                final HomogeneousPoint3D homPoint = (HomogeneousPoint3D) point;
                mX = homPoint.getInhomX();
                mY = homPoint.getInhomY();
                mZ = homPoint.getInhomZ();
                break;
        }
    }

    /**
     * Returns X homogeneous coordinate of this 3D point.
     *
     * @return X homogeneous coordinate.
     */
    @Override
    public double getHomX() {
        return getX();
    }

    /**
     * Returns Y homogeneous coordinate of this 3D point.
     *
     * @return Y homogeneous coordinate.
     */
    @Override
    public double getHomY() {
        return getY();
    }

    /**
     * Returns Z homogeneous coordinate of this 3D point.
     *
     * @return Z homogeneous coordinate.
     */
    @Override
    public double getHomZ() {
        return getZ();
    }

    /**
     * Returns W homogeneous coordinate of this 3D point.
     *
     * @return W homogeneous coordinate.
     */
    @Override
    public double getHomW() {
        return 1.0;
    }

    /**
     * Sets coordinates of this 3D point instance using provided homogeneous
     * coordinates.
     *
     * @param homX x homogeneous coordinate.
     * @param homY y homogeneous coordinate.
     * @param homZ z homogeneous coordinate.
     * @param homW w homogeneous coordinate.
     */
    @Override
    public void setHomogeneousCoordinates(final double homX, final double homY,
                                          final double homZ, final double homW) {

        mX = homX / homW;
        mY = homY / homW;
        mZ = homZ / homW;
    }

    /**
     * Returns X inhomogeneous coordinate of this 3D point.
     *
     * @return X inhomogeneous coordinate.
     */
    @Override
    public double getInhomX() {
        return getX();
    }

    /**
     * Sets X inhomogeneous coordinate of this 3D point.
     *
     * @param inhomX inhomogeneous coordinate.
     */
    @Override
    public void setInhomX(final double inhomX) {
        mX = inhomX;
    }

    /**
     * Returns Y inhomogeneous coordinate of this 3D point.
     *
     * @return Y inhomogeneous coordinate.
     */
    @Override
    public double getInhomY() {
        return getY();
    }

    /**
     * Sets Y inhomogeneous coordinate of this 3D point.
     *
     * @param inhomY Y inhomogeneous coordinate.
     */
    @Override
    public void setInhomY(final double inhomY) {
        mY = inhomY;
    }

    /**
     * Returns Z inhomogeneous coordinate of this 3D point.
     *
     * @return Z inhomogeneous coordinate.
     */
    @Override
    public double getInhomZ() {
        return getZ();
    }

    /**
     * Sets Z inhomogeneous coordinate of this 3D point.
     *
     * @param inhomZ Z inhomogeneous coordinate.
     */
    @Override
    public void setInhomZ(final double inhomZ) {
        mZ = inhomZ;
    }

    /**
     * Sets coordinates of this 3D point instance using provided inhomogeneous
     * coordinates.
     *
     * @param inhomX x inhomogeneous coordinate.
     * @param inhomY y inhomogeneous coordinate.
     * @param inhomZ z inhomogeneous coordinate.
     */
    @Override
    public void setInhomogeneousCoordinates(
            final double inhomX, final double inhomY, final double inhomZ) {
        setCoordinates(inhomX, inhomY, inhomZ);
    }

    /**
     * Checks if provided object equals current 2D point.
     *
     * @param obj Object to compare.
     * @return True if both objects are considered to be equal, false otherwise.
     */
    @Override
    public boolean equals(final Object obj) {
        if (!(obj instanceof Point3D)) {
            return false;
        }
        if (obj == this) {
            return true;
        }

        final Point3D point = (Point3D) obj;
        return equals(point);
    }

    /**
     * Returns hash code value.
     *
     * @return Hash code value.
     */
    @Override
    public int hashCode() {
        return Objects.hash(mX, mY, mZ, 1.0);
    }

    /**
     * Checks if the homogeneous 3D point described by this instance equals the
     * input Point3D (using a comparison threshold).
     *
     * @param point     Point that will be compared to.
     * @param threshold threshold grade of tolerance to determine whether the
     *                  points are equal or not. It is used because due to machine precision, the
     *                  values might not be exactly equal (if not provided
     *                  DEFAULT_COMPARISON_THRESHOLD is used).
     * @return True if current point and input point are the same, false
     * otherwise.
     * @throws IllegalArgumentException Raised if threshold is negative.
     */
    @Override
    public boolean equals(final Point3D point, final double threshold) {

        switch (point.getType()) {
            case INHOMOGENEOUS_COORDINATES:
                return equals((InhomogeneousPoint3D) point, threshold);
            case HOMOGENEOUS_COORDINATES:
            default:
                return equals((HomogeneousPoint3D) point, threshold);
        }
    }

    /**
     * Checks if the homogeneous 3D point described by this instance equals the
     * input HomogeneousPoint3D (using a comparison threshold).
     *
     * @param point     Point that will be compared to.
     * @param threshold threshold grade of tolerance to determine whether the
     *                  points are equal or not. It is used because due to machine precision, the
     *                  values might not be exactly equal (if not provided
     *                  DEFAULT_COMPARISON_THRESHOLD is used).
     * @return True if current point and input point are the same, false
     * otherwise.
     * @throws IllegalArgumentException Raised if threshold is negative.
     */
    public boolean equals(final HomogeneousPoint3D point, final double threshold) {

        if (threshold < MIN_THRESHOLD) {
            throw new IllegalArgumentException();
        }

        final boolean dX = Math.abs((point.getX() / point.getW()) - mX) <= threshold;
        final boolean dY = Math.abs((point.getY() / point.getW()) - mY) <= threshold;
        final boolean dZ = Math.abs((point.getZ() / point.getW()) - mZ) <= threshold;

        return (dX && dY && dZ);
    }

    /**
     * Checks if the homogeneous 3D point described by this instance equals the
     * input HomogeneousPoint3D (using a comparison threshold).
     *
     * @param point Point that will be compared to.
     * @return True if current point and input point are the same, false
     * otherwise
     */
    public boolean equals(final HomogeneousPoint3D point) {
        return equals(point, DEFAULT_COMPARISON_THRESHOLD);
    }

    /**
     * Checks if the homogeneous 3D point described by this instance equals the
     * input InhomogeneousPoint3D (using a comparison threshold).
     *
     * @param point     Point that will be compared to.
     * @param threshold threshold grade of tolerance to determine whether the
     *                  points are equal or not. It is used because due to machine precision, the
     *                  values might not be exactly equal (if not provided
     *                  DEFAULT_COMPARISON_THRESHOLD is used).
     * @return True if current point and input point are the same, false
     * otherwise.
     * @throws IllegalArgumentException Raised if threshold is negative.
     */
    public boolean equals(final InhomogeneousPoint3D point, double threshold) {

        if (threshold < MIN_THRESHOLD) {
            throw new IllegalArgumentException();
        }

        final boolean dX = Math.abs(point.getX() - mX) <= threshold;
        final boolean dY = Math.abs(point.getY() - mY) <= threshold;
        final boolean dZ = Math.abs(point.getZ() - mZ) <= threshold;

        return (dX && dY && dZ);
    }

    /**
     * Checks if the homogeneous 3D point described by this instance equals the
     * input InhomogeneousPoint3D (using a comparison threshold).
     *
     * @param point Point that will be compared to.
     * @return True if current point and input point are the same, false
     * otherwise.
     */
    public boolean equals(final InhomogeneousPoint3D point) {
        return equals(point, DEFAULT_COMPARISON_THRESHOLD);
    }

    /**
     * Checks whether this Point3D is at infinity or not.
     *
     * @return True if the point is at infinity. False otherwise.
     */
    @Override
    public boolean isAtInfinity() {
        return (Double.isInfinite(mX) || Double.isNaN(mX) ||
                Double.isInfinite(mY) || Double.isNaN(mY) ||
                Double.isInfinite(mZ) || Double.isNaN(mZ));
    }

    /**
     * Returns the type of coordinates used to represent a Point3D.
     *
     * @return Type of coordinates of this 3D point.
     */
    @Override
    public CoordinatesType getType() {
        return CoordinatesType.INHOMOGENEOUS_COORDINATES;
    }

    /**
     * Converts this instance into an homogeneous 3D point and returns the
     * result as a new homogeneous 3D point instance.
     *
     * @return Converts and returns this point as an homogeneous 3D point.
     */
    public HomogeneousPoint3D toHomogeneous() {
        return new HomogeneousPoint3D(mX, mY, mZ, 1.0);
    }

    /**
     * Returns an array containing the coordinates of this Point3D.
     *
     * @return Array containing coordinates of this Point3D.
     */
    @Override
    public double[] asArray() {
        final double[] out = new double[POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
        asArray(out);
        return out;
    }

    /**
     * Uses provided array to store the coordinates of this InhomogeneousPoint3D
     *
     * @param array Array where coordinates will be stored.
     * @throws IllegalArgumentException Raised if length of array is not 2.
     */
    @Override
    public void asArray(double[] array) {
        if (array.length != POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH)
            throw new IllegalArgumentException();
        array[0] = mX;
        array[1] = mY;
        array[2] = mZ;
    }
}
