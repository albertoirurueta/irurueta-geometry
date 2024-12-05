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
 * Subclass of Point2D defining an inhomogeneous 2D point.
 * An inhomogeneous 2D point is defined by two coordinates: (x, y), where x and
 * y are the horizontal and vertical coordinates, respectively.
 * Homogeneous 2D points can be transformed into inhomogeneous ones by
 * normalizing both x and y coordinates by the w factor as follows:
 * Homogeneous 2d point (x,y,w) -&lt; Inhomogeneous 2D point (x/w,y/w).
 * Note that if and homogeneous 2D point is at infinity (w=0), its corresponding
 * inhomogeneous point will be at infinity by setting (x=inf,y=inf). Because of
 * this, and because of machine precision, usually Homogeneous points are better
 * suited when working at far distances or for numerical purposes, and their
 * inhomogeneous counterparts are better suited when computing Euclidean
 * distances, etc.
 */
public class InhomogeneousPoint2D extends Point2D implements Serializable {

    /**
     * Defines the X coordinate of an inhomogeneous 2D point.
     */
    private double x;

    /**
     * Defines the Y coordinate of an inhomogeneous 2D point.
     */
    private double y;

    /**
     * Empty constructor.
     */
    public InhomogeneousPoint2D() {
        super();
        x = y = 0.0;
    }

    /**
     * Constructor of this class. This constructor sets a new inhomogeneous v
     * array containing the coordinates X and Y of the given point.
     *
     * @param v Array of length 2 containing the 2D coordinates of an
     *          inhomogeneous point.
     * @throws IllegalArgumentException Raised when the size of the array is not
     *                                  2.
     */
    public InhomogeneousPoint2D(final double[] v) {
        super();
        setCoordinates(v);
    }

    /**
     * Constructor of this class. This constructor sets a new inhomogeneous 2D
     * point using the coordinates x and y of the given point.
     *
     * @param x X coordinate of the given 2D point.
     * @param y Y coordinate of the given 2D point.
     */
    public InhomogeneousPoint2D(final double x, final double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * This constructor sets a new inhomogeneous 2D point using as
     * initialization provided Point2D instance.
     *
     * @param point Point to initialize new instance to.
     */
    public InhomogeneousPoint2D(final Point2D point) {
        setCoordinates(point);
    }

    /**
     * Returns the X coordinate of the given homogeneous 2D point instance.
     *
     * @return X coordinate.
     */
    public double getX() {
        return x;
    }

    /**
     * Sets the X coordinate of this homogeneous point.
     *
     * @param x X coordinate.
     */
    public void setX(final double x) {
        this.x = x;
    }

    /**
     * Returns the Y coordinate of the given homogeneous 2D point instance.
     *
     * @return Y coordinate.
     */
    public double getY() {
        return y;
    }

    /**
     * Sets the Y coordinate of this homogeneous point.
     *
     * @param y Y coordinate.
     */
    public void setY(final double y) {
        this.y = y;
    }

    /**
     * Sets the coordinates of this inhomogeneous 2D point by using provided X
     * and Y coordinates.
     *
     * @param x X coordinate.
     * @param y Y coordinate.
     */
    public void setCoordinates(final double x, final double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Sets the coordinates of a 2D point using an array containing its
     * coordinates.
     *
     * @param v Array containing the coordinates of the point.
     * @throws IllegalArgumentException Raised if provided array does not have a
     *                                  valid size.
     */
    @Override
    public final void setCoordinates(final double[] v) {
        if (v.length != POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH) {
            throw new IllegalArgumentException();
        } else {
            x = v[0];
            y = v[1];
        }
    }

    /**
     * Sets coordinates of this instance using the coordinates of provided 2D
     * point.
     *
     * @param point Input point.
     */
    @Override
    public final void setCoordinates(final Point2D point) {
        switch (point.getType()) {
            case INHOMOGENEOUS_COORDINATES:
                final var inhomPoint = (InhomogeneousPoint2D) point;
                x = inhomPoint.getX();
                y = inhomPoint.getY();
                break;
            case HOMOGENEOUS_COORDINATES:
            default:
                final var homPoint = (HomogeneousPoint2D) point;
                x = homPoint.getInhomX();
                y = homPoint.getInhomY();
                break;
        }
    }

    /**
     * Returns X homogeneous coordinate of this 2d point.
     *
     * @return X homogeneous coordinate.
     */
    @Override
    public double getHomX() {
        return getX();
    }

    /**
     * Returns Y homogeneous coordinate of this 2d point.
     *
     * @return Y homogeneous coordinate.
     */
    @Override
    public double getHomY() {
        return getY();
    }

    /**
     * Returns W homogeneous coordinate of this 2d point.
     *
     * @return W homogeneous coordinate.
     */
    @Override
    public double getHomW() {
        return 1.0;
    }

    /**
     * Sets coordinates of this 2d point instance using provided homogeneous
     * coordinates.
     *
     * @param homX x homogeneous coordinate.
     * @param homY y homogeneous coordinate.
     * @param homW w homogeneous coordinate.
     */
    @Override
    public void setHomogeneousCoordinates(final double homX, final double homY, final double homW) {
        x = homX / homW;
        y = homY / homW;
    }

    /**
     * Returns X inhomogeneous coordinate of this 2d point.
     *
     * @return X inhomogeneous coordinate.
     */
    @Override
    public double getInhomX() {
        return getX();
    }

    /**
     * Sets X inhomogeneous coordinate of this 2d point.
     *
     * @param inhomX inhomogeneous coordinate.
     */
    @Override
    public void setInhomX(final double inhomX) {
        x = inhomX;
    }

    /**
     * Returns Y inhomogeneous coordinate of this 2d point.
     *
     * @return Y inhomogeneous coordinate.
     */
    @Override
    public double getInhomY() {
        return getY();
    }

    /**
     * Sets Y inhomogeneous coordinate of this 2d point.
     *
     * @param inhomY Y inhomogeneous coordinate.
     */
    @Override
    public void setInhomY(final double inhomY) {
        y = inhomY;
    }

    /**
     * Sets coordinates of this 2d point instance using provided inhomogeneous
     * coordinates.
     *
     * @param inhomX x inhomogeneous coordinate.
     * @param inhomY y inhomogeneous coordinate.
     */
    @Override
    public void setInhomogeneousCoordinates(final double inhomX, final double inhomY) {
        setCoordinates(inhomX, inhomY);
    }

    /**
     * Checks if provided object equals current 2D point.
     *
     * @param obj Object to compare.
     * @return True if both objects are considered to be equal, false otherwise.
     */
    @Override
    public boolean equals(final Object obj) {
        if (!(obj instanceof Point2D point)) {
            return false;
        }
        if (obj == this) {
            return true;
        }

        return equals(point);
    }

    /**
     * Returns hash code value.
     *
     * @return Hash code value.
     */
    @Override
    public int hashCode() {
        return Objects.hash(x, y, 1.0);
    }

    /**
     * Checks if the homogeneous 2d point described by this instance equals the
     * input {@link Point2D} (using a comparison threshold).
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
    public boolean equals(final Point2D point, final double threshold) {
        if (point.getType() == CoordinatesType.INHOMOGENEOUS_COORDINATES) {
            return equals((InhomogeneousPoint2D) point, threshold);
        } else {
            return equals((HomogeneousPoint2D) point, threshold);
        }
    }

    /**
     * Checks if the homogeneous 2d point described by this instance equals the
     * input HomogeneousPoint2d (using a comparison threshold).
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
    public boolean equals(final HomogeneousPoint2D point, final double threshold) {
        if (threshold < MIN_THRESHOLD) {
            throw new IllegalArgumentException();
        }

        final var dX = Math.abs((point.getX() / point.getW()) - x) <= threshold;
        final var dY = Math.abs((point.getY() / point.getW()) - y) <= threshold;

        return (dX && dY);
    }

    /**
     * Checks if the homogeneous 2d point described by this instance equals the
     * input HomogeneousPoint2d (using a comparison threshold).
     *
     * @param point Point that will be compared to.
     * @return True if current point and input point are the same, false
     * otherwise.
     */
    public boolean equals(final HomogeneousPoint2D point) {
        return equals(point, DEFAULT_COMPARISON_THRESHOLD);
    }

    /**
     * Checks if the homogeneous 2d point described by this instance equals the
     * input InhomogeneousPoint2d (using a comparison threshold).
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
    public boolean equals(final InhomogeneousPoint2D point, final double threshold) {
        if (threshold < MIN_THRESHOLD) {
            throw new IllegalArgumentException();
        }

        final var dX = Math.abs(point.getX() - x) <= threshold;
        final var dY = Math.abs(point.getY() - y) <= threshold;

        return (dX && dY);
    }

    /**
     * Checks if the homogeneous 2d point described by this instance equals the
     * input InhomogeneousPoint2d (using a comparison threshold).
     *
     * @param point Point that will be compared to.
     * @return True if current point and input point are the same, false
     * otherwise.
     */
    public boolean equals(final InhomogeneousPoint2D point) {
        return equals(point, DEFAULT_COMPARISON_THRESHOLD);
    }

    /**
     * Checks whether this Point2D is at infinity or not.
     *
     * @return True if the point is at infinity. False otherwise.
     */
    @Override
    public boolean isAtInfinity() {
        return (Double.isInfinite(x) || Double.isNaN(x) || Double.isInfinite(y) || Double.isNaN(y));
    }

    /**
     * Returns the type of coordinates used to represent a Point2D.
     *
     * @return Type of coordinates of this 2d point.
     */
    @Override
    public CoordinatesType getType() {
        return CoordinatesType.INHOMOGENEOUS_COORDINATES;
    }

    /**
     * Converts this instance into an homogeneous 2D point and returns the
     * result as a new homogeneous 2D point instance.
     *
     * @return Converts and returns this point as an homogeneous 2D point.
     */
    public HomogeneousPoint2D toHomogeneous() {
        return new HomogeneousPoint2D(x, y, 1.0);
    }

    /**
     * Returns an array containing the coordinates of this Point2D.
     *
     * @return Array containing coordinates of this Point2D.
     */
    @Override
    public double[] asArray() {
        final var out = new double[POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH];
        asArray(out);
        return out;
    }

    /**
     * Uses provided array to store the coordinates of this InhomogeneousPoint2D
     *
     * @param array Array where coordinates will be stored.
     * @throws IllegalArgumentException Raised if length of array is not 2.
     */
    @Override
    public void asArray(final double[] array) {
        if (array.length != POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH) {
            throw new IllegalArgumentException();
        }
        array[0] = x;
        array[1] = y;
    }
}
