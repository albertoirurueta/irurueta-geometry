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
 * Subclass of Point2D defining an homogeneous 2D point.
 * An homogeneous 2d point is defined by three coordinates: (x,y,w), where
 * x and y are the horizontal and vertical coordinates, respectively, and w
 * is a normalization (homogenization) factor. Homogeneous 2d points at
 * infinity are expressed using w=0 (x,y,0) where (x,y) describe the direction
 * of the 2d point towards infinity.
 * Inhomogeneous 2d points can be transformed into homogeneous 2d points by
 * setting the w coordinate to one (w=1, not at infinity) as follows:
 * Inhomogeneous 2d point (x,y) -&lt; Homogeneous 2d point (x,y,1).
 */
public class HomogeneousPoint2D extends Point2D implements Serializable {

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
    public HomogeneousPoint2D() {
        super();
        mX = mY = 0.0;
        mW = 1.0;
        mNormalized = false;
    }

    /**
     * Constructor of this class. This constructor sets a new homogeneous
     * v array containing the coordinates X, Y and W of the given point.
     *
     * @param v Array of length 3 containing the 2D coordinates of an
     *          homogeneous point.
     * @throws IllegalArgumentException Raised when the size of the array is
     *                                  different of 3.
     */
    public HomogeneousPoint2D(final double[] v) {
        super();
        setCoordinates(v);
    }

    /**
     * Constructor of this class. This constructor sets a new homogeneous 2D
     * point using the coordinates X, Y and W of the given point.
     *
     * @param x X coordinate of the given 2D point.
     * @param y Y coordinate of the given 2D point.
     * @param w W coordinate of the given 2D point.
     */
    public HomogeneousPoint2D(final double x, final double y, final double w) {
        mX = x;
        mY = y;
        mW = w;
        mNormalized = false;
    }

    /**
     * This constructor sets a new homogeneous 2D point using as initialization
     * provided Point2D instance.
     *
     * @param point Point to initialize new instance to.
     */
    public HomogeneousPoint2D(final Point2D point) {
        setCoordinates(point);
    }

    /**
     * Returns the X coordinate of the given homogeneous 2D point instance.
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
        mNormalized = false;
    }

    /**
     * Returns the Y coordinate of the given homogeneous 2D point instance.
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
        mNormalized = false;
    }

    /**
     * Returns the W coordinate of the given homogeneous 2D point instance.
     *
     * @return W coordinate.
     */
    public double getW() {
        return mW;
    }

    /**
     * Sets the W coordinate of this homogeneous point.
     *
     * @param w W coordinate.
     */
    public void setW(final double w) {
        mW = w;
        mNormalized = false;
    }

    /**
     * Sets the coordinates of this homogeneous 2D point by using provided X,
     * Y and W coordinates.
     *
     * @param x X coordinate.
     * @param y Y coordinate.
     * @param w W coordinate.
     */
    public void setCoordinates(final double x, final double y, final double w) {
        mX = x;
        mY = y;
        mW = w;
        mNormalized = false;
    }

    /**
     * Sets the coordinates of a 2d point using an array containing its
     * coordinates.
     *
     * @param v Array containing the coordinates of the point.
     * @throws IllegalArgumentException Raised if provided array does not have
     *                                  a valid size.
     */
    @Override
    public final void setCoordinates(final double[] v) {
        if (v.length != POINT2D_HOMOGENEOUS_COORDINATES_LENGTH) {
            throw new IllegalArgumentException();
        } else {
            mX = v[0];
            mY = v[1];
            mW = v[2];
            mNormalized = false;
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
                final InhomogeneousPoint2D inhomPoint = (InhomogeneousPoint2D) point;
                mX = inhomPoint.getX();
                mY = inhomPoint.getY();
                mW = 1.0;
                mNormalized = false;
                break;

            case HOMOGENEOUS_COORDINATES:
            default:
                final HomogeneousPoint2D homPoint = (HomogeneousPoint2D) point;
                mX = homPoint.getX();
                mY = homPoint.getY();
                mW = homPoint.getW();
                mNormalized = false;
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
        return getW();
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
    public void setHomogeneousCoordinates(final double homX, final double homY,
                                          final double homW) {
        setCoordinates(homX, homY, homW);
    }

    /**
     * Returns X inhomogeneous coordinate of this 2d point.
     *
     * @return X inhomogeneous coordinate.
     */
    @Override
    public double getInhomX() {
        return (mX / mW);
    }

    /**
     * Sets X inhomogeneous coordinate of this 2d point.
     *
     * @param inhomX inhomogeneous coordinate.
     */
    @Override
    public void setInhomX(final double inhomX) {
        mX = inhomX * mW;
        mNormalized = false;
    }

    /**
     * Returns Y inhomogeneous coordinate of this 2d point.
     *
     * @return Y inhomogeneous coordinate.
     */
    @Override
    public double getInhomY() {
        return (mY / mW);
    }

    /**
     * Sets Y inhomogeneous coordinate of this 2d point.
     *
     * @param inhomY Y inhomogeneous coordinate.
     */
    @Override
    public void setInhomY(final double inhomY) {
        mY = inhomY * mW;
        mNormalized = false;
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
        mX = inhomX;
        mY = inhomY;
        mW = 1.0;
        mNormalized = false;
    }

    /**
     * Checks if provided object equals current 2D point.
     *
     * @param obj Object to compare.
     * @return True if both objects are considered to be equal, false otherwise.
     */
    @Override
    public boolean equals(final Object obj) {
        if (!(obj instanceof Point2D)) {
            return false;
        }
        if (obj == this) {
            return true;
        }

        final Point2D point = (Point2D) obj;
        return equals(point);
    }

    /**
     * Returns hash code value.
     *
     * @return Hash code value.
     */
    @Override
    public int hashCode() {
        return Objects.hash(mX, mY, mW);
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
        switch (point.getType()) {
            case INHOMOGENEOUS_COORDINATES:
                return equals((InhomogeneousPoint2D) point, threshold);
            case HOMOGENEOUS_COORDINATES:
            default:
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

        normalize();
        point.normalize();

        // compute sign for the case when points have different sign
        final double signThis = (mW > 0.0) ? 1.0 : -1.0;
        final double signPoint = (point.mW > 0.0) ? 1.0 : -1.0;

        final double normThis = Math.sqrt(mX * mX + mY * mY + mW * mW) * signThis;
        final double normPoint = Math.sqrt(point.mX * point.mX + point.mY * point.mY +
                point.mW * point.mW) * signPoint;

        final boolean validX = Math.abs(mX / normThis - point.mX / normPoint)
                <= threshold;
        final boolean validY = Math.abs(mY / normThis - point.mY / normPoint)
                <= threshold;
        final boolean validW = Math.abs(mW / normThis - point.mW / normPoint)
                <= threshold;

        return (validX && validY && validW);
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

        final boolean dX = Math.abs(point.getX() - (mX / mW)) <= threshold;
        final boolean dY = Math.abs(point.getY() - (mY / mW)) <= threshold;
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
        return isAtInfinity(DEFAULT_INFINITY_THRESHOLD);
    }

    /**
     * Checks whether this homogeneous 2D point is at infinity or not. An
     * homogeneous 2D point is at infinity when W coordinates are equal or close
     * to zero.
     *
     * @param threshold Grade of tolerance to determine whether the point is at
     *                  infinity or not. It is used because due to machine precision, the values
     *                  might not be exactly equal.
     * @return True if point is at infinity, false otherwise.
     * @throws IllegalArgumentException Raised if threshold is negative.
     */
    public boolean isAtInfinity(final double threshold) {
        if (threshold < MIN_THRESHOLD) {
            throw new IllegalArgumentException();
        }

        return (Math.abs(mW) <= threshold);
    }

    /**
     * Returns the type of coordinates used to represent a Point2D.
     *
     * @return Type of coordinates of this 2d point.
     */
    @Override
    public CoordinatesType getType() {
        return CoordinatesType.HOMOGENEOUS_COORDINATES;
    }

    /**
     * Method to normalize a 2d point by dividing all homogeneous components by
     * its norm. This only applies to homogeneous 2d points, because they are
     * defined up to scale.
     */
    @SuppressWarnings("DuplicatedCode")
    @Override
    public void normalize() {
        if (!mNormalized) {
            double norm = Math.sqrt(mX * mX + mY * mY + mW * mW);
            if (norm > PRECISION) {
                mX /= norm;
                mY /= norm;
                mW /= norm;
                mNormalized = true;
            }
        }
    }

    /**
     * Returns boolean indicating whether this point has already been mNormalized
     *
     * @return True if mNormalized, false otherwise.
     */
    @Override
    public boolean isNormalized() {
        return mNormalized;
    }

    /**
     * Converts this instance into an inhomogeneous 2D point and returns the
     * result as a new inhomogeneous 2D point instance.
     *
     * @return Converts and returns this point as an inhomogeneous 2D point.
     */
    public InhomogeneousPoint2D toInhomogeneous() {
        return new InhomogeneousPoint2D(mX / mW, mY / mW);
    }

    /**
     * Returns an array containing the coordinates of this Point2D.
     *
     * @return Array containing coordinates of this Point2D.
     */
    @Override
    public double[] asArray() {
        double[] out = new double[POINT2D_HOMOGENEOUS_COORDINATES_LENGTH];
        asArray(out);
        return out;
    }

    /**
     * Uses provided array to store the coordinates of this HomogeneousPoint2D
     *
     * @param array Array where coordinates will be stored.
     * @throws IllegalArgumentException Raised if length of array is not 3.
     */
    @Override
    public void asArray(double[] array) {
        if (array.length != POINT2D_HOMOGENEOUS_COORDINATES_LENGTH) {
            throw new IllegalArgumentException();
        }
        array[0] = mX;
        array[1] = mY;
        array[2] = mW;
    }
}
