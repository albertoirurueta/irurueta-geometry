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
 * This class defines a 2D rectangle aligned with the horizontal and vertical
 * axes.
 * A rectangle is defined by two corners (top-left and bottom-right).
 * Notice that vertical coordinates are defined from bottom to top, hence,
 * top is a larger value than bottom.
 */
public class Rectangle implements Serializable {

    /**
     * Constant defining default threshold value used when none is provided.
     */
    public static final double DEFAULT_THRESHOLD = 1e-9;

    /**
     * Top left coordinate of rectangle.
     */
    private Point2D mTopLeft;

    /**
     * Bottom right coordinate of rectangle.
     */
    private Point2D mBottomRight;

    /**
     * Empty constructor.
     * Creates a rectangle centered at the origin and having unitary area.
     */
    public Rectangle() {
        mTopLeft = new InhomogeneousPoint2D(-0.5, -0.5);
        mBottomRight = new InhomogeneousPoint2D(0.5, 0.5);
    }

    /**
     * Constructor with boundaries.
     *
     * @param topLeft     top-left coordinates.
     * @param bottomRight bottom-right coordinates.
     */
    public Rectangle(final Point2D topLeft, final Point2D bottomRight) {
        setBounds(topLeft, bottomRight);
    }

    /**
     * Constructor with boundaries.
     *
     * @param left   left coordinate.
     * @param top    top coordinate.
     * @param right  right coordinate.
     * @param bottom bottom coordinate.
     */
    public Rectangle(
            final double left, final double top, final double right,
            final double bottom) {
        setBounds(left, top, right, bottom);
    }

    /**
     * Constructor from 2D box.
     *
     * @param box a 2D box.
     */
    public Rectangle(final Box2D box) {
        fromBox(box);
    }

    /**
     * Gets top-left corner.
     *
     * @return top-left corner.
     */
    public Point2D getTopLeft() {
        return mTopLeft;
    }

    /**
     * Sets top-left corner.
     *
     * @param topLeft top-left corner.
     */
    public void setTopLeft(final Point2D topLeft) {
        mTopLeft = topLeft;
    }

    /**
     * Gets bottom-right corner.
     *
     * @return bottom-right corner.
     */
    public Point2D getBottomRight() {
        return mBottomRight;
    }

    /**
     * Sets bottom-right corner.
     *
     * @param bottomRight bottom-right corner.
     */
    public void setBottomRight(final Point2D bottomRight) {
        mBottomRight = bottomRight;
    }

    /**
     * Sets rectangle boundaries
     *
     * @param topLeft     top-left coordinates.
     * @param bottomRight bottom-right coordinates.
     */
    public final void setBounds(final Point2D topLeft, final Point2D bottomRight) {
        mTopLeft = topLeft;
        mBottomRight = bottomRight;
    }

    /**
     * Sets rectangle boundaries.
     *
     * @param left   left coordinate.
     * @param top    top coordinate.
     * @param right  right coordinate.
     * @param bottom bottom coordinate.
     */
    public final void setBounds(
            final double left, final double top, final double right,
            final double bottom) {
        setBounds(new InhomogeneousPoint2D(left, top),
                new InhomogeneousPoint2D(right, bottom));
    }

    /**
     * Gets center of rectangle.
     *
     * @param topLeft     top-left coordinates.
     * @param bottomRight bottom-right coordinates.
     * @param result      instance where center of rectangle will be stored.
     */
    public static void getCenter(
            final Point2D topLeft, final Point2D bottomRight,
            final Point2D result) {
        getCenter(topLeft.getInhomX(), topLeft.getInhomY(),
                bottomRight.getInhomX(), bottomRight.getInhomY(), result);
    }

    /**
     * Gets center of rectangle.
     *
     * @param left   left coordinate.
     * @param top    top coordinate
     * @param right  right coordinate.
     * @param bottom bottom coordinate.
     * @param result instance where center of rectangle will be stored.
     */
    public static void getCenter(
            final double left, final double top, final double right,
            final double bottom, final Point2D result) {
        result.setInhomogeneousCoordinates(0.5 * (left + right),
                0.5 * (top + bottom));
    }

    /**
     * Gets center of rectangle.
     *
     * @param topLeft     top-left coordinates.
     * @param bottomRight bottom-right coordinates.
     * @return center of rectangle.
     */
    public static Point2D getCenter(final Point2D topLeft, final Point2D bottomRight) {
        final InhomogeneousPoint2D result = new InhomogeneousPoint2D();
        getCenter(topLeft, bottomRight, result);
        return result;
    }

    /**
     * Gets center of rectangle.
     *
     * @param left   left coordinate.
     * @param top    top coordinate.
     * @param right  right coordinate.
     * @param bottom bottom coordinate.
     * @return center of rectangle.
     */
    public static Point2D getCenter(
            final double left, final double top, final double right,
            final double bottom) {
        final InhomogeneousPoint2D result = new InhomogeneousPoint2D();
        getCenter(left, top, right, bottom, result);
        return result;
    }

    /**
     * Gets center of rectangle.
     *
     * @param result instance where coordinates of center of rectangle will be
     *               stored.
     */
    public void getCenter(final Point2D result) {
        getCenter(mTopLeft.getInhomX(), mTopLeft.getInhomY(),
                mBottomRight.getInhomX(), mBottomRight.getInhomY(), result);
    }

    /**
     * Gets center of rectangle.
     *
     * @return a new point containing center of rectangle.
     */
    public Point2D getCenter() {
        final InhomogeneousPoint2D result = new InhomogeneousPoint2D();
        getCenter(result);
        return result;
    }

    /**
     * Sets values of this rectangle from provided 2D box.
     *
     * @param box a 2D box containing rectangle boundaries.
     */
    public final void fromBox(final Box2D box) {
        box.toRectangle(this);
    }

    /**
     * Creates a 2D box instance equivalent to this rectangle.
     *
     * @return a new 2D box instance equivalent to this rectangle.
     */
    public Box2D toBox() {
        final Box2D result = new Box2D();
        toBox(result);
        return result;
    }

    /**
     * Sets values into provided box instance to make it equivalent to this rectangle.
     *
     * @param result instance where values will be stored.
     */
    public void toBox(final Box2D result) {
        result.fromRectangle(this);
    }

    /**
     * Gets signed width of a rectangle defined as the difference between
     * bottom-right and top-left horizontal coordinates.
     * Signed width will be positive if bottom-right corner is located at the
     * rightmost position and will be negative otherwise.
     *
     * @param topLeft     top-left corner.
     * @param bottomRight bottom-right corner.
     * @return signed width.
     */
    public static double getSignedWidth(final Point2D topLeft, final Point2D bottomRight) {
        return bottomRight.getInhomX() - topLeft.getInhomX();
    }

    /**
     * Gets signed width of this rectangle defined as the difference between
     * its bottom-right and top-left horizontal coordinates.
     * Signed width will be positive if bottom-right corner is located at the
     * rightmost position and will be negative otherwise.
     *
     * @return signed width of this rectangle.
     */
    public double getSignedWidth() {
        return getSignedWidth(mTopLeft, mBottomRight);
    }

    /**
     * Gets width of a rectangle defined as the absolute difference between
     * bottom-right and top-left horizontal coordinates.
     *
     * @param topLeft     top-left corner.
     * @param bottomRight bottom-right corner.
     * @return width the rectangle.
     */
    public static double getWidth(final Point2D topLeft, final Point2D bottomRight) {
        return Math.abs(getSignedWidth(topLeft, bottomRight));
    }

    /**
     * Gets width of this rectangle defined as the absolute difference between
     * bottom-right and top-left horizontal coordinates.
     *
     * @return width of this rectangle.
     */
    public double getWidth() {
        return getWidth(mTopLeft, mBottomRight);
    }

    /**
     * Gets signed height of a rectangle defined as the difference between
     * its bottom-right and top-left vertical coordinates.
     * Signed height will be negative if bottom-right corner is located at the
     * lowest position and will be positive otherwise.
     *
     * @param topLeft     top-left corner.
     * @param bottomRight bottom-rigth corner.
     * @return signed height.
     */
    public static double getSignedHeight(
            final Point2D topLeft, final Point2D bottomRight) {
        return bottomRight.getInhomY() - topLeft.getInhomY();
    }

    /**
     * Gets signed height of this rectangle defined as the difference between
     * its bototm-right and top-left vertical coordinates.
     * Signed height will be negative if bottom right corner is located at the
     * lowest position and will be positive otherwise.
     *
     * @return signed height of this rectangle.
     */
    public double getSignedHeight() {
        return getSignedHeight(mTopLeft, mBottomRight);
    }

    /**
     * Gets height of a rectangle defined as the absolute difference between
     * bottom-rigth and top-left vertical coordinates.
     *
     * @param topLeft     top-left corner.
     * @param bottomRight bottom-right corner.
     * @return height of the rectangle.
     */
    public static double getHeight(
            final Point2D topLeft, final Point2D bottomRight) {
        return Math.abs(getSignedHeight(topLeft, bottomRight));
    }

    /**
     * Gets height of this rectangle defined as the absolute difference between
     * bottom-right and top-left vertical coordinates.
     *
     * @return height of this rectangle.
     */
    public double getHeight() {
        return getHeight(mTopLeft, mBottomRight);
    }

    /**
     * Gets area of the rectangle defined by provided top-left and bototm-right
     * corners.
     *
     * @param topLeft     top-left corner.
     * @param bottomRight bottom-right corner
     * @return area of rectangle.
     */
    public static double getArea(final Point2D topLeft, final Point2D bottomRight) {
        return getWidth(topLeft, bottomRight) * getHeight(topLeft, bottomRight);
    }

    /**
     * Gets area of the rectangle defined by provided corner coordinates.
     *
     * @param left   left coordinate.
     * @param top    top coordinate.
     * @param right  right coordinate.
     * @param bottom bottom coordinate.
     * @return area of rectangle.
     */
    public static double getArea(
            final double left, final double top, final double right,
            final double bottom) {
        return getArea(left - right, top - bottom);
    }

    /**
     * Gets area of the rectangle having provided width and height values.
     *
     * @param width  width of rectangle.
     * @param height height of rectangle.
     * @return area of rectangle.
     */
    public static double getArea(final double width, final double height) {
        return Math.abs(width) * Math.abs(height);
    }

    /**
     * Gets area of this rectangle.
     *
     * @return area of this rectangle.
     */
    public double getArea() {
        return getArea(mTopLeft, mBottomRight);
    }

    /**
     * Sets the center of this rectangle preserving its width and height.
     *
     * @param center new center to be set.
     */
    public void setCenter(final Point2D center) {
        setCenter(center.getInhomX(), center.getInhomY());
    }

    /**
     * Sets center of this rectangle preserving its width and height.
     *
     * @param centerX x coordinate of new center to be set.
     * @param centerY y coordinate of new center to be set.
     */
    public void setCenter(final double centerX, final double centerY) {
        final double width = getSignedWidth();
        final double height = getSignedHeight();
        setCenterAndSize(centerX, centerY, width, height);
    }

    /**
     * Sets center and size (width and height) of this rectangle.
     *
     * @param center new center to be set.
     * @param width  new width to be set. (can be signed).
     * @param height new height to be set. (can be signed).
     */
    public void setCenterAndSize(
            final Point2D center,
            final double width, final double height) {
        setCenterAndSize(center.getInhomX(), center.getInhomY(), width, height);
    }

    /**
     * Sets center and size of this rectangle.
     *
     * @param centerX x coordinate of new center to be set.
     * @param centerY y coordinate of new center to be set.
     * @param width   new width to be set. (can be signed).
     * @param height  new height to be set. (can be signed).
     */
    public void setCenterAndSize(
            final double centerX, final double centerY, final double width,
            final double height) {
        final double halfWidth = width / 2.0;
        final double halfHeight = height / 2.0;
        mTopLeft = new InhomogeneousPoint2D(centerX - halfWidth,
                centerY - halfHeight);
        mBottomRight = new InhomogeneousPoint2D(centerX + halfWidth,
                centerY + halfHeight);
    }

    /**
     * Gets perimeter of a rectangle having provided width and height.
     *
     * @param width  width of rectangle.
     * @param height height of rectangle.
     * @return perimeter of a rectangle having provided width and height.
     */
    public static double getPerimeter(
            final double width, final double height) {
        return 2.0 * (width + height);
    }

    /**
     * Gets perimeter of a rectangle having provided top-left and bottom-right
     * corners.
     *
     * @param topLeft     top-left corner.
     * @param bottomRight bottom-right corner.
     * @return perimeter of rectangle.
     */
    public static double getPerimeter(final Point2D topLeft, final Point2D bottomRight) {
        return getPerimeter(topLeft.getInhomX(), topLeft.getInhomY(),
                bottomRight.getInhomX(), bottomRight.getInhomY());
    }

    /**
     * Gets perimeter of a rectangle having provided top-left and bototm-right
     * corners.
     *
     * @param left   left coordinate.
     * @param top    top coordinate.
     * @param right  right coordinate.
     * @param bottom borrom coordinate.
     * @return perimeter of rectangle.
     */
    public static double getPerimeter(
            final double left, final double top, final double right,
            final double bottom) {
        final double width = Math.abs(right - left);
        final double height = Math.abs(bottom - top);
        return getPerimeter(width, height);
    }

    /**
     * Gets perimeter of this rectangle.
     *
     * @return perimeter of this rectangle.
     */
    public double getPerimeter() {
        return getPerimeter(mTopLeft, mBottomRight);
    }

    /**
     * Indicates if provided (x,y) coordinates are located inside of the
     * rectangle defined by top-left and bottom-right coordinates up to a
     * certain threshold.
     * A positive threshold makes inside area smaller, a negative threshold
     * increases the inside area.
     *
     * @param x         x coordinate of point to be checked.
     * @param y         y coordinate of point to be checked.
     * @param left      left coordinate.
     * @param top       top coordinate.
     * @param right     right coordinate.
     * @param bottom    bottom coordinate.
     * @param threshold threshold to use as a margin to determine whether a
     *                  point lies inside or not.
     * @return true if point is inside, false otherwise.
     */
    public static boolean isInside(
            final double x, final double y, final double left, final double top,
            final double right, final double bottom, final double threshold) {

        return !isAtLeftSide(x, y, left, top, right, bottom, threshold) &&
                !isAtTopLeftCorner(x, y, left, top, right, bottom, threshold) &&
                !isAtTopSide(x, y, left, top, right, bottom, threshold) &&
                !isAtTopRightCorner(x, y, left, top, right, bottom, threshold) &&
                !isAtRightSide(x, y, left, top, right, bottom, threshold) &&
                !isAtBottomRightCorner(x, y, left, top, right, bottom, threshold) &&
                !isAtBottomSide(x, y, left, top, right, bottom, threshold) &&
                !isAtBottomLeftCorner(x, y, left, top, right, bottom, threshold);
    }

    /**
     * Indicates if provided (x,y) coordinates are located inside of the
     * rectangle defined by top-left and bottom-right coordinates.
     *
     * @param x      x coordinate of point to be checked.
     * @param y      y coordinate of point to be checked.
     * @param left   left coordinate.
     * @param top    top coordinate.
     * @param right  right coordinate.
     * @param bottom bottom coordinate.
     * @return true if point is inside, false otherwise.
     */
    public static boolean isInside(
            final double x, final double y, final double left, final double top,
            final double right, final double bottom) {
        return isInside(x, y, left, top, right, bottom, 0.0);
    }

    /**
     * Indicates if provided point is located inside of the rectangle defined by
     * top-left and bottom-right coordinates up to a certain threshold.
     * A positive threshold makes inside area smaller, a negative threshold
     * increases the inside area.
     *
     * @param point     point to be checked.
     * @param left      left coordinate.
     * @param top       top coordinate.
     * @param right     right coordinate.
     * @param bottom    bottom coordinate.
     * @param threshold threshold to use as a margin to determine whether
     *                  point lies inside or not.
     * @return true if point is inside, false otherwise.
     */
    public static boolean isInside(
            final Point2D point, final double left, final double top,
            final double right, final double bottom, final double threshold) {
        return isInside(point.getInhomX(), point.getInhomY(), left, top, right,
                bottom, threshold);
    }

    /**
     * Indicates if provided point is located inside of the rectangle defined by
     * top-left and bottom-right coordinates.
     *
     * @param point  point to be checked.
     * @param left   left coordinate.
     * @param top    top coordinate.
     * @param right  right coordinate.
     * @param bottom bottom coordinate.
     * @return true if point is inside, false otherwise.
     */
    public static boolean isInside(
            final Point2D point, final double left, final double top,
            final double right, final double bottom) {
        return isInside(point, left, top, right, bottom, 0.0);
    }

    /**
     * Indicates if provided point is located inside of the rectangle defined by
     * provided center and size up to a certain threshold.
     * A positive threshold makes inside area smaller, a negative threshold
     * increases the inside area.
     *
     * @param point     point to be checked.
     * @param center    center of rectangle.
     * @param width     width of rectangle.
     * @param height    height of rectangle.
     * @param threshold threshold to use as a margin to determine whether point
     *                  lies inside or not.
     * @return true if point is inside, false otherwise.
     */
    public static boolean isInside(
            final Point2D point, final Point2D center, final double width,
            final double height, final double threshold) {
        final double halfWidth = width / 2.0;
        final double halfHeight = height / 2.0;
        return isInside(point, center.getInhomX() - halfWidth,
                center.getInhomY() - halfHeight,
                center.getInhomX() + halfWidth,
                center.getInhomY() + halfHeight, threshold);
    }

    /**
     * Indicates if provided point is located inside of the rectangle defined by
     * provided center and size.
     *
     * @param point  point to be checked.
     * @param center center of rectangle.
     * @param width  width of rectangle.
     * @param height height of rectangle.
     * @return true if point is inside, false otherwise.
     */
    public static boolean isInside(
            final Point2D point, final Point2D center, final double width,
            final double height) {
        return isInside(point, center, width, height, 0.0);
    }

    /**
     * Indicates if provided point is located inside of the rectangle defined by
     * provided top-left and bottom-right corners up to a certain threshold.
     * A positive threshold makes inside area smaller, a negative threshold
     * increases the inside area.
     *
     * @param x           x coordinate of point to be checked.
     * @param y           y coordinate of point to be checked.
     * @param topLeft     top-left corner.
     * @param bottomRight bottom-right corner.
     * @param threshold   threshold to use as a margin to determine whether
     *                    point lies inside or not.
     * @return true if point is inside, false otherwise.
     */
    public static boolean isInside(
            final double x, final double y, final Point2D topLeft,
            final Point2D bottomRight, final double threshold) {
        return isInside(x, y, topLeft.getInhomX(), topLeft.getInhomY(),
                bottomRight.getInhomX(), bottomRight.getInhomY(), threshold);
    }

    /**
     * Indicates if provided point is located inside of the rectangle defined by
     * provided top-left and bottom-right corners.
     *
     * @param x           x coordinate of point to be checked.
     * @param y           y coordinate of point to be checked.
     * @param topLeft     top-left corner.
     * @param bottomRight bottom-right corner.
     * @return true if point is inside, false otherwise.
     */
    public static boolean isInside(
            final double x, final double y, final Point2D topLeft,
            final Point2D bottomRight) {
        return isInside(x, y, topLeft, bottomRight, 0.0);
    }

    /**
     * Indicates if provided point is located inside of the rectangle defined by
     * provided top-left and bottom-right corners up to a certain threshold.
     * A positive threshold makes inside area smaller, a negative threshold
     * increases the inside area.
     *
     * @param point       point to be checked.
     * @param topLeft     top-left corner.
     * @param bottomRight bottom-right corner.
     * @param threshold   threshold to use as a margin to deterine whether point
     *                    lies inside or not.
     * @return true if point is inside, false otherwise.
     */
    public static boolean isInside(
            final Point2D point, final Point2D topLeft,
            final Point2D bottomRight, final double threshold) {
        return isInside(point, topLeft.getInhomX(), topLeft.getInhomY(),
                bottomRight.getInhomX(), bottomRight.getInhomY(), threshold);
    }

    /**
     * Indicates if provided point is located inside of the rectangle defined by
     * provided top-left and bottom-right corners.
     *
     * @param point       point to be checked.
     * @param topLeft     top-left corner.
     * @param bottomRight bottom-right corner.
     * @return true if point is inside, false otherwise.
     */
    public static boolean isInside(
            final Point2D point, final Point2D topLeft,
            final Point2D bottomRight) {
        return isInside(point, topLeft, bottomRight, 0.0);
    }

    /**
     * Indicates if provided point coordinates are located inside this rectangle
     * up to a certain threshold.
     * A positive threshold makes inside area smaller, a negative threshold
     * increases the inside area.
     *
     * @param x         x coordinate of point to be checked.
     * @param y         y coordinate of point to be checked.
     * @param threshold threshold to use as a margin to determine whether point
     *                  lies inside or not.
     * @return true if point is inside, false otherwise.
     */
    public boolean isInside(
            final double x, final double y, final double threshold) {
        return isInside(x, y, mTopLeft, mBottomRight, threshold);
    }

    /**
     * Indicates if provided point coordinates are located inside this
     * rectangle.
     *
     * @param x x coordinate of point to be checked.
     * @param y y coordinate of point to be checked.
     * @return true if point is inside, false otherwise.
     */
    public boolean isInside(
            final double x, final double y) {
        return isInside(x, y, mTopLeft, mBottomRight);
    }

    /**
     * Indicates if provided point is located inside this rectangle up to a
     * certain threshold.
     * A positive threshold makes inside area smaller, a negative threshold
     * increases the inside area.
     *
     * @param point     point to be checked.
     * @param threshold threshold to use as a margin to determine whether point
     *                  lies inside or not.
     * @return true if point is inside, false otherwise.
     */
    public boolean isInside(
            final Point2D point, final double threshold) {
        return isInside(point, mTopLeft, mBottomRight, threshold);
    }

    /**
     * Indicates if provided point is located inside this rectangle.
     *
     * @param point point to be checked.
     * @return true if point is inside, false otherwise.
     */
    public boolean isInside(final Point2D point) {
        return isInside(point, mTopLeft, mBottomRight);
    }

    /**
     * Indicates if point at provided coordinates are located at left side of
     * rectangle defined by provided top-left and bottom-right corners up to a
     * certain threshold.
     * A positive threshold moves left border to the right, a negative threshold
     * moves left border to the left.
     *
     * @param x         x coordinate of point to be checked.
     * @param y         y coordinate of point to be checked.
     * @param left      left coordinate of rectangle.
     * @param top       top coordinate of rectangle.
     * @param right     right coordinate of rectangle.
     * @param bottom    bottom coordinate of rectangle.
     * @param threshold threshold to use as a margin to determine whether point
     *                  lies at left side or not.
     * @return true if point is at left side, false otherwise.
     */
    public static boolean isAtLeftSide(
            final double x, final double y, final double left,
            final double top, final double right, final double bottom,
            final double threshold) {
        // fix values in case that corners are reversed
        final double left2 = Math.min(left, right);
        final double top2 = Math.max(top, bottom);
        final double bottom2 = Math.min(top, bottom);

        return x < (left2 + threshold) && y >= (bottom2 + threshold) &&
                y <= (top2 - threshold);
    }

    /**
     * Indicates if point at provided coordinates is located at left side of
     * rectangle defined by provided top-left and bottom-right corners.
     *
     * @param x      x coordinate of point to be checked.
     * @param y      y coordinate of point to be checked.
     * @param left   left coordinate of rectangle.
     * @param top    top coordinate of rectangle.
     * @param right  right coordinate of rectangle.
     * @param bottom bottom coordinate of rectangle.
     * @return true if point is at left side, false otherwise.
     */
    public static boolean isAtLeftSide(
            final double x, final double y, final double left,
            final double top, final double right, final double bottom) {
        return isAtLeftSide(x, y, left, top, right, bottom, 0.0);
    }

    /**
     * Indicates if provided point is located at left side of rectangle defined
     * by provided top-left and bottom-right corners up to a certain threshold.
     * A positive threshold moves left border to the right, a negative threshold
     * moves left border to the left.
     *
     * @param point     point to be checked.
     * @param left      left coordinate of rectangle.
     * @param top       top coordinate of rectangle.
     * @param right     right coordinate of rectangle.
     * @param bottom    bottom coordinate of rectangle.
     * @param threshold threshold to use as a margin to determine whether point
     *                  lies at left side or not.
     * @return true if point is at left side, false otherwise.
     */
    public static boolean isAtLeftSide(
            final Point2D point, final double left, final double top,
            final double right, final double bottom, final double threshold) {
        return isAtLeftSide(point.getInhomX(), point.getInhomY(), left, top,
                right, bottom, threshold);
    }

    /**
     * Indicates if provided point is located at left side of rectangle defined
     * by provided top-left and bottom-right corners.
     *
     * @param point  point to be checked.
     * @param left   left coordinate of rectangle.
     * @param top    top coordinate of rectangle.
     * @param right  right coordinate of rectangle.
     * @param bottom bottom coordinate of rectangle.
     * @return true if point is at left side, false otherwise.
     */
    public static boolean isAtLeftSide(
            final Point2D point, final double left, final double top,
            final double right, final double bottom) {
        return isAtLeftSide(point, left, top, right, bottom, 0.0);
    }

    /**
     * Indicates if provided point coordinates are located at left side of
     * rectangle defined by provided top-left and bottom-right corners up to a
     * certain threshold.
     * A positive threshold moves left border to the right, a negative threshold
     * moves left border to the left.
     *
     * @param x           x coordinate of point to be checked.
     * @param y           y coordinate of point to be checked.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @param threshold   threshold to use as a margin to determine whether point
     *                    lies at left side or not.
     * @return true if point is at left side, false otherwise.
     */
    public static boolean isAtLeftSide(
            final double x, final double y, final Point2D topLeft,
            final Point2D bottomRight, final double threshold) {
        return isAtLeftSide(x, y, topLeft.getInhomX(), topLeft.getInhomY(),
                bottomRight.getInhomX(), bottomRight.getInhomY(), threshold);
    }

    /**
     * Indicates if provided point coordinates are located at left side of
     * rectangle defined by provided top-left and bottom-right corners.
     *
     * @param x           x coordinate of point to be checked.
     * @param y           y coordinate of point to be checked.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @return true if point is at left side, false otherwise.
     */
    public static boolean isAtLeftSide(
            final double x, final double y, final Point2D topLeft,
            final Point2D bottomRight) {
        return isAtLeftSide(x, y, topLeft, bottomRight, 0.0);
    }

    /**
     * Indicates if provided point is located at left side of rectangle defined
     * by provided top-left and bottom-right corners up to a certain threshold.
     * A positive threshold moves left border to the right, a negative threshold
     * moves left border to the left.
     *
     * @param point       point to be checked.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @param threshold   threshold to use as a margin to determine whether point
     *                    lies at left side or not.
     * @return true if point lies at left side, false otherwise.
     */
    public static boolean isAtLeftSide(
            final Point2D point, final Point2D topLeft,
            final Point2D bottomRight, final double threshold) {
        return isAtLeftSide(point.getInhomX(), point.getInhomY(), topLeft,
                bottomRight, threshold);
    }

    /**
     * Indicates if provided point is located at left side of rectangle defined
     * by provided top-left and bottom-right corners.
     *
     * @param point       point to be checked.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @return true if point lies at left side, false otherwise.
     */
    public static boolean isAtLeftSide(
            final Point2D point, final Point2D topLeft,
            final Point2D bottomRight) {
        return isAtLeftSide(point, topLeft, bottomRight, 0.0);
    }

    /**
     * Indicates if provided point coordinates are located at left side of
     * rectangle defined by provided center and size values.
     * A positive threshold moves left border to the right, a negative threshold
     * moves border to the left.
     *
     * @param x         x coordinate of point to be checked.
     * @param y         y coordinate of point to be checked.
     * @param center    center of rectangle.
     * @param width     width of rectangle.
     * @param height    height of rectangle.
     * @param threshold threshold to use as a margin to determine whether point
     *                  lies at left side or not.
     * @return true if point is at left side, false otherwise.
     */
    public static boolean isAtLeftSide(
            final double x, final double y, final Point2D center,
            final double width, final double height, final double threshold) {
        double halfWidth = width / 2.0;
        double halfHeight = height / 2.0;
        double centerX = center.getInhomX();
        double centerY = center.getInhomY();
        return isAtLeftSide(x, y, centerX - halfWidth, centerY - halfHeight,
                centerX + halfWidth, centerY + halfHeight, threshold);
    }

    /**
     * Indicates if provided point coordinates are located at left side of
     * rectangle defined by provided center and size values.
     *
     * @param x      x coordinate of point to be checked.
     * @param y      y coordinate of point to be checked.
     * @param center center of rectangle.
     * @param width  width of rectangle.
     * @param height height of rectangle.
     * @return true if point is at left side, false otherwise.
     */
    public static boolean isAtLeftSide(
            final double x, final double y, final Point2D center,
            final double width, final double height) {
        return isAtLeftSide(x, y, center, width, height, 0.0);
    }

    /**
     * Indicates if provided point is located at left side of rectangle defined
     * by provided center and size values up to a certain threshold.
     * A positive threshold moves left border to the right, a negative threshold
     * moves left border to the left.
     *
     * @param point     point to be checked.
     * @param center    center of rectangle.
     * @param width     width of rectangle.
     * @param height    height of rectangle.
     * @param threshold threshold to use as a margin to determine whether point
     *                  lies at left side or not.
     * @return true if point is at left side, false otherwise.
     */
    public static boolean isAtLeftSide(
            final Point2D point, final Point2D center,
            final double width, final double height, final double threshold) {
        return isAtLeftSide(point.getInhomX(), point.getInhomY(), center, width,
                height, threshold);
    }

    /**
     * Indicates if provided point is located at left side of rectangle defined
     * by provided center and size values.
     *
     * @param point  point to be checked.
     * @param center center of rectangle.
     * @param width  width of rectangle.
     * @param height height of rectangle.
     * @return true if point is at left side, false otherwise.
     */
    public static boolean isAtLeftSide(
            final Point2D point, final Point2D center,
            final double width, final double height) {
        return isAtLeftSide(point, center, width, height, 0.0);
    }

    /**
     * Indicates if provided point coordinates are located at left side of this
     * rectangle up to a certain threshold.
     * A positive threshold moves left border to the right, a negative threshold
     * moves left border to the left.
     *
     * @param x         x coordinate of point to be checked.
     * @param y         y coordinate of point to be checked.
     * @param threshold threshold to use as a margin to determine whether point
     *                  lies at left side or not.
     * @return true if point is at left side, false otherwise.
     */
    public boolean isAtLeftSide(
            final double x, final double y, final double threshold) {
        return isAtLeftSide(x, y, mTopLeft, mBottomRight, threshold);
    }

    /**
     * Indicates if provided point coordinates are located at left side of this
     * rectangle.
     *
     * @param x x coordinate of point to be checked.
     * @param y y coordinate of point to be checked.
     * @return true if point is at left side, false otherwise.
     */
    public boolean isAtLeftSide(
            final double x, final double y) {
        return isAtLeftSide(x, y, 0.0);
    }

    /**
     * Indicates if provided point is located at left side of this rectangle up
     * to a certain threshold.
     * A positive threshold moves left border to the right, a negative threshold
     * moves left border to the left.
     *
     * @param point     point to be checked.
     * @param threshold threshold to use as a margin to determine whether point
     *                  lies at left side or not.
     * @return true if point is at left side, false otherwise.
     */
    public boolean isAtLeftSide(final Point2D point, final double threshold) {
        return isAtLeftSide(point.getInhomX(), point.getInhomY(), threshold);
    }

    /**
     * Indicates if provided point is located at left side of this rectangle.
     *
     * @param point point to be checked.
     * @return true if point is at left side, false otherwise.
     */
    public boolean isAtLeftSide(final Point2D point) {
        return isAtLeftSide(point, 0.0);
    }

    /**
     * Indicates if provided point coordinates are located at top-left corner of
     * rectangle defined by provided top-left and bottom-right corners up to a
     * certain threshold.
     * A positive threshold moves top-left corner towards the rectangle
     * interior, a negative threshold moves top-left corner towards rectangle
     * exterior.
     *
     * @param x         x coordinate of point to be checked.
     * @param y         y coordinate of point to be checked.
     * @param left      left coordinate of rectangle.
     * @param top       top coordinate of rectangle.
     * @param right     right coordinate of rectangle.
     * @param bottom    bottom coordinate of rectangle.
     * @param threshold threshold to use as a margin to determine whether point
     *                  lies at top-left corner or not.
     * @return true if point is at top-left corner, false otherwise.
     */
    public static boolean isAtTopLeftCorner(
            final double x, final double y, final double left,
            final double top, final double right, final double bottom,
            final double threshold) {
        // fix values in case that corners are reversed
        final double top2 = Math.max(top, bottom);
        final double left2 = Math.min(left, right);

        return x < (left2 + threshold) && y > (top2 - threshold);
    }

    /**
     * Indicates if point at provided coordinates is located at top-left corner
     * of rectangle defined by provided top-left and bottom-right corners.
     *
     * @param x      x coordinate of point to be checked.
     * @param y      y coordinate of point to be checked.
     * @param left   left coordinate of rectangle.
     * @param top    top coordinate of rectangle.
     * @param right  right coordinate of rectangle.
     * @param bottom bottom coordinate of rectangle.
     * @return true if point is at top-left corner, false otherwise.
     */
    public static boolean isAtTopLeftCorner(
            final double x, final double y, final double left,
            final double top, final double right, final double bottom) {
        return isAtTopLeftCorner(x, y, left, top, right, bottom, 0.0);
    }

    /**
     * Indicates if provided point is located at top-left corner of rectangle
     * defined by provided top-left and bottom-right corners up to a certain
     * threshold.
     * A positive threshold moves top-left corner towards the rectangle
     * interior, a negative threshold moves top-left corner towards rectangle
     * exterior.
     *
     * @param point     point to be checked.
     * @param left      left coordinate of rectangle.
     * @param top       top coordinate of rectangle.
     * @param right     right coordinate of rectangle.
     * @param bottom    bottom coordinate of rectangle.
     * @param threshold threshold to use as a margin to determine whether point
     *                  lies at top-left or not.
     * @return true if point is at top-left corner, false otherwise.
     */
    public static boolean isAtTopLeftCorner(
            final Point2D point, final double left,
            final double top, final double right, final double bottom,
            final double threshold) {
        return isAtTopLeftCorner(point.getInhomX(), point.getInhomY(), left,
                top, right, bottom, threshold);
    }

    /**
     * Indicates if provided point is located at top-left corner of rectangle
     * defined by provided top-left and bottom-right corners.
     *
     * @param point  point to be checked.
     * @param left   left coordinate of rectangle.
     * @param top    top coordinate of rectangle.
     * @param right  right coordinate of rectangle.
     * @param bottom bottom coordinate of rectangle.
     * @return true if point is at top-left corner, false otherwise.
     */
    public static boolean isAtTopLeftCorner(
            final Point2D point, final double left,
            final double top, final double right, final double bottom) {
        return isAtTopLeftCorner(point, left, top, right, bottom, 0.0);
    }

    /**
     * Indicates if provided point coordinates are located at top-left corner of
     * rectangle defined by provided top-left and bottom-right corners up to a
     * certain threshold.
     * A positive threshold moves top-left corner towards the rectangle
     * interior, a negative threshold moves top-left corner towards rectangle
     * exterior.
     *
     * @param x           x coordinate of point to be checked.
     * @param y           y coordinate of point to be checked.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @param threshold   threshold to use as a margin to determine whether point
     *                    lies at top-left corner or not.
     * @return true if point is at top-left corner, false otherwise.
     */
    public static boolean isAtTopLeftCorner(
            final double x, final double y, final Point2D topLeft,
            final Point2D bottomRight, final double threshold) {
        return isAtTopLeftCorner(x, y, topLeft.getInhomX(), topLeft.getInhomY(),
                bottomRight.getInhomX(), bottomRight.getInhomY(), threshold);
    }

    /**
     * Indicates if provided point coordinates are located at top-left corner of
     * rectangle defined by provided top-left and bottom-right corners.
     *
     * @param x           x coordinate of point to be checked.
     * @param y           y coordinate of point to be checked.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @return true if point is at top-left corner, false otherwise.
     */
    public static boolean isAtTopLeftCorner(
            final double x, final double y, final Point2D topLeft,
            final Point2D bottomRight) {
        return isAtTopLeftCorner(x, y, topLeft, bottomRight, 0.0);
    }

    /**
     * Indicates if provided point is located at top-left corner of rectangle
     * defined by provided top-left and bottom-right corners up to a certain
     * threshold.
     * A positive threshold moves top-left corner towards the rectangle
     * interior, a negative threshold moves top-left corner towards rectangle
     * exterior.
     *
     * @param point       point to be checked.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @param threshold   threshold to use as a margin to determine whether point
     *                    lies at top-left corner or not.
     * @return true if point is at top-left corner, false otherwise.
     */
    public static boolean isAtTopLeftCorner(
            final Point2D point, final Point2D topLeft,
            final Point2D bottomRight, final double threshold) {
        return isAtTopLeftCorner(point.getInhomX(), point.getInhomY(), topLeft,
                bottomRight, threshold);
    }

    /**
     * Indicates if provided point is located at top-left corner of rectangle
     * defined by provided top-left and bottom-right corners.
     *
     * @param point       point to be checked.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @return true if point is at top-left corner, false otherwise.
     */
    public static boolean isAtTopLeftCorner(
            final Point2D point, final Point2D topLeft,
            final Point2D bottomRight) {
        return isAtTopLeftCorner(point, topLeft, bottomRight, 0.0);
    }

    /**
     * Indicates if provided point coordinates are located at top-left corner of
     * rectangle defined by provided center and size values up to a certain
     * threshold.
     * A positive threshold moves top-left corner towards the rectangle
     * interior, a negative threshold moves top-left corner towards rectangle
     * exterior.
     *
     * @param x         x coordinate of point to be checked.
     * @param y         y coordinate of point to be checked.
     * @param center    center of rectangle.
     * @param width     width of rectangle.
     * @param height    height of rectangle.
     * @param threshold threshold to use as a margin to determine whether point
     *                  lies at top-left corner or not.
     * @return true if point is at top-left corner, false otherwise.
     */
    public static boolean isAtTopLeftCorner(
            final double x, final double y, final Point2D center,
            final double width, final double height, final double threshold) {
        final double halfWidth = width / 2.0;
        final double halfHeight = height / 2.0;
        final double centerX = center.getInhomX();
        final double centerY = center.getInhomY();
        return isAtTopLeftCorner(x, y, centerX - halfWidth,
                centerY - halfHeight, centerX + halfWidth,
                centerY + halfHeight, threshold);
    }

    /**
     * Indicates if provided point coordinates are located at top-left corner of
     * rectangle defined by provided center and size values.
     *
     * @param x      x coordinate of point to be checked.
     * @param y      y coordinate of point to be checked.
     * @param center center of rectangle.
     * @param width  width of rectangle.
     * @param height height of rectangle.
     * @return true if point is at top-left corner, false otherwise.
     */
    public static boolean isAtTopLeftCorner(
            final double x, final double y, final Point2D center,
            final double width, final double height) {
        return isAtTopLeftCorner(x, y, center, width, height, 0.0);
    }

    /**
     * Indicates if provided point is located at top-left corner of rectangle
     * defined by provided center and size values up to a certain threshold.
     * A positive threshold moves top-left corner towards the rectangle
     * interior, a negative threshold moves top-left corner towards rectangle
     * exterior.
     *
     * @param point     point to be checked.
     * @param center    center of rectangle.
     * @param width     width of rectangle.
     * @param height    height of rectangle.
     * @param threshold threshold to use as a margin to determine whether point
     *                  lies at top-left corner or not.
     * @return true if point is at top-left corner, false otherwise.
     */
    public static boolean isAtTopLeftCorner(
            final Point2D point, final Point2D center,
            final double width, final double height, final double threshold) {
        return isAtTopLeftCorner(point.getInhomX(), point.getInhomY(), center,
                width, height, threshold);
    }

    /**
     * Indicates if provided point is located at top-left corner of rectangle
     * defined by provided center and size values.
     *
     * @param point  point to be checked.
     * @param center center of rectangle.
     * @param width  width of rectangle.
     * @param height height of rectangle.
     * @return true if point is at top-left corner, false otherwise.
     */
    public static boolean isAtTopLeftCorner(
            final Point2D point, final Point2D center,
            final double width, final double height) {
        return isAtTopLeftCorner(point, center, width, height, 0.0);
    }

    /**
     * Indicates if provided point coordinates are located at top-left corner of
     * this rectangle up to a certain threshold.
     * A positive threshold moves top-left corner towards the rectangle
     * interior, a negative threshold moves top-left corner towards rectangle
     * exterior.
     *
     * @param x         x coordinate of point to be checked.
     * @param y         y coordinate of point to be checked.
     * @param threshold threshold to use as a margin to determine whether point
     *                  lies at top-left corner or not.
     * @return true if point is at top-left corner, false otherwise.
     */
    public boolean isAtTopLeftCorner(
            final double x, final double y, final double threshold) {
        return isAtTopLeftCorner(x, y, mTopLeft, mBottomRight, threshold);
    }

    /**
     * Indicates if provided point coordinates are located at top-left corner of
     * this rectangle.
     *
     * @param x x coordinate of point to be checked.
     * @param y y coordinate of point to be checked.
     * @return true if point is at top-left corner, false otherwise.
     */
    public boolean isAtTopLeftCorner(
            final double x, final double y) {
        return isAtTopLeftCorner(x, y, 0.0);
    }

    /**
     * Indicates if provided point is located at top-left corner of this
     * rectangle up to a certain threshold.
     * A positive threshold moves top-left corner towards the rectangle
     * interior, a negative threshold moves top-left corner towards rectangle
     * exterior.
     *
     * @param point     point to be checked.
     * @param threshold threshold to use as a margin to determine whether point
     *                  lies at top-left corner or not.
     * @return true if point is at top-left corner, false otherwise.
     */
    public boolean isAtTopLeftCorner(
            final Point2D point, final double threshold) {
        return isAtTopLeftCorner(point.getInhomX(), point.getInhomY(),
                threshold);
    }

    /**
     * Indicates if provided point is located at top-left corner of this
     * rectangle.
     *
     * @param point point to be checked.
     * @return true if point is at top-left corner, false otherwise.
     */
    public boolean isAtTopLeftCorner(
            final Point2D point) {
        return isAtTopLeftCorner(point, 0.0);
    }

    /**
     * Indicates if provided point coordinates are located at top side of
     * rectangle defined by provided top-left and bottom-right corners up to a
     * certain threshold.
     * A positive threshold moves top border downwards, a negative threshold
     * moves top border upwards.
     *
     * @param x         x coordinate of point to be checked.
     * @param y         y coordinate of point to be checked.
     * @param left      left coordinate of rectangle.
     * @param top       top coordinate of rectangle.
     * @param right     right coordinate of rectangle.
     * @param bottom    bottom coordinate of rectangle.
     * @param threshold threshold to use as a margin to determine whether point
     *                  lies at top side or not.
     * @return true if point is at top side, false otherwise.
     */
    public static boolean isAtTopSide(
            final double x, final double y, final double left,
            final double top, final double right, final double bottom,
            final double threshold) {
        final double top2 = Math.max(top, bottom);
        final double left2 = Math.min(left, right);
        final double right2 = Math.max(left, right);

        return y > (top2 - threshold) && x >= (left2 + threshold) &&
                x <= (right2 - threshold);
    }

    /**
     * Indicates if point at provided coordinates is located at top side of
     * rectangle defined by provided top-left and bottom-right corners.
     *
     * @param x      x coordinate of point to be checked.
     * @param y      y coordinate of point to be checked.
     * @param left   left coordinate of rectangle.
     * @param top    top coordinate of rectangle.
     * @param right  right coordinate of rectangle.
     * @param bottom bottom coordinate of rectangle.
     * @return true if point is at top side, false otherwise.
     */
    public static boolean isAtTopSide(
            final double x, final double y, final double left,
            final double top, final double right, final double bottom) {
        return isAtTopSide(x, y, left, top, right, bottom, 0.0);
    }

    /**
     * Indicates if provided point is located at top side of rectangle defined
     * by provided top-left and bottom-right corners up to a certain threshold.
     * A positive threshold moves top border downwards, a negative threshold
     * moves top border upwards.
     *
     * @param point     point to be checked.
     * @param left      left coordinate of rectangle.
     * @param top       top coordinate of rectangle.
     * @param right     right coordinate of rectangle.
     * @param bottom    bottom coordinate of rectangle.
     * @param threshold threshold to use as a margin to determine whether point
     *                  lies at top side or not.
     * @return true if point is at top side, false otherwise.
     */
    public static boolean isAtTopSide(
            final Point2D point, final double left, final double top,
            final double right, final double bottom, final double threshold) {
        return isAtTopSide(point.getInhomX(), point.getInhomY(), left, top,
                right, bottom, threshold);
    }

    /**
     * Indicates if provided point is located at top side of rectangle defined
     * by provided top-left and bottom-right corners.
     *
     * @param point  point to be checked.
     * @param left   left coordinate of rectangle.
     * @param top    top coordinate of rectangle.
     * @param right  right coordinate of rectangle.
     * @param bottom bottom coordinate of rectangle.
     * @return true if point is at top side, false otherwise.
     */
    public static boolean isAtTopSide(
            final Point2D point, final double left, final double top,
            final double right, final double bottom) {
        return isAtTopSide(point, left, top, right, bottom, 0.0);
    }

    /**
     * Indicates if provided point coordinates are located at top side of
     * rectangle defined by provided top-left and bottom-right corners up to a
     * certain threshold.
     * A positive threshold moves top border downwards, a negative threshold
     * moves top border upwards.
     *
     * @param x           x coordinate of point to be checked.
     * @param y           y coordinate of point to be checked.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @param threshold   threshold to use as a margin to determine whether point
     *                    lies at top side or not.
     * @return true if point is at top side, false otherwise.
     */
    public static boolean isAtTopSide(
            final double x, final double y, final Point2D topLeft,
            final Point2D bottomRight, final double threshold) {
        return isAtTopSide(x, y, topLeft.getInhomX(), topLeft.getInhomY(),
                bottomRight.getInhomX(), bottomRight.getInhomY(), threshold);
    }

    /**
     * Indicates if provided point coordinates are located at top side of
     * rectangle defined by provided top-left and bottom-right corners.
     *
     * @param x           x coordinate of point to be checked.
     * @param y           y coordinate of point to be checked.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @return true if point is at top side, false otherwise.
     */
    public static boolean isAtTopSide(
            final double x, final double y, final Point2D topLeft,
            final Point2D bottomRight) {
        return isAtTopSide(x, y, topLeft, bottomRight, 0.0);
    }

    /**
     * Indicates if provided point is located at top side of rectangle defined
     * by provided top-left and bottom-right corners.
     * A positive threshold moves top border downwards, a negative threshold
     * moves top border upwards.
     *
     * @param point       point to be checked.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @param threshold   threshold to use as a margin to determine whether point
     *                    lies at top side or not.
     * @return true if point is at top side, false otherwise.
     */
    public static boolean isAtTopSide(
            final Point2D point, final Point2D topLeft,
            final Point2D bottomRight, final double threshold) {
        return isAtTopSide(point.getInhomX(), point.getInhomY(), topLeft,
                bottomRight, threshold);
    }

    /**
     * Indicates if provided point is located at top side of rectangle defined
     * by provided top-left and bottom-right corners.
     *
     * @param point       point to be checked.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @return true if point is at top side, false otherwise.
     */
    public static boolean isAtTopSide(
            final Point2D point, final Point2D topLeft,
            final Point2D bottomRight) {
        return isAtTopSide(point, topLeft, bottomRight, 0.0);
    }

    /**
     * Indicates if provided point coordinates are located at top side of
     * rectangle defined by provided center and size values up to a certain
     * threshold.
     * A positive threshold moves top border downwards, a negative threshold
     * moves top border upwards.
     *
     * @param x         x coordinate of point to be checked.
     * @param y         y coordinate of point to be checked.
     * @param center    center of rectangle.
     * @param width     width of rectangle.
     * @param height    height of rectangle.
     * @param threshold threshold to use as a margin to determine whether point
     *                  lies at top side or not.
     * @return true if point is at top side, false otherwise.
     */
    public static boolean isAtTopSide(
            final double x, final double y, final Point2D center,
            final double width, final double height, final double threshold) {
        final double halfWidth = width / 2.0;
        final double halfHeight = height / 2.0;
        final double centerX = center.getInhomX();
        final double centerY = center.getInhomY();
        return isAtTopSide(x, y, centerX - halfWidth, centerY - halfHeight,
                centerX + halfWidth, centerY + halfHeight, threshold);
    }

    /**
     * Indicates if provided point coordinates are located at top side of
     * rectangle defined by provided center and size values.
     *
     * @param x      x coordinate of point to be checked.
     * @param y      y coordinate of point to be checked.
     * @param center center of rectangle.
     * @param width  width of rectangle.
     * @param height height of rectangle.
     * @return true if point is at top side, false otherwise.
     */
    public static boolean isAtTopSide(
            final double x, final double y, final Point2D center,
            final double width, final double height) {
        return isAtTopSide(x, y, center, width, height, 0.0);
    }

    /**
     * Indicates if provided point is located at top side of rectangle defined
     * by provided center and size values up to a certain threshold.
     * A positive threshold moves top border downwards, a negative threshold
     * moves top border upwards.
     *
     * @param point     point to be checked.
     * @param center    center of rectangle.
     * @param width     width of rectangle.
     * @param height    height of rectangle.
     * @param threshold threshold to use as a margin to determine whether point
     *                  lies at top side or not.
     * @return true if point is at top side, false otherwise.
     */
    public static boolean isAtTopSide(
            final Point2D point, final Point2D center,
            final double width, final double height, final double threshold) {
        return isAtTopSide(point.getInhomX(), point.getInhomY(), center, width,
                height, threshold);
    }

    /**
     * Indicates if provided point is located at top side of rectangle defined
     * by provided center and size values.
     *
     * @param point  point to be checked.
     * @param center center of rectangle.
     * @param width  width of rectangle.
     * @param height height of rectangle.
     * @return true if point is at top side, false otherwise.
     */
    public static boolean isAtTopSide(
            final Point2D point, final Point2D center,
            final double width, final double height) {
        return isAtTopSide(point, center, width, height, 0.0);
    }

    /**
     * Indicates if provided point coordinates are located at top side of this
     * rectangle up to a certain threshold.
     * A positive threshold moves top border downwards, a negative threshold
     * moves top border upwards.
     *
     * @param x         x coordinate of point to be checked.
     * @param y         y coordinate of point to be checked.
     * @param threshold threshold to use as a margin to determine whether point
     *                  lies at top side or not.
     * @return true if point is at top side, false otherwise.
     */
    public boolean isAtTopSide(
            final double x, final double y, final double threshold) {
        return isAtTopSide(x, y, mTopLeft, mBottomRight, threshold);
    }

    /**
     * Indicates if provided point coordinates are located at top side of this
     * rectangle.
     *
     * @param x x coordinate of point to be checked.
     * @param y y coordinate of point to be checked.
     * @return true if point is at top side, false otherwise.
     */
    public boolean isAtTopSide(final double x, final double y) {
        return isAtTopSide(x, y, 0.0);
    }

    /**
     * Indicates if provided point is located at top side of this rectangle up
     * to a certain threshold.
     * A positive threshold moves top border downwards, a negative threshold
     * moves top border upwards.
     *
     * @param point     point to be checked.
     * @param threshold threshold to use as a margin to determine whether point
     *                  lies at top side or not.
     * @return true if point is at top side, false otherwise.
     */
    public boolean isAtTopSide(final Point2D point, final double threshold) {
        return isAtTopSide(point.getInhomX(), point.getInhomY(), threshold);
    }

    /**
     * Indicates if provided point is located at top side of this rectangle.
     *
     * @param point point to be checked.
     * @return true if point is at top side, false otherwise.
     */
    public boolean isAtTopSide(final Point2D point) {
        return isAtTopSide(point, 0.0);
    }

    /**
     * Indicates if provided point coordinates are located at top-right corner
     * of rectangle defined by provided top-left and bottom-right corners up to
     * a certain threshold.
     * A positive threshold moves top-right corner towards rectangle interior,
     * a negative threshold moves top-right corner towards rectangle exterior.
     *
     * @param x         x coordinate of point to be checked.
     * @param y         y coordinate of point to be checked.
     * @param left      left coordinate of rectangle.
     * @param top       top coordinate of rectangle.
     * @param right     right coordinate of rectangle.
     * @param bottom    bottom coordinate of rectangle.
     * @param threshold threshold to use as a margin to determine whether point
     *                  lies at top-right corner or not.
     * @return true if point is at top-right corner, false otherwise.
     */
    public static boolean isAtTopRightCorner(
            final double x, final double y, final double left,
            final double top, final double right, final double bottom,
            final double threshold) {
        // fix values in case that corners are reversed
        final double top2 = Math.max(top, bottom);
        final double right2 = Math.max(left, right);

        return x > (right2 - threshold) && y > (top2 - threshold);
    }

    /**
     * Indicates if point at provided coordinates is located at top-right corner
     * of rectangle defined by provided top-left and bottom-right corners.
     *
     * @param x      x coordinate of point to be checked.
     * @param y      y coordinate of point to be checked.
     * @param left   left coordinate of rectangle.
     * @param top    top coordinate of rectangle.
     * @param right  right coordinate of rectangle.
     * @param bottom bottom coordinate of rectangle.
     * @return true if point is at top-right corner, false otherwise.
     */
    public static boolean isAtTopRightCorner(
            final double x, final double y, final double left,
            final double top, final double right, final double bottom) {
        return isAtTopRightCorner(x, y, left, top, right, bottom, 0.0);
    }

    /**
     * Indicates if provided point is located at top-right corner of rectangle
     * defined by provided top-left and bottom-right corners up to a certain
     * threshold.
     * A positive threshold moves top-right corner towards rectangle interior,
     * a negative threshold moves top-right corner towards rectangle exterior.
     *
     * @param point     point to be checked.
     * @param left      left coordinate of rectangle.
     * @param top       top coordinate of rectangle.
     * @param right     right coordinate of rectangle.
     * @param bottom    bottom coordinate of rectangle.
     * @param threshold threshold to use as a margin to determine whether point
     *                  lies at top-right corner or not.
     * @return true if point is at top-right corner, false otherwise.
     */
    public static boolean isAtTopRightCorner(
            final Point2D point, final double left,
            final double top, final double right, final double bottom,
            final double threshold) {
        return isAtTopRightCorner(point.getInhomX(), point.getInhomY(), left,
                top, right, bottom, threshold);
    }

    /**
     * Indicates if provided point is located at top-right corner of rectangle
     * defined by provided top-left and bottom-right corners.
     *
     * @param point  point to be checked.
     * @param left   left coordinate of rectangle.
     * @param top    top coordinate of rectangle.
     * @param right  right coordinate of rectangle.
     * @param bottom bottom coordinate of rectangle.
     * @return true if point is at top-right corner, false otherwise.
     */
    public static boolean isAtTopRightCorner(
            final Point2D point, final double left,
            final double top, final double right, final double bottom) {
        return isAtTopRightCorner(point, left, top, right, bottom, 0.0);
    }

    /**
     * Indicates if provided point coordinates are located at top-right corner
     * of rectangle defined by provided top-left and bottom-right corners up to
     * a certain threshold.
     * A positive threshold moves top-right corner towards rectangle interior,
     * a negative threshold moves top-right corner towards rectangle exterior.
     *
     * @param x           x coordinate of point to be checked.
     * @param y           y coordinate of point to be checked.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @param threshold   threshold to use as margin to determine whether point
     *                    lies at top-right corner or not.
     * @return true if point is at top-right corner, false otherwise.
     */
    public static boolean isAtTopRightCorner(
            final double x, final double y,
            final Point2D topLeft, final Point2D bottomRight,
            final double threshold) {
        return isAtTopRightCorner(x, y, topLeft.getInhomX(),
                topLeft.getInhomY(), bottomRight.getInhomX(),
                bottomRight.getInhomY(), threshold);
    }

    /**
     * Indicates if provided point coordinates are located at top-right corner
     * of rectangle defined by provided top-left and bottom-right corners.
     *
     * @param x           x coordinate of point to be checked.
     * @param y           y coordinate of point to be checked.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @return true if point is at top-right corner, false otherwise.
     */
    public static boolean isAtTopRightCorner(
            final double x, final double y,
            final Point2D topLeft, final Point2D bottomRight) {
        return isAtTopRightCorner(x, y, topLeft, bottomRight, 0.0);
    }

    /**
     * Indicates if provided point is located at top-right corner of rectangle
     * defined by provided top-left and bottom-right corners up to a certain
     * threshold.
     * A positive threshold moves top-right corner towards rectangle interior,
     * a negative threshold moves top-right corner towards rectangle exterior.
     *
     * @param point       point to be checked.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @param threshold   threshold to use as a margin to determine whether point
     *                    liest at top-right corner side or not.
     * @return true if point is at top-right corner, false otherwise.
     */
    public static boolean isAtTopRightCorner(
            final Point2D point, final Point2D topLeft,
            final Point2D bottomRight, final double threshold) {
        return isAtTopRightCorner(point.getInhomX(), point.getInhomY(), topLeft,
                bottomRight, threshold);
    }

    /**
     * Indicates if provided point is located at top-right corner of rectangle
     * defined by provided top-left and bottom-right corners.
     *
     * @param point       point to be checked.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @return true if point is at top-left corner, false otherwise.
     */
    public static boolean isAtTopRightCorner(
            final Point2D point, final Point2D topLeft,
            final Point2D bottomRight) {
        return isAtTopRightCorner(point, topLeft, bottomRight, 0.0);
    }

    /**
     * Indicates if provided point coordinates are located at top-right corner
     * of rectangle defined by provided center and size values up to a certain
     * threshold.
     * A positive threshold moves top-right corner towards rectangle interior,
     * a negative threshold moves top-right corner towards rectangle exterior.
     *
     * @param x         x coordinate of point to be checked.
     * @param y         y coordinate of point to be checked.
     * @param center    center of rectangle.
     * @param width     width of rectangle.
     * @param height    height of rectangle.
     * @param threshold threshold to use as a margin to determine whether point
     *                  lies at top-left corner or not.
     * @return true if point is at top-right corner, false otherwise.
     */
    public static boolean isAtTopRightCorner(
            final double x, final double y, final Point2D center,
            final double width, final double height, final double threshold) {
        final double halfWidth = width / 2.0;
        final double halfHeight = height / 2.0;
        final double centerX = center.getInhomX();
        final double centerY = center.getInhomY();
        return isAtTopRightCorner(x, y, centerX - halfWidth,
                centerY - halfHeight, centerX + halfWidth,
                centerY + halfHeight, threshold);
    }

    /**
     * Indicates if provided point coordinates are located at top-right corner
     * of rectangle defined by provided center and size values.
     *
     * @param x      x coordinate of point to be checked.
     * @param y      y coordinate of point to be checked.
     * @param center center of rectangle.
     * @param width  width of rectangle.
     * @param height height of rectangle.
     * @return true if point is at top-right corner, false otherwise.
     */
    public static boolean isAtTopRightCorner(
            final double x, final double y, final Point2D center,
            final double width, final double height) {
        return isAtTopRightCorner(x, y, center, width, height, 0.0);
    }

    /**
     * Indicates if provided point is located at top-right corner of rectangle
     * defined by provided center and size values up to a certain threshold.
     * A positive threshold moves top-right corner towards rectangle interior,
     * a negative threshold moves top-right corner towards rectangle exterior.
     *
     * @param point     point to be checked.
     * @param center    center of rectangle.
     * @param width     width of rectangle.
     * @param height    height of rectangle.
     * @param threshold threshold to use as a margin to determine whether point
     *                  lies at top-right corner or not.
     * @return true if point is at top-right corner, false otherwise.
     */
    public static boolean isAtTopRightCorner(
            final Point2D point, final Point2D center,
            final double width, final double height, final double threshold) {
        return isAtTopRightCorner(point.getInhomX(), point.getInhomY(), center,
                width, height, threshold);
    }

    /**
     * Indicates if provided point is located at top-right corner of rectangle
     * defined by provided center and size values.
     *
     * @param point  point to be checked.
     * @param center center of rectangle.
     * @param width  width of rectangle.
     * @param height height of rectangle.
     * @return true if point is at top-right corner, false otherwise.
     */
    public static boolean isAtTopRightCorner(
            final Point2D point, final Point2D center,
            final double width, final double height) {
        return isAtTopRightCorner(point, center, width, height, 0.0);
    }

    /**
     * Indicates if provided point coordinates are located at top-right corner
     * of this rectangle up to a certain threshold.
     * A positive threshold moves top-right corner towards rectangle interior,
     * a negative threshold moves top-right corner towards rectangle exterior.
     *
     * @param x         x coordinate of point to be checked.
     * @param y         y coordinate of point to be checked.
     * @param threshold threshold to use as a margin to determine whether point
     *                  lies at top-right corner or not.
     * @return true if point is at top-right corner, false otherwise.
     */
    public boolean isAtTopRightCorner(
            final double x, final double y, final double threshold) {
        return isAtTopRightCorner(x, y, mTopLeft, mBottomRight, threshold);
    }

    /**
     * Indicates if provided point coordinates are located at top-right corner
     * of this rectangle.
     *
     * @param x x coordinate of point to be checked.
     * @param y y coordinate of point to be checked.
     * @return true if point is at top-right corner, false otherwise.
     */
    public boolean isAtTopRightCorner(
            final double x, final double y) {
        return isAtTopRightCorner(x, y, 0.0);
    }

    /**
     * Indicates if provided point is located at top-right corner of this
     * rectangle up to a certain threshold.
     * A positive threshold moves top-right corner towards rectangle interior,
     * a negative threshold moves top-right corner towards rectangle exterior.
     *
     * @param point     point to be checked.
     * @param threshold threshold to use as a margin to determine whether point
     *                  lies at top-right corner or not.
     * @return true if point is at top-right corner, false otherwise.
     */
    public boolean isAtTopRightCorner(
            final Point2D point, final double threshold) {
        return isAtTopRightCorner(point.getInhomX(), point.getInhomY(),
                threshold);
    }

    /**
     * Indicates if provided point is located at top-right corner of this
     * rectangle.
     *
     * @param point point to be checked.
     * @return true if point is at top-right corner, false otherwise.
     */
    public boolean isAtTopRightCorner(
            final Point2D point) {
        return isAtTopRightCorner(point, 0.0);
    }

    /**
     * Indicates if provided point coordinates are located at right side of
     * rectangle defined by provided top-left and bottom-right corners up to a
     * certain threshold.
     * A positive threshold moves right border to the left, a negative threshold
     * moves right border to the right.
     *
     * @param x         x coordinate of point to be checked.
     * @param y         y coodinate of point to be checked.
     * @param left      left coordinate of rectangle.
     * @param top       top coordinate of rectangle.
     * @param right     right coordinate of rectangle.
     * @param bottom    bottom coordinate of rectangle.
     * @param threshold threshold to use as a margin to determine whether point
     *                  lies at right side or not.
     * @return true if point is at right side, false otherwise.
     */
    public static boolean isAtRightSide(
            final double x, final double y, final double left,
            final double top, final double right, final double bottom,
            final double threshold) {
        // fix values in case that corners are reversed
        final double right2 = Math.max(left, right);
        final double top2 = Math.max(top, bottom);
        final double bottom2 = Math.min(top, bottom);

        return x > (right2 - threshold) && y >= (bottom2 + threshold) &&
                y <= (top2 - threshold);
    }

    /**
     * Indicates if point at provided coordinates is located at right side of
     * rectangle defined by provided top-left and bottom-right corners.
     *
     * @param x      x coordinate of poitn to be checked.
     * @param y      y coordinate of point to be checked.
     * @param left   left coordinate of rectangle.
     * @param top    top coordinate of rectangle.
     * @param right  right coordinate of rectangle.
     * @param bottom bottom coordinate of rectangle.
     * @return true if point is at right side, false otherwise.
     */
    public static boolean isAtRightSide(
            final double x, final double y, final double left,
            final double top, final double right, final double bottom) {
        return isAtRightSide(x, y, left, top, right, bottom, 0.0);
    }

    /**
     * Indicates if provided point is located at right side of rectangle defined
     * by provided top-left and bottom-right corners up to a certain threshold.
     * A positive threshold moves right border to the left, a negative threshold
     * moves right border to the right.
     *
     * @param point     point to be checked.
     * @param left      left coordinate of rectangle.
     * @param top       top coordinate of rectangle.
     * @param right     right coordinate of rectangle.
     * @param bottom    bottom coordinate of rectangle.
     * @param threshold threshold to use as a margin to determine whether point
     *                  lies at right side or not.
     * @return true if point is at right side, false otherwise.
     */
    public static boolean isAtRightSide(
            final Point2D point, final double left, final double top,
            final double right, final double bottom, final double threshold) {
        return isAtRightSide(point.getInhomX(), point.getInhomY(), left, top,
                right, bottom, threshold);
    }

    /**
     * Indicates if provided point is located at right side of rectangle defined
     * by provided top-left and bottom-right corners.
     *
     * @param point  point to be checked.
     * @param left   left coordinate of rectangle.
     * @param top    top coordinate of rectangle.
     * @param right  right coordinate of rectangle.
     * @param bottom bottom coordinate of rectangle.
     * @return true if point is at right side, false otherwise.
     */
    public static boolean isAtRightSide(
            final Point2D point, final double left, final double top,
            final double right, final double bottom) {
        return isAtRightSide(point, left, top, right, bottom, 0.0);
    }

    /**
     * Indicates if provided point coordinates are located at right side of
     * rectangle defined by provided top-left and bottom-right corners up to a
     * certain threshold.
     * A positive threshold moves right border to the left, a negative threshold
     * moves right border to the right.
     *
     * @param x           x coordinate of point to be checked.
     * @param y           y coordinate of point to be checked.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @param threshold   threshold to use as a margin to determine whether point
     *                    lies at right side or not.
     * @return true if point is at right side, false otherwise.
     */
    public static boolean isAtRightSide(
            final double x, final double y, final Point2D topLeft,
            final Point2D bottomRight, final double threshold) {
        return isAtRightSide(x, y, topLeft.getInhomX(), topLeft.getInhomY(),
                bottomRight.getInhomX(), bottomRight.getInhomY(), threshold);
    }

    /**
     * Indicates if provided point coordinates are located at right side of
     * rectangle defined by provided top-left and bottom-right corners.
     *
     * @param x           x coordinate of point to be checked.
     * @param y           y coordinate of point to be checked.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @return true if point is at right side, false otherwise.
     */
    public static boolean isAtRightSide(
            final double x, final double y, final Point2D topLeft,
            final Point2D bottomRight) {
        return isAtRightSide(x, y, topLeft, bottomRight, 0.0);
    }

    /**
     * Indicates if provided point is located at right side of rectangle defined
     * by provided top-left and bottom-right corners.
     * A positive threshold moves right border to the left, a negative threshold
     * moves right border to the right.
     *
     * @param point       point to be checked.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @param threshold   thrshold to use as a margin to determine whether point
     *                    lies at right side or not.
     * @return true if point lies at right side, false otherwise.
     */
    public static boolean isAtRightSide(
            final Point2D point, final Point2D topLeft,
            final Point2D bottomRight, final double threshold) {
        return isAtRightSide(point.getInhomX(), point.getInhomY(), topLeft,
                bottomRight, threshold);
    }

    /**
     * Indicates if provided point is located at right side of rectangle defined
     * by provided top-left and bottom-right corners.
     *
     * @param point       point to be checked.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @return true if point lies at right side, false otherwise.
     */
    public static boolean isAtRightSide(
            final Point2D point, final Point2D topLeft,
            final Point2D bottomRight) {
        return isAtRightSide(point, topLeft, bottomRight, 0.0);
    }

    /**
     * Indicates if provided point coordinates are located at right side of
     * rectangle defined by provided center and size values.
     * A positive threshold moves right border to the left, a negative threshold
     * moves right border to the right.
     *
     * @param x         x coordinate of point to be checked.
     * @param y         y coordinate of point to be checked.
     * @param center    center of rectangle.
     * @param width     width of rectangle.
     * @param height    height of rectangle.
     * @param threshold threshold to use as a margin to determine whether point
     *                  lies at right side or not.
     * @return true if point is at right side, false otherwise.
     */
    public static boolean isAtRightSide(
            final double x, final double y, final Point2D center,
            final double width, final double height, final double threshold) {
        final double halfWidth = width / 2.0;
        final double halfHeight = height / 2.0;
        final double centerX = center.getInhomX();
        final double centerY = center.getInhomY();
        return isAtRightSide(x, y, centerX - halfWidth, centerY - halfHeight,
                centerX + halfWidth, centerY + halfHeight, threshold);
    }

    /**
     * Indicates if provided point coordinates are located at right side of
     * rectangle defined by provided center and size values.
     *
     * @param x      x coordinate of point to be checked.
     * @param y      y coordinate of point to be checked.
     * @param center center of rectangle.
     * @param width  width of rectangle.
     * @param height height of rectangle.
     * @return true if point is at right side, false otherwise.
     */
    public static boolean isAtRightSide(
            final double x, final double y, final Point2D center,
            final double width, final double height) {
        return isAtRightSide(x, y, center, width, height, 0.0);
    }

    /**
     * Indicates if provided point is located at right side of rectangle defined
     * by provided center and size values up to a certain threshold.
     * A positive threshold moves right border to the left, a negative threshold
     * moves right border to the right.
     *
     * @param point     point to be checked.
     * @param center    center of rectangle.
     * @param width     width of rectangle.
     * @param height    height of rectangle.
     * @param threshold threshold to use as a margin to determine whether point
     *                  lies at left side or not.
     * @return true if point is at left side, false otherwise.
     */
    public static boolean isAtRightSide(
            final Point2D point, final Point2D center,
            final double width, final double height, final double threshold) {
        return isAtRightSide(point.getInhomX(), point.getInhomY(), center,
                width, height, threshold);
    }

    /**
     * Indicates if provided point is located at right side of rectangle defined
     * by provided center and size values.
     *
     * @param point  point to be checked.
     * @param center center of rectangle.
     * @param width  width of rectangle.
     * @param height height of rectangle.
     * @return true if point is at right side, false otherwise.
     */
    public static boolean isAtRightSide(
            final Point2D point, final Point2D center,
            final double width, final double height) {
        return isAtRightSide(point, center, width, height, 0.0);
    }

    /**
     * Indicates if provided point coordinates are located at right side of this
     * rectangle up to a certain threshold.
     * A positive threshold moves right border to the left, a negative threshold
     * moves right border to the right.
     *
     * @param x         x coordinate of point to be checked.
     * @param y         y coordinate of point to be checked.
     * @param threshold threshold to use as a margin to determine whether point
     *                  lies at right side or not.
     * @return true if point is at right side, false otherwise.
     */
    public boolean isAtRightSide(
            final double x, final double y, final double threshold) {
        return isAtRightSide(x, y, mTopLeft, mBottomRight, threshold);
    }

    /**
     * Indicates if provided point coordinates are located at right side of this
     * rectangle.
     *
     * @param x x coordinate of point to be checked.
     * @param y y coordinate of point to be checked.
     * @return true if point is at right side, false otherwise.
     */
    public boolean isAtRightSide(
            final double x, final double y) {
        return isAtRightSide(x, y, 0.0);
    }

    /**
     * Indicates if provided point is located at right side of this rectangle up
     * to a certain threshold.
     * A positive threshold moves right border to the left, a negative threshold
     * moves right border to the right.
     *
     * @param point     point to be checked.
     * @param threshold threshold to use as a margin to determine whether point
     *                  lies at right side or not.
     * @return true if point is at right side, false otherwise.
     */
    public boolean isAtRightSide(
            final Point2D point, final double threshold) {
        return isAtRightSide(point.getInhomX(), point.getInhomY(), threshold);
    }

    /**
     * Indicates if provided point is located at right side of this rectangle.
     *
     * @param point point to be checked.
     * @return true if point is at right side, false otherwise.
     */
    public boolean isAtRightSide(final Point2D point) {
        return isAtRightSide(point, 0.0);
    }

    /**
     * Indicates if provided point coordinates are located at bottom-right
     * corner of rectangle defined by provided top-left and bottom-right corners
     * up to a certain threshold.
     * A positive threshold moves bottom-right corner towards the rectangle
     * interior, a negative threshold moves bottom-right corner towards
     * rectangle exterior.
     *
     * @param x         x coordinate of point to be checked.
     * @param y         y coordinate of point to be checked.
     * @param left      left coordinate of rectangle.
     * @param top       top coordinate of rectangle.
     * @param right     right coordinate of rectangle.
     * @param bottom    bottom coordinate of rectangle.
     * @param threshold threshold to use as a margin to determine whether point
     *                  lies at bototm-right corner, false otherwise
     * @return true if point is at bototm-right corner, false otherwise.
     */
    public static boolean isAtBottomRightCorner(
            final double x, final double y, final double left,
            final double top, final double right, final double bottom,
            final double threshold) {
        // fix values in case that corners are reversed
        final double bottom2 = Math.min(top, bottom);
        final double right2 = Math.max(left, right);

        return x > (right2 - threshold) && y < (bottom2 + threshold);
    }

    /**
     * Indicates if point at provided coordinates is located at bottom-right
     * corner of rectangle defined by provided top-left and bototm-right
     * corners.
     *
     * @param x      x coordinate of point to be checked.
     * @param y      y coordinate of point to be checked.
     * @param left   left coordinate of rectangle.
     * @param top    top coordinate of rectangle.
     * @param right  right coordinate of rectangle.
     * @param bottom bottom coordinate of rectangle.
     * @return true if point is at bottom-right corner, false otherwise.
     */
    public static boolean isAtBottomRightCorner(
            final double x, final double y, final double left,
            final double top, final double right, final double bottom) {
        return isAtBottomRightCorner(x, y, left, top, right, bottom, 0.0);
    }

    /**
     * Indicates if provided point is located at bottom-right corner of
     * rectangle defined by provided top-left and bottom-right corners up to a
     * certain threshold.
     * A positive threshold moves bottom-right corner towards the rectangle
     * interior, a negative threshold moves bottom-right corner towards
     * rectangle exterior.
     *
     * @param point     point to be checked.
     * @param left      left coordinate of rectangle.
     * @param top       top coordinate of rectangle.
     * @param right     right coordinate of rectangle.
     * @param bottom    bottom coordinate of rectangle.
     * @param threshold threshold to use as a margin to determine whether point
     *                  lies at bottom-right or not.
     * @return true if point is at top-left corner, false otherwise.
     */
    public static boolean isAtBottomRightCorner(
            final Point2D point, final double left,
            final double top, final double right, final double bottom,
            final double threshold) {
        return isAtBottomRightCorner(point.getInhomX(), point.getInhomY(), left,
                top, right, bottom, threshold);
    }

    /**
     * Indicates if provided point is located at bottom-right corner of
     * rectangle defined by provided top-left and bottom-right corners.
     *
     * @param point  point to be checked.
     * @param left   left coordinate of rectangle.
     * @param top    top coordinate of rectangle.
     * @param right  right coordinate of rectangle.
     * @param bottom bottom coordinate of rectangle.
     * @return true if point is at top-left corner, false otherwise.
     */
    public static boolean isAtBottomRightCorner(
            final Point2D point, final double left,
            final double top, final double right, final double bottom) {
        return isAtBottomRightCorner(point, left, top, right, bottom, 0.0);
    }

    /**
     * Indicates if provided point coordinates are located at bottom-right
     * corner of rectangle defined by provided top-left and bottom-right corners
     * up to a certain threshold.
     * A positive threshold moves bottom-right corner towards the rectangle
     * interior, a negative threshold moves bottom-right corner towards
     * rectangle exterior.
     *
     * @param x           x coordinate of point to be checked.
     * @param y           y coordinate of point to be checked.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @param threshold   threshold to use as a margin to determine whether point
     *                    lies at bottom-right corner or not.
     * @return true if point is at top-left corner, false otherwise.
     */
    public static boolean isAtBottomRightCorner(
            final double x, final double y,
            final Point2D topLeft, final Point2D bottomRight,
            final double threshold) {
        return isAtBottomRightCorner(x, y, topLeft.getInhomX(),
                topLeft.getInhomY(), bottomRight.getInhomX(),
                bottomRight.getInhomY(), threshold);
    }

    /**
     * Indicates if provided point coordinates are located at bottom-right
     * corner of rectangle defined by provided top-left and bottom-right
     * corners.
     *
     * @param x           x coordinate of point to be checked.
     * @param y           y coordinate of point to be checked.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @return true if point is at bottom-right corner, false otherwise.
     */
    public static boolean isAtBottomRightCorner(
            final double x, final double y,
            final Point2D topLeft, final Point2D bottomRight) {
        return isAtBottomRightCorner(x, y, topLeft, bottomRight, 0.0);
    }

    /**
     * Indicates if provided point is located at bottom-right corner of
     * rectangle defined by provided top-left and bottom-right corners up to a
     * certain threshold.
     * A positive threshold moves bottom-right corner towards the rectangle
     * interior, a negative threshold moves bottom-right corner towards
     * rectangle exterior.
     *
     * @param point       point to be checked.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @param threshold   threshold to use as a margin to determine whether point
     *                    lies at bottom-right corner or not
     * @return true if point is at bottom-right corner, false otherwise.
     */
    public static boolean isAtBottomRightCorner(
            final Point2D point, final Point2D topLeft,
            final Point2D bottomRight, final double threshold) {
        return isAtBottomRightCorner(point.getInhomX(), point.getInhomY(),
                topLeft, bottomRight, threshold);
    }

    /**
     * Indicates if provided point is located at bottom-right corner of
     * rectangle defined by provided top-left and bottom-right corners.
     *
     * @param point       point to be checked.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @return true if point is at bottom-right corner, false otherwise.
     */
    public static boolean isAtBottomRightCorner(
            final Point2D point, final Point2D topLeft,
            final Point2D bottomRight) {
        return isAtBottomRightCorner(point, topLeft, bottomRight, 0.0);
    }

    /**
     * Indicates if provided point is located bottom-right corner of rectangle
     * defined by provided center and size values up to a certain threshold.
     * A positive threshold moves bottom-right corner towards the rectangle
     * interior, a negative threshold moves bottom-right corner towards
     * rectangle exterior.
     *
     * @param x         x coordinate of point to be checked.
     * @param y         y coordinate of point to be checked.
     * @param center    center of rectangle.
     * @param width     width of rectangle.
     * @param height    height of rectangle.
     * @param threshold threshold to use as a margin to determine whether point
     *                  lies at bottom-right corner or not.
     * @return true if point is at bottom-right corner, false otherwise.
     */
    public static boolean isAtBottomRightCorner(
            final double x, final double y,
            final Point2D center, final double width, final double height,
            final double threshold) {
        final double halfWidth = width / 2.0;
        final double halfHeight = height / 2.0;
        final double centerX = center.getInhomX();
        final double centerY = center.getInhomY();
        return isAtBottomRightCorner(x, y, centerX - halfWidth,
                centerY - halfHeight, centerX + halfWidth,
                centerY + halfHeight, threshold);
    }

    /**
     * Indicates if provided point coordinates are located at bottom-right
     * corner of rectangle defined by provided center and size values.
     *
     * @param x      x coordinate of point to be checked.
     * @param y      y coordinate of point to be checked.
     * @param center center of rectangle.
     * @param width  width of rectangle.
     * @param height height of rectangle.
     * @return true if point is at bototm-right corner, false otherwise.
     */
    public static boolean isAtBottomRightCorner(
            final double x, final double y,
            final Point2D center, final double width, final double height) {
        return isAtBottomRightCorner(x, y, center, width, height, 0.0);
    }

    /**
     * Indicates if provided point is located at bottom-right corner of
     * rectangle defined by provided center and size values up to a certain
     * threshold.
     * A positive threshold moves bottom-right corner towards the rectangle
     * interior, a negative threshold moves bottom-right corner towards
     * rectangle exterior.
     *
     * @param point     point to be checked.
     * @param center    center of rectangle.
     * @param width     width of rectangle.
     * @param height    height of rectangle.
     * @param threshold threshold to use as a margin to determine whether point
     *                  lies at bottom-right corner or not.
     * @return true if point is at bottom-right corner, false otherwise.
     */
    public static boolean isAtBottomRightCorner(
            final Point2D point, final Point2D center,
            final double width, final double height, final double threshold) {
        return isAtBottomRightCorner(point.getInhomX(), point.getInhomY(),
                center, width, height, threshold);
    }

    /**
     * indicates if provided point is located at bottom-right corner of
     * rectangle defined by provided center and size values.
     *
     * @param point  point to be checked.
     * @param center center of rectangle.
     * @param width  width of rectangle.
     * @param height height of rectangle.
     * @return true if point is at bottom-right corner, false otherwise.
     */
    public static boolean isAtBottomRightCorner(
            final Point2D point, final Point2D center,
            final double width, final double height) {
        return isAtBottomRightCorner(point, center, width, height, 0.0);
    }

    /**
     * Indicates if provided point coordinates are located at bottom-right
     * corner of this rectangle up to a certain threshold.
     * A positive threshold moves bottom-right corner towards the rectangle
     * interior, a negative threshold moves bottom-right corner towards
     * rectangle exterior.
     *
     * @param x         x coordinate of point to be checked.
     * @param y         y coordinate of point to be checked.
     * @param threshold threshold to use as a margin to determine whether point
     *                  lies at bottom-right corner or not.
     * @return true if point is at bototm-right corner, false otherwise.
     */
    public boolean isAtBottomRightCorner(
            final double x, final double y, final double threshold) {
        return isAtBottomRightCorner(x, y, mTopLeft, mBottomRight, threshold);
    }

    /**
     * Indicates if provided point is located at bottom-right corner of this
     * rectangle.
     *
     * @param x x coordinate of point to be checked.
     * @param y y coordinate of point to be checked.
     * @return true if point is at bototm-right corner, false otherwise.
     */
    public boolean isAtBottomRightCorner(
            final double x, final double y) {
        return isAtBottomRightCorner(x, y, 0.0);
    }

    /**
     * Indicates if provided point is located at bottom-right corner of this
     * rectangle.
     * A positive threshold moves bottom-right corner towards the rectangle
     * interior, a negative threshold moves bottom-right corner towards
     * rectangle exterior.
     *
     * @param point     point to be checked.
     * @param threshold threshold to use as a margin to determine whether point
     *                  lies at bottom-right corner or not.
     * @return true if point is at bottom-right corner, false otherwise.
     */
    public boolean isAtBottomRightCorner(
            final Point2D point, final double threshold) {
        return isAtBottomRightCorner(point.getInhomX(), point.getInhomY(),
                threshold);
    }

    /**
     * Indicates if provided point is located at bottom-right corner of this
     * rectangle.
     *
     * @param point point to be checked.
     * @return true if point is at bottom-right corner, false otherwise.
     */
    public boolean isAtBottomRightCorner(final Point2D point) {
        return isAtBottomRightCorner(point, 0.0);
    }

    /**
     * Indicates if provided point coordinates are located at bottom side of
     * rectangle defined by provided top-left and bottom-right corners up to a
     * certain threshold.
     * A positive threshold moves bottom border upwards, a negative threshold
     * moves bottom border downwards
     *
     * @param x         x coordinate of point to be checked.
     * @param y         y coordinate of point to be checked.
     * @param left      left coordinate of rectangle.
     * @param top       top coordinate of rectangle.
     * @param right     right coordinate of rectangle.
     * @param bottom    bottom coordinate of rectangle.
     * @param threshold threshold to use as a margin to determine whether point
     *                  lies at bottom side or not.
     * @return true if point is at bottom side, false otherwise.
     */
    public static boolean isAtBottomSide(
            final double x, final double y, final double left,
            final double top, final double right, final double bottom, final double threshold) {
        final double bottom2 = Math.min(top, bottom);
        final double left2 = Math.min(left, right);
        final double right2 = Math.max(left, right);

        return y < (bottom2 + threshold) && x >= (left2 + threshold) &&
                x <= (right2 - threshold);
    }

    /**
     * Indicates if point at provided coordinates is located at bottom side of
     * rectangle defined by provided top-left and bottom-right corners.
     *
     * @param x      x coordinate of point to be checked.
     * @param y      y coordinate of point to be checked.
     * @param left   left coordinate of rectangle.
     * @param top    top coordinate of rectangle.
     * @param right  right coordinate of rectangle.
     * @param bottom bottom coordinate of rectangle.
     * @return true if point is at bottom side, false otherwise.
     */
    public static boolean isAtBottomSide(
            final double x, final double y, final double left,
            final double top, final double right, final double bottom) {
        return isAtBottomSide(x, y, left, top, right, bottom, 0.0);
    }

    /**
     * Indicates if provided point is located at bottom side of rectangle
     * defined by provided top-left and bottom-right corners up to a certain
     * threshold.
     * A positive threshold moves bottom border upwards, a negative threshold
     * moves bottom border downwards
     *
     * @param point     point to be checked.
     * @param left      left coordinate of rectangle.
     * @param top       top coordinate of rectangle.
     * @param right     right coordinate of rectangle.
     * @param bottom    bottom coordinate of rectangle.
     * @param threshold threshold to use as a margin to determine whether point
     *                  lies at bottom side or not.
     * @return true if point is at bottom side, false otherwise.
     */
    public static boolean isAtBottomSide(
            final Point2D point, final double left, final double top,
            final double right, final double bottom, final double threshold) {
        return isAtBottomSide(point.getInhomX(), point.getInhomY(), left, top,
                right, bottom, threshold);
    }

    /**
     * Indicates if provided point is located at bottom side of rectangle
     * defined by provided top-left and bottom-right corners.
     *
     * @param point  point to be checked.
     * @param left   left coordinate of rectangle.
     * @param top    top coordinate of rectangle.
     * @param right  right coordinate of rectangle.
     * @param bottom bottom coordinate of rectangle.
     * @return true if point is at bottom side, false otherwise.
     */
    public static boolean isAtBottomSide(
            final Point2D point, final double left, final double top,
            final double right, final double bottom) {
        return isAtBottomSide(point, left, top, right, bottom, 0.0);
    }

    /**
     * Indicates if provided point coordinates are located at bottom side of
     * rectangle defined by provided top-left and bottom-right corners up to a
     * certain threshold.
     * A positive threshold moves bottom border upwards, a negative threshold
     * moves bottom border downwards
     *
     * @param x           x coordinate of point to be checked.
     * @param y           y coordinate of point to be checked.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @param threshold   threshold to use as a margin to determine whether point
     *                    lies at bottom side or not.
     * @return true if point is at bottom side, false otherwise.
     */
    public static boolean isAtBottomSide(
            final double x, final double y, final Point2D topLeft,
            final Point2D bottomRight, final double threshold) {
        return isAtBottomSide(x, y, topLeft.getInhomX(), topLeft.getInhomY(),
                bottomRight.getInhomX(), bottomRight.getInhomY(), threshold);
    }

    /**
     * Indicates if provided point coordinates are located at bottom side of
     * rectangle defined by provided top-left and bottom-right corners.
     *
     * @param x           x coordinate of point to be checked.
     * @param y           y coordinate of point to be checked.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @return true if point is at bottom side, false otherwise.
     */
    public static boolean isAtBottomSide(
            final double x, final double y, final Point2D topLeft,
            final Point2D bottomRight) {
        return isAtBottomSide(x, y, topLeft, bottomRight, 0.0);
    }

    /**
     * Indicates if provided point is located at bottom side of rectangle
     * defined by provided top-left and bottom-right corners.
     * A positive threshold moves bottom border upwards, a negative threshold
     * moves bottom border downwards
     *
     * @param point       point to be checked.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @param threshold   threshold to use as a margin to determine whether point
     *                    lies at bottom side or not.
     * @return true if point is at bottom side, false otherwise.
     */
    public static boolean isAtBottomSide(
            final Point2D point, final Point2D topLeft,
            final Point2D bottomRight, final double threshold) {
        return isAtBottomSide(point.getInhomX(), point.getInhomY(), topLeft,
                bottomRight, threshold);
    }

    /**
     * Indicates if provided point is located at bottom side of rectangle
     * defined by provided top-left and bottom-right corners.
     *
     * @param point       point to be checked.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @return true if point is at bottom side, false otherwise.
     */
    public static boolean isAtBottomSide(
            final Point2D point, final Point2D topLeft,
            final Point2D bottomRight) {
        return isAtBottomSide(point, topLeft, bottomRight, 0.0);
    }

    /**
     * Indicates if provided point coordinates are located at bottom side of
     * rectangle defined by provided center and size values up to a certain
     * threshold.
     * A positive threshold moves bottom border upwards, a negative threshold
     * moves bottom border downwards
     *
     * @param x         x coordinate of point to be checked.
     * @param y         y coordinate of point to be checked.
     * @param center    center of rectangle.
     * @param width     width of rectangle.
     * @param height    height of rectangle.
     * @param threshold threshold to use as a margin to determine whether point
     *                  lies at bototm side or not.
     * @return true if point is at bottom side, false otherwise.
     */
    public static boolean isAtBottomSide(
            final double x, final double y, final Point2D center,
            final double width, final double height, final double threshold) {
        final double halfWidth = width / 2.0;
        final double halfHeight = height / 2.0;
        final double centerX = center.getInhomX();
        final double centerY = center.getInhomY();
        return isAtBottomSide(x, y, centerX - halfWidth, centerY - halfHeight,
                centerX + halfWidth, centerY + halfHeight, threshold);
    }

    /**
     * Indicates if provided point coordinates are located at bottom side of
     * rectangle defined by provided center and size values.
     *
     * @param x      x coordinate of point to be checked.
     * @param y      y coordinate of point to be checked.
     * @param center center of rectangle.
     * @param width  width of rectangle.
     * @param height height of rectangle.
     * @return true if point is at top side, false otherwise.
     */
    public static boolean isAtBottomSide(
            final double x, final double y, final Point2D center,
            final double width, final double height) {
        return isAtBottomSide(x, y, center, width, height, 0.0);
    }

    /**
     * Indicates if provided point is located at bottom side of rectangle
     * defined by provided center and size values up to a certain threshold.
     * A positive threshold moves bottom border upwards, a negative threshold
     * moves bottom border downwards
     *
     * @param point     point to be checked.
     * @param center    center of rectangle.
     * @param width     width of rectangle.
     * @param height    height of rectangle.
     * @param threshold threshold to use as a margin to determine whether point
     *                  lies at bottom side or not.
     * @return true if point is at bottom side, false otherwise.
     */
    public static boolean isAtBottomSide(
            final Point2D point, final Point2D center,
            final double width, final double height, final double threshold) {
        return isAtBottomSide(point.getInhomX(), point.getInhomY(), center,
                width, height, threshold);
    }

    /**
     * Indicates if provided point is located at bottom side of rectangle
     * defined by provided center and size values.
     *
     * @param point  point to be checked.
     * @param center center of rectangle.
     * @param width  width of rectangle.
     * @param height height of rectangle.
     * @return true if point is at bottom side, false otherwise.
     */
    public static boolean isAtBottomSide(
            final Point2D point, final Point2D center,
            final double width, final double height) {
        return isAtBottomSide(point, center, width, height, 0.0);
    }

    /**
     * Indicates if provided point coordinates are located at bottom side of
     * this rectangle up to a certain threshold.
     * A positive threshold moves bottom border upwards, a negative threshold
     * moves bottom border downwards
     *
     * @param x         x coordinate of point to be checked.
     * @param y         y coordinate of point to be checked.
     * @param threshold threshold to use as a margin to determine whether point
     *                  lies at bottom side or not.
     * @return true if point is at bottom side, false otherwise.
     */
    public boolean isAtBottomSide(
            final double x, final double y, final double threshold) {
        return isAtBottomSide(x, y, mTopLeft, mBottomRight, threshold);
    }

    /**
     * Indicates if provided point coordinates are located at bottom side of
     * this rectangle.
     *
     * @param x x coordinate of point to be checked.
     * @param y y coordinate of point to be checked.
     * @return true if point is at bottom side, false otherwise.
     */
    public boolean isAtBottomSide(final double x, final double y) {
        return isAtBottomSide(x, y, 0.0);
    }

    /**
     * Indicates if provided point is located at bottom side of this rectangle.
     * A positive threshold moves bottom border upwards, a negative threshold
     * moves bottom border downwards
     *
     * @param point     point to be checked.
     * @param threshold threshold to use as a margin to determine whether point
     *                  lies at bottom side or not.
     * @return true if point is at bottom side, false otherwise.
     */
    public boolean isAtBottomSide(final Point2D point, final double threshold) {
        return isAtBottomSide(point.getInhomX(), point.getInhomY(), threshold);
    }

    /**
     * Indicates if provided point is located at bottom side of this rectangle.
     *
     * @param point point to be checked.
     * @return true if point is at bottom side, false otherwise.
     */
    public boolean isAtBottomSide(final Point2D point) {
        return isAtBottomSide(point, 0.0);
    }

    /**
     * Indicates if provided point coordinates are located at bottom-left corner
     * of rectangle defined by provided top-left and bottom-right corners up to
     * a certain threshold.
     * A positive threshold moves bottom-right corner towards rectangle
     * interior, a negative threshold moves top-right corner towards rectangle
     * exterior.
     *
     * @param x         x coordinate of point to be checked.
     * @param y         y coordinate of point to be checked.
     * @param left      left coordinate of rectangle.
     * @param top       top coordinate of rectangle.
     * @param right     right coordinate of rectangle.
     * @param bottom    bottom coordinate of rectangle.
     * @param threshold threshold to use as a margin to determine whether point
     *                  lies at bottom-left corner or not.
     * @return true if point is at bottom-left corner, false otherwise.
     */
    public static boolean isAtBottomLeftCorner(
            final double x, final double y, final double left,
            final double top, final double right, final double bottom, final double threshold) {
        // fix values in case that corners are reversed
        final double bottom2 = Math.min(top, bottom);
        final double left2 = Math.min(left, right);

        return x < (left2 + threshold) && y < (bottom2 + threshold);
    }

    /**
     * Indicates if point at provided coordinates is located at bottom-left
     * corner of rectangle defined by provided top-left and bottom-right
     * corners.
     *
     * @param x      x coordinate of point to be checked.
     * @param y      y coordinate of point to be checked.
     * @param left   left coordinate of rectangle.
     * @param top    top coordinate of rectangle.
     * @param right  right coordinate of rectangle.
     * @param bottom bottom coordinate of rectangle.
     * @return true if point is at bottom-left corner, false otherwise.
     */
    public static boolean isAtBottomLeftCorner(
            final double x, final double y, final double left,
            final double top, final double right, final double bottom) {
        return isAtBottomLeftCorner(x, y, left, top, right, bottom, 0.0);
    }

    /**
     * Indicates if provided point is located at bottom-left corner of rectangle
     * defined by provided top-left and bototm-right corners up to a certain
     * threshold.
     * A positive threshold moves bottom-right corner towards rectangle
     * interior, a negative threshold moves top-right corner towards rectangle
     * exterior.
     *
     * @param point     point to be checked.
     * @param left      left coordinate of rectangle.
     * @param top       top coordinate of rectangle.
     * @param right     right coordinate of rectangle.
     * @param bottom    bottom coordinate of rectangle.
     * @param threshold threshold to use as a margin to determine whether point
     *                  lies at bottom-left corner or not.
     * @return true if point is at bottom-left corner, false otherwise.
     */
    public static boolean isAtBottomLeftCorner(
            final Point2D point, final double left,
            final double top, final double right, final double bottom,
            final double threshold) {
        return isAtBottomLeftCorner(point.getInhomX(), point.getInhomY(), left,
                top, right, bottom, threshold);
    }

    /**
     * Indicates if provided point is located at bottom-left corner of rectangle
     * defined by provided top-left and bottom-right corner.
     *
     * @param point  point to be checked.
     * @param left   left coordinate of rectangle.
     * @param top    top coordinate of rectangle.
     * @param right  right coordinate of rectangle.
     * @param bottom bottom coordinate of rectangle.
     * @return true if point is at bottom-left corner, false otherwise.
     */
    public static boolean isAtBottomLeftCorner(
            final Point2D point, final double left,
            final double top, final double right, final double bottom) {
        return isAtBottomLeftCorner(point, left, top, right, bottom, 0.0);
    }

    /**
     * Indicates if provided point coordinates are located at bottom-left corner
     * of rectangle defined by provided top-left and bototm-right corners up to
     * a certain threshold.
     * A positive threshold moves bottom-right corner towards rectangle
     * interior, a negative threshold moves top-right corner towards rectangle
     * exterior.
     *
     * @param x           x coordinate of point to be checked.
     * @param y           y coordinate of point to be checked.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @param threshold   threshold to use as margin to determine whether point
     *                    lies at bottom-left corner or not.
     * @return true if point is at bottom-left corner, false otherwise.
     */
    public static boolean isAtBottomLeftCorner(
            final double x, final double y,
            final Point2D topLeft, final Point2D bottomRight,
            final double threshold) {
        return isAtBottomLeftCorner(x, y, topLeft.getInhomX(),
                topLeft.getInhomY(), bottomRight.getInhomX(),
                bottomRight.getInhomY(), threshold);
    }

    /**
     * Indicates if provided point coordinates are located at bottom-left corner
     * of rectangle defined by provided top-left and bottom-right corners.
     *
     * @param x           x coordinate of point to be checked.
     * @param y           y coordinate of point to be checked.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @return true if point is at bottom-left corner, false otherwise.
     */
    public static boolean isAtBottomLeftCorner(
            final double x, final double y,
            final Point2D topLeft, final Point2D bottomRight) {
        return isAtBottomLeftCorner(x, y, topLeft, bottomRight, 0.0);
    }

    /**
     * Indicates if provided point is located at bottom-left corner of rectangle
     * defined by provided top-left and bottom-right corners up to a certain
     * threshold.
     * A positive threshold moves bottom-right corner towards rectangle
     * interior, a negative threshold moves top-right corner towards rectangle
     * exterior.
     *
     * @param point       point to be checked.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @param threshold   threshold to use as a margin to
     * @return true if point is at bottom-left corner, false otherwise.
     */
    public static boolean isAtBottomLeftCorner(
            final Point2D point, final Point2D topLeft,
            final Point2D bottomRight, final double threshold) {
        return isAtBottomLeftCorner(point.getInhomX(), point.getInhomY(),
                topLeft, bottomRight, threshold);
    }

    /**
     * Indicates if provided point is located at bottom-left corner of rectangle
     * defined by provided top-left and bottom-right corners.
     *
     * @param point       point to be checked.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @return true if point is at bottom-left corner, false otherwise.
     */
    public static boolean isAtBottomLeftCorner(
            final Point2D point, final Point2D topLeft,
            final Point2D bottomRight) {
        return isAtBottomLeftCorner(point, topLeft, bottomRight, 0.0);
    }

    /**
     * Indicates if provided point coordinates are located at bottom-left corner
     * of rectangle defined by provided center and size values up to a certain
     * threshold.
     * A positive threshold moves bottom-right corner towards rectangle
     * interior, a negative threshold moves top-right corner towards rectangle
     * exterior.
     *
     * @param x         x coordinate of point to be checked.
     * @param y         y coordinate of point to be checked.
     * @param center    center of rectangle.
     * @param width     width of rectangle.
     * @param height    height of rectangle.
     * @param threshold threshold to use as a margin to determine whether point
     *                  lies at bototm-left corner or not.
     * @return true if point is at bottom-left corner, false otherwise.
     */
    public static boolean isAtBottomLeftCorner(
            final double x, final double y,
            final Point2D center, final double width, final double height,
            final double threshold) {
        final double halfWidth = width / 2.0;
        final double halfHeight = height / 2.0;
        final double centerX = center.getInhomX();
        final double centerY = center.getInhomY();
        return isAtBottomLeftCorner(x, y, centerX - halfWidth,
                centerY - halfHeight, centerX + halfWidth,
                centerY + halfHeight, threshold);
    }

    /**
     * Indicates if provided point coordinates are located at bottom-left corner
     * of rectangle defined by provided center and size values up to a certain
     * threshold.
     *
     * @param x      x coordinate of point to be checked.
     * @param y      y coordinate of point to be checked.
     * @param center center of rectangle.
     * @param width  width of rectangle.
     * @param height height of rectangle.
     * @return true if point is at bottom-left corner, false otherwise.
     */
    public static boolean isAtBottomLeftCorner(
            final double x, final double y,
            final Point2D center, final double width, final double height) {
        return isAtBottomLeftCorner(x, y, center, width, height, 0.0);
    }

    /**
     * Indicates if provided point is located at bottom-left corner of rectangle
     * defined by provided center and size values up to a certain threshold.
     * A positive threshold moves bottom-right corner towards rectangle
     * interior, a negative threshold moves top-right corner towards rectangle
     * exterior.
     *
     * @param point     point to be checked.
     * @param center    center of rectangle.
     * @param width     width of rectangle.
     * @param height    height of rectangle.
     * @param threshold threshold to use as a margin to determine whether point
     *                  lies at top-right corner or not.
     * @return true if point is at bottom-left corner, false otherwise.
     */
    public static boolean isAtBottomLeftCorner(
            final Point2D point, final Point2D center,
            final double width, final double height, final double threshold) {
        return isAtBottomLeftCorner(point.getInhomX(), point.getInhomY(),
                center, width, height, threshold);
    }

    /**
     * Indicates if provided point is located at bottom-left corner of rectangle
     * defined by provided center and size values.
     *
     * @param point  point to be checked.
     * @param center center of rectangle.
     * @param width  width of rectangle.
     * @param height height of rectangle.
     * @return true if point is at bottom-left corner, false otherwise.
     */
    public static boolean isAtBottomLeftCorner(
            final Point2D point, final Point2D center,
            final double width, final double height) {
        return isAtBottomLeftCorner(point, center, width, height, 0.0);
    }

    /**
     * Indicates if provided point coordinates are located at bottom-left corner
     * of this rectangle up to a certain threshold.
     * A positive threshold moves bottom-right corner towards rectangle
     * interior, a negative threshold moves top-right corner towards rectangle
     * exterior.
     *
     * @param x         x coordinate of point to be checked.
     * @param y         y coordinate of point to be checked.
     * @param threshold threshold to use as a margin to determine whether point
     *                  lies at bottom-left corner or not.
     * @return true if point is at bottom-left corner, false otherwise.
     */
    public boolean isAtBottomLeftCorner(
            final double x, final double y, final double threshold) {
        return isAtBottomLeftCorner(x, y, mTopLeft, mBottomRight, threshold);
    }

    /**
     * Indicates if provided point coordinates are located at bottom-left corner
     * of this rectangle.
     *
     * @param x x coordinate of point to be checked.
     * @param y y coordinate of point to be checked.
     * @return true if point is at bottom-left corner, false otherwise.
     */
    public boolean isAtBottomLeftCorner(final double x, final double y) {
        return isAtBottomLeftCorner(x, y, 0.0);
    }

    /**
     * Indicates if provided point is located at bottom-left corner of this
     * rectangle up to a certain threshold.
     * A positive threshold moves bottom-right corner towards rectangle
     * interior, a negative threshold moves top-right corner towards rectangle
     * exterior.
     *
     * @param point     point to be checked.
     * @param threshold threshold to use as a margin to determine whether point
     *                  lies at bottom-left corner or not.
     * @return true if point is at bottom-left corner, false otherwise.
     */
    public boolean isAtBottomLeftCorner(final Point2D point, final double threshold) {
        return isAtBottomLeftCorner(point.getInhomX(), point.getInhomY(),
                threshold);
    }

    /**
     * Indicates if provided point is located at bottom-left corner of this
     * rectangle.
     *
     * @param point point to be checked.
     * @return true if point is at bottom-left corner, false otherwise.
     */
    public boolean isAtBottomLeftCorner(final Point2D point) {
        return isAtBottomLeftCorner(point, 0.0);
    }

    /**
     * Gets signed distance to closest point in the left border of the
     * rectangle.
     * A negative distance indicates that point is placed inside the rectangle,
     * a positive distance indicates that point is placed outside the rectangle.
     *
     * @param x      x coordinate of point to obtain signed distance for.
     * @param y      y coordinate of point to obtain signed distance for.
     * @param left   left coordinate of rectangle.
     * @param top    top coordinate of rectangle.
     * @param right  right coordinate of rectangle.
     * @param bottom bottom coordinate of rectangle.
     * @return signed distance to closest point in the rectangle left border.
     */
    public static double getSignedDistanceToLeftSide(
            final double x, final double y,
            final double left, final double top, final double right, final double bottom) {
        // fix values in case that corners are reversed
        final double left2 = Math.min(left, right);
        final double right2 = Math.max(left, right);
        final double bottom2 = Math.min(top, bottom);
        final double top2 = Math.max(top, bottom);

        if (isAtLeftSide(x, y, left2, top2, right2, bottom2) ||
                isInside(x, y, left2, top2, right2, bottom2)) {
            // at left, inside
            return left2 - x;
        } else if (isAtRightSide(x, y, left2, top2, right2, bottom2)) {
            // at right
            return x - left2;
        } else if (isAtTopLeftCorner(x, y, left2, top2, right2, bottom2) ||
                isAtTopSide(x, y, left2, top2, right2, bottom2) ||
                isAtTopRightCorner(x, y, left2, top2, right2, bottom2)) {
            // at top or top corners
            final double diffX = x - left2;
            final double diffY = y - top2;
            return Math.sqrt(diffX * diffX + diffY * diffY);
        } else {
            // at bottom or bottom corners
            final double diffX = x - left2;
            final double diffY = y - bottom2;
            return Math.sqrt(diffX * diffX + diffY * diffY);
        }
    }

    /**
     * Gets signed distance to closest point in the left border of the
     * rectangle.
     * A negative distance indicates that point is placed inside the rectangle,
     * a positive distance indicates that point is placed outside the rectangle.
     *
     * @param point  point to obtain signed distance for.
     * @param left   left coordinate of rectangle.
     * @param top    top coordinate of rectangle.
     * @param right  right coordinate of rectangle.
     * @param bottom bottom coordinate of rectangle.
     * @return signed distance to closest point in the rectangle left border.
     */
    public static double getSignedDistanceToLeftSide(
            final Point2D point, final double left,
            final double top, final double right, final double bottom) {
        return getSignedDistanceToLeftSide(point.getInhomX(), point.getInhomY(),
                left, top, right, bottom);
    }

    /**
     * Gets signed distance to closest point in the left border of the
     * rectangle.
     * A negative distance indicates that point is placed inside the rectangle,
     * a positive distance indicates that point is placed outside the rectangle.
     *
     * @param x           x coordinate of point to obtain signed distance for.
     * @param y           y coordinate of point to obtain signed distance for.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @return signed distance to closest point in the rectangle left border.
     */
    public static double getSignedDistanceToLeftSide(
            final double x, final double y,
            final Point2D topLeft, final Point2D bottomRight) {
        return getSignedDistanceToLeftSide(x, y, topLeft.getInhomX(),
                topLeft.getInhomY(), bottomRight.getInhomX(),
                bottomRight.getInhomY());
    }

    /**
     * Gets signed distance to closest point in the left border of the
     * rectangle.
     * A negative distance indicates that point is placed inside the rectangle,
     * a positive distance indicates that point is placed outside the rectangle.
     *
     * @param point       point to obtain signed distance for.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @return signed distance to closest point in the rectangle left border.
     */
    public static double getSignedDistanceToLeftSide(
            final Point2D point, final Point2D topLeft, final Point2D bottomRight) {
        return getSignedDistanceToLeftSide(point.getInhomX(), point.getInhomY(),
                topLeft, bottomRight);
    }

    /**
     * Gets signed distance to closest point in the left border of the
     * rectangle.
     * A negative distance indicates that point is placed inside the rectangle,
     * a positive distance indicates that point is placed outside the rectangle.
     *
     * @param x      x coordinate of point to obtain signed distance for.
     * @param y      y coordinate of point to obtain signed distance for.
     * @param center center of rectangle.
     * @param width  width of rectangle.
     * @param height height of rectangle.
     * @return signed distance to closest point in the rectangle left border.
     */
    public static double getSignedDistanceToLeftSide(
            final double x, final double y,
            final Point2D center, final double width, final double height) {
        final double halfWidth = width / 2.0;
        final double halfHeight = height / 2.0;
        final double centerX = center.getInhomX();
        final double centerY = center.getInhomY();
        return getSignedDistanceToLeftSide(x, y, centerX - halfWidth,
                centerY - halfHeight, centerX + halfWidth,
                centerY + halfHeight);
    }

    /**
     * Gets signed distance to closest point in the left border of the
     * rectangle.
     * A negative distance indicates that point is placed inside the rectangle,
     * a positive distance indicates that point is placed outside the rectangle.
     *
     * @param point  point to obtain signed distance for.
     * @param center center of rectangle.
     * @param width  width of rectangle.
     * @param height height of rectangle.
     * @return signed distance to closest point in the rectangle left border.
     */
    public static double getSignedDistanceToLeftSide(
            final Point2D point, final Point2D center,
            final double width, final double height) {
        return getSignedDistanceToLeftSide(point.getInhomX(), point.getInhomY(),
                center, width, height);
    }

    /**
     * Gets signed distance to closest point in the left border of this
     * rectangle.
     * A negative distance indicates that point is placed inside the rectangle,
     * a positive distance indicates that point is placed outside the rectangle.
     *
     * @param x x coordinate of point to obtain signed distance for.
     * @param y y coordinate of point to obtain signed distance for.
     * @return signed distance to closest point in the rectangle left border.
     */
    public double getSignedDistanceToLeftSide(
            final double x, final double y) {
        return getSignedDistanceToLeftSide(x, y, mTopLeft, mBottomRight);
    }

    /**
     * Gets signed distance to closest point in the left border of this
     * rectangle.
     * A negative distance indicates that point is placed inside the rectangle,
     * a positive distance indicates that point is placed outside the rectangle.
     *
     * @param point point to obtain signed distance for.
     * @return signed distance to closest point in the rectangle left border.
     */
    public double getSignedDistanceToLeftSide(final Point2D point) {
        return getSignedDistanceToLeftSide(point.getInhomX(),
                point.getInhomY());
    }

    /**
     * Gets signed distance to closest point in the top border of the rectangle.
     * A negative distance indicates that point is placed inside the rectangle,
     * a positive distance indicates that point is placed outside the rectangle.
     *
     * @param x      x coordinate of point to obtain signed distance for.
     * @param y      y coordinate of point to obtain signed distance for.
     * @param left   left coordinate of rectangle.
     * @param top    top coordinate of rectangle.
     * @param right  right coordinate of rectangle.
     * @param bottom bottom coordinate of rectangle.
     * @return signed distance to closest point in the rectangle top border.
     */
    public static double getSignedDistanceToTopSide(
            final double x, final double y,
            final double left, final double top,
            final double right, final double bottom) {
        // fix values in case that corners are reversed
        final double left2 = Math.min(left, right);
        final double right2 = Math.max(left, right);
        final double bottom2 = Math.min(top, bottom);
        final double top2 = Math.max(top, bottom);

        if (isAtTopSide(x, y, left2, top2, right2, bottom2) ||
                isInside(x, y, left2, top2, right2, bottom2)) {
            // at top, inside or bottom
            return y - top2;
        } else if (isAtBottomSide(x, y, left2, top2, right2, bottom2)) {
            return top2 - y;
        } else if (isAtTopLeftCorner(x, y, left2, top2, right2, bottom2) ||
                isAtLeftSide(x, y, left2, top2, right2, bottom2) ||
                isAtBottomLeftCorner(x, y, left2, top2, right2, bottom2)) {
            // at left or left corners
            final double diffX = x - left2;
            final double diffY = y - top2;
            return Math.sqrt(diffX * diffX + diffY * diffY);
        } else {
            // at right or right corners
            final double diffX = x - right2;
            final double diffY = y - top2;
            return Math.sqrt(diffX * diffX + diffY * diffY);
        }
    }

    /**
     * Gets signed distance to closest point in the top border of the
     * rectangle.
     * A negative distance indicates that point is placed inside the rectangle,
     * a positive distance indicates that point is placed outside the rectangle.
     *
     * @param point  point to obtain signed distance for.
     * @param left   left coordinate of rectangle.
     * @param top    top coordinate of rectangle.
     * @param right  right coordinate of rectangle.
     * @param bottom bottom coordinate of rectangle.
     * @return signed distance to closest point in the rectangle top border.
     */
    public static double getSignedDistanceToTopSide(
            final Point2D point, final double left, final double top,
            final double right, final double bottom) {
        return getSignedDistanceToTopSide(point.getInhomX(), point.getInhomY(),
                left, top, right, bottom);
    }

    /**
     * Gets signed distance to closest point in the top border of the
     * rectangle.
     * A negative distance indicates that point is placed inside the rectangle,
     * a positive distance indicates that point is placed outside the rectangle.
     *
     * @param x           x coordinate of point to obtain signed distance for.
     * @param y           y coordinate of point to obtain signed distance for.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @return signed distance to closest point in the rectangle top border.
     */
    public static double getSignedDistanceToTopSide(
            final double x, final double y,
            final Point2D topLeft, final Point2D bottomRight) {
        return getSignedDistanceToTopSide(x, y, topLeft.getInhomX(),
                topLeft.getInhomY(), bottomRight.getInhomX(),
                bottomRight.getInhomY());
    }

    /**
     * Gets signed distance to closest point in the top border of the
     * rectangle.
     * A negative distance indicates that point is placed inside the rectangle,
     * a positive distance indicates that point is placed outside the rectangle.
     *
     * @param point       point to obtain signed distance for.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @return signed distance to closest point in the rectangle top border.
     */
    public static double getSignedDistanceToTopSide(
            final Point2D point, final Point2D topLeft, final Point2D bottomRight) {
        return getSignedDistanceToTopSide(point.getInhomX(), point.getInhomY(),
                topLeft, bottomRight);
    }

    /**
     * Gets signed distance to closest point in the top border of the
     * rectangle.
     * A negative distance indicates that point is placed inside the rectangle,
     * a positive distance indicates that point is placed outside the rectangle.
     *
     * @param x      x coordinate of point to obtain signed distance for.
     * @param y      y coordinate of point to obtain signed distance for.
     * @param center center of rectangle.
     * @param width  width of rectangle.
     * @param height height of rectangle.
     * @return signed distance to closest point in the rectangle top border.
     */
    public static double getSignedDistanceToTopSide(
            final double x, final double y,
            final Point2D center, final double width, final double height) {
        final double halfWidth = width / 2.0;
        final double halfHeight = height / 2.0;
        final double centerX = center.getInhomX();
        final double centerY = center.getInhomY();
        return getSignedDistanceToTopSide(x, y, centerX - halfWidth,
                centerY - halfHeight, centerX + halfWidth,
                centerY + halfHeight);
    }

    /**
     * Gets signed distance to closest point in the top border of the
     * rectangle.
     * A negative distance indicates that point is placed inside the rectangle,
     * a positive distance indicates that point is placed outside the rectangle.
     *
     * @param point  point to obtain signed distance for.
     * @param center center of rectangle.
     * @param width  width of rectangle.
     * @param height height of rectangle.
     * @return signed distance to closest point in the rectangle top border.
     */
    public static double getSignedDistanceToTopSide(
            final Point2D point, final Point2D center,
            final double width, final double height) {
        return getSignedDistanceToTopSide(point.getInhomX(), point.getInhomY(),
                center, width, height);
    }

    /**
     * Gets signed distance to closest point in the top border of this
     * rectangle.
     * A negative distance indicates that point is placed inside the rectangle,
     * a positive distance indicates that point is placed outside the rectangle.
     *
     * @param x x coordinate of point to obtain signed distance for.
     * @param y y coordinate of point to obtain signed distance for.
     * @return signed distance to closest point in the rectangle top border.
     */
    public double getSignedDistanceToTopSide(final double x, final double y) {
        return getSignedDistanceToTopSide(x, y, mTopLeft, mBottomRight);
    }

    /**
     * Gets signed distance to closest point in the top border of this
     * rectangle.
     * A negative distance indicates that point is placed inside the rectangle,
     * a positive distance indicates that point is placed outside the rectangle.
     *
     * @param point point to obtain signed distance for.
     * @return signed distance to closest point in the rectangle top border.
     */
    public double getSignedDistanceToTopSide(final Point2D point) {
        return getSignedDistanceToTopSide(point.getInhomX(), point.getInhomY());
    }

    /**
     * Gets signed distance to closest point in the right border of the
     * rectangle.
     * A negative distance indicates that point is placed inside the rectangle,
     * a positive distance indicates that point is placed outside the rectangle.
     *
     * @param x      x coordinate of point to obtain signed distance for.
     * @param y      y coordinate of point to obtain signed distance for.
     * @param left   left coordinate of rectangle.
     * @param top    top coordinate of rectangle.
     * @param right  right coordinate of rectangle.
     * @param bottom bottom coordinate of rectangle.
     * @return signed distance to closest point in the rectangle right border.
     */
    public static double getSignedDistanceToRightSide(
            final double x, final double y,
            final double left, final double top, final double right, final double bottom) {
        // fix values inc ase that corners are reversed
        final double left2 = Math.min(left, right);
        final double right2 = Math.max(left, right);
        final double bottom2 = Math.min(top, bottom);
        final double top2 = Math.max(top, bottom);

        if (isAtRightSide(x, y, left2, top2, right2, bottom2) ||
                isInside(x, y, left2, top2, right2, bottom2)) {
            // at right, inside
            return x - right2;
        } else if (isAtLeftSide(x, y, left2, top2, right2, bottom2)) {
            // at left
            return right2 - x;
        } else if (isAtTopLeftCorner(x, y, left2, top2, right2, bottom2) ||
                isAtTopSide(x, y, left2, top2, right2, bottom2) ||
                isAtTopRightCorner(x, y, left2, top2, right2, bottom2)) {
            // at top or top corners
            final double diffX = x - right2;
            final double diffY = y - top2;
            return Math.sqrt(diffX * diffX + diffY * diffY);
        } else {
            // at bottom or bottom corners
            final double diffX = x - right2;
            final double diffY = y - bottom2;
            return Math.sqrt(diffX * diffX + diffY * diffY);
        }
    }

    /**
     * Gets signed distance to closest point in the right border of the
     * rectangle.
     * A negative distance indicates that point is placed inside the rectangle,
     * a positive distance indicates that point is placed outside the rectangle.
     *
     * @param point  point to obtain signed distance for.
     * @param left   left coordinate of rectangle.
     * @param top    top coordinate of rectangle.
     * @param right  right coordinate of rectangle.
     * @param bottom bottom coordinate of rectangle.
     * @return signed distance to closest point in the rectangle right border.
     */
    public static double getSignedDistanceToRightSide(
            final Point2D point, final double left, final double top,
            final double right, final double bottom) {
        return getSignedDistanceToRightSide(point.getInhomX(),
                point.getInhomY(), left, top, right, bottom);
    }

    /**
     * Gets signed distance to closest point in the right border of the
     * rectangle.
     * A negative distance indicates that point is placed inside the rectangle,
     * a positive distance indicates that point is placed outside the rectangle.
     *
     * @param x           x coordinate of point to obtain signed distance for.
     * @param y           y coordinate of point to obtain signed distance for.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @return signed distance to closest point in the rectangle right border.
     */
    public static double getSignedDistanceToRightSide(
            final double x, final double y,
            final Point2D topLeft, final Point2D bottomRight) {
        return getSignedDistanceToRightSide(x, y, topLeft.getInhomX(),
                topLeft.getInhomY(), bottomRight.getInhomX(),
                bottomRight.getInhomY());
    }

    /**
     * Gets signed distance to closest point in the right border of the
     * rectangle.
     * A negative distance indicates that point is placed inside the rectangle,
     * a positive distance indicates that point is placed outside the rectangle.
     *
     * @param point       point to obtain signed distance for.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @return signed distance to closest point in the rectangle right border.
     */
    public static double getSignedDistanceToRightSide(
            final Point2D point, final Point2D topLeft, final Point2D bottomRight) {
        return getSignedDistanceToRightSide(point.getInhomX(),
                point.getInhomY(), topLeft, bottomRight);
    }

    /**
     * Gets signed distance to closest point in the right border of the
     * rectangle.
     * A negative distance indicates that point is placed inside the rectangle,
     * a positive distance indicates that point is placed outside the rectangle.
     *
     * @param x      x coordinate of point to obtain signed distance for.
     * @param y      y coordinate of point to obtain signed distance for.
     * @param center center of rectangle.
     * @param width  width of rectangle.
     * @param height height of rectangle.
     * @return signed distance to closest point in the rectangle right border.
     */
    public static double getSignedDistanceToRightSide(
            final double x, final double y,
            final Point2D center, final double width, final double height) {
        final double halfWidth = width / 2.0;
        final double halfHeight = height / 2.0;
        final double centerX = center.getInhomX();
        final double centerY = center.getInhomY();
        return getSignedDistanceToRightSide(x, y, centerX - halfWidth,
                centerY - halfHeight, centerX + halfWidth,
                centerY + halfHeight);
    }

    /**
     * Gets signed distance to closest point in the right border of the
     * rectangle.
     * A negative distance indicates that point is placed inside the rectangle,
     * a positive distance indicates that point is placed outside the rectangle.
     *
     * @param point  point to obtain signed distance for.
     * @param center center of rectangle.
     * @param width  width of rectangle.
     * @param height height of rectangle.
     * @return signed distance to closest point in the rectangle right border.
     */
    public static double getSignedDistanceToRightSide(
            final Point2D point, final Point2D center,
            final double width, final double height) {
        return getSignedDistanceToRightSide(point.getInhomX(),
                point.getInhomY(), center, width, height);
    }

    /**
     * Gets signed distance to closest point in the right border of this
     * rectangle.
     * A negative distance indicates that point is placed inside the rectangle,
     * a positive distance indicates that point is placed outside the rectangle.
     *
     * @param x x coordinate of point to obtain signed distance for.
     * @param y y coordinate of point to obtain signed distance for.
     * @return signed distance to closest point in the rectangle right border.
     */
    public double getSignedDistanceToRightSide(final double x, final double y) {
        return getSignedDistanceToRightSide(x, y, mTopLeft, mBottomRight);
    }

    /**
     * Gets signed distance to closest point in the right border of this
     * rectangle.
     * A negative distance indicates that point is placed inside the rectangle,
     * a positive distance indicates that point is placed outside the rectangle.
     *
     * @param point point to obtain signed distance for.
     * @return signed distance to closest point in the rectangle right border.
     */
    public double getSignedDistanceToRightSide(final Point2D point) {
        return getSignedDistanceToRightSide(point.getInhomX(),
                point.getInhomY());
    }

    /**
     * Gets signed distance to closest point in the bottom border of the
     * rectangle.
     * A negative distance indicates that point is placed inside the rectangle,
     * a positive distance indicates that point is placed outside the rectangle.
     *
     * @param x      x coordinate of point to obtain signed distance for.
     * @param y      y coordinate of point to obtain signed distance for.
     * @param left   left coordinate of rectangle.
     * @param top    top coordinate of rectangle.
     * @param right  right coordinate of rectangle.
     * @param bottom bottom coordinate of rectangle.
     * @return signed distance to closest point in the rectangle bottom border.
     */
    public static double getSignedDistanceToBottomSide(
            final double x, final double y,
            final double left, final double top,
            final double right, final double bottom) {
        // fix values in case that corners are reversed
        final double left2 = Math.min(left, right);
        final double right2 = Math.max(left, right);
        final double bottom2 = Math.min(top, bottom);
        final double top2 = Math.max(top, bottom);

        if (isAtBottomSide(x, y, left2, top2, right2, bottom2) ||
                isInside(x, y, left2, top2, right2, bottom2)) {
            // at bottom, inside or top
            return bottom2 - y;
        } else if (isAtTopSide(x, y, left2, top2, right2, bottom2)) {
            // at top
            return y - bottom2;
        } else if (isAtBottomLeftCorner(x, y, left2, top2, right2, bottom2) ||
                isAtLeftSide(x, y, left2, top2, right2, bottom2) ||
                isAtTopLeftCorner(x, y, left2, top2, right2, bottom2)) {
            // at left or left corners
            final double diffX = x - left2;
            final double diffY = y - bottom2;
            return Math.sqrt(diffX * diffX + diffY * diffY);
        } else {
            // at right or right corners
            final double diffX = x - right2;
            final double diffY = y - bottom2;
            return Math.sqrt(diffX * diffX + diffY * diffY);
        }
    }

    /**
     * Gets signed distance to closest point in the bottom border of the
     * rectangle.
     * A negative distance indicates that point is placed inside the rectangle,
     * a positive distance indicates that point is placed outside the rectangle.
     *
     * @param point  point to obtain signed distance for.
     * @param left   left coordinate of rectangle.
     * @param top    top coordinate of rectangle.
     * @param right  right coordinate of rectangle.
     * @param bottom bottom coordinate of rectangle.
     * @return signed distance to closest point in the rectangle bottom border.
     */
    public static double getSignedDistanceToBottomSide(
            final Point2D point, final double left, final double top,
            final double right, final double bottom) {
        return getSignedDistanceToBottomSide(point.getInhomX(),
                point.getInhomY(), left, top, right, bottom);
    }

    /**
     * Gets signed distance to closest point in the bottom border of the
     * rectangle.
     * A negative distance indicates that point is placed inside the rectangle,
     * a positive distance indicates that point is placed outside the rectangle.
     *
     * @param x           x coordinate of point to obtain signed distance for.
     * @param y           y coordinate of point to obtain signed distance for.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @return signed distance to closest point in the rectangle bottom border.
     */
    public static double getSignedDistanceToBottomSide(
            final double x, final double y,
            final Point2D topLeft, final Point2D bottomRight) {
        return getSignedDistanceToBottomSide(x, y, topLeft.getInhomX(),
                topLeft.getInhomY(), bottomRight.getInhomX(),
                bottomRight.getInhomY());
    }

    /**
     * Gets signed distance to closest point in the bottom border of the
     * rectangle.
     * A negative distance indicates that point is placed inside the rectangle,
     * a positive distance indicates that point is placed outside the rectangle.
     *
     * @param point       point to obtain signed distance for.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @return signed distance to closest point in the rectangle bottom border.
     */
    public static double getSignedDistanceToBottomSide(
            final Point2D point, final Point2D topLeft, final Point2D bottomRight) {
        return getSignedDistanceToBottomSide(point.getInhomX(),
                point.getInhomY(), topLeft, bottomRight);
    }

    /**
     * Gets signed distance to closest point in the bottom border of the
     * rectangle.
     * A negative distance indicates that point is placed inside the rectangle,
     * a positive distance indicates that point is placed outside the rectangle.
     *
     * @param x      x coordinate of point to obtain signed distance for.
     * @param y      y coordinate of point to obtain signed distance for.
     * @param center center of rectangle.
     * @param width  width of rectangle.
     * @param height height of rectangle.
     * @return signed distance to closest point in the rectangle bottom border.
     */
    public static double getSignedDistanceToBottomSide(
            final double x, final double y,
            final Point2D center, final double width, final double height) {
        final double halfWidth = width / 2.0;
        final double halfHeight = height / 2.0;
        final double centerX = center.getInhomX();
        final double centerY = center.getInhomY();
        return getSignedDistanceToBottomSide(x, y, centerX - halfWidth,
                centerY - halfHeight, centerX + halfWidth,
                centerY + halfHeight);
    }

    /**
     * Gets signed distance to closest point in the bottom border of the
     * rectangle.
     * A negative distance indicates that point is placed inside the rectangle,
     * a positive distance indicates that point is placed outside the rectangle.
     *
     * @param point  point to obtain signed distance for.
     * @param center center of rectangle.
     * @param width  width of rectangle.
     * @param height height of rectangle.
     * @return signed distance to closest point in the rectangle bottom border.
     */
    public static double getSignedDistanceToBottomSide(
            final Point2D point, final Point2D center,
            final double width, final double height) {
        return getSignedDistanceToBottomSide(point.getInhomX(),
                point.getInhomY(), center, width, height);
    }

    /**
     * Gets signed distance to closest point in the bottom border of this
     * rectangle.
     * A negative distance indicates that point is placed inside the rectangle,
     * a positive distance indicates that point is placed outside the rectangle.
     *
     * @param x x coordinate of point to obtain signed distance for.
     * @param y y coordinate of point to obtain signed distance for.
     * @return signed distance to closest point in the rectangle bottom border.
     */
    public double getSignedDistanceToBottomSide(final double x, final double y) {
        return getSignedDistanceToBottomSide(x, y, mTopLeft, mBottomRight);
    }

    /**
     * Gets signed distance to closest point in the bottom border of this
     * rectangle.
     * A negative distance indicates that point is placed inside the rectangle,
     * a positive distance indicates that point is placed outside the rectangle.
     *
     * @param point point to obtain signed distance for.
     * @return signed distance to closest point in the rectangle bottom border.
     */
    public double getSignedDistanceToBottomSide(final Point2D point) {
        return getSignedDistanceToBottomSide(point.getInhomX(),
                point.getInhomY());
    }

    /**
     * Gets signed distance to closest point in the rectangle locus.
     * A negative distance indicates that point is placed inside the rectangle,
     * a positive distance indicates that point is placed outside the rectangle.
     *
     * @param x      x coordinate of point to obtain signed distance for.
     * @param y      y coordinate of point to obtain signed distance for.
     * @param left   left coordinate of rectangle.
     * @param top    top coordinate of rectangle.
     * @param right  right coordinate of rectangle.
     * @param bottom bottom coordinate of rectangle.
     * @return signed distance to closest point in the rectangle locus.
     */
    public static double getSignedDistance(
            final double x, final double y, final double left,
            final double top, final double right, final double bottom) {
        // fix values in case that corners are reversed
        final double left2 = Math.min(left, right);
        final double right2 = Math.max(left, right);
        final double bottom2 = Math.min(top, bottom);
        final double top2 = Math.max(top, bottom);

        if (isAtLeftSide(x, y, left2, top2, right2, bottom2)) {
            // left side
            return getSignedDistanceToLeftSide(x, y, left2, top2, right2,
                    bottom2);

        } else if (isAtTopSide(x, y, left2, top2, right2, bottom2)) {
            // top side
            return getSignedDistanceToTopSide(x, y, left2, top2, right2,
                    bottom2);

        } else if (isAtRightSide(x, y, left2, top2, right2, bottom2)) {
            // right side
            return getSignedDistanceToRightSide(x, y, left2, top2, right2,
                    bottom2);

        } else if (isAtBottomSide(x, y, left2, top2, right2, bottom2)) {
            // bottom side
            return getSignedDistanceToBottomSide(x, y, left2, top2, right2,
                    bottom2);
        } else if (isAtTopLeftCorner(x, y, left2, top2, right2, bottom2)) {
            // top-left corner
            return getSignedDistanceToLeftSide(x, y, left2, top2, right2,
                    bottom2);

        } else if (isAtTopRightCorner(x, y, left2, top2, right2, bottom2)) {
            // top-right corner
            return getSignedDistanceToTopSide(x, y, left2, top2, right2,
                    bottom2);

        } else if (isAtBottomRightCorner(x, y, left2, top2, right2, bottom2)) {
            // bottom-right corner
            return getSignedDistanceToRightSide(x, y, left2, top2, right2,
                    bottom2);

        } else if (isAtBottomLeftCorner(x, y, left2, top2, right2, bottom2)) {
            // bottom-left corner
            return getSignedDistanceToBottomSide(x, y, left2, top2, right2,
                    bottom2);
        } else {
            // inside. When inside, distance is negative
            // return closest distance to one of the sides (the largest value)
            final double leftDist = getSignedDistanceToLeftSide(x, y, left2, top2,
                    right2, bottom2);
            final double topDist = getSignedDistanceToTopSide(x, y, left2, top2,
                    right2, bottom2);
            final double rightDist = getSignedDistanceToRightSide(x, y, left2, top2,
                    right2, bottom2);
            final double bottomDist = getSignedDistanceToBottomSide(x, y, left2, top2,
                    right2, bottom2);

            double largest = -Double.MAX_VALUE;
            if (leftDist > largest) {
                largest = leftDist;
            }
            if (topDist > largest) {
                largest = topDist;
            }
            if (rightDist > largest) {
                largest = rightDist;
            }
            if (bottomDist > largest) {
                largest = bottomDist;
            }

            return largest;
        }
    }

    /**
     * Gets signed distance to closest point in the rectangle locus.
     * A negative distance indicates that point is placed inside the rectangle,
     * a positive distance indicates that point is placed outside the rectangle.
     *
     * @param point  point to obtain signed distance for.
     * @param left   left coordinate of rectangle.
     * @param top    top coordinate of rectangle.
     * @param right  right coordinate of rectangle.
     * @param bottom bottom coordinate of rectangle.
     * @return signed distance to closest point in the rectangle locus.
     */
    public static double getSignedDistance(
            final Point2D point, final double left,
            final double top, final double right, final double bottom) {
        return getSignedDistance(point.getInhomX(), point.getInhomY(), left,
                top, right, bottom);
    }

    /**
     * Gets signed distance to closest point in the rectangle locus.
     * A negative distance indicates that point is placed inside the rectangle,
     * a positive distance indicates that point is placed outside the rectangle.
     *
     * @param x           x coordinate of point to obtain signed distance for.
     * @param y           y coordinate of point to obtain signed distance for.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @return signed distance to closest point in the rectangle locus.
     */
    public static double getSignedDistance(
            final double x, final double y, final Point2D topLeft,
            final Point2D bottomRight) {
        return getSignedDistance(x, y, topLeft.getInhomX(), topLeft.getInhomY(),
                bottomRight.getInhomX(), bottomRight.getInhomY());
    }

    /**
     * Gets signed distance to closest point in the rectangle locus.
     * A negative distance indicates that point is placed inside the rectangle,
     * a positive distance indicates that point is placed outside the rectangle.
     *
     * @param point       point to obtain signed distance for.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @return signed distance to closest point in the rectangle locus.
     */
    public static double getSignedDistance(
            final Point2D point, final Point2D topLeft,
            final Point2D bottomRight) {
        return getSignedDistance(point.getInhomX(), point.getInhomY(), topLeft,
                bottomRight);
    }

    /**
     * Gets signed distance to closest point in the rectangle locus.
     * A negative distance indicates that point is placed inside the rectangle,
     * a positive distance indicates that point is placed outside the rectangle.
     *
     * @param x      x coordinate of point to obtain signed distance for.
     * @param y      y coordinate of point to obtain signed distance for.
     * @param center center of rectangle.
     * @param width  width of rectangle.
     * @param height height of rectangle.
     * @return signed distance to closest point in the rectangle locus.
     */
    public static double getSignedDistance(
            final double x, final double y, final Point2D center,
            final double width, final double height) {
        final double halfWidth = width / 2.0;
        final double halfHeight = height / 2.0;
        final double centerX = center.getInhomX();
        final double centerY = center.getInhomY();
        return getSignedDistance(x, y, centerX - halfWidth,
                centerY - halfHeight, centerX + halfWidth,
                centerY + halfHeight);
    }

    /**
     * Gets signed distance to closest point in the rectangle locus.
     * A negative distance indicates that point is placed inside the rectangle,
     * a positive distance indicates that point is placed outside the rectangle.
     *
     * @param point  point to obtain signed distance for.
     * @param center center of rectangle.
     * @param width  width of rectangle.
     * @param height height of rectangle.
     * @return signed distance to closest point in the rectangle locus.
     */
    public static double getSignedDistance(
            final Point2D point, final Point2D center,
            final double width, final double height) {
        return getSignedDistance(point.getInhomX(), point.getInhomY(), center,
                width, height);
    }

    /**
     * Gets signed distance to closest point in the rectangle locus.
     * A negative distance indicates that point is placed inside the rectangle,
     * a positive distance indicates that point is placed outside the rectangle.
     *
     * @param x x coordinate of point to obtain signed distance for.
     * @param y y coordinate of point to obtain signed distance for.
     * @return signed distance to closest point in the rectangle locus.
     */
    public double getSignedDistance(final double x, final double y) {
        return getSignedDistance(x, y, mTopLeft, mBottomRight);
    }

    /**
     * Gets signed distance to closest point in the rectangle locus.
     * A negative distance indicates that point is placed inside the rectangle,
     * a positive distance indicates that point is placed outside the rectangle.
     *
     * @param point point to obtain signed distance for.
     * @return signed distance to closest point in the rectangle locus.
     */
    public double getSignedDistance(final Point2D point) {
        return getSignedDistance(point.getInhomX(), point.getInhomY());
    }

    /**
     * Gets distance to closest point in the left border of the rectangle.
     *
     * @param x      x coordinate of point to obtain distance for.
     * @param y      y coordinate of point to obtain distance for.
     * @param left   left coordinate of rectangle.
     * @param top    top coordinate of rectangle.
     * @param right  right coordinate of rectangle.
     * @param bottom bottom coordinate of rectangle.
     * @return distance to closest point in the rectangle left border.
     */
    public static double getDistanceToLeftSide(
            final double x, final double y,
            final double left, final double top, final double right, final double bottom) {
        return Math.abs(getSignedDistanceToLeftSide(x, y, left, top, right,
                bottom));
    }

    /**
     * Gets distance to closest point in the left border of the rectangle.
     *
     * @param point  point to obtain distance for.
     * @param left   left coordinate of rectangle.
     * @param top    top coordinate of rectangle.
     * @param right  right coordinate of rectangle.
     * @param bottom bottom coordinate of rectangle.
     * @return distance to closest point in the rectangle left border.
     */
    public static double getDistanceToLeftSide(
            final Point2D point,
            final double left, final double top, final double right, final double bottom) {
        return Math.abs(getSignedDistanceToLeftSide(point, left, top, right,
                bottom));
    }

    /**
     * Gets distance to closest point in the left border of the rectangle.
     *
     * @param x           x coordinate of point to obtain distance for.
     * @param y           y coordinate of point to obtain distance for.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @return distance to closest point in the rectangle left border.
     */
    public static double getDistanceToLeftSide(
            final double x, final double y,
            final Point2D topLeft, final Point2D bottomRight) {
        return Math.abs(getSignedDistanceToLeftSide(x, y, topLeft,
                bottomRight));
    }

    /**
     * Gets distance to closest point in the left border of the rectangle.
     *
     * @param point       point to obtain distance for.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @return distance to closest point in the rectangle left border.
     */
    public static double getDistanceToLeftSide(
            final Point2D point, final Point2D topLeft,
            final Point2D bottomRight) {
        return Math.abs(getSignedDistanceToLeftSide(point, topLeft,
                bottomRight));
    }

    /**
     * Gets distance to closest point in the left border of the rectangle.
     *
     * @param x      x coordinate of point to obtain distance for.
     * @param y      y coordinate of point to obtain distance for.
     * @param center center of rectangle.
     * @param width  width of rectangle.
     * @param height height of rectangle.
     * @return distance to closest point in the rectangle left border.
     */
    public static double getDistanceToLeftSide(
            final double x, final double y,
            final Point2D center, final double width, final double height) {
        return Math.abs(getSignedDistanceToLeftSide(x, y, center, width,
                height));
    }

    /**
     * Gets distance to closest point in the left border of the rectangle.
     *
     * @param point  point to obtain distance for.
     * @param center center of rectangle.
     * @param width  width of rectangle.
     * @param height height of rectangle.
     * @return distance to closest point in the rectangle left border.
     */
    public static double getDistanceToLeftSide(
            final Point2D point, final Point2D center,
            final double width, final double height) {
        return Math.abs(getSignedDistanceToLeftSide(point, center, width,
                height));
    }

    /**
     * Gets distance to closest point in the left border of this rectangle.
     *
     * @param x x coordinate of point to obtain distance for.
     * @param y y coordinate of point to obtain distance for.
     * @return distance to closest point in the rectangle left border.
     */
    public double getDistanceToLeftSide(final double x, final double y) {
        return Math.abs(getSignedDistanceToLeftSide(x, y));
    }

    /**
     * Gets distance to closest point in the left border of this rectangle.
     *
     * @param point point to obtain distance for.
     * @return distance to closest point in the rectangle left border.
     */
    public double getDistanceToLeftSide(final Point2D point) {
        return Math.abs(getSignedDistanceToLeftSide(point));
    }

    /**
     * Gets distance to closest point in the top border of the rectangle.
     *
     * @param x      x coordinate of point to obtain distance for.
     * @param y      y coordinate of point to obtain distance for.
     * @param left   left coordinate of rectangle.
     * @param top    top coordinate of rectangle.
     * @param right  right coordinate of rectangle.
     * @param bottom bottom coordinate of rectangle.
     * @return distance to closest point in the rectangle top border.
     */
    public static double getDistanceToTopSide(
            final double x, final double y, final double left,
            final double top, final double right, final double bottom) {
        return Math.abs(getSignedDistanceToTopSide(x, y, left, top, right,
                bottom));
    }

    /**
     * Gets distance to closest point in the top border of the rectangle.
     *
     * @param point  point to obtain distance for.
     * @param left   left coordinate of rectangle.
     * @param top    top coordinate of rectangle.
     * @param right  right coordinate of rectangle.
     * @param bottom bottom coordinate of rectangle.
     * @return distance to closest point in the rectangle top border.
     */
    public static double getDistanceToTopSide(
            final Point2D point, final double left,
            final double top, final double right, final double bottom) {
        return Math.abs(getSignedDistanceToTopSide(point, left, top, right,
                bottom));
    }

    /**
     * Gets distance to closest point in the top border of the rectangle.
     *
     * @param x           x coordinate of point to obtain distance for.
     * @param y           y coordinate of point to obtain distance for.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @return distance to closest point in the rectangle top border.
     */
    public static double getDistanceToTopSide(
            final double x, final double y,
            final Point2D topLeft, final Point2D bottomRight) {
        return Math.abs(getSignedDistanceToTopSide(x, y, topLeft, bottomRight));
    }

    /**
     * Gets distance to closest point in the top border of the rectangle.
     *
     * @param point       point to obtain distance for.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @return distance to closest point in the rectangle top border.
     */
    public static double getDistanceToTopSide(
            final Point2D point, final Point2D topLeft,
            final Point2D bottomRight) {
        return Math.abs(getSignedDistanceToTopSide(point, topLeft,
                bottomRight));
    }

    /**
     * Gets distance to closest point in the top border of the rectangle.
     *
     * @param x      x coordinate of point to obtain distance for.
     * @param y      y coordinate of point to obtain distance for.
     * @param center center of rectangle.
     * @param width  width of rectangle.
     * @param height height of rectangle.
     * @return distance to closest point in the rectangle top border.
     */
    public static double getDistanceToTopSide(
            final double x, final double y,
            final Point2D center, final double width, final double height) {
        return Math.abs(getSignedDistanceToTopSide(x, y, center, width,
                height));
    }

    /**
     * Gets distance to closest point in the top border of the rectangle.
     *
     * @param point  point to obtain distance for.
     * @param center center of rectangle.
     * @param width  width of rectangle.
     * @param height height of rectangle.
     * @return distance to closest point in the rectangle top border.
     */
    public static double getDistanceToTopSide(
            final Point2D point, final Point2D center,
            final double width, final double height) {
        return Math.abs(getSignedDistanceToTopSide(point, center, width,
                height));
    }

    /**
     * Gets distance to closest point in the top border of this rectangle.
     *
     * @param x x coordinate of point to obtain distance for.
     * @param y y coordinate of point to obtain distance for.
     * @return distance to closest point in the rectangle top border.
     */
    public double getDistanceToTopSide(final double x, final double y) {
        return Math.abs(getSignedDistanceToTopSide(x, y));
    }

    /**
     * Gets distance to closest point in the top border of this rectangle.
     *
     * @param point point to obtain distance for.
     * @return distance to closest point in the rectangle top border.
     */
    public double getDistanceToTopSide(final Point2D point) {
        return Math.abs(getSignedDistanceToTopSide(point));
    }

    /**
     * Gets distance to closest point in the right border of the rectangle.
     *
     * @param x      x coordinate of point to obtain distance for.
     * @param y      y coordinate of point to obtain distance for.
     * @param left   left coordinate of rectangle.
     * @param top    top coordinate of rectangle.
     * @param right  right coordinate of rectangle.
     * @param bottom bottom coordinate of rectangle.
     * @return distance to closest point in the rectangle right border.
     */
    public static double getDistanceToRightSide(
            final double x, final double y, final double left,
            final double top, final double right, final double bottom) {
        return Math.abs(getSignedDistanceToRightSide(x, y, left, top, right,
                bottom));
    }

    /**
     * Gets distance to closest point in the right border of the rectangle.
     *
     * @param point  point to obtain distance for.
     * @param left   left coordinate of rectangle.
     * @param top    top coordinate of rectangle.
     * @param right  right coordinate of rectangle.
     * @param bottom bottom coordinate of rectangle.
     * @return distance to closest point in the rectangle right border.
     */
    public static double getDistanceToRightSide(
            final Point2D point, final double left,
            final double top, final double right, final double bottom) {
        return Math.abs(getSignedDistanceToRightSide(point, left, top, right,
                bottom));
    }

    /**
     * Gets distance to closest point in the right border of the rectangle.
     *
     * @param x           x coordinate of point to obtain distance for.
     * @param y           y coordinate of point to obtain distance for.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @return distance to closest point in the rectangle right border
     */
    public static double getDistanceToRightSide(
            final double x, final double y,
            final Point2D topLeft, final Point2D bottomRight) {
        return Math.abs(getSignedDistanceToRightSide(x, y, topLeft,
                bottomRight));
    }

    /**
     * Gets distance to closest point in the right border of the rectangle.
     *
     * @param point       point to obtain distance for.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @return distance to closest point in the rectangle right border.
     */
    public static double getDistanceToRightSide(
            final Point2D point, final Point2D topLeft,
            final Point2D bottomRight) {
        return Math.abs(getSignedDistanceToRightSide(point, topLeft,
                bottomRight));
    }

    /**
     * Gets distance to closest point in the right border of the rectangle.
     *
     * @param x      x coordinate of point to obtain distance for.
     * @param y      y coordinate of point to obtain distance for.
     * @param center center of rectangle.
     * @param width  width of rectangle.
     * @param height height of rectangle.
     * @return distance to closest point in the rectangle right border.
     */
    public static double getDistanceToRightSide(
            final double x, final double y,
            final Point2D center, final double width, final double height) {
        return Math.abs(getSignedDistanceToRightSide(x, y, center, width,
                height));
    }

    /**
     * Gets distance to closest point in the right border of the rectangle.
     *
     * @param point  point to obtain distance for.
     * @param center center of rectangle.
     * @param width  width of rectangle.
     * @param height height of rectangle.
     * @return distance to closest point in the rectangle right border.
     */
    public static double getDistanceToRightSide(
            final Point2D point, final Point2D center,
            final double width, final double height) {
        return Math.abs(getSignedDistanceToRightSide(point, center, width,
                height));
    }

    /**
     * Gets distance to closest point in the right border of this rectangle.
     *
     * @param x x coordinate of point to obtain distance for.
     * @param y y coordinate of point to obtain distance for.
     * @return distance to closest point in the rectangle right border.
     */
    public double getDistanceToRightSide(final double x, final double y) {
        return Math.abs(getSignedDistanceToRightSide(x, y));
    }

    /**
     * Gets distance to closest point in the right border of this rectangle.
     *
     * @param point point to obtain distance for.
     * @return distance to closest point in the rectangle right border.
     */
    public double getDistanceToRightSide(final Point2D point) {
        return Math.abs(getSignedDistanceToRightSide(point));
    }

    /**
     * Gets distance to closest point in the bottom border of the rectangle.
     *
     * @param x      x coordinate of point to obtain distance for.
     * @param y      y coordinate of point to obtain distance for.
     * @param left   left coordinate of rectangle.
     * @param top    top coordinate of rectangle.
     * @param right  right coordinate of rectangle.
     * @param bottom bottom coordinate of rectangle.
     * @return distance to closest point in the rectangle bottom border.
     */
    public static double getDistanceToBottomSide(
            final double x, final double y,
            final double left, final double top, final double right, final double bottom) {
        return Math.abs(getSignedDistanceToBottomSide(x, y, left, top, right,
                bottom));
    }

    /**
     * Gets distance to closest point in the bottom border of the rectangle.
     *
     * @param point  point to obtain distance for.
     * @param left   left coordinate of rectangle.
     * @param top    top coordinate of rectangle.
     * @param right  right coordinate of rectangle.
     * @param bottom bottom coordinate of rectangle.
     * @return distance to closest point in the rectangle bottom border.
     */
    public static double getDistanceToBottomSide(
            final Point2D point, final double left,
            final double top, final double right, final double bottom) {
        return Math.abs(getSignedDistanceToBottomSide(point, left, top, right,
                bottom));
    }

    /**
     * Gets distance to closest point in the bottom border of the rectangle.
     *
     * @param x           x coordinate of point to obtain distance for.
     * @param y           y coordinate of point to obtain distance for.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @return distance to closest point in the rectangle bottom border.
     */
    public static double getDistanceToBottomSide(
            final double x, final double y,
            final Point2D topLeft, final Point2D bottomRight) {
        return Math.abs(getSignedDistanceToBottomSide(x, y, topLeft,
                bottomRight));
    }

    /**
     * Gets distance to closest point in the bottom border of the rectangle.
     *
     * @param point       point to obtain distance for.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @return distance to closest point in the rectangle bottom border.
     */
    public static double getDistanceToBottomSide(
            final Point2D point, final Point2D topLeft,
            final Point2D bottomRight) {
        return Math.abs(getSignedDistanceToBottomSide(point, topLeft,
                bottomRight));
    }

    /**
     * Gets distance to closest point in the bottom border of the rectangle.
     *
     * @param x      x coordinate of point to obtain distance for.
     * @param y      y coordinate of point to obtain distance for.
     * @param center center of rectangle.
     * @param width  width of rectangle.
     * @param height height of rectangle.
     * @return distance to closest point in the rectangle bottom border.
     */
    public static double getDistanceToBottomSide(
            final double x, final double y,
            final Point2D center, final double width, final double height) {
        return Math.abs(getSignedDistanceToBottomSide(x, y, center, width,
                height));
    }

    /**
     * Gets distance to closest point in the bottom border of the rectangle.
     *
     * @param point  point to obtain distance for.
     * @param center center of rectangle.
     * @param width  width of rectangle.
     * @param height height of rectangle.
     * @return distance to closest point in the rectangle bottom border.
     */
    public static double getDistanceToBottomSide(
            final Point2D point, final Point2D center,
            final double width, final double height) {
        return Math.abs(getSignedDistanceToBottomSide(point, center, width,
                height));
    }

    /**
     * Gets distance to closest point in the bottom border of this rectangle.
     *
     * @param x x coordinate of point to obtain distance for.
     * @param y y coordinate of point to obtain distance for.
     * @return distance to closest point in the rectangle bottom border.
     */
    public double getDistanceToBottomSide(final double x, final double y) {
        return Math.abs(getSignedDistanceToBottomSide(x, y));
    }

    /**
     * Gets distance to closest point in the bottom border of this rectangle.
     *
     * @param point point to obtain distance for.
     * @return distance to closest point in the rectangle bottom.
     */
    public double getDistanceToBottomSide(final Point2D point) {
        return Math.abs(getSignedDistanceToBottomSide(point));
    }

    /**
     * Gets distance to closest point in the rectangle locus.
     *
     * @param x      x coordinate of point to obtain distance for.
     * @param y      y coordinate of point to obtain distance for.
     * @param left   left coordinate of rectangle.
     * @param top    top coordinate of rectangle.
     * @param right  right coordinate of rectangle.
     * @param bottom bottom coordinate of rectangle.
     * @return distance to closest point in the rectangle locus.
     */
    public static double getDistance(
            final double x, final double y, final double left,
            final double top, final double right, final double bottom) {
        return Math.abs(getSignedDistance(x, y, left, top, right, bottom));
    }

    /**
     * Gets distance to closest point in the rectangle locus.
     *
     * @param point  point to obtain distance for.
     * @param left   left coordinate of rectangle.
     * @param top    top coordinate of rectangle.
     * @param right  right coordinate of rectangle.
     * @param bottom bottom coordinate of rectangle.
     * @return distance to closest point in the rectangle locus.
     */
    public static double getDistance(
            final Point2D point, final double left, final double top,
            final double right, final double bottom) {
        return Math.abs(getSignedDistance(point, left, top, right, bottom));
    }

    /**
     * Gets distance to closest point in the rectangle locus.
     *
     * @param x           x coordinate of point to obtain distance for.
     * @param y           y coordinate of point to obtain distance for.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @return distance to closest point in the rectangle locus.
     */
    public static double getDistance(
            final double x, final double y, final Point2D topLeft,
            final Point2D bottomRight) {
        return Math.abs(getSignedDistance(x, y, topLeft, bottomRight));
    }

    /**
     * Gets distance to closest point in the rectangle locus.
     *
     * @param point       point to obtain distance for.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @return distance to closest point in the rectangle lcous.
     */
    public static double getDistance(
            final Point2D point, final Point2D topLeft,
            final Point2D bottomRight) {
        return Math.abs(getSignedDistance(point, topLeft, bottomRight));
    }

    /**
     * Gets distance to closest point in the rectangle locus.
     *
     * @param x      x coordinate of point to obtain distance for.
     * @param y      y coordinate of point to obtain distance for.
     * @param center center of rectangle.
     * @param width  width of rectangle.
     * @param height height of rectangle.
     * @return distance to closest point in the rectangle locus.
     */
    public static double getDistance(
            final double x, final double y, final Point2D center,
            final double width, final double height) {
        return Math.abs(getSignedDistance(x, y, center, width, height));
    }

    /**
     * Gets distance to closest point in the rectangle locus.
     *
     * @param point  point to obtain distance for.
     * @param center center of rectangle.
     * @param width  width of rectangle.
     * @param height height of rectangle.
     * @return distance to closest point in the rectangle locus.
     */
    public static double getDistance(
            final Point2D point, final Point2D center,
            final double width, final double height) {
        return Math.abs(getSignedDistance(point, center, width, height));
    }

    /**
     * Gets distance to closest point in the rectangle locus.
     *
     * @param x x coordinate of point to obtain distance for.
     * @param y y coordinate of point to obtain distance for.
     * @return distance to closest point in the rectangle locus.
     */
    public double getDistance(final double x, final double y) {
        return Math.abs(getSignedDistance(x, y));
    }

    /**
     * Gets distance to closest point in the rectangle locus.
     *
     * @param point point to obtain distance for.
     * @return distance to closest point in the rectangle locus.
     */
    public double getDistance(final Point2D point) {
        return Math.abs(getSignedDistance(point));
    }

    /**
     * Gets closest point in the rectangle locus defined by provided top-left
     * and bottom-right corners to provided point coordinates.
     *
     * @param x      x point coordinate to find closest point to.
     * @param y      y point coordinate to find closest point to.
     * @param left   left coordinate of rectangle.
     * @param top    top coordinate of rectangle.
     * @param right  right coordinate of rectangle.
     * @param bottom bottom coordinate of rectangle.
     * @param result instance where coordinates of resulting closest point will
     *               be stored.
     */
    public static void closestPoint(
            final double x, final double y, final double left, final double top,
            final double right, final double bottom, final Point2D result) {
        // fix values in case that corners are reversed
        final double left2 = Math.min(left, right);
        final double right2 = Math.max(left, right);
        final double bottom2 = Math.min(top, bottom);
        final double top2 = Math.max(top, bottom);

        if (isAtLeftSide(x, y, left2, top2, right2, bottom2)) {
            // left side
            result.setInhomogeneousCoordinates(left2, y);

        } else if (isAtTopSide(x, y, left2, top2, right2, bottom2)) {
            // top side
            result.setInhomogeneousCoordinates(x, top2);

        } else if (isAtRightSide(x, y, left2, top2, right2, bottom2)) {
            // right side
            result.setInhomogeneousCoordinates(right2, y);

        } else if (isAtBottomSide(x, y, left2, top2, right2, bottom2)) {
            // bottom side
            result.setInhomogeneousCoordinates(x, bottom2);

        } else if (isAtTopLeftCorner(x, y, left2, top2, right2, bottom2)) {
            // top-left corner. Return top-left corner
            result.setInhomogeneousCoordinates(left2, top2);

        } else if (isAtTopRightCorner(x, y, left2, top2, right2, bottom2)) {
            // top-right corner. Return top-right corner
            result.setInhomogeneousCoordinates(right2, top2);

        } else if (isAtBottomRightCorner(x, y, left2, top2, right2, bottom2)) {
            // bottom-right corner. Return bottom-right corner
            result.setInhomogeneousCoordinates(right2, bottom2);

        } else if (isAtBottomLeftCorner(x, y, left, top2, right2, bottom2)) {
            // bottom-left corner. Return bottom-left corner
            result.setInhomogeneousCoordinates(left2, bottom2);

        } else {
            // inside.
            // Pick the side with the shortest distance to a border
            final double leftDist = getDistanceToLeftSide(x, y, left2, top2, right2,
                    bottom2);
            final double topDist = getDistanceToTopSide(x, y, left2, top2, right2,
                    bottom2);
            final double rightDist = getDistanceToRightSide(x, y, left2, top2, right2,
                    bottom2);
            final double bottomDist = getDistanceToBottomSide(x, y, left2, top2,
                    right2, bottom2);

            if (leftDist < topDist && leftDist < rightDist &&
                    leftDist < bottomDist) {
                // pick left side
                result.setInhomogeneousCoordinates(left2, y);

            } else if (topDist < leftDist && topDist < rightDist &&
                    topDist < bottomDist) {
                // pick top side
                result.setInhomogeneousCoordinates(x, top2);

            } else if (rightDist < leftDist && rightDist < topDist &&
                    rightDist < bottomDist) {
                // pick right side
                result.setInhomogeneousCoordinates(right2, y);

            } else {
                // pick bottom side
                result.setInhomogeneousCoordinates(x, bottom2);
            }
        }
    }

    /**
     * Gets closest point in the rectangle locus defined by provided top-left
     * and bottom-right corners to provided point coordinates.
     *
     * @param point  point to find closest point to.
     * @param left   left coordinate of rectangle.
     * @param top    top coordinate of rectangle.
     * @param right  right coordinate of rectangle.
     * @param bottom bottom coordinate of rectangle.
     * @param result instance where coordinates of resulting closest point will
     *               be stored.
     */
    public static void closestPoint(
            final Point2D point, final double left, final double top,
            final double right, final double bottom, final Point2D result) {
        closestPoint(point.getInhomX(), point.getInhomY(), left, top, right,
                bottom, result);
    }

    /**
     * Gets closest point in the rectangle locus defined by provided top-left
     * and bottom-right corners to provided point coordinates.
     *
     * @param x           x point coordinate to find closest point to.
     * @param y           y point coordinate to find closest point to.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @param result      instance where coordinates of resulting closest point will
     *                    be stored.
     */
    public static void closestPoint(
            final double x, final double y, final Point2D topLeft,
            final Point2D bottomRight, final Point2D result) {
        closestPoint(x, y, topLeft.getInhomX(), topLeft.getInhomY(),
                bottomRight.getInhomX(), bottomRight.getInhomY(), result);
    }

    /**
     * Gets closest point in the rectangle locus defined by provided top-left
     * and bottom-right corners to provided point.
     *
     * @param point       point to find closest point to.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @param result      instance where coordinates of resulting closest point will
     *                    be stored.
     */
    public static void closestPoint(
            final Point2D point, final Point2D topLeft,
            final Point2D bottomRight, final Point2D result) {
        closestPoint(point.getInhomX(), point.getInhomY(), topLeft, bottomRight,
                result);
    }

    /**
     * Gets closest point in the rectangle locus defined by provided center and
     * rectangle size.
     *
     * @param x      x point coordinate to find closest point to.
     * @param y      y point coordinate to find closest point to.
     * @param center center of rectangle.
     * @param width  width of rectangle.
     * @param height height of rectangle.
     * @param result instance where coordinates of resulting closest point will
     *               be stored.
     */
    public static void closestPoint(
            final double x, final double y, final Point2D center,
            final double width, final double height, final Point2D result) {
        final double halfWidth = width / 2.0;
        final double halfHeight = height / 2.0;
        final double centerX = center.getInhomX();
        final double centerY = center.getInhomY();
        closestPoint(x, y, centerX - halfWidth, centerY - halfHeight,
                centerX + halfWidth, centerY + halfHeight, result);
    }

    /**
     * Gets closest point in the rectangle locus defined by provided center
     * rectangle size.
     *
     * @param point  point to find closest point to.
     * @param center center of rectangle.
     * @param width  width of rectangle.
     * @param height height of rectangle.
     * @param result instance where coordinates of resulting closest point will
     *               be stored.
     */
    public static void closestPoint(
            final Point2D point, final Point2D center, final double width,
            final double height, final Point2D result) {
        closestPoint(point.getInhomX(), point.getInhomY(), center, width,
                height, result);
    }

    /**
     * Gets closest point in this rectangle locus.
     *
     * @param x      x point coordinate to find closest point to.
     * @param y      y point coordinate to find closest point to.
     * @param result instance where coordinates of resulting closest point will
     *               be stored.
     */
    public void closestPoint(final double x, final double y, final Point2D result) {
        closestPoint(x, y, mTopLeft, mBottomRight, result);
    }

    /**
     * Gets closest point in this rectangle locus.
     *
     * @param point  point to find closest point to.
     * @param result instance where coordinates of resulting closest point will
     *               be stored.
     */
    public void closestPoint(final Point2D point, final Point2D result) {
        closestPoint(point, mTopLeft, mBottomRight, result);
    }

    /**
     * Gets closest point in the rectangle locus defined by provided top-left
     * and bottom-right corners to provided point coordinates.
     *
     * @param x      x point coordinate to find closest point to.
     * @param y      y point coordinate to find closest point to.
     * @param left   left coordinate of rectangle.
     * @param top    top coordinate of rectangle.
     * @param right  right coordinate of rectangle.
     * @param bottom bottom coordinate of rectangle.
     * @return closest point in the retangle locus.
     */
    public static Point2D getClosestPoint(
            final double x, final double y, final double left,
            final double top, final double right, final double bottom) {
        final Point2D result = Point2D.create();
        closestPoint(x, y, left, top, right, bottom, result);
        return result;
    }

    /**
     * Gets closest point in the rectangle locus defined by provided top-left
     * and bottom-right corners to provided point.
     *
     * @param point  point to find closest point to.
     * @param left   left coordinate of rectangle.
     * @param top    top coordinate of rectangle.
     * @param right  right coordinate of rectangle.
     * @param bottom bottom coordinate of rectangle.
     * @return closest point in the rectangle locus.
     */
    public static Point2D getClosestPoint(
            final Point2D point, final double left,
            final double top, final double right, final double bottom) {
        final Point2D result = Point2D.create();
        closestPoint(point, left, top, right, bottom, result);
        return result;
    }

    /**
     * Gets closest point in the rectangle locus defined by provided top-left
     * and bottom-right corners to provided point coordinates.
     *
     * @param x           x point coordinate to find closest point to.
     * @param y           y point coordinate to find closest point to.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @return closest point in the rectangle locus.
     */
    public static Point2D getClosestPoint(
            final double x, final double y, final Point2D topLeft,
            final Point2D bottomRight) {
        final Point2D result = Point2D.create();
        closestPoint(x, y, topLeft, bottomRight, result);
        return result;
    }

    /**
     * Gets closest point in the rectangle locus defined by provided top-left
     * and bottom-right corners to provided point.
     *
     * @param point       point to find closest point to.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @return closest point in the rectangle locus.
     */
    public static Point2D getClosestPoint(
            final Point2D point, final Point2D topLeft,
            final Point2D bottomRight) {
        final Point2D result = Point2D.create();
        closestPoint(point, topLeft, bottomRight, result);
        return result;
    }

    /**
     * Gets closest point in the rectangle locus defined by provided center
     * and rectangle size to provided point.
     *
     * @param x      x point coordinate to find closest point to.
     * @param y      y point coordinate to find closest point to.
     * @param center center of rectangle.
     * @param width  width of rectangle.
     * @param height height of rectangle.
     * @return closest point in the rectangle locus.
     */
    public static Point2D getClosestPoint(
            final double x, final double y, final Point2D center,
            final double width, final double height) {
        final Point2D result = Point2D.create();
        closestPoint(x, y, center, width, height, result);
        return result;
    }

    /**
     * Gets closest point in the rectangle locus defined by provided center
     * and rectangle size to provided point.
     *
     * @param point  point to find closest point to.
     * @param center center of rectangle.
     * @param width  width of rectangle.
     * @param height height of rectangle.
     * @return closest point in the rectangle locus.
     */
    public static Point2D getClosestPoint(
            final Point2D point, final Point2D center,
            final double width, final double height) {
        final Point2D result = Point2D.create();
        closestPoint(point, center, width, height, result);
        return result;
    }

    /**
     * Gets closest point in this rectangle locus to provided point coordinates.
     *
     * @param x x coordinate to find closest point to.
     * @param y y coordinate to find closest point to.
     * @return closest point in the rectangle locus.
     */
    public Point2D getClosestPoint(final double x, final double y) {
        final Point2D result = Point2D.create();
        closestPoint(x, y, result);
        return result;
    }

    /**
     * Gets closest point in this rectangle locus to provided point.
     *
     * @param point point to find closest point to.
     * @return closest point in the rectangle locus.
     */
    public Point2D getClosestPoint(final Point2D point) {
        final Point2D result = Point2D.create();
        closestPoint(point, result);
        return result;
    }

    /**
     * Indicates whether provided point coordinates belong to the locus of the
     * left side of the rectangle defined by provided top-left and bottom-right
     * corners up to a certain threshold.
     *
     * @param x         x coordinate of point to be checked.
     * @param y         y coordinate of point to be checked.
     * @param left      left coordinate of rectangle.
     * @param top       top coordinate of rectangle.
     * @param right     right coordinate of rectangle.
     * @param bottom    bottom coordinate of rectangle.
     * @param threshold threshold to use as a margin to determine whether
     *                  point is locus to left side of rectangle.
     * @return true if point is locus to left side of rectangle, false
     * otherwise.
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public static boolean isLocusToLeftSide(
            final double x, final double y, final double left,
            final double top, final double right, final double bottom,
            final double threshold) {
        if (threshold < 0.0) {
            throw new IllegalArgumentException();
        }

        return getDistanceToLeftSide(x, y, left, top, right, bottom)
                <= threshold;
    }

    /**
     * Indicates whether provided point belongs to the locus of the left side
     * of the rectangle defined by provided top-left and bottom-right corners up
     * to a certain threshold.
     *
     * @param point     point to be checked.
     * @param left      left coordinate of rectangle.
     * @param top       top coordinate of rectangle.
     * @param right     right coordinate of rectangle.
     * @param bottom    bottom coordinate of rectangle.
     * @param threshold threshold to use as a margin to determine whether
     *                  point is locus to left side of rectangle.
     * @return true if point is locus to left side of rectangle, false
     * otherwise.
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public static boolean isLocusToLeftSide(
            final Point2D point, final double left,
            final double top, final double right, final double bottom,
            final double threshold) {
        return isLocusToLeftSide(point.getInhomX(), point.getInhomY(), left,
                top, right, bottom, threshold);
    }

    /**
     * Indicates whether provided point coordinates belong to the locus of the
     * left side of the rectangle defined by provided top-left and bottom-right
     * corners up to a certain threshold.
     *
     * @param x           x coordinate of point to be checked.
     * @param y           y coordinate of point to be checked.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @param threshold   threshold to use as a margin to determine whether point
     *                    is locus to left side of rectangle.
     * @return true if point is locus to left side of rectangle, false
     * otherwise.
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public static boolean isLocusToLeftSide(
            final double x, final double y, final Point2D topLeft,
            final Point2D bottomRight, final double threshold) {
        return isLocusToLeftSide(x, y, topLeft.getInhomX(), topLeft.getInhomY(),
                bottomRight.getInhomX(), bottomRight.getInhomY(), threshold);
    }

    /**
     * Indicates whether provided point belongs to the locus of the left side of
     * the rectangle defined by provided top-left and bottom-right corners up to
     * a certain threshold.
     *
     * @param point       point to be checked.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @param threshold   threshold to use as a margin to determine whether point
     *                    is locus to left side of rectangle.
     * @return true if point is locus to left side of rectangle, false
     * otherwise.
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public static boolean isLocusToLeftSide(
            final Point2D point, final Point2D topLeft,
            final Point2D bottomRight, final double threshold) {
        return isLocusToLeftSide(point.getInhomX(), point.getInhomY(), topLeft,
                bottomRight, threshold);
    }

    /**
     * Indicates whether provided point coordinates belong to the locus of the
     * left side of the rectangle defined by provided rectangle center and size
     * up to a certain threshold.
     *
     * @param x         x coordinate of point to be checked.
     * @param y         y coordinate of point to be checked.
     * @param center    center of rectangle.
     * @param width     width of rectangle.
     * @param height    height of rectangle.
     * @param threshold threshold to use as a margin to determine whether point
     *                  is locus to left side of rectangle.
     * @return true if point is locus to left side of rectangle, false
     * otherwise.
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public static boolean isLocusToLeftSide(
            final double x, final double y, final Point2D center,
            final double width, final double height, final double threshold) {
        final double halfWidth = width / 2.0;
        final double halfHeight = height / 2.0;
        final double centerX = center.getInhomX();
        final double centerY = center.getInhomY();
        return isLocusToLeftSide(x, y, centerX - halfWidth,
                centerY - halfHeight, centerX + halfWidth,
                centerY + halfHeight, threshold);
    }

    /**
     * Indicates whether provided point belongs to the locus of the left side
     * of the rectangle defined by provided rectangle center and size up to a
     * certain threshold.
     *
     * @param point     point to be checked.
     * @param center    center of rectangle.
     * @param width     width of rectangle.
     * @param height    height of rectangle.
     * @param threshold threshold to use as a margin to determine whether point
     *                  is locus to left side of rectangle.
     * @return true if point is locus to left side of rectangle, false
     * otherwise.
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public static boolean isLocusToLeftSide(
            final Point2D point, final Point2D center,
            final double width, final double height, final double threshold) {
        return isLocusToLeftSide(point.getInhomX(), point.getInhomY(), center,
                width, height, threshold);
    }

    /**
     * Indicates whether provided point coordinates belong to the locus of the
     * left side of this rectangle.
     *
     * @param x         x coordinate of point to be checked.
     * @param y         y coordinate of point to be checked.
     * @param threshold threshold to use as a margin to determine whether point
     *                  is locus to left side of rectangle.
     * @return true if point is locus to left side of rectangle, false
     * otherwise.
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public boolean isLocusToLeftSide(
            final double x, final double y, final double threshold) {
        return isLocusToLeftSide(x, y, mTopLeft, mBottomRight, threshold);
    }

    /**
     * Indicates whether provided point belongs to the locus of the left side
     * of this rectangle.
     *
     * @param point     point to be checked.
     * @param threshold threshold to use as a margin to determine whether point
     *                  is locus to left side of rectangle.
     * @return true if point is locus to left side of rectangle, false
     * otherwise.
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public boolean isLocusToLeftSide(
            final Point2D point, final double threshold) {
        return isLocusToLeftSide(point, mTopLeft, mBottomRight, threshold);
    }

    /**
     * Indicates whether provided point coordinates belong to the locus of the
     * left side of the rectangle defined by provided top-left and bottom-right
     * corners.
     *
     * @param x      x coordinate of point to be checked.
     * @param y      y coordinate of point to be checked.
     * @param left   left coordinate of rectangle.
     * @param top    top coordinate of rectangle.
     * @param right  right coordinate of rectangle.
     * @param bottom bottom coordinate of rectangle.
     * @return true if point is locus to left side of rectangle, false
     * otherwise.
     */
    public static boolean isLocusToLeftSide(
            final double x, final double y, final double left,
            final double top, final double right, final double bottom) {
        return isLocusToLeftSide(x, y, left, top, right, bottom,
                DEFAULT_THRESHOLD);
    }

    /**
     * Indicates whether provided point belongs to the locus of the left side
     * of the rectangle defined by provided top-left and bottom-right corners.
     *
     * @param point  point to be checked.
     * @param left   left coordinate of rectangle.
     * @param top    top coordinate of rectangle.
     * @param right  right coordinate of rectangle.
     * @param bottom bottom coordinate of rectangle.
     * @return true if point is locus to left side of rectangle, false
     * otherwise.
     */
    public static boolean isLocusToLeftSide(
            final Point2D point, final double left,
            final double top, final double right, final double bottom) {
        return isLocusToLeftSide(point, left, top, right, bottom,
                DEFAULT_THRESHOLD);
    }

    /**
     * Indicates whether provided point coordinates belong to the locus of the
     * left side of the rectangle defined by provided top-left and bottom-right
     * corners up to a certain threshold.
     *
     * @param x           x coordinate of point to be checked.
     * @param y           y coordinate of point to be checked.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @return true if point is locus to left side of rectangle, false
     * otherwise.
     */
    public static boolean isLocusToLeftSide(
            final double x, final double y, final Point2D topLeft,
            final Point2D bottomRight) {
        return isLocusToLeftSide(x, y, topLeft, bottomRight, DEFAULT_THRESHOLD);
    }

    /**
     * Indicates whether provided point belongs to the locus of the left side of
     * the rectangle defined by provided top-left and bottom-right corners.
     *
     * @param point       point to be checked.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @return true if point is locus to left side of rectangle, false
     * otherwise.
     */
    public static boolean isLocusToLeftSide(
            final Point2D point, final Point2D topLeft,
            final Point2D bottomRight) {
        return isLocusToLeftSide(point, topLeft, bottomRight,
                DEFAULT_THRESHOLD);
    }

    /**
     * Indicates whether provided point coordinates belong to the locus of the
     * left side of the rectangle defined by provided rectangle center and size.
     *
     * @param x      x coordinate of point to be checked.
     * @param y      y coordinate of point to be checked.
     * @param center center of rectangle.
     * @param width  width of rectangle.
     * @param height height of rectangle.
     * @return true if point is locus to left side of rectangle, false
     * otherwise.
     */
    public static boolean isLocusToLeftSide(
            final double x, final double y, final Point2D center,
            final double width, final double height) {
        return isLocusToLeftSide(x, y, center, width, height,
                DEFAULT_THRESHOLD);
    }

    /**
     * Indicates whether provided point belongs to the locus of the left side
     * of the rectangle defined by provided rectangle center and size.
     *
     * @param point  point to be checked.
     * @param center center of rectangle.
     * @param width  width of rectangle.
     * @param height height of rectangle.
     * @return true if point is locus to left side of rectangle, false
     * otherwise.
     */
    public static boolean isLocusToLeftSide(
            final Point2D point, final Point2D center,
            final double width, final double height) {
        return isLocusToLeftSide(point, center, width, height,
                DEFAULT_THRESHOLD);
    }

    /**
     * Indicates whether provided point coordinates belong to the locus of the
     * left side of this rectangle.
     *
     * @param x x coordinate of point to be checked.
     * @param y y coordinate of point to be checked.
     * @return true if point is locus to left side of rectangle, false
     * otherwise.
     */
    public boolean isLocusToLeftSide(
            final double x, final double y) {
        return isLocusToLeftSide(x, y, DEFAULT_THRESHOLD);
    }

    /**
     * Indicates whether provided point belongs to the locus of the left side of
     * this rectangle.
     *
     * @param point point to be checked.
     * @return true if point is locus to left side of rectangle, false
     * otherwise.
     */
    public boolean isLocusToLeftSide(final Point2D point) {
        return isLocusToLeftSide(point, DEFAULT_THRESHOLD);
    }

    /**
     * Indicates whether provided point coordinates belong to the locus of the
     * top side of the rectangle defined by provided top-left and bottom-right
     * corners up to a certain threshold.
     *
     * @param x         x coordinate of point to be checked.
     * @param y         y coordinate of point to be checked.
     * @param left      left coordinate of rectangle.
     * @param top       top coordinate of rectangle.
     * @param right     right coordinate of rectangle.
     * @param bottom    bottom coordinate of rectangle.
     * @param threshold threshold to use as a margin to determine whether
     *                  point is locus to top side of rectangle.
     * @return true if point is locus to top side of rectangle, false
     * otherwise.
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public static boolean isLocusToTopSide(
            final double x, final double y, final double left,
            final double top, final double right, final double bottom,
            final double threshold) {
        if (threshold < 0.0) {
            throw new IllegalArgumentException();
        }

        return getDistanceToTopSide(x, y, left, top, right, bottom)
                <= threshold;
    }

    /**
     * Indicates whether provided point belongs to the locus of the top side
     * of the rectangle defined by provided top-left and bottom-right corners up
     * to a certain threshold.
     *
     * @param point     point to be checked.
     * @param left      left coordinate of rectangle.
     * @param top       top coordinate of rectangle.
     * @param right     right coordinate of rectangle.
     * @param bottom    bottom coordinate of rectangle.
     * @param threshold threshold to use as a margin to determine whether
     *                  point is locus to top side of rectangle.
     * @return true if point is locus to top side of rectangle, false
     * otherwise.
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public static boolean isLocusToTopSide(
            final Point2D point, final double left,
            final double top, final double right, final double bottom,
            final double threshold) {
        return isLocusToTopSide(point.getInhomX(), point.getInhomY(), left,
                top, right, bottom, threshold);
    }

    /**
     * Indicates whether provided point coordinates belong to the locus of the
     * top side of the rectangle defined by provided top-left and bottom-right
     * corners up to a certain threshold.
     *
     * @param x           x coordinate of point to be checked.
     * @param y           y coordinate of point to be checked.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @param threshold   threshold to use as a margin to determine whether point
     *                    is locus to top side of rectangle.
     * @return true if point is locus to top side of rectangle, false
     * otherwise.
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public static boolean isLocusToTopSide(
            final double x, final double y, final Point2D topLeft,
            final Point2D bottomRight, final double threshold) {
        return isLocusToTopSide(x, y, topLeft.getInhomX(), topLeft.getInhomY(),
                bottomRight.getInhomX(), bottomRight.getInhomY(), threshold);
    }

    /**
     * Indicates whether provided point belongs to the locus of the top side of
     * the rectangle defined by provided top-left and bottom-right corners up to
     * a certain threshold.
     *
     * @param point       point to be checked.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @param threshold   threshold to use as a margin to determine whether point
     *                    is locus to top side of rectangle.
     * @return true if point is locus to top side of rectangle, false
     * otherwise.
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public static boolean isLocusToTopSide(
            final Point2D point, final Point2D topLeft,
            final Point2D bottomRight, final double threshold) {
        return isLocusToTopSide(point.getInhomX(), point.getInhomY(), topLeft,
                bottomRight, threshold);
    }

    /**
     * Indicates whether provided point coordinates belong to the locus of the
     * top side of the rectangle defined by provided rectangle center and size
     * up to a certain threshold.
     *
     * @param x         x coordinate of point to be checked.
     * @param y         y coordinate of point to be checked.
     * @param center    center of rectangle.
     * @param width     width of rectangle.
     * @param height    height of rectangle.
     * @param threshold threshold to use as a margin to determine whether point
     *                  is locus to top side of rectangle.
     * @return true if point is locus to top side of rectangle, false
     * otherwise.
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public static boolean isLocusToTopSide(
            final double x, final double y, final Point2D center,
            final double width, final double height, final double threshold) {
        final double halfWidth = width / 2.0;
        final double halfHeight = height / 2.0;
        final double centerX = center.getInhomX();
        final double centerY = center.getInhomY();
        return isLocusToTopSide(x, y, centerX - halfWidth,
                centerY - halfHeight, centerX + halfWidth,
                centerY + halfHeight, threshold);
    }

    /**
     * Indicates whether provided point belongs to the locus of the top side
     * of the rectangle defined by provided rectangle center and size up to a
     * certain threshold.
     *
     * @param point     point to be checked.
     * @param center    center of rectangle.
     * @param width     width of rectangle.
     * @param height    height of rectangle.
     * @param threshold threshold to use as a margin to determine whether point
     *                  is locus to top side of rectangle.
     * @return true if point is locus to top side of rectangle, false
     * otherwise.
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public static boolean isLocusToTopSide(
            final Point2D point, final Point2D center,
            final double width, final double height, final double threshold) {
        return isLocusToTopSide(point.getInhomX(), point.getInhomY(), center,
                width, height, threshold);
    }

    /**
     * Indicates whether provided point coordinates belong to the locus of the
     * top side of this rectangle.
     *
     * @param x         x coordinate of point to be checked.
     * @param y         y coordinate of point to be checked.
     * @param threshold threshold to use as a margin to determine whether point
     *                  is locus to top side of retangle.
     * @return true if point is locus to top side of rectangle, false
     * otherwise.
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public boolean isLocusToTopSide(
            final double x, final double y, final double threshold) {
        return isLocusToTopSide(x, y, mTopLeft, mBottomRight, threshold);
    }

    /**
     * Indicates whether provided point belongs to the locus of the top side
     * of this rectangle.
     *
     * @param point     point to be checked.
     * @param threshold threshold to use as a margin to determine whether point
     *                  is locus to top side of rectangle.
     * @return true if point is locus to top side of rectangle, false
     * otherwise.
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public boolean isLocusToTopSide(final Point2D point, final double threshold) {
        return isLocusToTopSide(point, mTopLeft, mBottomRight, threshold);
    }

    /**
     * Indicates whether provided point coordinates belong to the locus of the
     * top side of the rectangle defined by provided top-left and bottom-right
     * corners.
     *
     * @param x      x coordinate of point to be checked.
     * @param y      y coordinate of point to be checked.
     * @param left   left coordinate of rectangle.
     * @param top    top coordinate of rectangle.
     * @param right  right coordinate of rectangle.
     * @param bottom bottom coordinate of rectangle.
     * @return true if point is locus to top side of rectangle, false
     * otherwise.
     */
    public static boolean isLocusToTopSide(
            final double x, final double y, final double left,
            final double top, final double right, final double bottom) {
        return isLocusToTopSide(x, y, left, top, right, bottom,
                DEFAULT_THRESHOLD);
    }

    /**
     * Indicates whether provided point belongs to the locus of the top side
     * of the rectangle defined by provided top-left and bottom-right corners.
     *
     * @param point  point to be checked.
     * @param left   left coordinate of rectangle.
     * @param top    top coordinate of rectangle.
     * @param right  right coordinate of rectangle.
     * @param bottom bottom coordinate of rectangle.
     * @return true if point is locus to top side of rectangle, false
     * otherwise.
     */
    public static boolean isLocusToTopSide(
            final Point2D point, final double left,
            final double top, final double right, final double bottom) {
        return isLocusToTopSide(point, left, top, right, bottom,
                DEFAULT_THRESHOLD);
    }

    /**
     * Indicates whether provided point coordinates belong to the locus of the
     * top side of the rectangle defined by provided top-left and bottom-right
     * corners up to a certain threshold.
     *
     * @param x           x coordinate of point to be checked.
     * @param y           y coordinate of point to be checked.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @return true if point is locus to top side of rectangle, false
     * otherwise.
     */
    public static boolean isLocusToTopSide(
            final double x, final double y, final Point2D topLeft,
            final Point2D bottomRight) {
        return isLocusToTopSide(x, y, topLeft, bottomRight, DEFAULT_THRESHOLD);
    }

    /**
     * Indicates whether provided point belongs to the locus of the top side of
     * the rectangle defined by provided top-left and bottom-right corners.
     *
     * @param point       point to be checked.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @return true if point is locus to top side of rectangle, false
     * otherwise.
     */
    public static boolean isLocusToTopSide(
            final Point2D point, final Point2D topLeft,
            final Point2D bottomRight) {
        return isLocusToTopSide(point, topLeft, bottomRight, DEFAULT_THRESHOLD);
    }

    /**
     * Indicates whether provided point coordinates belong to the locus of the
     * top side of the rectangle defined by provided rectangle center and size.
     *
     * @param x      x coordinate of point to be checked.
     * @param y      y coordinate of point to be checked.
     * @param center center of rectangle.
     * @param width  width of rectangle.
     * @param height height of rectangle.
     * @return true if point is locus to top side of rectangle, false
     * otherwise.
     */
    public static boolean isLocusToTopSide(
            final double x, final double y, final Point2D center,
            final double width, final double height) {
        return isLocusToTopSide(x, y, center, width, height, DEFAULT_THRESHOLD);
    }

    /**
     * Indicates whether provided point belongs to the locus of the top side
     * of the rectangle defined by provided rectangle center and size.
     *
     * @param point  point to be checked.
     * @param center center of rectangle.
     * @param width  width of rectangle.
     * @param height height of rectangle.
     * @return true if point is locus to top side of rectangle, false
     * otherwise.
     */
    public static boolean isLocusToTopSide(
            final Point2D point, final Point2D center,
            final double width, final double height) {
        return isLocusToTopSide(point, center, width, height,
                DEFAULT_THRESHOLD);
    }

    /**
     * Indicates whether provided point coordinates belong to the locus of the
     * top side of this rectangle.
     *
     * @param x x coordinate of point to be checked.
     * @param y y coordinate of point to be checked.
     * @return true if point is locus to top side of rectangle, false
     * otherwise.
     */
    public boolean isLocusToTopSide(
            final double x, final double y) {
        return isLocusToTopSide(x, y, DEFAULT_THRESHOLD);
    }

    /**
     * Indicates whether provided point belongs to the locus of the top side of
     * this rectangle.
     *
     * @param point point to be checked.
     * @return true if point is locus to top side of rectangle, false
     * otherwise.
     */
    public boolean isLocusToTopSide(final Point2D point) {
        return isLocusToTopSide(point, DEFAULT_THRESHOLD);
    }

    /**
     * Indicates whether provided point coordinates belong to the locus of the
     * right side of the rectangle defined by provided top-left and bottom-right
     * corners up to a certain threshold.
     *
     * @param x         x coordinate of point to be checked.
     * @param y         y coordinate of point to be checked.
     * @param left      left coordinate of rectangle.
     * @param top       top coordinate of rectangle.
     * @param right     right coordinate of rectangle.
     * @param bottom    bottom coordinate of rectangle.
     * @param threshold threshold to use as a margin to determine whether
     *                  point is locus to right side of rectangle.
     * @return true if point is locus to right side of rectangle, false
     * otherwise.
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public static boolean isLocusToRightSide(
            final double x, final double y, final double left,
            final double top, final double right, final double bottom, final double threshold) {
        if (threshold < 0.0) {
            throw new IllegalArgumentException();
        }

        return getDistanceToRightSide(x, y, left, top, right, bottom)
                <= threshold;
    }

    /**
     * Indicates whether provided point belongs to the locus of the right side
     * of the rectangle defined by provided top-left and bottom-right corners up
     * to a certain threshold.
     *
     * @param point     point to be checked.
     * @param left      left coordinate of rectangle.
     * @param top       top coordinate of rectangle.
     * @param right     right coordinate of rectangle.
     * @param bottom    bottom coordinate of rectangle.
     * @param threshold threshold to use as a margin to determine whether
     *                  point is locus to right side of rectangle.
     * @return true if point is locus to right side of rectangle, false
     * otherwise.
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public static boolean isLocusToRightSide(
            final Point2D point, final double left,
            final double top, final double right, final double bottom,
            final double threshold) {
        return isLocusToRightSide(point.getInhomX(), point.getInhomY(), left,
                top, right, bottom, threshold);
    }

    /**
     * Indicates whether provided point coordinates belong to the locus of the
     * right side of the rectangle defined by provided top-left and bottom-right
     * corners up to a certain threshold.
     *
     * @param x           x coordinate of point to be checked.
     * @param y           y coordinate of point to be checked.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @param threshold   threshold to use as a margin to determine whether point
     *                    is locus to right side of rectangle.
     * @return true if point is locus to right side of rectangle, false
     * otherwise.
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public static boolean isLocusToRightSide(
            final double x, final double y,
            final Point2D topLeft, final Point2D bottomRight, final double threshold) {
        return isLocusToRightSide(x, y, topLeft.getInhomX(),
                topLeft.getInhomY(), bottomRight.getInhomX(),
                bottomRight.getInhomY(), threshold);
    }

    /**
     * Indicates whether provided point belongs to the locus of the right side
     * of the rectangle defined by provided top-left and bottom-right corners up
     * to a certain threshold.
     *
     * @param point       point to be checked.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @param threshold   threshold to use as a margin to determine whether point
     *                    is locus to right side of rectangle.
     * @return true if point is locus to right side of rectangle, false
     * otherwise.
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public static boolean isLocusToRightSide(
            final Point2D point, final Point2D topLeft,
            final Point2D bottomRight, final double threshold) {
        return isLocusToRightSide(point.getInhomX(), point.getInhomY(), topLeft,
                bottomRight, threshold);
    }

    /**
     * Indicates whether provided point coordinates belong to the locus of the
     * right side of the rectangle defined by provided rectangle center and size
     * up to a certain threshold.
     *
     * @param x         x coordinate of point to be checked.
     * @param y         y coordinate of point to be checked.
     * @param center    center of rectangle.
     * @param width     width of rectangle.
     * @param height    height of rectangle.
     * @param threshold threshold to use as a margin to determine whether point
     *                  is locus to right side of rectangle.
     * @return true if point is locus to right side of rectangle, false
     * otherwise.
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public static boolean isLocusToRightSide(
            final double x, final double y, final Point2D center,
            final double width, final double height, final double threshold) {
        final double halfWidth = width / 2.0;
        final double halfHeight = height / 2.0;
        final double centerX = center.getInhomX();
        final double centerY = center.getInhomY();
        return isLocusToRightSide(x, y, centerX - halfWidth,
                centerY - halfHeight, centerX + halfWidth,
                centerY + halfHeight, threshold);
    }

    /**
     * Indicates whether provided point belongs to the locus of the right side
     * of the rectangle defined by provided rectangle center and size up to a
     * certain threshold.
     *
     * @param point     point to be checked.
     * @param center    center of rectangle.
     * @param width     width of rectangle.
     * @param height    height of rectangle.
     * @param threshold threshold to use as a margin to determine whether point
     *                  is locus to right side of rectangle.
     * @return true if point is locus to right side of rectangle, false
     * otherwise.
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public static boolean isLocusToRightSide(
            final Point2D point, final Point2D center,
            final double width, final double height, final double threshold) {
        return isLocusToRightSide(point.getInhomX(), point.getInhomY(), center,
                width, height, threshold);
    }

    /**
     * Indicates whether provided point coordinates belong to the locus of the
     * right side of this rectangle.
     *
     * @param x         x coordinate of point to be checked.
     * @param y         y coordinate of point to be checked.
     * @param threshold threshold to use as a margin to determine whether point
     *                  is locus to right side of retangle.
     * @return true if point is locus to right side of rectangle, false
     * otherwise.
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public boolean isLocusToRightSide(
            final double x, final double y, final double threshold) {
        return isLocusToRightSide(x, y, mTopLeft, mBottomRight, threshold);
    }

    /**
     * Indicates whether provided point belongs to the locus of the right side
     * of this rectangle.
     *
     * @param point     point to be checked.
     * @param threshold threshold to use as a margin to determine whether point
     *                  is locus to right side of rectangle.
     * @return true if point is locus to right side of rectangle, false
     * otherwise.
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public boolean isLocusToRightSide(final Point2D point, final double threshold) {
        return isLocusToRightSide(point, mTopLeft, mBottomRight, threshold);
    }

    /**
     * Indicates whether provided point coordinates belong to the locus of the
     * right side of the rectangle defined by provided top-left and bottom-right
     * corners.
     *
     * @param x      x coordinate of point to be checked.
     * @param y      y coordinate of point to be checked.
     * @param left   left coordinate of rectangle.
     * @param top    top coordinate of rectangle.
     * @param right  right coordinate of rectangle.
     * @param bottom bottom coordinate of rectangle.
     * @return true if point is locus to right side of rectangle, false
     * otherwise.
     */
    public static boolean isLocusToRightSide(
            final double x, final double y, final double left,
            final double top, final double right, final double bottom) {
        return isLocusToRightSide(x, y, left, top, right, bottom,
                DEFAULT_THRESHOLD);
    }

    /**
     * Indicates whether provided point belongs to the locus of the right side
     * of the rectangle defined by provided top-left and bottom-right corners.
     *
     * @param point  point to be checked.
     * @param left   left coordinate of rectangle.
     * @param top    top coordinate of rectangle.
     * @param right  right coordinate of rectangle.
     * @param bottom bottom coordinate of rectangle.
     * @return true if point is locus to right side of rectangle, false
     * otherwise.
     */
    public static boolean isLocusToRightSide(
            final Point2D point, final double left,
            final double top, final double right, final double bottom) {
        return isLocusToRightSide(point, left, top, right, bottom,
                DEFAULT_THRESHOLD);
    }

    /**
     * Indicates whether provided point coordinates belong to the locus of the
     * right side of the rectangle defined by provided top-left and bottom-right
     * corners up to a certain threshold.
     *
     * @param x           x coordinate of point to be checked.
     * @param y           y coordinate of point to be checked.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @return true if point is locus to right side of rectangle, false
     * otherwise.
     */
    public static boolean isLocusToRightSide(
            final double x, final double y,
            final Point2D topLeft, final Point2D bottomRight) {
        return isLocusToRightSide(x, y, topLeft, bottomRight,
                DEFAULT_THRESHOLD);
    }

    /**
     * Indicates whether provided point belongs to the locus of the right side
     * of the rectangle defined by provided top-left and bottom-right corners.
     *
     * @param point       point to be checked.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @return true if point is locus to right side of rectangle, false
     * otherwise.
     */
    public static boolean isLocusToRightSide(
            final Point2D point, final Point2D topLeft,
            final Point2D bottomRight) {
        return isLocusToRightSide(point, topLeft, bottomRight,
                DEFAULT_THRESHOLD);
    }

    /**
     * Indicates whether provided point coordinates belong to the locus of the
     * right side of the rectangle defined by provided rectangle center and
     * size.
     *
     * @param x      x coordinate of point to be checked.
     * @param y      y coordinate of point to be checked.
     * @param center center of rectangle.
     * @param width  width of rectangle.
     * @param height height of rectangle.
     * @return true if point is locus to right side of rectangle, false
     * otherwise.
     */
    public static boolean isLocusToRightSide(
            final double x, final double y, final Point2D center,
            final double width, final double height) {
        return isLocusToRightSide(x, y, center, width, height,
                DEFAULT_THRESHOLD);
    }

    /**
     * Indicates whether provided point belongs to the locus of the right side
     * of the rectangle defined by provided rectangle center and size.
     *
     * @param point  point to be checked.
     * @param center center of rectangle.
     * @param width  width of rectangle.
     * @param height height of rectangle.
     * @return true if point is locus to right side of rectangle, false
     * otherwise.
     */
    public static boolean isLocusToRightSide(
            final Point2D point, final Point2D center,
            final double width, final double height) {
        return isLocusToRightSide(point, center, width, height,
                DEFAULT_THRESHOLD);
    }

    /**
     * Indicates whether provided point coordinates belong to the locus of the
     * right side of this rectangle.
     *
     * @param x x coordinate of point to be checked.
     * @param y y coordinate of point to be checked.
     * @return true if point is locus to right side of rectangle, false
     * otherwise.
     */
    public boolean isLocusToRightSide(
            final double x, final double y) {
        return isLocusToRightSide(x, y, DEFAULT_THRESHOLD);
    }

    /**
     * Indicates whether provided point belongs to the locus of the right side
     * of this rectangle.
     *
     * @param point point to be checked.
     * @return true if point is locus to right side of rectangle, false
     * otherwise.
     */
    public boolean isLocusToRightSide(final Point2D point) {
        return isLocusToRightSide(point, DEFAULT_THRESHOLD);
    }

    /**
     * Indicates whether provided point coordinates belong to the locus of the
     * bottom side of the rectangle defined by provided top-left and
     * bottom-right corners up to a certain threshold.
     *
     * @param x         x coordinate of point to be checked.
     * @param y         y coordinate of point to be checked.
     * @param left      left coordinate of rectangle.
     * @param top       top coordinate of rectangle.
     * @param right     right coordinate of rectangle.
     * @param bottom    bottom coordinate of rectangle.
     * @param threshold threshold to use as a margin to determine whether
     *                  point is locus to right side of rectangle.
     * @return true if point is locus to bottom side of rectangle, false
     * otherwise.
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public static boolean isLocusToBottomSide(
            final double x, final double y, final double left,
            final double top, final double right, final double bottom, final double threshold) {
        if (threshold < 0.0) {
            throw new IllegalArgumentException("threshold must be positive");
        }

        return getDistanceToBottomSide(x, y, left, top, right, bottom)
                <= threshold;
    }

    /**
     * Indicates whether provided point belongs to the locus of the bottom side
     * of the rectangle defined by provided top-left and bottom-right corners up
     * to a certain threshold.
     *
     * @param point     point to be checked.
     * @param left      left coordinate of rectangle.
     * @param top       top coordinate of rectangle.
     * @param right     right coordinate of rectangle.
     * @param bottom    bottom coordinate of rectangle.
     * @param threshold threshold to use as a margin to determine whether
     *                  point is locus to bottom side of rectangle.
     * @return true if point is locus to bottom side of rectangle, false
     * otherwise.
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public static boolean isLocusToBottomSide(
            final Point2D point, final double left,
            final double top, final double right, final double bottom, final double threshold) {
        return isLocusToBottomSide(point.getInhomX(), point.getInhomY(), left,
                top, right, bottom, threshold);
    }

    /**
     * Indicates whether provided point coordinates belong to the locus of the
     * bottom side of the rectangle defined by provided top-left and
     * bottom-right
     * corners up to a certain threshold.
     *
     * @param x           x coordinate of point to be checked.
     * @param y           y coordinate of point to be checked.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @param threshold   threshold to use as a margin to determine whether point
     *                    is locus to bottom side of rectangle.
     * @return true if point is locus to bottom side of rectangle, false
     * otherwise.
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public static boolean isLocusToBottomSide(
            final double x, final double y,
            final Point2D topLeft, final Point2D bottomRight, final double threshold) {
        return isLocusToBottomSide(x, y, topLeft.getInhomX(),
                topLeft.getInhomY(), bottomRight.getInhomX(),
                bottomRight.getInhomY(), threshold);
    }

    /**
     * Indicates whether provided point belongs to the locus of the bottom side
     * of the rectangle defined by provided top-left and bottom-right corners up
     * to a certain threshold.
     *
     * @param point       point to be checked.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @param threshold   threshold to use as a margin to determine whether point
     *                    is locus to bottom side of rectangle.
     * @return true if point is locus to bottom side of rectangle, false
     * otherwise.
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public static boolean isLocusToBottomSide(
            final Point2D point, final Point2D topLeft,
            final Point2D bottomRight, final double threshold) {
        return isLocusToBottomSide(point.getInhomX(), point.getInhomY(),
                topLeft, bottomRight, threshold);
    }

    /**
     * Indicates whether provided point coordinates belong to the locus of the
     * bottom side of the rectangle defined by provided rectangle center and size
     * up to a certain threshold.
     *
     * @param x         x coordinate of point to be checked.
     * @param y         y coordinate of point to be checked.
     * @param center    center of rectangle.
     * @param width     width of rectangle.
     * @param height    height of rectangle.
     * @param threshold threshold to use as a margin to determine whether point
     *                  is locus to bottom side of rectangle.
     * @return true if point is locus to bottom side of rectangle, false
     * otherwise.
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public static boolean isLocusToBottomSide(
            final double x, final double y,
            final Point2D center, final double width, final double height,
            final double threshold) {
        final double halfWidth = width / 2.0;
        final double halfHeight = height / 2.0;
        final double centerX = center.getInhomX();
        final double centerY = center.getInhomY();
        return isLocusToBottomSide(x, y, centerX - halfWidth,
                centerY - halfHeight, centerX + halfWidth,
                centerY + halfHeight, threshold);
    }

    /**
     * Indicates whether provided point belongs to the locus of the bottom side
     * of the rectangle defined by provided rectangle center and size up to a
     * certain threshold.
     *
     * @param point     point to be checked.
     * @param center    center of rectangle.
     * @param width     width of rectangle.
     * @param height    height of rectangle.
     * @param threshold threshold to use as a margin to determine whether point
     *                  is locus to bottom side of rectangle.
     * @return true if point is locus to bottom side of rectangle, false
     * otherwise.
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public static boolean isLocusToBottomSide(
            final Point2D point, final Point2D center,
            final double width, final double height, final double threshold) {
        return isLocusToBottomSide(point.getInhomX(), point.getInhomY(), center,
                width, height, threshold);
    }

    /**
     * Indicates whether provided point coordinates belong to the locus of the
     * bottom side of this rectangle.
     *
     * @param x         x coordinate of point to be checked.
     * @param y         y coordinate of point to be checked.
     * @param threshold threshold to use as a margin to determine whether point
     *                  is locus to bottom side of rectangle.
     * @return true if point is locus to bottom side of rectangle, false
     * otherwise.
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public boolean isLocusToBottomSide(
            final double x, final double y, final double threshold) {
        return isLocusToBottomSide(x, y, mTopLeft, mBottomRight, threshold);
    }

    /**
     * Indicates whether provided point belongs to the locus of the bottom side
     * of this rectangle.
     *
     * @param point     point to be checked.
     * @param threshold threshold to use as a margin to determine whether point
     *                  is locus to bottom side of rectangle.
     * @return true if point is locus to bottom side of rectangle, false
     * otherwise.
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public boolean isLocusToBottomSide(
            final Point2D point, final double threshold) {
        return isLocusToBottomSide(point, mTopLeft, mBottomRight, threshold);
    }

    /**
     * Indicates whether provided point coordinates belong to the locus of the
     * bottom side of the rectangle defined by provided top-left and
     * bottom-right corners.
     *
     * @param x      x coordinate of point to be checked.
     * @param y      y coordinate of point to be checked.
     * @param left   left coordinate of rectangle.
     * @param top    top coordinate of rectangle.
     * @param right  right coordinate of rectangle.
     * @param bottom bottom coordinate of rectangle.
     * @return true if point is locus to bottom side of rectangle, false
     * otherwise.
     */
    public static boolean isLocusToBottomSide(
            final double x, final double y, final double left,
            final double top, final double right, final double bottom) {
        return isLocusToBottomSide(x, y, left, top, right, bottom,
                DEFAULT_THRESHOLD);
    }

    /**
     * Indicates whether provided point belongs to the locus of the bottom side
     * of the rectangle defined by provided top-left and bottom-right corners.
     *
     * @param point  point to be checked.
     * @param left   left coordinate of rectangle.
     * @param top    top coordinate of rectangle.
     * @param right  right coordinate of rectangle.
     * @param bottom bottom coordinate of rectangle.
     * @return true if point is locus to bottom side of rectangle, false
     * otherwise.
     */
    public static boolean isLocusToBottomSide(
            final Point2D point, final double left,
            final double top, final double right, final double bottom) {
        return isLocusToBottomSide(point, left, top, right, bottom,
                DEFAULT_THRESHOLD);
    }

    /**
     * Indicates whether provided point coordinates belong to the locus of the
     * bottom side of the rectangle defined by provided top-left and
     * bottom-right corners up to a certain threshold.
     *
     * @param x           x coordinate of point to be checked.
     * @param y           y coordinate of point to be checked.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @return true if point is locus to bottom side of rectangle, false
     * otherwise.
     */
    public static boolean isLocusToBottomSide(
            final double x, final double y,
            final Point2D topLeft, final Point2D bottomRight) {
        return isLocusToBottomSide(x, y, topLeft, bottomRight,
                DEFAULT_THRESHOLD);
    }

    /**
     * Indicates whether provided point belongs to the locus of the bottom side
     * of the rectangle defined by provided top-left and bottom-right corners.
     *
     * @param point       point to be checked.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @return true if point is locus to bottom side of rectangle, false
     * otherwise.
     */
    public static boolean isLocusToBottomSide(
            final Point2D point, final Point2D topLeft,
            final Point2D bottomRight) {
        return isLocusToBottomSide(point, topLeft, bottomRight,
                DEFAULT_THRESHOLD);
    }

    /**
     * Indicates whether provided point coordinates belong to the locus of the
     * bottom side of the rectangle defined by provided rectangle center and
     * size.
     *
     * @param x      x coordinate of point to be checked.
     * @param y      y coordinate of point to be checked.
     * @param center center of rectangle.
     * @param width  width of rectangle.
     * @param height height of rectangle.
     * @return true if point is locus to bottom side of rectangle, false
     * otherwise.
     */
    public static boolean isLocusToBottomSide(
            final double x, final double y,
            final Point2D center, final double width, final double height) {
        return isLocusToBottomSide(x, y, center, width, height,
                DEFAULT_THRESHOLD);
    }

    /**
     * Indicates whether provided point belongs to the locus of the bottom side
     * of the rectangle defined by provided rectangle center and size.
     *
     * @param point  point to be checked.
     * @param center center of rectangle.
     * @param width  width of rectangle.
     * @param height height of rectangle.
     * @return true if point is locus to bottom side of rectangle, false
     * otherwise.
     */
    public static boolean isLocusToBottomSide(
            final Point2D point, final Point2D center,
            final double width, final double height) {
        return isLocusToBottomSide(point, center, width, height,
                DEFAULT_THRESHOLD);
    }

    /**
     * Indicates whether provided point coordinates belong to the locus of the
     * bottom side of this rectangle.
     *
     * @param x x coordinate of point to be checked.
     * @param y y coordinate of point to be checked.
     * @return true if point is locus to bottom side of rectangle, false
     * otherwise.
     */
    public boolean isLocusToBottomSide(final double x, final double y) {
        return isLocusToBottomSide(x, y, DEFAULT_THRESHOLD);
    }

    /**
     * Indicates whether provided point belongs to the locus of the bottom side
     * of this rectangle.
     *
     * @param point point to be checked.
     * @return true if point is locus to bottom side of rectangle, false
     * otherwise.
     */
    public boolean isLocusToBottomSide(final Point2D point) {
        return isLocusToBottomSide(point, DEFAULT_THRESHOLD);
    }

    /**
     * Indicates whether provided point coordinates belong to the locus of the
     * rectangle defined by provided top-left and bottom-right corners up to a
     * certain threshold.
     *
     * @param x         x coordinate of point to be checked.
     * @param y         y coordinate of point to be checked.
     * @param left      left coordinate of rectangle.
     * @param top       top coordinate of rectangle.
     * @param right     right coordinate of rectangle.
     * @param bottom    bottom coordinate of rectangle.
     * @param threshold threshold to use as a margin to determine whether
     *                  point is locus of rectangle.
     * @return true if point is locus of rectangle, false otherwise.
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public static boolean isLocus(
            final double x, final double y, final double left, final double top,
            final double right, final double bottom, final double threshold) {
        if (threshold < 0.0) {
            throw new IllegalArgumentException("threshold must be positive");
        }

        return isLocusToLeftSide(x, y, left, top, right, bottom, threshold) ||
                isLocusToTopSide(x, y, left, top, right, bottom, threshold) ||
                isLocusToRightSide(x, y, left, top, right, bottom, threshold) ||
                isLocusToBottomSide(x, y, left, top, right, bottom, threshold);
    }

    /**
     * Indicates whether provided point belongs to the locus of the rectangle
     * defined by provided top-left and bottom-right corners up to a certain
     * threshold.
     *
     * @param point     point to be checked.
     * @param left      left coordinate of rectangle.
     * @param top       top coordinate of rectangle.
     * @param right     right coordinate of rectangle.
     * @param bottom    bottom coordinate of rectangle.
     * @param threshold threshold to use as a margin to determine whether
     *                  point is locus of rectangle.
     * @return true if point is locus of rectangle, false otherwise.
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public static boolean isLocus(
            final Point2D point, final double left, final double top,
            final double right, final double bottom, final double threshold) {
        return isLocusToLeftSide(point, left, top, right, bottom, threshold) ||
                isLocusToTopSide(point, left, top, right, bottom, threshold) ||
                isLocusToRightSide(point, left, top, right, bottom, threshold) ||
                isLocusToBottomSide(point, left, top, right, bottom, threshold);
    }

    /**
     * Indicates whether provided point coordinates belong to the locus of the
     * rectangle defined by provided top-left and bottom-right corners up to a
     * certain threshold.
     *
     * @param x           x coordinate of point to be checked.
     * @param y           y coordinate of point to be checked.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @param threshold   threshold to use as a margin to determine whether
     *                    point is locus of rectangle.
     * @return true if point is locus of rectangle, false otherwise.
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public static boolean isLocus(
            final double x, final double y, final Point2D topLeft,
            final Point2D bottomRight, final double threshold) {
        return isLocusToLeftSide(x, y, topLeft, bottomRight, threshold) ||
                isLocusToTopSide(x, y, topLeft, bottomRight, threshold) ||
                isLocusToRightSide(x, y, topLeft, bottomRight, threshold) ||
                isLocusToBottomSide(x, y, topLeft, bottomRight, threshold);
    }

    /**
     * Indicates whether provided point belongs to the locus of the rectangle
     * defined by provided top-left and bottom-right corners up to a certain
     * threshold.
     *
     * @param point       point to be checked.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @param threshold   threshold to use as a margin to determine whether
     *                    point is locus of rectangle.
     * @return true if point is locus of rectangle, false otherwise.
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public static boolean isLocus(
            final Point2D point, final Point2D topLeft,
            final Point2D bottomRight, final double threshold) {
        return isLocusToLeftSide(point, topLeft, bottomRight, threshold) ||
                isLocusToTopSide(point, topLeft, bottomRight, threshold) ||
                isLocusToRightSide(point, topLeft, bottomRight, threshold) ||
                isLocusToBottomSide(point, topLeft, bottomRight, threshold);
    }

    /**
     * Indicates whether provided point coordinates belong to the locus of the
     * rectangle defined by provided center and size up to a certain threshold.
     *
     * @param x         x coordinate of point to be checked.
     * @param y         y coordinate of point to be checked.
     * @param center    center of rectangle.
     * @param width     width of rectangle.
     * @param height    height of rectangle.
     * @param threshold threshold to use as a margin to determine whether
     *                  point is locus of rectangle.
     * @return true if point is locus of rectangle, false otherwise.
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public static boolean isLocus(
            final double x, final double y, final Point2D center,
            final double width, final double height, final double threshold) {
        return isLocusToLeftSide(x, y, center, width, height, threshold) ||
                isLocusToTopSide(x, y, center, width, height, threshold) ||
                isLocusToRightSide(x, y, center, width, height, threshold) ||
                isLocusToBottomSide(x, y, center, width, height, threshold);
    }

    /**
     * Indicates whether provided point belongs to the locus of the rectangle
     * defined by provided center and size up to a certain threshold.
     *
     * @param point     point to be checked.
     * @param center    center of rectangle.
     * @param width     width of rectangle.
     * @param height    height of rectangle.
     * @param threshold threshold to use as a margin to determine whether point
     *                  is locus of rectangle.
     * @return true if point is locus of rectangle, false otherwise.
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public static boolean isLocus(
            final Point2D point, final Point2D center, final double width,
            final double height, final double threshold) {
        return isLocusToLeftSide(point, center, width, height, threshold) ||
                isLocusToTopSide(point, center, width, height, threshold) ||
                isLocusToRightSide(point, center, width, height, threshold) ||
                isLocusToBottomSide(point, center, width, height, threshold);
    }

    /**
     * Indicates whether provided point coordinates belong to the locus of this
     * rectangle up to a certain threshold.
     *
     * @param x         x coordinate of point to be checked.
     * @param y         y coordinate of point to be checked.
     * @param threshold threshold to use as a margin to determine whether
     *                  point is locus of rectangle.
     * @return true if point is locus of rectangle, false otherwise.
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public boolean isLocus(
            final double x, final double y, final double threshold) {
        return isLocus(x, y, mTopLeft, mBottomRight, threshold);
    }

    /**
     * Indicates whether provided point coordinates belong to the locus of this
     * rectangle up to a certain threshold.
     *
     * @param point     point to be checked.
     * @param threshold threshold to use as a margin to determine whether
     *                  point is locus of rectangle.
     * @return true if point is locus of rectangle, false otherwise.
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public boolean isLocus(
            final Point2D point, final double threshold) {
        return isLocus(point, mTopLeft, mBottomRight, threshold);
    }

    /**
     * Indicates whether provided point coordinates belong to the locus of the
     * rectangle defined by provided top-left and bottom-right corners up to a
     * certain threshold.
     *
     * @param x      x coordinate of point to be checked.
     * @param y      y coordinate of point to be checked.
     * @param left   left coordinate of rectangle.
     * @param top    top coordinate of rectangle.
     * @param right  right coordinate of rectangle.
     * @param bottom bottom coordinate of rectangle.
     * @return true if point is locus of rectangle, false otherwise.
     */
    public static boolean isLocus(
            final double x, final double y, final double left, final double top,
            final double right, final double bottom) {
        return isLocus(x, y, left, top, right, bottom, DEFAULT_THRESHOLD);
    }

    /**
     * Indicates whether provided point belongs to the locus of the rectangle
     * defined by provided top-left and bottom-right corners.
     *
     * @param point  point to be checked.
     * @param left   left coordinate of rectangle.
     * @param top    top coordinate of rectangle.
     * @param right  right coordinate of rectangle.
     * @param bottom bottom coordinate of rectangle.
     * @return true if point is locus of rectangle, false otherwise.
     */
    public static boolean isLocus(
            final Point2D point, final double left, final double top,
            final double right, final double bottom) {
        return isLocus(point, left, top, right, bottom, DEFAULT_THRESHOLD);
    }

    /**
     * Indicates whether provided point coordinates belong to the locus of the
     * rectangle defined by provided top-left and bottom-right corners.
     *
     * @param x           x coordinate of point to be checked.
     * @param y           y coordinate of point to be checked.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @return true if point is locus of rectangle, false otherwise.
     */
    public static boolean isLocus(
            final double x, final double y, final Point2D topLeft,
            final Point2D bottomRight) {
        return isLocus(x, y, topLeft, bottomRight, DEFAULT_THRESHOLD);
    }

    /**
     * Indicates whether provided point belongs to the locus of the rectangle
     * defined by provided top-left and bottom-right corners.
     *
     * @param point       point to be checked.
     * @param topLeft     top-left corner of rectangle.
     * @param bottomRight bottom-right corner of rectangle.
     * @return true if point is locus of rectangle, false otherwise.
     */
    public static boolean isLocus(
            final Point2D point, final Point2D topLeft,
            final Point2D bottomRight) {
        return isLocus(point, topLeft, bottomRight, DEFAULT_THRESHOLD);
    }

    /**
     * Indicates whether provided point coordinates belong to the locus of the
     * rectangle defined by provided center and size.
     *
     * @param x      x coordinate of point to be checked.
     * @param y      y coordinate of point to be checked.
     * @param center center of rectangle.
     * @param width  width of rectangle.
     * @param height height of rectangle.
     * @return true if point is locus of rectangle, false otherwise.
     */
    public static boolean isLocus(
            final double x, final double y, final Point2D center,
            final double width, final double height) {
        return isLocus(x, y, center, width, height, DEFAULT_THRESHOLD);
    }

    /**
     * Indicates whether provided point belongs to the locus of the rectangle
     * defined by provided center and size.
     *
     * @param point  point to be checked.
     * @param center center of rectangle.
     * @param width  width of rectangle.
     * @param height height of rectangle.
     * @return true if point is locus of rectangle, false otherwise.
     */
    public static boolean isLocus(
            final Point2D point, final Point2D center, final double width,
            final double height) {
        return isLocus(point, center, width, height, DEFAULT_THRESHOLD);
    }

    /**
     * Indicates whether provided point coordinates belong to the locus of this
     * rectangle up to a certain threshold.
     *
     * @param x x coordinate of point to be checked.
     * @param y y coordinate of point to be checked.
     * @return true if point is locus of rectangle, false otherwise.
     */
    public boolean isLocus(final double x, final double y) {
        return isLocus(x, y, DEFAULT_THRESHOLD);
    }

    /**
     * Indicates whether provided point coordinates belong to the locus of this
     * rectangle.
     *
     * @param point point to be checked.
     * @return true if point is locus of rectangle, false otherwise.
     */
    public boolean isLocus(final Point2D point) {
        return isLocus(point, DEFAULT_THRESHOLD);
    }
}
