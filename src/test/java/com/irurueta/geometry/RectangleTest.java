/*
 * Copyright (C) 2016 Alberto Irurueta Carro (alberto@irurueta.com)
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

import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

class RectangleTest {

    private static final double ABSOLUTE_ERROR = 1e-9;
    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    @Test
    void testConstants() {
        assertEquals(1e-9, Rectangle.DEFAULT_THRESHOLD, 0.0);
    }

    @Test
    void testConstructor() {
        // test empty constructor
        var r = new Rectangle();

        // check default values
        assertEquals(new InhomogeneousPoint2D(-0.5, -0.5), r.getTopLeft());
        assertEquals(new InhomogeneousPoint2D(0.5, 0.5), r.getBottomRight());
        assertEquals(new InhomogeneousPoint2D(0, 0), r.getCenter());
        assertEquals(1.0, r.getSignedWidth(), ABSOLUTE_ERROR);
        assertEquals(1.0, r.getWidth(), ABSOLUTE_ERROR);
        assertEquals(1.0, r.getSignedHeight(), ABSOLUTE_ERROR);
        assertEquals(1.0, r.getHeight(), ABSOLUTE_ERROR);
        assertEquals(1.0, r.getArea(), ABSOLUTE_ERROR);
        assertEquals(4.0, r.getPerimeter(), ABSOLUTE_ERROR);

        // test constructor with top-left and bottom-right corners
        final var randomizer = new UniformRandomizer();
        final var left = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var top = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var right = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var bottom = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var topLeft = new InhomogeneousPoint2D(left, top);
        final var bottomRight = new InhomogeneousPoint2D(right, bottom);
        final var center = new InhomogeneousPoint2D((left + right) / 2.0, (top + bottom) / 2.0);
        final var signedWidth = right - left;
        final var signedHeight = bottom - top;
        final var width = Math.abs(signedWidth);
        final var height = Math.abs(signedHeight);
        final var area = width * height;
        final var perimeter = 2 * (width + height);

        r = new Rectangle(topLeft, bottomRight);

        // check correctness
        assertEquals(r.getTopLeft(), topLeft);
        assertEquals(r.getBottomRight(), bottomRight);
        assertEquals(r.getCenter(), center);
        assertEquals(r.getSignedWidth(), signedWidth, ABSOLUTE_ERROR);
        assertEquals(r.getSignedHeight(), signedHeight, ABSOLUTE_ERROR);
        assertEquals(r.getWidth(), width, ABSOLUTE_ERROR);
        assertEquals(r.getHeight(), height, ABSOLUTE_ERROR);
        assertEquals(r.getArea(), area, ABSOLUTE_ERROR);
        assertEquals(r.getPerimeter(), perimeter, ABSOLUTE_ERROR);

        // test constructor with corner coordinates
        r = new Rectangle(left, top, right, bottom);

        // check correctness
        assertEquals(topLeft, r.getTopLeft());
        assertEquals(bottomRight, r.getBottomRight());
        assertEquals(center, r.getCenter());
        assertEquals(signedWidth, r.getSignedWidth(), ABSOLUTE_ERROR);
        assertEquals(signedHeight, r.getSignedHeight(), ABSOLUTE_ERROR);
        assertEquals(width, r.getWidth(), ABSOLUTE_ERROR);
        assertEquals(height, r.getHeight(), ABSOLUTE_ERROR);
        assertEquals(area, r.getArea(), ABSOLUTE_ERROR);
        assertEquals(perimeter, r.getPerimeter(), ABSOLUTE_ERROR);

        // test constructor with box
        final var lo = new InhomogeneousPoint2D(left, bottom);
        final var hi = new InhomogeneousPoint2D(right, top);
        final var box = new Box2D(lo, hi);
        r = new Rectangle(box);

        // check correctness
        assertEquals(topLeft, r.getTopLeft());
        assertEquals(bottomRight, r.getBottomRight());
        assertEquals(center, r.getCenter());
        assertEquals(signedWidth, r.getSignedWidth(), ABSOLUTE_ERROR);
        assertEquals(signedHeight, r.getSignedHeight(), ABSOLUTE_ERROR);
        assertEquals(width, r.getWidth(), ABSOLUTE_ERROR);
        assertEquals(height, r.getHeight(), ABSOLUTE_ERROR);
        assertEquals(area, r.getArea(), ABSOLUTE_ERROR);
        assertEquals(perimeter, r.getPerimeter(), ABSOLUTE_ERROR);
    }

    @Test
    void testGetSetTopLeft() {
        final var r = new Rectangle();

        // check default value
        assertEquals(new InhomogeneousPoint2D(-0.5, -0.5), r.getTopLeft());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var topLeft = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        r.setTopLeft(topLeft);

        // check correctness
        assertEquals(topLeft, r.getTopLeft());
    }

    @Test
    void testGetSetBottomRight() {
        final var r = new Rectangle();

        // check default value
        assertEquals(new InhomogeneousPoint2D(0.5, 0.5), r.getBottomRight());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var bottomRight = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        r.setBottomRight(bottomRight);

        // check correctness
        assertEquals(bottomRight, r.getBottomRight());
    }

    @Test
    void testSetBounds() {
        var r = new Rectangle();

        // initial values
        assertEquals(new InhomogeneousPoint2D(-0.5, -0.5), r.getTopLeft());
        assertEquals(new InhomogeneousPoint2D(0.5, 0.5), r.getBottomRight());

        // set new values with top-left and bottom-right corners
        final var randomizer = new UniformRandomizer();
        final var left = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var top = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var right = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var bottom = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var topLeft = new InhomogeneousPoint2D(left, top);
        final var bottomRight = new InhomogeneousPoint2D(right, bottom);

        r.setBounds(topLeft, bottomRight);

        // check correctness
        assertEquals(topLeft, r.getTopLeft());
        assertEquals(bottomRight, r.getBottomRight());

        // set new values with top, left, bottom, right values
        r = new Rectangle();

        // initial values
        assertEquals(new InhomogeneousPoint2D(-0.5, -0.5), r.getTopLeft());
        assertEquals(new InhomogeneousPoint2D(0.5, 0.5), r.getBottomRight());

        r.setBounds(left, top, right, bottom);

        // check correctness
        assertEquals(topLeft, r.getTopLeft());
        assertEquals(bottomRight, r.getBottomRight());
    }

    @Test
    void testGetSetCenter() {
        final var randomizer = new UniformRandomizer();
        final var left = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var top = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var right = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var bottom = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var topLeft = new InhomogeneousPoint2D(left, top);
        final var bottomRight = new InhomogeneousPoint2D(right, bottom);

        final var center = new InhomogeneousPoint2D((left + right) / 2.0, (top + bottom) / 2.0);

        // test static method with corners
        var result = Point2D.create();
        Rectangle.getCenter(topLeft, bottomRight, result);

        // check correctness
        assertEquals(result, center);

        // test static method with coordinates
        result = Point2D.create();
        Rectangle.getCenter(left, top, right, bottom, result);

        // check correctness
        assertEquals(result, center);

        // test static method with new instance and corners
        result = Rectangle.getCenter(topLeft, bottomRight);

        // check correctness
        assertEquals(result, center);

        // test static method with new instance and coordinates
        result = Rectangle.getCenter(left, top, right, bottom);

        // check correctness
        assertEquals(result, center);

        // test non-static methods
        var r = new Rectangle();

        // check initial value
        assertEquals(new InhomogeneousPoint2D(0.0, 0.0), r.getCenter());

        // set center coordinates
        r.setCenter(center.getInhomX(), center.getInhomY());

        // check correctness
        assertEquals(center, r.getCenter());
        final var center2 = Point2D.create();
        r.getCenter(center2);
        assertEquals(center, center2);

        // set center
        r = new Rectangle();
        r.setCenter(center);

        // check correctness
        assertEquals(center, r.getCenter());
    }

    @Test
    void testFromToBox() {
        final var randomizer = new UniformRandomizer();
        final var left = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var top = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var right = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var bottom = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var topLeft = new InhomogeneousPoint2D(left, top);
        final var bottomRight = new InhomogeneousPoint2D(right, bottom);
        final var rectangle = new Rectangle(topLeft, bottomRight);

        final var lo = new InhomogeneousPoint2D(left, bottom);
        final var hi = new InhomogeneousPoint2D(right, top);
        final var box = new Box2D(lo, hi);

        // from box
        final var rectangle2 = new Rectangle();
        rectangle2.fromBox(box);

        // check correctness
        assertEquals(rectangle.getTopLeft(), rectangle2.getTopLeft());
        assertEquals(rectangle.getBottomRight(), rectangle2.getBottomRight());

        // to box
        final var box2 = rectangle.toBox();

        final var box3 = new Box2D();
        rectangle.toBox(box3);

        // check correctness
        assertEquals(box.getLo(), box2.getLo());
        assertEquals(box.getHi(), box2.getHi());
        assertEquals(box.getLo(), box3.getLo());
        assertEquals(box.getHi(), box3.getHi());
    }

    @Test
    void testGetSignedWidth() {
        final var randomizer = new UniformRandomizer();
        final var left = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var top = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var right = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var bottom = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var signedWidth = right - left;

        // test static method
        final var topLeft = new InhomogeneousPoint2D(left, top);
        final var bottomRight = new InhomogeneousPoint2D(right, bottom);

        assertEquals(Rectangle.getSignedWidth(topLeft, bottomRight), signedWidth, ABSOLUTE_ERROR);

        // test non-static method
        final var r = new Rectangle(left, top, right, bottom);

        assertEquals(r.getSignedWidth(), signedWidth, ABSOLUTE_ERROR);
    }

    @Test
    void testGetWidth() {
        final var randomizer = new UniformRandomizer();
        final var left = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var top = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var right = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var bottom = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var width = Math.abs(right - left);

        // test static method
        final var topLeft = new InhomogeneousPoint2D(left, top);
        final var bottomRight = new InhomogeneousPoint2D(right, bottom);

        assertEquals(Rectangle.getWidth(topLeft, bottomRight), width, ABSOLUTE_ERROR);

        // test non-static method
        final var r = new Rectangle(left, top, right, bottom);

        assertEquals(r.getWidth(), width, ABSOLUTE_ERROR);
    }

    @Test
    void testGetSignedHeight() {
        final var randomizer = new UniformRandomizer();
        final var left = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var top = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var right = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var bottom = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var signedHeight = bottom - top;

        // test static method
        final var topLeft = new InhomogeneousPoint2D(left, top);
        final var bottomRight = new InhomogeneousPoint2D(right, bottom);

        assertEquals(Rectangle.getSignedHeight(topLeft, bottomRight), signedHeight, ABSOLUTE_ERROR);

        // test non-static method
        final var r = new Rectangle(left, top, right, bottom);

        assertEquals(r.getSignedHeight(), signedHeight, ABSOLUTE_ERROR);
    }

    @Test
    void testGetHeight() {
        final var randomizer = new UniformRandomizer();
        final var left = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var top = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var right = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var bottom = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var height = Math.abs(bottom - top);

        // test static method
        final var topLeft = new InhomogeneousPoint2D(left, top);
        final var bottomRight = new InhomogeneousPoint2D(right, bottom);

        assertEquals(Rectangle.getHeight(topLeft, bottomRight), height, ABSOLUTE_ERROR);

        // test non-static method
        final var r = new Rectangle(left, top, right, bottom);

        assertEquals(r.getHeight(), height, ABSOLUTE_ERROR);
    }

    @Test
    void testGetArea() {
        final var randomizer = new UniformRandomizer();
        final var left = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var top = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var right = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var bottom = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var width = Math.abs(right - left);
        final var height = Math.abs(bottom - top);

        final var area = width * height;

        // test static methods
        final var topLeft = new InhomogeneousPoint2D(left, top);
        final var bottomRight = new InhomogeneousPoint2D(right, bottom);

        assertEquals(area, Rectangle.getArea(topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(area, Rectangle.getArea(left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(area, Rectangle.getArea(width, height), ABSOLUTE_ERROR);

        // test non-static method
        final var r = new Rectangle(left, top, right, bottom);

        assertEquals(area, r.getArea(), ABSOLUTE_ERROR);
    }

    @Test
    void testSetCenterAndSize() {
        final var randomizer = new UniformRandomizer();
        final var left = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var top = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var right = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var bottom = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var width = Math.abs(right - left);
        final var height = Math.abs(bottom - top);

        final var centerX = 0.5 * (left + right);
        final var centerY = 0.5 * (bottom + top);

        final var center = new InhomogeneousPoint2D(centerX, centerY);

        final var topLeft = new InhomogeneousPoint2D(centerX - 0.5 * width, centerY - 0.5 * height);
        final var bottomRight = new InhomogeneousPoint2D(centerX + 0.5 * width, centerY + 0.5 * height);

        // test with center coordinates
        var r = new Rectangle();
        r.setCenterAndSize(centerX, centerY, width, height);

        // check correctness
        assertEquals(center, r.getCenter());
        assertEquals(width, r.getWidth(), ABSOLUTE_ERROR);
        assertEquals(height, r.getHeight(), ABSOLUTE_ERROR);
        assertEquals(topLeft, r.getTopLeft());
        assertEquals(bottomRight, r.getBottomRight());

        // test with center point
        r = new Rectangle();
        r.setCenterAndSize(center, width, height);

        // check correctness
        assertEquals(center, r.getCenter());
        assertEquals(width, r.getWidth(), ABSOLUTE_ERROR);
        assertEquals(height, r.getHeight(), ABSOLUTE_ERROR);
        assertEquals(topLeft, r.getTopLeft());
        assertEquals(bottomRight, r.getBottomRight());
    }

    @Test
    void testGetPerimeter() {
        final var randomizer = new UniformRandomizer();
        final var left = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var top = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var right = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var bottom = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var width = Math.abs(right - left);
        final var height = Math.abs(bottom - top);
        final var perimeter = 2.0 * (width + height);

        final var topLeft = new InhomogeneousPoint2D(left, top);
        final var bottomRight = new InhomogeneousPoint2D(right, bottom);

        // test static methods
        assertEquals(perimeter, Rectangle.getPerimeter(width, height), ABSOLUTE_ERROR);
        assertEquals(perimeter, Rectangle.getPerimeter(topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getPerimeter(left, top, right, bottom), perimeter, ABSOLUTE_ERROR);

        // test non-static method
        final var r = new Rectangle(left, top, right, bottom);
        assertEquals(perimeter, r.getPerimeter(), ABSOLUTE_ERROR);
    }

    @Test
    void testIsInside() {
        final var randomizer = new UniformRandomizer();
        final var centerX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var centerY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var width = randomizer.nextDouble(0, MAX_RANDOM_VALUE);
        final var height = randomizer.nextDouble(0, MAX_RANDOM_VALUE);

        final var left = centerX - 0.5 * width;
        final var right = centerX + 0.5 * width;
        final var top = centerY - 0.5 * height;
        final var bottom = centerY + 0.5 * height;

        final var insideX = centerX + width * randomizer.nextDouble(-0.5, 0.5);
        final var insideY = centerY + height * randomizer.nextDouble(-0.5, 0.5);

        final var signX = randomizer.nextBoolean() ? 1 : -1;
        final var signY = randomizer.nextBoolean() ? 1 : -1;
        final var outsideX = centerX + width * signX * randomizer.nextDouble(1.5, 2.5);
        final var outsideY = centerY + height * signY * randomizer.nextDouble(1.5, 2.5);

        final var insidePoint = new InhomogeneousPoint2D(insideX, insideY);
        final var outsidePoint = new InhomogeneousPoint2D(outsideX, outsideY);
        final var center = new InhomogeneousPoint2D(centerX, centerY);
        final var topLeft = new InhomogeneousPoint2D(left, top);
        final var bottomRight = new InhomogeneousPoint2D(right, bottom);

        // test static methods with point, corner coordinates and threshold
        assertTrue(Rectangle.isInside(insideX, insideY, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isInside(outsideX, outsideY, left, top, right, bottom, ABSOLUTE_ERROR));

        assertFalse(Rectangle.isInside(insideX, insideY, left, top, right, bottom, Math.max(width, height)));
        assertTrue(Rectangle.isInside(insideX, insideY, left, top, right, bottom, -Math.max(width, height)));

        assertFalse(Rectangle.isInside(outsideX, outsideY, left, top, right, bottom, Math.max(width, height)));
        assertTrue(Rectangle.isInside(outsideX, outsideY, left, top, right, bottom,
                -3.0 * Math.max(width, height)));

        // test static methods with point and corner coordinates
        assertTrue(Rectangle.isInside(insideX, insideY, left, top, right, bottom));
        assertFalse(Rectangle.isInside(outsideX, outsideY, left, top, right, bottom));

        // test static methods with point, corner coordinates and threshold
        assertTrue(Rectangle.isInside(insidePoint, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isInside(outsidePoint, left, top, right, bottom, ABSOLUTE_ERROR));

        assertFalse(Rectangle.isInside(insidePoint, left, top, right, bottom, Math.max(width, height)));
        assertTrue(Rectangle.isInside(insidePoint, left, top, right, bottom, -Math.max(width, height)));

        assertFalse(Rectangle.isInside(outsidePoint, left, top, right, bottom, Math.max(width, height)));
        assertTrue(Rectangle.isInside(outsidePoint, left, top, right, bottom,
                -3.0 * Math.max(width, height)));

        // test static methods with point and corner coordinates
        assertTrue(Rectangle.isInside(insidePoint, left, top, right, bottom));
        assertFalse(Rectangle.isInside(outsidePoint, left, top, right, bottom));

        // test static methods with point, center, size and threshold
        assertTrue(Rectangle.isInside(insidePoint, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isInside(outsidePoint, center, width, height, ABSOLUTE_ERROR));

        assertFalse(Rectangle.isInside(insidePoint, center, width, height, Math.max(width, height)));
        assertTrue(Rectangle.isInside(insidePoint, center, width, height, -Math.max(width, height)));

        assertFalse(Rectangle.isInside(outsidePoint, center, width, height, Math.max(width, height)));
        assertTrue(Rectangle.isInside(outsidePoint, center, width, height, -3.0 * Math.max(width, height)));

        // test static methods with point, center and size
        assertTrue(Rectangle.isInside(insidePoint, center, width, height));
        assertFalse(Rectangle.isInside(outsidePoint, center, width, height));

        // test static methods with point coordinates, corners and threshold
        assertTrue(Rectangle.isInside(insideX, insideY, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isInside(outsideX, outsideY, topLeft, bottomRight, ABSOLUTE_ERROR));

        assertFalse(Rectangle.isInside(insideX, insideY, topLeft, bottomRight, Math.max(width, height)));
        assertTrue(Rectangle.isInside(insideX, insideY, topLeft, bottomRight, -Math.max(width, height)));

        assertFalse(Rectangle.isInside(outsideX, outsideY, topLeft, bottomRight, Math.max(width, height)));
        assertTrue(Rectangle.isInside(outsideX, outsideY, topLeft, bottomRight,
                -3.0 * Math.max(width, height)));

        // test static methods with point coordinates and corners
        assertTrue(Rectangle.isInside(insideX, insideY, topLeft, bottomRight));
        assertFalse(Rectangle.isInside(outsideX, outsideY, topLeft, bottomRight));

        // test static methods with point, corners and threshold
        assertTrue(Rectangle.isInside(insidePoint, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isInside(outsidePoint, topLeft, bottomRight, ABSOLUTE_ERROR));

        assertFalse(Rectangle.isInside(insidePoint, topLeft, bottomRight, Math.max(width, height)));
        assertTrue(Rectangle.isInside(insidePoint, topLeft, bottomRight, -Math.max(width, height)));

        assertFalse(Rectangle.isInside(outsidePoint, topLeft, bottomRight, Math.max(width, height)));
        assertTrue(Rectangle.isInside(outsidePoint, topLeft, bottomRight, -3.0 * Math.max(width, height)));

        // test static methods with point and corners
        assertTrue(Rectangle.isInside(insidePoint, topLeft, bottomRight));
        assertFalse(Rectangle.isInside(outsidePoint, topLeft, bottomRight));

        // test non-static method with point coordinates and threshold
        final var r = new Rectangle(left, top, right, bottom);

        assertTrue(r.isInside(insideX, insideY, ABSOLUTE_ERROR));
        assertFalse(r.isInside(outsideX, outsideY, ABSOLUTE_ERROR));

        assertFalse(r.isInside(insideX, insideY, Math.max(width, height)));
        assertTrue(r.isInside(insideX, insideY, -Math.max(width, height)));

        assertFalse(r.isInside(outsideX, outsideY, Math.max(width, height)));
        assertTrue(r.isInside(outsideX, outsideY, -3.0 * Math.max(width, height)));

        // test non-static method with point coordinates
        assertTrue(r.isInside(insideX, insideY));
        assertFalse(r.isInside(outsideX, outsideY));

        // test non-static method with point and threshold
        assertTrue(r.isInside(insidePoint, ABSOLUTE_ERROR));
        assertFalse(r.isInside(outsidePoint, ABSOLUTE_ERROR));

        assertFalse(r.isInside(insidePoint, Math.max(width, height)));
        assertTrue(r.isInside(insidePoint, -Math.max(width, height)));

        assertFalse(r.isInside(outsidePoint, Math.max(width, height)));
        assertTrue(r.isInside(outsidePoint, -3.0 * Math.max(width, height)));

        // test non-static method with point
        assertTrue(r.isInside(insidePoint));
        assertFalse(r.isInside(outsidePoint));
    }

    @Test
    void testIsAtLeftSide() {
        final var randomizer = new UniformRandomizer();
        final var centerX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var centerY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var width = randomizer.nextDouble(0, MAX_RANDOM_VALUE);
        final var height = randomizer.nextDouble(0, MAX_RANDOM_VALUE);

        final var left = centerX - 0.5 * width;
        final var right = centerX + 0.5 * width;
        final var top = centerY + 0.5 * height;
        final var bottom = centerY - 0.5 * height;

        final var topLeft = new InhomogeneousPoint2D(left, top);
        final var bottomRight = new InhomogeneousPoint2D(right, bottom);
        final var center = new InhomogeneousPoint2D(centerX, centerY);

        final var pointAtLeftSide = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5), centerY);
        final var pointAtTopLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopSide = new InhomogeneousPoint2D(
                centerX,
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtRightSide = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY);
        final var pointAtBottomRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomSide = new InhomogeneousPoint2D(
                centerX,
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var insidePoint = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(-0.5, 0.5),
                centerY + height * randomizer.nextDouble(-0.5, 0.5));

        // test static method with point and corner coordinates and threshold
        assertTrue(Rectangle.isAtLeftSide(pointAtLeftSide.getInhomX(),
                pointAtLeftSide.getInhomY(), left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtLeftSide(pointAtTopLeftCorner.getInhomX(),
                pointAtTopLeftCorner.getInhomY(), left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtLeftSide(pointAtTopSide.getInhomX(),
                pointAtTopSide.getInhomY(), left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtLeftSide(pointAtTopRightCorner.getInhomX(),
                pointAtTopRightCorner.getInhomY(), left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtLeftSide(pointAtRightSide.getInhomX(),
                pointAtRightSide.getInhomY(), left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtLeftSide(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtLeftSide(pointAtBottomSide.getInhomX(),
                pointAtBottomSide.getInhomY(), left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtLeftSide(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtLeftSide(insidePoint.getInhomX(),
                insidePoint.getInhomY(), left, top, right, bottom, ABSOLUTE_ERROR));

        // test static method with point and corner coordinates
        assertTrue(Rectangle.isAtLeftSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(),
                left, top, right, bottom));
        assertFalse(Rectangle.isAtLeftSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                left, top, right, bottom));
        assertFalse(Rectangle.isAtLeftSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(),
                left, top, right, bottom));
        assertFalse(Rectangle.isAtLeftSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                left, top, right, bottom));
        assertFalse(Rectangle.isAtLeftSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(),
                left, top, right, bottom));
        assertFalse(Rectangle.isAtLeftSide(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY(),
                left, top, right, bottom));
        assertFalse(Rectangle.isAtLeftSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(),
                left, top, right, bottom));
        assertFalse(Rectangle.isAtLeftSide(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(),
                left, top, right, bottom));
        assertFalse(Rectangle.isAtLeftSide(insidePoint.getInhomX(), insidePoint.getInhomY(),
                left, top, right, bottom));

        // test static method with point, corner coordinate and threshold
        assertTrue(Rectangle.isAtLeftSide(pointAtLeftSide, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtLeftSide(pointAtTopLeftCorner, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtLeftSide(pointAtTopSide, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtLeftSide(pointAtTopRightCorner, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtLeftSide(pointAtRightSide, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtLeftSide(pointAtBottomRightCorner, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtLeftSide(pointAtBottomSide, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtLeftSide(pointAtBottomLeftCorner, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtLeftSide(insidePoint, left, top, right, bottom, ABSOLUTE_ERROR));

        // test static method with point, corner coordinate
        assertTrue(Rectangle.isAtLeftSide(pointAtLeftSide, left, top, right, bottom));
        assertFalse(Rectangle.isAtLeftSide(pointAtTopLeftCorner, left, top, right, bottom));
        assertFalse(Rectangle.isAtLeftSide(pointAtTopSide, left, top, right, bottom));
        assertFalse(Rectangle.isAtLeftSide(pointAtTopRightCorner, left, top, right, bottom));
        assertFalse(Rectangle.isAtLeftSide(pointAtRightSide, left, top, right, bottom));
        assertFalse(Rectangle.isAtLeftSide(pointAtBottomRightCorner, left, top, right, bottom));
        assertFalse(Rectangle.isAtLeftSide(pointAtBottomSide, left, top, right, bottom));
        assertFalse(Rectangle.isAtLeftSide(pointAtBottomLeftCorner, left, top, right, bottom));
        assertFalse(Rectangle.isAtLeftSide(insidePoint, left, top, right, bottom));

        // test static method with point coordinates, corners and threshold
        assertTrue(Rectangle.isAtLeftSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(),
                topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtLeftSide(pointAtTopLeftCorner.getInhomX(),
                pointAtTopLeftCorner.getInhomY(), topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtLeftSide(pointAtTopSide.getInhomX(),
                pointAtTopSide.getInhomY(), topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtLeftSide(pointAtTopRightCorner.getInhomX(),
                pointAtTopRightCorner.getInhomY(), topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtLeftSide(pointAtRightSide.getInhomX(),
                pointAtRightSide.getInhomY(), topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtLeftSide(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtLeftSide(pointAtBottomSide.getInhomX(),
                pointAtBottomSide.getInhomY(), topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtLeftSide(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtLeftSide(insidePoint.getInhomX(),
                insidePoint.getInhomY(), topLeft, bottomRight, ABSOLUTE_ERROR));

        // test static method with point coordinates and corners
        assertTrue(Rectangle.isAtLeftSide(pointAtLeftSide.getInhomX(),
                pointAtLeftSide.getInhomY(), topLeft, bottomRight));
        assertFalse(Rectangle.isAtLeftSide(pointAtTopLeftCorner.getInhomX(),
                pointAtTopLeftCorner.getInhomY(), topLeft, bottomRight));
        assertFalse(Rectangle.isAtLeftSide(pointAtTopSide.getInhomX(),
                pointAtTopSide.getInhomY(), topLeft, bottomRight));
        assertFalse(Rectangle.isAtLeftSide(pointAtTopRightCorner.getInhomX(),
                pointAtTopRightCorner.getInhomY(), topLeft, bottomRight));
        assertFalse(Rectangle.isAtLeftSide(pointAtRightSide.getInhomX(),
                pointAtRightSide.getInhomY(), topLeft, bottomRight));
        assertFalse(Rectangle.isAtLeftSide(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), topLeft, bottomRight));
        assertFalse(Rectangle.isAtLeftSide(pointAtBottomSide.getInhomX(),
                pointAtBottomSide.getInhomY(), topLeft, bottomRight));
        assertFalse(Rectangle.isAtLeftSide(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), topLeft, bottomRight));
        assertFalse(Rectangle.isAtLeftSide(insidePoint.getInhomX(),
                insidePoint.getInhomY(), topLeft, bottomRight));

        // test static method with point coordinates, corners and threshold
        assertTrue(Rectangle.isAtLeftSide(pointAtLeftSide, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtLeftSide(pointAtTopLeftCorner, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtLeftSide(pointAtTopSide, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtLeftSide(pointAtTopRightCorner, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtLeftSide(pointAtRightSide, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtLeftSide(pointAtBottomRightCorner, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtLeftSide(pointAtBottomSide, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtLeftSide(pointAtBottomLeftCorner, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtLeftSide(insidePoint, topLeft, bottomRight, ABSOLUTE_ERROR));

        // test static method with point coordinates and corners
        assertTrue(Rectangle.isAtLeftSide(pointAtLeftSide, topLeft, bottomRight));
        assertFalse(Rectangle.isAtLeftSide(pointAtTopLeftCorner, topLeft, bottomRight));
        assertFalse(Rectangle.isAtLeftSide(pointAtTopSide, topLeft, bottomRight));
        assertFalse(Rectangle.isAtLeftSide(pointAtTopRightCorner, topLeft, bottomRight));
        assertFalse(Rectangle.isAtLeftSide(pointAtRightSide, topLeft, bottomRight));
        assertFalse(Rectangle.isAtLeftSide(pointAtBottomRightCorner, topLeft, bottomRight));
        assertFalse(Rectangle.isAtLeftSide(pointAtBottomSide, topLeft, bottomRight));
        assertFalse(Rectangle.isAtLeftSide(pointAtBottomLeftCorner, topLeft, bottomRight));
        assertFalse(Rectangle.isAtLeftSide(insidePoint, topLeft, bottomRight));

        // test static method with point coordinates, center, sizes and threshold
        assertTrue(Rectangle.isAtLeftSide(pointAtLeftSide.getInhomX(),
                pointAtLeftSide.getInhomY(), center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtLeftSide(pointAtTopLeftCorner.getInhomX(),
                pointAtTopLeftCorner.getInhomY(), center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtLeftSide(pointAtTopSide.getInhomX(),
                pointAtTopSide.getInhomY(), center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtLeftSide(pointAtTopRightCorner.getInhomX(),
                pointAtTopRightCorner.getInhomY(), center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtLeftSide(pointAtRightSide.getInhomX(),
                pointAtRightSide.getInhomY(), center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtLeftSide(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtLeftSide(pointAtBottomSide.getInhomX(),
                pointAtBottomSide.getInhomY(), center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtLeftSide(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtLeftSide(insidePoint.getInhomX(),
                insidePoint.getInhomY(), center, width, height, ABSOLUTE_ERROR));

        // test static method with point coordinates, center and sizes
        assertTrue(Rectangle.isAtLeftSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(),
                center, width, height));
        assertFalse(Rectangle.isAtLeftSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                center, width, height));
        assertFalse(Rectangle.isAtLeftSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(),
                center, width, height));
        assertFalse(Rectangle.isAtLeftSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                center, width, height));
        assertFalse(Rectangle.isAtLeftSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(),
                center, width, height));
        assertFalse(Rectangle.isAtLeftSide(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY(),
                center, width, height));
        assertFalse(Rectangle.isAtLeftSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(),
                center, width, height));
        assertFalse(Rectangle.isAtLeftSide(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(),
                center, width, height));
        assertFalse(Rectangle.isAtLeftSide(insidePoint.getInhomX(), insidePoint.getInhomY(), center, width, height));

        // test static method with point, center, sizes and threshold
        assertTrue(Rectangle.isAtLeftSide(pointAtLeftSide, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtLeftSide(pointAtTopLeftCorner, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtLeftSide(pointAtTopSide, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtLeftSide(pointAtTopRightCorner, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtLeftSide(pointAtRightSide, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtLeftSide(pointAtBottomRightCorner, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtLeftSide(pointAtBottomSide, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtLeftSide(pointAtBottomLeftCorner, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtLeftSide(insidePoint, center, width, height, ABSOLUTE_ERROR));

        // test static method with point, center and sizes
        assertTrue(Rectangle.isAtLeftSide(pointAtLeftSide, center, width, height));
        assertFalse(Rectangle.isAtLeftSide(pointAtTopLeftCorner, center, width, height));
        assertFalse(Rectangle.isAtLeftSide(pointAtTopSide, center, width, height));
        assertFalse(Rectangle.isAtLeftSide(pointAtTopRightCorner, center, width, height));
        assertFalse(Rectangle.isAtLeftSide(pointAtRightSide, center, width, height));
        assertFalse(Rectangle.isAtLeftSide(pointAtBottomRightCorner, center, width, height));
        assertFalse(Rectangle.isAtLeftSide(pointAtBottomSide, center, width, height));
        assertFalse(Rectangle.isAtLeftSide(pointAtBottomLeftCorner, center, width, height));
        assertFalse(Rectangle.isAtLeftSide(insidePoint, center, width, height));

        // test non-static method with point coordinates and threshold
        final var r = new Rectangle(left, top, right, bottom);

        assertTrue(r.isAtLeftSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isAtLeftSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isAtLeftSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isAtLeftSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                ABSOLUTE_ERROR));
        assertFalse(r.isAtLeftSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isAtLeftSide(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY(),
                ABSOLUTE_ERROR));
        assertFalse(r.isAtLeftSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isAtLeftSide(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(),
                ABSOLUTE_ERROR));
        assertFalse(r.isAtLeftSide(insidePoint.getInhomX(), insidePoint.getInhomY(), ABSOLUTE_ERROR));

        // test non-static method with point coordinates
        assertTrue(r.isAtLeftSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY()));
        assertFalse(r.isAtLeftSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY()));
        assertFalse(r.isAtLeftSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY()));
        assertFalse(r.isAtLeftSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY()));
        assertFalse(r.isAtLeftSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY()));
        assertFalse(r.isAtLeftSide(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY()));
        assertFalse(r.isAtLeftSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY()));
        assertFalse(r.isAtLeftSide(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY()));
        assertFalse(r.isAtLeftSide(insidePoint.getInhomX(), insidePoint.getInhomY()));

        // test non-static method with point and threshold
        assertTrue(r.isAtLeftSide(pointAtLeftSide, ABSOLUTE_ERROR));
        assertFalse(r.isAtLeftSide(pointAtTopLeftCorner, ABSOLUTE_ERROR));
        assertFalse(r.isAtLeftSide(pointAtTopSide, ABSOLUTE_ERROR));
        assertFalse(r.isAtLeftSide(pointAtTopRightCorner, ABSOLUTE_ERROR));
        assertFalse(r.isAtLeftSide(pointAtRightSide, ABSOLUTE_ERROR));
        assertFalse(r.isAtLeftSide(pointAtBottomRightCorner, ABSOLUTE_ERROR));
        assertFalse(r.isAtLeftSide(pointAtBottomSide, ABSOLUTE_ERROR));
        assertFalse(r.isAtLeftSide(pointAtBottomLeftCorner, ABSOLUTE_ERROR));
        assertFalse(r.isAtLeftSide(insidePoint, ABSOLUTE_ERROR));

        // test non-static method with point
        assertTrue(r.isAtLeftSide(pointAtLeftSide));
        assertFalse(r.isAtLeftSide(pointAtTopLeftCorner));
        assertFalse(r.isAtLeftSide(pointAtTopSide));
        assertFalse(r.isAtLeftSide(pointAtTopRightCorner));
        assertFalse(r.isAtLeftSide(pointAtRightSide));
        assertFalse(r.isAtLeftSide(pointAtBottomRightCorner));
        assertFalse(r.isAtLeftSide(pointAtBottomSide));
        assertFalse(r.isAtLeftSide(pointAtBottomLeftCorner));
        assertFalse(r.isAtLeftSide(insidePoint));
    }

    @Test
    void testIsAtTopLeftCorner() {
        final var randomizer = new UniformRandomizer();
        final var centerX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var centerY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var width = randomizer.nextDouble(0, MAX_RANDOM_VALUE);
        final var height = randomizer.nextDouble(0, MAX_RANDOM_VALUE);

        final var left = centerX - 0.5 * width;
        final var right = centerX + 0.5 * width;
        final var top = centerY + 0.5 * height;
        final var bottom = centerY - 0.5 * height;

        final var topLeft = new InhomogeneousPoint2D(left, top);
        final var bottomRight = new InhomogeneousPoint2D(right, bottom);
        final var center = new InhomogeneousPoint2D(centerX, centerY);

        final var pointAtLeftSide = new InhomogeneousPoint2D(centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY);
        final var pointAtTopLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopSide = new InhomogeneousPoint2D(
                centerX,
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtRightSide = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY);
        final var pointAtBottomRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomSide = new InhomogeneousPoint2D(
                centerX,
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var insidePoint = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(-0.5, 0.5),
                centerY + height * randomizer.nextDouble(-0.5, 0.5));

        // test static method with point and corner coordinates and threshold
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(),
                left, top, right, bottom, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isAtTopLeftCorner(pointAtTopLeftCorner.getInhomX(),
                pointAtTopLeftCorner.getInhomY(), left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(),
                left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(),
                left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(),
                left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopLeftCorner(insidePoint.getInhomX(),
                insidePoint.getInhomY(), left, top, right, bottom, ABSOLUTE_ERROR));

        // test static method with point and corner coordinates
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(),
                left, top, right, bottom));
        assertTrue(Rectangle.isAtTopLeftCorner(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                left, top, right, bottom));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(),
                left, top, right, bottom));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                left, top, right, bottom));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(),
                left, top, right, bottom));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), left, top, right, bottom));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(),
                left, top, right, bottom));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), left, top, right, bottom));
        assertFalse(Rectangle.isAtTopLeftCorner(insidePoint.getInhomX(), insidePoint.getInhomY(),
                left, top, right, bottom));

        // test static method with point, corner coordinate and threshold
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtLeftSide, left, top, right, bottom, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isAtTopLeftCorner(pointAtTopLeftCorner, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtTopSide, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtTopRightCorner, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtRightSide, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtBottomRightCorner, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtBottomSide, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtBottomLeftCorner, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopLeftCorner(insidePoint, left, top, right, bottom, ABSOLUTE_ERROR));

        // test static method with point, corner coordinate
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtLeftSide, left, top, right, bottom));
        assertTrue(Rectangle.isAtTopLeftCorner(pointAtTopLeftCorner, left, top, right, bottom));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtTopSide, left, top, right, bottom));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtTopRightCorner, left, top, right, bottom));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtRightSide, left, top, right, bottom));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtBottomRightCorner, left, top, right, bottom));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtBottomSide, left, top, right, bottom));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtBottomLeftCorner, left, top, right, bottom));
        assertFalse(Rectangle.isAtTopLeftCorner(insidePoint, left, top, right, bottom));

        // test static method with point coordinates, corners and threshold
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), topLeft,
                bottomRight, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isAtTopLeftCorner(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), topLeft,
                bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), topLeft,
                bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), topLeft,
                bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopLeftCorner(insidePoint.getInhomX(), insidePoint.getInhomY(), topLeft, bottomRight,
                ABSOLUTE_ERROR));

        // test static method with point coordinates and corners
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), topLeft,
                bottomRight));
        assertTrue(Rectangle.isAtTopLeftCorner(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                topLeft, bottomRight));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), topLeft,
                bottomRight));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                topLeft, bottomRight));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), topLeft,
                bottomRight));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), topLeft, bottomRight));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), topLeft,
                bottomRight));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), topLeft, bottomRight));
        assertFalse(Rectangle.isAtTopLeftCorner(insidePoint.getInhomX(),
                insidePoint.getInhomY(), topLeft, bottomRight));

        // test static method with point coordinates, corners and threshold
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtLeftSide, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isAtTopLeftCorner(pointAtTopLeftCorner, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtTopSide, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtTopRightCorner, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtRightSide, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtBottomRightCorner, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtBottomSide, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtBottomLeftCorner, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopLeftCorner(insidePoint, topLeft, bottomRight, ABSOLUTE_ERROR));

        // test static method with point coordinates and corners
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtLeftSide, topLeft, bottomRight));
        assertTrue(Rectangle.isAtTopLeftCorner(pointAtTopLeftCorner, topLeft, bottomRight));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtTopSide, topLeft, bottomRight));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtTopRightCorner, topLeft, bottomRight));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtRightSide, topLeft, bottomRight));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtBottomRightCorner, topLeft, bottomRight));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtBottomSide, topLeft, bottomRight));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtBottomLeftCorner, topLeft, bottomRight));
        assertFalse(Rectangle.isAtTopLeftCorner(insidePoint, topLeft, bottomRight));

        // test static method with point coordinates, center, sizes and threshold
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), center, width,
                height, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isAtTopLeftCorner(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), center, width,
                height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), center,
                width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), center,
                width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopLeftCorner(insidePoint.getInhomX(), insidePoint.getInhomY(), center, width, height,
                ABSOLUTE_ERROR));

        // test static method with point coordinates, center and sizes
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), center, width,
                height));
        assertTrue(Rectangle.isAtTopLeftCorner(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                center, width, height));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), center, width,
                height));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                center, width, height));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), center,
                width, height));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), center, width, height));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), center,
                width, height));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), center, width, height));
        assertFalse(Rectangle.isAtTopLeftCorner(insidePoint.getInhomX(), insidePoint.getInhomY(), center, width,
                height));

        // test static method with point, center, sizes and threshold
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtLeftSide, center, width, height, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isAtTopLeftCorner(pointAtTopLeftCorner, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtTopSide, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtTopRightCorner, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtRightSide, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtBottomRightCorner, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtBottomSide, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtBottomLeftCorner, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopLeftCorner(insidePoint, center, width, height, ABSOLUTE_ERROR));

        // test static method with point, center and sizes
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtLeftSide, center, width, height));
        assertTrue(Rectangle.isAtTopLeftCorner(pointAtTopLeftCorner, center, width, height));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtTopSide, center, width, height));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtTopRightCorner, center, width, height));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtRightSide, center, width, height));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtBottomRightCorner, center, width, height));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtBottomSide, center, width, height));
        assertFalse(Rectangle.isAtTopLeftCorner(pointAtBottomLeftCorner, center, width, height));
        assertFalse(Rectangle.isAtTopLeftCorner(insidePoint, center, width, height));

        // test non-static method with point coordinates and threshold
        final var r = new Rectangle(left, top, right, bottom);

        assertFalse(r.isAtTopLeftCorner(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), ABSOLUTE_ERROR));
        assertTrue(r.isAtTopLeftCorner(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                ABSOLUTE_ERROR));
        assertFalse(r.isAtTopLeftCorner(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isAtTopLeftCorner(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                ABSOLUTE_ERROR));
        assertFalse(r.isAtTopLeftCorner(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isAtTopLeftCorner(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY(),
                ABSOLUTE_ERROR));
        assertFalse(r.isAtTopLeftCorner(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isAtTopLeftCorner(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(),
                ABSOLUTE_ERROR));
        assertFalse(r.isAtTopLeftCorner(insidePoint.getInhomX(), insidePoint.getInhomY(), ABSOLUTE_ERROR));

        // test non-static method with point coordinates
        assertFalse(r.isAtTopLeftCorner(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY()));
        assertTrue(r.isAtTopLeftCorner(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY()));
        assertFalse(r.isAtTopLeftCorner(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY()));
        assertFalse(r.isAtTopLeftCorner(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY()));
        assertFalse(r.isAtTopLeftCorner(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY()));
        assertFalse(r.isAtTopLeftCorner(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY()));
        assertFalse(r.isAtTopLeftCorner(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY()));
        assertFalse(r.isAtTopLeftCorner(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY()));
        assertFalse(r.isAtTopLeftCorner(insidePoint.getInhomX(), insidePoint.getInhomY()));

        // test non-static method with point and threshold
        assertFalse(r.isAtTopLeftCorner(pointAtLeftSide, ABSOLUTE_ERROR));
        assertTrue(r.isAtTopLeftCorner(pointAtTopLeftCorner, ABSOLUTE_ERROR));
        assertFalse(r.isAtTopLeftCorner(pointAtTopSide, ABSOLUTE_ERROR));
        assertFalse(r.isAtTopLeftCorner(pointAtTopRightCorner, ABSOLUTE_ERROR));
        assertFalse(r.isAtTopLeftCorner(pointAtRightSide, ABSOLUTE_ERROR));
        assertFalse(r.isAtTopLeftCorner(pointAtBottomRightCorner, ABSOLUTE_ERROR));
        assertFalse(r.isAtTopLeftCorner(pointAtBottomSide, ABSOLUTE_ERROR));
        assertFalse(r.isAtTopLeftCorner(pointAtBottomLeftCorner, ABSOLUTE_ERROR));
        assertFalse(r.isAtTopLeftCorner(insidePoint, ABSOLUTE_ERROR));

        // test non-static method with point
        assertFalse(r.isAtTopLeftCorner(pointAtLeftSide));
        assertTrue(r.isAtTopLeftCorner(pointAtTopLeftCorner));
        assertFalse(r.isAtTopLeftCorner(pointAtTopSide));
        assertFalse(r.isAtTopLeftCorner(pointAtTopRightCorner));
        assertFalse(r.isAtTopLeftCorner(pointAtRightSide));
        assertFalse(r.isAtTopLeftCorner(pointAtBottomRightCorner));
        assertFalse(r.isAtTopLeftCorner(pointAtBottomSide));
        assertFalse(r.isAtTopLeftCorner(pointAtBottomLeftCorner));
        assertFalse(r.isAtTopLeftCorner(insidePoint));
    }

    @Test
    void testIsAtTopSide() {
        final var randomizer = new UniformRandomizer();
        final var centerX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var centerY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var width = randomizer.nextDouble(0, MAX_RANDOM_VALUE);
        final var height = randomizer.nextDouble(0, MAX_RANDOM_VALUE);

        final var left = centerX - 0.5 * width;
        final var right = centerX + 0.5 * width;
        final var top = centerY + 0.5 * height;
        final var bottom = centerY - 0.5 * height;

        final var topLeft = new InhomogeneousPoint2D(left, top);
        final var bottomRight = new InhomogeneousPoint2D(right, bottom);
        final var center = new InhomogeneousPoint2D(centerX, centerY);

        final var pointAtLeftSide = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5), centerY);
        final var pointAtTopLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopSide = new InhomogeneousPoint2D(centerX,
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtRightSide = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5), centerY);
        final var pointAtBottomRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomSide = new InhomogeneousPoint2D(
                centerX, centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var insidePoint = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(-0.5, 0.5),
                centerY + height * randomizer.nextDouble(-0.5, 0.5));

        // test static method with point and corner coordinates and threshold
        assertFalse(Rectangle.isAtTopSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(),
                left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                left, top, right, bottom, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isAtTopSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(),
                left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(),
                left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopSide(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY(),
                left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(),
                left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopSide(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(),
                left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopSide(insidePoint.getInhomX(), insidePoint.getInhomY(),
                left, top, right, bottom, ABSOLUTE_ERROR));

        // test static method with point and corner coordinates
        assertFalse(Rectangle.isAtTopSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(),
                left, top, right, bottom));
        assertFalse(Rectangle.isAtTopSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                left, top, right, bottom));
        assertTrue(Rectangle.isAtTopSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(),
                left, top, right, bottom));
        assertFalse(Rectangle.isAtTopSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                left, top, right, bottom));
        assertFalse(Rectangle.isAtTopSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(),
                left, top, right, bottom));
        assertFalse(Rectangle.isAtTopSide(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY(),
                left, top, right, bottom));
        assertFalse(Rectangle.isAtTopSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(),
                left, top, right, bottom));
        assertFalse(Rectangle.isAtTopSide(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(),
                left, top, right, bottom));
        assertFalse(Rectangle.isAtTopSide(insidePoint.getInhomX(), insidePoint.getInhomY(), left, top, right, bottom));

        // test static method with point, corner coordinate and threshold
        assertFalse(Rectangle.isAtTopSide(pointAtLeftSide, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopSide(pointAtTopLeftCorner, left, top, right, bottom, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isAtTopSide(pointAtTopSide, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopSide(pointAtTopRightCorner, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopSide(pointAtRightSide, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopSide(pointAtBottomRightCorner, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopSide(pointAtBottomSide, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopSide(pointAtBottomLeftCorner, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopSide(insidePoint, left, top, right, bottom, ABSOLUTE_ERROR));

        // test static method with point, corner coordinate
        assertFalse(Rectangle.isAtTopSide(pointAtLeftSide, left, top, right, bottom));
        assertFalse(Rectangle.isAtTopSide(pointAtTopLeftCorner, left, top, right, bottom));
        assertTrue(Rectangle.isAtTopSide(pointAtTopSide, left, top, right, bottom));
        assertFalse(Rectangle.isAtTopSide(pointAtTopRightCorner, left, top, right, bottom));
        assertFalse(Rectangle.isAtTopSide(pointAtRightSide, left, top, right, bottom));
        assertFalse(Rectangle.isAtTopSide(pointAtBottomRightCorner, left, top, right, bottom));
        assertFalse(Rectangle.isAtTopSide(pointAtBottomSide, left, top, right, bottom));
        assertFalse(Rectangle.isAtTopSide(pointAtBottomLeftCorner, left, top, right, bottom));
        assertFalse(Rectangle.isAtTopSide(insidePoint, left, top, right, bottom));

        // test static method with point coordinates, corners and threshold
        assertFalse(Rectangle.isAtTopSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(),
                topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                topLeft, bottomRight, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isAtTopSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(),
                topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(),
                topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopSide(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY(),
                topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(),
                topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopSide(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(),
                topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopSide(insidePoint.getInhomX(), insidePoint.getInhomY(),
                topLeft, bottomRight, ABSOLUTE_ERROR));

        // test static method with point coordinates and corners
        assertFalse(Rectangle.isAtTopSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(),
                topLeft, bottomRight));
        assertFalse(Rectangle.isAtTopSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                topLeft, bottomRight));
        assertTrue(Rectangle.isAtTopSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), topLeft, bottomRight));
        assertFalse(Rectangle.isAtTopSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                topLeft, bottomRight));
        assertFalse(Rectangle.isAtTopSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(),
                topLeft, bottomRight));
        assertFalse(Rectangle.isAtTopSide(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY(),
                topLeft, bottomRight));
        assertFalse(Rectangle.isAtTopSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(),
                topLeft, bottomRight));
        assertFalse(Rectangle.isAtTopSide(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(),
                topLeft, bottomRight));
        assertFalse(Rectangle.isAtTopSide(insidePoint.getInhomX(), insidePoint.getInhomY(), topLeft, bottomRight));

        // test static method with point coordinates, corners and threshold
        assertFalse(Rectangle.isAtTopSide(pointAtLeftSide, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopSide(pointAtTopLeftCorner, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isAtTopSide(pointAtTopSide, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopSide(pointAtTopRightCorner, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopSide(pointAtRightSide, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopSide(pointAtBottomRightCorner, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopSide(pointAtBottomSide, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopSide(pointAtBottomLeftCorner, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopSide(insidePoint, topLeft, bottomRight, ABSOLUTE_ERROR));

        // test static method with point coordinates and corners
        assertFalse(Rectangle.isAtTopSide(pointAtLeftSide, topLeft, bottomRight));
        assertFalse(Rectangle.isAtTopSide(pointAtTopLeftCorner, topLeft, bottomRight));
        assertTrue(Rectangle.isAtTopSide(pointAtTopSide, topLeft, bottomRight));
        assertFalse(Rectangle.isAtTopSide(pointAtTopRightCorner, topLeft, bottomRight));
        assertFalse(Rectangle.isAtTopSide(pointAtRightSide, topLeft, bottomRight));
        assertFalse(Rectangle.isAtTopSide(pointAtBottomRightCorner, topLeft, bottomRight));
        assertFalse(Rectangle.isAtTopSide(pointAtBottomSide, topLeft, bottomRight));
        assertFalse(Rectangle.isAtTopSide(pointAtBottomLeftCorner, topLeft, bottomRight));
        assertFalse(Rectangle.isAtTopSide(insidePoint, topLeft, bottomRight));

        // test static method with point coordinates, center, sizes and threshold
        assertFalse(Rectangle.isAtTopSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(),
                center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                center, width, height, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isAtTopSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(),
                center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(),
                center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopSide(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY(),
                center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(),
                center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopSide(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(),
                center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopSide(insidePoint.getInhomX(), insidePoint.getInhomY(), center, width, height,
                ABSOLUTE_ERROR));

        // test static method with point coordinates, center and sizes
        assertFalse(Rectangle.isAtTopSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(),
                center, width, height));
        assertFalse(Rectangle.isAtTopSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                center, width, height));
        assertTrue(Rectangle.isAtTopSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(),
                center, width, height));
        assertFalse(Rectangle.isAtTopSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                center, width, height));
        assertFalse(Rectangle.isAtTopSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(),
                center, width, height));
        assertFalse(Rectangle.isAtTopSide(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY(),
                center, width, height));
        assertFalse(Rectangle.isAtTopSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(),
                center, width, height));
        assertFalse(Rectangle.isAtTopSide(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(),
                center, width, height));
        assertFalse(Rectangle.isAtTopSide(insidePoint.getInhomX(), insidePoint.getInhomY(), center, width, height));

        // test static method with point, center, sizes and threshold
        assertFalse(Rectangle.isAtTopSide(pointAtLeftSide, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopSide(pointAtTopLeftCorner, center, width, height, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isAtTopSide(pointAtTopSide, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopSide(pointAtTopRightCorner, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopSide(pointAtRightSide, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopSide(pointAtBottomRightCorner, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopSide(pointAtBottomSide, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopSide(pointAtBottomLeftCorner, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopSide(insidePoint, center, width, height, ABSOLUTE_ERROR));

        // test static method with point, center and sizes
        assertFalse(Rectangle.isAtTopSide(pointAtLeftSide, center, width, height));
        assertFalse(Rectangle.isAtTopSide(pointAtTopLeftCorner, center, width, height));
        assertTrue(Rectangle.isAtTopSide(pointAtTopSide, center, width, height));
        assertFalse(Rectangle.isAtTopSide(pointAtTopRightCorner, center, width, height));
        assertFalse(Rectangle.isAtTopSide(pointAtRightSide, center, width, height));
        assertFalse(Rectangle.isAtTopSide(pointAtBottomRightCorner, center, width, height));
        assertFalse(Rectangle.isAtTopSide(pointAtBottomSide, center, width, height));
        assertFalse(Rectangle.isAtTopSide(pointAtBottomLeftCorner, center, width, height));
        assertFalse(Rectangle.isAtTopSide(insidePoint, center, width, height));

        // test non-static method with point coordinates and threshold
        final var r = new Rectangle(left, top, right, bottom);

        assertFalse(r.isAtTopSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isAtTopSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(), ABSOLUTE_ERROR));
        assertTrue(r.isAtTopSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isAtTopSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isAtTopSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isAtTopSide(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY(),
                ABSOLUTE_ERROR));
        assertFalse(r.isAtTopSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isAtTopSide(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(),
                ABSOLUTE_ERROR));
        assertFalse(r.isAtTopSide(insidePoint.getInhomX(), insidePoint.getInhomY(), ABSOLUTE_ERROR));

        // test non-static method with point coordinates
        assertFalse(r.isAtTopSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY()));
        assertFalse(r.isAtTopSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY()));
        assertTrue(r.isAtTopSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY()));
        assertFalse(r.isAtTopSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY()));
        assertFalse(r.isAtTopSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY()));
        assertFalse(r.isAtTopSide(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY()));
        assertFalse(r.isAtTopSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY()));
        assertFalse(r.isAtTopSide(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY()));
        assertFalse(r.isAtTopSide(insidePoint.getInhomX(), insidePoint.getInhomY()));

        // test non-static method with point and threshold
        assertFalse(r.isAtTopSide(pointAtLeftSide, ABSOLUTE_ERROR));
        assertFalse(r.isAtTopSide(pointAtTopLeftCorner, ABSOLUTE_ERROR));
        assertTrue(r.isAtTopSide(pointAtTopSide, ABSOLUTE_ERROR));
        assertFalse(r.isAtTopSide(pointAtTopRightCorner, ABSOLUTE_ERROR));
        assertFalse(r.isAtTopSide(pointAtRightSide, ABSOLUTE_ERROR));
        assertFalse(r.isAtTopSide(pointAtBottomRightCorner, ABSOLUTE_ERROR));
        assertFalse(r.isAtTopSide(pointAtBottomSide, ABSOLUTE_ERROR));
        assertFalse(r.isAtTopSide(pointAtBottomLeftCorner, ABSOLUTE_ERROR));
        assertFalse(r.isAtTopSide(insidePoint, ABSOLUTE_ERROR));

        // test non-static method with point
        assertFalse(r.isAtTopSide(pointAtLeftSide));
        assertFalse(r.isAtTopSide(pointAtTopLeftCorner));
        assertTrue(r.isAtTopSide(pointAtTopSide));
        assertFalse(r.isAtTopSide(pointAtTopRightCorner));
        assertFalse(r.isAtTopSide(pointAtRightSide));
        assertFalse(r.isAtTopSide(pointAtBottomRightCorner));
        assertFalse(r.isAtTopSide(pointAtBottomSide));
        assertFalse(r.isAtTopSide(pointAtBottomLeftCorner));
        assertFalse(r.isAtTopSide(insidePoint));
    }

    @Test
    void testIsAtTopRightCorner() {
        final var randomizer = new UniformRandomizer();
        final var centerX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var centerY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var width = randomizer.nextDouble(0, MAX_RANDOM_VALUE);
        final var height = randomizer.nextDouble(0, MAX_RANDOM_VALUE);

        final var left = centerX - 0.5 * width;
        final var right = centerX + 0.5 * width;
        final var top = centerY + 0.5 * height;
        final var bottom = centerY - 0.5 * height;

        final var topLeft = new InhomogeneousPoint2D(left, top);
        final var bottomRight = new InhomogeneousPoint2D(right, bottom);
        final var center = new InhomogeneousPoint2D(centerX, centerY);

        final var pointAtLeftSide = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5), centerY);
        final var pointAtTopLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopSide = new InhomogeneousPoint2D(centerX,
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtRightSide = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5), centerY);
        final var pointAtBottomRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomSide = new InhomogeneousPoint2D(
                centerX, centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var insidePoint = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(-0.5, 0.5),
                centerY + height * randomizer.nextDouble(-0.5, 0.5));

        // test static method with point and corner coordinates and threshold
        assertFalse(Rectangle.isAtTopRightCorner(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(),
                left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(),
                left, top, right, bottom, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isAtTopRightCorner(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(),
                left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(),
                left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopRightCorner(insidePoint.getInhomX(), insidePoint.getInhomY(),
                left, top, right, bottom, ABSOLUTE_ERROR));

        // test static method with point and corner coordinates
        assertFalse(Rectangle.isAtTopRightCorner(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(),
                left, top, right, bottom));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                left, top, right, bottom));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(),
                left, top, right, bottom));
        assertTrue(Rectangle.isAtTopRightCorner(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                left, top, right, bottom));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(),
                left, top, right, bottom));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), left, top, right, bottom));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(),
                left, top, right, bottom));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), left, top, right, bottom));
        assertFalse(Rectangle.isAtTopRightCorner(insidePoint.getInhomX(), insidePoint.getInhomY(),
                left, top, right, bottom));

        // test static method with point, corner coordinate and threshold
        assertFalse(Rectangle.isAtTopRightCorner(pointAtLeftSide, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtTopLeftCorner, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtTopSide, left, top, right, bottom, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isAtTopRightCorner(pointAtTopRightCorner, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtRightSide, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtBottomRightCorner, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtBottomSide, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtBottomLeftCorner, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopRightCorner(insidePoint, left, top, right, bottom, ABSOLUTE_ERROR));

        // test static method with point, corner coordinate
        assertFalse(Rectangle.isAtTopRightCorner(pointAtLeftSide, left, top, right, bottom));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtTopLeftCorner, left, top, right, bottom));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtTopSide, left, top, right, bottom));
        assertTrue(Rectangle.isAtTopRightCorner(pointAtTopRightCorner, left, top, right, bottom));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtRightSide, left, top, right, bottom));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtBottomRightCorner, left, top, right, bottom));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtBottomSide, left, top, right, bottom));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtBottomLeftCorner, left, top, right, bottom));
        assertFalse(Rectangle.isAtTopRightCorner(insidePoint, left, top, right, bottom));

        // test static method with point coordinates, corners and threshold
        assertFalse(Rectangle.isAtTopRightCorner(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(),
                topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(),
                topLeft, bottomRight, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isAtTopRightCorner(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), topLeft,
                bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(),
                topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopRightCorner(insidePoint.getInhomX(), insidePoint.getInhomY(), topLeft, bottomRight,
                ABSOLUTE_ERROR));

        // test static method with point coordinates and corners
        assertFalse(Rectangle.isAtTopRightCorner(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), topLeft,
                bottomRight));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                topLeft, bottomRight));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), topLeft,
                bottomRight));
        assertTrue(Rectangle.isAtTopRightCorner(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                topLeft, bottomRight));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), topLeft,
                bottomRight));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), topLeft, bottomRight));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), topLeft,
                bottomRight));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), topLeft, bottomRight));
        assertFalse(Rectangle.isAtTopRightCorner(insidePoint.getInhomX(), insidePoint.getInhomY(), topLeft,
                bottomRight));

        // test static method with point coordinates, corners and threshold
        assertFalse(Rectangle.isAtTopRightCorner(pointAtLeftSide, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtTopLeftCorner, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtTopSide, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isAtTopRightCorner(pointAtTopRightCorner, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtRightSide, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtBottomRightCorner, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtBottomSide, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtBottomLeftCorner, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopRightCorner(insidePoint, topLeft, bottomRight, ABSOLUTE_ERROR));

        // test static method with point coordinates and corners
        assertFalse(Rectangle.isAtTopRightCorner(pointAtLeftSide, topLeft, bottomRight));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtTopLeftCorner, topLeft, bottomRight));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtTopSide, topLeft, bottomRight));
        assertTrue(Rectangle.isAtTopRightCorner(pointAtTopRightCorner, topLeft, bottomRight));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtRightSide, topLeft, bottomRight));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtBottomRightCorner, topLeft, bottomRight));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtBottomSide, topLeft, bottomRight));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtBottomLeftCorner, topLeft, bottomRight));
        assertFalse(Rectangle.isAtTopRightCorner(insidePoint, topLeft, bottomRight));

        // test static method with point coordinates, center, sizes and threshold
        assertFalse(Rectangle.isAtTopRightCorner(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), center,
                width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), center, width,
                height, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isAtTopRightCorner(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), center,
                width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), center,
                width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopRightCorner(insidePoint.getInhomX(), insidePoint.getInhomY(), center, width,
                height, ABSOLUTE_ERROR));

        // test static method with point coordinates, center and sizes
        assertFalse(Rectangle.isAtTopRightCorner(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), center,
                width, height));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                center, width, height));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), center, width,
                height));
        assertTrue(Rectangle.isAtTopRightCorner(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                center, width, height));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), center,
                width, height));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), center, width, height));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), center,
                width, height));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), center, width, height));
        assertFalse(Rectangle.isAtTopRightCorner(insidePoint.getInhomX(), insidePoint.getInhomY(), center, width,
                height));

        // test static method with point, center, sizes and threshold
        assertFalse(Rectangle.isAtTopRightCorner(pointAtLeftSide, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtTopLeftCorner, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtTopSide, center, width, height, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isAtTopRightCorner(pointAtTopRightCorner, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtRightSide, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtBottomRightCorner, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtBottomSide, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtBottomLeftCorner, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtTopRightCorner(insidePoint, center, width, height, ABSOLUTE_ERROR));

        // test static method with point, center and sizes
        assertFalse(Rectangle.isAtTopRightCorner(pointAtLeftSide, center, width, height));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtTopLeftCorner, center, width, height));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtTopSide, center, width, height));
        assertTrue(Rectangle.isAtTopRightCorner(pointAtTopRightCorner, center, width, height));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtRightSide, center, width, height));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtBottomRightCorner, center, width, height));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtBottomSide, center, width, height));
        assertFalse(Rectangle.isAtTopRightCorner(pointAtBottomLeftCorner, center, width, height));
        assertFalse(Rectangle.isAtTopRightCorner(insidePoint, center, width, height));

        // test non-static method with point coordinates and threshold
        final var r = new Rectangle(left, top, right, bottom);

        assertFalse(r.isAtTopRightCorner(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isAtTopRightCorner(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                ABSOLUTE_ERROR));
        assertFalse(r.isAtTopRightCorner(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), ABSOLUTE_ERROR));
        assertTrue(r.isAtTopRightCorner(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                ABSOLUTE_ERROR));
        assertFalse(r.isAtTopRightCorner(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isAtTopRightCorner(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY(),
                ABSOLUTE_ERROR));
        assertFalse(r.isAtTopRightCorner(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isAtTopRightCorner(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(),
                ABSOLUTE_ERROR));
        assertFalse(r.isAtTopRightCorner(insidePoint.getInhomX(), insidePoint.getInhomY(), ABSOLUTE_ERROR));

        // test non-static method with point coordinates
        assertFalse(r.isAtTopRightCorner(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY()));
        assertFalse(r.isAtTopRightCorner(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY()));
        assertFalse(r.isAtTopRightCorner(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY()));
        assertTrue(r.isAtTopRightCorner(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY()));
        assertFalse(r.isAtTopRightCorner(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY()));
        assertFalse(r.isAtTopRightCorner(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY()));
        assertFalse(r.isAtTopRightCorner(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY()));
        assertFalse(r.isAtTopRightCorner(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY()));
        assertFalse(r.isAtTopRightCorner(insidePoint.getInhomX(), insidePoint.getInhomY()));

        // test non-static method with point and threshold
        assertFalse(r.isAtTopRightCorner(pointAtLeftSide, ABSOLUTE_ERROR));
        assertFalse(r.isAtTopRightCorner(pointAtTopLeftCorner, ABSOLUTE_ERROR));
        assertFalse(r.isAtTopRightCorner(pointAtTopSide, ABSOLUTE_ERROR));
        assertTrue(r.isAtTopRightCorner(pointAtTopRightCorner, ABSOLUTE_ERROR));
        assertFalse(r.isAtTopRightCorner(pointAtRightSide, ABSOLUTE_ERROR));
        assertFalse(r.isAtTopRightCorner(pointAtBottomRightCorner, ABSOLUTE_ERROR));
        assertFalse(r.isAtTopRightCorner(pointAtBottomSide, ABSOLUTE_ERROR));
        assertFalse(r.isAtTopRightCorner(pointAtBottomLeftCorner, ABSOLUTE_ERROR));
        assertFalse(r.isAtTopRightCorner(insidePoint, ABSOLUTE_ERROR));

        // test non-static method with point
        assertFalse(r.isAtTopRightCorner(pointAtLeftSide));
        assertFalse(r.isAtTopRightCorner(pointAtTopLeftCorner));
        assertFalse(r.isAtTopRightCorner(pointAtTopSide));
        assertTrue(r.isAtTopRightCorner(pointAtTopRightCorner));
        assertFalse(r.isAtTopRightCorner(pointAtRightSide));
        assertFalse(r.isAtTopRightCorner(pointAtBottomRightCorner));
        assertFalse(r.isAtTopRightCorner(pointAtBottomSide));
        assertFalse(r.isAtTopRightCorner(pointAtBottomLeftCorner));
        assertFalse(r.isAtTopRightCorner(insidePoint));
    }

    @Test
    void testIsAtRightSide() {
        final var randomizer = new UniformRandomizer();
        final var centerX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var centerY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var width = randomizer.nextDouble(0, MAX_RANDOM_VALUE);
        final var height = randomizer.nextDouble(0, MAX_RANDOM_VALUE);

        final var left = centerX - 0.5 * width;
        final var right = centerX + 0.5 * width;
        final var top = centerY + 0.5 * height;
        final var bottom = centerY - 0.5 * height;

        final var topLeft = new InhomogeneousPoint2D(left, top);
        final var bottomRight = new InhomogeneousPoint2D(right, bottom);
        final var center = new InhomogeneousPoint2D(centerX, centerY);

        final var pointAtLeftSide = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5), centerY);
        final var pointAtTopLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopSide = new InhomogeneousPoint2D(
                centerX, centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtRightSide = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5), centerY);
        final var pointAtBottomRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomSide = new InhomogeneousPoint2D(
                centerX, centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var insidePoint = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(-0.5, 0.5),
                centerY + height * randomizer.nextDouble(-0.5, 0.5));

        // test static method with point and corner coordinates and threshold
        assertFalse(Rectangle.isAtRightSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), left, top, right,
                bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtRightSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(), left,
                top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtRightSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), left, top, right,
                bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtRightSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(), left,
                top, right, bottom, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isAtRightSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), left, top, right,
                bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtRightSide(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY(),
                left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtRightSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), left, top,
                right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtRightSide(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(),
                left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtRightSide(insidePoint.getInhomX(), insidePoint.getInhomY(), left, top, right, bottom,
                ABSOLUTE_ERROR));

        // test static method with point and corner coordinates
        assertFalse(Rectangle.isAtRightSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), left, top, right,
                bottom));
        assertFalse(Rectangle.isAtRightSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(), left,
                top, right, bottom));
        assertFalse(Rectangle.isAtRightSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), left, top, right,
                bottom));
        assertFalse(Rectangle.isAtRightSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(), left,
                top, right, bottom));
        assertTrue(Rectangle.isAtRightSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), left, top,
                right, bottom));
        assertFalse(Rectangle.isAtRightSide(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY(),
                left, top, right, bottom));
        assertFalse(Rectangle.isAtRightSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), left, top,
                right, bottom));
        assertFalse(Rectangle.isAtRightSide(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(),
                left, top, right, bottom));
        assertFalse(Rectangle.isAtRightSide(insidePoint.getInhomX(), insidePoint.getInhomY(), left, top, right,
                bottom));

        // test static method with point, corner coordinate and threshold
        assertFalse(Rectangle.isAtRightSide(pointAtLeftSide, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtRightSide(pointAtTopLeftCorner, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtRightSide(pointAtTopSide, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtRightSide(pointAtTopRightCorner, left, top, right, bottom, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isAtRightSide(pointAtRightSide, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtRightSide(pointAtBottomRightCorner, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtRightSide(pointAtBottomSide, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtRightSide(pointAtBottomLeftCorner, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtRightSide(insidePoint, left, top, right, bottom, ABSOLUTE_ERROR));

        // test static method with point, corner coordinate
        assertFalse(Rectangle.isAtRightSide(pointAtLeftSide, left, top, right, bottom));
        assertFalse(Rectangle.isAtRightSide(pointAtTopLeftCorner, left, top, right, bottom));
        assertFalse(Rectangle.isAtRightSide(pointAtTopSide, left, top, right, bottom));
        assertFalse(Rectangle.isAtRightSide(pointAtTopRightCorner, left, top, right, bottom));
        assertTrue(Rectangle.isAtRightSide(pointAtRightSide, left, top, right, bottom));
        assertFalse(Rectangle.isAtRightSide(pointAtBottomRightCorner, left, top, right, bottom));
        assertFalse(Rectangle.isAtRightSide(pointAtBottomSide, left, top, right, bottom));
        assertFalse(Rectangle.isAtRightSide(pointAtBottomLeftCorner, left, top, right, bottom));
        assertFalse(Rectangle.isAtRightSide(insidePoint, left, top, right, bottom));

        // test static method with point coordinates, corners and threshold
        assertFalse(Rectangle.isAtRightSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), topLeft,
                bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtRightSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtRightSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), topLeft,
                bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtRightSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                topLeft, bottomRight, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isAtRightSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), topLeft,
                bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtRightSide(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY(),
                topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtRightSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), topLeft,
                bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtRightSide(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(),
                topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtRightSide(insidePoint.getInhomX(), insidePoint.getInhomY(), topLeft, bottomRight,
                ABSOLUTE_ERROR));

        // test static method with point coordinates and corners
        assertFalse(Rectangle.isAtRightSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), topLeft,
                bottomRight));
        assertFalse(Rectangle.isAtRightSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                topLeft, bottomRight));
        assertFalse(Rectangle.isAtRightSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), topLeft,
                bottomRight));
        assertFalse(Rectangle.isAtRightSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                topLeft, bottomRight));
        assertTrue(Rectangle.isAtRightSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), topLeft,
                bottomRight));
        assertFalse(Rectangle.isAtRightSide(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY(),
                topLeft, bottomRight));
        assertFalse(Rectangle.isAtRightSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), topLeft,
                bottomRight));
        assertFalse(Rectangle.isAtRightSide(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(),
                topLeft, bottomRight));
        assertFalse(Rectangle.isAtRightSide(insidePoint.getInhomX(), insidePoint.getInhomY(), topLeft, bottomRight));

        // test static method with point coordinates, corners and threshold
        assertFalse(Rectangle.isAtRightSide(pointAtLeftSide, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtRightSide(pointAtTopLeftCorner, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtRightSide(pointAtTopSide, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtRightSide(pointAtTopRightCorner, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isAtRightSide(pointAtRightSide, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtRightSide(pointAtBottomRightCorner, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtRightSide(pointAtBottomSide, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtRightSide(pointAtBottomLeftCorner, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtRightSide(insidePoint, topLeft, bottomRight, ABSOLUTE_ERROR));

        // test static method with point coordinates and corners
        assertFalse(Rectangle.isAtRightSide(pointAtLeftSide, topLeft, bottomRight));
        assertFalse(Rectangle.isAtRightSide(pointAtTopLeftCorner, topLeft, bottomRight));
        assertFalse(Rectangle.isAtRightSide(pointAtTopSide, topLeft, bottomRight));
        assertFalse(Rectangle.isAtRightSide(pointAtTopRightCorner, topLeft, bottomRight));
        assertTrue(Rectangle.isAtRightSide(pointAtRightSide, topLeft, bottomRight));
        assertFalse(Rectangle.isAtRightSide(pointAtBottomRightCorner, topLeft, bottomRight));
        assertFalse(Rectangle.isAtRightSide(pointAtBottomSide, topLeft, bottomRight));
        assertFalse(Rectangle.isAtRightSide(pointAtBottomLeftCorner, topLeft, bottomRight));
        assertFalse(Rectangle.isAtRightSide(insidePoint, topLeft, bottomRight));

        // test static method with point coordinates, center, sizes and threshold
        assertFalse(Rectangle.isAtRightSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), center, width,
                height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtRightSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(), center,
                width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtRightSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), center, width,
                height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtRightSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                center, width, height, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isAtRightSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), center, width,
                height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtRightSide(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY(),
                center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtRightSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), center, width,
                height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtRightSide(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(),
                center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtRightSide(insidePoint.getInhomX(), insidePoint.getInhomY(), center, width, height,
                ABSOLUTE_ERROR));

        // test static method with point coordinates, center and sizes
        assertFalse(Rectangle.isAtRightSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), center, width,
                height));
        assertFalse(Rectangle.isAtRightSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(), center,
                width, height));
        assertFalse(Rectangle.isAtRightSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), center, width,
                height));
        assertFalse(Rectangle.isAtRightSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                center, width, height));
        assertTrue(Rectangle.isAtRightSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), center, width,
                height));
        assertFalse(Rectangle.isAtRightSide(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY(),
                center, width, height));
        assertFalse(Rectangle.isAtRightSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), center, width,
                height));
        assertFalse(Rectangle.isAtRightSide(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(),
                center, width, height));
        assertFalse(Rectangle.isAtRightSide(insidePoint.getInhomX(), insidePoint.getInhomY(), center, width, height));

        // test static method with point, center, sizes and threshold
        assertFalse(Rectangle.isAtRightSide(pointAtLeftSide, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtRightSide(pointAtTopLeftCorner, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtRightSide(pointAtTopSide, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtRightSide(pointAtTopRightCorner, center, width, height, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isAtRightSide(pointAtRightSide, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtRightSide(pointAtBottomRightCorner, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtRightSide(pointAtBottomSide, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtRightSide(pointAtBottomLeftCorner, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtRightSide(insidePoint, center, width, height, ABSOLUTE_ERROR));

        // test static method with point, center and sizes
        assertFalse(Rectangle.isAtRightSide(pointAtLeftSide, center, width, height));
        assertFalse(Rectangle.isAtRightSide(pointAtTopLeftCorner, center, width, height));
        assertFalse(Rectangle.isAtRightSide(pointAtTopSide, center, width, height));
        assertFalse(Rectangle.isAtRightSide(pointAtTopRightCorner, center, width, height));
        assertTrue(Rectangle.isAtRightSide(pointAtRightSide, center, width, height));
        assertFalse(Rectangle.isAtRightSide(pointAtBottomRightCorner, center, width, height));
        assertFalse(Rectangle.isAtRightSide(pointAtBottomSide, center, width, height));
        assertFalse(Rectangle.isAtRightSide(pointAtBottomLeftCorner, center, width, height));
        assertFalse(Rectangle.isAtRightSide(insidePoint, center, width, height));

        // test non-static method with point coordinates and threshold
        final var r = new Rectangle(left, top, right, bottom);

        assertFalse(r.isAtRightSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isAtRightSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                ABSOLUTE_ERROR));
        assertFalse(r.isAtRightSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isAtRightSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                ABSOLUTE_ERROR));
        assertTrue(r.isAtRightSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isAtRightSide(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY(),
                ABSOLUTE_ERROR));
        assertFalse(r.isAtRightSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isAtRightSide(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(),
                ABSOLUTE_ERROR));
        assertFalse(r.isAtRightSide(insidePoint.getInhomX(), insidePoint.getInhomY(), ABSOLUTE_ERROR));

        // test non-static method with point coordinates
        assertFalse(r.isAtRightSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY()));
        assertFalse(r.isAtRightSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY()));
        assertFalse(r.isAtRightSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY()));
        assertFalse(r.isAtRightSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY()));
        assertTrue(r.isAtRightSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY()));
        assertFalse(r.isAtRightSide(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY()));
        assertFalse(r.isAtRightSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY()));
        assertFalse(r.isAtRightSide(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY()));
        assertFalse(r.isAtRightSide(insidePoint.getInhomX(), insidePoint.getInhomY()));

        // test non-static method with point and threshold
        assertFalse(r.isAtRightSide(pointAtLeftSide, ABSOLUTE_ERROR));
        assertFalse(r.isAtRightSide(pointAtTopLeftCorner, ABSOLUTE_ERROR));
        assertFalse(r.isAtRightSide(pointAtTopSide, ABSOLUTE_ERROR));
        assertFalse(r.isAtRightSide(pointAtTopRightCorner, ABSOLUTE_ERROR));
        assertTrue(r.isAtRightSide(pointAtRightSide, ABSOLUTE_ERROR));
        assertFalse(r.isAtRightSide(pointAtBottomRightCorner, ABSOLUTE_ERROR));
        assertFalse(r.isAtRightSide(pointAtBottomSide, ABSOLUTE_ERROR));
        assertFalse(r.isAtRightSide(pointAtBottomLeftCorner, ABSOLUTE_ERROR));
        assertFalse(r.isAtRightSide(insidePoint, ABSOLUTE_ERROR));

        // test non-static method with point
        assertFalse(r.isAtRightSide(pointAtLeftSide));
        assertFalse(r.isAtRightSide(pointAtTopLeftCorner));
        assertFalse(r.isAtRightSide(pointAtTopSide));
        assertFalse(r.isAtRightSide(pointAtTopRightCorner));
        assertTrue(r.isAtRightSide(pointAtRightSide));
        assertFalse(r.isAtRightSide(pointAtBottomRightCorner));
        assertFalse(r.isAtRightSide(pointAtBottomSide));
        assertFalse(r.isAtRightSide(pointAtBottomLeftCorner));
        assertFalse(r.isAtRightSide(insidePoint));
    }

    @Test
    void testIsAtBottomRightCorner() {
        final var randomizer = new UniformRandomizer();
        final var centerX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var centerY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var width = randomizer.nextDouble(0, MAX_RANDOM_VALUE);
        final var height = randomizer.nextDouble(0, MAX_RANDOM_VALUE);

        final var left = centerX - 0.5 * width;
        final var right = centerX + 0.5 * width;
        final var top = centerY + 0.5 * height;
        final var bottom = centerY - 0.5 * height;

        final var topLeft = new InhomogeneousPoint2D(left, top);
        final var bottomRight = new InhomogeneousPoint2D(right, bottom);
        final var center = new InhomogeneousPoint2D(centerX, centerY);

        final var pointAtLeftSide = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5), centerY);
        final var pointAtTopLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopSide = new InhomogeneousPoint2D(centerX,
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtRightSide = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5), centerY);
        final var pointAtBottomRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomSide = new InhomogeneousPoint2D(
                centerX, centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var insidePoint = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(-0.5, 0.5),
                centerY + height * randomizer.nextDouble(-0.5, 0.5));

        // test static method with point and corner coordinates and threshold
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), left,
                top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), left, top,
                right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtTopRightCorner.getInhomX(),
                pointAtTopRightCorner.getInhomY(), left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), left,
                top, right, bottom, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isAtBottomRightCorner(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), left,
                top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomRightCorner(insidePoint.getInhomX(), insidePoint.getInhomY(), left, top, right,
                bottom, ABSOLUTE_ERROR));

        // test static method with point and corner coordinates
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), left,
                top, right, bottom));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                left, top, right, bottom));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), left, top,
                right, bottom));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtTopRightCorner.getInhomX(),
                pointAtTopRightCorner.getInhomY(), left, top, right, bottom));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), left,
                top, right, bottom));
        assertTrue(Rectangle.isAtBottomRightCorner(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), left, top, right, bottom));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), left,
                top, right, bottom));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), left, top, right, bottom));
        assertFalse(Rectangle.isAtBottomRightCorner(insidePoint.getInhomX(), insidePoint.getInhomY(), left, top, right,
                bottom));

        // test static method with point, corner coordinate and threshold
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtLeftSide, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtTopLeftCorner, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtTopSide, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtTopRightCorner, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtRightSide, left, top, right, bottom, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isAtBottomRightCorner(pointAtBottomRightCorner, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtBottomSide, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtBottomLeftCorner, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomRightCorner(insidePoint, left, top, right, bottom, ABSOLUTE_ERROR));

        // test static method with point, corner coordinate
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtLeftSide, left, top, right, bottom));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtTopLeftCorner, left, top, right, bottom));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtTopSide, left, top, right, bottom));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtTopRightCorner, left, top, right, bottom));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtRightSide, left, top, right, bottom));
        assertTrue(Rectangle.isAtBottomRightCorner(pointAtBottomRightCorner, left, top, right, bottom));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtBottomSide, left, top, right, bottom));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtBottomLeftCorner, left, top, right, bottom));
        assertFalse(Rectangle.isAtBottomRightCorner(insidePoint, left, top, right, bottom));

        // test static method with point coordinates, corners and threshold
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), topLeft,
                bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), topLeft,
                bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtTopRightCorner.getInhomX(),
                pointAtTopRightCorner.getInhomY(), topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(),
                topLeft, bottomRight, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isAtBottomRightCorner(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(),
                topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomRightCorner(insidePoint.getInhomX(), insidePoint.getInhomY(), topLeft,
                bottomRight, ABSOLUTE_ERROR));

        // test static method with point coordinates and corners
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), topLeft,
                bottomRight));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                topLeft, bottomRight));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), topLeft,
                bottomRight));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtTopRightCorner.getInhomX(),
                pointAtTopRightCorner.getInhomY(), topLeft, bottomRight));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(),
                topLeft, bottomRight));
        assertTrue(Rectangle.isAtBottomRightCorner(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), topLeft, bottomRight));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(),
                topLeft, bottomRight));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), topLeft, bottomRight));
        assertFalse(Rectangle.isAtBottomRightCorner(insidePoint.getInhomX(), insidePoint.getInhomY(), topLeft,
                bottomRight));

        // test static method with point coordinates, corners and threshold
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtLeftSide, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtTopLeftCorner, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtTopSide, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtTopRightCorner, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtRightSide, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isAtBottomRightCorner(pointAtBottomRightCorner, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtBottomSide, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtBottomLeftCorner, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomRightCorner(insidePoint, topLeft, bottomRight, ABSOLUTE_ERROR));

        // test static method with point coordinates and corners
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtLeftSide, topLeft, bottomRight));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtTopLeftCorner, topLeft, bottomRight));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtTopSide, topLeft, bottomRight));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtTopRightCorner, topLeft, bottomRight));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtRightSide, topLeft, bottomRight));
        assertTrue(Rectangle.isAtBottomRightCorner(pointAtBottomRightCorner, topLeft, bottomRight));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtBottomSide, topLeft, bottomRight));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtBottomLeftCorner, topLeft, bottomRight));
        assertFalse(Rectangle.isAtBottomRightCorner(insidePoint, topLeft, bottomRight));

        // test static method with point coordinates, center, sizes and threshold
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), center,
                width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), center,
                width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtTopRightCorner.getInhomX(),
                pointAtTopRightCorner.getInhomY(), center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), center,
                width, height, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isAtBottomRightCorner(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(),
                center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomRightCorner(insidePoint.getInhomX(), insidePoint.getInhomY(), center, width,
                height, ABSOLUTE_ERROR));

        // test static method with point coordinates, center and sizes
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), center,
                width, height));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                center, width, height));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), center,
                width, height));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtTopRightCorner.getInhomX(),
                pointAtTopRightCorner.getInhomY(), center, width, height));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), center,
                width, height));
        assertTrue(Rectangle.isAtBottomRightCorner(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), center, width, height));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(),
                center, width, height));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), center, width, height));
        assertFalse(Rectangle.isAtBottomRightCorner(insidePoint.getInhomX(), insidePoint.getInhomY(), center, width,
                height));

        // test static method with point, center, sizes and threshold
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtLeftSide, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtTopLeftCorner, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtTopSide, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtTopRightCorner, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtRightSide, center, width, height, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isAtBottomRightCorner(pointAtBottomRightCorner, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtBottomSide, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtBottomLeftCorner, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomRightCorner(insidePoint, center, width, height, ABSOLUTE_ERROR));

        // test static method with point, center and sizes
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtLeftSide, center, width, height));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtTopLeftCorner, center, width, height));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtTopSide, center, width, height));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtTopRightCorner, center, width, height));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtRightSide, center, width, height));
        assertTrue(Rectangle.isAtBottomRightCorner(pointAtBottomRightCorner, center, width, height));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtBottomSide, center, width, height));
        assertFalse(Rectangle.isAtBottomRightCorner(pointAtBottomLeftCorner, center, width, height));
        assertFalse(Rectangle.isAtBottomRightCorner(insidePoint, center, width, height));

        // test non-static method with point coordinates and threshold
        final var r = new Rectangle(left, top, right, bottom);

        assertFalse(r.isAtBottomRightCorner(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isAtBottomRightCorner(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                ABSOLUTE_ERROR));
        assertFalse(r.isAtBottomRightCorner(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isAtBottomRightCorner(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                ABSOLUTE_ERROR));
        assertFalse(r.isAtBottomRightCorner(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(),
                ABSOLUTE_ERROR));
        assertTrue(r.isAtBottomRightCorner(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY(),
                ABSOLUTE_ERROR));
        assertFalse(r.isAtBottomRightCorner(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(),
                ABSOLUTE_ERROR));
        assertFalse(r.isAtBottomRightCorner(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(),
                ABSOLUTE_ERROR));
        assertFalse(r.isAtBottomRightCorner(insidePoint.getInhomX(), insidePoint.getInhomY(), ABSOLUTE_ERROR));

        // test non-static method with point coordinates
        assertFalse(r.isAtBottomRightCorner(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY()));
        assertFalse(r.isAtBottomRightCorner(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY()));
        assertFalse(r.isAtBottomRightCorner(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY()));
        assertFalse(r.isAtBottomRightCorner(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY()));
        assertFalse(r.isAtBottomRightCorner(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY()));
        assertTrue(r.isAtBottomRightCorner(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY()));
        assertFalse(r.isAtBottomRightCorner(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY()));
        assertFalse(r.isAtBottomRightCorner(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY()));
        assertFalse(r.isAtBottomRightCorner(insidePoint.getInhomX(), insidePoint.getInhomY()));

        // test non-static method with point and threshold
        assertFalse(r.isAtBottomRightCorner(pointAtLeftSide, ABSOLUTE_ERROR));
        assertFalse(r.isAtBottomRightCorner(pointAtTopLeftCorner, ABSOLUTE_ERROR));
        assertFalse(r.isAtBottomRightCorner(pointAtTopSide, ABSOLUTE_ERROR));
        assertFalse(r.isAtBottomRightCorner(pointAtTopRightCorner, ABSOLUTE_ERROR));
        assertFalse(r.isAtBottomRightCorner(pointAtRightSide, ABSOLUTE_ERROR));
        assertTrue(r.isAtBottomRightCorner(pointAtBottomRightCorner, ABSOLUTE_ERROR));
        assertFalse(r.isAtBottomRightCorner(pointAtBottomSide, ABSOLUTE_ERROR));
        assertFalse(r.isAtBottomRightCorner(pointAtBottomLeftCorner, ABSOLUTE_ERROR));
        assertFalse(r.isAtBottomRightCorner(insidePoint, ABSOLUTE_ERROR));

        // test non-static method with point
        assertFalse(r.isAtBottomRightCorner(pointAtLeftSide));
        assertFalse(r.isAtBottomRightCorner(pointAtTopLeftCorner));
        assertFalse(r.isAtBottomRightCorner(pointAtTopSide));
        assertFalse(r.isAtBottomRightCorner(pointAtTopRightCorner));
        assertFalse(r.isAtBottomRightCorner(pointAtRightSide));
        assertTrue(r.isAtBottomRightCorner(pointAtBottomRightCorner));
        assertFalse(r.isAtBottomRightCorner(pointAtBottomSide));
        assertFalse(r.isAtBottomRightCorner(pointAtBottomLeftCorner));
        assertFalse(r.isAtBottomRightCorner(insidePoint));
    }

    @Test
    void testIsAtBottomSide() {
        final var randomizer = new UniformRandomizer();
        final var centerX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var centerY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var width = randomizer.nextDouble(0, MAX_RANDOM_VALUE);
        final var height = randomizer.nextDouble(0, MAX_RANDOM_VALUE);

        final var left = centerX - 0.5 * width;
        final var right = centerX + 0.5 * width;
        final var top = centerY + 0.5 * height;
        final var bottom = centerY - 0.5 * height;

        final var topLeft = new InhomogeneousPoint2D(left, top);
        final var bottomRight = new InhomogeneousPoint2D(right, bottom);
        final var center = new InhomogeneousPoint2D(centerX, centerY);

        final var pointAtLeftSide = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5), centerY);
        final var pointAtTopLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopSide = new InhomogeneousPoint2D(
                centerX, centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtRightSide = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5), centerY);
        final var pointAtBottomRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomSide = new InhomogeneousPoint2D(
                centerX, centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var insidePoint = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(-0.5, 0.5),
                centerY + height * randomizer.nextDouble(-0.5, 0.5));

        // test static method with point and corner coordinates and threshold
        assertFalse(Rectangle.isAtBottomSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), left, top, right,
                bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(), left,
                top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), left, top, right,
                bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), left, top,
                right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomSide(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY(),
                left, top, right, bottom, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isAtBottomSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), left, top,
                right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomSide(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(),
                left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomSide(insidePoint.getInhomX(), insidePoint.getInhomY(), left, top, right,
                bottom, ABSOLUTE_ERROR));

        // test static method with point and corner coordinates
        assertFalse(Rectangle.isAtBottomSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), left, top, right,
                bottom));
        assertFalse(Rectangle.isAtBottomSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(), left,
                top, right, bottom));
        assertFalse(Rectangle.isAtBottomSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), left, top, right,
                bottom));
        assertFalse(Rectangle.isAtBottomSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                left, top, right, bottom));
        assertFalse(Rectangle.isAtBottomSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), left, top,
                right, bottom));
        assertFalse(Rectangle.isAtBottomSide(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY(),
                left, top, right, bottom));
        assertTrue(Rectangle.isAtBottomSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), left, top,
                right, bottom));
        assertFalse(Rectangle.isAtBottomSide(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(),
                left, top, right, bottom));
        assertFalse(Rectangle.isAtBottomSide(insidePoint.getInhomX(), insidePoint.getInhomY(), left, top, right,
                bottom));

        // test static method with point, corner coordinate and threshold
        assertFalse(Rectangle.isAtBottomSide(pointAtLeftSide, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomSide(pointAtTopLeftCorner, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomSide(pointAtTopSide, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomSide(pointAtTopRightCorner, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomSide(pointAtRightSide, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomSide(pointAtBottomRightCorner, left, top, right, bottom, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isAtBottomSide(pointAtBottomSide, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomSide(pointAtBottomLeftCorner, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomSide(insidePoint, left, top, right, bottom, ABSOLUTE_ERROR));

        // test static method with point, corner coordinate
        assertFalse(Rectangle.isAtBottomSide(pointAtLeftSide, left, top, right, bottom));
        assertFalse(Rectangle.isAtBottomSide(pointAtTopLeftCorner, left, top, right, bottom));
        assertFalse(Rectangle.isAtBottomSide(pointAtTopSide, left, top, right, bottom));
        assertFalse(Rectangle.isAtBottomSide(pointAtTopRightCorner, left, top, right, bottom));
        assertFalse(Rectangle.isAtBottomSide(pointAtRightSide, left, top, right, bottom));
        assertFalse(Rectangle.isAtBottomSide(pointAtBottomRightCorner, left, top, right, bottom));
        assertTrue(Rectangle.isAtBottomSide(pointAtBottomSide, left, top, right, bottom));
        assertFalse(Rectangle.isAtBottomSide(pointAtBottomLeftCorner, left, top, right, bottom));
        assertFalse(Rectangle.isAtBottomSide(insidePoint, left, top, right, bottom));

        // test static method with point coordinates, corners and threshold
        assertFalse(Rectangle.isAtBottomSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), topLeft,
                bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), topLeft,
                bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), topLeft,
                bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomSide(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY(),
                topLeft, bottomRight, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isAtBottomSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), topLeft,
                bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomSide(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(),
                topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomSide(insidePoint.getInhomX(), insidePoint.getInhomY(), topLeft, bottomRight,
                ABSOLUTE_ERROR));

        // test static method with point coordinates and corners
        assertFalse(Rectangle.isAtBottomSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), topLeft,
                bottomRight));
        assertFalse(Rectangle.isAtBottomSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                topLeft, bottomRight));
        assertFalse(Rectangle.isAtBottomSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), topLeft,
                bottomRight));
        assertFalse(Rectangle.isAtBottomSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                topLeft, bottomRight));
        assertFalse(Rectangle.isAtBottomSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), topLeft,
                bottomRight));
        assertFalse(Rectangle.isAtBottomSide(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY(),
                topLeft, bottomRight));
        assertTrue(Rectangle.isAtBottomSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), topLeft,
                bottomRight));
        assertFalse(Rectangle.isAtBottomSide(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(),
                topLeft, bottomRight));
        assertFalse(Rectangle.isAtBottomSide(insidePoint.getInhomX(), insidePoint.getInhomY(), topLeft, bottomRight));

        // test static method with point coordinates, corners and threshold
        assertFalse(Rectangle.isAtBottomSide(pointAtLeftSide, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomSide(pointAtTopLeftCorner, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomSide(pointAtTopSide, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomSide(pointAtTopRightCorner, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomSide(pointAtRightSide, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomSide(pointAtBottomRightCorner, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isAtBottomSide(pointAtBottomSide, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomSide(pointAtBottomLeftCorner, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomSide(insidePoint, topLeft, bottomRight, ABSOLUTE_ERROR));

        // test static method with point coordinates and corners
        assertFalse(Rectangle.isAtBottomSide(pointAtLeftSide, topLeft, bottomRight));
        assertFalse(Rectangle.isAtBottomSide(pointAtTopLeftCorner, topLeft, bottomRight));
        assertFalse(Rectangle.isAtBottomSide(pointAtTopSide, topLeft, bottomRight));
        assertFalse(Rectangle.isAtBottomSide(pointAtTopRightCorner, topLeft, bottomRight));
        assertFalse(Rectangle.isAtBottomSide(pointAtRightSide, topLeft, bottomRight));
        assertFalse(Rectangle.isAtBottomSide(pointAtBottomRightCorner, topLeft, bottomRight));
        assertTrue(Rectangle.isAtBottomSide(pointAtBottomSide, topLeft, bottomRight));
        assertFalse(Rectangle.isAtBottomSide(pointAtBottomLeftCorner, topLeft, bottomRight));
        assertFalse(Rectangle.isAtBottomSide(insidePoint, topLeft, bottomRight));

        // test static method with point coordinates, center, sizes and threshold
        assertFalse(Rectangle.isAtBottomSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), center, width,
                height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(), center,
                width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), center, width,
                height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), center, width,
                height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomSide(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY(),
                center, width, height, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isAtBottomSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), center, width,
                height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomSide(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(),
                center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomSide(insidePoint.getInhomX(), insidePoint.getInhomY(), center, width, height,
                ABSOLUTE_ERROR));

        // test static method with point coordinates, center and sizes
        assertFalse(Rectangle.isAtBottomSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), center, width,
                height));
        assertFalse(Rectangle.isAtBottomSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(), center,
                width, height));
        assertFalse(Rectangle.isAtBottomSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), center, width,
                height));
        assertFalse(Rectangle.isAtBottomSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                center, width, height));
        assertFalse(Rectangle.isAtBottomSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), center, width,
                height));
        assertFalse(Rectangle.isAtBottomSide(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY(),
                center, width, height));
        assertTrue(Rectangle.isAtBottomSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), center,
                width, height));
        assertFalse(Rectangle.isAtBottomSide(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(),
                center, width, height));
        assertFalse(Rectangle.isAtBottomSide(insidePoint.getInhomX(), insidePoint.getInhomY(), center, width, height));

        // test static method with point, center, sizes and threshold
        assertFalse(Rectangle.isAtBottomSide(pointAtLeftSide, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomSide(pointAtTopLeftCorner, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomSide(pointAtTopSide, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomSide(pointAtTopRightCorner, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomSide(pointAtRightSide, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomSide(pointAtBottomRightCorner, center, width, height, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isAtBottomSide(pointAtBottomSide, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomSide(pointAtBottomLeftCorner, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomSide(insidePoint, center, width, height, ABSOLUTE_ERROR));

        // test static method with point, center and sizes
        assertFalse(Rectangle.isAtBottomSide(pointAtLeftSide, center, width, height));
        assertFalse(Rectangle.isAtBottomSide(pointAtTopLeftCorner, center, width, height));
        assertFalse(Rectangle.isAtBottomSide(pointAtTopSide, center, width, height));
        assertFalse(Rectangle.isAtBottomSide(pointAtTopRightCorner, center, width, height));
        assertFalse(Rectangle.isAtBottomSide(pointAtRightSide, center, width, height));
        assertFalse(Rectangle.isAtBottomSide(pointAtBottomRightCorner, center, width, height));
        assertTrue(Rectangle.isAtBottomSide(pointAtBottomSide, center, width, height));
        assertFalse(Rectangle.isAtBottomSide(pointAtBottomLeftCorner, center, width, height));
        assertFalse(Rectangle.isAtBottomSide(insidePoint, center, width, height));

        // test non-static method with point coordinates and threshold
        final var r = new Rectangle(left, top, right, bottom);

        assertFalse(r.isAtBottomSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isAtBottomSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                ABSOLUTE_ERROR));
        assertFalse(r.isAtBottomSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isAtBottomSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                ABSOLUTE_ERROR));
        assertFalse(r.isAtBottomSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isAtBottomSide(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY(),
                ABSOLUTE_ERROR));
        assertTrue(r.isAtBottomSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isAtBottomSide(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(),
                ABSOLUTE_ERROR));
        assertFalse(r.isAtBottomSide(insidePoint.getInhomX(), insidePoint.getInhomY(), ABSOLUTE_ERROR));

        // test non-static method with point coordinates
        assertFalse(r.isAtBottomSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY()));
        assertFalse(r.isAtBottomSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY()));
        assertFalse(r.isAtBottomSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY()));
        assertFalse(r.isAtBottomSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY()));
        assertFalse(r.isAtBottomSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY()));
        assertFalse(r.isAtBottomSide(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY()));
        assertTrue(r.isAtBottomSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY()));
        assertFalse(r.isAtBottomSide(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY()));
        assertFalse(r.isAtBottomSide(insidePoint.getInhomX(), insidePoint.getInhomY()));

        // test non-static method with point and threshold
        assertFalse(r.isAtBottomSide(pointAtLeftSide, ABSOLUTE_ERROR));
        assertFalse(r.isAtBottomSide(pointAtTopLeftCorner, ABSOLUTE_ERROR));
        assertFalse(r.isAtBottomSide(pointAtTopSide, ABSOLUTE_ERROR));
        assertFalse(r.isAtBottomSide(pointAtTopRightCorner, ABSOLUTE_ERROR));
        assertFalse(r.isAtBottomSide(pointAtRightSide, ABSOLUTE_ERROR));
        assertFalse(r.isAtBottomSide(pointAtBottomRightCorner, ABSOLUTE_ERROR));
        assertTrue(r.isAtBottomSide(pointAtBottomSide, ABSOLUTE_ERROR));
        assertFalse(r.isAtBottomSide(pointAtBottomLeftCorner, ABSOLUTE_ERROR));
        assertFalse(r.isAtBottomSide(insidePoint, ABSOLUTE_ERROR));

        // test non-static method with point
        assertFalse(r.isAtBottomSide(pointAtLeftSide));
        assertFalse(r.isAtBottomSide(pointAtTopLeftCorner));
        assertFalse(r.isAtBottomSide(pointAtTopSide));
        assertFalse(r.isAtBottomSide(pointAtTopRightCorner));
        assertFalse(r.isAtBottomSide(pointAtRightSide));
        assertFalse(r.isAtBottomSide(pointAtBottomRightCorner));
        assertTrue(r.isAtBottomSide(pointAtBottomSide));
        assertFalse(r.isAtBottomSide(pointAtBottomLeftCorner));
        assertFalse(r.isAtBottomSide(insidePoint));
    }

    @Test
    void testIsAtBottomLeftCorner() {
        final var randomizer = new UniformRandomizer();
        final var centerX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var centerY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var width = randomizer.nextDouble(0, MAX_RANDOM_VALUE);
        final var height = randomizer.nextDouble(0, MAX_RANDOM_VALUE);

        final var left = centerX - 0.5 * width;
        final var right = centerX + 0.5 * width;
        final var top = centerY + 0.5 * height;
        final var bottom = centerY - 0.5 * height;

        final var topLeft = new InhomogeneousPoint2D(left, top);
        final var bottomRight = new InhomogeneousPoint2D(right, bottom);
        final var center = new InhomogeneousPoint2D(centerX, centerY);

        final var pointAtLeftSide = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5), centerY);
        final var pointAtTopLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopSide = new InhomogeneousPoint2D(
                centerX, centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtRightSide = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5), centerY);
        final var pointAtBottomRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomSide = new InhomogeneousPoint2D(
                centerX, centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var insidePoint = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(-0.5, 0.5),
                centerY + height * randomizer.nextDouble(-0.5, 0.5));

        // test static method with point and corner coordinates and threshold
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), left, top,
                right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), left, top,
                right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), left,
                top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), left,
                top, right, bottom, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isAtBottomLeftCorner(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomLeftCorner(insidePoint.getInhomX(), insidePoint.getInhomY(), left, top, right,
                bottom, ABSOLUTE_ERROR));

        // test static method with point and corner coordinates
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), left, top,
                right, bottom));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                left, top, right, bottom));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), left, top,
                right, bottom));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                left, top, right, bottom));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), left,
                top, right, bottom));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), left, top, right, bottom));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), left,
                top, right, bottom));
        assertTrue(Rectangle.isAtBottomLeftCorner(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), left, top, right, bottom));
        assertFalse(Rectangle.isAtBottomLeftCorner(insidePoint.getInhomX(), insidePoint.getInhomY(), left, top, right,
                bottom));

        // test static method with point, corner coordinate and threshold
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtLeftSide, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtTopLeftCorner, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtTopSide, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtTopRightCorner, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtRightSide, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtBottomRightCorner, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtBottomSide, left, top, right, bottom, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isAtBottomLeftCorner(pointAtBottomLeftCorner, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomLeftCorner(insidePoint, left, top, right, bottom, ABSOLUTE_ERROR));

        // test static method with point, corner coordinate
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtLeftSide, left, top, right, bottom));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtTopLeftCorner, left, top, right, bottom));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtTopSide, left, top, right, bottom));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtTopRightCorner, left, top, right, bottom));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtRightSide, left, top, right, bottom));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtBottomRightCorner, left, top, right, bottom));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtBottomSide, left, top, right, bottom));
        assertTrue(Rectangle.isAtBottomLeftCorner(pointAtBottomLeftCorner, left, top, right, bottom));
        assertFalse(Rectangle.isAtBottomLeftCorner(insidePoint, left, top, right, bottom));

        // test static method with point coordinates, corners and threshold
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), topLeft,
                bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), topLeft,
                bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), topLeft,
                bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(),
                topLeft, bottomRight, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isAtBottomLeftCorner(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomLeftCorner(insidePoint.getInhomX(), insidePoint.getInhomY(), topLeft,
                bottomRight, ABSOLUTE_ERROR));

        // test static method with point coordinates and corners
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), topLeft,
                bottomRight));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                topLeft, bottomRight));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), topLeft,
                bottomRight));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                topLeft, bottomRight));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), topLeft,
                bottomRight));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), topLeft, bottomRight));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(),
                topLeft, bottomRight));
        assertTrue(Rectangle.isAtBottomLeftCorner(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), topLeft, bottomRight));
        assertFalse(Rectangle.isAtBottomLeftCorner(insidePoint.getInhomX(), insidePoint.getInhomY(), topLeft,
                bottomRight));

        // test static method with point coordinates, corners and threshold
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtLeftSide, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtTopLeftCorner, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtTopSide, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtTopRightCorner, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtRightSide, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtBottomRightCorner, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtBottomSide, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isAtBottomLeftCorner(pointAtBottomLeftCorner, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomLeftCorner(insidePoint, topLeft, bottomRight, ABSOLUTE_ERROR));

        // test static method with point coordinates and corners
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtLeftSide, topLeft, bottomRight));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtTopLeftCorner, topLeft, bottomRight));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtTopSide, topLeft, bottomRight));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtTopRightCorner, topLeft, bottomRight));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtRightSide, topLeft, bottomRight));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtBottomRightCorner, topLeft, bottomRight));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtBottomSide, topLeft, bottomRight));
        assertTrue(Rectangle.isAtBottomLeftCorner(pointAtBottomLeftCorner, topLeft, bottomRight));
        assertFalse(Rectangle.isAtBottomLeftCorner(insidePoint, topLeft, bottomRight));

        // test static method with point coordinates, center, sizes and threshold
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), center,
                width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), center,
                width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), center,
                width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), center,
                width, height, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isAtBottomLeftCorner(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomLeftCorner(insidePoint.getInhomX(), insidePoint.getInhomY(), center, width,
                height, ABSOLUTE_ERROR));

        // test static method with point coordinates, center and sizes
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), center,
                width, height));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                center, width, height));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), center,
                width, height));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                center, width, height));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), center,
                width, height));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), center, width, height));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), center,
                width, height));
        assertTrue(Rectangle.isAtBottomLeftCorner(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), center, width, height));
        assertFalse(Rectangle.isAtBottomLeftCorner(insidePoint.getInhomX(), insidePoint.getInhomY(), center, width,
                height));

        // test static method with point, center, sizes and threshold
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtLeftSide, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtTopLeftCorner, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtTopSide, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtTopRightCorner, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtRightSide, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtBottomRightCorner, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtBottomSide, center, width, height, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isAtBottomLeftCorner(pointAtBottomLeftCorner, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isAtBottomLeftCorner(insidePoint, center, width, height, ABSOLUTE_ERROR));

        // test static method with point, center and sizes
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtLeftSide, center, width, height));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtTopLeftCorner, center, width, height));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtTopSide, center, width, height));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtTopRightCorner, center, width, height));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtRightSide, center, width, height));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtBottomRightCorner, center, width, height));
        assertFalse(Rectangle.isAtBottomLeftCorner(pointAtBottomSide, center, width, height));
        assertTrue(Rectangle.isAtBottomLeftCorner(pointAtBottomLeftCorner, center, width, height));
        assertFalse(Rectangle.isAtBottomLeftCorner(insidePoint, center, width, height));

        // test non-static method with point coordinates and threshold
        final var r = new Rectangle(left, top, right, bottom);

        assertFalse(r.isAtBottomLeftCorner(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isAtBottomLeftCorner(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                ABSOLUTE_ERROR));
        assertFalse(r.isAtBottomLeftCorner(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isAtBottomLeftCorner(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                ABSOLUTE_ERROR));
        assertFalse(r.isAtBottomLeftCorner(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isAtBottomLeftCorner(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY(),
                ABSOLUTE_ERROR));
        assertFalse(r.isAtBottomLeftCorner(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(),
                ABSOLUTE_ERROR));
        assertTrue(r.isAtBottomLeftCorner(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(),
                ABSOLUTE_ERROR));
        assertFalse(r.isAtBottomLeftCorner(insidePoint.getInhomX(), insidePoint.getInhomY(), ABSOLUTE_ERROR));

        // test non-static method with point coordinates
        assertFalse(r.isAtBottomLeftCorner(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY()));
        assertFalse(r.isAtBottomLeftCorner(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY()));
        assertFalse(r.isAtBottomLeftCorner(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY()));
        assertFalse(r.isAtBottomLeftCorner(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY()));
        assertFalse(r.isAtBottomLeftCorner(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY()));
        assertFalse(r.isAtBottomLeftCorner(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY()));
        assertFalse(r.isAtBottomLeftCorner(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY()));
        assertTrue(r.isAtBottomLeftCorner(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY()));
        assertFalse(r.isAtBottomLeftCorner(insidePoint.getInhomX(), insidePoint.getInhomY()));

        // test non-static method with point and threshold
        assertFalse(r.isAtBottomLeftCorner(pointAtLeftSide, ABSOLUTE_ERROR));
        assertFalse(r.isAtBottomLeftCorner(pointAtTopLeftCorner, ABSOLUTE_ERROR));
        assertFalse(r.isAtBottomLeftCorner(pointAtTopSide, ABSOLUTE_ERROR));
        assertFalse(r.isAtBottomLeftCorner(pointAtTopRightCorner, ABSOLUTE_ERROR));
        assertFalse(r.isAtBottomLeftCorner(pointAtRightSide, ABSOLUTE_ERROR));
        assertFalse(r.isAtBottomLeftCorner(pointAtBottomRightCorner, ABSOLUTE_ERROR));
        assertFalse(r.isAtBottomLeftCorner(pointAtBottomSide, ABSOLUTE_ERROR));
        assertTrue(r.isAtBottomLeftCorner(pointAtBottomLeftCorner, ABSOLUTE_ERROR));
        assertFalse(r.isAtBottomLeftCorner(insidePoint, ABSOLUTE_ERROR));

        // test non-static method with point
        assertFalse(r.isAtBottomLeftCorner(pointAtLeftSide));
        assertFalse(r.isAtBottomLeftCorner(pointAtTopLeftCorner));
        assertFalse(r.isAtBottomLeftCorner(pointAtTopSide));
        assertFalse(r.isAtBottomLeftCorner(pointAtTopRightCorner));
        assertFalse(r.isAtBottomLeftCorner(pointAtRightSide));
        assertFalse(r.isAtBottomLeftCorner(pointAtBottomRightCorner));
        assertFalse(r.isAtBottomLeftCorner(pointAtBottomSide));
        assertTrue(r.isAtBottomLeftCorner(pointAtBottomLeftCorner));
        assertFalse(r.isAtBottomLeftCorner(insidePoint));
    }

    @Test
    void testGetSignedDistanceToLeftSide() {
        final var randomizer = new UniformRandomizer();
        final var centerX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var centerY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var width = randomizer.nextDouble(0, MAX_RANDOM_VALUE);
        final var height = randomizer.nextDouble(0, MAX_RANDOM_VALUE);

        final var left = centerX - 0.5 * width;
        final var right = centerX + 0.5 * width;
        final var top = centerY + 0.5 * height;
        final var bottom = centerY - 0.5 * height;

        final var topLeft = new InhomogeneousPoint2D(left, top);
        final var bottomLeft = new InhomogeneousPoint2D(left, bottom);
        final var bottomRight = new InhomogeneousPoint2D(right, bottom);
        final var center = new InhomogeneousPoint2D(centerX, centerY);

        final var pointAtLeftSide = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5), centerY);
        final var pointAtTopLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopSide = new InhomogeneousPoint2D(
                centerX, centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtRightSide = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5), centerY);
        final var pointAtBottomRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomSide = new InhomogeneousPoint2D(
                centerX, centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var insidePoint = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(-0.5, 0.5),
                centerY + height * randomizer.nextDouble(-0.5, 0.5));

        // test static method with point and corner coordinates
        assertEquals(Rectangle.getSignedDistanceToLeftSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(),
                left, top, right, bottom), left - pointAtLeftSide.getInhomX(), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToLeftSide(pointAtTopLeftCorner.getInhomX(),
                pointAtTopLeftCorner.getInhomY(), left, top, right, bottom), pointAtTopLeftCorner.distanceTo(topLeft),
                ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToLeftSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), left,
                top, right, bottom), pointAtTopSide.distanceTo(topLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToLeftSide(pointAtTopRightCorner.getInhomX(),
                pointAtTopRightCorner.getInhomY(), left, top, right, bottom), pointAtTopRightCorner.distanceTo(topLeft),
                ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToLeftSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(),
                left, top, right, bottom), pointAtRightSide.getInhomX() - left, ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToLeftSide(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), left, top, right, bottom),
                pointAtBottomRightCorner.distanceTo(bottomLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToLeftSide(pointAtBottomSide.getInhomX(),
                pointAtBottomSide.getInhomY(), left, top, right, bottom), pointAtBottomSide.distanceTo(bottomLeft),
                ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToLeftSide(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), left, top, right, bottom),
                pointAtBottomLeftCorner.distanceTo(bottomLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToLeftSide(insidePoint.getInhomX(), insidePoint.getInhomY(), left, top,
                right, bottom), left - insidePoint.getInhomX(), ABSOLUTE_ERROR);

        // test static method with point, corner coordinates
        assertEquals(Rectangle.getSignedDistanceToLeftSide(pointAtLeftSide, left, top, right, bottom),
                left - pointAtLeftSide.getInhomX(), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToLeftSide(pointAtTopLeftCorner, left, top, right, bottom),
                pointAtTopLeftCorner.distanceTo(topLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToLeftSide(pointAtTopSide, left, top, right, bottom),
                pointAtTopSide.distanceTo(topLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToLeftSide(pointAtTopRightCorner, left, top, right, bottom),
                pointAtTopRightCorner.distanceTo(topLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToLeftSide(pointAtRightSide, left, top, right, bottom),
                pointAtRightSide.getInhomX() - left, ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToLeftSide(pointAtBottomRightCorner, left, top, right, bottom),
                pointAtBottomRightCorner.distanceTo(bottomLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToLeftSide(pointAtBottomSide, left, top, right, bottom),
                pointAtBottomSide.distanceTo(bottomLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToLeftSide(pointAtBottomLeftCorner, left, top, right, bottom),
                pointAtBottomLeftCorner.distanceTo(bottomLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToLeftSide(insidePoint, left, top, right, bottom),
                left - insidePoint.getInhomX(), ABSOLUTE_ERROR);

        // test static method with point coordinates and corners
        assertEquals(Rectangle.getSignedDistanceToLeftSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(),
                topLeft, bottomRight), left - pointAtLeftSide.getInhomX(), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToLeftSide(pointAtTopLeftCorner.getInhomX(),
                pointAtTopLeftCorner.getInhomY(), topLeft, bottomRight), pointAtTopLeftCorner.distanceTo(topLeft),
                ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToLeftSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(),
                topLeft, bottomRight), pointAtTopSide.distanceTo(topLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToLeftSide(pointAtTopRightCorner.getInhomX(),
                pointAtTopRightCorner.getInhomY(), topLeft, bottomRight), pointAtTopRightCorner.distanceTo(topLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToLeftSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(),
                topLeft, bottomRight), pointAtRightSide.getInhomX() - left, ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToLeftSide(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), topLeft, bottomRight),
                pointAtBottomRightCorner.distanceTo(bottomLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToLeftSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(),
                topLeft, bottomRight), pointAtBottomSide.distanceTo(bottomLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToLeftSide(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), topLeft, bottomRight),
                pointAtBottomLeftCorner.distanceTo(bottomLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToLeftSide(insidePoint.getInhomX(), insidePoint.getInhomY(), topLeft,
                bottomRight), left - insidePoint.getInhomX(), ABSOLUTE_ERROR);

        // test static method with point and corners
        assertEquals(Rectangle.getSignedDistanceToLeftSide(pointAtLeftSide, topLeft, bottomRight),
                left - pointAtLeftSide.getInhomX(), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToLeftSide(pointAtTopLeftCorner, topLeft, bottomRight),
                pointAtTopLeftCorner.distanceTo(topLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToLeftSide(pointAtTopSide, topLeft, bottomRight),
                pointAtTopSide.distanceTo(topLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToLeftSide(pointAtTopRightCorner, topLeft, bottomRight),
                pointAtTopRightCorner.distanceTo(topLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToLeftSide(pointAtRightSide, topLeft, bottomRight),
                pointAtRightSide.getInhomX() - left, ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToLeftSide(pointAtBottomRightCorner, topLeft, bottomRight),
                pointAtBottomRightCorner.distanceTo(bottomLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToLeftSide(pointAtBottomSide, topLeft, bottomRight),
                pointAtBottomSide.distanceTo(bottomLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToLeftSide(pointAtBottomLeftCorner, topLeft, bottomRight),
                pointAtBottomLeftCorner.distanceTo(bottomLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToLeftSide(insidePoint, topLeft, bottomRight),
                left - insidePoint.getInhomX(), ABSOLUTE_ERROR);

        // test static method with point coordinates, center and size
        assertEquals(Rectangle.getSignedDistanceToLeftSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(),
                center, width, height), left - pointAtLeftSide.getInhomX(), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToLeftSide(pointAtTopLeftCorner.getInhomX(),
                pointAtTopLeftCorner.getInhomY(), center, width, height), pointAtTopLeftCorner.distanceTo(topLeft),
                ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToLeftSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(),
                center, width, height), pointAtTopSide.distanceTo(topLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToLeftSide(pointAtTopRightCorner.getInhomX(),
                pointAtTopRightCorner.getInhomY(), center, width, height), pointAtTopRightCorner.distanceTo(topLeft),
                ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToLeftSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(),
                center, width, height), pointAtRightSide.getInhomX() - left, ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToLeftSide(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), center, width, height),
                pointAtBottomRightCorner.distanceTo(bottomLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToLeftSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(),
                center, width, height), pointAtBottomSide.distanceTo(bottomLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToLeftSide(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), center, width, height),
                pointAtBottomLeftCorner.distanceTo(bottomLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToLeftSide(insidePoint.getInhomX(), insidePoint.getInhomY(), center,
                width, height), left - insidePoint.getInhomX(), ABSOLUTE_ERROR);

        // test static method with point, center and size
        assertEquals(Rectangle.getSignedDistanceToLeftSide(pointAtLeftSide, center, width, height),
                left - pointAtLeftSide.getInhomX(), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToLeftSide(pointAtTopLeftCorner, center, width, height),
                pointAtTopLeftCorner.distanceTo(topLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToLeftSide(pointAtTopSide, center, width, height),
                pointAtTopSide.distanceTo(topLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToLeftSide(pointAtTopRightCorner, center, width, height),
                pointAtTopRightCorner.distanceTo(topLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToLeftSide(pointAtRightSide, center, width, height),
                pointAtRightSide.getInhomX() - left, ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToLeftSide(pointAtBottomRightCorner, center, width, height),
                pointAtBottomRightCorner.distanceTo(bottomLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToLeftSide(pointAtBottomSide, center, width, height),
                pointAtBottomSide.distanceTo(bottomLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToLeftSide(pointAtBottomLeftCorner, center, width, height),
                pointAtBottomLeftCorner.distanceTo(bottomLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToLeftSide(insidePoint, center, width, height),
                left - insidePoint.getInhomX(), ABSOLUTE_ERROR);

        // test non-static method with point coordinates
        var r = new Rectangle(left, top, right, bottom);

        assertEquals(r.getSignedDistanceToLeftSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY()),
                left - pointAtLeftSide.getInhomX(), ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToLeftSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY()),
                pointAtTopLeftCorner.distanceTo(topLeft), ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToLeftSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY()),
                pointAtTopSide.distanceTo(topLeft), ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToLeftSide(pointAtTopRightCorner.getInhomX(),
                pointAtTopRightCorner.getInhomY()), pointAtTopRightCorner.distanceTo(topLeft), ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToLeftSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY()),
                pointAtRightSide.getInhomX() - left, ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToLeftSide(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY()), pointAtBottomRightCorner.distanceTo(bottomLeft), ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToLeftSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY()),
                pointAtBottomSide.distanceTo(bottomLeft), ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToLeftSide(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY()), pointAtBottomLeftCorner.distanceTo(bottomLeft), ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToLeftSide(insidePoint.getInhomX(), insidePoint.getInhomY()),
                left - insidePoint.getInhomX(), ABSOLUTE_ERROR);

        // test non-static method with point
        r = new Rectangle(left, top, right, bottom);

        assertEquals(r.getSignedDistanceToLeftSide(pointAtLeftSide), left - pointAtLeftSide.getInhomX(),
                ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToLeftSide(pointAtTopLeftCorner), pointAtTopLeftCorner.distanceTo(topLeft),
                ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToLeftSide(pointAtTopSide), pointAtTopSide.distanceTo(topLeft), ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToLeftSide(pointAtTopRightCorner), pointAtTopRightCorner.distanceTo(topLeft),
                ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToLeftSide(pointAtRightSide), pointAtRightSide.getInhomX() - left,
                ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToLeftSide(pointAtBottomRightCorner),
                pointAtBottomRightCorner.distanceTo(bottomLeft), ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToLeftSide(pointAtBottomSide), pointAtBottomSide.distanceTo(bottomLeft),
                ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToLeftSide(pointAtBottomLeftCorner),
                pointAtBottomLeftCorner.distanceTo(bottomLeft), ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToLeftSide(insidePoint), left - insidePoint.getInhomX(), ABSOLUTE_ERROR);
    }

    @Test
    void testGetSignedDistanceToTopSide() {
        final var randomizer = new UniformRandomizer();
        final var centerX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var centerY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var width = randomizer.nextDouble(0, MAX_RANDOM_VALUE);
        final var height = randomizer.nextDouble(0, MAX_RANDOM_VALUE);

        final var left = centerX - 0.5 * width;
        final var right = centerX + 0.5 * width;
        final var top = centerY + 0.5 * height;
        final var bottom = centerY - 0.5 * height;

        final var topLeft = new InhomogeneousPoint2D(left, top);
        final var topRight = new InhomogeneousPoint2D(right, top);
        final var bottomRight = new InhomogeneousPoint2D(right, bottom);
        final var center = new InhomogeneousPoint2D(centerX, centerY);

        final var pointAtLeftSide = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5), centerY);
        final var pointAtTopLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopSide = new InhomogeneousPoint2D(
                centerX, centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtRightSide = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5), centerY);
        final var pointAtBottomRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomSide = new InhomogeneousPoint2D(
                centerX, centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var insidePoint = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(-0.5, 0.5),
                centerY + height * randomizer.nextDouble(-0.5, 0.5));

        // test static method with point and corner coordinates
        assertEquals(Rectangle.getSignedDistanceToTopSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(),
                left, top, right, bottom), pointAtLeftSide.distanceTo(topLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToTopSide(pointAtTopLeftCorner.getInhomX(),
                pointAtTopLeftCorner.getInhomY(), left, top, right, bottom), pointAtTopLeftCorner.distanceTo(topLeft),
                ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToTopSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(),
                left, top, right, bottom), pointAtTopSide.getInhomY() - top, ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToTopSide(pointAtTopRightCorner.getInhomX(),
                pointAtTopRightCorner.getInhomY(), left, top, right, bottom),
                pointAtTopRightCorner.distanceTo(topRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToTopSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(),
                left, top, right, bottom), pointAtRightSide.distanceTo(topRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToTopSide(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), left, top, right, bottom),
                pointAtBottomRightCorner.distanceTo(topRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToTopSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(),
                left, top, right, bottom), top - pointAtBottomSide.getInhomY(), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToTopSide(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), left, top, right, bottom),
                pointAtBottomLeftCorner.distanceTo(topLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToTopSide(insidePoint.getInhomX(), insidePoint.getInhomY(), left, top,
                right, bottom), insidePoint.getInhomY() - top, ABSOLUTE_ERROR);

        // test static method with point, corner coordinates
        assertEquals(Rectangle.getSignedDistanceToTopSide(pointAtLeftSide, left, top, right, bottom),
                pointAtLeftSide.distanceTo(topLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToTopSide(pointAtTopLeftCorner, left, top, right, bottom),
                pointAtTopLeftCorner.distanceTo(topLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToTopSide(pointAtTopSide, left, top, right, bottom),
                pointAtTopSide.getInhomY() - top, ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToTopSide(pointAtTopRightCorner, left, top, right, bottom),
                pointAtTopRightCorner.distanceTo(topRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToTopSide(pointAtRightSide, left, top, right, bottom),
                pointAtRightSide.distanceTo(topRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToTopSide(pointAtBottomRightCorner, left, top, right, bottom),
                pointAtBottomRightCorner.distanceTo(topRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToTopSide(pointAtBottomSide, left, top, right, bottom),
                top - pointAtBottomSide.getInhomY(), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToTopSide(pointAtBottomLeftCorner, left, top, right, bottom),
                pointAtBottomLeftCorner.distanceTo(topLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToTopSide(insidePoint, left, top, right, bottom),
                insidePoint.getInhomY() - top, ABSOLUTE_ERROR);

        // test static method with point coordinates and corners
        assertEquals(Rectangle.getSignedDistanceToTopSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(),
                topLeft, bottomRight), pointAtLeftSide.distanceTo(topLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToTopSide(pointAtTopLeftCorner.getInhomX(),
                pointAtTopLeftCorner.getInhomY(), topLeft, bottomRight), pointAtTopLeftCorner.distanceTo(topLeft),
                ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToTopSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(),
                topLeft, bottomRight), pointAtTopSide.getInhomY() - top, ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToTopSide(pointAtTopRightCorner.getInhomX(),
                pointAtTopRightCorner.getInhomY(), topLeft, bottomRight), pointAtTopRightCorner.distanceTo(topRight),
                ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToTopSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(),
                topLeft, bottomRight), pointAtRightSide.distanceTo(topRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToTopSide(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), topLeft, bottomRight),
                pointAtBottomRightCorner.distanceTo(topRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToTopSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(),
                topLeft, bottomRight), top - pointAtBottomSide.getInhomY(), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToTopSide(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), topLeft, bottomRight), pointAtBottomLeftCorner.distanceTo(topLeft),
                ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToTopSide(insidePoint.getInhomX(), insidePoint.getInhomY(), topLeft,
                bottomRight), insidePoint.getInhomY() - top, ABSOLUTE_ERROR);

        // test static method with point and corners
        assertEquals(Rectangle.getSignedDistanceToTopSide(pointAtLeftSide, topLeft, bottomRight),
                pointAtLeftSide.distanceTo(topLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToTopSide(pointAtTopLeftCorner, topLeft, bottomRight),
                pointAtTopLeftCorner.distanceTo(topLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToTopSide(pointAtTopSide, topLeft, bottomRight),
                pointAtTopSide.getInhomY() - top, ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToTopSide(pointAtTopRightCorner, topLeft, bottomRight),
                pointAtTopRightCorner.distanceTo(topRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToTopSide(pointAtRightSide, topLeft, bottomRight),
                pointAtRightSide.distanceTo(topRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToTopSide(pointAtBottomRightCorner, topLeft, bottomRight),
                pointAtBottomRightCorner.distanceTo(topRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToTopSide(pointAtBottomSide, topLeft, bottomRight),
                top - pointAtBottomSide.getInhomY(), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToTopSide(pointAtBottomLeftCorner, topLeft, bottomRight),
                pointAtBottomLeftCorner.distanceTo(topLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToTopSide(insidePoint, topLeft, bottomRight),
                insidePoint.getInhomY() - top, ABSOLUTE_ERROR);

        // test static method with point coordinates, center and sizes
        assertEquals(Rectangle.getSignedDistanceToTopSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(),
                center, width, height), pointAtLeftSide.distanceTo(topLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToTopSide(pointAtTopLeftCorner.getInhomX(),
                pointAtTopLeftCorner.getInhomY(), center, width, height), pointAtTopLeftCorner.distanceTo(topLeft),
                ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToTopSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(),
                center, width, height), pointAtTopSide.getInhomY() - top, ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToTopSide(pointAtTopRightCorner.getInhomX(),
                pointAtTopRightCorner.getInhomY(), center, width, height), pointAtTopRightCorner.distanceTo(topRight),
                ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToTopSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(),
                center, width, height), pointAtRightSide.distanceTo(topRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToTopSide(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), center, width, height),
                pointAtBottomRightCorner.distanceTo(topRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToTopSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(),
                center, width, height), top - pointAtBottomSide.getInhomY(), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToTopSide(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), center, width, height),
                pointAtBottomLeftCorner.distanceTo(topLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToTopSide(insidePoint.getInhomX(), insidePoint.getInhomY(), center,
                width, height), insidePoint.getInhomY() - top, ABSOLUTE_ERROR);

        // test static method with point, center and sizes
        assertEquals(Rectangle.getSignedDistanceToTopSide(pointAtLeftSide, center, width, height),
                pointAtLeftSide.distanceTo(topLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToTopSide(pointAtTopLeftCorner, center, width, height),
                pointAtTopLeftCorner.distanceTo(topLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToTopSide(pointAtTopSide, center, width, height),
                pointAtTopSide.getInhomY() - top, ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToTopSide(pointAtTopRightCorner, center, width, height),
                pointAtTopRightCorner.distanceTo(topRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToTopSide(pointAtRightSide, center, width, height),
                pointAtRightSide.distanceTo(topRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToTopSide(pointAtBottomRightCorner, center, width, height),
                pointAtBottomRightCorner.distanceTo(topRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToTopSide(pointAtBottomSide, center, width, height),
                top - pointAtBottomSide.getInhomY(), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToTopSide(pointAtBottomLeftCorner, center, width, height),
                pointAtBottomLeftCorner.distanceTo(topLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToTopSide(insidePoint, center, width, height),
                insidePoint.getInhomY() - top, ABSOLUTE_ERROR);

        // test non-static method with point coordinates
        var r = new Rectangle(left, top, right, bottom);

        assertEquals(r.getSignedDistanceToTopSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY()),
                pointAtLeftSide.distanceTo(topLeft), ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToTopSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY()),
                pointAtTopLeftCorner.distanceTo(topLeft), ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToTopSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY()),
                pointAtTopSide.getInhomY() - top, ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToTopSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY()),
                pointAtTopRightCorner.distanceTo(topRight), ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToTopSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY()),
                pointAtRightSide.distanceTo(topRight), ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToTopSide(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY()), pointAtBottomRightCorner.distanceTo(topRight), ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToTopSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY()),
                top - pointAtBottomSide.getInhomY(), ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToTopSide(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY()), pointAtBottomLeftCorner.distanceTo(topLeft), ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToTopSide(insidePoint.getInhomX(), insidePoint.getInhomY()),
                insidePoint.getInhomY() - top, ABSOLUTE_ERROR);

        // test non-static method with point coordinates
        r = new Rectangle(left, top, right, bottom);

        assertEquals(r.getSignedDistanceToTopSide(pointAtLeftSide), pointAtLeftSide.distanceTo(topLeft),
                ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToTopSide(pointAtTopLeftCorner), pointAtTopLeftCorner.distanceTo(topLeft),
                ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToTopSide(pointAtTopSide), pointAtTopSide.getInhomY() - top,
                ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToTopSide(pointAtTopRightCorner), pointAtTopRightCorner.distanceTo(topRight),
                ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToTopSide(pointAtRightSide), pointAtRightSide.distanceTo(topRight),
                ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToTopSide(pointAtBottomRightCorner),
                pointAtBottomRightCorner.distanceTo(topRight), ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToTopSide(pointAtBottomSide), top - pointAtBottomSide.getInhomY(),
                ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToTopSide(pointAtBottomLeftCorner), pointAtBottomLeftCorner.distanceTo(topLeft),
                ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToTopSide(insidePoint), insidePoint.getInhomY() - top, ABSOLUTE_ERROR);
    }

    @Test
    void testGetSignedDistanceToRightSide() {
        final var randomizer = new UniformRandomizer();
        final var centerX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var centerY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var width = randomizer.nextDouble(0, MAX_RANDOM_VALUE);
        final var height = randomizer.nextDouble(0, MAX_RANDOM_VALUE);

        final var left = centerX - 0.5 * width;
        final var right = centerX + 0.5 * width;
        final var top = centerY + 0.5 * height;
        final var bottom = centerY - 0.5 * height;

        final var topLeft = new InhomogeneousPoint2D(left, top);
        final var topRight = new InhomogeneousPoint2D(right, top);
        final var bottomRight = new InhomogeneousPoint2D(right, bottom);
        final var center = new InhomogeneousPoint2D(centerX, centerY);

        final var pointAtLeftSide = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5), centerY);
        final var pointAtTopLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopSide = new InhomogeneousPoint2D(
                centerX, centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtRightSide = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5), centerY);
        final var pointAtBottomRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomSide = new InhomogeneousPoint2D(
                centerX, centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var insidePoint = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(-0.5, 0.5),
                centerY + height * randomizer.nextDouble(-0.5, 0.5));

        // test static method with point and corner coordinates
        assertEquals(Rectangle.getSignedDistanceToRightSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(),
                left, top, right, bottom), right - pointAtLeftSide.getInhomX(), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToRightSide(pointAtTopLeftCorner.getInhomX(),
                pointAtTopLeftCorner.getInhomY(), left, top, right, bottom), pointAtTopLeftCorner.distanceTo(topRight),
                ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToRightSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(),
                left, top, right, bottom), pointAtTopSide.distanceTo(topRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToRightSide(pointAtTopRightCorner.getInhomX(),
                pointAtTopRightCorner.getInhomY(), left, top, right, bottom),
                pointAtTopRightCorner.distanceTo(topRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToRightSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(),
                left, top, right, bottom), pointAtRightSide.getInhomX() - right, ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToRightSide(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), left, top, right, bottom),
                pointAtBottomRightCorner.distanceTo(bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToRightSide(pointAtBottomSide.getInhomX(),
                pointAtBottomSide.getInhomY(), left, top, right, bottom), pointAtBottomSide.distanceTo(bottomRight),
                ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToRightSide(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), left, top, right, bottom),
                pointAtBottomLeftCorner.distanceTo(bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToRightSide(insidePoint.getInhomX(), insidePoint.getInhomY(), left, top,
                right, bottom), insidePoint.getInhomX() - right, ABSOLUTE_ERROR);

        // test static method with point, corner coordinates
        assertEquals(Rectangle.getSignedDistanceToRightSide(pointAtLeftSide, left, top, right, bottom),
                right - pointAtLeftSide.getInhomX(), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToRightSide(pointAtTopLeftCorner, left, top, right, bottom),
                pointAtTopLeftCorner.distanceTo(topRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToRightSide(pointAtTopSide, left, top, right, bottom),
                pointAtTopSide.distanceTo(topRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToRightSide(pointAtTopRightCorner, left, top, right, bottom),
                pointAtTopRightCorner.distanceTo(topRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToRightSide(pointAtRightSide, left, top, right, bottom),
                pointAtRightSide.getInhomX() - right, ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToRightSide(pointAtBottomRightCorner, left, top, right, bottom),
                pointAtBottomRightCorner.distanceTo(bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToRightSide(pointAtBottomSide, left, top, right, bottom),
                pointAtBottomSide.distanceTo(bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToRightSide(pointAtBottomLeftCorner, left, top, right, bottom),
                pointAtBottomLeftCorner.distanceTo(bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToRightSide(insidePoint, left, top, right, bottom),
                insidePoint.getInhomX() - right, ABSOLUTE_ERROR);

        // test static method with point coordinates and corners
        assertEquals(Rectangle.getSignedDistanceToRightSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(),
                topLeft, bottomRight), right - pointAtLeftSide.getInhomX(), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToRightSide(pointAtTopLeftCorner.getInhomX(),
                pointAtTopLeftCorner.getInhomY(), topLeft, bottomRight), pointAtTopLeftCorner.distanceTo(topRight),
                ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToRightSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(),
                topLeft, bottomRight), pointAtTopSide.distanceTo(topRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToRightSide(pointAtTopRightCorner.getInhomX(),
                pointAtTopRightCorner.getInhomY(), topLeft, bottomRight), pointAtTopRightCorner.distanceTo(topRight),
                ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToRightSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(),
                topLeft, bottomRight), pointAtRightSide.getInhomX() - right, ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToRightSide(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), topLeft, bottomRight),
                pointAtBottomRightCorner.distanceTo(bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToRightSide(pointAtBottomSide.getInhomX(),
                pointAtBottomSide.getInhomY(), topLeft, bottomRight), pointAtBottomSide.distanceTo(bottomRight),
                ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToRightSide(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), topLeft, bottomRight),
                pointAtBottomLeftCorner.distanceTo(bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToRightSide(insidePoint.getInhomX(), insidePoint.getInhomY(), topLeft,
                bottomRight), insidePoint.getInhomX() - right, ABSOLUTE_ERROR);

        // test static method with point and corners
        assertEquals(Rectangle.getSignedDistanceToRightSide(pointAtLeftSide, topLeft, bottomRight),
                right - pointAtLeftSide.getInhomX(), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToRightSide(pointAtTopLeftCorner, topLeft, bottomRight),
                pointAtTopLeftCorner.distanceTo(topRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToRightSide(pointAtTopSide, topLeft, bottomRight),
                pointAtTopSide.distanceTo(topRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToRightSide(pointAtTopRightCorner, topLeft, bottomRight),
                pointAtTopRightCorner.distanceTo(topRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToRightSide(pointAtRightSide, topLeft, bottomRight),
                pointAtRightSide.getInhomX() - right, ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToRightSide(pointAtBottomRightCorner, topLeft, bottomRight),
                pointAtBottomRightCorner.distanceTo(bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToRightSide(pointAtBottomSide, topLeft, bottomRight),
                pointAtBottomSide.distanceTo(bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToRightSide(pointAtBottomLeftCorner, topLeft, bottomRight),
                pointAtBottomLeftCorner.distanceTo(bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToRightSide(insidePoint, topLeft, bottomRight),
                insidePoint.getInhomX() - right, ABSOLUTE_ERROR);

        // test static method with point coordinates, center and size
        assertEquals(Rectangle.getSignedDistanceToRightSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(),
                center, width, height), right - pointAtLeftSide.getInhomX(), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToRightSide(pointAtTopLeftCorner.getInhomX(),
                pointAtTopLeftCorner.getInhomY(), center, width, height), pointAtTopLeftCorner.distanceTo(topRight),
                ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToRightSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(),
                center, width, height), pointAtTopSide.distanceTo(topRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToRightSide(pointAtTopRightCorner.getInhomX(),
                pointAtTopRightCorner.getInhomY(), center, width, height), pointAtTopRightCorner.distanceTo(topRight),
                ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToRightSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(),
                center, width, height), pointAtRightSide.getInhomX() - right, ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToRightSide(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), center, width, height),
                pointAtBottomRightCorner.distanceTo(bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToRightSide(pointAtBottomSide.getInhomX(),
                pointAtBottomSide.getInhomY(), center, width, height), pointAtBottomSide.distanceTo(bottomRight),
                ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToRightSide(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), center, width, height),
                pointAtBottomLeftCorner.distanceTo(bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToRightSide(insidePoint.getInhomX(), insidePoint.getInhomY(), center,
                width, height), insidePoint.getInhomX() - right, ABSOLUTE_ERROR);

        // test static method with point, center and size
        assertEquals(Rectangle.getSignedDistanceToRightSide(pointAtLeftSide, center, width, height),
                right - pointAtLeftSide.getInhomX(), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToRightSide(pointAtTopLeftCorner, center, width, height),
                pointAtTopLeftCorner.distanceTo(topRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToRightSide(pointAtTopSide, center, width, height),
                pointAtTopSide.distanceTo(topRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToRightSide(pointAtTopRightCorner, center, width, height),
                pointAtTopRightCorner.distanceTo(topRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToRightSide(pointAtRightSide, center, width, height),
                pointAtRightSide.getInhomX() - right, ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToRightSide(pointAtBottomRightCorner, center, width, height),
                pointAtBottomRightCorner.distanceTo(bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToRightSide(pointAtBottomSide, center, width, height),
                pointAtBottomSide.distanceTo(bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToRightSide(pointAtBottomLeftCorner, center, width, height),
                pointAtBottomLeftCorner.distanceTo(bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToRightSide(insidePoint, center, width, height),
                insidePoint.getInhomX() - right, ABSOLUTE_ERROR);

        // test non-static method with point coordinates
        var r = new Rectangle(left, top, right, bottom);

        assertEquals(r.getSignedDistanceToRightSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY()),
                right - pointAtLeftSide.getInhomX(), ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToRightSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY()),
                pointAtTopLeftCorner.distanceTo(topRight), ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToRightSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY()),
                pointAtTopSide.distanceTo(topRight), ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToRightSide(pointAtTopRightCorner.getInhomX(),
                pointAtTopRightCorner.getInhomY()), pointAtTopRightCorner.distanceTo(topRight), ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToRightSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY()),
                pointAtRightSide.getInhomX() - right, ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToRightSide(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY()), pointAtBottomRightCorner.distanceTo(bottomRight),
                ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToRightSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY()),
                pointAtBottomSide.distanceTo(bottomRight), ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToRightSide(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY()), pointAtBottomLeftCorner.distanceTo(bottomRight), ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToRightSide(insidePoint.getInhomX(), insidePoint.getInhomY()),
                insidePoint.getInhomX() - right, ABSOLUTE_ERROR);

        // test non-static method with point
        r = new Rectangle(left, top, right, bottom);

        assertEquals(r.getSignedDistanceToRightSide(pointAtLeftSide), right - pointAtLeftSide.getInhomX(),
                ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToRightSide(pointAtTopLeftCorner), pointAtTopLeftCorner.distanceTo(topRight),
                ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToRightSide(pointAtTopSide), pointAtTopSide.distanceTo(topRight),
                ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToRightSide(pointAtTopRightCorner), pointAtTopRightCorner.distanceTo(topRight),
                ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToRightSide(pointAtRightSide), pointAtRightSide.getInhomX() - right,
                ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToRightSide(pointAtBottomRightCorner),
                pointAtBottomRightCorner.distanceTo(bottomRight), ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToRightSide(pointAtBottomSide), pointAtBottomSide.distanceTo(bottomRight),
                ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToRightSide(pointAtBottomLeftCorner),
                pointAtBottomLeftCorner.distanceTo(bottomRight), ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToRightSide(insidePoint), insidePoint.getInhomX() - right,
                ABSOLUTE_ERROR);
    }

    @Test
    void testGetSignedDistanceToBottomSide() {
        final var randomizer = new UniformRandomizer();
        final var centerX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var centerY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var width = randomizer.nextDouble(0, MAX_RANDOM_VALUE);
        final var height = randomizer.nextDouble(0, MAX_RANDOM_VALUE);

        final var left = centerX - 0.5 * width;
        final var right = centerX + 0.5 * width;
        final var top = centerY + 0.5 * height;
        final var bottom = centerY - 0.5 * height;

        final var topLeft = new InhomogeneousPoint2D(left, top);
        final var bottomLeft = new InhomogeneousPoint2D(left, bottom);
        final var bottomRight = new InhomogeneousPoint2D(right, bottom);
        final var center = new InhomogeneousPoint2D(centerX, centerY);

        final var pointAtLeftSide = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5), centerY);
        final var pointAtTopLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopSide = new InhomogeneousPoint2D(
                centerX, centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtRightSide = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5), centerY);
        final var pointAtBottomRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomSide = new InhomogeneousPoint2D(
                centerX, centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var insidePoint = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(-0.5, 0.5),
                centerY + height * randomizer.nextDouble(-0.5, 0.5));

        // test static method with point and corner coordinates
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(),
                left, top, right, bottom), pointAtLeftSide.distanceTo(bottomLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtTopLeftCorner.getInhomX(),
                pointAtTopLeftCorner.getInhomY(), left, top, right, bottom),
                pointAtTopLeftCorner.distanceTo(bottomLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(),
                left, top, right, bottom), pointAtTopSide.getInhomY() - bottom, ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtTopRightCorner.getInhomX(),
                pointAtTopRightCorner.getInhomY(), left, top, right, bottom),
                pointAtTopRightCorner.distanceTo(bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(),
                left, top, right, bottom), pointAtRightSide.distanceTo(bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), left, top, right, bottom),
                pointAtBottomRightCorner.distanceTo(bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtBottomSide.getInhomX(),
                pointAtBottomSide.getInhomY(), left, top, right, bottom), bottom - pointAtBottomSide.getInhomY(),
                ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), left, top, right, bottom),
                pointAtBottomLeftCorner.distanceTo(bottomLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(insidePoint.getInhomX(), insidePoint.getInhomY(), left,
                top, right, bottom), bottom - insidePoint.getInhomY(), ABSOLUTE_ERROR);

        // test static method with point, corner coordinates
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtLeftSide, left, top, right, bottom),
                pointAtLeftSide.distanceTo(bottomLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtTopLeftCorner, left, top, right, bottom),
                pointAtTopLeftCorner.distanceTo(bottomLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtTopSide, left, top, right, bottom),
                pointAtTopSide.getInhomY() - bottom, ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtTopRightCorner, left, top, right, bottom),
                pointAtTopRightCorner.distanceTo(bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtRightSide, left, top, right, bottom),
                pointAtRightSide.distanceTo(bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtBottomRightCorner, left, top, right, bottom),
                pointAtBottomRightCorner.distanceTo(bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtBottomSide, left, top, right, bottom),
                bottom - pointAtBottomSide.getInhomY(), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtBottomLeftCorner, left, top, right, bottom),
                pointAtBottomLeftCorner.distanceTo(bottomLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(insidePoint, left, top, right, bottom),
                bottom - insidePoint.getInhomY(), ABSOLUTE_ERROR);

        // test static method with point coordinates and corners
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(),
                topLeft, bottomRight), pointAtLeftSide.distanceTo(bottomLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtTopLeftCorner.getInhomX(),
                pointAtTopLeftCorner.getInhomY(), topLeft, bottomRight), pointAtTopLeftCorner.distanceTo(bottomLeft),
                ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(),
                topLeft, bottomRight), pointAtTopSide.getInhomY() - bottom, ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtTopRightCorner.getInhomX(),
                pointAtTopRightCorner.getInhomY(), topLeft, bottomRight), pointAtTopRightCorner.distanceTo(bottomRight),
                ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(),
                topLeft, bottomRight), pointAtRightSide.distanceTo(bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), topLeft, bottomRight),
                pointAtBottomRightCorner.distanceTo(bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtBottomSide.getInhomX(),
                pointAtBottomSide.getInhomY(), topLeft, bottomRight), bottom - pointAtBottomSide.getInhomY(),
                ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), topLeft, bottomRight),
                pointAtBottomLeftCorner.distanceTo(bottomLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(insidePoint.getInhomX(), insidePoint.getInhomY(), topLeft,
                bottomRight), bottom - insidePoint.getInhomY(), ABSOLUTE_ERROR);

        // test static method with point coordinates and corners
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtLeftSide, left, top, right, bottom),
                pointAtLeftSide.distanceTo(bottomLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtTopLeftCorner, left, top, right, bottom),
                pointAtTopLeftCorner.distanceTo(bottomLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtTopSide, left, top, right, bottom),
                pointAtTopSide.getInhomY() - bottom, ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtTopRightCorner, left, top, right, bottom),
                pointAtTopRightCorner.distanceTo(bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtRightSide, left, top, right, bottom),
                pointAtRightSide.distanceTo(bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtBottomRightCorner, left, top, right, bottom),
                pointAtBottomRightCorner.distanceTo(bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtBottomSide, left, top, right, bottom),
                bottom - pointAtBottomSide.getInhomY(), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtBottomLeftCorner, left, top, right, bottom),
                pointAtBottomLeftCorner.distanceTo(bottomLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(insidePoint, left, top, right, bottom),
                bottom - insidePoint.getInhomY(), ABSOLUTE_ERROR);

        // test static method with point and corners
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtLeftSide, topLeft, bottomRight),
                pointAtLeftSide.distanceTo(bottomLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtTopLeftCorner, topLeft, bottomRight),
                pointAtTopLeftCorner.distanceTo(bottomLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtTopSide, topLeft, bottomRight),
                pointAtTopSide.getInhomY() - bottom, ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtTopRightCorner, topLeft, bottomRight),
                pointAtTopRightCorner.distanceTo(bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtRightSide, topLeft, bottomRight),
                pointAtRightSide.distanceTo(bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtBottomRightCorner, topLeft, bottomRight),
                pointAtBottomRightCorner.distanceTo(bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtBottomSide, topLeft, bottomRight),
                bottom - pointAtBottomSide.getInhomY(), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtBottomLeftCorner, topLeft, bottomRight),
                pointAtBottomLeftCorner.distanceTo(bottomLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(insidePoint, topLeft, bottomRight),
                bottom - insidePoint.getInhomY(), ABSOLUTE_ERROR);

        // test static method with point coordinates, center and size
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(),
                center, width, height), pointAtLeftSide.distanceTo(bottomLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtTopLeftCorner.getInhomX(),
                pointAtTopLeftCorner.getInhomY(), center, width, height), pointAtTopLeftCorner.distanceTo(bottomLeft),
                ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(),
                center, width, height), pointAtTopSide.getInhomY() - bottom, ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtTopRightCorner.getInhomX(),
                pointAtTopRightCorner.getInhomY(), center, width, height),
                pointAtTopRightCorner.distanceTo(bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(),
                center, width, height), pointAtRightSide.distanceTo(bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), center, width, height),
                pointAtBottomRightCorner.distanceTo(bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtBottomSide.getInhomX(),
                pointAtBottomSide.getInhomY(), center, width, height), bottom - pointAtBottomSide.getInhomY(),
                ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), center, width, height),
                pointAtBottomLeftCorner.distanceTo(bottomLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(insidePoint.getInhomX(), insidePoint.getInhomY(), center,
                width, height), bottom - insidePoint.getInhomY(), ABSOLUTE_ERROR);

        // test static method with point, center and size
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtLeftSide, center, width, height),
                pointAtLeftSide.distanceTo(bottomLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtTopLeftCorner, center, width, height),
                pointAtTopLeftCorner.distanceTo(bottomLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtTopSide, center, width, height),
                pointAtTopSide.getInhomY() - bottom, ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtTopRightCorner, center, width, height),
                pointAtTopRightCorner.distanceTo(bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtRightSide, center, width, height),
                pointAtRightSide.distanceTo(bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtBottomRightCorner, center, width, height),
                pointAtBottomRightCorner.distanceTo(bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtBottomSide, center, width, height),
                bottom - pointAtBottomSide.getInhomY(), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(pointAtBottomLeftCorner, center, width, height),
                pointAtBottomLeftCorner.distanceTo(bottomLeft), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistanceToBottomSide(insidePoint, center, width, height),
                bottom - insidePoint.getInhomY(), ABSOLUTE_ERROR);

        // test non-static method with point coordinates
        var r = new Rectangle(left, top, right, bottom);

        assertEquals(r.getSignedDistanceToBottomSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY()),
                pointAtLeftSide.distanceTo(bottomLeft), ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToBottomSide(pointAtTopLeftCorner.getInhomX(),
                pointAtTopLeftCorner.getInhomY()), pointAtTopLeftCorner.distanceTo(bottomLeft), ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToBottomSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY()),
                pointAtTopSide.getInhomY() - bottom, ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToBottomSide(pointAtTopRightCorner.getInhomX(),
                pointAtTopRightCorner.getInhomY()), pointAtTopRightCorner.distanceTo(bottomRight), ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToBottomSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY()),
                pointAtRightSide.distanceTo(bottomRight), ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToBottomSide(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY()), pointAtBottomRightCorner.distanceTo(bottomRight),
                ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToBottomSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY()),
                bottom - pointAtBottomSide.getInhomY(), ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToBottomSide(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY()), pointAtBottomLeftCorner.distanceTo(bottomLeft), ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToBottomSide(insidePoint.getInhomX(), insidePoint.getInhomY()),
                bottom - insidePoint.getInhomY(), ABSOLUTE_ERROR);

        // test non-static method with point
        r = new Rectangle(left, top, right, bottom);

        assertEquals(r.getSignedDistanceToBottomSide(pointAtLeftSide), pointAtLeftSide.distanceTo(bottomLeft),
                ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToBottomSide(pointAtTopLeftCorner), pointAtTopLeftCorner.distanceTo(bottomLeft),
                ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToBottomSide(pointAtTopSide), pointAtTopSide.getInhomY() - bottom,
                ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToBottomSide(pointAtTopRightCorner),
                pointAtTopRightCorner.distanceTo(bottomRight), ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToBottomSide(pointAtRightSide), pointAtRightSide.distanceTo(bottomRight),
                ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToBottomSide(pointAtBottomRightCorner),
                pointAtBottomRightCorner.distanceTo(bottomRight), ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToBottomSide(pointAtBottomSide), bottom - pointAtBottomSide.getInhomY(),
                ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToBottomSide(pointAtBottomLeftCorner),
                pointAtBottomLeftCorner.distanceTo(bottomLeft), ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistanceToBottomSide(insidePoint), bottom - insidePoint.getInhomY(),
                ABSOLUTE_ERROR);
    }

    @Test
    void testGetSignedDistance() {
        final var randomizer = new UniformRandomizer();
        final var centerX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var centerY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var width = randomizer.nextDouble(0, MAX_RANDOM_VALUE);
        final var height = randomizer.nextDouble(0, MAX_RANDOM_VALUE);

        final var left = centerX - 0.5 * width;
        final var right = centerX + 0.5 * width;
        final var top = centerY + 0.5 * height;
        final var bottom = centerY - 0.5 * height;

        final var topLeft = new InhomogeneousPoint2D(left, top);
        final var bottomRight = new InhomogeneousPoint2D(right, bottom);
        final var center = new InhomogeneousPoint2D(centerX, centerY);

        final var pointAtLeftSide = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5), centerY);
        final var pointAtTopLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopSide = new InhomogeneousPoint2D(
                centerX, centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtRightSide = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5), centerY);
        final var pointAtBottomRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomSide = new InhomogeneousPoint2D(
                centerX, centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var insidePoint = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(-0.5, 0.5),
                centerY + height * randomizer.nextDouble(-0.5, 0.5));

        // closest inside distance
        var r = new Rectangle(left, top, right, bottom);
        final var leftDist = r.getDistanceToLeftSide(insidePoint);
        final var topDist = r.getDistanceToTopSide(insidePoint);
        final var rightDist = r.getDistanceToRightSide(insidePoint);
        final var bottomDist = r.getDistanceToBottomSide(insidePoint);
        final var insideDist = -Math.min(Math.min(leftDist, topDist), Math.min(rightDist, bottomDist));

        // test static method with point coordinates and corners
        assertEquals(Rectangle.getSignedDistance(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), left, top,
                right, bottom), Rectangle.getSignedDistanceToLeftSide(pointAtLeftSide, topLeft, bottomRight),
                ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistance(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                left, top, right, bottom), Rectangle.getSignedDistanceToLeftSide(pointAtTopLeftCorner, topLeft,
                bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistance(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), left, top,
                right, bottom), Rectangle.getSignedDistanceToTopSide(pointAtTopSide, topLeft, bottomRight),
                ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistance(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                left, top, right, bottom), Rectangle.getSignedDistanceToTopSide(pointAtTopRightCorner, topLeft,
                bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistance(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), left, top,
                right, bottom), Rectangle.getSignedDistanceToRightSide(pointAtRightSide, topLeft, bottomRight),
                ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistance(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), left, top, right, bottom),
                Rectangle.getSignedDistanceToRightSide(pointAtBottomRightCorner, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistance(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), left,
                top, right, bottom), Rectangle.getSignedDistanceToBottomSide(pointAtBottomSide, topLeft, bottomRight),
                ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistance(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), left, top, right, bottom),
                Rectangle.getSignedDistanceToBottomSide(pointAtBottomLeftCorner, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistance(insidePoint.getInhomX(), insidePoint.getInhomY(), left, top, right,
                bottom), insideDist, ABSOLUTE_ERROR);

        // test static method with point and corner coordinates
        assertEquals(Rectangle.getSignedDistance(pointAtLeftSide, left, top, right, bottom),
                Rectangle.getSignedDistanceToLeftSide(pointAtLeftSide, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistance(pointAtTopLeftCorner, left, top, right, bottom),
                Rectangle.getSignedDistanceToLeftSide(pointAtTopLeftCorner, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistance(pointAtTopSide, left, top, right, bottom),
                Rectangle.getSignedDistanceToTopSide(pointAtTopSide, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistance(pointAtTopRightCorner, left, top, right, bottom),
                Rectangle.getSignedDistanceToTopSide(pointAtTopRightCorner, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistance(pointAtRightSide, left, top, right, bottom),
                Rectangle.getSignedDistanceToRightSide(pointAtRightSide, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistance(pointAtBottomRightCorner, left, top, right, bottom),
                Rectangle.getSignedDistanceToRightSide(pointAtBottomRightCorner, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistance(pointAtBottomSide, left, top, right, bottom),
                Rectangle.getSignedDistanceToBottomSide(pointAtBottomSide, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistance(pointAtBottomLeftCorner, left, top, right, bottom),
                Rectangle.getSignedDistanceToBottomSide(pointAtBottomLeftCorner, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistance(insidePoint, left, top, right, bottom), insideDist, ABSOLUTE_ERROR);

        // test static method with point coordinates and corners
        assertEquals(Rectangle.getSignedDistance(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), topLeft,
                bottomRight), Rectangle.getSignedDistanceToLeftSide(pointAtLeftSide, topLeft, bottomRight),
                ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistance(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                topLeft, bottomRight), Rectangle.getSignedDistanceToLeftSide(pointAtTopLeftCorner, topLeft,
                bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistance(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), topLeft,
                bottomRight), Rectangle.getSignedDistanceToTopSide(pointAtTopSide, topLeft, bottomRight),
                ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistance(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                topLeft, bottomRight), Rectangle.getSignedDistanceToTopSide(pointAtTopRightCorner, topLeft,
                bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistance(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), topLeft,
                bottomRight), Rectangle.getSignedDistanceToRightSide(pointAtRightSide, topLeft, bottomRight),
                ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistance(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), topLeft, bottomRight),
                Rectangle.getSignedDistanceToRightSide(pointAtBottomRightCorner, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistance(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), topLeft,
                bottomRight), Rectangle.getSignedDistanceToBottomSide(pointAtBottomSide, topLeft, bottomRight),
                ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistance(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), topLeft, bottomRight),
                Rectangle.getSignedDistanceToBottomSide(pointAtBottomLeftCorner, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistance(insidePoint.getInhomX(), insidePoint.getInhomY(), topLeft,
                bottomRight), insideDist, ABSOLUTE_ERROR);

        // test static method with point and corners
        assertEquals(Rectangle.getSignedDistance(pointAtLeftSide, topLeft, bottomRight),
                Rectangle.getSignedDistanceToLeftSide(pointAtLeftSide, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistance(pointAtTopLeftCorner, topLeft, bottomRight),
                Rectangle.getSignedDistanceToLeftSide(pointAtTopLeftCorner, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistance(pointAtTopSide, topLeft, bottomRight),
                Rectangle.getSignedDistanceToTopSide(pointAtTopSide, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistance(pointAtTopRightCorner, topLeft, bottomRight),
                Rectangle.getSignedDistanceToTopSide(pointAtTopRightCorner, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistance(pointAtRightSide, topLeft, bottomRight),
                Rectangle.getSignedDistanceToRightSide(pointAtRightSide, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistance(pointAtBottomRightCorner, topLeft, bottomRight),
                Rectangle.getSignedDistanceToRightSide(pointAtBottomRightCorner, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistance(pointAtBottomSide, topLeft, bottomRight),
                Rectangle.getSignedDistanceToBottomSide(pointAtBottomSide, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistance(pointAtBottomLeftCorner, topLeft, bottomRight),
                Rectangle.getSignedDistanceToBottomSide(pointAtBottomLeftCorner, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistance(insidePoint, topLeft, bottomRight), insideDist, ABSOLUTE_ERROR);

        // test static method with point coordinates and corners
        assertEquals(Rectangle.getSignedDistance(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), center,
                width, height), Rectangle.getSignedDistanceToLeftSide(pointAtLeftSide, topLeft, bottomRight),
                ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistance(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                center, width, height), Rectangle.getSignedDistanceToLeftSide(pointAtTopLeftCorner, topLeft,
                bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistance(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), center, width,
                height), Rectangle.getSignedDistanceToTopSide(pointAtTopSide, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistance(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                center, width, height), Rectangle.getSignedDistanceToTopSide(pointAtTopRightCorner, topLeft,
                bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistance(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), center,
                width, height), Rectangle.getSignedDistanceToRightSide(pointAtRightSide, topLeft, bottomRight),
                ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistance(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), center, width, height),
                Rectangle.getSignedDistanceToRightSide(pointAtBottomRightCorner, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistance(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), center,
                width, height), Rectangle.getSignedDistanceToBottomSide(pointAtBottomSide, topLeft, bottomRight),
                ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistance(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), center, width, height),
                Rectangle.getSignedDistanceToBottomSide(pointAtBottomLeftCorner, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistance(insidePoint.getInhomX(), insidePoint.getInhomY(), center, width,
                height), insideDist, ABSOLUTE_ERROR);

        // test static method with point and corners
        assertEquals(Rectangle.getSignedDistance(pointAtLeftSide, center, width, height),
                Rectangle.getSignedDistanceToLeftSide(pointAtLeftSide, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistance(pointAtTopLeftCorner, center, width, height),
                Rectangle.getSignedDistanceToLeftSide(pointAtTopLeftCorner, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistance(pointAtTopSide, center, width, height),
                Rectangle.getSignedDistanceToTopSide(pointAtTopSide, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistance(pointAtTopRightCorner, center, width, height),
                Rectangle.getSignedDistanceToTopSide(pointAtTopRightCorner, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistance(pointAtRightSide, center, width, height),
                Rectangle.getSignedDistanceToRightSide(pointAtRightSide, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistance(pointAtBottomRightCorner, center, width, height),
                Rectangle.getSignedDistanceToRightSide(pointAtBottomRightCorner, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistance(pointAtBottomSide, center, width, height),
                Rectangle.getSignedDistanceToBottomSide(pointAtBottomSide, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistance(pointAtBottomLeftCorner, center, width, height),
                Rectangle.getSignedDistanceToBottomSide(pointAtBottomLeftCorner, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Rectangle.getSignedDistance(insidePoint, center, width, height), insideDist, ABSOLUTE_ERROR);

        // test with non-static method with point coordinates
        r = new Rectangle(left, top, right, bottom);

        assertEquals(r.getSignedDistance(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY()),
                Rectangle.getSignedDistanceToLeftSide(pointAtLeftSide, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistance(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY()),
                Rectangle.getSignedDistanceToLeftSide(pointAtTopLeftCorner, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistance(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY()),
                Rectangle.getSignedDistanceToTopSide(pointAtTopSide, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistance(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY()),
                Rectangle.getSignedDistanceToTopSide(pointAtTopRightCorner, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistance(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY()),
                Rectangle.getSignedDistanceToRightSide(pointAtRightSide, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistance(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY()),
                Rectangle.getSignedDistanceToRightSide(pointAtBottomRightCorner, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistance(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY()),
                Rectangle.getSignedDistanceToBottomSide(pointAtBottomSide, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistance(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY()),
                Rectangle.getSignedDistanceToBottomSide(pointAtBottomLeftCorner, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistance(insidePoint.getInhomX(), insidePoint.getInhomY()), insideDist, ABSOLUTE_ERROR);

        // test with non-static method with point
        r = new Rectangle(left, top, right, bottom);

        assertEquals(r.getSignedDistance(pointAtLeftSide), Rectangle.getSignedDistanceToLeftSide(pointAtLeftSide,
                topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistance(pointAtTopLeftCorner),
                Rectangle.getSignedDistanceToLeftSide(pointAtTopLeftCorner, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistance(pointAtTopSide), Rectangle.getSignedDistanceToTopSide(pointAtTopSide, topLeft,
                bottomRight), ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistance(pointAtTopRightCorner),
                Rectangle.getSignedDistanceToTopSide(pointAtTopRightCorner, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistance(pointAtRightSide),
                Rectangle.getSignedDistanceToRightSide(pointAtRightSide, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistance(pointAtBottomRightCorner),
                Rectangle.getSignedDistanceToRightSide(pointAtBottomRightCorner, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistance(pointAtBottomSide),
                Rectangle.getSignedDistanceToBottomSide(pointAtBottomSide, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistance(pointAtBottomLeftCorner),
                Rectangle.getSignedDistanceToBottomSide(pointAtBottomLeftCorner, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(r.getSignedDistance(insidePoint), insideDist, ABSOLUTE_ERROR);
    }

    @Test
    void testGetDistanceToLeftSide() {
        final var randomizer = new UniformRandomizer();
        final var centerX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var centerY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var width = randomizer.nextDouble(0, MAX_RANDOM_VALUE);
        final var height = randomizer.nextDouble(0, MAX_RANDOM_VALUE);

        final var left = centerX - 0.5 * width;
        final var right = centerX + 0.5 * width;
        final var top = centerY + 0.5 * height;
        final var bottom = centerY - 0.5 * height;

        final var topLeft = new InhomogeneousPoint2D(left, top);
        final var bottomRight = new InhomogeneousPoint2D(right, bottom);
        final var center = new InhomogeneousPoint2D(centerX, centerY);

        final var pointAtLeftSide = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5), centerY);
        final var pointAtTopLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopSide = new InhomogeneousPoint2D(
                centerX, centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtRightSide = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5), centerY);
        final var pointAtBottomRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomSide = new InhomogeneousPoint2D(
                centerX, centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var insidePoint = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(-0.5, 0.5),
                centerY + height * randomizer.nextDouble(-0.5, 0.5));

        // test static method with point and corner coordinates
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtLeftSide, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), left, top,
                        right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtTopLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                        left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtTopSide, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), left, top,
                        right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtTopRightCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                        left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtRightSide, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), left, top,
                        right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtBottomRightCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(pointAtBottomRightCorner.getInhomX(),
                        pointAtBottomRightCorner.getInhomY(), left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtBottomSide, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), left, top,
                        right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtBottomLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(pointAtBottomLeftCorner.getInhomX(),
                        pointAtBottomLeftCorner.getInhomY(), left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(insidePoint, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(insidePoint.getInhomX(), insidePoint.getInhomY(), left, top, right,
                        bottom), ABSOLUTE_ERROR);

        // test static method with point and corner coordinates
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtLeftSide, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(pointAtLeftSide, left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtTopLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(pointAtTopLeftCorner, left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtTopSide, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(pointAtTopSide, left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtTopRightCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(pointAtTopRightCorner, left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtRightSide, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(pointAtRightSide, left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtBottomRightCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(pointAtBottomRightCorner, left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtBottomSide, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(pointAtBottomSide, left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtBottomLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(pointAtBottomLeftCorner, left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(insidePoint, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(insidePoint, left, top, right, bottom), ABSOLUTE_ERROR);

        // test static method with point coordinates and corners
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtLeftSide, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), topLeft,
                        bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtTopLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                        topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtTopSide, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), topLeft,
                        bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtTopRightCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                        topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtRightSide, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), topLeft,
                        bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtBottomRightCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(pointAtBottomRightCorner.getInhomX(),
                        pointAtBottomRightCorner.getInhomY(), topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtBottomSide, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), topLeft,
                        bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtBottomLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(pointAtBottomLeftCorner.getInhomX(),
                        pointAtBottomLeftCorner.getInhomY(), topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(insidePoint, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(insidePoint.getInhomX(), insidePoint.getInhomY(), topLeft, bottomRight),
                ABSOLUTE_ERROR);

        // test static method with point and corners
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtLeftSide, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(pointAtLeftSide, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtTopLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(pointAtTopLeftCorner, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtTopSide, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(pointAtTopSide, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtTopRightCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(pointAtTopRightCorner, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtRightSide, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(pointAtRightSide, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtBottomRightCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(pointAtBottomRightCorner, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtBottomSide, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(pointAtBottomSide, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtBottomLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(pointAtBottomLeftCorner, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(insidePoint, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(insidePoint, topLeft, bottomRight), ABSOLUTE_ERROR);

        // test static method with point coordinates, center and size
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtLeftSide, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), center, width,
                        height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtTopLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                        center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtTopSide, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), center, width,
                        height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtTopRightCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                        center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtRightSide, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), center,
                        width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtBottomRightCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(pointAtBottomRightCorner.getInhomX(),
                        pointAtBottomRightCorner.getInhomY(), center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtBottomSide, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), center,
                        width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtBottomLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(pointAtBottomLeftCorner.getInhomX(),
                        pointAtBottomLeftCorner.getInhomY(), center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(insidePoint, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(insidePoint.getInhomX(), insidePoint.getInhomY(), center, width,
                        height), ABSOLUTE_ERROR);

        // test static method with point, center and size
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtLeftSide, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(pointAtLeftSide, center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtTopLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(pointAtTopLeftCorner, center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtTopSide, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(pointAtTopSide, center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtTopRightCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(pointAtTopRightCorner, center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtRightSide, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(pointAtRightSide, center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtBottomRightCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(pointAtBottomRightCorner, center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtBottomSide, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(pointAtBottomSide, center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtBottomLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(pointAtBottomLeftCorner, center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(insidePoint, topLeft, bottomRight)),
                Rectangle.getDistanceToLeftSide(insidePoint, center, width, height), ABSOLUTE_ERROR);

        // test non-static method with point coordinates
        var r = new Rectangle(left, top, right, bottom);

        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtLeftSide, topLeft, bottomRight)),
                r.getDistanceToLeftSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtTopLeftCorner, topLeft, bottomRight)),
                r.getDistanceToLeftSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY()),
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtTopSide, topLeft, bottomRight)),
                r.getDistanceToLeftSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtTopRightCorner, topLeft, bottomRight)),
                r.getDistanceToLeftSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY()),
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtRightSide, topLeft, bottomRight)),
                r.getDistanceToLeftSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtBottomRightCorner, topLeft, bottomRight)),
                r.getDistanceToLeftSide(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY()),
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtBottomSide, topLeft, bottomRight)),
                r.getDistanceToLeftSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtBottomLeftCorner, topLeft, bottomRight)),
                r.getDistanceToLeftSide(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY()),
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(insidePoint, topLeft, bottomRight)),
                r.getDistanceToLeftSide(insidePoint.getInhomX(), insidePoint.getInhomY()), ABSOLUTE_ERROR);

        // test non-static method with point
        r = new Rectangle(left, top, right, bottom);

        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtLeftSide, topLeft, bottomRight)),
                r.getDistanceToLeftSide(pointAtLeftSide), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtTopLeftCorner, topLeft, bottomRight)),
                r.getDistanceToLeftSide(pointAtTopLeftCorner), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtTopSide, topLeft, bottomRight)),
                r.getDistanceToLeftSide(pointAtTopSide), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtTopRightCorner, topLeft, bottomRight)),
                r.getDistanceToLeftSide(pointAtTopRightCorner), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtRightSide, topLeft, bottomRight)),
                r.getDistanceToLeftSide(pointAtRightSide), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtBottomRightCorner, topLeft, bottomRight)),
                r.getDistanceToLeftSide(pointAtBottomRightCorner), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtBottomSide, topLeft, bottomRight)),
                r.getDistanceToLeftSide(pointAtBottomSide), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(pointAtBottomLeftCorner, topLeft, bottomRight)),
                r.getDistanceToLeftSide(pointAtBottomLeftCorner), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToLeftSide(insidePoint, topLeft, bottomRight)),
                r.getDistanceToLeftSide(insidePoint), ABSOLUTE_ERROR);
    }

    @Test
    void testGetDistanceToTopSide() {
        final var randomizer = new UniformRandomizer();
        final var centerX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var centerY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var width = randomizer.nextDouble(0, MAX_RANDOM_VALUE);
        final var height = randomizer.nextDouble(0, MAX_RANDOM_VALUE);

        final var left = centerX - 0.5 * width;
        final var right = centerX + 0.5 * width;
        final var top = centerY + 0.5 * height;
        final var bottom = centerY - 0.5 * height;

        final var topLeft = new InhomogeneousPoint2D(left, top);
        final var bottomRight = new InhomogeneousPoint2D(right, bottom);
        final var center = new InhomogeneousPoint2D(centerX, centerY);

        final var pointAtLeftSide = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5), centerY);
        final var pointAtTopLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopSide = new InhomogeneousPoint2D(
                centerX, centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtRightSide = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5), centerY);
        final var pointAtBottomRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomSide = new InhomogeneousPoint2D(
                centerX, centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var insidePoint = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(-0.5, 0.5),
                centerY + height * randomizer.nextDouble(-0.5, 0.5));

        // test static method with point and corner coordinates
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtLeftSide, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), left, top,
                        right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtTopLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(), left,
                        top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtTopSide, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), left, top, right,
                        bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtTopRightCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                        left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtRightSide, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), left, top,
                        right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtBottomRightCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(pointAtBottomRightCorner.getInhomX(),
                        pointAtBottomRightCorner.getInhomY(), left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtBottomSide, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), left, top,
                        right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtBottomLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(),
                        left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(insidePoint, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(insidePoint.getInhomX(), insidePoint.getInhomY(), left, top, right,
                        bottom), ABSOLUTE_ERROR);

        // test static method with point and corner coordinates
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtLeftSide, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(pointAtLeftSide, left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtTopLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(pointAtTopLeftCorner, left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtTopSide, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(pointAtTopSide, left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtTopRightCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(pointAtTopRightCorner, left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtRightSide, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(pointAtRightSide, left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtBottomRightCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(pointAtBottomRightCorner, left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtBottomSide, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(pointAtBottomSide, left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtBottomLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(pointAtBottomLeftCorner, left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(insidePoint, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(insidePoint, left, top, right, bottom), ABSOLUTE_ERROR);

        // test static method with point coordinates and corners
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtLeftSide, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), topLeft,
                        bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtTopLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                        topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtTopSide, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), topLeft,
                        bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtTopRightCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                        topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtRightSide, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), topLeft,
                        bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtBottomRightCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(pointAtBottomRightCorner.getInhomX(),
                        pointAtBottomRightCorner.getInhomY(), topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtBottomSide, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), topLeft,
                        bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtBottomLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(),
                        topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(insidePoint, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(insidePoint.getInhomX(), insidePoint.getInhomY(), topLeft, bottomRight),
                ABSOLUTE_ERROR);

        // test static method with point and corners
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtLeftSide, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(pointAtLeftSide, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtTopLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(pointAtTopLeftCorner, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtTopSide, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(pointAtTopSide, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtTopRightCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(pointAtTopRightCorner, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtRightSide, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(pointAtRightSide, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtBottomRightCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(pointAtBottomRightCorner, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtBottomSide, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(pointAtBottomSide, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtBottomLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(pointAtBottomLeftCorner, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(insidePoint, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(insidePoint, topLeft, bottomRight), ABSOLUTE_ERROR);

        // test static method with point coordinates, center and size
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtLeftSide, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), center, width,
                        height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtTopLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                        center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtTopSide, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), center, width,
                        height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtTopRightCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                        center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtRightSide, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), center,
                        width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtBottomRightCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(pointAtBottomRightCorner.getInhomX(),
                        pointAtBottomRightCorner.getInhomY(), center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtBottomSide, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), center,
                        width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtBottomLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(),
                        center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(insidePoint, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(insidePoint.getInhomX(), insidePoint.getInhomY(), center, width, height),
                ABSOLUTE_ERROR);

        // test static method with point, center and size
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtLeftSide, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(pointAtLeftSide, center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtTopLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(pointAtTopLeftCorner, center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtTopSide, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(pointAtTopSide, center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtTopRightCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(pointAtTopRightCorner, center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtRightSide, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(pointAtRightSide, center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtBottomRightCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(pointAtBottomRightCorner, center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtBottomSide, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(pointAtBottomSide, center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtBottomLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(pointAtBottomLeftCorner, center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(insidePoint, topLeft, bottomRight)),
                Rectangle.getDistanceToTopSide(insidePoint, center, width, height), ABSOLUTE_ERROR);

        // test non-static method with point coordinates
        var r = new Rectangle(left, top, right, bottom);

        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtLeftSide, topLeft, bottomRight)),
                r.getDistanceToTopSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtTopLeftCorner, topLeft, bottomRight)),
                r.getDistanceToTopSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY()),
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtTopSide, topLeft, bottomRight)),
                r.getDistanceToTopSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtTopRightCorner, topLeft, bottomRight)),
                r.getDistanceToTopSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY()),
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtRightSide, topLeft, bottomRight)),
                r.getDistanceToTopSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtBottomRightCorner, topLeft, bottomRight)),
                r.getDistanceToTopSide(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY()),
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtBottomSide, topLeft, bottomRight)),
                r.getDistanceToTopSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtBottomLeftCorner, topLeft, bottomRight)),
                r.getDistanceToTopSide(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY()),
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(insidePoint, topLeft, bottomRight)),
                r.getDistanceToTopSide(insidePoint.getInhomX(), insidePoint.getInhomY()), ABSOLUTE_ERROR);

        // test non-static method with point
        r = new Rectangle(left, top, right, bottom);

        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtLeftSide, topLeft, bottomRight)),
                r.getDistanceToTopSide(pointAtLeftSide), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtTopLeftCorner, topLeft, bottomRight)),
                r.getDistanceToTopSide(pointAtTopLeftCorner), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtTopSide, topLeft, bottomRight)),
                r.getDistanceToTopSide(pointAtTopSide), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtTopRightCorner, topLeft, bottomRight)),
                r.getDistanceToTopSide(pointAtTopRightCorner), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtRightSide, topLeft, bottomRight)),
                r.getDistanceToTopSide(pointAtRightSide), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtBottomRightCorner, topLeft, bottomRight)),
                r.getDistanceToTopSide(pointAtBottomRightCorner), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtBottomSide, topLeft, bottomRight)),
                r.getDistanceToTopSide(pointAtBottomSide), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(pointAtBottomLeftCorner, topLeft, bottomRight)),
                r.getDistanceToTopSide(pointAtBottomLeftCorner), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToTopSide(insidePoint, topLeft, bottomRight)),
                r.getDistanceToTopSide(insidePoint), ABSOLUTE_ERROR);
    }

    @Test
    void testGetDistanceToRightSide() {
        final var randomizer = new UniformRandomizer();
        final var centerX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var centerY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var width = randomizer.nextDouble(0, MAX_RANDOM_VALUE);
        final var height = randomizer.nextDouble(0, MAX_RANDOM_VALUE);

        final var left = centerX - 0.5 * width;
        final var right = centerX + 0.5 * width;
        final var top = centerY + 0.5 * height;
        final var bottom = centerY - 0.5 * height;

        final var topLeft = new InhomogeneousPoint2D(left, top);
        final var bottomRight = new InhomogeneousPoint2D(right, bottom);
        final var center = new InhomogeneousPoint2D(centerX, centerY);

        final var pointAtLeftSide = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5), centerY);
        final var pointAtTopLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopSide = new InhomogeneousPoint2D(
                centerX, centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtRightSide = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5), centerY);
        final var pointAtBottomRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomSide = new InhomogeneousPoint2D(
                centerX, centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var insidePoint = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(-0.5, 0.5),
                centerY + height * randomizer.nextDouble(-0.5, 0.5));

        // test static method with point and corner coordinates
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtLeftSide, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), left, top,
                        right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtTopLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                        left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtTopSide, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), left, top,
                        right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtTopRightCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                        left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtRightSide, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), left, top,
                        right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtBottomRightCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(pointAtBottomRightCorner.getInhomX(),
                        pointAtBottomRightCorner.getInhomY(), left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtBottomSide, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), left,
                        top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtBottomLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(pointAtBottomLeftCorner.getInhomX(),
                        pointAtBottomLeftCorner.getInhomY(), left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(insidePoint, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(insidePoint.getInhomX(), insidePoint.getInhomY(), left, top, right,
                        bottom), ABSOLUTE_ERROR);

        // test static method with point and corner coordinates
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtLeftSide, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(pointAtLeftSide, left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtTopLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(pointAtTopLeftCorner, left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtTopSide, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(pointAtTopSide, left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtTopRightCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(pointAtTopRightCorner, left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtRightSide, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(pointAtRightSide, left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtBottomRightCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(pointAtBottomRightCorner, left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtBottomSide, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(pointAtBottomSide, left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtBottomLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(pointAtBottomLeftCorner, left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(insidePoint, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(insidePoint, left, top, right, bottom), ABSOLUTE_ERROR);

        // test static method with point coordinates and corners
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtLeftSide, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), topLeft,
                        bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtTopLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                        topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtTopSide, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), topLeft,
                        bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtTopRightCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                        topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtRightSide, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), topLeft,
                        bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtBottomRightCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(pointAtBottomRightCorner.getInhomX(),
                        pointAtBottomRightCorner.getInhomY(), topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtBottomSide, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), topLeft,
                        bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtBottomLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(pointAtBottomLeftCorner.getInhomX(),
                        pointAtBottomLeftCorner.getInhomY(), topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(insidePoint, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(insidePoint.getInhomX(), insidePoint.getInhomY(), topLeft,
                        bottomRight), ABSOLUTE_ERROR);

        // test static method with point and corners
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtLeftSide, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(pointAtLeftSide, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtTopLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(pointAtTopLeftCorner, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtTopSide, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(pointAtTopSide, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtTopRightCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(pointAtTopRightCorner, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtRightSide, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(pointAtRightSide, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtBottomRightCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(pointAtBottomRightCorner, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtBottomSide, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(pointAtBottomSide, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtBottomLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(pointAtBottomLeftCorner, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(insidePoint, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(insidePoint, topLeft, bottomRight), ABSOLUTE_ERROR);

        // test static method with point coordinates, center and size
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtLeftSide, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), center,
                        width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtTopLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                        center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtTopSide, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), center, width,
                        height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtTopRightCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                        center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtRightSide, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), center,
                        width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtBottomRightCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(pointAtBottomRightCorner.getInhomX(),
                        pointAtBottomRightCorner.getInhomY(), center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtBottomSide, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), center,
                        width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtBottomLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(pointAtBottomLeftCorner.getInhomX(),
                        pointAtBottomLeftCorner.getInhomY(), center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(insidePoint, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(insidePoint.getInhomX(), insidePoint.getInhomY(), center, width,
                        height), ABSOLUTE_ERROR);

        // test static method with point, center and size
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtLeftSide, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(pointAtLeftSide, center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtTopLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(pointAtTopLeftCorner, center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtTopSide, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(pointAtTopSide, center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtTopRightCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(pointAtTopRightCorner, center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtRightSide, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(pointAtRightSide, center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtBottomRightCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(pointAtBottomRightCorner, center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtBottomSide, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(pointAtBottomSide, center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtBottomLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(pointAtBottomLeftCorner, center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(insidePoint, topLeft, bottomRight)),
                Rectangle.getDistanceToRightSide(insidePoint, center, width, height), ABSOLUTE_ERROR);

        // test non-static method with point coordinates
        var r = new Rectangle(left, top, right, bottom);

        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtLeftSide, topLeft, bottomRight)),
                r.getDistanceToRightSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtTopLeftCorner, topLeft, bottomRight)),
                r.getDistanceToRightSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY()),
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtTopSide, topLeft, bottomRight)),
                r.getDistanceToRightSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtTopRightCorner, topLeft, bottomRight)),
                r.getDistanceToRightSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY()),
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtRightSide, topLeft, bottomRight)),
                r.getDistanceToRightSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtBottomRightCorner, topLeft, bottomRight)),
                r.getDistanceToRightSide(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY()),
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtBottomSide, topLeft, bottomRight)),
                r.getDistanceToRightSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtBottomLeftCorner, topLeft, bottomRight)),
                r.getDistanceToRightSide(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY()),
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(insidePoint, topLeft, bottomRight)),
                r.getDistanceToRightSide(insidePoint.getInhomX(), insidePoint.getInhomY()), ABSOLUTE_ERROR);

        // test non-static method with point
        r = new Rectangle(left, top, right, bottom);

        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtLeftSide, topLeft, bottomRight)),
                r.getDistanceToRightSide(pointAtLeftSide), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtTopLeftCorner, topLeft, bottomRight)),
                r.getDistanceToRightSide(pointAtTopLeftCorner), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtTopSide, topLeft, bottomRight)),
                r.getDistanceToRightSide(pointAtTopSide), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtTopRightCorner, topLeft, bottomRight)),
                r.getDistanceToRightSide(pointAtTopRightCorner), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtRightSide, topLeft, bottomRight)),
                r.getDistanceToRightSide(pointAtRightSide), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtBottomRightCorner, topLeft, bottomRight)),
                r.getDistanceToRightSide(pointAtBottomRightCorner), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtBottomSide, topLeft, bottomRight)),
                r.getDistanceToRightSide(pointAtBottomSide), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(pointAtBottomLeftCorner, topLeft, bottomRight)),
                r.getDistanceToRightSide(pointAtBottomLeftCorner), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToRightSide(insidePoint, topLeft, bottomRight)),
                r.getDistanceToRightSide(insidePoint), ABSOLUTE_ERROR);
    }

    @Test
    void testGetDistanceToBottomSide() {
        final var randomizer = new UniformRandomizer();
        final var centerX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var centerY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var width = randomizer.nextDouble(0, MAX_RANDOM_VALUE);
        final var height = randomizer.nextDouble(0, MAX_RANDOM_VALUE);

        final var left = centerX - 0.5 * width;
        final var right = centerX + 0.5 * width;
        final var top = centerY + 0.5 * height;
        final var bottom = centerY - 0.5 * height;

        final var topLeft = new InhomogeneousPoint2D(left, top);
        final var bottomRight = new InhomogeneousPoint2D(right, bottom);
        final var center = new InhomogeneousPoint2D(centerX, centerY);

        final var pointAtLeftSide = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5), centerY);
        final var pointAtTopLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopSide = new InhomogeneousPoint2D(
                centerX, centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtRightSide = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5), centerY);
        final var pointAtBottomRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomSide = new InhomogeneousPoint2D(
                centerX, centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var insidePoint = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(-0.5, 0.5),
                centerY + height * randomizer.nextDouble(-0.5, 0.5));

        // test static method with point and corner coordinates
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtLeftSide, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), left, top,
                        right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtTopLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                        left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtTopSide, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), left, top,
                        right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtTopRightCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                        left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtRightSide, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), left, top,
                        right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtBottomRightCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(pointAtBottomRightCorner.getInhomX(),
                        pointAtBottomRightCorner.getInhomY(), left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtBottomSide, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), left,
                        top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtBottomLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(pointAtBottomLeftCorner.getInhomX(),
                        pointAtBottomLeftCorner.getInhomY(), left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(insidePoint, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(insidePoint.getInhomX(), insidePoint.getInhomY(), left, top, right,
                        bottom), ABSOLUTE_ERROR);

        // test static method with point and corner coordinates
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtLeftSide, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(pointAtLeftSide, left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtTopLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(pointAtTopLeftCorner, left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtTopSide, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(pointAtTopSide, left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtTopRightCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(pointAtTopRightCorner, left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtRightSide, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(pointAtRightSide, left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtBottomRightCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(pointAtBottomRightCorner, left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtBottomSide, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(pointAtBottomSide, left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtBottomLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(pointAtBottomLeftCorner, left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(insidePoint, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(insidePoint, left, top, right, bottom), ABSOLUTE_ERROR);

        // test static method with point coordinates and corners
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtLeftSide, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), topLeft,
                        bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtTopLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                        topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtTopSide, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), topLeft,
                        bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtTopRightCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                        topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtRightSide, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), topLeft,
                        bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtBottomRightCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(pointAtBottomRightCorner.getInhomX(),
                        pointAtBottomRightCorner.getInhomY(), topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtBottomSide, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), topLeft,
                        bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtBottomLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(pointAtBottomLeftCorner.getInhomX(),
                        pointAtBottomLeftCorner.getInhomY(), topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(insidePoint, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(insidePoint.getInhomX(), insidePoint.getInhomY(),
                        topLeft, bottomRight), ABSOLUTE_ERROR);

        // test static method with point and corners
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtLeftSide, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(pointAtLeftSide, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtTopLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(pointAtTopLeftCorner, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtTopSide, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(pointAtTopSide, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtTopRightCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(pointAtTopRightCorner, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtRightSide, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(pointAtRightSide, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtBottomRightCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(pointAtBottomRightCorner, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtBottomSide, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(pointAtBottomSide, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtBottomLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(pointAtBottomLeftCorner, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(insidePoint, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(insidePoint, topLeft, bottomRight), ABSOLUTE_ERROR);

        // test static method with point coordinates, center and size
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtLeftSide, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), center,
                        width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtTopLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                        center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtTopSide, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), center, width,
                        height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtTopRightCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                        center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtRightSide, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), center,
                        width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtBottomRightCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(pointAtBottomRightCorner.getInhomX(),
                        pointAtBottomRightCorner.getInhomY(), center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtBottomSide, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), center,
                        width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtBottomLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(pointAtBottomLeftCorner.getInhomX(),
                        pointAtBottomLeftCorner.getInhomY(), center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(insidePoint, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(insidePoint.getInhomX(), insidePoint.getInhomY(), center, width,
                        height), ABSOLUTE_ERROR);

        // test static method with point, center and size
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtLeftSide, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(pointAtLeftSide, center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtTopLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(pointAtTopLeftCorner, center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtTopSide, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(pointAtTopSide, center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtTopRightCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(pointAtTopRightCorner, center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtRightSide, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(pointAtRightSide, center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtBottomRightCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(pointAtBottomRightCorner, center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtBottomSide, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(pointAtBottomSide, center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtBottomLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(pointAtBottomLeftCorner, center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(insidePoint, topLeft, bottomRight)),
                Rectangle.getDistanceToBottomSide(insidePoint, center, width, height), ABSOLUTE_ERROR);

        // test non-static method with point coordinates
        var r = new Rectangle(left, top, right, bottom);

        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtLeftSide, topLeft, bottomRight)),
                r.getDistanceToBottomSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtTopLeftCorner, topLeft, bottomRight)),
                r.getDistanceToBottomSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY()),
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtTopSide, topLeft, bottomRight)),
                r.getDistanceToBottomSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtTopRightCorner, topLeft, bottomRight)),
                r.getDistanceToBottomSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY()),
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtRightSide, topLeft, bottomRight)),
                r.getDistanceToBottomSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtBottomRightCorner, topLeft, bottomRight)),
                r.getDistanceToBottomSide(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY()),
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtBottomSide, topLeft, bottomRight)),
                r.getDistanceToBottomSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY()),
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtBottomLeftCorner, topLeft, bottomRight)),
                r.getDistanceToBottomSide(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY()),
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(insidePoint, topLeft, bottomRight)),
                r.getDistanceToBottomSide(insidePoint.getInhomX(), insidePoint.getInhomY()), ABSOLUTE_ERROR);

        // test non-static method with point
        r = new Rectangle(left, top, right, bottom);

        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtLeftSide, topLeft, bottomRight)),
                r.getDistanceToBottomSide(pointAtLeftSide), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtTopLeftCorner, topLeft, bottomRight)),
                r.getDistanceToBottomSide(pointAtTopLeftCorner), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtTopSide, topLeft, bottomRight)),
                r.getDistanceToBottomSide(pointAtTopSide), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtTopRightCorner, topLeft, bottomRight)),
                r.getDistanceToBottomSide(pointAtTopRightCorner), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtRightSide, topLeft, bottomRight)),
                r.getDistanceToBottomSide(pointAtRightSide), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtBottomRightCorner, topLeft, bottomRight)),
                r.getDistanceToBottomSide(pointAtBottomRightCorner), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtBottomSide, topLeft, bottomRight)),
                r.getDistanceToBottomSide(pointAtBottomSide), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(pointAtBottomLeftCorner, topLeft, bottomRight)),
                r.getDistanceToBottomSide(pointAtBottomLeftCorner), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistanceToBottomSide(insidePoint, topLeft, bottomRight)),
                r.getDistanceToBottomSide(insidePoint), ABSOLUTE_ERROR);
    }

    @Test
    void testGetDistance() {
        final var randomizer = new UniformRandomizer();
        final var centerX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var centerY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var width = randomizer.nextDouble(0, MAX_RANDOM_VALUE);
        final var height = randomizer.nextDouble(0, MAX_RANDOM_VALUE);

        final var left = centerX - 0.5 * width;
        final var right = centerX + 0.5 * width;
        final var top = centerY + 0.5 * height;
        final var bottom = centerY - 0.5 * height;

        final var topLeft = new InhomogeneousPoint2D(left, top);
        final var bottomRight = new InhomogeneousPoint2D(right, bottom);
        final var center = new InhomogeneousPoint2D(centerX, centerY);

        final var pointAtLeftSide = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5), centerY);
        final var pointAtTopLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopSide = new InhomogeneousPoint2D(
                centerX, centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtRightSide = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5), centerY);
        final var pointAtBottomRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomSide = new InhomogeneousPoint2D(
                centerX, centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var insidePoint = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(-0.5, 0.5),
                centerY + height * randomizer.nextDouble(-0.5, 0.5));

        // test static method with point and corner coordinates
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtLeftSide, topLeft, bottomRight)),
                Rectangle.getDistance(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), left, top, right,
                        bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtTopLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistance(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(), left, top,
                        right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtTopSide, topLeft, bottomRight)),
                Rectangle.getDistance(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), left, top, right, bottom),
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtTopRightCorner, topLeft, bottomRight)),
                Rectangle.getDistance(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(), left, top,
                        right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtRightSide, topLeft, bottomRight)),
                Rectangle.getDistance(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), left, top, right,
                        bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtBottomRightCorner, topLeft, bottomRight)),
                Rectangle.getDistance(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY(), left,
                        top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtBottomSide, topLeft, bottomRight)),
                Rectangle.getDistance(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), left, top, right,
                        bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtBottomLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistance(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(), left,
                        top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(insidePoint, topLeft, bottomRight)),
                Rectangle.getDistance(insidePoint.getInhomX(), insidePoint.getInhomY(), left, top, right, bottom),
                ABSOLUTE_ERROR);

        // test static method with point and corner coordinates
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtLeftSide, topLeft, bottomRight)),
                Rectangle.getDistance(pointAtLeftSide, left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtTopLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistance(pointAtTopLeftCorner, left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtTopSide, topLeft, bottomRight)),
                Rectangle.getDistance(pointAtTopSide, left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtTopRightCorner, topLeft, bottomRight)),
                Rectangle.getDistance(pointAtTopRightCorner, left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtRightSide, topLeft, bottomRight)),
                Rectangle.getDistance(pointAtRightSide, left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtBottomRightCorner, topLeft, bottomRight)),
                Rectangle.getDistance(pointAtBottomRightCorner, left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtBottomSide, topLeft, bottomRight)),
                Rectangle.getDistance(pointAtBottomSide, left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtBottomLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistance(pointAtBottomLeftCorner, left, top, right, bottom), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(insidePoint, topLeft, bottomRight)),
                Rectangle.getDistance(insidePoint, left, top, right, bottom), ABSOLUTE_ERROR);

        // test static method with point coordinates and corners
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtLeftSide, topLeft, bottomRight)),
                Rectangle.getDistance(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), topLeft, bottomRight),
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtTopLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistance(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(), topLeft,
                        bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtTopSide, topLeft, bottomRight)),
                Rectangle.getDistance(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), topLeft, bottomRight),
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtTopRightCorner, topLeft, bottomRight)),
                Rectangle.getDistance(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(), topLeft,
                        bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtRightSide, topLeft, bottomRight)),
                Rectangle.getDistance(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), topLeft, bottomRight),
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtBottomRightCorner, topLeft, bottomRight)),
                Rectangle.getDistance(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY(),
                        topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtBottomSide, topLeft, bottomRight)),
                Rectangle.getDistance(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), topLeft,
                        bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtBottomLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistance(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(), topLeft,
                        bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(insidePoint, topLeft, bottomRight)),
                Rectangle.getDistance(insidePoint.getInhomX(), insidePoint.getInhomY(), topLeft, bottomRight),
                ABSOLUTE_ERROR);

        // test static method with point and corners
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtLeftSide, topLeft, bottomRight)),
                Rectangle.getDistance(pointAtLeftSide, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtTopLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistance(pointAtTopLeftCorner, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtTopSide, topLeft, bottomRight)),
                Rectangle.getDistance(pointAtTopSide, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtTopRightCorner, topLeft, bottomRight)),
                Rectangle.getDistance(pointAtTopRightCorner, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtRightSide, topLeft, bottomRight)),
                Rectangle.getDistance(pointAtRightSide, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtBottomRightCorner, topLeft, bottomRight)),
                Rectangle.getDistance(pointAtBottomRightCorner, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtBottomSide, topLeft, bottomRight)),
                Rectangle.getDistance(pointAtBottomSide, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getDistance(pointAtBottomLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistance(pointAtBottomLeftCorner, topLeft, bottomRight), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(insidePoint, topLeft, bottomRight)),
                Rectangle.getDistance(insidePoint, topLeft, bottomRight), ABSOLUTE_ERROR);

        // test static method with point coordinates, center and size
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtLeftSide, topLeft, bottomRight)),
                Rectangle.getDistance(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), center, width, height),
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtTopLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistance(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(), center, width,
                        height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtTopSide, topLeft, bottomRight)),
                Rectangle.getDistance(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), center, width, height),
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtTopRightCorner, topLeft, bottomRight)),
                Rectangle.getDistance(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(), center,
                        width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtRightSide, topLeft, bottomRight)),
                Rectangle.getDistance(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), center, width,
                        height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtBottomRightCorner, topLeft, bottomRight)),
                Rectangle.getDistance(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY(),
                        center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtBottomSide, topLeft, bottomRight)),
                Rectangle.getDistance(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), center, width,
                        height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtBottomLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistance(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(), center,
                        width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(insidePoint, topLeft, bottomRight)),
                Rectangle.getDistance(insidePoint.getInhomX(), insidePoint.getInhomY(), center, width, height),
                ABSOLUTE_ERROR);

        // test static method with point, center and size
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtLeftSide, topLeft, bottomRight)),
                Rectangle.getDistance(pointAtLeftSide, center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtTopLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistance(pointAtTopLeftCorner, center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtTopSide, topLeft, bottomRight)),
                Rectangle.getDistance(pointAtTopSide, center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtTopRightCorner, topLeft, bottomRight)),
                Rectangle.getDistance(pointAtTopRightCorner, center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtRightSide, topLeft, bottomRight)),
                Rectangle.getDistance(pointAtRightSide, center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtBottomRightCorner, topLeft, bottomRight)),
                Rectangle.getDistance(pointAtBottomRightCorner, center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtBottomSide, topLeft, bottomRight)),
                Rectangle.getDistance(pointAtBottomSide, center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtBottomLeftCorner, topLeft, bottomRight)),
                Rectangle.getDistance(pointAtBottomLeftCorner, center, width, height), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(insidePoint, topLeft, bottomRight)),
                Rectangle.getDistance(insidePoint, center, width, height), ABSOLUTE_ERROR);

        // test non-static method with point coordinates
        var r = new Rectangle(left, top, right, bottom);

        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtLeftSide, topLeft, bottomRight)),
                r.getDistance(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtTopLeftCorner, topLeft, bottomRight)),
                r.getDistance(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtTopSide, topLeft, bottomRight)),
                r.getDistance(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtTopRightCorner, topLeft, bottomRight)),
                r.getDistance(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtRightSide, topLeft, bottomRight)),
                r.getDistance(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtBottomRightCorner, topLeft, bottomRight)),
                r.getDistance(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY()),
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtBottomSide, topLeft, bottomRight)),
                r.getDistance(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtBottomLeftCorner, topLeft, bottomRight)),
                r.getDistance(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY()),
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(insidePoint, topLeft, bottomRight)),
                r.getDistance(insidePoint.getInhomX(), insidePoint.getInhomY()), ABSOLUTE_ERROR);

        // test non-static method with point
        r = new Rectangle(left, top, right, bottom);

        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtLeftSide, topLeft, bottomRight)),
                r.getDistance(pointAtLeftSide), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtTopLeftCorner, topLeft, bottomRight)),
                r.getDistance(pointAtTopLeftCorner), ABSOLUTE_ERROR);
        assertEquals( Math.abs(Rectangle.getSignedDistance(pointAtTopSide, topLeft, bottomRight)),
                r.getDistance(pointAtTopSide), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtTopRightCorner, topLeft, bottomRight)),
                r.getDistance(pointAtTopRightCorner), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtRightSide, topLeft, bottomRight)),
                r.getDistance(pointAtRightSide), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtBottomRightCorner, topLeft, bottomRight)),
                r.getDistance(pointAtBottomRightCorner), ABSOLUTE_ERROR);
        assertEquals( Math.abs(Rectangle.getSignedDistance(pointAtBottomSide, topLeft, bottomRight)),
                r.getDistance(pointAtBottomSide), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(pointAtBottomLeftCorner, topLeft, bottomRight)),
                r.getDistance(pointAtBottomLeftCorner), ABSOLUTE_ERROR);
        assertEquals(Math.abs(Rectangle.getSignedDistance(insidePoint, topLeft, bottomRight)),
                r.getDistance(insidePoint), ABSOLUTE_ERROR);
    }

    @Test
    void testClosestPoint() {
        final var randomizer = new UniformRandomizer();
        final var centerX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var centerY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var width = randomizer.nextDouble(0, MAX_RANDOM_VALUE);
        final var height = randomizer.nextDouble(0, MAX_RANDOM_VALUE);

        final var left = centerX - 0.5 * width;
        final var right = centerX + 0.5 * width;
        final var top = centerY + 0.5 * height;
        final var bottom = centerY - 0.5 * height;

        final var topLeft = new InhomogeneousPoint2D(left, top);
        final var topRight = new InhomogeneousPoint2D(right, top);
        final var bottomLeft = new InhomogeneousPoint2D(left, bottom);
        final var bottomRight = new InhomogeneousPoint2D(right, bottom);
        final var center = new InhomogeneousPoint2D(centerX, centerY);

        final var pointAtLeftSide = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5), centerY);
        final var pointAtTopLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopSide = new InhomogeneousPoint2D(
                centerX, centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtRightSide = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5), centerY);
        final var pointAtBottomRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomSide = new InhomogeneousPoint2D(
                centerX, centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var insidePoint = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(-0.5, 0.5),
                centerY + height * randomizer.nextDouble(-0.5, 0.5));

        var r = new Rectangle(left, top, right, bottom);
        final var leftDist = r.getDistanceToLeftSide(insidePoint);
        final var topDist = r.getDistanceToTopSide(insidePoint);
        final var rightDist = r.getDistanceToRightSide(insidePoint);
        final var bottomDist = r.getDistanceToBottomSide(insidePoint);
        final Point2D closestInside;
        if (leftDist < topDist && leftDist < rightDist && leftDist < bottomDist) {
            // left is closest
            closestInside = new InhomogeneousPoint2D(left, insidePoint.getInhomY());
        } else if (topDist < leftDist && topDist < rightDist && topDist < bottomDist) {
            // top is closest
            closestInside = new InhomogeneousPoint2D(insidePoint.getInhomX(), top);
        } else if (rightDist < leftDist && rightDist < topDist && rightDist < bottomDist) {
            // right is closest
            closestInside = new InhomogeneousPoint2D(right, insidePoint.getInhomY());
        } else {
            // bottom is closest
            closestInside = new InhomogeneousPoint2D(insidePoint.getInhomX(), bottom);
        }

        // test static method with point and corner coordinates
        var result = Point2D.create();
        Rectangle.closestPoint(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), left, top, right, bottom,
                result);
        assertEquals(new InhomogeneousPoint2D(left, pointAtLeftSide.getInhomY()), result);
        result = Point2D.create();
        Rectangle.closestPoint(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(), left, top, right,
                bottom, result);
        assertEquals(topLeft, result);
        result = Point2D.create();
        Rectangle.closestPoint(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), left, top, right, bottom,
                result);
        assertEquals(new InhomogeneousPoint2D(pointAtTopSide.getInhomX(), top), result);
        result = Point2D.create();
        Rectangle.closestPoint(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(), left, top, right,
                bottom, result);
        assertEquals(topRight, result);
        result = Point2D.create();
        Rectangle.closestPoint(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), left, top, right, bottom,
                result);
        assertEquals(new InhomogeneousPoint2D(right, pointAtRightSide.getInhomY()), result);
        result = Point2D.create();
        Rectangle.closestPoint(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY(), left, top,
                right, bottom, result);
        assertEquals(bottomRight, result);
        result = Point2D.create();
        Rectangle.closestPoint(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), left, top, right, bottom,
                result);
        assertEquals(new InhomogeneousPoint2D(pointAtBottomSide.getInhomX(), bottom), result);
        result = Point2D.create();
        Rectangle.closestPoint(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(), left, top,
                right, bottom, result);
        assertEquals(bottomLeft, result);
        result = Point2D.create();
        Rectangle.closestPoint(insidePoint.getInhomX(), insidePoint.getInhomY(), left, top, right, bottom, result);
        assertEquals(closestInside, result);

        // test static method with point and corner coordinates
        result = Point2D.create();
        Rectangle.closestPoint(pointAtLeftSide, left, top, right, bottom, result);
        assertEquals(new InhomogeneousPoint2D(left, pointAtLeftSide.getInhomY()), result);
        result = Point2D.create();
        Rectangle.closestPoint(pointAtTopLeftCorner, left, top, right, bottom, result);
        assertEquals(topLeft, result);
        result = Point2D.create();
        Rectangle.closestPoint(pointAtTopSide, left, top, right, bottom, result);
        assertEquals(new InhomogeneousPoint2D(pointAtTopSide.getInhomX(), top), result);
        result = Point2D.create();
        Rectangle.closestPoint(pointAtTopRightCorner, left, top, right, bottom, result);
        assertEquals(topRight, result);
        result = Point2D.create();
        Rectangle.closestPoint(pointAtRightSide, left, top, right, bottom, result);
        assertEquals(new InhomogeneousPoint2D(right, pointAtRightSide.getInhomY()), result);
        result = Point2D.create();
        Rectangle.closestPoint(pointAtBottomRightCorner, left, top, right, bottom, result);
        assertEquals(bottomRight, result);
        result = Point2D.create();
        Rectangle.closestPoint(pointAtBottomSide, left, top, right, bottom, result);
        assertEquals(new InhomogeneousPoint2D(pointAtBottomSide.getInhomX(), bottom), result);
        result = Point2D.create();
        Rectangle.closestPoint(pointAtBottomLeftCorner, left, top, right, bottom, result);
        assertEquals(bottomLeft, result);
        result = Point2D.create();
        Rectangle.closestPoint(insidePoint, left, top, right, bottom, result);
        assertEquals(closestInside, result);

        // test static method with point coordinates and corners
        result = Point2D.create();
        Rectangle.closestPoint(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), topLeft, bottomRight, result);
        assertEquals(new InhomogeneousPoint2D(left, pointAtLeftSide.getInhomY()), result);
        result = Point2D.create();
        Rectangle.closestPoint(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(), topLeft, bottomRight,
                result);
        assertEquals(topLeft, result);
        result = Point2D.create();
        Rectangle.closestPoint(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), topLeft, bottomRight, result);
        assertEquals(new InhomogeneousPoint2D(pointAtTopSide.getInhomX(), top), result);
        result = Point2D.create();
        Rectangle.closestPoint(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(), topLeft,
                bottomRight, result);
        assertEquals(topRight, result);
        result = Point2D.create();
        Rectangle.closestPoint(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), topLeft, bottomRight,
                result);
        assertEquals(new InhomogeneousPoint2D(right, pointAtRightSide.getInhomY()), result);
        result = Point2D.create();
        Rectangle.closestPoint(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY(), topLeft,
                bottomRight, result);
        assertEquals(bottomRight, result);
        result = Point2D.create();
        Rectangle.closestPoint(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), topLeft, bottomRight,
                result);
        assertEquals(new InhomogeneousPoint2D(pointAtBottomSide.getInhomX(), bottom), result);
        result = Point2D.create();
        Rectangle.closestPoint(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(), topLeft,
                bottomRight, result);
        assertEquals(bottomLeft, result);
        result = Point2D.create();
        Rectangle.closestPoint(insidePoint.getInhomX(), insidePoint.getInhomY(), topLeft, bottomRight, result);
        assertEquals(closestInside, result);

        // test static method with point and corners
        result = Point2D.create();
        Rectangle.closestPoint(pointAtLeftSide, topLeft, bottomRight, result);
        assertEquals(new InhomogeneousPoint2D(left, pointAtLeftSide.getInhomY()), result);
        result = Point2D.create();
        Rectangle.closestPoint(pointAtTopLeftCorner, topLeft, bottomRight, result);
        assertEquals(topLeft, result);
        result = Point2D.create();
        Rectangle.closestPoint(pointAtTopSide, topLeft, bottomRight, result);
        assertEquals(new InhomogeneousPoint2D(pointAtTopSide.getInhomX(), top), result);
        result = Point2D.create();
        Rectangle.closestPoint(pointAtTopRightCorner, topLeft, bottomRight, result);
        assertEquals(topRight, result);
        result = Point2D.create();
        Rectangle.closestPoint(pointAtRightSide, topLeft, bottomRight, result);
        assertEquals(new InhomogeneousPoint2D(right, pointAtRightSide.getInhomY()), result);
        result = Point2D.create();
        Rectangle.closestPoint(pointAtBottomRightCorner, topLeft, bottomRight, result);
        assertEquals(bottomRight, result);
        result = Point2D.create();
        Rectangle.closestPoint(pointAtBottomSide, topLeft, bottomRight, result);
        assertEquals(new InhomogeneousPoint2D(pointAtBottomSide.getInhomX(), bottom), result);
        result = Point2D.create();
        Rectangle.closestPoint(pointAtBottomLeftCorner, topLeft, bottomRight, result);
        assertEquals(bottomLeft, result);
        result = Point2D.create();
        Rectangle.closestPoint(insidePoint, topLeft, bottomRight, result);
        assertEquals(closestInside, result);

        // test static method with point coordinates, center and size
        result = Point2D.create();
        Rectangle.closestPoint(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), center, width, height, result);
        assertEquals(new InhomogeneousPoint2D(left, pointAtLeftSide.getInhomY()), result);
        result = Point2D.create();
        Rectangle.closestPoint(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(), center, width,
                height, result);
        assertEquals(topLeft, result);
        result = Point2D.create();
        Rectangle.closestPoint(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), center, width, height, result);
        assertEquals(new InhomogeneousPoint2D(pointAtTopSide.getInhomX(), top), result);
        result = Point2D.create();
        Rectangle.closestPoint(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(), center, width,
                height, result);
        assertEquals(topRight, result);
        result = Point2D.create();
        Rectangle.closestPoint(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), center, width, height,
                result);
        assertEquals(new InhomogeneousPoint2D(right, pointAtRightSide.getInhomY()), result);
        result = Point2D.create();
        Rectangle.closestPoint(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY(), center,
                width, height, result);
        assertEquals(bottomRight, result);
        result = Point2D.create();
        Rectangle.closestPoint(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), center, width, height,
                result);
        assertEquals(new InhomogeneousPoint2D(pointAtBottomSide.getInhomX(), bottom), result);
        result = Point2D.create();
        Rectangle.closestPoint(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(), center, width,
                height, result);
        assertEquals(bottomLeft, result);
        result = Point2D.create();
        Rectangle.closestPoint(insidePoint.getInhomX(), insidePoint.getInhomY(), center, width, height, result);
        assertEquals(closestInside, result);

        // test static method with point, center and size
        result = Point2D.create();
        Rectangle.closestPoint(pointAtLeftSide, center, width, height, result);
        assertEquals(new InhomogeneousPoint2D(left, pointAtLeftSide.getInhomY()), result);
        result = Point2D.create();
        Rectangle.closestPoint(pointAtTopLeftCorner, center, width, height, result);
        assertEquals(topLeft, result);
        result = Point2D.create();
        Rectangle.closestPoint(pointAtTopSide, center, width, height, result);
        assertEquals(new InhomogeneousPoint2D(pointAtTopSide.getInhomX(), top), result);
        result = Point2D.create();
        Rectangle.closestPoint(pointAtTopRightCorner, center, width, height, result);
        assertEquals(topRight, result);
        result = Point2D.create();
        Rectangle.closestPoint(pointAtRightSide, center, width, height, result);
        assertEquals(new InhomogeneousPoint2D(right, pointAtRightSide.getInhomY()), result);
        result = Point2D.create();
        Rectangle.closestPoint(pointAtBottomRightCorner, center, width, height, result);
        assertEquals(bottomRight, result);
        result = Point2D.create();
        Rectangle.closestPoint(pointAtBottomSide, center, width, height, result);
        assertEquals(new InhomogeneousPoint2D(pointAtBottomSide.getInhomX(), bottom), result);
        result = Point2D.create();
        Rectangle.closestPoint(pointAtBottomLeftCorner, center, width, height, result);
        assertEquals(bottomLeft, result);
        result = Point2D.create();
        Rectangle.closestPoint(insidePoint, center, width, height, result);
        assertEquals(closestInside, result);

        // test non-static method with point coordinates
        r = new Rectangle(left, top, right, bottom);

        result = Point2D.create();
        r.closestPoint(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), result);
        assertEquals(new InhomogeneousPoint2D(left, pointAtLeftSide.getInhomY()), result);
        result = Point2D.create();
        r.closestPoint(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(), result);
        assertEquals(topLeft, result);
        result = Point2D.create();
        r.closestPoint(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), result);
        assertEquals(new InhomogeneousPoint2D(pointAtTopSide.getInhomX(), top), result);
        result = Point2D.create();
        r.closestPoint(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(), result);
        assertEquals(topRight, result);
        result = Point2D.create();
        r.closestPoint(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), result);
        assertEquals(new InhomogeneousPoint2D(right, pointAtRightSide.getInhomY()), result);
        result = Point2D.create();
        r.closestPoint(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY(), result);
        assertEquals(bottomRight, result);
        result = Point2D.create();
        r.closestPoint(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), result);
        assertEquals(new InhomogeneousPoint2D(pointAtBottomSide.getInhomX(), bottom), result);
        result = Point2D.create();
        r.closestPoint(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(), result);
        assertEquals(bottomLeft, result);
        result = Point2D.create();
        r.closestPoint(insidePoint.getInhomX(), insidePoint.getInhomY(), result);
        assertEquals(closestInside, result);

        // test non-static method with point coordinates
        r = new Rectangle(left, top, right, bottom);

        result = Point2D.create();
        r.closestPoint(pointAtLeftSide, result);
        assertEquals(new InhomogeneousPoint2D(left, pointAtLeftSide.getInhomY()), result);
        result = Point2D.create();
        r.closestPoint(pointAtTopLeftCorner, result);
        assertEquals(topLeft, result);
        result = Point2D.create();
        r.closestPoint(pointAtTopSide, result);
        assertEquals(new InhomogeneousPoint2D(pointAtTopSide.getInhomX(), top), result);
        result = Point2D.create();
        r.closestPoint(pointAtTopRightCorner, result);
        assertEquals(topRight, result);
        result = Point2D.create();
        r.closestPoint(pointAtRightSide, result);
        assertEquals(new InhomogeneousPoint2D(right, pointAtRightSide.getInhomY()), result);
        result = Point2D.create();
        r.closestPoint(pointAtBottomRightCorner, result);
        assertEquals(bottomRight, result);
        result = Point2D.create();
        r.closestPoint(pointAtBottomSide, result);
        assertEquals(new InhomogeneousPoint2D(pointAtBottomSide.getInhomX(), bottom), result);
        result = Point2D.create();
        r.closestPoint(pointAtBottomLeftCorner, result);
        assertEquals(bottomLeft, result);
        result = Point2D.create();
        r.closestPoint(insidePoint, result);
        assertEquals(closestInside, result);
    }

    @Test
    void testGetClosestPoint() {
        final var randomizer = new UniformRandomizer();
        final var centerX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var centerY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var width = randomizer.nextDouble(0, MAX_RANDOM_VALUE);
        final var height = randomizer.nextDouble(0, MAX_RANDOM_VALUE);

        final var left = centerX - 0.5 * width;
        final var right = centerX + 0.5 * width;
        final var top = centerY + 0.5 * height;
        final var bottom = centerY - 0.5 * height;

        final var topLeft = new InhomogeneousPoint2D(left, top);
        final var topRight = new InhomogeneousPoint2D(right, top);
        final var bottomLeft = new InhomogeneousPoint2D(left, bottom);
        final var bottomRight = new InhomogeneousPoint2D(right, bottom);
        final var center = new InhomogeneousPoint2D(centerX, centerY);

        final var pointAtLeftSide = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5), centerY);
        final var pointAtTopLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopSide = new InhomogeneousPoint2D(
                centerX, centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtRightSide = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5), centerY);
        final var pointAtBottomRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomSide = new InhomogeneousPoint2D(
                centerX, centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var insidePoint = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(-0.5, 0.5),
                centerY + height * randomizer.nextDouble(-0.5, 0.5));

        var r = new Rectangle(left, top, right, bottom);
        final var leftDist = r.getDistanceToLeftSide(insidePoint);
        final var topDist = r.getDistanceToTopSide(insidePoint);
        final var rightDist = r.getDistanceToRightSide(insidePoint);
        final var bottomDist = r.getDistanceToBottomSide(insidePoint);
        final Point2D closestInside;
        if (leftDist < topDist && leftDist < rightDist && leftDist < bottomDist) {
            // lest is closest
            closestInside = new InhomogeneousPoint2D(left, insidePoint.getInhomY());
        } else if (topDist < leftDist && topDist < rightDist && topDist < bottomDist) {
            // top is closest
            closestInside = new InhomogeneousPoint2D(insidePoint.getInhomX(), top);
        } else if (rightDist < leftDist && rightDist < topDist && rightDist < bottomDist) {
            // right is closest
            closestInside = new InhomogeneousPoint2D(right, insidePoint.getInhomY());
        } else {
            // bottom is closest
            closestInside = new InhomogeneousPoint2D(insidePoint.getInhomX(), bottom);
        }

        // test static method with point and corner coordinates
        Point2D result = Rectangle.getClosestPoint(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), left, top,
                right, bottom);
        assertEquals(new InhomogeneousPoint2D(left, pointAtLeftSide.getInhomY()), result);
        result = Rectangle.getClosestPoint(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(), left,
                top, right, bottom);
        assertEquals(topLeft, result);
        result = Rectangle.getClosestPoint(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), left, top, right,
                bottom);
        assertEquals(new InhomogeneousPoint2D(pointAtTopSide.getInhomX(), top), result);
        result = Rectangle.getClosestPoint(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(), left,
                top, right, bottom);
        assertEquals(topRight, result);
        result = Rectangle.getClosestPoint(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), left, top, right,
                bottom);
        assertEquals(new InhomogeneousPoint2D(right, pointAtRightSide.getInhomY()), result);
        result = Rectangle.getClosestPoint(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY(),
                left, top, right, bottom);
        assertEquals(bottomRight, result);
        result = Rectangle.getClosestPoint(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), left, top,
                right, bottom);
        assertEquals(new InhomogeneousPoint2D(pointAtBottomSide.getInhomX(), bottom), result);
        result = Rectangle.getClosestPoint(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(),
                left, top, right, bottom);
        assertEquals(bottomLeft, result);
        result = Rectangle.getClosestPoint(insidePoint.getInhomX(), insidePoint.getInhomY(), left, top, right, bottom);
        assertEquals(closestInside, result);

        // test static method with point and corner coordinates
        result = Rectangle.getClosestPoint(pointAtLeftSide, left, top, right, bottom);
        assertEquals(new InhomogeneousPoint2D(left, pointAtLeftSide.getInhomY()), result);
        result = Rectangle.getClosestPoint(pointAtTopLeftCorner, left, top, right, bottom);
        assertEquals(topLeft, result);
        result = Rectangle.getClosestPoint(pointAtTopSide, left, top, right, bottom);
        assertEquals(new InhomogeneousPoint2D(pointAtTopSide.getInhomX(), top), result);
        result = Rectangle.getClosestPoint(pointAtTopRightCorner, left, top, right, bottom);
        assertEquals(topRight, result);
        result = Rectangle.getClosestPoint(pointAtRightSide, left, top, right, bottom);
        assertEquals(new InhomogeneousPoint2D(right, pointAtRightSide.getInhomY()), result);
        result = Rectangle.getClosestPoint(pointAtBottomRightCorner, left, top, right, bottom);
        assertEquals(bottomRight, result);
        result = Rectangle.getClosestPoint(pointAtBottomSide, left, top, right, bottom);
        assertEquals(new InhomogeneousPoint2D(pointAtBottomSide.getInhomX(), bottom), result);
        result = Rectangle.getClosestPoint(pointAtBottomLeftCorner, left, top, right, bottom);
        assertEquals(bottomLeft, result);
        result = Rectangle.getClosestPoint(insidePoint, left, top, right, bottom);
        assertEquals(closestInside, result);

        // test static method with point coordinates and corners
        result = Rectangle.getClosestPoint(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), topLeft,
                bottomRight);
        assertEquals(new InhomogeneousPoint2D(left, pointAtLeftSide.getInhomY()), result);
        result = Rectangle.getClosestPoint(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(), topLeft,
                bottomRight);
        assertEquals(topLeft, result);
        result = Rectangle.getClosestPoint(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), topLeft,
                bottomRight);
        assertEquals(new InhomogeneousPoint2D(pointAtTopSide.getInhomX(), top), result);
        result = Rectangle.getClosestPoint(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                topLeft, bottomRight);
        assertEquals(topRight, result);
        result = Rectangle.getClosestPoint(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), topLeft,
                bottomRight);
        assertEquals(new InhomogeneousPoint2D(right, pointAtRightSide.getInhomY()), result);
        result = Rectangle.getClosestPoint(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY(),
                topLeft, bottomRight);
        assertEquals(bottomRight, result);
        result = Rectangle.getClosestPoint(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), topLeft,
                bottomRight);
        assertEquals(new InhomogeneousPoint2D(pointAtBottomSide.getInhomX(), bottom), result);
        result = Rectangle.getClosestPoint(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(),
                topLeft, bottomRight);
        assertEquals(bottomLeft, result);
        result = Rectangle.getClosestPoint(insidePoint.getInhomX(), insidePoint.getInhomY(), topLeft, bottomRight);
        assertEquals(closestInside, result);

        // test static method with point and corners
        result = Rectangle.getClosestPoint(pointAtLeftSide, topLeft, bottomRight);
        assertEquals(new InhomogeneousPoint2D(left, pointAtLeftSide.getInhomY()), result);
        result = Rectangle.getClosestPoint(pointAtTopLeftCorner, topLeft, bottomRight);
        assertEquals(topLeft, result);
        result = Rectangle.getClosestPoint(pointAtTopSide, topLeft, bottomRight);
        assertEquals(new InhomogeneousPoint2D(pointAtTopSide.getInhomX(), top), result);
        result = Rectangle.getClosestPoint(pointAtTopRightCorner, topLeft, bottomRight);
        assertEquals(topRight, result);
        result = Rectangle.getClosestPoint(pointAtRightSide, topLeft, bottomRight);
        assertEquals(new InhomogeneousPoint2D(right, pointAtRightSide.getInhomY()), result);
        result = Rectangle.getClosestPoint(pointAtBottomRightCorner, topLeft, bottomRight);
        assertEquals(result, bottomRight);
        result = Rectangle.getClosestPoint(pointAtBottomSide, topLeft, bottomRight);
        assertEquals(new InhomogeneousPoint2D(pointAtBottomSide.getInhomX(), bottom), result);
        result = Rectangle.getClosestPoint(pointAtBottomLeftCorner, topLeft, bottomRight);
        assertEquals(bottomLeft, result);
        result = Rectangle.getClosestPoint(insidePoint, topLeft, bottomRight);
        assertEquals(closestInside, result);

        // test static method with point coordinates, center and size
        result = Rectangle.getClosestPoint(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), center, width,
                height);
        assertEquals(new InhomogeneousPoint2D(left, pointAtLeftSide.getInhomY()), result);
        result = Rectangle.getClosestPoint(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(), center,
                width, height);
        assertEquals(topLeft, result);
        result = Rectangle.getClosestPoint(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), center, width,
                height);
        assertEquals(new InhomogeneousPoint2D(pointAtTopSide.getInhomX(), top), result);
        result = Rectangle.getClosestPoint(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(), center,
                width, height);
        assertEquals(topRight, result);
        result = Rectangle.getClosestPoint(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), center, width,
                height);
        assertEquals(new InhomogeneousPoint2D(right, pointAtRightSide.getInhomY()), result);
        result = Rectangle.getClosestPoint(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY(),
                center, width, height);
        assertEquals(bottomRight, result);
        result = Rectangle.getClosestPoint(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), center, width,
                height);
        assertEquals(new InhomogeneousPoint2D(pointAtBottomSide.getInhomX(), bottom), result);
        result = Rectangle.getClosestPoint(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(),
                center, width, height);
        assertEquals(bottomLeft, result);
        result = Rectangle.getClosestPoint(insidePoint.getInhomX(), insidePoint.getInhomY(), center, width, height);
        assertEquals(closestInside, result);

        // test static method with point, center and size
        result = Rectangle.getClosestPoint(pointAtLeftSide, center, width, height);
        assertEquals(new InhomogeneousPoint2D(left, pointAtLeftSide.getInhomY()), result);
        result = Rectangle.getClosestPoint(pointAtTopLeftCorner, center, width, height);
        assertEquals(result, topLeft);
        result = Rectangle.getClosestPoint(pointAtTopSide, center, width, height);
        assertEquals(new InhomogeneousPoint2D(pointAtTopSide.getInhomX(), top), result);
        result = Rectangle.getClosestPoint(pointAtTopRightCorner, center, width, height);
        assertEquals(topRight, result);
        result = Rectangle.getClosestPoint(pointAtRightSide, center, width, height);
        assertEquals(new InhomogeneousPoint2D(right, pointAtRightSide.getInhomY()), result);
        result = Rectangle.getClosestPoint(pointAtBottomRightCorner, center, width, height);
        assertEquals(bottomRight, result);
        result = Rectangle.getClosestPoint(pointAtBottomSide, center, width, height);
        assertEquals(new InhomogeneousPoint2D(pointAtBottomSide.getInhomX(), bottom), result);
        result = Rectangle.getClosestPoint(pointAtBottomLeftCorner, center, width, height);
        assertEquals(bottomLeft, result);
        result = Rectangle.getClosestPoint(insidePoint, center, width, height);
        assertEquals(closestInside, result);

        // test non-static method with point coordinates
        r = new Rectangle(left, top, right, bottom);

        result = r.getClosestPoint(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY());
        assertEquals(new InhomogeneousPoint2D(left, pointAtLeftSide.getInhomY()), result);
        result = r.getClosestPoint(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY());
        assertEquals(topLeft, result);
        result = r.getClosestPoint(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY());
        assertEquals(new InhomogeneousPoint2D(pointAtTopSide.getInhomX(), top), result);
        result = r.getClosestPoint(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY());
        assertEquals(topRight, result);
        result = r.getClosestPoint(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY());
        assertEquals(new InhomogeneousPoint2D(right, pointAtRightSide.getInhomY()), result);
        result = r.getClosestPoint(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY());
        assertEquals(bottomRight, result);
        result = r.getClosestPoint(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY());
        assertEquals(new InhomogeneousPoint2D(pointAtBottomSide.getInhomX(), bottom), result);
        result = r.getClosestPoint(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY());
        assertEquals(bottomLeft, result);
        result = r.getClosestPoint(insidePoint.getInhomX(), insidePoint.getInhomY());
        assertEquals(closestInside, result);

        // test non-static method with point coordinates
        r = new Rectangle(left, top, right, bottom);

        result = r.getClosestPoint(pointAtLeftSide);
        assertEquals(new InhomogeneousPoint2D(left, pointAtLeftSide.getInhomY()), result);
        result = r.getClosestPoint(pointAtTopLeftCorner);
        assertEquals(topLeft, result);
        result = r.getClosestPoint(pointAtTopSide);
        assertEquals(new InhomogeneousPoint2D(pointAtTopSide.getInhomX(), top), result);
        result = r.getClosestPoint(pointAtTopRightCorner);
        assertEquals(topRight, result);
        result = r.getClosestPoint(pointAtRightSide);
        assertEquals(new InhomogeneousPoint2D(right, pointAtRightSide.getInhomY()), result);
        result = r.getClosestPoint(pointAtBottomRightCorner);
        assertEquals(result, bottomRight);
        result = r.getClosestPoint(pointAtBottomSide);
        assertEquals(new InhomogeneousPoint2D(pointAtBottomSide.getInhomX(), bottom), result);
        result = r.getClosestPoint(pointAtBottomLeftCorner);
        assertEquals(bottomLeft, result);
        result = r.getClosestPoint(insidePoint);
        assertEquals(closestInside, result);
    }

    @Test
    void testIsLocusToLeftSide() {
        final var randomizer = new UniformRandomizer();
        final var centerX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var centerY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var width = randomizer.nextDouble(0, MAX_RANDOM_VALUE);
        final var height = randomizer.nextDouble(0, MAX_RANDOM_VALUE);

        final var left = centerX - 0.5 * width;
        final var right = centerX + 0.5 * width;
        final var top = centerY + 0.5 * height;
        final var bottom = centerY - 0.5 * height;

        final var topLeft = new InhomogeneousPoint2D(left, top);
        final var bottomRight = new InhomogeneousPoint2D(right, bottom);
        final var center = new InhomogeneousPoint2D(centerX, centerY);

        final var pointAtLeftSide = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5), centerY);
        final var pointAtTopLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopSide = new InhomogeneousPoint2D(
                centerX, centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtRightSide = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5), centerY);
        final var pointAtBottomRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomSide = new InhomogeneousPoint2D(
                centerX, centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var insidePoint = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(-0.5, 0.5),
                centerY + height * randomizer.nextDouble(-0.5, 0.5));

        var alpha = randomizer.nextDouble();
        final var pointAtLeftBorder = new InhomogeneousPoint2D(left, alpha * bottom + (1.0 - alpha) * top);
        alpha = randomizer.nextDouble();
        final var pointAtTopBorder = new InhomogeneousPoint2D(alpha * left + (1.0 - alpha) * right, top);
        alpha = randomizer.nextDouble();
        final var pointAtRightBorder = new InhomogeneousPoint2D(right, alpha * bottom + (1.0 - alpha) * top);
        alpha = randomizer.nextDouble();
        final var pointAtBottomBorder = new InhomogeneousPoint2D(alpha * left + (1.0 - alpha) * right, bottom);

        // test static method with point and corner coordinates and threshold
        assertFalse(Rectangle.isLocusToLeftSide(pointAtLeftSide.getInhomX(),
                pointAtLeftSide.getInhomY(), left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtTopLeftCorner.getInhomX(),
                pointAtTopLeftCorner.getInhomY(), left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtTopSide.getInhomX(),
                pointAtTopSide.getInhomY(), left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtTopRightCorner.getInhomX(),
                pointAtTopRightCorner.getInhomY(), left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtRightSide.getInhomX(),
                pointAtRightSide.getInhomY(), left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtBottomSide.getInhomX(),
                pointAtBottomSide.getInhomY(), left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(insidePoint.getInhomX(),
                insidePoint.getInhomY(), left, top, right, bottom, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isLocusToLeftSide(pointAtLeftBorder.getInhomX(),
                pointAtLeftBorder.getInhomY(), left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtTopBorder.getInhomX(),
                pointAtTopBorder.getInhomY(), left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtRightBorder.getInhomX(),
                pointAtRightBorder.getInhomY(), left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtBottomBorder.getInhomX(),
                pointAtBottomBorder.getInhomY(), left, top, right, bottom, ABSOLUTE_ERROR));

        // test static method with point, corner coordinates and threshold
        assertFalse(Rectangle.isLocusToLeftSide(pointAtLeftSide, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtTopLeftCorner, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtTopSide, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtTopRightCorner, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtRightSide, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtBottomRightCorner, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtBottomSide, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtBottomLeftCorner, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(insidePoint, left, top, right, bottom, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isLocusToLeftSide(pointAtLeftBorder, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtTopBorder, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtRightBorder, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtBottomBorder, left, top, right, bottom, ABSOLUTE_ERROR));

        // test static method with point coordinates, corners and threshold
        assertFalse(Rectangle.isLocusToLeftSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), topLeft,
                bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), topLeft,
                bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), topLeft,
                bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), topLeft,
                bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(insidePoint.getInhomX(), insidePoint.getInhomY(), topLeft, bottomRight,
                ABSOLUTE_ERROR));
        assertTrue(Rectangle.isLocusToLeftSide(pointAtLeftBorder.getInhomX(), pointAtLeftBorder.getInhomY(), topLeft,
                bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtTopBorder.getInhomX(), pointAtTopBorder.getInhomY(), topLeft,
                bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtRightBorder.getInhomX(), pointAtRightBorder.getInhomY(), topLeft,
                bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtBottomBorder.getInhomX(), pointAtBottomBorder.getInhomY(),
                topLeft, bottomRight, ABSOLUTE_ERROR));

        // test static method with point, corners and threshold
        assertFalse(Rectangle.isLocusToLeftSide(pointAtLeftSide, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtTopLeftCorner, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtTopSide, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtTopRightCorner, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtRightSide, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtBottomRightCorner, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtBottomSide, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtBottomLeftCorner, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(insidePoint, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isLocusToLeftSide(pointAtLeftBorder, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtTopBorder, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtRightBorder, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtBottomBorder, topLeft, bottomRight, ABSOLUTE_ERROR));

        // test static method with point coordinates, center, size and threshold
        assertFalse(Rectangle.isLocusToLeftSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), center, width,
                height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), center, width,
                height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), center,
                width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), center,
                width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(insidePoint.getInhomX(), insidePoint.getInhomY(), center, width, height,
                ABSOLUTE_ERROR));
        assertTrue(Rectangle.isLocusToLeftSide(pointAtLeftBorder.getInhomX(), pointAtLeftBorder.getInhomY(), center,
                width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtTopBorder.getInhomX(), pointAtTopBorder.getInhomY(), center,
                width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtRightBorder.getInhomX(), pointAtRightBorder.getInhomY(), center,
                width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtBottomBorder.getInhomX(), pointAtBottomBorder.getInhomY(),
                center, width, height, ABSOLUTE_ERROR));

        // test static method with point, center, size and threshold
        assertFalse(Rectangle.isLocusToLeftSide(pointAtLeftSide, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtTopLeftCorner, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtTopSide, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtTopRightCorner, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtRightSide, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtBottomRightCorner, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtBottomSide, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtBottomLeftCorner, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(insidePoint, center, width, height, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isLocusToLeftSide(pointAtLeftBorder, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtTopBorder, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtRightBorder, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtBottomBorder, center, width, height, ABSOLUTE_ERROR));

        // test non-static method with point coordinates and threshold
        var r = new Rectangle(left, top, right, bottom);
        assertFalse(r.isLocusToLeftSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isLocusToLeftSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                ABSOLUTE_ERROR));
        assertFalse(r.isLocusToLeftSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isLocusToLeftSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                ABSOLUTE_ERROR));
        assertFalse(r.isLocusToLeftSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isLocusToLeftSide(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY(),
                ABSOLUTE_ERROR));
        assertFalse(r.isLocusToLeftSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isLocusToLeftSide(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(),
                ABSOLUTE_ERROR));
        assertFalse(r.isLocusToLeftSide(insidePoint.getInhomX(), insidePoint.getInhomY(), ABSOLUTE_ERROR));
        assertTrue(r.isLocusToLeftSide(pointAtLeftBorder.getInhomX(), pointAtLeftBorder.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isLocusToLeftSide(pointAtTopBorder.getInhomX(), pointAtTopBorder.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isLocusToLeftSide(pointAtRightBorder.getInhomX(), pointAtRightBorder.getInhomY(),
                ABSOLUTE_ERROR));
        assertFalse(r.isLocusToLeftSide(pointAtBottomBorder.getInhomX(), pointAtBottomBorder.getInhomY(),
                ABSOLUTE_ERROR));

        // test non-static method with point and threshold
        r = new Rectangle(left, top, right, bottom);
        assertFalse(r.isLocusToLeftSide(pointAtLeftSide, ABSOLUTE_ERROR));
        assertFalse(r.isLocusToLeftSide(pointAtTopLeftCorner, ABSOLUTE_ERROR));
        assertFalse(r.isLocusToLeftSide(pointAtTopSide, ABSOLUTE_ERROR));
        assertFalse(r.isLocusToLeftSide(pointAtTopRightCorner, ABSOLUTE_ERROR));
        assertFalse(r.isLocusToLeftSide(pointAtRightSide, ABSOLUTE_ERROR));
        assertFalse(r.isLocusToLeftSide(pointAtBottomRightCorner, ABSOLUTE_ERROR));
        assertFalse(r.isLocusToLeftSide(pointAtBottomSide, ABSOLUTE_ERROR));
        assertFalse(r.isLocusToLeftSide(pointAtBottomLeftCorner, ABSOLUTE_ERROR));
        assertFalse(r.isLocusToLeftSide(insidePoint, ABSOLUTE_ERROR));
        assertTrue(r.isLocusToLeftSide(pointAtLeftBorder, ABSOLUTE_ERROR));
        assertFalse(r.isLocusToLeftSide(pointAtTopBorder, ABSOLUTE_ERROR));
        assertFalse(r.isLocusToLeftSide(pointAtRightBorder, ABSOLUTE_ERROR));
        assertFalse(r.isLocusToLeftSide(pointAtBottomBorder, ABSOLUTE_ERROR));

        // test without threshold

        // test static method with point and corner coordinates and no threshold
        assertFalse(Rectangle.isLocusToLeftSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), left, top,
                right, bottom));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                left, top, right, bottom));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), left, top,
                right, bottom));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                left, top, right, bottom));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), left, top,
                right, bottom));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), left, top, right, bottom));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), left,
                top, right, bottom));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), left, top, right, bottom));
        assertFalse(Rectangle.isLocusToLeftSide(insidePoint.getInhomX(), insidePoint.getInhomY(), left, top, right,
                bottom));
        assertTrue(Rectangle.isLocusToLeftSide(pointAtLeftBorder.getInhomX(), pointAtLeftBorder.getInhomY(), left, top,
                right, bottom));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtTopBorder.getInhomX(), pointAtTopBorder.getInhomY(), left, top,
                right, bottom));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtRightBorder.getInhomX(), pointAtRightBorder.getInhomY(), left,
                top, right, bottom));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtBottomBorder.getInhomX(), pointAtBottomBorder.getInhomY(), left,
                top, right, bottom));

        // test static method with point, corner coordinates and threshold
        assertFalse(Rectangle.isLocusToLeftSide(pointAtLeftSide, left, top, right, bottom));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtTopLeftCorner, left, top, right, bottom));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtTopSide, left, top, right, bottom));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtTopRightCorner, left, top, right, bottom));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtRightSide, left, top, right, bottom));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtBottomRightCorner, left, top, right, bottom));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtBottomSide, left, top, right, bottom));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtBottomLeftCorner, left, top, right, bottom));
        assertFalse(Rectangle.isLocusToLeftSide(insidePoint, left, top, right, bottom));
        assertTrue(Rectangle.isLocusToLeftSide(pointAtLeftBorder, left, top, right, bottom));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtTopBorder, left, top, right, bottom));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtRightBorder, left, top, right, bottom));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtBottomBorder, left, top, right, bottom));

        // test static method with point coordinates, corners and threshold
        assertFalse(Rectangle.isLocusToLeftSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), topLeft,
                bottomRight));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), topLeft,
                bottomRight));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), topLeft,
                bottomRight));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), topLeft,
                bottomRight));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToLeftSide(insidePoint.getInhomX(), insidePoint.getInhomY(), topLeft,
                bottomRight));
        assertTrue(Rectangle.isLocusToLeftSide(pointAtLeftBorder.getInhomX(), pointAtLeftBorder.getInhomY(), topLeft,
                bottomRight));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtTopBorder.getInhomX(), pointAtTopBorder.getInhomY(), topLeft,
                bottomRight));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtRightBorder.getInhomX(), pointAtRightBorder.getInhomY(),
                topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtBottomBorder.getInhomX(), pointAtBottomBorder.getInhomY(),
                topLeft, bottomRight));

        // test static method with point, corners and threshold
        assertFalse(Rectangle.isLocusToLeftSide(pointAtLeftSide, topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtTopLeftCorner, topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtTopSide, topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtTopRightCorner, topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtRightSide, topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtBottomRightCorner, topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtBottomSide, topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtBottomLeftCorner, topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToLeftSide(insidePoint, topLeft, bottomRight));
        assertTrue(Rectangle.isLocusToLeftSide(pointAtLeftBorder, topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtTopBorder, topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtRightBorder, topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtBottomBorder, topLeft, bottomRight));

        // test static method with point coordinates, center, size and threshold
        assertFalse(Rectangle.isLocusToLeftSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), center, width,
                height));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                center, width, height));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), center, width,
                height));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                center, width, height));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), center,
                width, height));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), center, width, height));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), center,
                width, height));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), center, width, height));
        assertFalse(Rectangle.isLocusToLeftSide(insidePoint.getInhomX(), insidePoint.getInhomY(), center, width,
                height));
        assertTrue(Rectangle.isLocusToLeftSide(pointAtLeftBorder.getInhomX(), pointAtLeftBorder.getInhomY(), center,
                width, height));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtTopBorder.getInhomX(), pointAtTopBorder.getInhomY(), center,
                width, height));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtRightBorder.getInhomX(), pointAtRightBorder.getInhomY(), center,
                width, height));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtBottomBorder.getInhomX(), pointAtBottomBorder.getInhomY(),
                center, width, height));

        // test static method with point, center, size and threshold
        assertFalse(Rectangle.isLocusToLeftSide(pointAtLeftSide, center, width, height));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtTopLeftCorner, center, width, height));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtTopSide, center, width, height));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtTopRightCorner, center, width, height));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtRightSide, center, width, height));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtBottomRightCorner, center, width, height));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtBottomSide, center, width, height));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtBottomLeftCorner, center, width, height));
        assertFalse(Rectangle.isLocusToLeftSide(insidePoint, center, width, height));
        assertTrue(Rectangle.isLocusToLeftSide(pointAtLeftBorder, center, width, height));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtTopBorder, center, width, height));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtRightBorder, center, width, height));
        assertFalse(Rectangle.isLocusToLeftSide(pointAtBottomBorder, center, width, height));

        // test non-static method with point coordinates and threshold
        r = new Rectangle(left, top, right, bottom);
        assertFalse(r.isLocusToLeftSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY()));
        assertFalse(r.isLocusToLeftSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY()));
        assertFalse(r.isLocusToLeftSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY()));
        assertFalse(r.isLocusToLeftSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY()));
        assertFalse(r.isLocusToLeftSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY()));
        assertFalse(r.isLocusToLeftSide(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY()));
        assertFalse(r.isLocusToLeftSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY()));
        assertFalse(r.isLocusToLeftSide(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY()));
        assertFalse(r.isLocusToLeftSide(insidePoint.getInhomX(), insidePoint.getInhomY()));
        assertTrue(r.isLocusToLeftSide(pointAtLeftBorder.getInhomX(), pointAtLeftBorder.getInhomY()));
        assertFalse(r.isLocusToLeftSide(pointAtTopBorder.getInhomX(), pointAtTopBorder.getInhomY()));
        assertFalse(r.isLocusToLeftSide(pointAtRightBorder.getInhomX(), pointAtRightBorder.getInhomY()));
        assertFalse(r.isLocusToLeftSide(pointAtBottomBorder.getInhomX(), pointAtBottomBorder.getInhomY()));

        // test non-static method with point and threshold
        r = new Rectangle(left, top, right, bottom);
        assertFalse(r.isLocusToLeftSide(pointAtLeftSide));
        assertFalse(r.isLocusToLeftSide(pointAtTopLeftCorner));
        assertFalse(r.isLocusToLeftSide(pointAtTopSide));
        assertFalse(r.isLocusToLeftSide(pointAtTopRightCorner));
        assertFalse(r.isLocusToLeftSide(pointAtRightSide));
        assertFalse(r.isLocusToLeftSide(pointAtBottomRightCorner));
        assertFalse(r.isLocusToLeftSide(pointAtBottomSide));
        assertFalse(r.isLocusToLeftSide(pointAtBottomLeftCorner));
        assertFalse(r.isLocusToLeftSide(insidePoint));
        assertTrue(r.isLocusToLeftSide(pointAtLeftBorder));
        assertFalse(r.isLocusToLeftSide(pointAtTopBorder));
        assertFalse(r.isLocusToLeftSide(pointAtRightBorder));
        assertFalse(r.isLocusToLeftSide(pointAtBottomBorder));
    }

    @Test
    void testIsLocusToTopSide() {
        final var randomizer = new UniformRandomizer();
        final var centerX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var centerY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var width = randomizer.nextDouble(0, MAX_RANDOM_VALUE);
        final var height = randomizer.nextDouble(0, MAX_RANDOM_VALUE);

        final var left = centerX - 0.5 * width;
        final var right = centerX + 0.5 * width;
        final var top = centerY + 0.5 * height;
        final var bottom = centerY - 0.5 * height;

        final var topLeft = new InhomogeneousPoint2D(left, top);
        final var bottomRight = new InhomogeneousPoint2D(right, bottom);
        final var center = new InhomogeneousPoint2D(centerX, centerY);

        final var pointAtLeftSide = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5), centerY);
        final var pointAtTopLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopSide = new InhomogeneousPoint2D(
                centerX, centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtRightSide = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5), centerY);
        final var pointAtBottomRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomSide = new InhomogeneousPoint2D(
                centerX, centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var insidePoint = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(-0.5, 0.5),
                centerY + height * randomizer.nextDouble(-0.5, 0.5));

        var alpha = randomizer.nextDouble();
        final var pointAtLeftBorder = new InhomogeneousPoint2D(left,
                alpha * bottom + (1.0 - alpha) * top);
        alpha = randomizer.nextDouble();
        var pointAtTopBorder = new InhomogeneousPoint2D(alpha * left + (1.0 - alpha) * right, top);
        alpha = randomizer.nextDouble();
        var pointAtRightBorder = new InhomogeneousPoint2D(right, alpha * bottom + (1.0 - alpha) * top);
        alpha = randomizer.nextDouble();
        var pointAtBottomBorder = new InhomogeneousPoint2D(alpha * left + (1.0 - alpha) * right, bottom);

        // test static method with point and corner coordinates and threshold
        assertFalse(Rectangle.isLocusToTopSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), left, top,
                right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), left, top,
                right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), left, top,
                right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), left, top,
                right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(),
                left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(insidePoint.getInhomX(), insidePoint.getInhomY(), left, top, right,
                bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtLeftBorder.getInhomX(), pointAtLeftBorder.getInhomY(), left, top,
                right, bottom, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isLocusToTopSide(pointAtTopBorder.getInhomX(), pointAtTopBorder.getInhomY(), left, top,
                right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtRightBorder.getInhomX(), pointAtRightBorder.getInhomY(), left,
                top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtBottomBorder.getInhomX(), pointAtBottomBorder.getInhomY(), left,
                top, right, bottom, ABSOLUTE_ERROR));

        // test static method with point, corner coordinates and threshold
        assertFalse(Rectangle.isLocusToTopSide(pointAtLeftSide, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtTopLeftCorner, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtTopSide, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtTopRightCorner, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtRightSide, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtBottomRightCorner, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtBottomSide, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtBottomLeftCorner, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(insidePoint, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtLeftBorder, left, top, right, bottom, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isLocusToTopSide(pointAtTopBorder, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtRightBorder, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtBottomBorder, left, top, right, bottom, ABSOLUTE_ERROR));

        // test static method with point coordinates, corners and threshold
        assertFalse(Rectangle.isLocusToTopSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), topLeft,
                bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), topLeft,
                bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), topLeft,
                bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), topLeft,
                bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(),
                topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(insidePoint.getInhomX(), insidePoint.getInhomY(), topLeft, bottomRight,
                ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtLeftBorder.getInhomX(), pointAtLeftBorder.getInhomY(), topLeft,
                bottomRight, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isLocusToTopSide(pointAtTopBorder.getInhomX(), pointAtTopBorder.getInhomY(), topLeft,
                bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtRightBorder.getInhomX(), pointAtRightBorder.getInhomY(), topLeft,
                bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtBottomBorder.getInhomX(), pointAtBottomBorder.getInhomY(),
                topLeft, bottomRight, ABSOLUTE_ERROR));

        // test static method with point, corners and threshold
        assertFalse(Rectangle.isLocusToTopSide(pointAtLeftSide, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtTopLeftCorner, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtTopSide, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtTopRightCorner, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtRightSide, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtBottomRightCorner, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtBottomSide, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtBottomLeftCorner, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(insidePoint, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtLeftBorder, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isLocusToTopSide(pointAtTopBorder, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtRightBorder, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtBottomBorder, topLeft, bottomRight, ABSOLUTE_ERROR));

        // test static method with point coordinates, center, size and threshold
        assertFalse(Rectangle.isLocusToTopSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), center, width,
                height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), center, width,
                height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), center,
                width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), center,
                width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(),
                center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(insidePoint.getInhomX(), insidePoint.getInhomY(), center, width, height,
                ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtLeftBorder.getInhomX(), pointAtLeftBorder.getInhomY(), center,
                width, height, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isLocusToTopSide(pointAtTopBorder.getInhomX(), pointAtTopBorder.getInhomY(), center, width,
                height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtRightBorder.getInhomX(), pointAtRightBorder.getInhomY(), center,
                width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtBottomBorder.getInhomX(), pointAtBottomBorder.getInhomY(), center,
                width, height, ABSOLUTE_ERROR));

        // test static method with point, center, size and threshold
        assertFalse(Rectangle.isLocusToTopSide(pointAtLeftSide, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtTopLeftCorner, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtTopSide, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtTopRightCorner, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtRightSide, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtBottomRightCorner, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtBottomSide, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtBottomLeftCorner, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(insidePoint, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtLeftBorder, center, width, height, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isLocusToTopSide(pointAtTopBorder, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtRightBorder, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToTopSide(pointAtBottomBorder, center, width, height, ABSOLUTE_ERROR));

        // test non-static method with point coordinates and threshold
        var r = new Rectangle(left, top, right, bottom);
        assertFalse(r.isLocusToTopSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isLocusToTopSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                ABSOLUTE_ERROR));
        assertFalse(r.isLocusToTopSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isLocusToTopSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                ABSOLUTE_ERROR));
        assertFalse(r.isLocusToTopSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isLocusToTopSide(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY(),
                ABSOLUTE_ERROR));
        assertFalse(r.isLocusToTopSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isLocusToTopSide(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(),
                ABSOLUTE_ERROR));
        assertFalse(r.isLocusToTopSide(insidePoint.getInhomX(), insidePoint.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isLocusToTopSide(pointAtLeftBorder.getInhomX(), pointAtLeftBorder.getInhomY(), ABSOLUTE_ERROR));
        assertTrue(r.isLocusToTopSide(pointAtTopBorder.getInhomX(), pointAtTopBorder.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isLocusToTopSide(pointAtRightBorder.getInhomX(), pointAtRightBorder.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isLocusToTopSide(pointAtBottomBorder.getInhomX(), pointAtBottomBorder.getInhomY(),
                ABSOLUTE_ERROR));

        // test non-static method with point and threshold
        r = new Rectangle(left, top, right, bottom);
        assertFalse(r.isLocusToTopSide(pointAtLeftSide, ABSOLUTE_ERROR));
        assertFalse(r.isLocusToTopSide(pointAtTopLeftCorner, ABSOLUTE_ERROR));
        assertFalse(r.isLocusToTopSide(pointAtTopSide, ABSOLUTE_ERROR));
        assertFalse(r.isLocusToTopSide(pointAtTopRightCorner, ABSOLUTE_ERROR));
        assertFalse(r.isLocusToTopSide(pointAtRightSide, ABSOLUTE_ERROR));
        assertFalse(r.isLocusToTopSide(pointAtBottomRightCorner, ABSOLUTE_ERROR));
        assertFalse(r.isLocusToTopSide(pointAtBottomSide, ABSOLUTE_ERROR));
        assertFalse(r.isLocusToTopSide(pointAtBottomLeftCorner, ABSOLUTE_ERROR));
        assertFalse(r.isLocusToTopSide(insidePoint, ABSOLUTE_ERROR));
        assertFalse(r.isLocusToTopSide(pointAtLeftBorder, ABSOLUTE_ERROR));
        assertTrue(r.isLocusToTopSide(pointAtTopBorder, ABSOLUTE_ERROR));
        assertFalse(r.isLocusToTopSide(pointAtRightBorder, ABSOLUTE_ERROR));
        assertFalse(r.isLocusToTopSide(pointAtBottomBorder, ABSOLUTE_ERROR));

        // test without threshold

        // test static method with point and corner coordinates and no threshold
        assertFalse(Rectangle.isLocusToTopSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), left, top,
                right, bottom));
        assertFalse(Rectangle.isLocusToTopSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(), left,
                top, right, bottom));
        assertFalse(Rectangle.isLocusToTopSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), left, top, right,
                bottom));
        assertFalse(Rectangle.isLocusToTopSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                left, top, right, bottom));
        assertFalse(Rectangle.isLocusToTopSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), left, top,
                right, bottom));
        assertFalse(Rectangle.isLocusToTopSide(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), left, top, right, bottom));
        assertFalse(Rectangle.isLocusToTopSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), left, top,
                right, bottom));
        assertFalse(Rectangle.isLocusToTopSide(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(),
                left, top, right, bottom));
        assertFalse(Rectangle.isLocusToTopSide(insidePoint.getInhomX(), insidePoint.getInhomY(), left, top, right,
                bottom));
        assertFalse(Rectangle.isLocusToTopSide(pointAtLeftBorder.getInhomX(), pointAtLeftBorder.getInhomY(), left, top,
                right, bottom));
        assertTrue(Rectangle.isLocusToTopSide(pointAtTopBorder.getInhomX(), pointAtTopBorder.getInhomY(), left, top,
                right, bottom));
        assertFalse(Rectangle.isLocusToTopSide(pointAtRightBorder.getInhomX(), pointAtRightBorder.getInhomY(), left,
                top, right, bottom));
        assertFalse(Rectangle.isLocusToTopSide(pointAtBottomBorder.getInhomX(), pointAtBottomBorder.getInhomY(), left,
                top, right, bottom));

        // test static method with point, corner coordinates and threshold
        assertFalse(Rectangle.isLocusToTopSide(pointAtLeftSide, left, top, right, bottom));
        assertFalse(Rectangle.isLocusToTopSide(pointAtTopLeftCorner, left, top, right, bottom));
        assertFalse(Rectangle.isLocusToTopSide(pointAtTopSide, left, top, right, bottom));
        assertFalse(Rectangle.isLocusToTopSide(pointAtTopRightCorner, left, top, right, bottom));
        assertFalse(Rectangle.isLocusToTopSide(pointAtRightSide, left, top, right, bottom));
        assertFalse(Rectangle.isLocusToTopSide(pointAtBottomRightCorner, left, top, right, bottom));
        assertFalse(Rectangle.isLocusToTopSide(pointAtBottomSide, left, top, right, bottom));
        assertFalse(Rectangle.isLocusToTopSide(pointAtBottomLeftCorner, left, top, right, bottom));
        assertFalse(Rectangle.isLocusToTopSide(insidePoint, left, top, right, bottom));
        assertFalse(Rectangle.isLocusToTopSide(pointAtLeftBorder, left, top, right, bottom));
        assertTrue(Rectangle.isLocusToTopSide(pointAtTopBorder, left, top, right, bottom));
        assertFalse(Rectangle.isLocusToTopSide(pointAtRightBorder, left, top, right, bottom));
        assertFalse(Rectangle.isLocusToTopSide(pointAtBottomBorder, left, top, right, bottom));

        // test static method with point coordinates, corners and threshold
        assertFalse(Rectangle.isLocusToTopSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), topLeft,
                bottomRight));
        assertFalse(Rectangle.isLocusToTopSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToTopSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), topLeft,
                bottomRight));
        assertFalse(Rectangle.isLocusToTopSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToTopSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), topLeft,
                bottomRight));
        assertFalse(Rectangle.isLocusToTopSide(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToTopSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), topLeft,
                bottomRight));
        assertFalse(Rectangle.isLocusToTopSide(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(),
                topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToTopSide(insidePoint.getInhomX(), insidePoint.getInhomY(), topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToTopSide(pointAtLeftBorder.getInhomX(), pointAtLeftBorder.getInhomY(), topLeft,
                bottomRight));
        assertTrue(Rectangle.isLocusToTopSide(pointAtTopBorder.getInhomX(), pointAtTopBorder.getInhomY(), topLeft,
                bottomRight));
        assertFalse(Rectangle.isLocusToTopSide(pointAtRightBorder.getInhomX(), pointAtRightBorder.getInhomY(), topLeft,
                bottomRight));
        assertFalse(Rectangle.isLocusToTopSide(pointAtBottomBorder.getInhomX(), pointAtBottomBorder.getInhomY(),
                topLeft, bottomRight));

        // test static method with point, corners and threshold
        assertFalse(Rectangle.isLocusToTopSide(pointAtLeftSide, topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToTopSide(pointAtTopLeftCorner, topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToTopSide(pointAtTopSide, topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToTopSide(pointAtTopRightCorner, topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToTopSide(pointAtRightSide, topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToTopSide(pointAtBottomRightCorner, topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToTopSide(pointAtBottomSide, topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToTopSide(pointAtBottomLeftCorner, topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToTopSide(insidePoint, topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToTopSide(pointAtLeftBorder, topLeft, bottomRight));
        assertTrue(Rectangle.isLocusToTopSide(pointAtTopBorder, topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToTopSide(pointAtRightBorder, topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToTopSide(pointAtBottomBorder, topLeft, bottomRight));

        // test static method with point coordinates, center, size and threshold
        assertFalse(Rectangle.isLocusToTopSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), center, width,
                height));
        assertFalse(Rectangle.isLocusToTopSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                center, width, height));
        assertFalse(Rectangle.isLocusToTopSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), center, width,
                height));
        assertFalse(Rectangle.isLocusToTopSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                center, width, height));
        assertFalse(Rectangle.isLocusToTopSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), center,
                width, height));
        assertFalse(Rectangle.isLocusToTopSide(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), center, width, height));
        assertFalse(Rectangle.isLocusToTopSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), center,
                width, height));
        assertFalse(Rectangle.isLocusToTopSide(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(),
                center, width, height));
        assertFalse(Rectangle.isLocusToTopSide(insidePoint.getInhomX(), insidePoint.getInhomY(), center, width,
                height));
        assertFalse(Rectangle.isLocusToTopSide(pointAtLeftBorder.getInhomX(), pointAtLeftBorder.getInhomY(), center,
                width, height));
        assertTrue(Rectangle.isLocusToTopSide(pointAtTopBorder.getInhomX(), pointAtTopBorder.getInhomY(), center, width,
                height));
        assertFalse(Rectangle.isLocusToTopSide(pointAtRightBorder.getInhomX(), pointAtRightBorder.getInhomY(), center,
                width, height));
        assertFalse(Rectangle.isLocusToTopSide(pointAtBottomBorder.getInhomX(), pointAtBottomBorder.getInhomY(), center,
                width, height));

        // test static method with point, center, size and threshold
        assertFalse(Rectangle.isLocusToTopSide(pointAtLeftSide, center, width, height));
        assertFalse(Rectangle.isLocusToTopSide(pointAtTopLeftCorner, center, width, height));
        assertFalse(Rectangle.isLocusToTopSide(pointAtTopSide, center, width, height));
        assertFalse(Rectangle.isLocusToTopSide(pointAtTopRightCorner, center, width, height));
        assertFalse(Rectangle.isLocusToTopSide(pointAtRightSide, center, width, height));
        assertFalse(Rectangle.isLocusToTopSide(pointAtBottomRightCorner, center, width, height));
        assertFalse(Rectangle.isLocusToTopSide(pointAtBottomSide, center, width, height));
        assertFalse(Rectangle.isLocusToTopSide(pointAtBottomLeftCorner, center, width, height));
        assertFalse(Rectangle.isLocusToTopSide(insidePoint, center, width, height));
        assertFalse(Rectangle.isLocusToTopSide(pointAtLeftBorder, center, width, height));
        assertTrue(Rectangle.isLocusToTopSide(pointAtTopBorder, center, width, height));
        assertFalse(Rectangle.isLocusToTopSide(pointAtRightBorder, center, width, height));
        assertFalse(Rectangle.isLocusToTopSide(pointAtBottomBorder, center, width, height));

        // test non-static method with point coordinates and threshold
        r = new Rectangle(left, top, right, bottom);
        assertFalse(r.isLocusToTopSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY()));
        assertFalse(r.isLocusToTopSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY()));
        assertFalse(r.isLocusToTopSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY()));
        assertFalse(r.isLocusToTopSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY()));
        assertFalse(r.isLocusToTopSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY()));
        assertFalse(r.isLocusToTopSide(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY()));
        assertFalse(r.isLocusToTopSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY()));
        assertFalse(r.isLocusToTopSide(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY()));
        assertFalse(r.isLocusToTopSide(insidePoint.getInhomX(), insidePoint.getInhomY()));
        assertFalse(r.isLocusToTopSide(pointAtLeftBorder.getInhomX(), pointAtLeftBorder.getInhomY()));
        assertTrue(r.isLocusToTopSide(pointAtTopBorder.getInhomX(), pointAtTopBorder.getInhomY()));
        assertFalse(r.isLocusToTopSide(pointAtRightBorder.getInhomX(), pointAtRightBorder.getInhomY()));
        assertFalse(r.isLocusToTopSide(pointAtBottomBorder.getInhomX(), pointAtBottomBorder.getInhomY()));

        // test non-static method with point and threshold
        r = new Rectangle(left, top, right, bottom);
        assertFalse(r.isLocusToTopSide(pointAtLeftSide));
        assertFalse(r.isLocusToTopSide(pointAtTopLeftCorner));
        assertFalse(r.isLocusToTopSide(pointAtTopSide));
        assertFalse(r.isLocusToTopSide(pointAtTopRightCorner));
        assertFalse(r.isLocusToTopSide(pointAtRightSide));
        assertFalse(r.isLocusToTopSide(pointAtBottomRightCorner));
        assertFalse(r.isLocusToTopSide(pointAtBottomSide));
        assertFalse(r.isLocusToTopSide(pointAtBottomLeftCorner));
        assertFalse(r.isLocusToTopSide(insidePoint));
        assertFalse(r.isLocusToTopSide(pointAtLeftBorder));
        assertTrue(r.isLocusToTopSide(pointAtTopBorder));
        assertFalse(r.isLocusToTopSide(pointAtRightBorder));
        assertFalse(r.isLocusToTopSide(pointAtBottomBorder));
    }

    @Test
    void testIsLocusToRightSide() {
        final var randomizer = new UniformRandomizer();
        final var centerX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var centerY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var width = randomizer.nextDouble(0, MAX_RANDOM_VALUE);
        final var height = randomizer.nextDouble(0, MAX_RANDOM_VALUE);

        final var left = centerX - 0.5 * width;
        final var right = centerX + 0.5 * width;
        final var top = centerY + 0.5 * height;
        final var bottom = centerY - 0.5 * height;

        final var topLeft = new InhomogeneousPoint2D(left, top);
        final var bottomRight = new InhomogeneousPoint2D(right, bottom);
        final var center = new InhomogeneousPoint2D(centerX, centerY);

        final var pointAtLeftSide = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5), centerY);
        final var pointAtTopLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopSide = new InhomogeneousPoint2D(
                centerX, centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtRightSide = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5), centerY);
        final var pointAtBottomRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomSide = new InhomogeneousPoint2D(
                centerX, centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var insidePoint = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(-0.5, 0.5),
                centerY + height * randomizer.nextDouble(-0.5, 0.5));

        var alpha = randomizer.nextDouble();
        final var pointAtLeftBorder = new InhomogeneousPoint2D(left, alpha * bottom + (1.0 - alpha) * top);
        alpha = randomizer.nextDouble();
        final var pointAtTopBorder = new InhomogeneousPoint2D(alpha * left + (1.0 - alpha) * right, top);
        alpha = randomizer.nextDouble();
        final var pointAtRightBorder = new InhomogeneousPoint2D(right, alpha * bottom + (1.0 - alpha) * top);
        alpha = randomizer.nextDouble();
        final var pointAtBottomBorder = new InhomogeneousPoint2D(alpha * left + (1.0 - alpha) * right, bottom);

        // test static method with point and corner coordinates and threshold
        assertFalse(Rectangle.isLocusToRightSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), left, top,
                right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), left, top,
                right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), left, top,
                right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), left,
                top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(insidePoint.getInhomX(), insidePoint.getInhomY(), left, top, right,
                bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtLeftBorder.getInhomX(), pointAtLeftBorder.getInhomY(), left,
                top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtTopBorder.getInhomX(), pointAtTopBorder.getInhomY(), left, top,
                right, bottom, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isLocusToRightSide(pointAtRightBorder.getInhomX(), pointAtRightBorder.getInhomY(), left,
                top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtBottomBorder.getInhomX(), pointAtBottomBorder.getInhomY(), left,
                top, right, bottom, ABSOLUTE_ERROR));

        // test static method with point, corner coordinates and threshold
        assertFalse(Rectangle.isLocusToRightSide(pointAtLeftSide, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtTopLeftCorner, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtTopSide, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtTopRightCorner, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtRightSide, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtBottomRightCorner, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtBottomSide, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtBottomLeftCorner, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(insidePoint, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtLeftBorder, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtTopBorder, left, top, right, bottom, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isLocusToRightSide(pointAtRightBorder, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtBottomBorder, left, top, right, bottom, ABSOLUTE_ERROR));

        // test static method with point coordinates, corners and threshold
        assertFalse(Rectangle.isLocusToRightSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), topLeft,
                bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), topLeft,
                bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), topLeft,
                bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), topLeft,
                bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(insidePoint.getInhomX(), insidePoint.getInhomY(), topLeft, bottomRight,
                ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtLeftBorder.getInhomX(), pointAtLeftBorder.getInhomY(), topLeft,
                bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtTopBorder.getInhomX(), pointAtTopBorder.getInhomY(), topLeft,
                bottomRight, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isLocusToRightSide(pointAtRightBorder.getInhomX(), pointAtRightBorder.getInhomY(), topLeft,
                bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtBottomBorder.getInhomX(), pointAtBottomBorder.getInhomY(),
                topLeft, bottomRight, ABSOLUTE_ERROR));

        // test static method with point, corners and threshold
        assertFalse(Rectangle.isLocusToRightSide(pointAtLeftSide, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtTopLeftCorner, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtTopSide, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtTopRightCorner, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtRightSide, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtBottomRightCorner, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtBottomSide, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtBottomLeftCorner, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(insidePoint, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtLeftBorder, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtTopBorder, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isLocusToRightSide(pointAtRightBorder, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtBottomBorder, topLeft, bottomRight, ABSOLUTE_ERROR));

        // test static method with point coordinates, center, size and threshold
        assertFalse(Rectangle.isLocusToRightSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), center,
                width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), center, width,
                height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), center,
                width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), center,
                width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(insidePoint.getInhomX(), insidePoint.getInhomY(), center, width,
                height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtLeftBorder.getInhomX(), pointAtLeftBorder.getInhomY(), center,
                width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtTopBorder.getInhomX(), pointAtTopBorder.getInhomY(), center,
                width, height, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isLocusToRightSide(pointAtRightBorder.getInhomX(), pointAtRightBorder.getInhomY(), center,
                width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtBottomBorder.getInhomX(), pointAtBottomBorder.getInhomY(),
                center, width, height, ABSOLUTE_ERROR));

        // test static method with point, center, size and threshold
        assertFalse(Rectangle.isLocusToRightSide(pointAtLeftSide, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtTopLeftCorner, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtTopSide, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtTopRightCorner, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtRightSide, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtBottomRightCorner, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtBottomSide, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtBottomLeftCorner, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(insidePoint, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtLeftBorder, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtTopBorder, center, width, height, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isLocusToRightSide(pointAtRightBorder, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToRightSide(pointAtBottomBorder, center, width, height, ABSOLUTE_ERROR));

        // test non-static method with point coordinates and threshold
        var r = new Rectangle(left, top, right, bottom);
        assertFalse(r.isLocusToRightSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isLocusToRightSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                ABSOLUTE_ERROR));
        assertFalse(r.isLocusToRightSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isLocusToRightSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                ABSOLUTE_ERROR));
        assertFalse(r.isLocusToRightSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isLocusToRightSide(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY(),
                ABSOLUTE_ERROR));
        assertFalse(r.isLocusToRightSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isLocusToRightSide(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(),
                ABSOLUTE_ERROR));
        assertFalse(r.isLocusToRightSide(insidePoint.getInhomX(), insidePoint.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isLocusToRightSide(pointAtLeftBorder.getInhomX(), pointAtLeftBorder.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isLocusToRightSide(pointAtTopBorder.getInhomX(), pointAtTopBorder.getInhomY(), ABSOLUTE_ERROR));
        assertTrue(r.isLocusToRightSide(pointAtRightBorder.getInhomX(), pointAtRightBorder.getInhomY(),
                ABSOLUTE_ERROR));
        assertFalse(r.isLocusToRightSide(pointAtBottomBorder.getInhomX(), pointAtBottomBorder.getInhomY(),
                ABSOLUTE_ERROR));

        // test non-static method with point and threshold
        r = new Rectangle(left, top, right, bottom);
        assertFalse(r.isLocusToRightSide(pointAtLeftSide, ABSOLUTE_ERROR));
        assertFalse(r.isLocusToRightSide(pointAtTopLeftCorner, ABSOLUTE_ERROR));
        assertFalse(r.isLocusToRightSide(pointAtTopSide, ABSOLUTE_ERROR));
        assertFalse(r.isLocusToRightSide(pointAtTopRightCorner, ABSOLUTE_ERROR));
        assertFalse(r.isLocusToRightSide(pointAtRightSide, ABSOLUTE_ERROR));
        assertFalse(r.isLocusToRightSide(pointAtBottomRightCorner, ABSOLUTE_ERROR));
        assertFalse(r.isLocusToRightSide(pointAtBottomSide, ABSOLUTE_ERROR));
        assertFalse(r.isLocusToRightSide(pointAtBottomLeftCorner, ABSOLUTE_ERROR));
        assertFalse(r.isLocusToRightSide(insidePoint, ABSOLUTE_ERROR));
        assertFalse(r.isLocusToRightSide(pointAtLeftBorder, ABSOLUTE_ERROR));
        assertFalse(r.isLocusToRightSide(pointAtTopBorder, ABSOLUTE_ERROR));
        assertTrue(r.isLocusToRightSide(pointAtRightBorder, ABSOLUTE_ERROR));
        assertFalse(r.isLocusToRightSide(pointAtBottomBorder, ABSOLUTE_ERROR));

        // test without threshold

        // test static method with point and corner coordinates and no threshold
        assertFalse(Rectangle.isLocusToRightSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), left, top,
                right, bottom));
        assertFalse(Rectangle.isLocusToRightSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                left, top, right, bottom));
        assertFalse(Rectangle.isLocusToRightSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), left, top,
                right, bottom));
        assertFalse(Rectangle.isLocusToRightSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                left, top, right, bottom));
        assertFalse(Rectangle.isLocusToRightSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), left, top,
                right, bottom));
        assertFalse(Rectangle.isLocusToRightSide(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), left, top, right, bottom));
        assertFalse(Rectangle.isLocusToRightSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), left,
                top, right, bottom));
        assertFalse(Rectangle.isLocusToRightSide(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), left, top, right, bottom));
        assertFalse(Rectangle.isLocusToRightSide(insidePoint.getInhomX(), insidePoint.getInhomY(), left, top, right,
                bottom));
        assertFalse(Rectangle.isLocusToRightSide(pointAtLeftBorder.getInhomX(), pointAtLeftBorder.getInhomY(), left,
                top, right, bottom));
        assertFalse(Rectangle.isLocusToRightSide(pointAtTopBorder.getInhomX(), pointAtTopBorder.getInhomY(), left, top,
                right, bottom));
        assertTrue(Rectangle.isLocusToRightSide(pointAtRightBorder.getInhomX(), pointAtRightBorder.getInhomY(), left,
                top, right, bottom));
        assertFalse(Rectangle.isLocusToRightSide(pointAtBottomBorder.getInhomX(), pointAtBottomBorder.getInhomY(),
                left, top, right, bottom));

        // test static method with point, corner coordinates and threshold
        assertFalse(Rectangle.isLocusToRightSide(pointAtLeftSide, left, top, right, bottom));
        assertFalse(Rectangle.isLocusToRightSide(pointAtTopLeftCorner, left, top, right, bottom));
        assertFalse(Rectangle.isLocusToRightSide(pointAtTopSide, left, top, right, bottom));
        assertFalse(Rectangle.isLocusToRightSide(pointAtTopRightCorner, left, top, right, bottom));
        assertFalse(Rectangle.isLocusToRightSide(pointAtRightSide, left, top, right, bottom));
        assertFalse(Rectangle.isLocusToRightSide(pointAtBottomRightCorner, left, top, right, bottom));
        assertFalse(Rectangle.isLocusToRightSide(pointAtBottomSide, left, top, right, bottom));
        assertFalse(Rectangle.isLocusToRightSide(pointAtBottomLeftCorner, left, top, right, bottom));
        assertFalse(Rectangle.isLocusToRightSide(insidePoint, left, top, right, bottom));
        assertFalse(Rectangle.isLocusToRightSide(pointAtLeftBorder, left, top, right, bottom));
        assertFalse(Rectangle.isLocusToRightSide(pointAtTopBorder, left, top, right, bottom));
        assertTrue(Rectangle.isLocusToRightSide(pointAtRightBorder, left, top, right, bottom));
        assertFalse(Rectangle.isLocusToRightSide(pointAtBottomBorder, left, top, right, bottom));

        // test static method with point coordinates, corners and threshold
        assertFalse(Rectangle.isLocusToRightSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), topLeft,
                bottomRight));
        assertFalse(Rectangle.isLocusToRightSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToRightSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), topLeft,
                bottomRight));
        assertFalse(Rectangle.isLocusToRightSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToRightSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), topLeft,
                bottomRight));
        assertFalse(Rectangle.isLocusToRightSide(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToRightSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), topLeft,
                bottomRight));
        assertFalse(Rectangle.isLocusToRightSide(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToRightSide(insidePoint.getInhomX(), insidePoint.getInhomY(), topLeft,
                bottomRight));
        assertFalse(Rectangle.isLocusToRightSide(pointAtLeftBorder.getInhomX(), pointAtLeftBorder.getInhomY(), topLeft,
                bottomRight));
        assertFalse(Rectangle.isLocusToRightSide(pointAtTopBorder.getInhomX(), pointAtTopBorder.getInhomY(), topLeft,
                bottomRight));
        assertTrue(Rectangle.isLocusToRightSide(pointAtRightBorder.getInhomX(), pointAtRightBorder.getInhomY(), topLeft,
                bottomRight));
        assertFalse(Rectangle.isLocusToRightSide(pointAtBottomBorder.getInhomX(), pointAtBottomBorder.getInhomY(),
                topLeft, bottomRight));

        // test static method with point, corners and threshold
        assertFalse(Rectangle.isLocusToRightSide(pointAtLeftSide, topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToRightSide(pointAtTopLeftCorner, topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToRightSide(pointAtTopSide, topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToRightSide(pointAtTopRightCorner, topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToRightSide(pointAtRightSide, topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToRightSide(pointAtBottomRightCorner, topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToRightSide(pointAtBottomSide, topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToRightSide(pointAtBottomLeftCorner, topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToRightSide(insidePoint, topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToRightSide(pointAtLeftBorder, topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToRightSide(pointAtTopBorder, topLeft, bottomRight));
        assertTrue(Rectangle.isLocusToRightSide(pointAtRightBorder, topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToRightSide(pointAtBottomBorder, topLeft, bottomRight));

        // test static method with point coordinates, center, size and threshold
        assertFalse(Rectangle.isLocusToRightSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), center,
                width, height));
        assertFalse(Rectangle.isLocusToRightSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                center, width, height));
        assertFalse(Rectangle.isLocusToRightSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), center, width,
                height));
        assertFalse(Rectangle.isLocusToRightSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                center, width, height));
        assertFalse(Rectangle.isLocusToRightSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), center,
                width, height));
        assertFalse(Rectangle.isLocusToRightSide(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), center, width, height));
        assertFalse(Rectangle.isLocusToRightSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), center,
                width, height));
        assertFalse(Rectangle.isLocusToRightSide(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), center, width, height));
        assertFalse(Rectangle.isLocusToRightSide(insidePoint.getInhomX(), insidePoint.getInhomY(), center, width,
                height));
        assertFalse(Rectangle.isLocusToRightSide(pointAtLeftBorder.getInhomX(), pointAtLeftBorder.getInhomY(), center,
                width, height));
        assertFalse(Rectangle.isLocusToRightSide(pointAtTopBorder.getInhomX(), pointAtTopBorder.getInhomY(), center,
                width, height));
        assertTrue(Rectangle.isLocusToRightSide(pointAtRightBorder.getInhomX(), pointAtRightBorder.getInhomY(), center,
                width, height));
        assertFalse(Rectangle.isLocusToRightSide(pointAtBottomBorder.getInhomX(), pointAtBottomBorder.getInhomY(),
                center, width, height));

        // test static method with point, center, size and threshold
        assertFalse(Rectangle.isLocusToRightSide(pointAtLeftSide, center, width, height));
        assertFalse(Rectangle.isLocusToRightSide(pointAtTopLeftCorner, center, width, height));
        assertFalse(Rectangle.isLocusToRightSide(pointAtTopSide, center, width, height));
        assertFalse(Rectangle.isLocusToRightSide(pointAtTopRightCorner, center, width, height));
        assertFalse(Rectangle.isLocusToRightSide(pointAtRightSide, center, width, height));
        assertFalse(Rectangle.isLocusToRightSide(pointAtBottomRightCorner, center, width, height));
        assertFalse(Rectangle.isLocusToRightSide(pointAtBottomSide, center, width, height));
        assertFalse(Rectangle.isLocusToRightSide(pointAtBottomLeftCorner, center, width, height));
        assertFalse(Rectangle.isLocusToRightSide(insidePoint, center, width, height));
        assertFalse(Rectangle.isLocusToRightSide(pointAtLeftBorder, center, width, height));
        assertFalse(Rectangle.isLocusToRightSide(pointAtTopBorder, center, width, height));
        assertTrue(Rectangle.isLocusToRightSide(pointAtRightBorder, center, width, height));
        assertFalse(Rectangle.isLocusToRightSide(pointAtBottomBorder, center, width, height));

        // test non-static method with point coordinates and threshold
        r = new Rectangle(left, top, right, bottom);
        assertFalse(r.isLocusToRightSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY()));
        assertFalse(r.isLocusToRightSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY()));
        assertFalse(r.isLocusToRightSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY()));
        assertFalse(r.isLocusToRightSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY()));
        assertFalse(r.isLocusToRightSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY()));
        assertFalse(r.isLocusToRightSide(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY()));
        assertFalse(r.isLocusToRightSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY()));
        assertFalse(r.isLocusToRightSide(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY()));
        assertFalse(r.isLocusToRightSide(insidePoint.getInhomX(), insidePoint.getInhomY()));
        assertFalse(r.isLocusToRightSide(pointAtLeftBorder.getInhomX(), pointAtLeftBorder.getInhomY()));
        assertFalse(r.isLocusToRightSide(pointAtTopBorder.getInhomX(), pointAtTopBorder.getInhomY()));
        assertTrue(r.isLocusToRightSide(pointAtRightBorder.getInhomX(), pointAtRightBorder.getInhomY()));
        assertFalse(r.isLocusToRightSide(pointAtBottomBorder.getInhomX(), pointAtBottomBorder.getInhomY()));

        // test non-static method with point and threshold
        r = new Rectangle(left, top, right, bottom);
        assertFalse(r.isLocusToRightSide(pointAtLeftSide));
        assertFalse(r.isLocusToRightSide(pointAtTopLeftCorner));
        assertFalse(r.isLocusToRightSide(pointAtTopSide));
        assertFalse(r.isLocusToRightSide(pointAtTopRightCorner));
        assertFalse(r.isLocusToRightSide(pointAtRightSide));
        assertFalse(r.isLocusToRightSide(pointAtBottomRightCorner));
        assertFalse(r.isLocusToRightSide(pointAtBottomSide));
        assertFalse(r.isLocusToRightSide(pointAtBottomLeftCorner));
        assertFalse(r.isLocusToRightSide(insidePoint));
        assertFalse(r.isLocusToRightSide(pointAtLeftBorder));
        assertFalse(r.isLocusToRightSide(pointAtTopBorder));
        assertTrue(r.isLocusToRightSide(pointAtRightBorder));
        assertFalse(r.isLocusToRightSide(pointAtBottomBorder));
    }

    @Test
    void testIsLocusToBottomSide() {
        final var randomizer = new UniformRandomizer();
        final var centerX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var centerY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var width = randomizer.nextDouble(0, MAX_RANDOM_VALUE);
        final var height = randomizer.nextDouble(0, MAX_RANDOM_VALUE);

        final var left = centerX - 0.5 * width;
        final var right = centerX + 0.5 * width;
        final var top = centerY + 0.5 * height;
        final var bottom = centerY - 0.5 * height;

        final var topLeft = new InhomogeneousPoint2D(left, top);
        final var bottomRight = new InhomogeneousPoint2D(right, bottom);
        final var center = new InhomogeneousPoint2D(centerX, centerY);

        final var pointAtLeftSide = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5), centerY);
        final var pointAtTopLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopSide = new InhomogeneousPoint2D(
                centerX, centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtRightSide = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5), centerY);
        final var pointAtBottomRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomSide = new InhomogeneousPoint2D(
                centerX, centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var insidePoint = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(-0.5, 0.5),
                centerY + height * randomizer.nextDouble(-0.5, 0.5));

        var alpha = randomizer.nextDouble();
        final var pointAtLeftBorder = new InhomogeneousPoint2D(left, alpha * bottom + (1.0 - alpha) * top);
        alpha = randomizer.nextDouble();
        final var pointAtTopBorder = new InhomogeneousPoint2D(alpha * left + (1.0 - alpha) * right, top);
        alpha = randomizer.nextDouble();
        final var pointAtRightBorder = new InhomogeneousPoint2D(right, alpha * bottom + (1.0 - alpha) * top);
        alpha = randomizer.nextDouble();
        final var pointAtBottomBorder = new InhomogeneousPoint2D(alpha * left + (1.0 - alpha) * right, bottom);

        // test static method with point and corner coordinates and threshold
        assertFalse(Rectangle.isLocusToBottomSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), left, top,
                right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), left, top,
                right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), left, top,
                right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), left,
                top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(insidePoint.getInhomX(), insidePoint.getInhomY(), left, top, right,
                bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtLeftBorder.getInhomX(), pointAtLeftBorder.getInhomY(), left,
                top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtTopBorder.getInhomX(), pointAtTopBorder.getInhomY(), left, top,
                right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtRightBorder.getInhomX(), pointAtRightBorder.getInhomY(), left,
                top, right, bottom, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isLocusToBottomSide(pointAtBottomBorder.getInhomX(), pointAtBottomBorder.getInhomY(), left,
                top, right, bottom, ABSOLUTE_ERROR));

        // test static method with point, corner coordinates and threshold
        assertFalse(Rectangle.isLocusToBottomSide(pointAtLeftSide, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtTopLeftCorner, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtTopSide, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtTopRightCorner, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtRightSide, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtBottomRightCorner, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtBottomSide, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtBottomLeftCorner, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(insidePoint, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtLeftBorder, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtTopBorder, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtRightBorder, left, top, right, bottom, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isLocusToBottomSide(pointAtBottomBorder, left, top, right, bottom, ABSOLUTE_ERROR));

        // test static method with point coordinates, corners and threshold
        assertFalse(Rectangle.isLocusToBottomSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), topLeft,
                bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), topLeft,
                bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), topLeft,
                bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), topLeft,
                bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(insidePoint.getInhomX(), insidePoint.getInhomY(), topLeft,
                bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtLeftBorder.getInhomX(), pointAtLeftBorder.getInhomY(), topLeft,
                bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtTopBorder.getInhomX(), pointAtTopBorder.getInhomY(), topLeft,
                bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtRightBorder.getInhomX(), pointAtRightBorder.getInhomY(),
                topLeft, bottomRight, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isLocusToBottomSide(pointAtBottomBorder.getInhomX(), pointAtBottomBorder.getInhomY(),
                topLeft, bottomRight, ABSOLUTE_ERROR));

        // test static method with point, corners and threshold
        assertFalse(Rectangle.isLocusToBottomSide(pointAtLeftSide, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtTopLeftCorner, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtTopSide, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtTopRightCorner, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtRightSide, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtBottomRightCorner, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtBottomSide, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtBottomLeftCorner, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(insidePoint, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtLeftBorder, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtTopBorder, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtRightBorder, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isLocusToBottomSide(pointAtBottomBorder, topLeft, bottomRight, ABSOLUTE_ERROR));

        // test static method with point coordinates, center, size and threshold
        assertFalse(Rectangle.isLocusToBottomSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), center,
                width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), center, width,
                height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), center,
                width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), center,
                width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(insidePoint.getInhomX(), insidePoint.getInhomY(), center, width,
                height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtLeftBorder.getInhomX(), pointAtLeftBorder.getInhomY(), center,
                width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtTopBorder.getInhomX(), pointAtTopBorder.getInhomY(), center,
                width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtRightBorder.getInhomX(), pointAtRightBorder.getInhomY(),
                center, width, height, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isLocusToBottomSide(pointAtBottomBorder.getInhomX(), pointAtBottomBorder.getInhomY(),
                center, width, height, ABSOLUTE_ERROR));

        // test static method with point, center, size and threshold
        assertFalse(Rectangle.isLocusToBottomSide(pointAtLeftSide, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtTopLeftCorner, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtTopSide, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtTopRightCorner, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtRightSide, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtBottomRightCorner, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtBottomSide, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtBottomLeftCorner, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(insidePoint, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtLeftBorder, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtTopBorder, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtRightBorder, center, width, height, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isLocusToBottomSide(pointAtBottomBorder, center, width, height, ABSOLUTE_ERROR));

        // test non-static method with point coordinates and threshold
        var r = new Rectangle(left, top, right, bottom);
        assertFalse(r.isLocusToBottomSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isLocusToBottomSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                ABSOLUTE_ERROR));
        assertFalse(r.isLocusToBottomSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isLocusToBottomSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                ABSOLUTE_ERROR));
        assertFalse(r.isLocusToBottomSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isLocusToBottomSide(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY(),
                ABSOLUTE_ERROR));
        assertFalse(r.isLocusToBottomSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(),
                ABSOLUTE_ERROR));
        assertFalse(r.isLocusToBottomSide(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(),
                ABSOLUTE_ERROR));
        assertFalse(r.isLocusToBottomSide(insidePoint.getInhomX(), insidePoint.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isLocusToBottomSide(pointAtLeftBorder.getInhomX(), pointAtLeftBorder.getInhomY(),
                ABSOLUTE_ERROR));
        assertFalse(r.isLocusToBottomSide(pointAtTopBorder.getInhomX(), pointAtTopBorder.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isLocusToBottomSide(pointAtRightBorder.getInhomX(), pointAtRightBorder.getInhomY(),
                ABSOLUTE_ERROR));
        assertTrue(r.isLocusToBottomSide(pointAtBottomBorder.getInhomX(), pointAtBottomBorder.getInhomY(),
                ABSOLUTE_ERROR));

        // test non-static method with point and threshold
        r = new Rectangle(left, top, right, bottom);
        assertFalse(r.isLocusToBottomSide(pointAtLeftSide, ABSOLUTE_ERROR));
        assertFalse(r.isLocusToBottomSide(pointAtTopLeftCorner, ABSOLUTE_ERROR));
        assertFalse(r.isLocusToBottomSide(pointAtTopSide, ABSOLUTE_ERROR));
        assertFalse(r.isLocusToBottomSide(pointAtTopRightCorner, ABSOLUTE_ERROR));
        assertFalse(r.isLocusToBottomSide(pointAtRightSide, ABSOLUTE_ERROR));
        assertFalse(r.isLocusToBottomSide(pointAtBottomRightCorner, ABSOLUTE_ERROR));
        assertFalse(r.isLocusToBottomSide(pointAtBottomSide, ABSOLUTE_ERROR));
        assertFalse(r.isLocusToBottomSide(pointAtBottomLeftCorner, ABSOLUTE_ERROR));
        assertFalse(r.isLocusToBottomSide(insidePoint, ABSOLUTE_ERROR));
        assertFalse(r.isLocusToBottomSide(pointAtLeftBorder, ABSOLUTE_ERROR));
        assertFalse(r.isLocusToBottomSide(pointAtTopBorder, ABSOLUTE_ERROR));
        assertFalse(r.isLocusToBottomSide(pointAtRightBorder, ABSOLUTE_ERROR));
        assertTrue(r.isLocusToBottomSide(pointAtBottomBorder, ABSOLUTE_ERROR));

        // test without threshold

        // test static method with point and corner coordinates and no threshold
        assertFalse(Rectangle.isLocusToBottomSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), left, top,
                right, bottom));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                left, top, right, bottom));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), left, top,
                right, bottom));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                left, top, right, bottom));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), left, top,
                right, bottom));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), left, top, right, bottom));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), left,
                top, right, bottom));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), left, top, right, bottom));
        assertFalse(Rectangle.isLocusToBottomSide(insidePoint.getInhomX(), insidePoint.getInhomY(), left, top, right,
                bottom));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtLeftBorder.getInhomX(), pointAtLeftBorder.getInhomY(), left,
                top, right, bottom));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtTopBorder.getInhomX(), pointAtTopBorder.getInhomY(), left, top,
                right, bottom));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtRightBorder.getInhomX(), pointAtRightBorder.getInhomY(), left,
                top, right, bottom));
        assertTrue(Rectangle.isLocusToBottomSide(pointAtBottomBorder.getInhomX(), pointAtBottomBorder.getInhomY(), left,
                top, right, bottom));

        // test static method with point, corner coordinates and threshold
        assertFalse(Rectangle.isLocusToBottomSide(pointAtLeftSide, left, top, right, bottom));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtTopLeftCorner, left, top, right, bottom));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtTopSide, left, top, right, bottom));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtTopRightCorner, left, top, right, bottom));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtRightSide, left, top, right, bottom));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtBottomRightCorner, left, top, right, bottom));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtBottomSide, left, top, right, bottom));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtBottomLeftCorner, left, top, right, bottom));
        assertFalse(Rectangle.isLocusToBottomSide(insidePoint, left, top, right, bottom));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtLeftBorder, left, top, right, bottom));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtTopBorder, left, top, right, bottom));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtRightBorder, left, top, right, bottom));
        assertTrue(Rectangle.isLocusToBottomSide(pointAtBottomBorder, left, top, right, bottom));

        // test static method with point coordinates, corners and threshold
        assertFalse(Rectangle.isLocusToBottomSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), topLeft,
                bottomRight));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), topLeft,
                bottomRight));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), topLeft,
                bottomRight));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), topLeft,
                bottomRight));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToBottomSide(insidePoint.getInhomX(), insidePoint.getInhomY(), topLeft,
                bottomRight));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtLeftBorder.getInhomX(), pointAtLeftBorder.getInhomY(), topLeft,
                bottomRight));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtTopBorder.getInhomX(), pointAtTopBorder.getInhomY(), topLeft,
                bottomRight));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtRightBorder.getInhomX(), pointAtRightBorder.getInhomY(),
                topLeft, bottomRight));
        assertTrue(Rectangle.isLocusToBottomSide(pointAtBottomBorder.getInhomX(), pointAtBottomBorder.getInhomY(),
                topLeft, bottomRight));

        // test static method with point, corners and threshold
        assertFalse(Rectangle.isLocusToBottomSide(pointAtLeftSide, topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtTopLeftCorner, topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtTopSide, topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtTopRightCorner, topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtRightSide, topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtBottomRightCorner, topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtBottomSide, topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtBottomLeftCorner, topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToBottomSide(insidePoint, topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtLeftBorder, topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtTopBorder, topLeft, bottomRight));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtRightBorder, topLeft, bottomRight));
        assertTrue(Rectangle.isLocusToBottomSide(pointAtBottomBorder, topLeft, bottomRight));

        // test static method with point coordinates, center, size and threshold
        assertFalse(Rectangle.isLocusToBottomSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), center,
                width, height));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(),
                center, width, height));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), center, width,
                height));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(),
                center, width, height));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), center,
                width, height));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtBottomRightCorner.getInhomX(),
                pointAtBottomRightCorner.getInhomY(), center, width, height));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), center,
                width, height));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtBottomLeftCorner.getInhomX(),
                pointAtBottomLeftCorner.getInhomY(), center, width, height));
        assertFalse(Rectangle.isLocusToBottomSide(insidePoint.getInhomX(), insidePoint.getInhomY(), center, width,
                height));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtLeftBorder.getInhomX(), pointAtLeftBorder.getInhomY(), center,
                width, height));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtTopBorder.getInhomX(), pointAtTopBorder.getInhomY(), center,
                width, height));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtRightBorder.getInhomX(), pointAtRightBorder.getInhomY(),
                center, width, height));
        assertTrue(Rectangle.isLocusToBottomSide(pointAtBottomBorder.getInhomX(), pointAtBottomBorder.getInhomY(),
                center, width, height));

        // test static method with point, center, size and threshold
        assertFalse(Rectangle.isLocusToBottomSide(pointAtLeftSide, center, width, height));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtTopLeftCorner, center, width, height));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtTopSide, center, width, height));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtTopRightCorner, center, width, height));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtRightSide, center, width, height));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtBottomRightCorner, center, width, height));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtBottomSide, center, width, height));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtBottomLeftCorner, center, width, height));
        assertFalse(Rectangle.isLocusToBottomSide(insidePoint, center, width, height));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtLeftBorder, center, width, height));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtTopBorder, center, width, height));
        assertFalse(Rectangle.isLocusToBottomSide(pointAtRightBorder, center, width, height));
        assertTrue(Rectangle.isLocusToBottomSide(pointAtBottomBorder, center, width, height));

        // test non-static method with point coordinates and threshold
        r = new Rectangle(left, top, right, bottom);
        assertFalse(r.isLocusToBottomSide(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY()));
        assertFalse(r.isLocusToBottomSide(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY()));
        assertFalse(r.isLocusToBottomSide(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY()));
        assertFalse(r.isLocusToBottomSide(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY()));
        assertFalse(r.isLocusToBottomSide(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY()));
        assertFalse(r.isLocusToBottomSide(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY()));
        assertFalse(r.isLocusToBottomSide(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY()));
        assertFalse(r.isLocusToBottomSide(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY()));
        assertFalse(r.isLocusToBottomSide(insidePoint.getInhomX(), insidePoint.getInhomY()));
        assertFalse(r.isLocusToBottomSide(pointAtLeftBorder.getInhomX(), pointAtLeftBorder.getInhomY()));
        assertFalse(r.isLocusToBottomSide(pointAtTopBorder.getInhomX(), pointAtTopBorder.getInhomY()));
        assertFalse(r.isLocusToBottomSide(pointAtRightBorder.getInhomX(), pointAtRightBorder.getInhomY()));
        assertTrue(r.isLocusToBottomSide(pointAtBottomBorder.getInhomX(), pointAtBottomBorder.getInhomY()));

        // test non-static method with point and threshold
        r = new Rectangle(left, top, right, bottom);
        assertFalse(r.isLocusToBottomSide(pointAtLeftSide));
        assertFalse(r.isLocusToBottomSide(pointAtTopLeftCorner));
        assertFalse(r.isLocusToBottomSide(pointAtTopSide));
        assertFalse(r.isLocusToBottomSide(pointAtTopRightCorner));
        assertFalse(r.isLocusToBottomSide(pointAtRightSide));
        assertFalse(r.isLocusToBottomSide(pointAtBottomRightCorner));
        assertFalse(r.isLocusToBottomSide(pointAtBottomSide));
        assertFalse(r.isLocusToBottomSide(pointAtBottomLeftCorner));
        assertFalse(r.isLocusToBottomSide(insidePoint));
        assertFalse(r.isLocusToBottomSide(pointAtLeftBorder));
        assertFalse(r.isLocusToBottomSide(pointAtTopBorder));
        assertFalse(r.isLocusToBottomSide(pointAtRightBorder));
        assertTrue(r.isLocusToBottomSide(pointAtBottomBorder));
    }

    @Test
    void testIsLocus() {
        final var randomizer = new UniformRandomizer();
        final var centerX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var centerY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var width = randomizer.nextDouble(0, MAX_RANDOM_VALUE);
        final var height = randomizer.nextDouble(0, MAX_RANDOM_VALUE);

        final var left = centerX - 0.5 * width;
        final var right = centerX + 0.5 * width;
        final var top = centerY + 0.5 * height;
        final var bottom = centerY - 0.5 * height;

        final var topLeft = new InhomogeneousPoint2D(left, top);
        final var bottomRight = new InhomogeneousPoint2D(right, bottom);
        final var center = new InhomogeneousPoint2D(centerX, centerY);

        final var pointAtLeftSide = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5), centerY);
        final var pointAtTopLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopSide = new InhomogeneousPoint2D(
                centerX, centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtRightSide = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5), centerY);
        final var pointAtBottomRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomSide = new InhomogeneousPoint2D(
                centerX, centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var insidePoint = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(-0.5, 0.5),
                centerY + height * randomizer.nextDouble(-0.5, 0.5));

        var alpha = randomizer.nextDouble();
        final var pointAtLeftBorder = new InhomogeneousPoint2D(left, alpha * bottom + (1.0 - alpha) * top);
        alpha = randomizer.nextDouble();
        final var pointAtTopBorder = new InhomogeneousPoint2D(alpha * left + (1.0 - alpha) * right, top);
        alpha = randomizer.nextDouble();
        final var pointAtRightBorder = new InhomogeneousPoint2D(right, alpha * bottom + (1.0 - alpha) * top);
        alpha = randomizer.nextDouble();
        final var pointAtBottomBorder = new InhomogeneousPoint2D(alpha * left + (1.0 - alpha) * right, bottom);

        // test static method with point and corner coordinates and threshold
        assertFalse(Rectangle.isLocus(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), left, top, right,
                bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocus(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(), left, top,
                right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocus(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), left, top, right, bottom,
                ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocus(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(), left, top,
                right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocus(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), left, top, right,
                bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocus(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY(), left,
                top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocus(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), left, top, right,
                bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocus(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(), left,
                top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocus(insidePoint.getInhomX(), insidePoint.getInhomY(), left, top, right, bottom,
                ABSOLUTE_ERROR));
        assertTrue(Rectangle.isLocus(pointAtLeftBorder.getInhomX(), pointAtLeftBorder.getInhomY(), left, top, right,
                bottom, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isLocus(pointAtTopBorder.getInhomX(), pointAtTopBorder.getInhomY(), left, top, right,
                bottom, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isLocus(pointAtRightBorder.getInhomX(), pointAtRightBorder.getInhomY(), left, top, right,
                bottom, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isLocus(pointAtBottomBorder.getInhomX(), pointAtBottomBorder.getInhomY(), left, top, right,
                bottom, ABSOLUTE_ERROR));

        // test static method with point, corner coordinates and threshold
        assertFalse(Rectangle.isLocus(pointAtLeftSide, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocus(pointAtTopLeftCorner, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocus(pointAtTopSide, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocus(pointAtTopRightCorner, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocus(pointAtRightSide, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocus(pointAtBottomRightCorner, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocus(pointAtBottomSide, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocus(pointAtBottomLeftCorner, left, top, right, bottom, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocus(insidePoint, left, top, right, bottom, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isLocus(pointAtLeftBorder, left, top, right, bottom, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isLocus(pointAtTopBorder, left, top, right, bottom, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isLocus(pointAtRightBorder, left, top, right, bottom, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isLocus(pointAtBottomBorder, left, top, right, bottom, ABSOLUTE_ERROR));

        // test static method with point coordinates, corners and threshold
        assertFalse(Rectangle.isLocus(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), topLeft, bottomRight,
                ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocus(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(), topLeft,
                bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocus(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), topLeft, bottomRight,
                ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocus(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(), topLeft,
                bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocus(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), topLeft, bottomRight,
                ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocus(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY(),
                topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocus(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), topLeft,
                bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocus(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(),
                topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocus(insidePoint.getInhomX(), insidePoint.getInhomY(), topLeft, bottomRight,
                ABSOLUTE_ERROR));
        assertTrue(Rectangle.isLocus(pointAtLeftBorder.getInhomX(), pointAtLeftBorder.getInhomY(), topLeft, bottomRight,
                ABSOLUTE_ERROR));
        assertTrue(Rectangle.isLocus(pointAtTopBorder.getInhomX(), pointAtTopBorder.getInhomY(), topLeft, bottomRight,
                ABSOLUTE_ERROR));
        assertTrue(Rectangle.isLocus(pointAtRightBorder.getInhomX(), pointAtRightBorder.getInhomY(), topLeft,
                bottomRight, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isLocus(pointAtBottomBorder.getInhomX(), pointAtBottomBorder.getInhomY(), topLeft,
                bottomRight, ABSOLUTE_ERROR));

        // test static method with point, corners and threshold
        assertFalse(Rectangle.isLocus(pointAtLeftSide, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocus(pointAtTopLeftCorner, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocus(pointAtTopSide, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocus(pointAtTopRightCorner, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocus(pointAtRightSide, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocus(pointAtBottomRightCorner, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocus(pointAtBottomSide, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocus(pointAtBottomLeftCorner, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocus(insidePoint, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isLocus(pointAtLeftBorder, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isLocus(pointAtTopBorder, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isLocus(pointAtRightBorder, topLeft, bottomRight, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isLocus(pointAtBottomBorder, topLeft, bottomRight, ABSOLUTE_ERROR));

        // test static method with point coordinates, center, size and threshold
        assertFalse(Rectangle.isLocus(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), center, width, height,
                ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocus(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(), center, width,
                height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocus(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), center, width, height,
                ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocus(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(), center,
                width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocus(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), center, width, height,
                ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocus(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY(),
                center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocus(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), center, width,
                height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocus(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(), center,
                width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocus(insidePoint.getInhomX(), insidePoint.getInhomY(), center, width, height,
                ABSOLUTE_ERROR));
        assertTrue(Rectangle.isLocus(pointAtLeftBorder.getInhomX(), pointAtLeftBorder.getInhomY(), center, width,
                height, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isLocus(pointAtTopBorder.getInhomX(), pointAtTopBorder.getInhomY(), center, width, height,
                ABSOLUTE_ERROR));
        assertTrue(Rectangle.isLocus(pointAtRightBorder.getInhomX(), pointAtRightBorder.getInhomY(), center, width,
                height, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isLocus(pointAtBottomBorder.getInhomX(), pointAtBottomBorder.getInhomY(), center, width,
                height, ABSOLUTE_ERROR));

        // test static method with point, center, size and threshold
        assertFalse(Rectangle.isLocus(pointAtLeftSide, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocus(pointAtTopLeftCorner, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocus(pointAtTopSide, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocus(pointAtTopRightCorner, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocus(pointAtRightSide, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocus(pointAtBottomRightCorner, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocus(pointAtBottomSide, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocus(pointAtBottomLeftCorner, center, width, height, ABSOLUTE_ERROR));
        assertFalse(Rectangle.isLocus(insidePoint, center, width, height, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isLocus(pointAtLeftBorder, center, width, height, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isLocus(pointAtTopBorder, center, width, height, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isLocus(pointAtRightBorder, center, width, height, ABSOLUTE_ERROR));
        assertTrue(Rectangle.isLocus(pointAtBottomBorder, center, width, height, ABSOLUTE_ERROR));

        // test non-static method with point coordinates and threshold
        Rectangle r = new Rectangle(left, top, right, bottom);
        assertFalse(r.isLocus(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isLocus(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isLocus(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isLocus(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isLocus(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isLocus(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY(),
                ABSOLUTE_ERROR));
        assertFalse(r.isLocus(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isLocus(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(), ABSOLUTE_ERROR));
        assertFalse(r.isLocus(insidePoint.getInhomX(), insidePoint.getInhomY(), ABSOLUTE_ERROR));
        assertTrue(r.isLocus(pointAtLeftBorder.getInhomX(), pointAtLeftBorder.getInhomY(), ABSOLUTE_ERROR));
        assertTrue(r.isLocus(pointAtTopBorder.getInhomX(), pointAtTopBorder.getInhomY(), ABSOLUTE_ERROR));
        assertTrue(r.isLocus(pointAtRightBorder.getInhomX(), pointAtRightBorder.getInhomY(), ABSOLUTE_ERROR));
        assertTrue(r.isLocus(pointAtBottomBorder.getInhomX(), pointAtBottomBorder.getInhomY(), ABSOLUTE_ERROR));

        // test non-static method with point and threshold
        r = new Rectangle(left, top, right, bottom);
        assertFalse(r.isLocus(pointAtLeftSide, ABSOLUTE_ERROR));
        assertFalse(r.isLocus(pointAtTopLeftCorner, ABSOLUTE_ERROR));
        assertFalse(r.isLocus(pointAtTopSide, ABSOLUTE_ERROR));
        assertFalse(r.isLocus(pointAtTopRightCorner, ABSOLUTE_ERROR));
        assertFalse(r.isLocus(pointAtRightSide, ABSOLUTE_ERROR));
        assertFalse(r.isLocus(pointAtBottomRightCorner, ABSOLUTE_ERROR));
        assertFalse(r.isLocus(pointAtBottomSide, ABSOLUTE_ERROR));
        assertFalse(r.isLocus(pointAtBottomLeftCorner, ABSOLUTE_ERROR));
        assertFalse(r.isLocus(insidePoint, ABSOLUTE_ERROR));
        assertTrue(r.isLocus(pointAtLeftBorder, ABSOLUTE_ERROR));
        assertTrue(r.isLocus(pointAtTopBorder, ABSOLUTE_ERROR));
        assertTrue(r.isLocus(pointAtRightBorder, ABSOLUTE_ERROR));
        assertTrue(r.isLocus(pointAtBottomBorder, ABSOLUTE_ERROR));

        // test without threshold

        // test static method with point and corner coordinates and no threshold
        assertFalse(Rectangle.isLocus(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), left, top, right,
                bottom));
        assertFalse(Rectangle.isLocus(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(), left, top,
                right, bottom));
        assertFalse(Rectangle.isLocus(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), left, top, right,
                bottom));
        assertFalse(Rectangle.isLocus(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(), left, top,
                right, bottom));
        assertFalse(Rectangle.isLocus(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), left, top, right,
                bottom));
        assertFalse(Rectangle.isLocus(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY(), left,
                top, right, bottom));
        assertFalse(Rectangle.isLocus(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), left, top, right,
                bottom));
        assertFalse(Rectangle.isLocus(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(), left,
                top, right, bottom));
        assertFalse(Rectangle.isLocus(insidePoint.getInhomX(), insidePoint.getInhomY(), left, top, right, bottom));
        assertTrue(Rectangle.isLocus(pointAtLeftBorder.getInhomX(), pointAtLeftBorder.getInhomY(), left, top, right,
                bottom));
        assertTrue(Rectangle.isLocus(pointAtTopBorder.getInhomX(), pointAtTopBorder.getInhomY(), left, top, right,
                bottom));
        assertTrue(Rectangle.isLocus(pointAtRightBorder.getInhomX(), pointAtRightBorder.getInhomY(), left, top, right,
                bottom));
        assertTrue(Rectangle.isLocus(pointAtBottomBorder.getInhomX(), pointAtBottomBorder.getInhomY(), left, top, right,
                bottom));

        // test static method with point, corner coordinates and threshold
        assertFalse(Rectangle.isLocus(pointAtLeftSide, left, top, right, bottom));
        assertFalse(Rectangle.isLocus(pointAtTopLeftCorner, left, top, right, bottom));
        assertFalse(Rectangle.isLocus(pointAtTopSide, left, top, right, bottom));
        assertFalse(Rectangle.isLocus(pointAtTopRightCorner, left, top, right, bottom));
        assertFalse(Rectangle.isLocus(pointAtRightSide, left, top, right, bottom));
        assertFalse(Rectangle.isLocus(pointAtBottomRightCorner, left, top, right, bottom));
        assertFalse(Rectangle.isLocus(pointAtBottomSide, left, top, right, bottom));
        assertFalse(Rectangle.isLocus(pointAtBottomLeftCorner, left, top, right, bottom));
        assertFalse(Rectangle.isLocus(insidePoint, left, top, right, bottom));
        assertTrue(Rectangle.isLocus(pointAtLeftBorder, left, top, right, bottom));
        assertTrue(Rectangle.isLocus(pointAtTopBorder, left, top, right, bottom));
        assertTrue(Rectangle.isLocus(pointAtRightBorder, left, top, right, bottom));
        assertTrue(Rectangle.isLocus(pointAtBottomBorder, left, top, right, bottom));

        // test static method with point coordinates, corners and threshold
        assertFalse(Rectangle.isLocus(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), topLeft, bottomRight));
        assertFalse(Rectangle.isLocus(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(), topLeft,
                bottomRight));
        assertFalse(Rectangle.isLocus(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), topLeft, bottomRight));
        assertFalse(Rectangle.isLocus(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(), topLeft,
                bottomRight));
        assertFalse(Rectangle.isLocus(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), topLeft,
                bottomRight));
        assertFalse(Rectangle.isLocus(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY(),
                topLeft, bottomRight));
        assertFalse(Rectangle.isLocus(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), topLeft,
                bottomRight));
        assertFalse(Rectangle.isLocus(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(), topLeft,
                bottomRight));
        assertFalse(Rectangle.isLocus(insidePoint.getInhomX(), insidePoint.getInhomY(), topLeft, bottomRight));
        assertTrue(Rectangle.isLocus(pointAtLeftBorder.getInhomX(), pointAtLeftBorder.getInhomY(), topLeft,
                bottomRight));
        assertTrue(Rectangle.isLocus(pointAtTopBorder.getInhomX(), pointAtTopBorder.getInhomY(), topLeft, bottomRight));
        assertTrue(Rectangle.isLocus(pointAtRightBorder.getInhomX(), pointAtRightBorder.getInhomY(), topLeft,
                bottomRight));
        assertTrue(Rectangle.isLocus(pointAtBottomBorder.getInhomX(), pointAtBottomBorder.getInhomY(), topLeft,
                bottomRight));

        // test static method with point, corners and threshold
        assertFalse(Rectangle.isLocus(pointAtLeftSide, topLeft, bottomRight));
        assertFalse(Rectangle.isLocus(pointAtTopLeftCorner, topLeft, bottomRight));
        assertFalse(Rectangle.isLocus(pointAtTopSide, topLeft, bottomRight));
        assertFalse(Rectangle.isLocus(pointAtTopRightCorner, topLeft, bottomRight));
        assertFalse(Rectangle.isLocus(pointAtRightSide, topLeft, bottomRight));
        assertFalse(Rectangle.isLocus(pointAtBottomRightCorner, topLeft, bottomRight));
        assertFalse(Rectangle.isLocus(pointAtBottomSide, topLeft, bottomRight));
        assertFalse(Rectangle.isLocus(pointAtBottomLeftCorner, topLeft, bottomRight));
        assertFalse(Rectangle.isLocus(insidePoint, topLeft, bottomRight));
        assertTrue(Rectangle.isLocus(pointAtLeftBorder, topLeft, bottomRight));
        assertTrue(Rectangle.isLocus(pointAtTopBorder, topLeft, bottomRight));
        assertTrue(Rectangle.isLocus(pointAtRightBorder, topLeft, bottomRight));
        assertTrue(Rectangle.isLocus(pointAtBottomBorder, topLeft, bottomRight));

        // test static method with point coordinates, center, size and threshold
        assertFalse(Rectangle.isLocus(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY(), center, width, height));
        assertFalse(Rectangle.isLocus(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY(), center, width,
                height));
        assertFalse(Rectangle.isLocus(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY(), center, width, height));
        assertFalse(Rectangle.isLocus(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY(), center,
                width, height));
        assertFalse(Rectangle.isLocus(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY(), center, width,
                height));
        assertFalse(Rectangle.isLocus(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY(),
                center, width, height));
        assertFalse(Rectangle.isLocus(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY(), center, width,
                height));
        assertFalse(Rectangle.isLocus(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY(), center,
                width, height));
        assertFalse(Rectangle.isLocus(insidePoint.getInhomX(), insidePoint.getInhomY(), center, width, height));
        assertTrue(Rectangle.isLocus(pointAtLeftBorder.getInhomX(), pointAtLeftBorder.getInhomY(), center, width,
                height));
        assertTrue(Rectangle.isLocus(pointAtTopBorder.getInhomX(), pointAtTopBorder.getInhomY(), center, width,
                height));
        assertTrue(Rectangle.isLocus(pointAtRightBorder.getInhomX(), pointAtRightBorder.getInhomY(), center, width,
                height));
        assertTrue(Rectangle.isLocus(pointAtBottomBorder.getInhomX(), pointAtBottomBorder.getInhomY(), center, width,
                height));

        // test static method with point, center, size and threshold
        assertFalse(Rectangle.isLocus(pointAtLeftSide, center, width, height));
        assertFalse(Rectangle.isLocus(pointAtTopLeftCorner, center, width, height));
        assertFalse(Rectangle.isLocus(pointAtTopSide, center, width, height));
        assertFalse(Rectangle.isLocus(pointAtTopRightCorner, center, width, height));
        assertFalse(Rectangle.isLocus(pointAtRightSide, center, width, height));
        assertFalse(Rectangle.isLocus(pointAtBottomRightCorner, center, width, height));
        assertFalse(Rectangle.isLocus(pointAtBottomSide, center, width, height));
        assertFalse(Rectangle.isLocus(pointAtBottomLeftCorner, center, width, height));
        assertFalse(Rectangle.isLocus(insidePoint, center, width, height));
        assertTrue(Rectangle.isLocus(pointAtLeftBorder, center, width, height));
        assertTrue(Rectangle.isLocus(pointAtTopBorder, center, width, height));
        assertTrue(Rectangle.isLocus(pointAtRightBorder, center, width, height));
        assertTrue(Rectangle.isLocus(pointAtBottomBorder, center, width, height));

        // test non-static method with point coordinates and threshold
        r = new Rectangle(left, top, right, bottom);
        assertFalse(r.isLocus(pointAtLeftSide.getInhomX(), pointAtLeftSide.getInhomY()));
        assertFalse(r.isLocus(pointAtTopLeftCorner.getInhomX(), pointAtTopLeftCorner.getInhomY()));
        assertFalse(r.isLocus(pointAtTopSide.getInhomX(), pointAtTopSide.getInhomY()));
        assertFalse(r.isLocus(pointAtTopRightCorner.getInhomX(), pointAtTopRightCorner.getInhomY()));
        assertFalse(r.isLocus(pointAtRightSide.getInhomX(), pointAtRightSide.getInhomY()));
        assertFalse(r.isLocus(pointAtBottomRightCorner.getInhomX(), pointAtBottomRightCorner.getInhomY()));
        assertFalse(r.isLocus(pointAtBottomSide.getInhomX(), pointAtBottomSide.getInhomY()));
        assertFalse(r.isLocus(pointAtBottomLeftCorner.getInhomX(), pointAtBottomLeftCorner.getInhomY()));
        assertFalse(r.isLocus(insidePoint.getInhomX(), insidePoint.getInhomY()));
        assertTrue(r.isLocus(pointAtLeftBorder.getInhomX(), pointAtLeftBorder.getInhomY()));
        assertTrue(r.isLocus(pointAtTopBorder.getInhomX(), pointAtTopBorder.getInhomY()));
        assertTrue(r.isLocus(pointAtRightBorder.getInhomX(), pointAtRightBorder.getInhomY()));
        assertTrue(r.isLocus(pointAtBottomBorder.getInhomX(), pointAtBottomBorder.getInhomY()));

        // test non-static method with point and threshold
        r = new Rectangle(left, top, right, bottom);
        assertFalse(r.isLocus(pointAtLeftSide));
        assertFalse(r.isLocus(pointAtTopLeftCorner));
        assertFalse(r.isLocus(pointAtTopSide));
        assertFalse(r.isLocus(pointAtTopRightCorner));
        assertFalse(r.isLocus(pointAtRightSide));
        assertFalse(r.isLocus(pointAtBottomRightCorner));
        assertFalse(r.isLocus(pointAtBottomSide));
        assertFalse(r.isLocus(pointAtBottomLeftCorner));
        assertFalse(r.isLocus(insidePoint));
        assertTrue(r.isLocus(pointAtLeftBorder));
        assertTrue(r.isLocus(pointAtTopBorder));
        assertTrue(r.isLocus(pointAtRightBorder));
        assertTrue(r.isLocus(pointAtBottomBorder));
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var randomizer = new UniformRandomizer();
        final var left = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var top = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var right = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var bottom = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var topLeft = new InhomogeneousPoint2D(left, top);
        final var bottomRight = new InhomogeneousPoint2D(right, bottom);

        final var r1 = new Rectangle(topLeft, bottomRight);

        assertSame(r1.getTopLeft(), topLeft);
        assertSame(r1.getBottomRight(), bottomRight);

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(r1);
        final var r2 = SerializationHelper.<Rectangle>deserialize(bytes);

        // check
        assertEquals(r1.getTopLeft(), r2.getTopLeft());
        assertNotSame(r1.getTopLeft(), r2.getTopLeft());
        assertEquals(r1.getBottomRight(), r2.getBottomRight());
        assertNotSame(r1.getBottomRight(), r2.getBottomRight());
    }
}
