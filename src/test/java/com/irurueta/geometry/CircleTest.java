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
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

class CircleTest {
    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    private static final double MIN_RANDOM_DEGREES = -180.0;
    private static final double MAX_RANDOM_DEGREES = 180.0;

    @Test
    void testConstants() {
        assertEquals(0.0, Circle.MIN_RADIUS, 0.0);
        assertEquals(1e-9, Circle.DEFAULT_THRESHOLD, 0.0);
        assertEquals(0.0, Circle.MIN_THRESHOLD, 0.0);
        assertEquals(1e-12, Circle.EPS, 0.0);
    }

    @Test
    void testConstructor() throws ColinearPointsException {
        // Test empty constructor
        var circle = new Circle();

        // check center is at origin and radius is 1.0
        assertTrue(circle.getCenter().equals(new InhomogeneousPoint2D(0.0, 0.0), ABSOLUTE_ERROR));
        assertEquals(1.0, circle.getRadius(), ABSOLUTE_ERROR);

        // Test constructor with center and radius
        final var randomizer = new UniformRandomizer();
        final var center = new InhomogeneousPoint2D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        circle = new Circle(center, radius);
        // check correctness
        assertTrue(circle.getCenter().equals(center, ABSOLUTE_ERROR));
        assertEquals(radius, circle.getRadius(), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new Circle(center, -radius));

        // test constructor with three points

        // pick 3 points belonging to the circle locus
        Point2D point1;
        Point2D point2;
        Point2D point3;
        boolean areEqual;
        do {
            var angle = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
            point1 = new HomogeneousPoint2D(
                    center.getInhomX() + radius * Math.cos(angle),
                    center.getInhomY() + radius * Math.sin(angle), 1.0);
            angle = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
            point2 = new HomogeneousPoint2D(
                    center.getInhomX() + radius * Math.cos(angle),
                    center.getInhomY() + radius * Math.sin(angle), 1.0);
            angle = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
            point3 = new HomogeneousPoint2D(
                    center.getInhomX() + radius * Math.cos(angle),
                    center.getInhomY() + radius * Math.sin(angle), 1.0);

            // ensure that all three points are different
            areEqual = point1.equals(point2, ABSOLUTE_ERROR) || point2.equals(point3, ABSOLUTE_ERROR)
                    || point3.equals(point1, ABSOLUTE_ERROR);
        } while (areEqual);


        // compute circle
        var circle2 = new Circle(point1, point2, point3);

        // check that both circles are equal
        circle = new Circle(center, radius);
        assertEquals(0.0, circle.getCenter().distanceTo(circle2.getCenter()), ABSOLUTE_ERROR);
        assertEquals(circle.getRadius(), circle2.getRadius(), ABSOLUTE_ERROR);

        // Force ColinearPointsException
        final var finalPoint1 = point1;
        final var finalPoint2 = point2;
        assertThrows(ColinearPointsException.class, () -> new Circle(finalPoint1, finalPoint2, finalPoint2));

        // test from conic
        circle = new Circle(center, radius);
        var conic = circle.toConic();
        circle2 = new Circle(conic);
        assertEquals(0.0, circle.getCenter().distanceTo(circle2.getCenter()), ABSOLUTE_ERROR);
        assertEquals(circle.getRadius(), circle2.getRadius(), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        conic = new Conic();
        final var finalConic = conic;
        assertThrows(IllegalArgumentException.class, () -> new Circle(finalConic));
    }

    @Test
    void testGetSetCenter() {
        final var randomizer = new UniformRandomizer();
        final var center = new InhomogeneousPoint2D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final var circle = new Circle();
        // check center
        assertTrue(circle.getCenter().equals(new InhomogeneousPoint2D(0.0, 0.0), ABSOLUTE_ERROR));

        // set center
        circle.setCenter(center);
        // check correctness
        assertTrue(circle.getCenter().equals(center, ABSOLUTE_ERROR));

        // Force NullPointerException
        assertThrows(NullPointerException.class, () -> circle.setCenter(null));
    }

    @Test
    void testGetSetRadius() {
        final var randomizer = new UniformRandomizer();
        final var radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final var circle = new Circle();
        // check radius
        assertEquals(1.0, circle.getRadius(), 0.0);

        // set radius
        circle.setRadius(radius);
        // check correctness
        assertEquals(radius, circle.getRadius(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> circle.setRadius(-radius));
    }

    @Test
    void testSetCenterAndRadius() {
        // Test constructor with center and radius
        final var randomizer = new UniformRandomizer();
        final var center = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final var circle = new Circle();
        // check center
        assertTrue(circle.getCenter().equals(new InhomogeneousPoint2D(0.0, 0.0), ABSOLUTE_ERROR));
        // check radius
        assertEquals(1.0, circle.getRadius(), 0.0);

        // set center and radius
        circle.setCenterAndRadius(center, radius);
        // check correctness
        assertTrue(circle.getCenter().equals(center, ABSOLUTE_ERROR));
        assertEquals(radius, circle.getRadius(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> circle.setCenterAndRadius(center, -radius));

        // Force NullPointerException
        assertThrows(NullPointerException.class, () -> circle.setCenterAndRadius(null, radius));
    }

    @Test
    void testSetParametersFromPoints() throws ColinearPointsException {
        final var randomizer = new UniformRandomizer();
        final var center = new InhomogeneousPoint2D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var radius = Math.abs(randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, MAX_RANDOM_VALUE));

        final var circle1 = new Circle(center, radius);

        // pick 3 points belonging to the circle locus
        Point2D point1;
        Point2D point2;
        Point2D point3;
        boolean areEqual;
        do {
            var angle = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
            point1 = new HomogeneousPoint2D(
                    center.getInhomX() + radius * Math.cos(angle),
                    center.getInhomY() + radius * Math.sin(angle), 1.0);
            angle = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
            point2 = new HomogeneousPoint2D(
                    center.getInhomX() + radius * Math.cos(angle),
                    center.getInhomY() + radius * Math.sin(angle), 1.0);
            angle = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
            point3 = new HomogeneousPoint2D(center.getInhomX() + radius * Math.cos(angle),
                    center.getInhomY() + radius * Math.sin(angle), 1.0);

            // ensure that all three points are different
            areEqual = point1.equals(point2, ABSOLUTE_ERROR) || point2.equals(point3, ABSOLUTE_ERROR)
                    || point3.equals(point1, ABSOLUTE_ERROR);
        } while (areEqual);

        // create new circle and set parameters
        final var circle2 = new Circle();

        circle2.setParametersFromPoints(point1, point2, point3);

        // check that both circles are equal
        assertEquals(0.0, circle1.getCenter().distanceTo(circle2.getCenter()), ABSOLUTE_ERROR);
        assertEquals(circle1.getRadius(), circle2.getRadius(), ABSOLUTE_ERROR);

        // Force ColinearPointsException
        final var finalPoint1 = point1;
        final var finalPoint2 = point2;
        assertThrows(ColinearPointsException.class,
                () -> circle2.setParametersFromPoints(finalPoint1, finalPoint2, finalPoint2));
    }

    @Test
    void testArea() {
        final var randomizer = new UniformRandomizer();
        final var center = new InhomogeneousPoint2D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final var circle = new Circle(center, radius);

        final var area = Math.PI * radius * radius;

        // Check correctness
        assertEquals(area, circle.getArea(), ABSOLUTE_ERROR);
        assertEquals(area, Circle.area(radius), ABSOLUTE_ERROR);
    }

    @Test
    void testPerimeter() {
        final var randomizer = new UniformRandomizer();
        final var center = new InhomogeneousPoint2D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final var circle = new Circle(center, radius);

        final var perimeter = 2.0 * Math.PI * radius;

        // Check correctness
        assertEquals(perimeter, circle.getPerimeter(), ABSOLUTE_ERROR);
        assertEquals(perimeter, Circle.perimeter(radius), ABSOLUTE_ERROR);
    }

    @Test
    void testCurvature() {
        final var randomizer = new UniformRandomizer();
        final var center = new InhomogeneousPoint2D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final var circle = new Circle(center, radius);

        assertEquals(1.0 / radius, Circle.curvature(radius), ABSOLUTE_ERROR);
        assertEquals(1.0 / radius, circle.getCurvature(), ABSOLUTE_ERROR);
    }

    @Test
    void testIsInside() {
        final var randomizer = new UniformRandomizer();
        final var center = new InhomogeneousPoint2D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var value = randomizer.nextDouble(0.2, 0.8);
        final var value2 = 1.0 + value;
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));

        final var circle = new Circle(center, radius);

        final var inside = new InhomogeneousPoint2D(
                center.getInhomX() + value * radius * Math.cos(theta),
                center.getInhomY() + value * radius * Math.sin(theta));
        final var outside = new InhomogeneousPoint2D(
                center.getInhomX() + value2 * radius * Math.cos(theta),
                center.getInhomY() + value2 * radius * Math.sin(theta));

        // check correctness
        assertTrue(circle.isInside(inside));
        assertTrue(circle.isInside(inside, ABSOLUTE_ERROR));

        assertFalse(circle.isInside(outside));
        assertFalse(circle.isInside(outside, ABSOLUTE_ERROR));

        // test for a large positive threshold
        assertTrue(circle.isInside(inside, radius));
        assertTrue(circle.isInside(outside, radius));

        assertFalse(circle.isInside(inside, -radius));
        assertFalse(circle.isInside(outside, -radius));
    }

    @Test
    void testSignedDistanceDistanceAndIsLocus() {
        final var randomizer = new UniformRandomizer();
        final var center = new InhomogeneousPoint2D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var value = randomizer.nextDouble(0.2, 0.8);
        final var value2 = 1.0 + value;
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));

        final var circle = new Circle(center, radius);

        // center is not locus
        assertFalse(circle.isLocus(center));
        assertFalse(circle.isLocus(center, ABSOLUTE_ERROR));
        // but for a large enough threshold it might be
        assertTrue(circle.isLocus(center, 2.0 * radius));

        final var inside = new InhomogeneousPoint2D(
                center.getInhomX() + value * radius * Math.cos(theta),
                center.getInhomY() + value * radius * Math.sin(theta));
        final var outside = new InhomogeneousPoint2D(
                center.getInhomX() + value2 * radius * Math.cos(theta),
                center.getInhomY() + value2 * radius * Math.sin(theta));
        final var zero = new InhomogeneousPoint2D(
                center.getInhomX() + radius * Math.cos(theta),
                center.getInhomY() + radius * Math.sin(theta));

        // check correctness
        assertEquals((value - 1.0) * radius, circle.getSignedDistance(inside), ABSOLUTE_ERROR);
        assertEquals((value - 1.0) * radius, Circle.signedDistance(circle, inside), ABSOLUTE_ERROR);

        assertEquals(Math.abs((value - 1.0) * radius), circle.getDistance(inside), ABSOLUTE_ERROR);
        assertEquals(Math.abs((value - 1.0) * radius), Circle.distance(circle, inside), ABSOLUTE_ERROR);

        // for inside point signed distance is negative
        assertTrue(circle.getSignedDistance(inside) <= 0.0);

        assertFalse(circle.isLocus(inside));
        assertFalse(circle.isLocus(inside, ABSOLUTE_ERROR));
        // true for a large enough threshold
        assertTrue(circle.isLocus(inside, radius));

        assertEquals((value2 - 1.0) * radius, circle.getSignedDistance(outside), ABSOLUTE_ERROR);
        assertEquals((value2 - 1.0) * radius, Circle.signedDistance(circle, outside), ABSOLUTE_ERROR);

        assertEquals(Math.abs((value2 - 1.0) * radius), circle.getDistance(outside), ABSOLUTE_ERROR);
        assertEquals(Math.abs((value2 - 1.0) * radius), Circle.distance(circle, outside), ABSOLUTE_ERROR);

        // for outside point distance is positive
        assertTrue(circle.getSignedDistance(outside) >= 0.0);

        assertFalse(circle.isLocus(outside));
        assertFalse(circle.isLocus(outside, ABSOLUTE_ERROR));
        //true for a large enough threshold
        assertTrue(circle.isLocus(outside, radius));

        // for point at locus of circle, distance is zero
        assertEquals(0.0, circle.getSignedDistance(zero), ABSOLUTE_ERROR);
        assertEquals(0.0, Circle.signedDistance(circle, zero), ABSOLUTE_ERROR);

        // zero is locus
        assertTrue(circle.isLocus(zero));
        assertTrue(circle.isLocus(zero, radius));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> circle.isLocus(zero, -radius));
    }

    @Test
    void testClosestPointAndIsLocus() throws UndefinedPointException {
        final var randomizer = new UniformRandomizer();
        final var center = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var value = randomizer.nextDouble(0.2, 0.8);
        final var value2 = 1.0 + value;
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));

        final var circle = new Circle(center, radius);

        // center is not locus
        assertFalse(circle.isLocus(center));
        assertFalse(circle.isLocus(center, ABSOLUTE_ERROR));
        // but for a large enough threshold it might be
        assertTrue(circle.isLocus(center, 2.0 * radius));

        final var inside = new InhomogeneousPoint2D(
                center.getInhomX() + value * radius * Math.cos(theta),
                center.getInhomY() + value * radius * Math.sin(theta));
        final var outside = new InhomogeneousPoint2D(
                center.getInhomX() + value2 * radius * Math.cos(theta),
                center.getInhomY() + value2 * radius * Math.sin(theta));
        final var zero = new InhomogeneousPoint2D(
                center.getInhomX() + radius * Math.cos(theta),
                center.getInhomY() + radius * Math.sin(theta));

        final var expectedPoint = new InhomogeneousPoint2D(zero);

        final var result = Point2D.create();

        // test for point inside (but far from center)
        assertTrue(circle.getClosestPoint(inside).equals(expectedPoint, ABSOLUTE_ERROR));
        circle.closestPoint(inside, result);
        assertTrue(result.equals(expectedPoint, ABSOLUTE_ERROR));

        assertFalse(circle.isLocus(inside));
        assertFalse(circle.isLocus(inside, ABSOLUTE_ERROR));
        // true for a large enough threshold
        assertTrue(circle.isLocus(inside, radius));

        // test for point outside of circle
        assertTrue(circle.getClosestPoint(outside).equals(expectedPoint, ABSOLUTE_ERROR));
        circle.closestPoint(outside, result);
        assertTrue(result.equals(expectedPoint, ABSOLUTE_ERROR));

        assertFalse(circle.isLocus(outside));
        assertFalse(circle.isLocus(outside, ABSOLUTE_ERROR));
        // true for a large enough threshold
        assertTrue(circle.isLocus(outside, radius));

        // test for point in circle boundary
        assertTrue(circle.getClosestPoint(zero).equals(expectedPoint, ABSOLUTE_ERROR));
        circle.closestPoint(zero, result);
        assertTrue(result.equals(expectedPoint, ABSOLUTE_ERROR));

        // zero is locus
        assertTrue(circle.isLocus(zero));
        assertTrue(circle.isLocus(zero, radius));

        // Force UndefinedPointException (by testing at center)
        assertThrows(UndefinedPointException.class, () -> circle.getClosestPoint(center));
        assertThrows(UndefinedPointException.class, () -> circle.closestPoint(center, result));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> circle.isLocus(zero, -radius));
    }

    @Test
    void testGetTangentLineAt() throws NotLocusException {
        final var randomizer = new UniformRandomizer();
        final var center = new HomogeneousPoint2D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 1.0);
        final var radius = Math.abs(randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, MAX_RANDOM_VALUE));
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        // angle corresponding to line slope
        final double theta2;
        if (theta > Math.PI / 2.0) {
            theta2 = theta - Math.PI;
        } else if (theta < -Math.PI / 2.0) {
            theta2 = theta + Math.PI;
        } else {
            theta2 = theta;
        }

        final var circle = new Circle(center, radius);

        final var point = new HomogeneousPoint2D(
                center.getInhomX() + radius * Math.cos(theta),
                center.getInhomY() + radius * Math.sin(theta), 1.0);
        point.normalize();

        assertTrue(circle.isLocus(point));

        // find tangent line at locus point
        final var line = circle.getTangentLineAt(point);
        final var line2 = circle.getTangentLineAt(point, Circle.DEFAULT_THRESHOLD);
        final var line3 = new Line2D();
        circle.tangentLineAt(point, line3, Circle.DEFAULT_THRESHOLD);

        // check that point is also at line's locus
        assertTrue(line.isLocus(point));
        assertTrue(line2.isLocus(point));
        assertEquals(line2, line3);

        // check that line angle is equal to theta
        final var lineAngle = line.getAngle();
        var theta3 = theta2 - Math.PI / 2.0;
        if (theta3 < -Math.PI / 2.0) {
            theta3 += Math.PI;
        } else if (theta3 > Math.PI / 2.0) {
            theta3 -= Math.PI;
        }
        assertEquals(lineAngle * 180.0 / Math.PI, theta3 * 180.0 / Math.PI, ABSOLUTE_ERROR);
    }

    @Test
    void testToConic() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var center = new HomogeneousPoint2D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 1.0);
        final var radius = Math.abs(randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, MAX_RANDOM_VALUE));
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        center.normalize();

        final var circle = new Circle(center, radius);
        final var conic = circle.toConic();

        // center is not locus
        assertFalse(circle.isLocus(center));
        assertFalse(circle.isLocus(center, ABSOLUTE_ERROR));
        assertFalse(conic.isLocus(center));
        assertFalse(conic.isLocus(center, ABSOLUTE_ERROR));
        // but for a large enough threshold it might be
        assertTrue(circle.isLocus(center, 2.0 * radius));
        assertTrue(conic.isLocus(center, 2.0 * radius));

        final var locus = new HomogeneousPoint2D(
                center.getInhomX() + radius * Math.cos(theta),
                center.getInhomY() + radius * Math.sin(theta), 1.0);
        locus.normalize();

        // test is locus
        assertTrue(circle.isLocus(locus, ABSOLUTE_ERROR));
        assertTrue(conic.isLocus(locus, ABSOLUTE_ERROR));

        // check correctness of estimated conic matrix (up to scale)
        final var m = new Matrix(Conic.BASECONIC_MATRIX_ROW_SIZE, Conic.BASECONIC_MATRIX_COLUMN_SIZE);
        m.setElementAt(0, 0, 1.0);
        m.setElementAt(1, 0, 0.0);
        m.setElementAt(2, 0, -center.getInhomX());

        m.setElementAt(0, 1, 0);
        m.setElementAt(1, 1, 1.0);
        m.setElementAt(2, 1, -center.getInhomY());

        m.setElementAt(0, 2, -center.getInhomX());
        m.setElementAt(1, 2, -center.getInhomY());
        m.setElementAt(2, 2, center.getInhomX() * center.getInhomX()
                + center.getInhomY() * center.getInhomY() - radius * radius);

        final var norm = Utils.normF(m);
        m.multiplyByScalar(1.0 / norm);

        assertTrue(m.equals(conic.asMatrix(), ABSOLUTE_ERROR));
    }

    @Test
    void testSetFromConic() {
        final var randomizer = new UniformRandomizer();
        final var center = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final var circle = new Circle(center, radius);

        // test from conic
        var conic = circle.toConic();
        final var circle2 = new Circle();
        circle2.setFromConic(conic);

        // check correctness
        assertEquals(0.0, circle.getCenter().distanceTo(circle2.getCenter()), ABSOLUTE_ERROR);
        assertEquals(circle.getRadius(), circle2.getRadius(), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        conic = new Conic();
        final var finalConic = conic;
        assertThrows(IllegalArgumentException.class, () -> circle2.setFromConic(finalConic));
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var randomizer = new UniformRandomizer();
        final var center = new InhomogeneousPoint2D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final var circle1 = new Circle(center, radius);

        // check
        assertSame(center, circle1.getCenter());
        assertEquals(radius, circle1.getRadius(), 0.0);

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(circle1);
        final var circle2 = SerializationHelper.<Circle>deserialize(bytes);

        // check
        assertEquals(circle1.getCenter(), circle2.getCenter());
        assertEquals(circle1.getRadius(), circle2.getRadius(), 0.0);
    }
}
