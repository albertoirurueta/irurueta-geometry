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
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class CircleTest {

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    private static final double MIN_RANDOM_DEGREES = -180.0;
    private static final double MAX_RANDOM_DEGREES = 180.0;

    @Test
    public void testConstants() {
        assertEquals(0.0, Circle.MIN_RADIUS, 0.0);
        assertEquals(1e-9, Circle.DEFAULT_THRESHOLD, 0.0);
        assertEquals(0.0, Circle.MIN_THRESHOLD, 0.0);
        assertEquals(1e-12, Circle.EPS, 0.0);
    }

    @Test
    public void testConstructor() throws ColinearPointsException {
        // Test empty constructor
        Circle circle = new Circle();

        // check center is at origin and radius is 1.0
        assertTrue(circle.getCenter().equals(new InhomogeneousPoint2D(0.0, 0.0),
                ABSOLUTE_ERROR));
        assertEquals(circle.getRadius(), 1.0, ABSOLUTE_ERROR);

        // Test constructor with center and radius
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Point2D center = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final double radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE));

        circle = new Circle(center, radius);
        // check correctness
        assertTrue(circle.getCenter().equals(center, ABSOLUTE_ERROR));
        assertEquals(circle.getRadius(), radius, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        circle = null;
        try {
            circle = new Circle(center, -radius);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(circle);

        // test constructor with three points

        // pick 3 points belonging to the circle locus
        Point2D point1;
        Point2D point2;
        Point2D point3;
        boolean areEqual;
        do {
            double angle = randomizer.nextDouble(MIN_RANDOM_DEGREES,
                    MAX_RANDOM_DEGREES) * Math.PI / 180.0;
            point1 = new HomogeneousPoint2D(
                    center.getInhomX() + radius * Math.cos(angle),
                    center.getInhomY() + radius * Math.sin(angle), 1.0);
            angle = randomizer.nextDouble(MIN_RANDOM_DEGREES,
                    MAX_RANDOM_DEGREES) * Math.PI / 180.0;
            point2 = new HomogeneousPoint2D(
                    center.getInhomX() + radius * Math.cos(angle),
                    center.getInhomY() + radius * Math.sin(angle), 1.0);
            angle = randomizer.nextDouble(MIN_RANDOM_DEGREES,
                    MAX_RANDOM_DEGREES) * Math.PI / 180.0;
            point3 = new HomogeneousPoint2D(
                    center.getInhomX() + radius * Math.cos(angle),
                    center.getInhomY() + radius * Math.sin(angle), 1.0);

            // ensure that all three points are different
            areEqual = point1.equals(point2, ABSOLUTE_ERROR) ||
                    point2.equals(point3, ABSOLUTE_ERROR) ||
                    point3.equals(point1, ABSOLUTE_ERROR);
        } while (areEqual);


        // compute circle
        Circle circle2 = new Circle(point1, point2, point3);

        // check that both circles are equal
        circle = new Circle(center, radius);
        assertEquals(circle.getCenter().distanceTo(circle2.getCenter()), 0.0,
                ABSOLUTE_ERROR);
        assertEquals(circle.getRadius(), circle2.getRadius(), ABSOLUTE_ERROR);

        // Force ColinearPointsException
        circle = null;
        try {
            circle = new Circle(point1, point2, point2);
            fail("ColinearPointsException expected but not thrown");
        } catch (final ColinearPointsException ignore) {
        }
        assertNull(circle);

        // test from conic
        circle = new Circle(center, radius);
        Conic conic = circle.toConic();
        circle2 = new Circle(conic);
        assertEquals(circle.getCenter().distanceTo(circle2.getCenter()), 0.0,
                ABSOLUTE_ERROR);
        assertEquals(circle.getRadius(), circle2.getRadius(), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        conic = new Conic();
        circle = null;
        try {
            circle = new Circle(conic);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(circle);
    }

    @Test
    public void testGetSetCenter() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Point2D center = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final Circle circle = new Circle();
        // check center
        assertTrue(circle.getCenter().equals(new InhomogeneousPoint2D(0.0, 0.0),
                ABSOLUTE_ERROR));

        // set center
        circle.setCenter(center);
        // check correctness
        assertTrue(circle.getCenter().equals(center, ABSOLUTE_ERROR));

        // Force NullPointerException
        try {
            circle.setCenter(null);
            fail("NullPointerException expected but not thrown");
        } catch (final NullPointerException ignore) {
        }
    }

    @Test
    public void testGetSetRadius() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE));

        final Circle circle = new Circle();
        // check radius
        assertEquals(circle.getRadius(), 1.0, 0.0);

        // set radius
        circle.setRadius(radius);
        // check correctness
        assertEquals(circle.getRadius(), radius, 0.0);

        // Force IllegalArgumentException
        try {
            circle.setRadius(-radius);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testSetCenterAndRadius() {
        // Test constructor with center and radius
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Point2D center = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final double radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE));

        final Circle circle = new Circle();
        // check center
        assertTrue(circle.getCenter().equals(new InhomogeneousPoint2D(0.0, 0.0),
                ABSOLUTE_ERROR));
        // check radius
        assertEquals(circle.getRadius(), 1.0, 0.0);

        // set center and radius
        circle.setCenterAndRadius(center, radius);
        // check correctness
        assertTrue(circle.getCenter().equals(center, ABSOLUTE_ERROR));
        assertEquals(circle.getRadius(), radius, 0.0);

        // Force IllegalArgumentException
        try {
            circle.setCenterAndRadius(center, -radius);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        // Force NullPointerException
        try {
            circle.setCenterAndRadius(null, radius);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final NullPointerException ignore) {
        }
    }

    @Test
    public void testSetParametersFromPoints() throws ColinearPointsException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Point2D center = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final double radius = Math.abs(randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0,
                MAX_RANDOM_VALUE));

        final Circle circle1 = new Circle(center, radius);

        // pick 3 points belonging to the circle locus
        Point2D point1;
        Point2D point2;
        Point2D point3;
        boolean areEqual;
        do {
            double angle = randomizer.nextDouble(MIN_RANDOM_DEGREES,
                    MAX_RANDOM_DEGREES) * Math.PI / 180.0;
            point1 = new HomogeneousPoint2D(
                    center.getInhomX() + radius * Math.cos(angle),
                    center.getInhomY() + radius * Math.sin(angle), 1.0);
            angle = randomizer.nextDouble(MIN_RANDOM_DEGREES,
                    MAX_RANDOM_DEGREES) * Math.PI / 180.0;
            point2 = new HomogeneousPoint2D(
                    center.getInhomX() + radius * Math.cos(angle),
                    center.getInhomY() + radius * Math.sin(angle), 1.0);
            angle = randomizer.nextDouble(MIN_RANDOM_DEGREES,
                    MAX_RANDOM_DEGREES) * Math.PI / 180.0;
            point3 = new HomogeneousPoint2D(
                    center.getInhomX() + radius * Math.cos(angle),
                    center.getInhomY() + radius * Math.sin(angle), 1.0);

            // ensure that all three points are different
            areEqual = point1.equals(point2, ABSOLUTE_ERROR) ||
                    point2.equals(point3, ABSOLUTE_ERROR) ||
                    point3.equals(point1, ABSOLUTE_ERROR);
        } while (areEqual);

        // create new circle and set parameters
        final Circle circle2 = new Circle();

        circle2.setParametersFromPoints(point1, point2, point3);

        // check that both circles are equal
        assertEquals(circle1.getCenter().distanceTo(circle2.getCenter()), 0.0,
                ABSOLUTE_ERROR);
        assertEquals(circle1.getRadius(), circle2.getRadius(), ABSOLUTE_ERROR);

        // Force ColinearPointsException
        try {
            circle2.setParametersFromPoints(point1, point2, point2);
            fail("ColinearPointsException expected but not thrown");
        } catch (final ColinearPointsException ignore) {
        }
    }

    @Test
    public void testArea() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Point2D center = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final double radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE));

        final Circle circle = new Circle(center, radius);

        final double area = Math.PI * radius * radius;

        // Check correctness
        assertEquals(circle.getArea(), area, ABSOLUTE_ERROR);
        assertEquals(Circle.area(radius), area, ABSOLUTE_ERROR);
    }

    @Test
    public void testPerimeter() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Point2D center = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final double radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE));

        final Circle circle = new Circle(center, radius);

        final double perimeter = 2.0 * Math.PI * radius;

        // Check correctness
        assertEquals(circle.getPerimeter(), perimeter, ABSOLUTE_ERROR);
        assertEquals(Circle.perimeter(radius), perimeter, ABSOLUTE_ERROR);
    }

    @Test
    public void testCurvature() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Point2D center = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final double radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE));

        final Circle circle = new Circle(center, radius);

        assertEquals(Circle.curvature(radius), 1.0 / radius, ABSOLUTE_ERROR);
        assertEquals(circle.getCurvature(), 1.0 / radius, ABSOLUTE_ERROR);
    }

    @Test
    public void testIsInside() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Point2D center = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final double radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE));
        final double value = randomizer.nextDouble(0.2, 0.8);
        final double value2 = 1.0 + value;
        final double theta = randomizer.nextDouble(MIN_RANDOM_DEGREES,
                MAX_RANDOM_DEGREES) * Math.PI / 180.0;

        final Circle circle = new Circle(center, radius);

        final Point2D inside = new InhomogeneousPoint2D(
                center.getInhomX() + value * radius * Math.cos(theta),
                center.getInhomY() + value * radius * Math.sin(theta));
        final Point2D outside = new InhomogeneousPoint2D(
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
    public void testSignedDistanceDistanceAndIsLocus() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Point2D center = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final double radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE));
        final double value = randomizer.nextDouble(0.2, 0.8);
        final double value2 = 1.0 + value;
        final double theta = randomizer.nextDouble(MIN_RANDOM_DEGREES,
                MAX_RANDOM_DEGREES) * Math.PI / 180.0;

        final Circle circle = new Circle(center, radius);

        // center is not locus
        assertFalse(circle.isLocus(center));
        assertFalse(circle.isLocus(center, ABSOLUTE_ERROR));
        // but for a large enough threshold it might be
        assertTrue(circle.isLocus(center, 2.0 * radius));

        final Point2D inside = new InhomogeneousPoint2D(
                center.getInhomX() + value * radius * Math.cos(theta),
                center.getInhomY() + value * radius * Math.sin(theta));
        final Point2D outside = new InhomogeneousPoint2D(
                center.getInhomX() + value2 * radius * Math.cos(theta),
                center.getInhomY() + value2 * radius * Math.sin(theta));
        final Point2D zero = new InhomogeneousPoint2D(
                center.getInhomX() + radius * Math.cos(theta),
                center.getInhomY() + radius * Math.sin(theta));

        // check correctness
        assertEquals(circle.getSignedDistance(inside), (value - 1.0) * radius,
                ABSOLUTE_ERROR);
        assertEquals(Circle.signedDistance(circle, inside),
                (value - 1.0) * radius, ABSOLUTE_ERROR);

        assertEquals(circle.getDistance(inside),
                Math.abs((value - 1.0) * radius), ABSOLUTE_ERROR);
        assertEquals(Circle.distance(circle, inside),
                Math.abs((value - 1.0) * radius), ABSOLUTE_ERROR);

        // for inside point signed distance is negative
        assertTrue(circle.getSignedDistance(inside) <= 0.0);

        assertFalse(circle.isLocus(inside));
        assertFalse(circle.isLocus(inside, ABSOLUTE_ERROR));
        // true for a large enough threshold
        assertTrue(circle.isLocus(inside, radius));

        assertEquals(circle.getSignedDistance(outside), (value2 - 1.0) * radius,
                ABSOLUTE_ERROR);
        assertEquals(Circle.signedDistance(circle, outside),
                (value2 - 1.0) * radius, ABSOLUTE_ERROR);

        assertEquals(circle.getDistance(outside),
                Math.abs((value2 - 1.0) * radius), ABSOLUTE_ERROR);
        assertEquals(Circle.distance(circle, outside),
                Math.abs((value2 - 1.0) * radius), ABSOLUTE_ERROR);

        // for outside point distance is positive
        assertTrue(circle.getSignedDistance(outside) >= 0.0);

        assertFalse(circle.isLocus(outside));
        assertFalse(circle.isLocus(outside, ABSOLUTE_ERROR));
        //true for a large enough threshold
        assertTrue(circle.isLocus(outside, radius));

        // for point at locus of circle, distance is zero
        assertEquals(circle.getSignedDistance(zero), 0.0, ABSOLUTE_ERROR);
        assertEquals(Circle.signedDistance(circle, zero), 0.0, ABSOLUTE_ERROR);

        // zero is locus
        assertTrue(circle.isLocus(zero));
        assertTrue(circle.isLocus(zero, radius));

        // Force IllegalArgumentException
        try {
            circle.isLocus(zero, -radius);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testClosestPointAndIsLocus() throws UndefinedPointException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Point2D center = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final double radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE));
        final double value = randomizer.nextDouble(0.2, 0.8);
        final double value2 = 1.0 + value;
        final double theta = randomizer.nextDouble(MIN_RANDOM_DEGREES,
                MAX_RANDOM_DEGREES) * Math.PI / 180.0;

        final Circle circle = new Circle(center, radius);

        // center is not locus
        assertFalse(circle.isLocus(center));
        assertFalse(circle.isLocus(center, ABSOLUTE_ERROR));
        // but for a large enough threshold it might be
        assertTrue(circle.isLocus(center, 2.0 * radius));


        final Point2D inside = new InhomogeneousPoint2D(
                center.getInhomX() + value * radius * Math.cos(theta),
                center.getInhomY() + value * radius * Math.sin(theta));
        final Point2D outside = new InhomogeneousPoint2D(
                center.getInhomX() + value2 * radius * Math.cos(theta),
                center.getInhomY() + value2 * radius * Math.sin(theta));
        final Point2D zero = new InhomogeneousPoint2D(
                center.getInhomX() + radius * Math.cos(theta),
                center.getInhomY() + radius * Math.sin(theta));

        final Point2D expectedPoint = new InhomogeneousPoint2D(zero);

        final Point2D result = Point2D.create();

        // test for point inside (but far from center)
        assertTrue(circle.getClosestPoint(inside).equals(expectedPoint,
                ABSOLUTE_ERROR));
        circle.closestPoint(inside, result);
        assertTrue(result.equals(expectedPoint, ABSOLUTE_ERROR));

        assertFalse(circle.isLocus(inside));
        assertFalse(circle.isLocus(inside, ABSOLUTE_ERROR));
        // true for a large enough threshold
        assertTrue(circle.isLocus(inside, radius));

        // test for point outside of circle
        assertTrue(circle.getClosestPoint(outside).equals(expectedPoint,
                ABSOLUTE_ERROR));
        circle.closestPoint(outside, result);
        assertTrue(result.equals(expectedPoint, ABSOLUTE_ERROR));

        assertFalse(circle.isLocus(outside));
        assertFalse(circle.isLocus(outside, ABSOLUTE_ERROR));
        // true for a large enough threshold
        assertTrue(circle.isLocus(outside, radius));

        // test for point in circle boundary
        assertTrue(circle.getClosestPoint(zero).equals(expectedPoint,
                ABSOLUTE_ERROR));
        circle.closestPoint(zero, result);
        assertTrue(result.equals(expectedPoint, ABSOLUTE_ERROR));

        // zero is locus
        assertTrue(circle.isLocus(zero));
        assertTrue(circle.isLocus(zero, radius));

        // Force UndefinedPointException (by testing at center)
        try {
            circle.getClosestPoint(center);
            fail("UndefinedPointException expected but not thrown");
        } catch (final UndefinedPointException ignore) {
        }
        try {
            circle.closestPoint(center, result);
            fail("UndefinedPointException expected but not thrown");
        } catch (final UndefinedPointException ignore) {
        }

        // Force IllegalArgumentException
        try {
            circle.isLocus(zero, -radius);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetTangentLineAt() throws NotLocusException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Point2D center = new HomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 1.0);
        final double radius = Math.abs(randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0,
                MAX_RANDOM_VALUE));
        final double theta = randomizer.nextDouble(MIN_RANDOM_DEGREES,
                MAX_RANDOM_DEGREES) * Math.PI / 180.0;
        // angle corresponding to line slope
        final double theta2;
        if (theta > Math.PI / 2.0) {
            theta2 = theta - Math.PI;
        } else if (theta < -Math.PI / 2.0) {
            theta2 = theta + Math.PI;
        } else {
            theta2 = theta;
        }

        final Circle circle = new Circle(center, radius);

        final Point2D point = new HomogeneousPoint2D(
                center.getInhomX() + radius * Math.cos(theta),
                center.getInhomY() + radius * Math.sin(theta), 1.0);
        point.normalize();

        assertTrue(circle.isLocus(point));

        // find tangent line at locus point
        final Line2D line = circle.getTangentLineAt(point);
        final Line2D line2 = circle.getTangentLineAt(point, Circle.DEFAULT_THRESHOLD);
        final Line2D line3 = new Line2D();
        circle.tangentLineAt(point, line3, Circle.DEFAULT_THRESHOLD);

        // check that point is also at line's locus
        assertTrue(line.isLocus(point));
        assertTrue(line2.isLocus(point));
        assertEquals(line2, line3);

        // check that line angle is equal to theta
        final double lineAngle = line.getAngle();
        double theta3 = theta2 - Math.PI / 2.0;
        if (theta3 < -Math.PI / 2.0) {
            theta3 += Math.PI;
        } else if (theta3 > Math.PI / 2.0) {
            theta3 -= Math.PI;
        }
        assertEquals(lineAngle * 180.0 / Math.PI,
                theta3 * 180.0 / Math.PI, ABSOLUTE_ERROR);
    }

    @Test
    public void testToConic() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Point2D center = new HomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 1.0);
        final double radius = Math.abs(randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0,
                MAX_RANDOM_VALUE));
        final double theta = randomizer.nextDouble(MIN_RANDOM_DEGREES,
                MAX_RANDOM_DEGREES) * Math.PI / 180.0;
        center.normalize();

        final Circle circle = new Circle(center, radius);
        final Conic conic = circle.toConic();

        // center is not locus
        assertFalse(circle.isLocus(center));
        assertFalse(circle.isLocus(center, ABSOLUTE_ERROR));
        assertFalse(conic.isLocus(center));
        assertFalse(conic.isLocus(center, ABSOLUTE_ERROR));
        // but for a large enough threshold it might be
        assertTrue(circle.isLocus(center, 2.0 * radius));
        assertTrue(conic.isLocus(center, 2.0 * radius));

        final Point2D locus = new HomogeneousPoint2D(
                center.getInhomX() + radius * Math.cos(theta),
                center.getInhomY() + radius * Math.sin(theta), 1.0);
        locus.normalize();

        // test is locus
        assertTrue(circle.isLocus(locus, ABSOLUTE_ERROR));
        assertTrue(conic.isLocus(locus, ABSOLUTE_ERROR));

        // check correctness of estimated conic matrix (up to scale)
        final Matrix m = new Matrix(Conic.BASECONIC_MATRIX_ROW_SIZE,
                Conic.BASECONIC_MATRIX_COLUMN_SIZE);
        m.setElementAt(0, 0, 1.0);
        m.setElementAt(1, 0, 0.0);
        m.setElementAt(2, 0, -center.getInhomX());

        m.setElementAt(0, 1, 0);
        m.setElementAt(1, 1, 1.0);
        m.setElementAt(2, 1, -center.getInhomY());

        m.setElementAt(0, 2, -center.getInhomX());
        m.setElementAt(1, 2, -center.getInhomY());
        m.setElementAt(2, 2, center.getInhomX() * center.getInhomX() +
                center.getInhomY() * center.getInhomY() - radius * radius);

        final double norm = Utils.normF(m);
        m.multiplyByScalar(1.0 / norm);

        assertTrue(m.equals(conic.asMatrix(), ABSOLUTE_ERROR));
    }

    @Test
    public void testSetFromConic() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Point2D center = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final double radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE));

        final Circle circle = new Circle(center, radius);

        // test from conic
        Conic conic = circle.toConic();
        final Circle circle2 = new Circle();
        circle2.setFromConic(conic);

        // check correctness
        assertEquals(circle.getCenter().distanceTo(circle2.getCenter()), 0.0,
                ABSOLUTE_ERROR);
        assertEquals(circle.getRadius(), circle2.getRadius(), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        conic = new Conic();
        try {
            circle2.setFromConic(conic);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }
}
