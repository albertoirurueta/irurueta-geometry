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

import com.irurueta.algebra.*;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

class Line2DTest {

    private static final int HOM_COORDS = 3;
    private static final int INHOM_COORDS = 2;

    private static final double PRECISION_ERROR = 1e-6;
    private static final double MIN_RANDOM_VALUE = 1.0;
    private static final double MAX_RANDOM_VALUE = 100.0;
    private static final double MIN_RANDOM_DISTANCE = -100.0;
    private static final double MAX_RANDOM_DISTANCE = 100.0;
    private static final double MIN_DEGREES = -90.0;
    private static final double MAX_DEGREES = 90.0;

    private static final double ABSOLUTE_ERROR = 1e-8;

    private static final int TIMES = 100;

    @Test
    void testConstants() {
        assertEquals(3, Line2D.LINE_NUMBER_PARAMS);
        assertEquals(1e-12, Line2D.DEFAULT_LOCUS_THRESHOLD, 0.0);
        assertEquals(0.0, Line2D.MIN_THRESHOLD, 0.0);
        assertEquals(1e-10, Line2D.DEFAULT_COMPARISON_THRESHOLD, 0.0);
    }

    @Test
    void testConstructor() throws WrongSizeException, NotReadyException, LockedException, DecomposerException,
            com.irurueta.algebra.NotAvailableException {

        final var randomizer = new UniformRandomizer();

        final var a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        var line = new Line2D();
        assertEquals(0.0, line.getA(), 0.0);
        assertEquals(0.0, line.getB(), 0.0);
        assertEquals(0.0, line.getC(), 0.0);
        assertFalse(line.isNormalized());

        line = new Line2D(a, b, c);
        assertEquals(line.getA(), a, 0.0);
        assertEquals(line.getB(), b, 0.0);
        assertEquals(line.getC(), c, 0.0);
        assertFalse(line.isNormalized());

        // instantiate line using array
        var array = new double[HOM_COORDS];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        line = new Line2D(array);
        assertEquals(array[0], line.getA(), 0.0);
        assertEquals(array[1], line.getB(), 0.0);
        assertEquals(array[2], line.getC(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new Line2D(new double[HOM_COORDS + 1]));

        // find 1st point
        final var array1 = new double[HOM_COORDS];
        randomizer.fill(array1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var lambda = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // find 2nd point coincident with 1st point (up to scale)
        final var array2 = new double[HOM_COORDS];
        ArrayUtils.multiplyByScalar(array1, lambda, array2);

        var point1 = Point2D.create(CoordinatesType.HOMOGENEOUS_COORDINATES, array1);
        var point2 = Point2D.create(CoordinatesType.HOMOGENEOUS_COORDINATES, array2);

        // Force CoincidentPointsException
        final var finalPoint1 = point1;
        final var finalPoint2 = point2;
        assertThrows(CoincidentPointsException.class, () -> new Line2D(finalPoint1, finalPoint2, false));

        // try with 2 non-coincident points
        var m = Matrix.createWithUniformRandomValues(2, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var decomposer = new SingularValueDecomposer(m);
        decomposer.decompose();

        // ensure we create a matrix with 2 non-linear dependent rows
        while (decomposer.getRank() < 2) {
            m = Matrix.createWithUniformRandomValues(2, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            decomposer.setInputMatrix(m);
            decomposer.decompose();
        }

        // V contains the null-space of m in the last column, which is the line
        //joining both points in the rows of m
        final var v = decomposer.getV();

        point1 = new HomogeneousPoint2D(m.getElementAt(0, 0),
                m.getElementAt(0, 1),
                m.getElementAt(0, 2));
        point2 = new HomogeneousPoint2D(m.getElementAt(1, 0),
                m.getElementAt(1, 1),
                m.getElementAt(1, 2));

        point1.normalize();
        point2.normalize();

        final var arrayLine = new double[HOM_COORDS];
        arrayLine[0] = v.getElementAt(0, 2);
        arrayLine[1] = v.getElementAt(1, 2);
        arrayLine[2] = v.getElementAt(2, 2);

        line = new Line2D(point1, point2);

        // check correctness of obtained line
        final var scaleA = line.getA() / arrayLine[0];
        final var scaleB = line.getB() / arrayLine[1];
        final var scaleC = line.getC() / arrayLine[2];

        assertEquals(scaleA, scaleB, PRECISION_ERROR);
        assertEquals(scaleB, scaleC, PRECISION_ERROR);
        assertEquals(scaleC, scaleA, PRECISION_ERROR);

        // create line using point and director vector
        array = new double[HOM_COORDS];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var point = Point2D.create(CoordinatesType.HOMOGENEOUS_COORDINATES, array);

        var directorVector = new double[INHOM_COORDS];
        randomizer.fill(directorVector, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var norm = com.irurueta.algebra.Utils.normF(directorVector);
        directorVector = ArrayUtils.multiplyByScalarAndReturnNew(directorVector, 1.0 / norm);

        line = new Line2D(point, directorVector);
        line.normalize();

        // check correctness of instantiated line
        assertTrue(line.isLocus(point));

        final var directorVector2 = line.getDirectorVector();

        // check that both director vectors are equal
        final var scale1 = directorVector2[0] / directorVector[0];
        final var scale2 = directorVector2[1] / directorVector[1];
        assertEquals(scale1, scale2, PRECISION_ERROR);

        // Force IllegalArgumentException
        final var wrongArray = new double[HOM_COORDS];
        assertThrows(IllegalArgumentException.class, () -> new Line2D(point, wrongArray));
    }

    @Test
    void testGettersAndSetters() {
        for (var t = 0; t < TIMES; t++) {
            var array = new double[HOM_COORDS];
            final var randomizer = new UniformRandomizer();
            randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            var line = new Line2D();

            // set parameters using array
            line.setParameters(array);
            assertEquals(line.getA(), array[0], 0.0);
            assertEquals(line.getB(), array[1], 0.0);
            assertEquals(line.getC(), array[2], 0.0);

            var a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            var b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            var c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            line.setA(a);
            line.setB(b);
            line.setC(c);

            assertEquals(a, line.getA(), 0.0);
            assertEquals(b, line.getB(), 0.0);
            assertEquals(c, line.getC(), 0.0);

            a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            line.setParameters(a, b, c);
            assertEquals(a, line.getA(), 0.0);
            assertEquals(b, line.getB(), 0.0);
            assertEquals(c, line.getC(), 0.0);

            assertEquals(line.getAngle(), Math.atan(-a / b), PRECISION_ERROR);
            assertEquals(line.getSlope(), -a / b, PRECISION_ERROR);

            // find random angle in radians
            final var angle = randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES) * Math.PI / 180.0;
            line.setAngle(angle);

            assertEquals(angle, line.getAngle(), PRECISION_ERROR);
            assertEquals(Math.atan(-line.getA() / line.getB()), line.getAngle(), PRECISION_ERROR);
            assertEquals(Math.tan(angle), line.getSlope(), PRECISION_ERROR);

            final var slope = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            line.setSlope(slope);
            assertEquals(-line.getA() / line.getB(), line.getSlope(), PRECISION_ERROR);
            assertEquals(slope, line.getSlope(), PRECISION_ERROR);

            array = new double[HOM_COORDS];
            randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            line = new Line2D(array);
            c = line.getC();
            b = line.getB();
            assertEquals(-c / b, line.getYIntercept(), PRECISION_ERROR);

            randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            line = new Line2D(array);
            final var yIntercept = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            line.setYIntercept(yIntercept);
            assertEquals(yIntercept, line.getYIntercept(), PRECISION_ERROR);
        }
    }

    @Test
    void testSetParametersFromPairOfPoints() throws WrongSizeException, NotReadyException, LockedException,
            DecomposerException, com.irurueta.algebra.NotAvailableException, CoincidentPointsException {

        var m = Matrix.createWithUniformRandomValues(2, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var decomposer = new SingularValueDecomposer(m);
        decomposer.decompose();

        while (decomposer.getRank() < 2) {
            m = Matrix.createWithUniformRandomValues(2, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            decomposer.setInputMatrix(m);
            decomposer.decompose();
        }

        final var v = decomposer.getV();

        final var point1 = new HomogeneousPoint2D(m.getElementAt(0, 0),
                m.getElementAt(0, 1),
                m.getElementAt(0, 2));
        final var point2 = new HomogeneousPoint2D(m.getElementAt(1, 0),
                m.getElementAt(1, 1),
                m.getElementAt(1, 2));

        point1.normalize();
        point2.normalize();

        final var lineArray = new double[HOM_COORDS];
        lineArray[0] = v.getElementAt(0, 2);
        lineArray[1] = v.getElementAt(1, 2);
        lineArray[2] = v.getElementAt(2, 2);

        // find co-linear point using a third point up to scale of the 1st one
        //using homogeneous coordinates
        final var randomizer = new UniformRandomizer();
        final var scaleValue = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var coincidentPoint = new HomogeneousPoint2D(
                scaleValue * point1.getHomX(),
                scaleValue * point1.getHomY(),
                scaleValue * point1.getHomW());

        final var line = new Line2D();
        line.setParametersFromPairOfPoints(point1, point2, false);

        // compare line parameters respect to computed line vector. Notice that
        // parameters will be equal up to scale
        final var scaleA = line.getA() / lineArray[0];
        final var scaleB = line.getB() / lineArray[1];
        final var scaleC = line.getC() / lineArray[2];

        assertEquals(scaleA, scaleB, PRECISION_ERROR);
        assertEquals(scaleB, scaleC, PRECISION_ERROR);
        assertEquals(scaleC, scaleA, PRECISION_ERROR);

        // Force CoincidentPointsException
        assertThrows(CoincidentPointsException.class,
                () -> line.setParametersFromPairOfPoints(point1, coincidentPoint, false));

        // if we try coincident points without raising an exception then line is
        // instantiated no matter what, although obtained parameters might be
        // numerically unstable
        line.setParametersFromPairOfPoints(point1, coincidentPoint, true);
    }

    @Test
    void testSetParametersFromPointAndDirectorVector() {
        final var array = new double[HOM_COORDS];
        final var randomizer = new UniformRandomizer();
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var point = Point2D.create(CoordinatesType.HOMOGENEOUS_COORDINATES, array);

        var directorVector = new double[INHOM_COORDS];
        randomizer.fill(directorVector, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var norm = com.irurueta.algebra.Utils.normF(directorVector);
        directorVector = ArrayUtils.multiplyByScalarAndReturnNew(directorVector, 1.0 / norm);

        final var line = new Line2D();

        line.setParametersFromPointAndDirectorVector(point, directorVector);
        line.normalize();

        // check correctness of instantiated line
        assertTrue(line.isLocus(point));

        final var directorVector2 = line.getDirectorVector();

        // check that both director vectors are equal
        final var scale1 = directorVector2[0] / directorVector[0];
        final var scale2 = directorVector2[1] / directorVector[1];
        assertEquals(scale1, scale2, PRECISION_ERROR);

        // Force IllegalArgumentException
        final var wrongArray = new double[HOM_COORDS];
        assertThrows(IllegalArgumentException.class,
                () -> line.setParametersFromPointAndDirectorVector(point, wrongArray));
    }

    @Test
    void testIsLocus() throws WrongSizeException, NotReadyException, LockedException, DecomposerException,
            com.irurueta.algebra.NotAvailableException {

        // randomly choose 2 points to find their corresponding line
        var m = Matrix.createWithUniformRandomValues(2, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var decomposer = new SingularValueDecomposer(m);
        decomposer.decompose();

        while (decomposer.getRank() < 2) {
            m = Matrix.createWithUniformRandomValues(2, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            decomposer.setInputMatrix(m);
            decomposer.decompose();
        }

        final var v = decomposer.getV();

        final var line = new Line2D(v.getElementAt(0, 2),
                v.getElementAt(1, 2),
                v.getElementAt(2, 2));

        final var point1 = new HomogeneousPoint2D(m.getElementAt(0, 0),
                m.getElementAt(0, 1),
                m.getElementAt(0, 2));

        final var point2 = new HomogeneousPoint2D(m.getElementAt(1, 0),
                m.getElementAt(1, 1),
                m.getElementAt(1, 2));

        point1.normalize();
        point2.normalize();

        // point1 and point2 belong to line
        assertTrue(line.isLocus(point1, PRECISION_ERROR));
        assertTrue(line.isLocus(point2, PRECISION_ERROR));

        // use line director vector to find another point outside of line focus
        final var point3 = new HomogeneousPoint2D(point1);

        final var normDirectorVector = Math.sqrt(line.getA() * line.getA() + line.getB() * line.getB());
        point3.setInhomogeneousCoordinates(point3.getInhomX() + line.getA(),
                point3.getInhomY() + line.getB());

        assertFalse(line.isLocus(point3, PRECISION_ERROR));

        // indeed point3 is at normDirectorVector distance from line
        assertEquals(normDirectorVector, line.signedDistance(point3), PRECISION_ERROR);
    }

    @Test
    void testSignedDistance() throws WrongSizeException, NotReadyException, LockedException, DecomposerException,
            com.irurueta.algebra.NotAvailableException {

        // randomly choose 2 points to find their corresponding line
        var m = Matrix.createWithUniformRandomValues(2, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var decomposer = new SingularValueDecomposer(m);
        decomposer.decompose();

        while (decomposer.getRank() < 2) {
            m = Matrix.createWithUniformRandomValues(2, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            decomposer.setInputMatrix(m);
            decomposer.decompose();
        }

        final var v = decomposer.getV();

        final var line = new Line2D(v.getElementAt(0, 2),
                v.getElementAt(1, 2),
                v.getElementAt(2, 2));

        final var point1 = new HomogeneousPoint2D(m.getElementAt(0, 0),
                m.getElementAt(0, 1),
                m.getElementAt(0, 2));

        final var point2 = new HomogeneousPoint2D(m.getElementAt(1, 0),
                m.getElementAt(1, 1),
                m.getElementAt(1, 2));

        point1.normalize();
        point2.normalize();

        // point1 and point2 belong to line, hence their distance is zero up to
        // machine precision
        assertEquals(0.0, line.signedDistance(point1), PRECISION_ERROR);
        assertEquals(0.0, line.signedDistance(point2), PRECISION_ERROR);

        // use line director vector to find another point at desired signed
        // distance
        final var randomizer = new UniformRandomizer();
        final var signedDistance = randomizer.nextDouble(MIN_RANDOM_DISTANCE, MAX_RANDOM_DISTANCE);

        final var point3 = new HomogeneousPoint2D(point1);

        final var normDirectorVector = Math.sqrt(line.getA() * line.getA() + line.getB() * line.getB());
        point3.setInhomogeneousCoordinates(
                point3.getInhomX() + signedDistance * line.getA() / normDirectorVector,
                point3.getInhomY() + signedDistance * line.getB() / normDirectorVector);

        assertEquals(signedDistance, line.signedDistance(point3), PRECISION_ERROR);
    }

    @Test
    void testAsArray() {
        final var randomizer = new UniformRandomizer();

        var a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var line = new Line2D(a, b, c);
        final var array = line.asArray();

        assertEquals(a, array[0], 0.0);
        assertEquals(b, array[1], 0.0);
        assertEquals(c, array[2], 0.0);

        a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        line.setParameters(a, b, c);
        line.asArray(array);

        assertEquals(a, array[0], 0.0);
        assertEquals(b, array[1], 0.0);
        assertEquals(c, array[2], 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> line.asArray(new double[HOM_COORDS + 1]));
    }

    @Test
    void testNormalize() {
        final var array = new double[HOM_COORDS];
        final var randomizer = new UniformRandomizer();
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        var line = new Line2D(array);
        assertFalse(line.isNormalized());

        // normalize line
        line.normalize();
        assertTrue(line.isNormalized());

        // return line as array
        final var array2 = line.asArray();

        // compare that both arrays are equal up to scale
        // check correctness of obtained line
        final var scaleA = array[0] / array2[0];
        final var scaleB = array[1] / array2[1];
        final var scaleC = array[2] / array2[2];

        assertEquals(scaleA, scaleB, PRECISION_ERROR);
        assertEquals(scaleB, scaleC, PRECISION_ERROR);
        assertEquals(scaleC, scaleA, PRECISION_ERROR);

        // if we provide zero values, then normalization has no effect
        line = new Line2D(0.0, 0.0, 0.0);

        assertFalse(line.isNormalized());

        line.normalize();

        assertFalse(line.isNormalized());
    }

    @Test
    void testDirectorVector() {
        final var array = new double[HOM_COORDS];
        final var randomizer = new UniformRandomizer();
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var line = new Line2D(array);

        final var n = line.getDirectorVector();

        assertEquals(2, n.length);
        assertEquals(array[0], n[0], 0.0);
        assertEquals(array[1], n[1], 0.0);

        // try again
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        line.setParameters(array);

        line.directorVector(n);

        assertEquals(2, n.length);
        assertEquals(array[0], n[0], 0.0);
        assertEquals(array[1], n[1], 0.0);
    }

    @Test
    void testIntersection() throws WrongSizeException, DecomposerException, NoIntersectionException {

        final var point1 = new HomogeneousPoint2D();
        final var point2 = new HomogeneousPoint2D();
        final var point3 = new HomogeneousPoint2D();

        // Create two intersecting lines using 3 points
        var m1 = Matrix.createWithUniformRandomValues(2, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var m2 = Matrix.createWithUniformRandomValues(2, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // set 1st point in common in both matrices
        m2.setSubmatrix(0, 0, 0, HOM_COORDS - 1,
                m1.getSubmatrix(0, 0, 0, HOM_COORDS - 1));

        // ensure that both matrices have rank 2 (points are not coincident)
        while (com.irurueta.algebra.Utils.rank(m1) < 2 || com.irurueta.algebra.Utils.rank(m2) < 2) {

            // create random matrices again
            m1 = Matrix.createWithUniformRandomValues(2, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            m2 = Matrix.createWithUniformRandomValues(2, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            // set 1st point in common in both matrices
            m2.setSubmatrix(0, 0, 0, HOM_COORDS - 1,
                    m1.getSubmatrix(0, 0, 0, HOM_COORDS - 1));
        }

        point1.setHomogeneousCoordinates(m1.getElementAt(0, 0),
                m1.getElementAt(0, 1), m1.getElementAt(0, 2));
        point2.setHomogeneousCoordinates(m1.getElementAt(1, 0),
                m1.getElementAt(1, 1), m1.getElementAt(1, 2));
        point3.setHomogeneousCoordinates(m2.getElementAt(1, 0),
                m2.getElementAt(1, 1), m2.getElementAt(1, 2));

        // Create two lines between point1-point2, and point1-point3
        final var line1 = new Line2D(point1, point2);
        final var line2 = new Line2D(point1, point3);

        // because both lines have in common point1, then their intersection must
        // be point1

        var intersection1 = line1.getIntersection(line2);
        final var intersection2 = line2.getIntersection(line1);

        final var intersection3 = Point2D.create();
        line1.intersection(line2, intersection3);

        final var intersection4 = Point2D.create();
        line2.intersection(line1, intersection4);

        // check correctness
        assertTrue(intersection1.equals(point1, ABSOLUTE_ERROR));
        assertTrue(intersection2.equals(point1, ABSOLUTE_ERROR));
        assertTrue(intersection3.equals(point1, ABSOLUTE_ERROR));
        assertTrue(intersection4.equals(point1, ABSOLUTE_ERROR));

        // Force NoIntersectionException (by attempting to intersect a line with itself
        assertThrows(NoIntersectionException.class, () -> line1.getIntersection(line1));
        assertThrows(NoIntersectionException.class, () -> line1.intersection(line1, intersection2));
    }

    @Test
    void testClosestPoint() throws WrongSizeException, NotReadyException, LockedException, DecomposerException,
            com.irurueta.algebra.NotAvailableException {

        // randomly choose 2 points to find their corresponding line
        var m = Matrix.createWithUniformRandomValues(2, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var decomposer = new SingularValueDecomposer(m);
        decomposer.decompose();

        while (decomposer.getRank() < 2) {
            m = Matrix.createWithUniformRandomValues(2, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            decomposer.setInputMatrix(m);
            decomposer.decompose();
        }

        final var point1 = new HomogeneousPoint2D(m.getElementAt(0, 0),
                m.getElementAt(0, 1),
                m.getElementAt(0, 2));

        final var point2 = new HomogeneousPoint2D(m.getElementAt(1, 0),
                m.getElementAt(1, 1),
                m.getElementAt(1, 2));

        final var line = new Line2D(point1, point2);

        // point1 and point2 belong to line, hence their distance is zero up to
        // machine precision
        assertEquals(0.0, line.signedDistance(point1), PRECISION_ERROR);
        assertEquals(0.0, line.signedDistance(point2), PRECISION_ERROR);
        // because they belong to line, their closest point to line are
        // themselves
        final var closestPoint1 = line.getClosestPoint(point1);
        final var closestPoint1b = line.getClosestPoint(point1, PRECISION_ERROR);
        final var closestPoint1c = Point2D.create();
        line.closestPoint(point1, closestPoint1c);
        final var closestPoint1d = Point2D.create();
        line.closestPoint(point1, closestPoint1d, ABSOLUTE_ERROR);

        assertTrue(closestPoint1.equals(point1, ABSOLUTE_ERROR));
        assertTrue(closestPoint1b.equals(point1, ABSOLUTE_ERROR));
        assertTrue(closestPoint1c.equals(point1, ABSOLUTE_ERROR));
        assertTrue(closestPoint1d.equals(point1, ABSOLUTE_ERROR));

        final var closestPoint2 = line.getClosestPoint(point2);
        final var closestPoint2b = line.getClosestPoint(point2, PRECISION_ERROR);
        final var closestPoint2c = Point2D.create();
        line.closestPoint(point2, closestPoint2c);
        final var closestPoint2d = Point2D.create();
        line.closestPoint(point2, closestPoint2d, ABSOLUTE_ERROR);

        assertTrue(closestPoint2.equals(point2, ABSOLUTE_ERROR));
        assertTrue(closestPoint2b.equals(point2, ABSOLUTE_ERROR));
        assertTrue(closestPoint2c.equals(point2, ABSOLUTE_ERROR));
        assertTrue(closestPoint2d.equals(point2, ABSOLUTE_ERROR));

        // use line director vector to find another point at desired signed
        // distance
        final var randomizer = new UniformRandomizer();
        final var signedDistance = randomizer.nextDouble(MIN_RANDOM_DISTANCE, MAX_RANDOM_DISTANCE);

        final var point3 = new HomogeneousPoint2D();

        final var normDirectorVector = Math.sqrt(line.getA() * line.getA() + line.getB() * line.getB());
        point3.setInhomogeneousCoordinates(
                point1.getInhomX() + signedDistance * line.getA() / normDirectorVector,
                point1.getInhomY() + signedDistance * line.getB() / normDirectorVector);

        assertEquals(line.signedDistance(point3), signedDistance, PRECISION_ERROR);

        // because point3 goes in line's perpendicular direction from point1, its
        // closest point belonging to the line will be point1
        final var closestPoint = line.getClosestPoint(point3);
        final var closestPointB = line.getClosestPoint(point3, PRECISION_ERROR);
        final var closestPointC = Point2D.create();
        line.closestPoint(point3, closestPointC);
        final var closestPointD = Point2D.create();
        line.closestPoint(point3, closestPointD, ABSOLUTE_ERROR);

        assertTrue(closestPoint.equals(point1, ABSOLUTE_ERROR));
        assertTrue(closestPointB.equals(point1, ABSOLUTE_ERROR));
        assertTrue(closestPointC.equals(point1, ABSOLUTE_ERROR));
        assertTrue(closestPointD.equals(point1, ABSOLUTE_ERROR));

        assertThrows(IllegalArgumentException.class, () -> line.getClosestPoint(point3, -PRECISION_ERROR));
        assertThrows(IllegalArgumentException.class, () -> line.closestPoint(point3, closestPointD, -ABSOLUTE_ERROR));
    }

    @Test
    void testDotProduct() {
        final var array = new double[HOM_COORDS];
        final var randomizer = new UniformRandomizer();
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // a random line
        final var line1 = new Line2D(array);

        // opposed sign line
        final var line2 = new Line2D(-line1.getA(), -line1.getB(), -line1.getC());

        // another random line
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var line3 = new Line2D(array);

        line1.normalize();
        line2.normalize();
        line3.normalize();

        // test for equal lines
        assertEquals(1.0, line1.dotProduct(line1), ABSOLUTE_ERROR);
        // test for opposed sign lines
        assertEquals(-1.0, line1.dotProduct(line2), ABSOLUTE_ERROR);
        // test for 2 random lines
        assertEquals(line1.getA() * line3.getA() + line1.getB() * line3.getB() + line1.getC() * line3.getC(),
                line1.dotProduct(line3), ABSOLUTE_ERROR);
    }

    @Test
    void testEqualsAndHashCode() {
        final var array = new double[HOM_COORDS];
        final var randomizer = new UniformRandomizer();
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        var line1 = new Line2D(array);
        var line2 = new Line2D(array);
        assertTrue(line1.equals(line2, ABSOLUTE_ERROR));
        assertTrue(line1.equals(line2));
        assertEquals(line1, line2);
        assertEquals(line1.hashCode(), line2.hashCode());

        array[0] = line1.getA() + randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        array[1] = line1.getB();
        array[2] = line1.getC();

        line2 = new Line2D(array);
        assertFalse(line1.equals(line2, 0.0));

        array[0] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        array[1] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        array[2] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        line1 = new Line2D(array);
        array[0] *= 2.0;
        line2 = new Line2D(array);
        assertTrue(line1.equals(line2, 2.0));
        assertFalse(line1.equals(line2, 0.0));

        // Force IllegalArgumentException
        final var finalLine1 = line1;
        assertThrows(IllegalArgumentException.class, () -> finalLine1.equals(finalLine1, -ABSOLUTE_ERROR));
    }

    @Test
    void testCreateCanonicalLineAtInfinity() {
        final var line = Line2D.createCanonicalLineAtInfinity();

        assertEquals(0.0, line.getA(), 0.0);
        assertEquals(0.0, line.getB(), 0.0);
        assertEquals(1.0, line.getC(), 0.0);

        // create a point at infinity
        final var randomizer = new UniformRandomizer();
        final var point = new HomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 0.0);

        // check that point at infinity is locus of line
        assertTrue(line.isLocus(point));
    }

    @Test
    void testSetAsCanonicalLineAtInfinity() {
        // create a point at infinity
        final var randomizer = new UniformRandomizer();
        final var point = new HomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 0.0);

        final var line = new Line2D(1.0, 1.0, 1.0);

        // initially line is not at infinity
        assertFalse(line.isLocus(point));

        // set line at infinity
        Line2D.setAsCanonicalLineAtInfinity(line);

        // check correctness
        assertEquals(0.0, line.getA(), 0.0);
        assertEquals(0.0, line.getB(), 0.0);
        assertEquals(1.0, line.getC(), 0.0);

        // check that point at infinity is locus of line
        assertTrue(line.isLocus(point));
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var randomizer = new UniformRandomizer();

        final var a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var line1 = new Line2D(a, b, c);

        // check
        assertEquals(a, line1.getA(), 0.0);
        assertEquals(b, line1.getB(), 0.0);
        assertEquals(c, line1.getC(), 0.0);
        assertFalse(line1.isNormalized());

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(line1);
        final var line2 = SerializationHelper.<Line2D>deserialize(bytes);

        // check
        assertEquals(line1, line2);
        assertNotSame(line1, line2);
    }
}
