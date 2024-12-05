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

class ConicTest {

    private static final double MIN_RANDOM_VALUE = -10.0;
    private static final double MAX_RANDOM_VALUE = 10.0;
    private static final double PRECISION_ERROR = 1e-8;
    private static final double LOCUS_THRESHOLD = 1e-8;
    private static final double PERPENDICULAR_THRESHOLD = 1e-5;

    private static final int CONIC_ROWS = 3;
    private static final int CONIC_COLS = 3;
    private static final int HOM_COORDS = 3;

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double MIN_RANDOM_DEGREES = -180.0;
    private static final double MAX_RANDOM_DEGREES = 180.0;

    private static final int TIMES = 10;

    @Test
    void testConstants() {
        assertEquals(3, BaseConic.BASECONIC_MATRIX_ROW_SIZE);
        assertEquals(3, BaseConic.BASECONIC_MATRIX_COLUMN_SIZE);
        assertEquals(6, BaseConic.N_PARAMS);
        assertEquals(1e-12, BaseConic.DEFAULT_LOCUS_THRESHOLD, 0.0);
        assertEquals(1e-12, BaseConic.DEFAULT_PERPENDICULAR_THRESHOLD, 0.0);
        assertEquals(0.0, BaseConic.MIN_THRESHOLD, 0.0);
    }

    @Test
    void testConstructor() throws WrongSizeException, IllegalArgumentException, NonSymmetricMatrixException,
            DecomposerException, CoincidentPointsException {

        final var randomizer = new UniformRandomizer();

        // Constructor
        var conic = new Conic();
        assertEquals(0.0, conic.getA(), 0.0);
        assertEquals(0.0, conic.getB(), 0.0);
        assertEquals(0.0, conic.getC(), 0.0);
        assertEquals(0.0, conic.getD(), 0.0);
        assertEquals(0.0, conic.getE(), 0.0);
        assertEquals(0.0, conic.getF(), 0.0);
        assertFalse(conic.isNormalized());

        // Constructor with params
        var a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var e = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var f = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        conic = new Conic(a, b, c, d, e, f);
        assertEquals(a, conic.getA(), 0.0);
        assertEquals(b, conic.getB(), 0.0);
        assertEquals(c, conic.getC(), 0.0);
        assertEquals(d, conic.getD(), 0.0);
        assertEquals(e, conic.getE(), 0.0);
        assertEquals(f, conic.getF(), 0.0);
        assertFalse(conic.isNormalized());

        // Constructor using matrix
        final var m = new Matrix(CONIC_ROWS, CONIC_COLS);
        // get random values
        a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        e = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        f = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        m.setElementAt(0, 0, a);
        m.setElementAt(0, 1, b);
        m.setElementAt(0, 2, d);
        m.setElementAt(1, 0, b);
        m.setElementAt(1, 1, c);
        m.setElementAt(1, 2, e);
        m.setElementAt(2, 0, d);
        m.setElementAt(2, 1, e);
        m.setElementAt(2, 2, f);

        conic = new Conic(m);

        assertEquals(m.getElementAt(0, 0), conic.getA(), 0.0);
        assertEquals(m.getElementAt(0, 1), conic.getB(), 0.0);
        assertEquals(m.getElementAt(1, 1), conic.getC(), 0.0);
        assertEquals(m.getElementAt(0, 2), conic.getD(), 0.0);
        assertEquals(m.getElementAt(1, 2), conic.getE(), 0.0);
        assertEquals(m.getElementAt(2, 2), conic.getF(), 0.0);

        // Constructor using matrix with wrong size exception
        final var m2 = new Matrix(CONIC_ROWS, CONIC_COLS + 1);
        assertThrows(IllegalArgumentException.class, () -> new Conic(m2));

        // Constructor using non-symmetric matrix
        final var m3 = new Matrix(CONIC_ROWS, CONIC_COLS);
        m3.setElementAt(0, 0, a);
        m3.setElementAt(0, 1, b);
        m3.setElementAt(0, 2, d);
        m3.setElementAt(1, 0, b + 1.0);
        m3.setElementAt(1, 1, c);
        m3.setElementAt(1, 2, e + 1.0);
        m3.setElementAt(2, 0, d + 1.0);
        m3.setElementAt(2, 1, e);
        m3.setElementAt(2, 2, f);

        assertThrows(NonSymmetricMatrixException.class, () -> new Conic(m3));

        // Constructor from 5 points
        final var m4 = Matrix.createWithUniformRandomValues(5, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        HomogeneousPoint2D point1 = new HomogeneousPoint2D(m4.getElementAt(0, 0),
                m4.getElementAt(0, 1), m4.getElementAt(0, 2));
        HomogeneousPoint2D point2 = new HomogeneousPoint2D(m4.getElementAt(1, 0),
                m4.getElementAt(1, 1), m4.getElementAt(1, 2));
        HomogeneousPoint2D point3 = new HomogeneousPoint2D(m4.getElementAt(2, 0),
                m4.getElementAt(2, 1), m4.getElementAt(2, 2));
        HomogeneousPoint2D point4 = new HomogeneousPoint2D(m4.getElementAt(3, 0),
                m4.getElementAt(3, 1), m4.getElementAt(3, 2));
        HomogeneousPoint2D point5 = new HomogeneousPoint2D(m4.getElementAt(4, 0),
                m4.getElementAt(4, 1), m4.getElementAt(4, 2));

        // estimate conic that lies inside provided 5 homogeneous 2D
        // points
        var conicMatrix = new Matrix(5, 6);
        var x = point1.getHomX();
        var y = point1.getHomY();
        var w = point1.getHomW();
        conicMatrix.setElementAt(0, 0, x * x);
        conicMatrix.setElementAt(0, 1, x * y);
        conicMatrix.setElementAt(0, 2, y * y);
        conicMatrix.setElementAt(0, 3, x * w);
        conicMatrix.setElementAt(0, 4, y * w);
        conicMatrix.setElementAt(0, 5, w * w);
        x = point2.getHomX();
        y = point2.getHomY();
        w = point2.getHomW();
        conicMatrix.setElementAt(1, 0, x * x);
        conicMatrix.setElementAt(1, 1, x * y);
        conicMatrix.setElementAt(1, 2, y * y);
        conicMatrix.setElementAt(1, 3, x * w);
        conicMatrix.setElementAt(1, 4, y * w);
        conicMatrix.setElementAt(1, 5, w * w);
        x = point3.getHomX();
        y = point3.getHomY();
        w = point3.getHomW();
        conicMatrix.setElementAt(2, 0, x * x);
        conicMatrix.setElementAt(2, 1, x * y);
        conicMatrix.setElementAt(2, 2, y * y);
        conicMatrix.setElementAt(2, 3, x * w);
        conicMatrix.setElementAt(2, 4, y * w);
        conicMatrix.setElementAt(2, 5, w * w);
        x = point4.getHomX();
        y = point4.getHomY();
        w = point4.getHomW();
        conicMatrix.setElementAt(3, 0, x * x);
        conicMatrix.setElementAt(3, 1, x * y);
        conicMatrix.setElementAt(3, 2, y * y);
        conicMatrix.setElementAt(3, 3, x * w);
        conicMatrix.setElementAt(3, 4, y * w);
        conicMatrix.setElementAt(3, 5, w * w);
        x = point5.getHomX();
        y = point5.getHomY();
        w = point5.getHomW();
        conicMatrix.setElementAt(4, 0, x * x);
        conicMatrix.setElementAt(4, 1, x * y);
        conicMatrix.setElementAt(4, 2, y * y);
        conicMatrix.setElementAt(4, 3, x * w);
        conicMatrix.setElementAt(4, 4, y * w);
        conicMatrix.setElementAt(4, 5, w * w);

        while (com.irurueta.algebra.Utils.rank(conicMatrix) < 5) {
            final var m5 = Matrix.createWithUniformRandomValues(5, HOM_COORDS, MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);

            point1 = new HomogeneousPoint2D(m5.getElementAt(0, 0), m5.getElementAt(0, 1),
                    m5.getElementAt(0, 2));
            point2 = new HomogeneousPoint2D(m5.getElementAt(1, 0), m5.getElementAt(1, 1),
                    m5.getElementAt(1, 2));
            point3 = new HomogeneousPoint2D(m5.getElementAt(2, 0), m5.getElementAt(2, 1),
                    m5.getElementAt(2, 2));
            point4 = new HomogeneousPoint2D(m5.getElementAt(3, 0), m5.getElementAt(3, 1),
                    m5.getElementAt(3, 2));
            point5 = new HomogeneousPoint2D(m5.getElementAt(4, 0), m5.getElementAt(4, 1),
                    m5.getElementAt(4, 2));

            conicMatrix = new Matrix(5, 6);
            x = point1.getHomX();
            y = point1.getHomY();
            w = point1.getHomW();
            conicMatrix.setElementAt(0, 0, x * x);
            conicMatrix.setElementAt(0, 1, x * y);
            conicMatrix.setElementAt(0, 2, y * y);
            conicMatrix.setElementAt(0, 3, x * w);
            conicMatrix.setElementAt(0, 4, y * w);
            conicMatrix.setElementAt(0, 5, w * w);
            x = point2.getHomX();
            y = point2.getHomY();
            w = point2.getHomW();
            conicMatrix.setElementAt(1, 0, x * x);
            conicMatrix.setElementAt(1, 1, x * y);
            conicMatrix.setElementAt(1, 2, y * y);
            conicMatrix.setElementAt(1, 3, x * w);
            conicMatrix.setElementAt(1, 4, y * w);
            conicMatrix.setElementAt(1, 5, w * w);
            x = point3.getHomX();
            y = point3.getHomY();
            w = point3.getHomW();
            conicMatrix.setElementAt(2, 0, x * x);
            conicMatrix.setElementAt(2, 1, x * y);
            conicMatrix.setElementAt(2, 2, y * y);
            conicMatrix.setElementAt(2, 3, x * w);
            conicMatrix.setElementAt(2, 4, y * w);
            conicMatrix.setElementAt(2, 5, w * w);
            x = point4.getHomX();
            y = point4.getHomY();
            w = point4.getHomW();
            conicMatrix.setElementAt(3, 0, x * x);
            conicMatrix.setElementAt(3, 1, x * y);
            conicMatrix.setElementAt(3, 2, y * y);
            conicMatrix.setElementAt(3, 3, x * w);
            conicMatrix.setElementAt(3, 4, y * w);
            conicMatrix.setElementAt(3, 5, w * w);
            x = point5.getHomX();
            y = point5.getHomY();
            w = point5.getHomW();
            conicMatrix.setElementAt(4, 0, x * x);
            conicMatrix.setElementAt(4, 1, x * y);
            conicMatrix.setElementAt(4, 2, y * y);
            conicMatrix.setElementAt(4, 3, x * w);
            conicMatrix.setElementAt(4, 4, y * w);
            conicMatrix.setElementAt(4, 5, w * w);
        }

        conic = new Conic(point1, point2, point3, point4, point5);
        assertTrue(conic.isLocus(point1, LOCUS_THRESHOLD));
        assertTrue(conic.isLocus(point2, LOCUS_THRESHOLD));
        assertTrue(conic.isLocus(point3, LOCUS_THRESHOLD));
        assertTrue(conic.isLocus(point4, LOCUS_THRESHOLD));
        assertTrue(conic.isLocus(point5, LOCUS_THRESHOLD));

        // Force CoincidentPointsException
        final var finalPoint1 = point1;
        final var finalPoint2 = point2;
        final var finalPoint3 = point3;
        final var finalPoint4 = point4;
        assertThrows(CoincidentPointsException.class, () -> new Conic(finalPoint1, finalPoint2, finalPoint3,
                finalPoint4, finalPoint4));
    }

    @Test
    void testGettersAndSetters() throws WrongSizeException, IllegalArgumentException, NonSymmetricMatrixException {

        final var randomizer = new UniformRandomizer();

        final var conic = new Conic();
        var a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var e = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var f = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        conic.setA(a);
        conic.setB(b);
        conic.setC(c);
        conic.setD(d);
        conic.setE(e);
        conic.setF(f);
        assertEquals(conic.getA(), a, 0.0);
        assertEquals(conic.getB(), b, 0.0);
        assertEquals(conic.getC(), c, 0.0);
        assertEquals(conic.getD(), d, 0.0);
        assertEquals(conic.getE(), e, 0.0);
        assertEquals(conic.getF(), f, 0.0);

        a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        e = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        f = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        conic.setParameters(a, b, c, d, e, f);
        assertEquals(conic.getA(), a, 0.0);
        assertEquals(conic.getB(), b, 0.0);
        assertEquals(conic.getC(), c, 0.0);
        assertEquals(conic.getD(), d, 0.0);
        assertEquals(conic.getE(), e, 0.0);
        assertEquals(conic.getF(), f, 0.0);

        final var m = new Matrix(CONIC_ROWS, CONIC_COLS);
        a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        e = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        f = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        m.setElementAt(0, 0, a);
        m.setElementAt(0, 1, b);
        m.setElementAt(0, 2, d);
        m.setElementAt(1, 0, b);
        m.setElementAt(1, 1, c);
        m.setElementAt(1, 2, e);
        m.setElementAt(2, 0, d);
        m.setElementAt(2, 1, e);
        m.setElementAt(2, 2, f);
        conic.setParameters(m);
        assertEquals(conic.getA(), m.getElementAt(0, 0), 0.0);
        assertEquals(conic.getB(), m.getElementAt(0, 1), 0.0);
        assertEquals(conic.getC(), m.getElementAt(1, 1), 0.0);
        assertEquals(conic.getD(), m.getElementAt(0, 2), 0.0);
        assertEquals(conic.getE(), m.getElementAt(1, 2), 0.0);
        assertEquals(conic.getF(), m.getElementAt(2, 2), 0.0);

        // Force IllegalArgumentException
        final var wrong = new Matrix(CONIC_ROWS + 1, CONIC_COLS + 1);
        assertThrows(IllegalArgumentException.class, () -> conic.setParameters(wrong));

        // Force NonSymmetricMatrixException
        final var m2 = new Matrix(CONIC_ROWS, CONIC_COLS);
        a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        e = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        f = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        m2.setElementAt(0, 0, a);
        m2.setElementAt(0, 1, b);
        m2.setElementAt(0, 2, d);
        m2.setElementAt(1, 0, b + 1.0);
        m2.setElementAt(1, 1, c);
        m2.setElementAt(1, 2, e);
        m2.setElementAt(2, 0, d + 1.0);
        m2.setElementAt(2, 1, e + 1.0);
        m2.setElementAt(2, 2, f);
        assertThrows(NonSymmetricMatrixException.class, () -> conic.setParameters(m2));
        conic.setParameters(m2, 1.0);
    }

    @Test
    void testAsMatrix() throws WrongSizeException, IllegalArgumentException, NonSymmetricMatrixException {

        final var randomizer = new UniformRandomizer();
        final var conic = new Conic();
        final var m = new Matrix(CONIC_ROWS, CONIC_COLS);
        final var a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var e = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var f = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        m.setElementAt(0, 0, a);
        m.setElementAt(0, 1, b);
        m.setElementAt(0, 2, d);
        m.setElementAt(1, 0, b);
        m.setElementAt(1, 1, c);
        m.setElementAt(1, 2, e);
        m.setElementAt(2, 0, d);
        m.setElementAt(2, 1, e);
        m.setElementAt(2, 2, f);
        conic.setParameters(m);

        final var m2 = conic.asMatrix();

        assertTrue(m.equals(m2));
    }

    @Test
    void testIsLocus() throws WrongSizeException, DecomposerException, IllegalArgumentException,
            NonSymmetricMatrixException, CoincidentPointsException, NotReadyException, LockedException,
            com.irurueta.algebra.NotAvailableException {

        var m = Matrix.createWithUniformRandomValues(5, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        var point1 = new HomogeneousPoint2D(
                m.getElementAt(0, 0), m.getElementAt(0, 1), m.getElementAt(0, 2));
        var point2 = new HomogeneousPoint2D(
                m.getElementAt(1, 0), m.getElementAt(1, 1), m.getElementAt(1, 2));
        var point3 = new HomogeneousPoint2D(
                m.getElementAt(2, 0), m.getElementAt(2, 1), m.getElementAt(2, 2));
        var point4 = new HomogeneousPoint2D(
                m.getElementAt(3, 0), m.getElementAt(3, 1), m.getElementAt(3, 2));
        var point5 = new HomogeneousPoint2D(
                m.getElementAt(4, 0), m.getElementAt(4, 1), m.getElementAt(4, 2));

        point1.normalize();
        point2.normalize();
        point3.normalize();
        point4.normalize();
        point5.normalize();

        // estimate conic that lies inside provided 5 inhomogeneous image
        // points
        var conicMatrix = new Matrix(5, 6);
        var x = point1.getHomX();
        var y = point1.getHomY();
        var w = point1.getHomW();
        conicMatrix.setElementAt(0, 0, x * x);
        conicMatrix.setElementAt(0, 1, 2.0 * x * y);
        conicMatrix.setElementAt(0, 2, y * y);
        conicMatrix.setElementAt(0, 3, 2.0 * x * w);
        conicMatrix.setElementAt(0, 4, 2.0 * y * w);
        conicMatrix.setElementAt(0, 5, w * w);
        x = point2.getHomX();
        y = point2.getHomY();
        w = point2.getHomW();
        conicMatrix.setElementAt(1, 0, x * x);
        conicMatrix.setElementAt(1, 1, 2.0 * x * y);
        conicMatrix.setElementAt(1, 2, y * y);
        conicMatrix.setElementAt(1, 3, 2.0 * x * w);
        conicMatrix.setElementAt(1, 4, 2.0 * y * w);
        conicMatrix.setElementAt(1, 5, w * w);
        x = point3.getHomX();
        y = point3.getHomY();
        w = point3.getHomW();
        conicMatrix.setElementAt(2, 0, x * x);
        conicMatrix.setElementAt(2, 1, 2.0 * x * y);
        conicMatrix.setElementAt(2, 2, y * y);
        conicMatrix.setElementAt(2, 3, 2.0 * x * w);
        conicMatrix.setElementAt(2, 4, 2.0 * y * w);
        conicMatrix.setElementAt(2, 5, w * w);
        x = point4.getHomX();
        y = point4.getHomY();
        w = point4.getHomW();
        conicMatrix.setElementAt(3, 0, x * x);
        conicMatrix.setElementAt(3, 1, 2.0 * x * y);
        conicMatrix.setElementAt(3, 2, y * y);
        conicMatrix.setElementAt(3, 3, 2.0 * x * w);
        conicMatrix.setElementAt(3, 4, 2.0 * y * w);
        conicMatrix.setElementAt(3, 5, w * w);
        x = point5.getHomX();
        y = point5.getHomY();
        w = point5.getHomW();
        conicMatrix.setElementAt(4, 0, x * x);
        conicMatrix.setElementAt(4, 1, 2.0 * x * y);
        conicMatrix.setElementAt(4, 2, y * y);
        conicMatrix.setElementAt(4, 3, 2.0 * x * w);
        conicMatrix.setElementAt(4, 4, 2.0 * y * w);
        conicMatrix.setElementAt(4, 5, w * w);

        while (com.irurueta.algebra.Utils.rank(conicMatrix) < 5) {
            m = Matrix.createWithUniformRandomValues(5, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            point1 = new HomogeneousPoint2D(m.getElementAt(0, 0), m.getElementAt(0, 1),
                    m.getElementAt(0, 2));
            point2 = new HomogeneousPoint2D(m.getElementAt(1, 0), m.getElementAt(1, 1),
                    m.getElementAt(1, 2));
            point3 = new HomogeneousPoint2D(m.getElementAt(2, 0), m.getElementAt(2, 1),
                    m.getElementAt(2, 2));
            point4 = new HomogeneousPoint2D(m.getElementAt(3, 0), m.getElementAt(3, 1),
                    m.getElementAt(3, 2));
            point5 = new HomogeneousPoint2D(m.getElementAt(4, 0), m.getElementAt(4, 1),
                    m.getElementAt(4, 2));

            point1.normalize();
            point2.normalize();
            point3.normalize();
            point4.normalize();
            point5.normalize();

            conicMatrix = new Matrix(5, 6);
            x = point1.getHomX();
            y = point1.getHomY();
            w = point1.getHomW();
            conicMatrix.setElementAt(0, 0, x * x);
            conicMatrix.setElementAt(0, 1, 2.0 * x * y);
            conicMatrix.setElementAt(0, 2, y * y);
            conicMatrix.setElementAt(0, 3, 2.0 * x * w);
            conicMatrix.setElementAt(0, 4, 2.0 * y * w);
            conicMatrix.setElementAt(0, 5, w * w);
            x = point2.getHomX();
            y = point2.getHomY();
            w = point2.getHomW();
            conicMatrix.setElementAt(1, 0, x * x);
            conicMatrix.setElementAt(1, 1, 2.0 * x * y);
            conicMatrix.setElementAt(1, 2, y * y);
            conicMatrix.setElementAt(1, 3, 2.0 * x * w);
            conicMatrix.setElementAt(1, 4, 2.0 * y * w);
            conicMatrix.setElementAt(1, 5, w * w);
            x = point3.getHomX();
            y = point3.getHomY();
            w = point3.getHomW();
            conicMatrix.setElementAt(2, 0, x * x);
            conicMatrix.setElementAt(2, 1, 2.0 * x * y);
            conicMatrix.setElementAt(2, 2, y * y);
            conicMatrix.setElementAt(2, 3, 2.0 * x * w);
            conicMatrix.setElementAt(2, 4, 2.0 * y * w);
            conicMatrix.setElementAt(2, 5, w * w);
            x = point4.getHomX();
            y = point4.getHomY();
            w = point4.getHomW();
            conicMatrix.setElementAt(3, 0, x * x);
            conicMatrix.setElementAt(3, 1, 2.0 * x * y);
            conicMatrix.setElementAt(3, 2, y * y);
            conicMatrix.setElementAt(3, 3, 2.0 * x * w);
            conicMatrix.setElementAt(3, 4, 2.0 * y * w);
            conicMatrix.setElementAt(3, 5, w * w);
            x = point5.getHomX();
            y = point5.getHomY();
            w = point5.getHomW();
            conicMatrix.setElementAt(4, 0, x * x);
            conicMatrix.setElementAt(4, 1, 2.0 * x * y);
            conicMatrix.setElementAt(4, 2, y * y);
            conicMatrix.setElementAt(4, 3, 2.0 * x * w);
            conicMatrix.setElementAt(4, 4, 2.0 * y * w);
            conicMatrix.setElementAt(4, 5, w * w);
        }

        final var decomposer = new SingularValueDecomposer(conicMatrix);
        decomposer.decompose();

        final var v = decomposer.getV();

        final var a = v.getElementAt(0, 5);
        final var b = v.getElementAt(1, 5);
        final var c = v.getElementAt(2, 5);
        final var d = v.getElementAt(3, 5);
        final var e = v.getElementAt(4, 5);
        final var f = v.getElementAt(5, 5);

        final var conicPoint = new Matrix(CONIC_ROWS, 1);
        conicPoint.setElementAt(0, 0, point1.getHomX());
        conicPoint.setElementAt(1, 0, point1.getHomY());
        conicPoint.setElementAt(2, 0, point1.getHomW());

        var norm = com.irurueta.algebra.Utils.normF(conicPoint);
        conicPoint.multiplyByScalar(1.0 / norm);

        final var resultConicMatrix = new Matrix(CONIC_ROWS, CONIC_COLS);
        resultConicMatrix.setElementAt(0, 0, a);
        resultConicMatrix.setElementAt(0, 1, b);
        resultConicMatrix.setElementAt(0, 2, d);
        resultConicMatrix.setElementAt(1, 0, b);
        resultConicMatrix.setElementAt(1, 1, c);
        resultConicMatrix.setElementAt(1, 2, e);
        resultConicMatrix.setElementAt(2, 0, d);
        resultConicMatrix.setElementAt(2, 1, e);
        resultConicMatrix.setElementAt(2, 2, f);

        norm = com.irurueta.algebra.Utils.normF(resultConicMatrix);
        resultConicMatrix.multiplyByScalar(1.0 / norm);

        // find line tangent to conic resultConicMatrix at point conicPoint
        final var lineParams = resultConicMatrix.multiplyAndReturnNew(conicPoint);

        norm = com.irurueta.algebra.Utils.normF(lineParams);
        lineParams.multiplyByScalar(1.0 / norm);

        // use director vector of tangent line to find a point outside conic
        var directVectorA = lineParams.getElementAt(0, 0);
        var directVectorB = lineParams.getElementAt(1, 0);
        final var directVectorNorm = Math.sqrt(directVectorA * directVectorA + directVectorB * directVectorB);
        directVectorA /= directVectorNorm;
        directVectorB /= directVectorNorm;
        final var outsidePoint = new InhomogeneousPoint2D(point1.getInhomX() + directVectorA,
                point1.getInhomY() + directVectorB);

        // instantiate new conic instance
        final var conic = new Conic(resultConicMatrix);

        // check that initial 5 points lie inside the conic
        assertTrue(conic.isLocus(point1, LOCUS_THRESHOLD));
        assertTrue(conic.isLocus(point2, LOCUS_THRESHOLD));
        assertTrue(conic.isLocus(point3, LOCUS_THRESHOLD));
        assertTrue(conic.isLocus(point4, LOCUS_THRESHOLD));
        assertTrue(conic.isLocus(point5, LOCUS_THRESHOLD));

        // check point outside of conic
        assertFalse(conic.isLocus(outsidePoint, LOCUS_THRESHOLD));

        // test constructor from 5 points
        final var conic2 = new Conic(point1, point2, point3, point4, point5);
        assertTrue(conic2.isLocus(point1, LOCUS_THRESHOLD));
        assertTrue(conic2.isLocus(point2, LOCUS_THRESHOLD));
        assertTrue(conic2.isLocus(point3, LOCUS_THRESHOLD));
        assertTrue(conic2.isLocus(point4, LOCUS_THRESHOLD));
        assertTrue(conic2.isLocus(point5, LOCUS_THRESHOLD));
    }

    @Test
    void testAngleBetweenPoints() throws WrongSizeException, IllegalArgumentException, NonSymmetricMatrixException {

        // initial 2D points
        final var point1Matrix = Matrix.createWithUniformRandomValues(HOM_COORDS, 1, MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final var point2Matrix = Matrix.createWithUniformRandomValues(HOM_COORDS, 1, MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        // transformation matrix
        final var transform = Matrix.createWithUniformRandomValues(HOM_COORDS, HOM_COORDS, MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        // transform points
        final var tPoint1 = transform.multiplyAndReturnNew(point1Matrix);
        final var tPoint2 = transform.multiplyAndReturnNew(point2Matrix);

        // compute angle of transformed points
        final var norm1 = com.irurueta.algebra.Utils.normF(tPoint1);
        final var norm2 = com.irurueta.algebra.Utils.normF(tPoint2);

        final var numerator = tPoint1.transposeAndReturnNew().multiplyAndReturnNew(
                tPoint2).getElementAt(0, 0);

        final var cosAngle = numerator / (norm1 * norm2);

        final var angle = Math.acos(cosAngle);

        // compute conic matrix as the product of transposed transform matrix
        // with itself
        final var transposedTransform = transform.transposeAndReturnNew();
        final var conicMatrix = transposedTransform.multiplyAndReturnNew(transform);

        // normalize conic matrix
        final var normConic = com.irurueta.algebra.Utils.normF(conicMatrix);
        conicMatrix.multiplyByScalar(1.0 / normConic);

        final var conic = new Conic(conicMatrix);

        final var point1 = new HomogeneousPoint2D(point1Matrix.getElementAt(0, 0),
                point1Matrix.getElementAt(1, 0), point1Matrix.getElementAt(2, 0));
        final var point2 = new HomogeneousPoint2D(point2Matrix.getElementAt(0, 0),
                point2Matrix.getElementAt(1, 0), point2Matrix.getElementAt(2, 0));

        assertEquals(conic.angleBetweenPoints(point1, point2), angle, PRECISION_ERROR);
    }

    @Test
    void testArePerpendicularPoints() throws WrongSizeException, DecomposerException, RankDeficientMatrixException,
            IllegalArgumentException, NonSymmetricMatrixException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            var matrixPoint1 = Matrix.createWithUniformRandomValues(HOM_COORDS, 1, MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);

            var norm = com.irurueta.algebra.Utils.normF(matrixPoint1);
            matrixPoint1.multiplyByScalar(1.0 / norm);

            var matrixPoint2 = Matrix.createWithUniformRandomValues(HOM_COORDS, 1, MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            matrixPoint2.setElementAt(0, 0, matrixPoint1.getElementAt(1, 0)
                    + matrixPoint1.getElementAt(2, 0));
            matrixPoint2.setElementAt(1, 0, -matrixPoint1.getElementAt(0, 0));
            matrixPoint2.setElementAt(2, 0, -matrixPoint1.getElementAt(0, 0));

            norm = com.irurueta.algebra.Utils.normF(matrixPoint2);
            matrixPoint2.multiplyByScalar(1.0 / norm);

            var transform = Matrix.createWithUniformRandomValues(HOM_COORDS, HOM_COORDS, MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            while (com.irurueta.algebra.Utils.rank(transform) < 3) {
                transform = Matrix.createWithUniformRandomValues(HOM_COORDS, HOM_COORDS, MIN_RANDOM_VALUE,
                        MAX_RANDOM_VALUE);
            }

            final var invTransform = com.irurueta.algebra.Utils.inverse(transform);

            var transformPointMatrix1 = transform.multiplyAndReturnNew(matrixPoint1);
            var transformPointMatrix2 = transform.multiplyAndReturnNew(matrixPoint2);

            final var transInvTransform = invTransform.transposeAndReturnNew();

            final var conicMatrix = transInvTransform.multiplyAndReturnNew(invTransform);
            norm = com.irurueta.algebra.Utils.normF(conicMatrix);
            conicMatrix.multiplyByScalar(1.0 / norm);

            var transformPoint1 = new HomogeneousPoint2D(transformPointMatrix1.toArray());
            var transformPoint2 = new HomogeneousPoint2D(transformPointMatrix2.toArray());

            final var conic = new Conic(conicMatrix);

            assertTrue(conic.arePerpendicularPoints(transformPoint1, transformPoint2, PERPENDICULAR_THRESHOLD));

            // trying non-perpendicular points
            matrixPoint1 = Matrix.createWithUniformRandomValues(HOM_COORDS, 1, MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            norm = com.irurueta.algebra.Utils.normF(matrixPoint1);
            matrixPoint1.multiplyByScalar(1.0 / norm);

            matrixPoint2 = Matrix.createWithUniformRandomValues(HOM_COORDS, 1, MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            norm = com.irurueta.algebra.Utils.normF(matrixPoint2);
            matrixPoint2.multiplyByScalar(1.0 / norm);

            var dotProduct = matrixPoint1.transposeAndReturnNew().multiplyAndReturnNew(matrixPoint2)
                    .getElementAt(0, 0);

            // ensure points are not perpendicular
            while (Math.abs(dotProduct) < PERPENDICULAR_THRESHOLD) {
                matrixPoint1 = Matrix.createWithUniformRandomValues(HOM_COORDS, 1, MIN_RANDOM_VALUE,
                        MAX_RANDOM_VALUE);
                norm = com.irurueta.algebra.Utils.normF(matrixPoint1);
                matrixPoint1.multiplyByScalar(1.0 / norm);

                matrixPoint2 = Matrix.createWithUniformRandomValues(HOM_COORDS, 1, MIN_RANDOM_VALUE,
                        MAX_RANDOM_VALUE);
                norm = com.irurueta.algebra.Utils.normF(matrixPoint2);
                matrixPoint2.multiplyByScalar(1.0 / norm);

                dotProduct = matrixPoint1.transposeAndReturnNew().multiplyAndReturnNew(matrixPoint2)
                        .getElementAt(0, 0);
            }

            transformPointMatrix1 = transform.multiplyAndReturnNew(matrixPoint1);
            transformPointMatrix2 = transform.multiplyAndReturnNew(matrixPoint2);

            transformPoint1 = new HomogeneousPoint2D(transformPointMatrix1.toArray());
            transformPoint2 = new HomogeneousPoint2D(transformPointMatrix2.toArray());

            if (conic.arePerpendicularPoints(transformPoint1, transformPoint2, 20.0
                    * PERPENDICULAR_THRESHOLD)) {
                continue;
            }
            assertFalse(conic.arePerpendicularPoints(transformPoint1, transformPoint2,
                    20.0 * PERPENDICULAR_THRESHOLD));
            numValid++;
        }
        assertTrue(numValid > 0);
    }

    @Test
    void testGetDualConic() throws WrongSizeException, DecomposerException, RankDeficientMatrixException,
            IllegalArgumentException, NonSymmetricMatrixException, DualConicNotAvailableException {

        var transformMatrix = Matrix.createWithUniformRandomValues(HOM_COORDS, HOM_COORDS, MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        while (com.irurueta.algebra.Utils.rank(transformMatrix) != 3) {
            transformMatrix = Matrix.createWithUniformRandomValues(HOM_COORDS, HOM_COORDS, MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
        }

        final var transformTransposedMatrix = transformMatrix.transposeAndReturnNew();

        final var conicMatrix = transformTransposedMatrix.multiplyAndReturnNew(transformMatrix);

        final var dualConicMatrix = com.irurueta.algebra.Utils.inverse(conicMatrix);

        final var conic = new Conic(conicMatrix);
        final var dualConic = conic.getDualConic();
        final var dualConic2 = new DualConic();
        conic.dualConic(dualConic2);

        final var dualConicMatrix2 = dualConic.asMatrix();
        assertEquals(dualConic.asMatrix(), dualConic2.asMatrix());

        // normalize dual conic matrices
        var norm = com.irurueta.algebra.Utils.normF(dualConicMatrix);
        dualConicMatrix.multiplyByScalar(1.0 / norm);

        norm = com.irurueta.algebra.Utils.normF(dualConicMatrix2);
        dualConicMatrix2.multiplyByScalar(1.0 / norm);

        // compute difference of normalized dual conic matrices
        final var diffMatrix = dualConicMatrix.subtractAndReturnNew(dualConicMatrix2);

        // ensure that difference matrix is almost zero by checking its norm
        norm = com.irurueta.algebra.Utils.normF(diffMatrix);
        assertEquals(0.0, norm, PRECISION_ERROR);
    }

    @Test
    void testGetConicType() {
        // passing a circle conic
        var a = 3.0;
        var b = 0.0;
        var c = 3.0;
        var d = 3.0;
        var e = 2.0;
        var f = 1.0;
        final var conic = new Conic();
        conic.setParameters(a, b, c, d, e, f);
        assertEquals(ConicType.CIRCLE_CONIC_TYPE, conic.getConicType());

        // passing a circle conic
        a = 2.0;
        b = 1.0;
        conic.setParameters(a, b, c, d, e, f);
        assertEquals(ConicType.ELLIPSE_CONIC_TYPE, conic.getConicType());

        // passing a parabola conic
        a = 4.0;
        b = 2.0;
        c = 1.0;
        conic.setParameters(a, b, c, d, e, f);
        assertEquals(ConicType.PARABOLA_CONIC_TYPE, conic.getConicType());

        // passing a hyperbola conic
        a = 2.0;
        b = 5.0;
        conic.setParameters(a, b, c, d, e, f);
        assertEquals(ConicType.HYPERBOLA_CONIC_TYPE, conic.getConicType());

        // passing a rectangular hyperbola conic
        a = -1.0;
        conic.setParameters(a, b, c, d, e, f);
        assertEquals(ConicType.RECTANGULAR_HYPERBOLA_CONIC_TYPE, conic.getConicType());
    }

    @Test
    void testNormalize() throws WrongSizeException, IllegalArgumentException, NonSymmetricMatrixException {

        final var t = Matrix.createWithUniformRandomValues(HOM_COORDS, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var transT = t.transposeAndReturnNew();

        // make symmetric matrix
        final var conicMatrix = transT.multiplyAndReturnNew(t);

        final var conic = new Conic(conicMatrix);
        assertFalse(conic.isNormalized());

        // normalize conic
        conic.normalize();
        assertTrue(conic.isNormalized());

        // return conic as matrix
        final var conicMatrix2 = conic.asMatrix();

        // compare that both matrices are equal up to scale, for that reason we
        // first normalize both matrices
        var norm = com.irurueta.algebra.Utils.normF(conicMatrix);
        conicMatrix.multiplyByScalar(1.0 / norm);

        norm = com.irurueta.algebra.Utils.normF(conicMatrix2);
        conicMatrix2.multiplyByScalar(1.0 / norm);

        // compute their difference
        final var diffMatrix = conicMatrix.subtractAndReturnNew(conicMatrix2);

        // finally, ensure that the norm of the difference matrix is almost zero
        // up to machine precision
        norm = com.irurueta.algebra.Utils.normF(diffMatrix);
        assertEquals(0.0, norm, PRECISION_ERROR);

        // check that when setting new values conic becomes non-normalized
        final var randomizer = new UniformRandomizer();
        final var value = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        conic.setA(value);
        assertFalse(conic.isNormalized());

        conic.normalize();
        assertTrue(conic.isNormalized());
        conic.setB(value);
        assertFalse(conic.isNormalized());

        conic.normalize();
        assertTrue(conic.isNormalized());
        conic.setC(value);
        assertFalse(conic.isNormalized());

        conic.normalize();
        assertTrue(conic.isNormalized());
        conic.setD(value);
        assertFalse(conic.isNormalized());

        conic.normalize();
        assertTrue(conic.isNormalized());
        conic.setE(value);
        assertFalse(conic.isNormalized());

        conic.normalize();
        assertTrue(conic.isNormalized());
        conic.setF(value);
        assertFalse(conic.isNormalized());

        conic.normalize();
        assertTrue(conic.isNormalized());
        conic.setParameters(value, value, value, value, value, value);
        assertFalse(conic.isNormalized());

        conic.normalize();
        assertTrue(conic.isNormalized());
        conic.setParameters(conicMatrix);
        assertFalse(conic.isNormalized());

        // when setting all values to zero, attempting to normalize has no effect
        conic.setParameters(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        assertFalse(conic.isNormalized());
        conic.normalize();
        assertFalse(conic.isNormalized());
    }

    @Test
    void testGetTangentLineAt() throws NotLocusException {
        for (var t = 0; t < TIMES; t++) {
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
            final var conic = circle.toConic();

            final var point = new HomogeneousPoint2D(
                    center.getInhomX() + radius * Math.cos(theta),
                    center.getInhomY() + radius * Math.sin(theta), 1.0);
            point.normalize();

            assertTrue(circle.isLocus(point));
            assertTrue(conic.isLocus(point));

            // find tangent line at locus point
            final var line = circle.getTangentLineAt(point);
            final var line2 = conic.getTangentLineAt(point);

            // check that point is also at lines' locus
            assertTrue(line.isLocus(point));
            assertTrue(line2.isLocus(point));

            assertTrue(line.equals(line2, ABSOLUTE_ERROR));

            // check that lines angle is equal to theta
            final var lineAngle1 = line.getAngle();
            final var lineAngle2 = line2.getAngle();
            var theta3 = theta2 - Math.PI / 2.0;
            if (theta3 < -Math.PI / 2.0) {
                theta3 += Math.PI;
            } else if (theta3 > Math.PI / 2.0) {
                theta3 -= Math.PI;
            }
            assertEquals(lineAngle1 * 180.0 / Math.PI, theta3 * 180.0 / Math.PI, ABSOLUTE_ERROR);
            assertEquals(lineAngle2 * 180.0 / Math.PI, theta3 * 180.0 / Math.PI, ABSOLUTE_ERROR);
        }
    }

    @Test
    void testCreateCanonicalAbsoluteConic() throws WrongSizeException {
        final var ac = Conic.createCanonicalAbsoluteConic();

        assertEquals(1.0, ac.getA(), 0.0);
        assertEquals(0.0, ac.getB(), 0.0);
        assertEquals(1.0, ac.getC(), 0.0);
        assertEquals(0.0, ac.getD(), 0.0);
        assertEquals(0.0, ac.getE(), 0.0);
        assertEquals(1.0, ac.getF(), 0.0);

        assertEquals(Matrix.identity(3, 3), ac.asMatrix());
    }

    @Test
    void testSetParametersFromPoints() throws WrongSizeException, DecomposerException, CoincidentPointsException {
        var m = Matrix.createWithUniformRandomValues(5, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        HomogeneousPoint2D point1 = new HomogeneousPoint2D(m.getElementAt(0, 0),
                m.getElementAt(0, 1), m.getElementAt(0, 2));
        HomogeneousPoint2D point2 = new HomogeneousPoint2D(m.getElementAt(1, 0),
                m.getElementAt(1, 1), m.getElementAt(1, 2));
        HomogeneousPoint2D point3 = new HomogeneousPoint2D(m.getElementAt(2, 0),
                m.getElementAt(2, 1), m.getElementAt(2, 2));
        HomogeneousPoint2D point4 = new HomogeneousPoint2D(m.getElementAt(3, 0),
                m.getElementAt(3, 1), m.getElementAt(3, 2));
        HomogeneousPoint2D point5 = new HomogeneousPoint2D(m.getElementAt(4, 0),
                m.getElementAt(4, 1), m.getElementAt(4, 2));

        // estimate conic that lies inside provided 5 homogeneous 2D
        // points
        var conicMatrix = new Matrix(5, 6);
        var x = point1.getHomX();
        var y = point1.getHomY();
        var w = point1.getHomW();
        conicMatrix.setElementAt(0, 0, x * x);
        conicMatrix.setElementAt(0, 1, x * y);
        conicMatrix.setElementAt(0, 2, y * y);
        conicMatrix.setElementAt(0, 3, x * w);
        conicMatrix.setElementAt(0, 4, y * w);
        conicMatrix.setElementAt(0, 5, w * w);
        x = point2.getHomX();
        y = point2.getHomY();
        w = point2.getHomW();
        conicMatrix.setElementAt(1, 0, x * x);
        conicMatrix.setElementAt(1, 1, x * y);
        conicMatrix.setElementAt(1, 2, y * y);
        conicMatrix.setElementAt(1, 3, x * w);
        conicMatrix.setElementAt(1, 4, y * w);
        conicMatrix.setElementAt(1, 5, w * w);
        x = point3.getHomX();
        y = point3.getHomY();
        w = point3.getHomW();
        conicMatrix.setElementAt(2, 0, x * x);
        conicMatrix.setElementAt(2, 1, x * y);
        conicMatrix.setElementAt(2, 2, y * y);
        conicMatrix.setElementAt(2, 3, x * w);
        conicMatrix.setElementAt(2, 4, y * w);
        conicMatrix.setElementAt(2, 5, w * w);
        x = point4.getHomX();
        y = point4.getHomY();
        w = point4.getHomW();
        conicMatrix.setElementAt(3, 0, x * x);
        conicMatrix.setElementAt(3, 1, x * y);
        conicMatrix.setElementAt(3, 2, y * y);
        conicMatrix.setElementAt(3, 3, x * w);
        conicMatrix.setElementAt(3, 4, y * w);
        conicMatrix.setElementAt(3, 5, w * w);
        x = point5.getHomX();
        y = point5.getHomY();
        w = point5.getHomW();
        conicMatrix.setElementAt(4, 0, x * x);
        conicMatrix.setElementAt(4, 1, x * y);
        conicMatrix.setElementAt(4, 2, y * y);
        conicMatrix.setElementAt(4, 3, x * w);
        conicMatrix.setElementAt(4, 4, y * w);
        conicMatrix.setElementAt(4, 5, w * w);

        while (com.irurueta.algebra.Utils.rank(conicMatrix) < 5) {
            m = Matrix.createWithUniformRandomValues(5, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            point1 = new HomogeneousPoint2D(m.getElementAt(0, 0), m.getElementAt(0, 1),
                    m.getElementAt(0, 2));
            point2 = new HomogeneousPoint2D(m.getElementAt(1, 0), m.getElementAt(1, 1),
                    m.getElementAt(1, 2));
            point3 = new HomogeneousPoint2D(m.getElementAt(2, 0), m.getElementAt(2, 1),
                    m.getElementAt(2, 2));
            point4 = new HomogeneousPoint2D(m.getElementAt(3, 0), m.getElementAt(3, 1),
                    m.getElementAt(3, 2));
            point5 = new HomogeneousPoint2D(m.getElementAt(4, 0), m.getElementAt(4, 1),
                    m.getElementAt(4, 2));

            conicMatrix = new Matrix(5, 6);
            x = point1.getHomX();
            y = point1.getHomY();
            w = point1.getHomW();
            conicMatrix.setElementAt(0, 0, x * x);
            conicMatrix.setElementAt(0, 1, x * y);
            conicMatrix.setElementAt(0, 2, y * y);
            conicMatrix.setElementAt(0, 3, x * w);
            conicMatrix.setElementAt(0, 4, y * w);
            conicMatrix.setElementAt(0, 5, w * w);
            x = point2.getHomX();
            y = point2.getHomY();
            w = point2.getHomW();
            conicMatrix.setElementAt(1, 0, x * x);
            conicMatrix.setElementAt(1, 1, x * y);
            conicMatrix.setElementAt(1, 2, y * y);
            conicMatrix.setElementAt(1, 3, x * w);
            conicMatrix.setElementAt(1, 4, y * w);
            conicMatrix.setElementAt(1, 5, w * w);
            x = point3.getHomX();
            y = point3.getHomY();
            w = point3.getHomW();
            conicMatrix.setElementAt(2, 0, x * x);
            conicMatrix.setElementAt(2, 1, x * y);
            conicMatrix.setElementAt(2, 2, y * y);
            conicMatrix.setElementAt(2, 3, x * w);
            conicMatrix.setElementAt(2, 4, y * w);
            conicMatrix.setElementAt(2, 5, w * w);
            x = point4.getHomX();
            y = point4.getHomY();
            w = point4.getHomW();
            conicMatrix.setElementAt(3, 0, x * x);
            conicMatrix.setElementAt(3, 1, x * y);
            conicMatrix.setElementAt(3, 2, y * y);
            conicMatrix.setElementAt(3, 3, x * w);
            conicMatrix.setElementAt(3, 4, y * w);
            conicMatrix.setElementAt(3, 5, w * w);
            x = point5.getHomX();
            y = point5.getHomY();
            w = point5.getHomW();
            conicMatrix.setElementAt(4, 0, x * x);
            conicMatrix.setElementAt(4, 1, x * y);
            conicMatrix.setElementAt(4, 2, y * y);
            conicMatrix.setElementAt(4, 3, x * w);
            conicMatrix.setElementAt(4, 4, y * w);
            conicMatrix.setElementAt(4, 5, w * w);
        }

        final var conic = new Conic();
        conic.setParametersFromPoints(point1, point2, point3, point4, point5);
        assertTrue(conic.isLocus(point1, LOCUS_THRESHOLD));
        assertTrue(conic.isLocus(point2, LOCUS_THRESHOLD));
        assertTrue(conic.isLocus(point3, LOCUS_THRESHOLD));
        assertTrue(conic.isLocus(point4, LOCUS_THRESHOLD));
        assertTrue(conic.isLocus(point5, LOCUS_THRESHOLD));

        // Force CoincidentPointsException
        final var finalPoint1 = point1;
        final var finalPoint2 = point2;
        final var finalPoint3 = point3;
        final var finalPoint4 = point4;
        assertThrows(CoincidentPointsException.class, () -> conic.setParametersFromPoints(finalPoint1, finalPoint2,
                finalPoint3, finalPoint4, finalPoint4));
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var randomizer = new UniformRandomizer();

        var a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var e = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var f = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var conic1 = new Conic(a, b, c, d, e, f);

        // check
        assertEquals(a, conic1.getA(), 0.0);
        assertEquals(b, conic1.getB(), 0.0);
        assertEquals(c, conic1.getC(), 0.0);
        assertEquals(d, conic1.getD(), 0.0);
        assertEquals(e, conic1.getE(), 0.0);
        assertEquals(f, conic1.getF(), 0.0);
        assertFalse(conic1.isNormalized());

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(conic1);
        final var conic2 = SerializationHelper.<Conic>deserialize(bytes);

        // check
        assertEquals(conic1.getA(), conic2.getA(), 0.0);
        assertEquals(conic1.getB(), conic2.getB(), 0.0);
        assertEquals(conic1.getC(), conic2.getC(), 0.0);
        assertEquals(conic1.getD(), conic2.getD(), 0.0);
        assertEquals(conic1.getE(), conic2.getE(), 0.0);
        assertEquals(conic1.getF(), conic2.getF(), 0.0);
        assertEquals(conic1.isNormalized(), conic2.isNormalized());
    }
}
