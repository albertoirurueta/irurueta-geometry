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
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class ConicTest {

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
    public void testConstructor() throws WrongSizeException,
            IllegalArgumentException, NonSymmetricMatrixException,
            DecomposerException, CoincidentPointsException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // Constructor
        Conic conic = new Conic();
        assertEquals(conic.getA(), 0.0, 0.0);
        assertEquals(conic.getB(), 0.0, 0.0);
        assertEquals(conic.getC(), 0.0, 0.0);
        assertEquals(conic.getD(), 0.0, 0.0);
        assertEquals(conic.getE(), 0.0, 0.0);
        assertEquals(conic.getF(), 0.0, 0.0);
        assertFalse(conic.isNormalized());

        // Constructor with params
        double a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double e = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double f = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        conic = new Conic(a, b, c, d, e, f);
        assertEquals(conic.getA(), a, 0.0);
        assertEquals(conic.getB(), b, 0.0);
        assertEquals(conic.getC(), c, 0.0);
        assertEquals(conic.getD(), d, 0.0);
        assertEquals(conic.getE(), e, 0.0);
        assertEquals(conic.getF(), f, 0.0);
        assertFalse(conic.isNormalized());

        // Constructor using matrix
        Matrix m = new Matrix(CONIC_ROWS, CONIC_COLS);
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

        assertEquals(conic.getA(), m.getElementAt(0, 0), 0.0);
        assertEquals(conic.getB(), m.getElementAt(0, 1), 0.0);
        assertEquals(conic.getC(), m.getElementAt(1, 1), 0.0);
        assertEquals(conic.getD(), m.getElementAt(0, 2), 0.0);
        assertEquals(conic.getE(), m.getElementAt(1, 2), 0.0);
        assertEquals(conic.getF(), m.getElementAt(2, 2), 0.0);

        // Constructor using matrix with wrong size exception
        m = new Matrix(CONIC_ROWS, CONIC_COLS + 1);
        conic = null;
        try {
            conic = new Conic(m);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(conic);

        // Constructor using non-symmetric matrix
        m = new Matrix(CONIC_ROWS, CONIC_COLS);
        m.setElementAt(0, 0, a);
        m.setElementAt(0, 1, b);
        m.setElementAt(0, 2, d);
        m.setElementAt(1, 0, b + 1.0);
        m.setElementAt(1, 1, c);
        m.setElementAt(1, 2, e + 1.0);
        m.setElementAt(2, 0, d + 1.0);
        m.setElementAt(2, 1, e);
        m.setElementAt(2, 2, f);

        conic = null;
        try {
            conic = new Conic(m);
            fail("NonSymmetricMatrixException expected but not thrown");
        } catch (final NonSymmetricMatrixException ignore) {
        }
        assertNull(conic);

        // Constructor from 5 points
        m = Matrix.createWithUniformRandomValues(5, HOM_COORDS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        HomogeneousPoint2D point1 = new HomogeneousPoint2D(
                m.getElementAt(0, 0), m.getElementAt(0, 1),
                m.getElementAt(0, 2));
        HomogeneousPoint2D point2 = new HomogeneousPoint2D(
                m.getElementAt(1, 0), m.getElementAt(1, 1),
                m.getElementAt(1, 2));
        HomogeneousPoint2D point3 = new HomogeneousPoint2D(
                m.getElementAt(2, 0), m.getElementAt(2, 1),
                m.getElementAt(2, 2));
        HomogeneousPoint2D point4 = new HomogeneousPoint2D(
                m.getElementAt(3, 0), m.getElementAt(3, 1),
                m.getElementAt(3, 2));
        HomogeneousPoint2D point5 = new HomogeneousPoint2D(
                m.getElementAt(4, 0), m.getElementAt(4, 1),
                m.getElementAt(4, 2));

        // estimate conic that lies inside of provided 5 homogeneous 2D
        // points
        Matrix conicMatrix = new Matrix(5, 6);
        double x = point1.getHomX();
        double y = point1.getHomY();
        double w = point1.getHomW();
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
            m = Matrix.createWithUniformRandomValues(5, HOM_COORDS,
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            point1 = new HomogeneousPoint2D(
                    m.getElementAt(0, 0), m.getElementAt(0, 1),
                    m.getElementAt(0, 2));
            point2 = new HomogeneousPoint2D(
                    m.getElementAt(1, 0), m.getElementAt(1, 1),
                    m.getElementAt(1, 2));
            point3 = new HomogeneousPoint2D(
                    m.getElementAt(2, 0), m.getElementAt(2, 1),
                    m.getElementAt(2, 2));
            point4 = new HomogeneousPoint2D(
                    m.getElementAt(3, 0), m.getElementAt(3, 1),
                    m.getElementAt(3, 2));
            point5 = new HomogeneousPoint2D(
                    m.getElementAt(4, 0), m.getElementAt(4, 1),
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

        conic = new Conic(point1, point2, point3, point4, point5);
        assertTrue(conic.isLocus(point1, LOCUS_THRESHOLD));
        assertTrue(conic.isLocus(point2, LOCUS_THRESHOLD));
        assertTrue(conic.isLocus(point3, LOCUS_THRESHOLD));
        assertTrue(conic.isLocus(point4, LOCUS_THRESHOLD));
        assertTrue(conic.isLocus(point5, LOCUS_THRESHOLD));

        // Force CoincidentPointsException
        conic = null;
        try {
            conic = new Conic(point1, point2, point3, point4, point4);
            fail("CoincidentPointsException expected but not thrown");
        } catch (final CoincidentPointsException ignore) {
        }
        assertNull(conic);
    }

    @Test
    public void testGettersAndSetters() throws WrongSizeException,
            IllegalArgumentException, NonSymmetricMatrixException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final Conic conic = new Conic();
        double a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double e = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double f = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
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

        Matrix m = new Matrix(CONIC_ROWS, CONIC_COLS);
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
        m = new Matrix(CONIC_ROWS + 1, CONIC_COLS + 1);
        try {
            conic.setParameters(m);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        // Force NonSymmetricMatrixException
        m = new Matrix(CONIC_ROWS, CONIC_COLS);
        a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        e = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        f = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        m.setElementAt(0, 0, a);
        m.setElementAt(0, 1, b);
        m.setElementAt(0, 2, d);
        m.setElementAt(1, 0, b + 1.0);
        m.setElementAt(1, 1, c);
        m.setElementAt(1, 2, e);
        m.setElementAt(2, 0, d + 1.0);
        m.setElementAt(2, 1, e + 1.0);
        m.setElementAt(2, 2, f);
        try {
            conic.setParameters(m);
            fail("NonSymmetricMatrixException expected but not thrown");
        } catch (final NonSymmetricMatrixException ignore) {
        }
        conic.setParameters(m, 1.0);
    }

    @Test
    public void testAsMatrix() throws WrongSizeException,
            IllegalArgumentException, NonSymmetricMatrixException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Conic conic = new Conic();
        final Matrix m = new Matrix(CONIC_ROWS, CONIC_COLS);
        final double a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double e = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double f = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
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

        final Matrix m2 = conic.asMatrix();

        assertTrue(m.equals(m2));
    }

    @Test
    public void testIsLocus() throws WrongSizeException, DecomposerException,
            IllegalArgumentException, NonSymmetricMatrixException,
            CoincidentPointsException, NotReadyException, LockedException,
            com.irurueta.algebra.NotAvailableException {

        Matrix m = Matrix.createWithUniformRandomValues(5, HOM_COORDS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        HomogeneousPoint2D point1 = new HomogeneousPoint2D(
                m.getElementAt(0, 0), m.getElementAt(0, 1),
                m.getElementAt(0, 2));
        HomogeneousPoint2D point2 = new HomogeneousPoint2D(
                m.getElementAt(1, 0), m.getElementAt(1, 1),
                m.getElementAt(1, 2));
        HomogeneousPoint2D point3 = new HomogeneousPoint2D(
                m.getElementAt(2, 0), m.getElementAt(2, 1),
                m.getElementAt(2, 2));
        HomogeneousPoint2D point4 = new HomogeneousPoint2D(
                m.getElementAt(3, 0), m.getElementAt(3, 1),
                m.getElementAt(3, 2));
        HomogeneousPoint2D point5 = new HomogeneousPoint2D(
                m.getElementAt(4, 0), m.getElementAt(4, 1),
                m.getElementAt(4, 2));

        point1.normalize();
        point2.normalize();
        point3.normalize();
        point4.normalize();
        point5.normalize();

        // estimate conic that lies inside of provided 5 inhomogeneous image
        // points
        Matrix conicMatrix = new Matrix(5, 6);
        double x = point1.getHomX();
        double y = point1.getHomY();
        double w = point1.getHomW();
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
            m = Matrix.createWithUniformRandomValues(5, HOM_COORDS,
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            point1 = new HomogeneousPoint2D(
                    m.getElementAt(0, 0), m.getElementAt(0, 1),
                    m.getElementAt(0, 2));
            point2 = new HomogeneousPoint2D(
                    m.getElementAt(1, 0), m.getElementAt(1, 1),
                    m.getElementAt(1, 2));
            point3 = new HomogeneousPoint2D(
                    m.getElementAt(2, 0), m.getElementAt(2, 1),
                    m.getElementAt(2, 2));
            point4 = new HomogeneousPoint2D(
                    m.getElementAt(3, 0), m.getElementAt(3, 1),
                    m.getElementAt(3, 2));
            point5 = new HomogeneousPoint2D(
                    m.getElementAt(4, 0), m.getElementAt(4, 1),
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

        final SingularValueDecomposer decomposer =
                new SingularValueDecomposer(conicMatrix);
        decomposer.decompose();

        final Matrix v = decomposer.getV();

        final double a = v.getElementAt(0, 5);
        final double b = v.getElementAt(1, 5);
        final double c = v.getElementAt(2, 5);
        final double d = v.getElementAt(3, 5);
        final double e = v.getElementAt(4, 5);
        final double f = v.getElementAt(5, 5);

        final Matrix conicPoint = new Matrix(CONIC_ROWS, 1);
        conicPoint.setElementAt(0, 0, point1.getHomX());
        conicPoint.setElementAt(1, 0, point1.getHomY());
        conicPoint.setElementAt(2, 0, point1.getHomW());

        double norm = com.irurueta.algebra.Utils.normF(conicPoint);
        conicPoint.multiplyByScalar(1.0 / norm);

        final Matrix resultConicMatrix = new Matrix(CONIC_ROWS, CONIC_COLS);
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
        final Matrix lineParams = resultConicMatrix.multiplyAndReturnNew(conicPoint);

        norm = com.irurueta.algebra.Utils.normF(lineParams);
        lineParams.multiplyByScalar(1.0 / norm);

        // use director vector of tangent line to find a point outside conic
        double directVectorA = lineParams.getElementAt(0, 0);
        double directVectorB = lineParams.getElementAt(1, 0);
        final double directVectorNorm = Math.sqrt(directVectorA * directVectorA +
                directVectorB * directVectorB);
        directVectorA /= directVectorNorm;
        directVectorB /= directVectorNorm;
        final InhomogeneousPoint2D outsidePoint =
                new InhomogeneousPoint2D(
                        point1.getInhomX() + directVectorA,
                        point1.getInhomY() + directVectorB);

        // instantiate new conic instance
        final Conic conic = new Conic(resultConicMatrix);

        // check that initial 5 points lie inside the conic
        assertTrue(conic.isLocus(point1, LOCUS_THRESHOLD));
        assertTrue(conic.isLocus(point2, LOCUS_THRESHOLD));
        assertTrue(conic.isLocus(point3, LOCUS_THRESHOLD));
        assertTrue(conic.isLocus(point4, LOCUS_THRESHOLD));
        assertTrue(conic.isLocus(point5, LOCUS_THRESHOLD));

        // check point outside of conic
        assertFalse(conic.isLocus(outsidePoint, LOCUS_THRESHOLD));

        // test constructor from 5 points
        final Conic conic2 = new Conic(point1, point2, point3, point4, point5);
        assertTrue(conic2.isLocus(point1, LOCUS_THRESHOLD));
        assertTrue(conic2.isLocus(point2, LOCUS_THRESHOLD));
        assertTrue(conic2.isLocus(point3, LOCUS_THRESHOLD));
        assertTrue(conic2.isLocus(point4, LOCUS_THRESHOLD));
        assertTrue(conic2.isLocus(point5, LOCUS_THRESHOLD));
    }

    @Test
    public void testAngleBetweenPoints() throws WrongSizeException,
            IllegalArgumentException, NonSymmetricMatrixException {

        // initial 2D points
        final Matrix point1Matrix = Matrix.createWithUniformRandomValues(HOM_COORDS,
                1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final Matrix point2Matrix = Matrix.createWithUniformRandomValues(HOM_COORDS,
                1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // transformation matrix
        final Matrix transform = Matrix.createWithUniformRandomValues(HOM_COORDS,
                HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // transform points
        final Matrix tPoint1 = transform.multiplyAndReturnNew(point1Matrix);
        final Matrix tPoint2 = transform.multiplyAndReturnNew(point2Matrix);

        // compute angle of transformed points
        final double norm1 = com.irurueta.algebra.Utils.normF(tPoint1);
        final double norm2 = com.irurueta.algebra.Utils.normF(tPoint2);

        final double numerator = tPoint1.transposeAndReturnNew().multiplyAndReturnNew(
                tPoint2).getElementAt(0, 0);

        final double cosAngle = numerator / (norm1 * norm2);

        final double angle = Math.acos(cosAngle);

        // compute conic matrix as the product of transposed transform matrix
        // with itself
        final Matrix transposedTransform = transform.transposeAndReturnNew();
        final Matrix conicMatrix = transposedTransform.multiplyAndReturnNew(
                transform);

        // normalize conic matrix
        final double normConic = com.irurueta.algebra.Utils.normF(conicMatrix);
        conicMatrix.multiplyByScalar(1.0 / normConic);

        final Conic conic = new Conic(conicMatrix);

        final Point2D point1 = new HomogeneousPoint2D(point1Matrix.getElementAt(0, 0),
                point1Matrix.getElementAt(1, 0),
                point1Matrix.getElementAt(2, 0));
        final Point2D point2 = new HomogeneousPoint2D(point2Matrix.getElementAt(0, 0),
                point2Matrix.getElementAt(1, 0),
                point2Matrix.getElementAt(2, 0));

        assertEquals(conic.angleBetweenPoints(point1, point2), angle,
                PRECISION_ERROR);
    }

    @Test
    public void testArePerpendicularPoints() throws WrongSizeException,
            DecomposerException, RankDeficientMatrixException,
            IllegalArgumentException, NonSymmetricMatrixException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            Matrix matrixPoint1 = Matrix.createWithUniformRandomValues(
                    HOM_COORDS, 1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            double norm = com.irurueta.algebra.Utils.normF(matrixPoint1);
            matrixPoint1.multiplyByScalar(1.0 / norm);

            Matrix matrixPoint2 = Matrix.createWithUniformRandomValues(HOM_COORDS,
                    1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            matrixPoint2.setElementAt(0, 0, matrixPoint1.getElementAt(1, 0) +
                    matrixPoint1.getElementAt(2, 0));
            matrixPoint2.setElementAt(1, 0, -matrixPoint1.getElementAt(0, 0));
            matrixPoint2.setElementAt(2, 0, -matrixPoint1.getElementAt(0, 0));

            norm = com.irurueta.algebra.Utils.normF(matrixPoint2);
            matrixPoint2.multiplyByScalar(1.0 / norm);

            Matrix transform = Matrix.createWithUniformRandomValues(HOM_COORDS,
                    HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            while (com.irurueta.algebra.Utils.rank(transform) < 3) {
                transform = Matrix.createWithUniformRandomValues(HOM_COORDS,
                        HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            }

            final Matrix invTransform = com.irurueta.algebra.Utils.inverse(transform);

            Matrix transformPointMatrix1 = transform.multiplyAndReturnNew(
                    matrixPoint1);
            Matrix transformPointMatrix2 = transform.multiplyAndReturnNew(
                    matrixPoint2);

            final Matrix transInvTransform = invTransform.transposeAndReturnNew();

            final Matrix conicMatrix = transInvTransform.multiplyAndReturnNew(
                    invTransform);
            norm = com.irurueta.algebra.Utils.normF(conicMatrix);
            conicMatrix.multiplyByScalar(1.0 / norm);

            Point2D transformPoint1 = new HomogeneousPoint2D(
                    transformPointMatrix1.toArray());
            Point2D transformPoint2 = new HomogeneousPoint2D(
                    transformPointMatrix2.toArray());

            final Conic conic = new Conic(conicMatrix);

            assertTrue(conic.arePerpendicularPoints(transformPoint1,
                    transformPoint2, PERPENDICULAR_THRESHOLD));

            // trying non-perpendicular points
            matrixPoint1 = Matrix.createWithUniformRandomValues(HOM_COORDS, 1,
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            norm = com.irurueta.algebra.Utils.normF(matrixPoint1);
            matrixPoint1.multiplyByScalar(1.0 / norm);

            matrixPoint2 = Matrix.createWithUniformRandomValues(HOM_COORDS, 1,
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            norm = com.irurueta.algebra.Utils.normF(matrixPoint2);
            matrixPoint2.multiplyByScalar(1.0 / norm);

            double dotProduct = matrixPoint1.transposeAndReturnNew().
                    multiplyAndReturnNew(matrixPoint2).getElementAt(0, 0);

            // ensure points are not perpendicular
            while (Math.abs(dotProduct) < PERPENDICULAR_THRESHOLD) {
                matrixPoint1 = Matrix.createWithUniformRandomValues(HOM_COORDS, 1,
                        MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                norm = com.irurueta.algebra.Utils.normF(matrixPoint1);
                matrixPoint1.multiplyByScalar(1.0 / norm);

                matrixPoint2 = Matrix.createWithUniformRandomValues(HOM_COORDS, 1,
                        MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                norm = com.irurueta.algebra.Utils.normF(matrixPoint2);
                matrixPoint2.multiplyByScalar(1.0 / norm);

                dotProduct = matrixPoint1.transposeAndReturnNew().
                        multiplyAndReturnNew(matrixPoint2).getElementAt(0, 0);
            }

            transformPointMatrix1 = transform.multiplyAndReturnNew(matrixPoint1);
            transformPointMatrix2 = transform.multiplyAndReturnNew(matrixPoint2);

            transformPoint1 = new HomogeneousPoint2D(
                    transformPointMatrix1.toArray());
            transformPoint2 = new HomogeneousPoint2D(
                    transformPointMatrix2.toArray());

            if (conic.arePerpendicularPoints(transformPoint1, transformPoint2, 20.0 * PERPENDICULAR_THRESHOLD)) {
                continue;
            }
            assertFalse(conic.arePerpendicularPoints(transformPoint1,
                    transformPoint2, 20.0 * PERPENDICULAR_THRESHOLD));
            numValid++;
        }
        assertTrue(numValid > 0);
    }

    @Test
    public void testGetDualConic() throws WrongSizeException,
            DecomposerException, RankDeficientMatrixException,
            IllegalArgumentException, NonSymmetricMatrixException,
            DualConicNotAvailableException {

        Matrix transformMatrix = Matrix.createWithUniformRandomValues(
                HOM_COORDS, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        while (com.irurueta.algebra.Utils.rank(transformMatrix) != 3) {
            transformMatrix = Matrix.createWithUniformRandomValues(
                    HOM_COORDS, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        }

        final Matrix transformTransposedMatrix =
                transformMatrix.transposeAndReturnNew();

        final Matrix conicMatrix = transformTransposedMatrix.multiplyAndReturnNew(
                transformMatrix);

        final Matrix dualConicMatrix = com.irurueta.algebra.Utils.inverse(
                conicMatrix);

        final Conic conic = new Conic(conicMatrix);
        final DualConic dualConic = conic.getDualConic();
        final Matrix dualConicMatrix2 = dualConic.asMatrix();

        // normalize dual conic matrices
        double norm = com.irurueta.algebra.Utils.normF(dualConicMatrix);
        dualConicMatrix.multiplyByScalar(1.0 / norm);

        norm = com.irurueta.algebra.Utils.normF(dualConicMatrix2);
        dualConicMatrix2.multiplyByScalar(1.0 / norm);

        // compute difference of normalized dual conic matrices
        final Matrix diffMatrix = dualConicMatrix.subtractAndReturnNew(
                dualConicMatrix2);

        // ensure that difference matrix is almost zero by checking its norm
        norm = com.irurueta.algebra.Utils.normF(diffMatrix);
        assertEquals(norm, 0.0, PRECISION_ERROR);
    }

    @Test
    public void testGetConicType() {
        // passing a circle conic
        double a = 3.0;
        double b = 0.0;
        double c = 3.0;
        double d = 3.0;
        double e = 2.0;
        double f = 1.0;
        final Conic conic = new Conic();
        conic.setParameters(a, b, c, d, e, f);
        assertEquals(conic.getConicType(), ConicType.CIRCLE_CONIC_TYPE);

        // passing a circle conic
        a = 2.0;
        b = 1.0;
        c = 3.0;
        d = 3.0;
        e = 2.0;
        f = 1.0;
        conic.setParameters(a, b, c, d, e, f);
        assertEquals(conic.getConicType(), ConicType.ELLIPSE_CONIC_TYPE);

        // passing a parabola conic
        a = 4.0;
        b = 2.0;
        c = 1.0;
        d = 3.0;
        e = 2.0;
        f = 1.0;
        conic.setParameters(a, b, c, d, e, f);
        assertEquals(conic.getConicType(), ConicType.PARABOLA_CONIC_TYPE);

        // passing a hyperbola conic
        a = 2.0;
        b = 5.0;
        c = 1.0;
        d = 3.0;
        e = 2.0;
        f = 1.0;
        conic.setParameters(a, b, c, d, e, f);
        assertEquals(conic.getConicType(), ConicType.HYPERBOLA_CONIC_TYPE);

        // passing a rectangular hyperbola conic
        a = -1.0;
        b = 5.0;
        c = 1.0;
        d = 3.0;
        e = 2.0;
        f = 1.0;
        conic.setParameters(a, b, c, d, e, f);
        assertEquals(conic.getConicType(),
                ConicType.RECTANGULAR_HYPERBOLA_CONIC_TYPE);
    }

    @Test
    public void testNormalize() throws WrongSizeException,
            IllegalArgumentException, NonSymmetricMatrixException {

        final Matrix t = Matrix.createWithUniformRandomValues(HOM_COORDS, HOM_COORDS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final Matrix transT = t.transposeAndReturnNew();

        // make symmetric matrix
        final Matrix conicMatrix = transT.multiplyAndReturnNew(t);

        final Conic conic = new Conic(conicMatrix);
        assertFalse(conic.isNormalized());

        // normalize conic
        conic.normalize();
        assertTrue(conic.isNormalized());

        // return conic as matrix
        final Matrix conicMatrix2 = conic.asMatrix();

        // compare that both matrices are equal up to scale, for that reason we
        // first normalize both matrices
        double norm = com.irurueta.algebra.Utils.normF(conicMatrix);
        conicMatrix.multiplyByScalar(1.0 / norm);

        norm = com.irurueta.algebra.Utils.normF(conicMatrix2);
        conicMatrix2.multiplyByScalar(1.0 / norm);

        // compute their difference
        final Matrix diffMatrix = conicMatrix.subtractAndReturnNew(conicMatrix2);

        // finally, ensure that the norm of the difference matrix is almost zero
        // up to machine precision
        norm = com.irurueta.algebra.Utils.normF(diffMatrix);
        assertEquals(norm, 0.0, PRECISION_ERROR);

        // check that when setting new values conic becomes non-normalized
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double value = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
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
    public void testGetTangentLineAt() throws NotLocusException {
        for (int t = 0; t < TIMES; t++) {
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
            final Conic conic = circle.toConic();

            final Point2D point = new HomogeneousPoint2D(
                    center.getInhomX() + radius * Math.cos(theta),
                    center.getInhomY() + radius * Math.sin(theta), 1.0);
            point.normalize();

            assertTrue(circle.isLocus(point));
            assertTrue(conic.isLocus(point));

            // find tangent line at locus point
            final Line2D line = circle.getTangentLineAt(point);
            final Line2D line2 = conic.getTangentLineAt(point);

            // check that point is also at lines' locus
            assertTrue(line.isLocus(point));
            assertTrue(line2.isLocus(point));

            assertTrue(line.equals(line2, ABSOLUTE_ERROR));

            // check that lines angle is equal to theta
            final double lineAngle1 = line.getAngle();
            final double lineAngle2 = line2.getAngle();
            double theta3 = theta2 - Math.PI / 2.0;
            if (theta3 < -Math.PI / 2.0) {
                theta3 += Math.PI;
            } else if (theta3 > Math.PI / 2.0) {
                theta3 -= Math.PI;
            }
            assertEquals(lineAngle1 * 180.0 / Math.PI,
                    theta3 * 180.0 / Math.PI, ABSOLUTE_ERROR);
            assertEquals(lineAngle2 * 180.0 / Math.PI,
                    theta3 * 180.0 / Math.PI, ABSOLUTE_ERROR);
        }
    }

    @Test
    public void testCreateCanonicalAbsoluteConic() throws WrongSizeException {
        final Conic ac = Conic.createCanonicalAbsoluteConic();

        assertEquals(ac.getA(), 1.0, 0.0);
        assertEquals(ac.getB(), 0.0, 0.0);
        assertEquals(ac.getC(), 1.0, 0.0);
        assertEquals(ac.getD(), 0.0, 0.0);
        assertEquals(ac.getE(), 0.0, 0.0);
        assertEquals(ac.getF(), 1.0, 0.0);

        assertEquals(ac.asMatrix(), Matrix.identity(3, 3));
    }
}
