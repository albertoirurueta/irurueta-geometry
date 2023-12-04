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
import org.junit.*;

import java.io.IOException;
import java.util.Random;

import static org.junit.Assert.*;

public class DualConicTest {

    private static final double MIN_RANDOM_VALUE = -10.0;
    private static final double MAX_RANDOM_VALUE = 10.0;
    private static final double PRECISION_ERROR = 1e-8;
    private static final double LOCUS_THRESHOLD = 1e-8;
    private static final double PERPENDICULAR_THRESHOLD = 1e-6;

    private static final int DUAL_CONIC_ROWS = 3;
    private static final int DUAL_CONIC_COLS = 3;
    private static final int HOM_COORDS = 3;

    private static final int TIMES = 10;

    @Test
    public void testConstants() {
        assertEquals(3, BaseConic.BASECONIC_MATRIX_ROW_SIZE);
        assertEquals(3, BaseConic.BASECONIC_MATRIX_COLUMN_SIZE);
        assertEquals(6, BaseConic.N_PARAMS);
        assertEquals(1e-12, BaseConic.DEFAULT_LOCUS_THRESHOLD, 0.0);
        assertEquals(1e-12, BaseConic.DEFAULT_PERPENDICULAR_THRESHOLD, 0.0);
        assertEquals(0.0, BaseConic.MIN_THRESHOLD, 0.0);
    }

    @Test
    public void testConstructor() throws WrongSizeException,
            IllegalArgumentException, NonSymmetricMatrixException,
            DecomposerException, CoincidentLinesException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // Constructor
        DualConic dualConic = new DualConic();
        assertEquals(0.0, dualConic.getA(), 0.0);
        assertEquals(0.0, dualConic.getB(), 0.0);
        assertEquals(0.0, dualConic.getC(), 0.0);
        assertEquals(0.0, dualConic.getD(), 0.0);
        assertEquals(0.0, dualConic.getE(), 0.0);
        assertEquals(0.0, dualConic.getF(), 0.0);
        assertFalse(dualConic.isNormalized());

        // Constructor with params
        double a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double e = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double f = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        dualConic = new DualConic(a, b, c, d, e, f);
        assertEquals(a, dualConic.getA(), 0.0);
        assertEquals(b, dualConic.getB(), 0.0);
        assertEquals(c, dualConic.getC(), 0.0);
        assertEquals(d, dualConic.getD(), 0.0);
        assertEquals(e, dualConic.getE(), 0.0);
        assertEquals(f, dualConic.getF(), 0.0);
        assertFalse(dualConic.isNormalized());

        // Constructor using matrix
        Matrix m = new Matrix(DUAL_CONIC_ROWS, DUAL_CONIC_COLS);
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
        dualConic = new DualConic(m);
        assertEquals(m.getElementAt(0, 0), dualConic.getA(), 0.0);
        assertEquals(m.getElementAt(0, 1), dualConic.getB(), 0.0);
        assertEquals(m.getElementAt(1, 1), dualConic.getC(), 0.0);
        assertEquals(m.getElementAt(0, 2), dualConic.getD(), 0.0);
        assertEquals(m.getElementAt(1, 2), dualConic.getE(), 0.0);
        assertEquals(m.getElementAt(2, 2), dualConic.getF(), 0.0);

        // Constructor using matrix with wrong size
        m = new Matrix(DUAL_CONIC_ROWS + 1, DUAL_CONIC_COLS);
        dualConic = null;
        try {
            dualConic = new DualConic(m);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(dualConic);

        // Constructor using non-symmetric matrix
        m = new Matrix(DUAL_CONIC_ROWS, DUAL_CONIC_COLS);
        m.setElementAt(0, 0, a);
        m.setElementAt(0, 1, b);
        m.setElementAt(0, 2, d);
        m.setElementAt(1, 0, b + 1.0);
        m.setElementAt(1, 1, c);
        m.setElementAt(1, 2, e + 1.0);
        m.setElementAt(2, 0, d + 1.0);
        m.setElementAt(2, 1, e);
        m.setElementAt(2, 2, f);
        try {
            dualConic = new DualConic(m);
            fail("NonSymmetricMatrixException expected but not thrown");
        } catch (final NonSymmetricMatrixException ignore) {
        }
        assertNull(dualConic);

        // Constructor from 5 lines
        m = Matrix.createWithUniformRandomValues(5, HOM_COORDS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        Line2D line1 = new Line2D(m.getElementAt(0, 0), m.getElementAt(0, 1),
                m.getElementAt(0, 2));
        Line2D line2 = new Line2D(m.getElementAt(1, 0), m.getElementAt(1, 1),
                m.getElementAt(1, 2));
        Line2D line3 = new Line2D(m.getElementAt(2, 0), m.getElementAt(2, 1),
                m.getElementAt(2, 2));
        Line2D line4 = new Line2D(m.getElementAt(3, 0), m.getElementAt(3, 1),
                m.getElementAt(3, 2));
        Line2D line5 = new Line2D(m.getElementAt(4, 0), m.getElementAt(4, 1),
                m.getElementAt(4, 2));

        line1.normalize();
        line2.normalize();
        line3.normalize();
        line4.normalize();
        line5.normalize();

        // estimate dual conic that lies inside provided 5 lines
        final Matrix m2 = new Matrix(5, 6);

        double l1 = line1.getA();
        double l2 = line1.getB();
        double l3 = line1.getC();
        m2.setElementAt(0, 0, l1 * l1);
        m2.setElementAt(0, 1, 2.0 * l1 * l2);
        m2.setElementAt(0, 2, l2 * l2);
        m2.setElementAt(0, 3, 2.0 * l1 * l3);
        m2.setElementAt(0, 4, 2.0 * l2 * l3);
        m2.setElementAt(0, 5, l3 * l3);

        l1 = line2.getA();
        l2 = line2.getB();
        l3 = line2.getC();
        m2.setElementAt(1, 0, l1 * l1);
        m2.setElementAt(1, 1, 2.0 * l1 * l2);
        m2.setElementAt(1, 2, l2 * l2);
        m2.setElementAt(1, 3, 2.0 * l1 * l3);
        m2.setElementAt(1, 4, 2.0 * l2 * l3);
        m2.setElementAt(1, 5, l3 * l3);

        l1 = line3.getA();
        l2 = line3.getB();
        l3 = line3.getC();
        m2.setElementAt(2, 0, l1 * l1);
        m2.setElementAt(2, 1, 2.0 * l1 * l2);
        m2.setElementAt(2, 2, l2 * l2);
        m2.setElementAt(2, 3, 2.0 * l1 * l3);
        m2.setElementAt(2, 4, 2.0 * l2 * l3);
        m2.setElementAt(2, 5, l3 * l3);

        l1 = line4.getA();
        l2 = line4.getB();
        l3 = line4.getC();
        m2.setElementAt(3, 0, l1 * l1);
        m2.setElementAt(3, 1, 2.0 * l1 * l2);
        m2.setElementAt(3, 2, l2 * l2);
        m2.setElementAt(3, 3, 2.0 * l1 * l3);
        m2.setElementAt(3, 4, 2.0 * l2 * l3);
        m2.setElementAt(3, 5, l3 * l3);

        l1 = line5.getA();
        l2 = line5.getB();
        l3 = line5.getC();
        m2.setElementAt(4, 0, l1 * l1);
        m2.setElementAt(4, 1, 2.0 * l1 * l2);
        m2.setElementAt(4, 2, l2 * l2);
        m2.setElementAt(4, 3, 2.0 * l1 * l3);
        m2.setElementAt(4, 4, 2.0 * l2 * l3);
        m2.setElementAt(4, 5, l3 * l3);

        while (com.irurueta.algebra.Utils.rank(m2) < 5) {
            m = Matrix.createWithUniformRandomValues(5, HOM_COORDS,
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            line1 = new Line2D(m.getElementAt(0, 0), m.getElementAt(0, 1),
                    m.getElementAt(0, 2));
            line2 = new Line2D(m.getElementAt(1, 0), m.getElementAt(1, 1),
                    m.getElementAt(1, 2));
            line3 = new Line2D(m.getElementAt(2, 0), m.getElementAt(2, 1),
                    m.getElementAt(2, 2));
            line4 = new Line2D(m.getElementAt(3, 0), m.getElementAt(3, 1),
                    m.getElementAt(3, 2));
            line5 = new Line2D(m.getElementAt(4, 0), m.getElementAt(4, 1),
                    m.getElementAt(4, 2));

            line1.normalize();
            line2.normalize();
            line3.normalize();
            line4.normalize();
            line5.normalize();


            l1 = line1.getA();
            l2 = line1.getB();
            l3 = line1.getC();
            m2.setElementAt(0, 0, l1 * l1);
            m2.setElementAt(0, 1, 2.0 * l1 * l2);
            m2.setElementAt(0, 2, l2 * l2);
            m2.setElementAt(0, 3, 2.0 * l1 * l3);
            m2.setElementAt(0, 4, 2.0 * l2 * l3);
            m2.setElementAt(0, 5, l3 * l3);

            l1 = line2.getA();
            l2 = line2.getB();
            l3 = line2.getC();
            m2.setElementAt(1, 0, l1 * l1);
            m2.setElementAt(1, 1, 2.0 * l1 * l2);
            m2.setElementAt(1, 2, l2 * l2);
            m2.setElementAt(1, 3, 2.0 * l1 * l3);
            m2.setElementAt(1, 4, 2.0 * l2 * l3);
            m2.setElementAt(1, 5, l3 * l3);

            l1 = line3.getA();
            l2 = line3.getB();
            l3 = line3.getC();
            m2.setElementAt(2, 0, l1 * l1);
            m2.setElementAt(2, 1, 2.0 * l1 * l2);
            m2.setElementAt(2, 2, l2 * l2);
            m2.setElementAt(2, 3, 2.0 * l1 * l3);
            m2.setElementAt(2, 4, 2.0 * l2 * l3);
            m2.setElementAt(2, 5, l3 * l3);

            l1 = line4.getA();
            l2 = line4.getB();
            l3 = line4.getC();
            m2.setElementAt(3, 0, l1 * l1);
            m2.setElementAt(3, 1, 2.0 * l1 * l2);
            m2.setElementAt(3, 2, l2 * l2);
            m2.setElementAt(3, 3, 2.0 * l1 * l3);
            m2.setElementAt(3, 4, 2.0 * l2 * l3);
            m2.setElementAt(3, 5, l3 * l3);

            l1 = line5.getA();
            l2 = line5.getB();
            l3 = line5.getC();
            m2.setElementAt(4, 0, l1 * l1);
            m2.setElementAt(4, 1, 2.0 * l1 * l2);
            m2.setElementAt(4, 2, l2 * l2);
            m2.setElementAt(4, 3, 2.0 * l1 * l3);
            m2.setElementAt(4, 4, 2.0 * l2 * l3);
            m2.setElementAt(4, 5, l3 * l3);
        }

        dualConic = new DualConic(line1, line2, line3, line4, line5);
        assertTrue(dualConic.isLocus(line1, PRECISION_ERROR));
        assertTrue(dualConic.isLocus(line2, PRECISION_ERROR));
        assertTrue(dualConic.isLocus(line3, PRECISION_ERROR));
        assertTrue(dualConic.isLocus(line4, PRECISION_ERROR));
        assertTrue(dualConic.isLocus(line5, PRECISION_ERROR));

        // Force CoincidentLinesException
        dualConic = null;
        try {
            dualConic = new DualConic(line1, line2, line3, line4, line4);
            fail("CoincidentLinesException expected but not thrown");
        } catch (final CoincidentLinesException ignore) {
        }
        assertNull(dualConic);
    }

    @Test
    public void testGettersAndSetters() throws WrongSizeException,
            IllegalArgumentException, NonSymmetricMatrixException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        DualConic dualConic = new DualConic();
        double a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double e = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double f = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        dualConic.setA(a);
        dualConic.setB(b);
        dualConic.setC(c);
        dualConic.setD(d);
        dualConic.setE(e);
        dualConic.setF(f);
        assertEquals(a, dualConic.getA(), 0.0);
        assertEquals(b, dualConic.getB(), 0.0);
        assertEquals(c, dualConic.getC(), 0.0);
        assertEquals(d, dualConic.getD(), 0.0);
        assertEquals(e, dualConic.getE(), 0.0);
        assertEquals(f, dualConic.getF(), 0.0);

        dualConic = new DualConic();
        final Matrix m = new Matrix(DUAL_CONIC_ROWS, DUAL_CONIC_COLS);
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
        dualConic.setParameters(m);
        assertEquals(m.getElementAt(0, 0), dualConic.getA(), 0.0);
        assertEquals(m.getElementAt(0, 1), dualConic.getB(), 0.0);
        assertEquals(m.getElementAt(1, 1), dualConic.getC(), 0.0);
        assertEquals(m.getElementAt(0, 2), dualConic.getD(), 0.0);
        assertEquals(m.getElementAt(1, 2), dualConic.getE(), 0.0);
        assertEquals(m.getElementAt(2, 2), dualConic.getF(), 0.0);
    }

    @Test
    public void testAsMatrix() throws WrongSizeException,
            IllegalArgumentException, NonSymmetricMatrixException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final DualConic dualConic = new DualConic();
        final Matrix m = new Matrix(DUAL_CONIC_ROWS, DUAL_CONIC_COLS);
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
        dualConic.setParameters(m);
        final Matrix m2 = dualConic.asMatrix();

        assertTrue(m.equals(m2, PRECISION_ERROR));
    }

    @Test
    public void testIsLocus() throws WrongSizeException, DecomposerException,
            RankDeficientMatrixException, IllegalArgumentException,
            NonSymmetricMatrixException, NotReadyException, LockedException,
            com.irurueta.algebra.NotAvailableException {

        Matrix m = Matrix.createWithUniformRandomValues(5, HOM_COORDS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        Line2D line1 = new Line2D(m.getElementAt(0, 0), m.getElementAt(0, 1),
                m.getElementAt(0, 2));
        Line2D line2 = new Line2D(m.getElementAt(1, 0), m.getElementAt(1, 1),
                m.getElementAt(1, 2));
        Line2D line3 = new Line2D(m.getElementAt(2, 0), m.getElementAt(2, 1),
                m.getElementAt(2, 2));
        Line2D line4 = new Line2D(m.getElementAt(3, 0), m.getElementAt(3, 1),
                m.getElementAt(3, 2));
        Line2D line5 = new Line2D(m.getElementAt(4, 0), m.getElementAt(4, 1),
                m.getElementAt(4, 2));

        line1.normalize();
        line2.normalize();
        line3.normalize();
        line4.normalize();
        line5.normalize();


        // estimate dual conic that lines inside provided 5 lines
        final Matrix systemOfEquationsMatrix = new Matrix(5, 6);

        double l1 = line1.getA();
        double l2 = line1.getB();
        double l3 = line1.getC();
        systemOfEquationsMatrix.setElementAt(0, 0, l1 * l1);
        systemOfEquationsMatrix.setElementAt(0, 1, 2.0 * l1 * l2);
        systemOfEquationsMatrix.setElementAt(0, 2, l2 * l2);
        systemOfEquationsMatrix.setElementAt(0, 3, 2.0 * l1 * l3);
        systemOfEquationsMatrix.setElementAt(0, 4, 2.0 * l2 * l3);
        systemOfEquationsMatrix.setElementAt(0, 5, l3 * l3);

        l1 = line2.getA();
        l2 = line2.getB();
        l3 = line2.getC();
        systemOfEquationsMatrix.setElementAt(1, 0, l1 * l1);
        systemOfEquationsMatrix.setElementAt(1, 1, 2.0 * l1 * l2);
        systemOfEquationsMatrix.setElementAt(1, 2, l2 * l2);
        systemOfEquationsMatrix.setElementAt(1, 3, 2.0 * l1 * l3);
        systemOfEquationsMatrix.setElementAt(1, 4, 2.0 * l2 * l3);
        systemOfEquationsMatrix.setElementAt(1, 5, l3 * l3);

        l1 = line3.getA();
        l2 = line3.getB();
        l3 = line3.getC();
        systemOfEquationsMatrix.setElementAt(2, 0, l1 * l1);
        systemOfEquationsMatrix.setElementAt(2, 1, 2.0 * l1 * l2);
        systemOfEquationsMatrix.setElementAt(2, 2, l2 * l2);
        systemOfEquationsMatrix.setElementAt(2, 3, 2.0 * l1 * l3);
        systemOfEquationsMatrix.setElementAt(2, 4, 2.0 * l2 * l3);
        systemOfEquationsMatrix.setElementAt(2, 5, l3 * l3);

        l1 = line4.getA();
        l2 = line4.getB();
        l3 = line4.getC();
        systemOfEquationsMatrix.setElementAt(3, 0, l1 * l1);
        systemOfEquationsMatrix.setElementAt(3, 1, 2.0 * l1 * l2);
        systemOfEquationsMatrix.setElementAt(3, 2, l2 * l2);
        systemOfEquationsMatrix.setElementAt(3, 3, 2.0 * l1 * l3);
        systemOfEquationsMatrix.setElementAt(3, 4, 2.0 * l2 * l3);
        systemOfEquationsMatrix.setElementAt(3, 5, l3 * l3);

        l1 = line5.getA();
        l2 = line5.getB();
        l3 = line5.getC();
        systemOfEquationsMatrix.setElementAt(4, 0, l1 * l1);
        systemOfEquationsMatrix.setElementAt(4, 1, 2.0 * l1 * l2);
        systemOfEquationsMatrix.setElementAt(4, 2, l2 * l2);
        systemOfEquationsMatrix.setElementAt(4, 3, 2.0 * l1 * l3);
        systemOfEquationsMatrix.setElementAt(4, 4, 2.0 * l2 * l3);
        systemOfEquationsMatrix.setElementAt(4, 5, l3 * l3);

        while (com.irurueta.algebra.Utils.rank(systemOfEquationsMatrix) < 5) {
            m = Matrix.createWithUniformRandomValues(5, HOM_COORDS,
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            line1 = new Line2D(m.getElementAt(0, 0), m.getElementAt(0, 1),
                    m.getElementAt(0, 2));
            line2 = new Line2D(m.getElementAt(1, 0), m.getElementAt(1, 1),
                    m.getElementAt(1, 2));
            line3 = new Line2D(m.getElementAt(2, 0), m.getElementAt(2, 1),
                    m.getElementAt(2, 2));
            line4 = new Line2D(m.getElementAt(3, 0), m.getElementAt(3, 1),
                    m.getElementAt(3, 2));
            line5 = new Line2D(m.getElementAt(4, 0), m.getElementAt(4, 1),
                    m.getElementAt(4, 2));

            line1.normalize();
            line2.normalize();
            line3.normalize();
            line4.normalize();
            line5.normalize();


            l1 = line1.getA();
            l2 = line1.getB();
            l3 = line1.getC();
            systemOfEquationsMatrix.setElementAt(0, 0, l1 * l1);
            systemOfEquationsMatrix.setElementAt(0, 1, 2.0 * l1 * l2);
            systemOfEquationsMatrix.setElementAt(0, 2, l2 * l2);
            systemOfEquationsMatrix.setElementAt(0, 3, 2.0 * l1 * l3);
            systemOfEquationsMatrix.setElementAt(0, 4, 2.0 * l2 * l3);
            systemOfEquationsMatrix.setElementAt(0, 5, l3 * l3);

            l1 = line2.getA();
            l2 = line2.getB();
            l3 = line2.getC();
            systemOfEquationsMatrix.setElementAt(1, 0, l1 * l1);
            systemOfEquationsMatrix.setElementAt(1, 1, 2.0 * l1 * l2);
            systemOfEquationsMatrix.setElementAt(1, 2, l2 * l2);
            systemOfEquationsMatrix.setElementAt(1, 3, 2.0 * l1 * l3);
            systemOfEquationsMatrix.setElementAt(1, 4, 2.0 * l2 * l3);
            systemOfEquationsMatrix.setElementAt(1, 5, l3 * l3);

            l1 = line3.getA();
            l2 = line3.getB();
            l3 = line3.getC();
            systemOfEquationsMatrix.setElementAt(2, 0, l1 * l1);
            systemOfEquationsMatrix.setElementAt(2, 1, 2.0 * l1 * l2);
            systemOfEquationsMatrix.setElementAt(2, 2, l2 * l2);
            systemOfEquationsMatrix.setElementAt(2, 3, 2.0 * l1 * l3);
            systemOfEquationsMatrix.setElementAt(2, 4, 2.0 * l2 * l3);
            systemOfEquationsMatrix.setElementAt(2, 5, l3 * l3);

            l1 = line4.getA();
            l2 = line4.getB();
            l3 = line4.getC();
            systemOfEquationsMatrix.setElementAt(3, 0, l1 * l1);
            systemOfEquationsMatrix.setElementAt(3, 1, 2.0 * l1 * l2);
            systemOfEquationsMatrix.setElementAt(3, 2, l2 * l2);
            systemOfEquationsMatrix.setElementAt(3, 3, 2.0 * l1 * l3);
            systemOfEquationsMatrix.setElementAt(3, 4, 2.0 * l2 * l3);
            systemOfEquationsMatrix.setElementAt(3, 5, l3 * l3);

            l1 = line5.getA();
            l2 = line5.getB();
            l3 = line5.getC();
            systemOfEquationsMatrix.setElementAt(4, 0, l1 * l1);
            systemOfEquationsMatrix.setElementAt(4, 1, 2.0 * l1 * l2);
            systemOfEquationsMatrix.setElementAt(4, 2, l2 * l2);
            systemOfEquationsMatrix.setElementAt(4, 3, 2.0 * l1 * l3);
            systemOfEquationsMatrix.setElementAt(4, 4, 2.0 * l2 * l3);
            systemOfEquationsMatrix.setElementAt(4, 5, l3 * l3);
        }

        final SingularValueDecomposer decomposer = new SingularValueDecomposer(
                systemOfEquationsMatrix);
        decomposer.decompose();

        final Matrix V = decomposer.getV();

        final double a = V.getElementAt(0, 5);
        final double b = V.getElementAt(1, 5);
        final double c = V.getElementAt(2, 5);
        final double d = V.getElementAt(3, 5);
        final double e = V.getElementAt(4, 5);
        final double f = V.getElementAt(5, 5);

        final Matrix dualConicLine = new Matrix(HOM_COORDS, 1);
        dualConicLine.setElementAt(0, 0, line1.getA());
        dualConicLine.setElementAt(1, 0, line1.getB());
        dualConicLine.setElementAt(2, 0, line1.getC());

        double norm = com.irurueta.algebra.Utils.normF(dualConicLine);
        dualConicLine.multiplyByScalar(1.0 / norm);

        final Matrix dualConicMatrix = new Matrix(DUAL_CONIC_ROWS, DUAL_CONIC_COLS);
        dualConicMatrix.setElementAt(0, 0, a);
        dualConicMatrix.setElementAt(0, 1, b);
        dualConicMatrix.setElementAt(0, 2, d);
        dualConicMatrix.setElementAt(1, 0, b);
        dualConicMatrix.setElementAt(1, 1, c);
        dualConicMatrix.setElementAt(1, 2, e);
        dualConicMatrix.setElementAt(2, 0, d);
        dualConicMatrix.setElementAt(2, 1, e);
        dualConicMatrix.setElementAt(2, 2, f);

        norm = com.irurueta.algebra.Utils.normF(dualConicMatrix);
        dualConicMatrix.multiplyByScalar(1.0 / norm);

        // find point where line is tangent to conic
        final Matrix homPointMatrix = dualConicMatrix.multiplyAndReturnNew(
                dualConicLine);

        // add director vector of tangent line to get a point outside of conic
        double directVectorA = dualConicLine.getElementAtIndex(0);
        double directVectorB = dualConicLine.getElementAtIndex(1);
        final double directVectorNorm = Math.sqrt(directVectorA * directVectorA +
                directVectorB * directVectorB);
        directVectorA /= directVectorNorm;
        directVectorB /= directVectorNorm;
        homPointMatrix.setElementAtIndex(0, homPointMatrix.getElementAtIndex(0)
                + directVectorA * homPointMatrix.getElementAtIndex(2));
        homPointMatrix.setElementAtIndex(1, homPointMatrix.getElementAtIndex(1)
                + directVectorB * homPointMatrix.getElementAtIndex(2));

        norm = com.irurueta.algebra.Utils.normF(homPointMatrix);
        homPointMatrix.multiplyByScalar(1.0 / norm);

        // get conic matrix by inverting dual conic matrix
        final Matrix conicMatrix = com.irurueta.algebra.Utils.inverse(
                dualConicMatrix);

        // find line vector outside dual conic as the product of conic matrix
        // and point outside of conic
        final Matrix outsideLineMatrix = conicMatrix.multiplyAndReturnNew(
                homPointMatrix);

        // instantiate line outside dual conic using computed vector
        final Line2D outsideLine = new Line2D(outsideLineMatrix.toArray());

        // instantiate new dual conic instance
        final DualConic dualConic = new DualConic(dualConicMatrix);

        // check that initial 5 lines lie inside the dual conic
        assertTrue(dualConic.isLocus(line1, LOCUS_THRESHOLD));
        assertTrue(dualConic.isLocus(line2, LOCUS_THRESHOLD));
        assertTrue(dualConic.isLocus(line3, LOCUS_THRESHOLD));
        assertTrue(dualConic.isLocus(line4, LOCUS_THRESHOLD));
        assertTrue(dualConic.isLocus(line5, LOCUS_THRESHOLD));

        // check line outside of dual conic
        assertFalse(dualConic.isLocus(outsideLine, LOCUS_THRESHOLD));
    }

    @Test
    public void testAngleBetweenLines() throws WrongSizeException,
            IllegalArgumentException, NonSymmetricMatrixException {

        // initial lines
        final Matrix lineMatrix1 = Matrix.createWithUniformRandomValues(HOM_COORDS, 1,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final Matrix lineMatrix2 = Matrix.createWithUniformRandomValues(HOM_COORDS, 1,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // transformation matrix
        final Matrix transform = Matrix.createWithUniformRandomValues(HOM_COORDS,
                HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // transform lines
        final Matrix tLine1 = transform.multiplyAndReturnNew(lineMatrix1);
        final Matrix tLine2 = transform.multiplyAndReturnNew(lineMatrix2);

        final double norm1 = com.irurueta.algebra.Utils.normF(tLine1);
        final double norm2 = com.irurueta.algebra.Utils.normF(tLine2);

        final double numerator = tLine1.transposeAndReturnNew().multiplyAndReturnNew(
                tLine2).getElementAt(0, 0);

        final double cosAngle = numerator / (norm1 * norm2);

        final double angle = Math.acos(cosAngle);

        // compute dual conic matrix as the product of transposed transform
        // matrix with itself
        final Matrix transposedTransform = transform.transposeAndReturnNew();
        final Matrix dualConicMatrix = transposedTransform.multiplyAndReturnNew(
                transform);

        // normalize conic matrix
        final double normDualConic = com.irurueta.algebra.Utils.normF(dualConicMatrix);
        dualConicMatrix.multiplyByScalar(1.0 / normDualConic);

        final DualConic dualConic = new DualConic(dualConicMatrix);

        final Line2D line1 = new Line2D(lineMatrix1.getElementAt(0, 0),
                lineMatrix1.getElementAt(1, 0), lineMatrix1.getElementAt(2, 0));
        final Line2D line2 = new Line2D(lineMatrix2.getElementAt(0, 0),
                lineMatrix2.getElementAt(1, 0), lineMatrix2.getElementAt(2, 0));

        assertEquals(dualConic.angleBetweenLines(line1, line2), angle,
                PRECISION_ERROR);
    }

    @Test
    public void testArePerpendicularLines() throws WrongSizeException,
            DecomposerException, RankDeficientMatrixException,
            IllegalArgumentException, NonSymmetricMatrixException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            // trying perpendicular angle
            Matrix matrixLine1 = Matrix.createWithUniformRandomValues(HOM_COORDS, 1,
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            double norm = com.irurueta.algebra.Utils.normF(matrixLine1);
            matrixLine1.multiplyByScalar(1.0 / norm);

            Matrix matrixLine2 = new Matrix(HOM_COORDS, 1);
            matrixLine2.setElementAt(0, 0, matrixLine1.getElementAt(1, 0) +
                    matrixLine1.getElementAt(2, 0));
            matrixLine2.setElementAt(1, 0, -matrixLine1.getElementAt(0, 0));
            matrixLine2.setElementAt(2, 0, -matrixLine1.getElementAt(0, 0));

            norm = com.irurueta.algebra.Utils.normF(matrixLine2);
            matrixLine2.multiplyByScalar(1.0 / norm);

            Matrix transform = Matrix.createWithUniformRandomValues(HOM_COORDS,
                    HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            while (com.irurueta.algebra.Utils.rank(transform) < 3) {
                transform = Matrix.createWithUniformRandomValues(HOM_COORDS,
                        HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            }

            final Matrix invTransform = com.irurueta.algebra.Utils.inverse(transform);

            Matrix transformLineMatrix1 = transform.multiplyAndReturnNew(
                    matrixLine1);
            Matrix transformLineMatrix2 = transform.multiplyAndReturnNew(
                    matrixLine2);

            final Matrix transInvTransform = invTransform.transposeAndReturnNew();

            final Matrix dualConicMatrix = transInvTransform.multiplyAndReturnNew(
                    invTransform);
            norm = com.irurueta.algebra.Utils.normF(dualConicMatrix);
            dualConicMatrix.multiplyByScalar(1.0 / norm);

            Line2D transformLine1 = new Line2D(transformLineMatrix1.toArray());
            Line2D transformLine2 = new Line2D(transformLineMatrix2.toArray());

            DualConic dualConic = new DualConic(dualConicMatrix);

            assertTrue(dualConic.arePerpendicularLines(transformLine1,
                    transformLine2, PERPENDICULAR_THRESHOLD));

            // trying non-perpendicular points
            double dotProduct;

            matrixLine1 = Matrix.createWithUniformRandomValues(HOM_COORDS, 1,
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            norm = com.irurueta.algebra.Utils.normF(matrixLine1);
            matrixLine1.multiplyByScalar(1.0 / norm);

            matrixLine2 = Matrix.createWithUniformRandomValues(HOM_COORDS, 1,
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            norm = com.irurueta.algebra.Utils.normF(matrixLine2);
            matrixLine2.multiplyByScalar(1.0 / norm);

            dotProduct = matrixLine1.transposeAndReturnNew().multiplyAndReturnNew(
                    matrixLine2).getElementAt(0, 0);

            // ensure lines are not perpendicular
            while (Math.abs(dotProduct) < PERPENDICULAR_THRESHOLD) {
                matrixLine1 = Matrix.createWithUniformRandomValues(HOM_COORDS, 1,
                        MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                norm = com.irurueta.algebra.Utils.normF(matrixLine1);
                matrixLine1.multiplyByScalar(1.0 / norm);

                matrixLine2 = Matrix.createWithUniformRandomValues(HOM_COORDS, 1,
                        MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                norm = com.irurueta.algebra.Utils.normF(matrixLine2);
                matrixLine2.multiplyByScalar(1.0 / norm);

                dotProduct = matrixLine1.transposeAndReturnNew().
                        multiplyAndReturnNew(matrixLine2).getElementAt(0, 0);
            }

            transformLineMatrix1 = transform.multiplyAndReturnNew(matrixLine1);
            transformLineMatrix2 = transform.multiplyAndReturnNew(matrixLine2);

            transformLine1 = new Line2D(transformLineMatrix1.toArray());
            transformLine2 = new Line2D(transformLineMatrix2.toArray());

            if (dualConic.arePerpendicularLines(transformLine1, transformLine2,
                    5.0 * PERPENDICULAR_THRESHOLD)) {
                continue;
            }
            assertFalse(dualConic.arePerpendicularLines(transformLine1,
                    transformLine2, 5.0 * PERPENDICULAR_THRESHOLD));

            dualConic = DualConic.createCanonicalDualAbsoluteConic();
            transformLine1 = new Line2D(1.0, 0.0, 0.0);
            transformLine2 = new Line2D(0.0, 1.0, 0.0);
            assertTrue(dualConic.arePerpendicularLines(transformLine1, transformLine2));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testGetConic() throws WrongSizeException, DecomposerException,
            RankDeficientMatrixException, IllegalArgumentException,
            NonSymmetricMatrixException, ConicNotAvailableException {

        Matrix transformMatrix = Matrix.createWithUniformRandomValues(
                HOM_COORDS, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        while (com.irurueta.algebra.Utils.rank(transformMatrix) != 3) {
            transformMatrix = Matrix.createWithUniformRandomValues(HOM_COORDS,
                    HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        }

        final Matrix transformTransposedMatrix =
                transformMatrix.transposeAndReturnNew();

        final Matrix dualConicMatrix = transformTransposedMatrix.multiplyAndReturnNew(
                transformMatrix);

        final Matrix conicMatrix1 = com.irurueta.algebra.Utils.inverse(
                dualConicMatrix);

        final DualConic dualConic = new DualConic(dualConicMatrix);

        final Conic conic = dualConic.getConic();
        final Matrix conicMatrix2 = conic.asMatrix();

        // normalize conic matrices
        double norm = com.irurueta.algebra.Utils.normF(conicMatrix1);
        conicMatrix1.multiplyByScalar(1.0 / norm);

        norm = com.irurueta.algebra.Utils.normF(conicMatrix2);
        conicMatrix2.multiplyByScalar(1.0 / norm);

        // compute difference of normalized conic matrices
        final Matrix diffMatrix = conicMatrix1.subtractAndReturnNew(conicMatrix2);

        // ensure that difference matrix is almost zero by checking its norm
        norm = com.irurueta.algebra.Utils.normF(diffMatrix);
        assertEquals(0.0, norm, PRECISION_ERROR);
    }

    @Test
    public void testSetParametersFromLines()
            throws WrongSizeException, DecomposerException,
            CoincidentLinesException {
        Matrix m = Matrix.createWithUniformRandomValues(5, HOM_COORDS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        Line2D line1 = new Line2D(m.getElementAt(0, 0), m.getElementAt(0, 1),
                m.getElementAt(0, 2));
        Line2D line2 = new Line2D(m.getElementAt(1, 0), m.getElementAt(1, 1),
                m.getElementAt(1, 2));
        Line2D line3 = new Line2D(m.getElementAt(2, 0), m.getElementAt(2, 1),
                m.getElementAt(2, 2));
        Line2D line4 = new Line2D(m.getElementAt(3, 0), m.getElementAt(3, 1),
                m.getElementAt(3, 2));
        Line2D line5 = new Line2D(m.getElementAt(4, 0), m.getElementAt(4, 1),
                m.getElementAt(4, 2));

        line1.normalize();
        line2.normalize();
        line3.normalize();
        line4.normalize();
        line5.normalize();

        // estimate dual conic that lies inside provided 5 lines
        final Matrix m2 = new Matrix(5, 6);

        double l1 = line1.getA();
        double l2 = line1.getB();
        double l3 = line1.getC();
        m2.setElementAt(0, 0, l1 * l1);
        m2.setElementAt(0, 1, 2.0 * l1 * l2);
        m2.setElementAt(0, 2, l2 * l2);
        m2.setElementAt(0, 3, 2.0 * l1 * l3);
        m2.setElementAt(0, 4, 2.0 * l2 * l3);
        m2.setElementAt(0, 5, l3 * l3);

        l1 = line2.getA();
        l2 = line2.getB();
        l3 = line2.getC();
        m2.setElementAt(1, 0, l1 * l1);
        m2.setElementAt(1, 1, 2.0 * l1 * l2);
        m2.setElementAt(1, 2, l2 * l2);
        m2.setElementAt(1, 3, 2.0 * l1 * l3);
        m2.setElementAt(1, 4, 2.0 * l2 * l3);
        m2.setElementAt(1, 5, l3 * l3);

        l1 = line3.getA();
        l2 = line3.getB();
        l3 = line3.getC();
        m2.setElementAt(2, 0, l1 * l1);
        m2.setElementAt(2, 1, 2.0 * l1 * l2);
        m2.setElementAt(2, 2, l2 * l2);
        m2.setElementAt(2, 3, 2.0 * l1 * l3);
        m2.setElementAt(2, 4, 2.0 * l2 * l3);
        m2.setElementAt(2, 5, l3 * l3);

        l1 = line4.getA();
        l2 = line4.getB();
        l3 = line4.getC();
        m2.setElementAt(3, 0, l1 * l1);
        m2.setElementAt(3, 1, 2.0 * l1 * l2);
        m2.setElementAt(3, 2, l2 * l2);
        m2.setElementAt(3, 3, 2.0 * l1 * l3);
        m2.setElementAt(3, 4, 2.0 * l2 * l3);
        m2.setElementAt(3, 5, l3 * l3);

        l1 = line5.getA();
        l2 = line5.getB();
        l3 = line5.getC();
        m2.setElementAt(4, 0, l1 * l1);
        m2.setElementAt(4, 1, 2.0 * l1 * l2);
        m2.setElementAt(4, 2, l2 * l2);
        m2.setElementAt(4, 3, 2.0 * l1 * l3);
        m2.setElementAt(4, 4, 2.0 * l2 * l3);
        m2.setElementAt(4, 5, l3 * l3);

        while (com.irurueta.algebra.Utils.rank(m2) < 5) {
            m = Matrix.createWithUniformRandomValues(5, HOM_COORDS,
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            line1 = new Line2D(m.getElementAt(0, 0), m.getElementAt(0, 1),
                    m.getElementAt(0, 2));
            line2 = new Line2D(m.getElementAt(1, 0), m.getElementAt(1, 1),
                    m.getElementAt(1, 2));
            line3 = new Line2D(m.getElementAt(2, 0), m.getElementAt(2, 1),
                    m.getElementAt(2, 2));
            line4 = new Line2D(m.getElementAt(3, 0), m.getElementAt(3, 1),
                    m.getElementAt(3, 2));
            line5 = new Line2D(m.getElementAt(4, 0), m.getElementAt(4, 1),
                    m.getElementAt(4, 2));

            line1.normalize();
            line2.normalize();
            line3.normalize();
            line4.normalize();
            line5.normalize();


            l1 = line1.getA();
            l2 = line1.getB();
            l3 = line1.getC();
            m2.setElementAt(0, 0, l1 * l1);
            m2.setElementAt(0, 1, 2.0 * l1 * l2);
            m2.setElementAt(0, 2, l2 * l2);
            m2.setElementAt(0, 3, 2.0 * l1 * l3);
            m2.setElementAt(0, 4, 2.0 * l2 * l3);
            m2.setElementAt(0, 5, l3 * l3);

            l1 = line2.getA();
            l2 = line2.getB();
            l3 = line2.getC();
            m2.setElementAt(1, 0, l1 * l1);
            m2.setElementAt(1, 1, 2.0 * l1 * l2);
            m2.setElementAt(1, 2, l2 * l2);
            m2.setElementAt(1, 3, 2.0 * l1 * l3);
            m2.setElementAt(1, 4, 2.0 * l2 * l3);
            m2.setElementAt(1, 5, l3 * l3);

            l1 = line3.getA();
            l2 = line3.getB();
            l3 = line3.getC();
            m2.setElementAt(2, 0, l1 * l1);
            m2.setElementAt(2, 1, 2.0 * l1 * l2);
            m2.setElementAt(2, 2, l2 * l2);
            m2.setElementAt(2, 3, 2.0 * l1 * l3);
            m2.setElementAt(2, 4, 2.0 * l2 * l3);
            m2.setElementAt(2, 5, l3 * l3);

            l1 = line4.getA();
            l2 = line4.getB();
            l3 = line4.getC();
            m2.setElementAt(3, 0, l1 * l1);
            m2.setElementAt(3, 1, 2.0 * l1 * l2);
            m2.setElementAt(3, 2, l2 * l2);
            m2.setElementAt(3, 3, 2.0 * l1 * l3);
            m2.setElementAt(3, 4, 2.0 * l2 * l3);
            m2.setElementAt(3, 5, l3 * l3);

            l1 = line5.getA();
            l2 = line5.getB();
            l3 = line5.getC();
            m2.setElementAt(4, 0, l1 * l1);
            m2.setElementAt(4, 1, 2.0 * l1 * l2);
            m2.setElementAt(4, 2, l2 * l2);
            m2.setElementAt(4, 3, 2.0 * l1 * l3);
            m2.setElementAt(4, 4, 2.0 * l2 * l3);
            m2.setElementAt(4, 5, l3 * l3);
        }

        final DualConic dualConic = new DualConic();
        dualConic.setParametersFromLines(line1, line2, line3, line4, line5);

        assertTrue(dualConic.isLocus(line1, PRECISION_ERROR));
        assertTrue(dualConic.isLocus(line2, PRECISION_ERROR));
        assertTrue(dualConic.isLocus(line3, PRECISION_ERROR));
        assertTrue(dualConic.isLocus(line4, PRECISION_ERROR));
        assertTrue(dualConic.isLocus(line5, PRECISION_ERROR));

        // Force CoincidentLinesException
        try {
            dualConic.setParametersFromLines(line1, line2, line3, line4, line4);
            fail("CoincidentLinesException expected but not thrown");
        } catch (final CoincidentLinesException ignore) {
        }
    }

    @Test
    public void testNormalize() throws WrongSizeException,
            IllegalArgumentException, NonSymmetricMatrixException {

        final Matrix t = Matrix.createWithUniformRandomValues(HOM_COORDS, HOM_COORDS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final Matrix transT = t.transposeAndReturnNew();

        // make symmetric matrix
        final Matrix dualConicMatrix = transT.multiplyAndReturnNew(t);

        final DualConic dualConic = new DualConic(dualConicMatrix);

        // normalize dual conic
        dualConic.normalize();

        // return dual conic as matrix
        final Matrix dualConicMatrix2 = dualConic.asMatrix();

        // compare that both matrices are equal up to scale, for that reason we
        // first normalize both matrices
        double norm = com.irurueta.algebra.Utils.normF(dualConicMatrix);
        dualConicMatrix.multiplyByScalar(1.0 / norm);

        norm = com.irurueta.algebra.Utils.normF(dualConicMatrix2);
        dualConicMatrix2.multiplyByScalar(1.0 / norm);

        // compute their difference
        final Matrix diffMatrix = dualConicMatrix.subtractAndReturnNew(
                dualConicMatrix2);

        // finally, ensure that the norm of the difference matrix is almost up to
        // machine precision
        norm = com.irurueta.algebra.Utils.normF(diffMatrix);

        assertEquals(0.0, norm, PRECISION_ERROR);
    }

    @Test
    public void testCreateCanonicalDualAbsoluteConic() throws WrongSizeException {
        final DualConic dac = DualConic.createCanonicalDualAbsoluteConic();

        assertEquals(1.0, dac.getA(), 0.0);
        assertEquals(0.0, dac.getB(), 0.0);
        assertEquals(1.0, dac.getC(), 0.0);
        assertEquals(0.0, dac.getD(), 0.0);
        assertEquals(0.0, dac.getE(), 0.0);
        assertEquals(1.0, dac.getF(), 0.0);

        assertEquals(dac.asMatrix(), Matrix.identity(3, 3));
    }

    @Test
    public void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        double a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double e = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double f = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final DualConic dualConic1 = new DualConic(a, b, c, d, e, f);

        // check
        assertEquals(a, dualConic1.getA(), 0.0);
        assertEquals(b, dualConic1.getB(), 0.0);
        assertEquals(c, dualConic1.getC(), 0.0);
        assertEquals(d, dualConic1.getD(), 0.0);
        assertEquals(e, dualConic1.getE(), 0.0);
        assertEquals(f, dualConic1.getF(), 0.0);
        assertFalse(dualConic1.isNormalized());

        // serialize and deserialize
        final byte[] bytes = SerializationHelper.serialize(dualConic1);
        final DualConic dualConic2 = SerializationHelper.deserialize(bytes);

        // check
        assertEquals(dualConic1.getA(), dualConic2.getA(), 0.0);
        assertEquals(dualConic1.getB(), dualConic2.getB(), 0.0);
        assertEquals(dualConic1.getC(), dualConic2.getC(), 0.0);
        assertEquals(dualConic1.getD(), dualConic2.getD(), 0.0);
        assertEquals(dualConic1.getE(), dualConic2.getE(), 0.0);
        assertEquals(dualConic1.getF(), dualConic2.getF(), 0.0);
        assertEquals(dualConic1.isNormalized(),
                dualConic2.isNormalized());
    }
}
