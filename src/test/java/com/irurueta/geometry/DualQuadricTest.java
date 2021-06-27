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

import java.io.IOException;
import java.util.Random;

import static org.junit.Assert.*;

public class DualQuadricTest {

    private static final double MIN_RANDOM_VALUE = -10.0;
    private static final double MAX_RANDOM_VALUE = 10.0;
    private static final double PRECISION_ERROR = 1e-8;
    private static final double LOCUS_THRESHOLD = 1e-8;
    private static final double PERPENDICULAR_THRESHOLD = 1e-6;

    private static final int DUAL_QUADRIC_ROWS = 4;
    private static final int DUAL_QUADRIC_COLS = 4;
    private static final int HOM_COORDS = 4;

    private static final int TIMES = 10;

    @Test
    public void testConstants() {
        assertEquals(4, BaseQuadric.BASEQUADRIC_MATRIX_ROW_SIZE);
        assertEquals(4, BaseQuadric.BASEQUADRIC_MATRIX_COLUMN_SIZE);
        assertEquals(10, BaseQuadric.N_PARAMS);
        assertEquals(1e-12, BaseQuadric.DEFAULT_LOCUS_THRESHOLD, 0.0);
        assertEquals(1e-12, BaseQuadric.DEFAULT_PERPENDICULAR_THRESHOLD, 0.0);
        assertEquals(0.0, BaseQuadric.MIN_THRESHOLD, 0.0);
    }

    @Test
    public void testConstructor() throws WrongSizeException,
            IllegalArgumentException, NonSymmetricMatrixException,
            DecomposerException, CoincidentPlanesException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // Constructor
        DualQuadric dualQuadric = new DualQuadric();
        assertEquals(dualQuadric.getA(), 0.0, 0.0);
        assertEquals(dualQuadric.getB(), 0.0, 0.0);
        assertEquals(dualQuadric.getC(), 0.0, 0.0);
        assertEquals(dualQuadric.getD(), 0.0, 0.0);
        assertEquals(dualQuadric.getE(), 0.0, 0.0);
        assertEquals(dualQuadric.getF(), 0.0, 0.0);
        assertEquals(dualQuadric.getG(), 0.0, 0.0);
        assertEquals(dualQuadric.getH(), 0.0, 0.0);
        assertEquals(dualQuadric.getI(), 0.0, 0.0);
        assertEquals(dualQuadric.getJ(), 0.0, 0.0);
        assertFalse(dualQuadric.isNormalized());

        // Constructor with params
        double a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double e = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double f = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double g = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double h = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double i = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double j = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        dualQuadric = new DualQuadric(a, b, c, d, e, f, g, h, i, j);
        assertEquals(dualQuadric.getA(), a, 0.0);
        assertEquals(dualQuadric.getB(), b, 0.0);
        assertEquals(dualQuadric.getC(), c, 0.0);
        assertEquals(dualQuadric.getD(), d, 0.0);
        assertEquals(dualQuadric.getE(), e, 0.0);
        assertEquals(dualQuadric.getF(), f, 0.0);
        assertEquals(dualQuadric.getG(), g, 0.0);
        assertEquals(dualQuadric.getH(), h, 0.0);
        assertEquals(dualQuadric.getI(), i, 0.0);
        assertEquals(dualQuadric.getJ(), j, 0.0);
        assertFalse(dualQuadric.isNormalized());

        // Constructor using matrix
        Matrix m = new Matrix(DUAL_QUADRIC_ROWS, DUAL_QUADRIC_COLS);
        a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        e = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        f = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        g = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        h = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        i = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        j = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        m.setElementAt(0, 0, a);
        m.setElementAt(1, 1, b);
        m.setElementAt(2, 2, c);
        m.setElementAt(3, 3, j);
        m.setElementAt(1, 0, d);
        m.setElementAt(0, 1, d);
        m.setElementAt(2, 1, e);
        m.setElementAt(1, 2, e);
        m.setElementAt(2, 0, f);
        m.setElementAt(0, 2, f);
        m.setElementAt(3, 0, g);
        m.setElementAt(0, 3, g);
        m.setElementAt(3, 1, h);
        m.setElementAt(1, 3, h);
        m.setElementAt(3, 2, i);
        m.setElementAt(2, 3, i);
        dualQuadric = new DualQuadric(m);

        assertEquals(dualQuadric.getA(), m.getElementAt(0, 0), 0.0);
        assertEquals(dualQuadric.getB(), m.getElementAt(1, 1), 0.0);
        assertEquals(dualQuadric.getC(), m.getElementAt(2, 2), 0.0);
        assertEquals(dualQuadric.getD(), m.getElementAt(1, 0), 0.0);
        assertEquals(dualQuadric.getE(), m.getElementAt(2, 1), 0.0);
        assertEquals(dualQuadric.getF(), m.getElementAt(2, 0), 0.0);
        assertEquals(dualQuadric.getG(), m.getElementAt(3, 0), 0.0);
        assertEquals(dualQuadric.getH(), m.getElementAt(1, 3), 0.0);
        assertEquals(dualQuadric.getI(), m.getElementAt(3, 2), 0.0);
        assertEquals(dualQuadric.getJ(), m.getElementAt(3, 3), 0.0);

        // Constructor using matrix with wrong size exception
        m = new Matrix(DUAL_QUADRIC_ROWS, DUAL_QUADRIC_COLS + 1);
        dualQuadric = null;
        try {
            dualQuadric = new DualQuadric(m);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(dualQuadric);

        // Constructor using non-symmetric matrix
        m = new Matrix(DUAL_QUADRIC_ROWS, DUAL_QUADRIC_COLS);
        m.setElementAt(0, 0, a);
        m.setElementAt(1, 1, b);
        m.setElementAt(2, 2, c);
        m.setElementAt(3, 3, j);
        m.setElementAt(1, 0, d);
        m.setElementAt(0, 1, d + 1.0);
        m.setElementAt(2, 1, e);
        m.setElementAt(1, 2, e + 1.0);
        m.setElementAt(2, 0, f);
        m.setElementAt(0, 2, f + 1.0);
        m.setElementAt(3, 0, g);
        m.setElementAt(0, 3, g + 1.0);
        m.setElementAt(3, 1, h);
        m.setElementAt(1, 3, h + 1.0);
        m.setElementAt(3, 2, i);
        m.setElementAt(2, 3, i + 1.0);

        dualQuadric = null;
        try {
            dualQuadric = new DualQuadric(m);
            fail("NonSymmetricMatrixException expected but not thrown");
        } catch (final NonSymmetricMatrixException ignore) {
        }
        assertNull(dualQuadric);

        // Constructor from 9 planes
        m = Matrix.createWithUniformRandomValues(9, HOM_COORDS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        Plane plane1 = new Plane(m.getElementAt(0, 0), m.getElementAt(0, 1),
                m.getElementAt(0, 2), m.getElementAt(0, 3));
        Plane plane2 = new Plane(m.getElementAt(1, 0), m.getElementAt(1, 1),
                m.getElementAt(1, 2), m.getElementAt(1, 3));
        Plane plane3 = new Plane(m.getElementAt(2, 0), m.getElementAt(2, 1),
                m.getElementAt(2, 2), m.getElementAt(2, 3));
        Plane plane4 = new Plane(m.getElementAt(3, 0), m.getElementAt(3, 1),
                m.getElementAt(3, 2), m.getElementAt(3, 3));
        Plane plane5 = new Plane(m.getElementAt(4, 0), m.getElementAt(4, 1),
                m.getElementAt(4, 2), m.getElementAt(4, 3));
        Plane plane6 = new Plane(m.getElementAt(5, 0), m.getElementAt(5, 1),
                m.getElementAt(5, 2), m.getElementAt(5, 3));
        Plane plane7 = new Plane(m.getElementAt(6, 0), m.getElementAt(6, 1),
                m.getElementAt(6, 2), m.getElementAt(6, 3));
        Plane plane8 = new Plane(m.getElementAt(7, 0), m.getElementAt(7, 1),
                m.getElementAt(7, 2), m.getElementAt(7, 3));
        Plane plane9 = new Plane(m.getElementAt(8, 0), m.getElementAt(8, 1),
                m.getElementAt(8, 2), m.getElementAt(8, 3));

        plane1.normalize();
        plane2.normalize();
        plane3.normalize();
        plane4.normalize();
        plane5.normalize();
        plane6.normalize();
        plane7.normalize();
        plane8.normalize();
        plane9.normalize();

        // estimate dual quadric that lies inside of provided 9 planes

        Matrix m2 = new Matrix(9, 10);
        double pA = plane1.getA();
        double pB = plane1.getB();
        double pC = plane1.getC();
        double pD = plane1.getD();
        m2.setElementAt(0, 0, pA * pA);
        m2.setElementAt(0, 1, pB * pB);
        m2.setElementAt(0, 2, pC * pC);
        m2.setElementAt(0, 3, 2.0 * pA * pB);
        m2.setElementAt(0, 4, 2.0 * pA * pC);
        m2.setElementAt(0, 5, 2.0 * pB * pC);
        m2.setElementAt(0, 6, 2.0 * pA * pD);
        m2.setElementAt(0, 7, 2.0 * pB * pD);
        m2.setElementAt(0, 8, 2.0 * pC * pD);
        m2.setElementAt(0, 9, pD * pD);
        pA = plane2.getA();
        pB = plane2.getB();
        pC = plane2.getC();
        pD = plane2.getD();
        m2.setElementAt(1, 0, pA * pA);
        m2.setElementAt(1, 1, pB * pB);
        m2.setElementAt(1, 2, pC * pC);
        m2.setElementAt(1, 3, 2.0 * pA * pB);
        m2.setElementAt(1, 4, 2.0 * pA * pC);
        m2.setElementAt(1, 5, 2.0 * pB * pC);
        m2.setElementAt(1, 6, 2.0 * pA * pD);
        m2.setElementAt(1, 7, 2.0 * pB * pD);
        m2.setElementAt(1, 8, 2.0 * pC * pD);
        m2.setElementAt(1, 9, pD * pD);
        pA = plane3.getA();
        pB = plane3.getB();
        pC = plane3.getC();
        pD = plane3.getD();
        m2.setElementAt(2, 0, pA * pA);
        m2.setElementAt(2, 1, pB * pB);
        m2.setElementAt(2, 2, pC * pC);
        m2.setElementAt(2, 3, 2.0 * pA * pB);
        m2.setElementAt(2, 4, 2.0 * pA * pC);
        m2.setElementAt(2, 5, 2.0 * pB * pC);
        m2.setElementAt(2, 6, 2.0 * pA * pD);
        m2.setElementAt(2, 7, 2.0 * pB * pD);
        m2.setElementAt(2, 8, 2.0 * pC * pD);
        m2.setElementAt(2, 9, pD * pD);
        pA = plane4.getA();
        pB = plane4.getB();
        pC = plane4.getC();
        pD = plane4.getD();
        m2.setElementAt(3, 0, pA * pA);
        m2.setElementAt(3, 1, pB * pB);
        m2.setElementAt(3, 2, pC * pC);
        m2.setElementAt(3, 3, 2.0 * pA * pB);
        m2.setElementAt(3, 4, 2.0 * pA * pC);
        m2.setElementAt(3, 5, 2.0 * pB * pC);
        m2.setElementAt(3, 6, 2.0 * pA * pD);
        m2.setElementAt(3, 7, 2.0 * pB * pD);
        m2.setElementAt(3, 8, 2.0 * pC * pD);
        m2.setElementAt(3, 9, pD * pD);
        pA = plane5.getA();
        pB = plane5.getB();
        pC = plane5.getC();
        pD = plane5.getD();
        m2.setElementAt(4, 0, pA * pA);
        m2.setElementAt(4, 1, pB * pB);
        m2.setElementAt(4, 2, pC * pC);
        m2.setElementAt(4, 3, 2.0 * pA * pB);
        m2.setElementAt(4, 4, 2.0 * pA * pC);
        m2.setElementAt(4, 5, 2.0 * pB * pC);
        m2.setElementAt(4, 6, 2.0 * pA * pD);
        m2.setElementAt(4, 7, 2.0 * pB * pD);
        m2.setElementAt(4, 8, 2.0 * pC * pD);
        m2.setElementAt(4, 9, pD * pD);
        pA = plane6.getA();
        pB = plane6.getB();
        pC = plane6.getC();
        pD = plane6.getD();
        m2.setElementAt(5, 0, pA * pA);
        m2.setElementAt(5, 1, pB * pB);
        m2.setElementAt(5, 2, pC * pC);
        m2.setElementAt(5, 3, 2.0 * pA * pB);
        m2.setElementAt(5, 4, 2.0 * pA * pC);
        m2.setElementAt(5, 5, 2.0 * pB * pC);
        m2.setElementAt(5, 6, 2.0 * pA * pD);
        m2.setElementAt(5, 7, 2.0 * pB * pD);
        m2.setElementAt(5, 8, 2.0 * pC * pD);
        m2.setElementAt(5, 9, pD * pD);
        pA = plane7.getA();
        pB = plane7.getB();
        pC = plane7.getC();
        pD = plane7.getD();
        m2.setElementAt(6, 0, pA * pA);
        m2.setElementAt(6, 1, pB * pB);
        m2.setElementAt(6, 2, pC * pC);
        m2.setElementAt(6, 3, 2.0 * pA * pB);
        m2.setElementAt(6, 4, 2.0 * pA * pC);
        m2.setElementAt(6, 5, 2.0 * pB * pC);
        m2.setElementAt(6, 6, 2.0 * pA * pD);
        m2.setElementAt(6, 7, 2.0 * pB * pD);
        m2.setElementAt(6, 8, 2.0 * pC * pD);
        m2.setElementAt(6, 9, pD * pD);
        pA = plane8.getA();
        pB = plane8.getB();
        pC = plane8.getC();
        pD = plane8.getD();
        m2.setElementAt(7, 0, pA * pA);
        m2.setElementAt(7, 1, pB * pB);
        m2.setElementAt(7, 2, pC * pC);
        m2.setElementAt(7, 3, 2.0 * pA * pB);
        m2.setElementAt(7, 4, 2.0 * pA * pC);
        m2.setElementAt(7, 5, 2.0 * pB * pC);
        m2.setElementAt(7, 6, 2.0 * pA * pD);
        m2.setElementAt(7, 7, 2.0 * pB * pD);
        m2.setElementAt(7, 8, 2.0 * pC * pD);
        m2.setElementAt(7, 9, pD * pD);
        pA = plane9.getA();
        pB = plane9.getB();
        pC = plane9.getC();
        pD = plane9.getD();
        m2.setElementAt(8, 0, pA * pA);
        m2.setElementAt(8, 1, pB * pB);
        m2.setElementAt(8, 2, pC * pC);
        m2.setElementAt(8, 3, 2.0 * pA * pB);
        m2.setElementAt(8, 4, 2.0 * pA * pC);
        m2.setElementAt(8, 5, 2.0 * pB * pC);
        m2.setElementAt(8, 6, 2.0 * pA * pD);
        m2.setElementAt(8, 7, 2.0 * pB * pD);
        m2.setElementAt(8, 8, 2.0 * pC * pD);
        m2.setElementAt(8, 9, pD * pD);

        while (com.irurueta.algebra.Utils.rank(m2) < 9) {
            m = Matrix.createWithUniformRandomValues(9, HOM_COORDS,
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            plane1 = new Plane(m.getElementAt(0, 0), m.getElementAt(0, 1),
                    m.getElementAt(0, 2), m.getElementAt(0, 3));
            plane2 = new Plane(m.getElementAt(1, 0), m.getElementAt(1, 1),
                    m.getElementAt(1, 2), m.getElementAt(1, 3));
            plane3 = new Plane(m.getElementAt(2, 0), m.getElementAt(2, 1),
                    m.getElementAt(2, 2), m.getElementAt(2, 3));
            plane4 = new Plane(m.getElementAt(3, 0), m.getElementAt(3, 1),
                    m.getElementAt(3, 2), m.getElementAt(3, 3));
            plane5 = new Plane(m.getElementAt(4, 0), m.getElementAt(4, 1),
                    m.getElementAt(4, 2), m.getElementAt(4, 3));
            plane6 = new Plane(m.getElementAt(5, 0), m.getElementAt(5, 1),
                    m.getElementAt(5, 2), m.getElementAt(5, 3));
            plane7 = new Plane(m.getElementAt(6, 0), m.getElementAt(6, 1),
                    m.getElementAt(6, 2), m.getElementAt(6, 3));
            plane8 = new Plane(m.getElementAt(7, 0), m.getElementAt(7, 1),
                    m.getElementAt(7, 2), m.getElementAt(7, 3));
            plane9 = new Plane(m.getElementAt(8, 0), m.getElementAt(8, 1),
                    m.getElementAt(8, 2), m.getElementAt(8, 3));

            plane1.normalize();
            plane2.normalize();
            plane3.normalize();
            plane4.normalize();
            plane5.normalize();
            plane6.normalize();
            plane7.normalize();
            plane8.normalize();
            plane9.normalize();

            // estimate dual quadric that lies inside of provided 9 planes

            m2 = new Matrix(9, 10);
            pA = plane1.getA();
            pB = plane1.getB();
            pC = plane1.getC();
            pD = plane1.getD();
            m2.setElementAt(0, 0, pA * pA);
            m2.setElementAt(0, 1, pB * pB);
            m2.setElementAt(0, 2, pC * pC);
            m2.setElementAt(0, 3, 2.0 * pA * pB);
            m2.setElementAt(0, 4, 2.0 * pA * pC);
            m2.setElementAt(0, 5, 2.0 * pB * pC);
            m2.setElementAt(0, 6, 2.0 * pA * pD);
            m2.setElementAt(0, 7, 2.0 * pB * pD);
            m2.setElementAt(0, 8, 2.0 * pC * pD);
            m2.setElementAt(0, 9, pD * pD);
            pA = plane2.getA();
            pB = plane2.getB();
            pC = plane2.getC();
            pD = plane2.getD();
            m2.setElementAt(1, 0, pA * pA);
            m2.setElementAt(1, 1, pB * pB);
            m2.setElementAt(1, 2, pC * pC);
            m2.setElementAt(1, 3, 2.0 * pA * pB);
            m2.setElementAt(1, 4, 2.0 * pA * pC);
            m2.setElementAt(1, 5, 2.0 * pB * pC);
            m2.setElementAt(1, 6, 2.0 * pA * pD);
            m2.setElementAt(1, 7, 2.0 * pB * pD);
            m2.setElementAt(1, 8, 2.0 * pC * pD);
            m2.setElementAt(1, 9, pD * pD);
            pA = plane3.getA();
            pB = plane3.getB();
            pC = plane3.getC();
            pD = plane3.getD();
            m2.setElementAt(2, 0, pA * pA);
            m2.setElementAt(2, 1, pB * pB);
            m2.setElementAt(2, 2, pC * pC);
            m2.setElementAt(2, 3, 2.0 * pA * pB);
            m2.setElementAt(2, 4, 2.0 * pA * pC);
            m2.setElementAt(2, 5, 2.0 * pB * pC);
            m2.setElementAt(2, 6, 2.0 * pA * pD);
            m2.setElementAt(2, 7, 2.0 * pB * pD);
            m2.setElementAt(2, 8, 2.0 * pC * pD);
            m2.setElementAt(2, 9, pD * pD);
            pA = plane4.getA();
            pB = plane4.getB();
            pC = plane4.getC();
            pD = plane4.getD();
            m2.setElementAt(3, 0, pA * pA);
            m2.setElementAt(3, 1, pB * pB);
            m2.setElementAt(3, 2, pC * pC);
            m2.setElementAt(3, 3, 2.0 * pA * pB);
            m2.setElementAt(3, 4, 2.0 * pA * pC);
            m2.setElementAt(3, 5, 2.0 * pB * pC);
            m2.setElementAt(3, 6, 2.0 * pA * pD);
            m2.setElementAt(3, 7, 2.0 * pB * pD);
            m2.setElementAt(3, 8, 2.0 * pC * pD);
            m2.setElementAt(3, 9, pD * pD);
            pA = plane5.getA();
            pB = plane5.getB();
            pC = plane5.getC();
            pD = plane5.getD();
            m2.setElementAt(4, 0, pA * pA);
            m2.setElementAt(4, 1, pB * pB);
            m2.setElementAt(4, 2, pC * pC);
            m2.setElementAt(4, 3, 2.0 * pA * pB);
            m2.setElementAt(4, 4, 2.0 * pA * pC);
            m2.setElementAt(4, 5, 2.0 * pB * pC);
            m2.setElementAt(4, 6, 2.0 * pA * pD);
            m2.setElementAt(4, 7, 2.0 * pB * pD);
            m2.setElementAt(4, 8, 2.0 * pC * pD);
            m2.setElementAt(4, 9, pD * pD);
            pA = plane6.getA();
            pB = plane6.getB();
            pC = plane6.getC();
            pD = plane6.getD();
            m2.setElementAt(5, 0, pA * pA);
            m2.setElementAt(5, 1, pB * pB);
            m2.setElementAt(5, 2, pC * pC);
            m2.setElementAt(5, 3, 2.0 * pA * pB);
            m2.setElementAt(5, 4, 2.0 * pA * pC);
            m2.setElementAt(5, 5, 2.0 * pB * pC);
            m2.setElementAt(5, 6, 2.0 * pA * pD);
            m2.setElementAt(5, 7, 2.0 * pB * pD);
            m2.setElementAt(5, 8, 2.0 * pC * pD);
            m2.setElementAt(5, 9, pD * pD);
            pA = plane7.getA();
            pB = plane7.getB();
            pC = plane7.getC();
            pD = plane7.getD();
            m2.setElementAt(6, 0, pA * pA);
            m2.setElementAt(6, 1, pB * pB);
            m2.setElementAt(6, 2, pC * pC);
            m2.setElementAt(6, 3, 2.0 * pA * pB);
            m2.setElementAt(6, 4, 2.0 * pA * pC);
            m2.setElementAt(6, 5, 2.0 * pB * pC);
            m2.setElementAt(6, 6, 2.0 * pA * pD);
            m2.setElementAt(6, 7, 2.0 * pB * pD);
            m2.setElementAt(6, 8, 2.0 * pC * pD);
            m2.setElementAt(6, 9, pD * pD);
            pA = plane8.getA();
            pB = plane8.getB();
            pC = plane8.getC();
            pD = plane8.getD();
            m2.setElementAt(7, 0, pA * pA);
            m2.setElementAt(7, 1, pB * pB);
            m2.setElementAt(7, 2, pC * pC);
            m2.setElementAt(7, 3, 2.0 * pA * pB);
            m2.setElementAt(7, 4, 2.0 * pA * pC);
            m2.setElementAt(7, 5, 2.0 * pB * pC);
            m2.setElementAt(7, 6, 2.0 * pA * pD);
            m2.setElementAt(7, 7, 2.0 * pB * pD);
            m2.setElementAt(7, 8, 2.0 * pC * pD);
            m2.setElementAt(7, 9, pD * pD);
            pA = plane9.getA();
            pB = plane9.getB();
            pC = plane9.getC();
            pD = plane9.getD();
            m2.setElementAt(8, 0, pA * pA);
            m2.setElementAt(8, 1, pB * pB);
            m2.setElementAt(8, 2, pC * pC);
            m2.setElementAt(8, 3, 2.0 * pA * pB);
            m2.setElementAt(8, 4, 2.0 * pA * pC);
            m2.setElementAt(8, 5, 2.0 * pB * pC);
            m2.setElementAt(8, 6, 2.0 * pA * pD);
            m2.setElementAt(8, 7, 2.0 * pB * pD);
            m2.setElementAt(8, 8, 2.0 * pC * pD);
            m2.setElementAt(8, 9, pD * pD);
        }

        dualQuadric = new DualQuadric(plane1, plane2, plane3, plane4, plane5,
                plane6, plane7, plane8, plane9);
        assertTrue(dualQuadric.isLocus(plane1, PRECISION_ERROR));
        assertTrue(dualQuadric.isLocus(plane2, PRECISION_ERROR));
        assertTrue(dualQuadric.isLocus(plane3, PRECISION_ERROR));
        assertTrue(dualQuadric.isLocus(plane4, PRECISION_ERROR));
        assertTrue(dualQuadric.isLocus(plane5, PRECISION_ERROR));
        assertTrue(dualQuadric.isLocus(plane6, PRECISION_ERROR));
        assertTrue(dualQuadric.isLocus(plane7, PRECISION_ERROR));
        assertTrue(dualQuadric.isLocus(plane8, PRECISION_ERROR));
        assertTrue(dualQuadric.isLocus(plane9, PRECISION_ERROR));

        // Force CoincidentPlanesException
        dualQuadric = null;
        try {
            dualQuadric = new DualQuadric(plane1, plane2, plane3, plane4,
                    plane5, plane6, plane7, plane8, plane8);
            fail("CoincidentPlanesException expected but not thrown");
        } catch (final CoincidentPlanesException ignore) {
        }
        assertNull(dualQuadric);
    }

    @Test
    public void testGettersAndSetters() throws WrongSizeException,
            IllegalArgumentException, NonSymmetricMatrixException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        DualQuadric dualQuadric = new DualQuadric();
        double a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double e = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double f = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double g = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double h = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double i = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double j = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        dualQuadric.setA(a);
        dualQuadric.setB(b);
        dualQuadric.setC(c);
        dualQuadric.setD(d);
        dualQuadric.setE(e);
        dualQuadric.setF(f);
        dualQuadric.setG(g);
        dualQuadric.setH(h);
        dualQuadric.setI(i);
        dualQuadric.setJ(j);
        assertEquals(dualQuadric.getA(), a, 0.0);
        assertEquals(dualQuadric.getB(), b, 0.0);
        assertEquals(dualQuadric.getC(), c, 0.0);
        assertEquals(dualQuadric.getD(), d, 0.0);
        assertEquals(dualQuadric.getE(), e, 0.0);
        assertEquals(dualQuadric.getF(), f, 0.0);
        assertEquals(dualQuadric.getG(), g, 0.0);
        assertEquals(dualQuadric.getH(), h, 0.0);
        assertEquals(dualQuadric.getI(), i, 0.0);
        assertEquals(dualQuadric.getJ(), j, 0.0);

        a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        e = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        f = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        g = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        h = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        i = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        j = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        dualQuadric.setParameters(a, b, c, d, e, f, g, h, i, j);
        assertEquals(dualQuadric.getA(), a, 0.0);
        assertEquals(dualQuadric.getB(), b, 0.0);
        assertEquals(dualQuadric.getC(), c, 0.0);
        assertEquals(dualQuadric.getD(), d, 0.0);
        assertEquals(dualQuadric.getE(), e, 0.0);
        assertEquals(dualQuadric.getF(), f, 0.0);
        assertEquals(dualQuadric.getG(), g, 0.0);
        assertEquals(dualQuadric.getH(), h, 0.0);
        assertEquals(dualQuadric.getI(), i, 0.0);
        assertEquals(dualQuadric.getJ(), j, 0.0);

        dualQuadric = new DualQuadric();
        final Matrix m = new Matrix(DUAL_QUADRIC_ROWS, DUAL_QUADRIC_COLS);
        a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        e = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        f = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        g = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        h = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        i = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        j = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        m.setElementAt(0, 0, a);
        m.setElementAt(1, 1, b);
        m.setElementAt(2, 2, c);
        m.setElementAt(3, 3, j);
        m.setElementAt(1, 0, d);
        m.setElementAt(0, 1, d);
        m.setElementAt(2, 1, e);
        m.setElementAt(1, 2, e);
        m.setElementAt(2, 0, f);
        m.setElementAt(0, 2, f);
        m.setElementAt(3, 0, g);
        m.setElementAt(0, 3, g);
        m.setElementAt(3, 1, h);
        m.setElementAt(1, 3, h);
        m.setElementAt(3, 2, i);
        m.setElementAt(2, 3, i);
        dualQuadric.setParameters(m);
        assertEquals(dualQuadric.getA(), m.getElementAt(0, 0), 0.0);
        assertEquals(dualQuadric.getB(), m.getElementAt(1, 1), 0.0);
        assertEquals(dualQuadric.getC(), m.getElementAt(2, 2), 0.0);
        assertEquals(dualQuadric.getD(), m.getElementAt(1, 0), 0.0);
        assertEquals(dualQuadric.getE(), m.getElementAt(2, 1), 0.0);
        assertEquals(dualQuadric.getF(), m.getElementAt(2, 0), 0.0);
        assertEquals(dualQuadric.getG(), m.getElementAt(3, 0), 0.0);
        assertEquals(dualQuadric.getH(), m.getElementAt(3, 1), 0.0);
        assertEquals(dualQuadric.getI(), m.getElementAt(3, 2), 0.0);
    }

    @Test
    public void testAsMatrix() throws WrongSizeException,
            IllegalArgumentException, NonSymmetricMatrixException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final DualQuadric dualQuadric = new DualQuadric();
        final Matrix m = new Matrix(DUAL_QUADRIC_ROWS, DUAL_QUADRIC_COLS);
        final double a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double e = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double f = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double g = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double h = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double i = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double j = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        m.setElementAt(0, 0, a);
        m.setElementAt(1, 1, b);
        m.setElementAt(2, 2, c);
        m.setElementAt(3, 3, j);
        m.setElementAt(1, 0, d);
        m.setElementAt(0, 1, d);
        m.setElementAt(2, 1, e);
        m.setElementAt(1, 2, e);
        m.setElementAt(2, 0, f);
        m.setElementAt(0, 2, f);
        m.setElementAt(3, 0, g);
        m.setElementAt(0, 3, g);
        m.setElementAt(3, 1, h);
        m.setElementAt(1, 3, h);
        m.setElementAt(3, 2, i);
        m.setElementAt(2, 3, i);
        dualQuadric.setParameters(m);
        final Matrix m2 = dualQuadric.asMatrix();

        assertTrue(m.equals(m2, PRECISION_ERROR));
    }

    @Test
    public void testIsLocus() throws WrongSizeException, DecomposerException,
            NotReadyException, LockedException,
            com.irurueta.algebra.NotAvailableException,
            RankDeficientMatrixException, IllegalArgumentException,
            NonSymmetricMatrixException {

        Matrix m = Matrix.createWithUniformRandomValues(9, HOM_COORDS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        Plane plane1 = new Plane(m.getElementAt(0, 0), m.getElementAt(0, 1),
                m.getElementAt(0, 2), m.getElementAt(0, 3));
        Plane plane2 = new Plane(m.getElementAt(1, 0), m.getElementAt(1, 1),
                m.getElementAt(1, 2), m.getElementAt(1, 3));
        Plane plane3 = new Plane(m.getElementAt(2, 0), m.getElementAt(2, 1),
                m.getElementAt(2, 2), m.getElementAt(2, 3));
        Plane plane4 = new Plane(m.getElementAt(3, 0), m.getElementAt(3, 1),
                m.getElementAt(3, 2), m.getElementAt(3, 3));
        Plane plane5 = new Plane(m.getElementAt(4, 0), m.getElementAt(4, 1),
                m.getElementAt(4, 2), m.getElementAt(4, 3));
        Plane plane6 = new Plane(m.getElementAt(5, 0), m.getElementAt(5, 1),
                m.getElementAt(5, 2), m.getElementAt(5, 3));
        Plane plane7 = new Plane(m.getElementAt(6, 0), m.getElementAt(6, 1),
                m.getElementAt(6, 2), m.getElementAt(6, 3));
        Plane plane8 = new Plane(m.getElementAt(7, 0), m.getElementAt(7, 1),
                m.getElementAt(7, 2), m.getElementAt(7, 3));
        Plane plane9 = new Plane(m.getElementAt(8, 0), m.getElementAt(8, 1),
                m.getElementAt(8, 2), m.getElementAt(8, 3));

        plane1.normalize();
        plane2.normalize();
        plane3.normalize();
        plane4.normalize();
        plane5.normalize();
        plane6.normalize();
        plane7.normalize();
        plane8.normalize();
        plane9.normalize();

        // estimate dual quadric that lies inside of provided 9 planes

        Matrix m2 = new Matrix(9, 10);
        double pA = plane1.getA();
        double pB = plane1.getB();
        double pC = plane1.getC();
        double pD = plane1.getD();
        m2.setElementAt(0, 0, pA * pA);
        m2.setElementAt(0, 1, pB * pB);
        m2.setElementAt(0, 2, pC * pC);
        m2.setElementAt(0, 3, 2.0 * pA * pB);
        m2.setElementAt(0, 4, 2.0 * pA * pC);
        m2.setElementAt(0, 5, 2.0 * pB * pC);
        m2.setElementAt(0, 6, 2.0 * pA * pD);
        m2.setElementAt(0, 7, 2.0 * pB * pD);
        m2.setElementAt(0, 8, 2.0 * pC * pD);
        m2.setElementAt(0, 9, pD * pD);
        pA = plane2.getA();
        pB = plane2.getB();
        pC = plane2.getC();
        pD = plane2.getD();
        m2.setElementAt(1, 0, pA * pA);
        m2.setElementAt(1, 1, pB * pB);
        m2.setElementAt(1, 2, pC * pC);
        m2.setElementAt(1, 3, 2.0 * pA * pB);
        m2.setElementAt(1, 4, 2.0 * pA * pC);
        m2.setElementAt(1, 5, 2.0 * pB * pC);
        m2.setElementAt(1, 6, 2.0 * pA * pD);
        m2.setElementAt(1, 7, 2.0 * pB * pD);
        m2.setElementAt(1, 8, 2.0 * pC * pD);
        m2.setElementAt(1, 9, pD * pD);
        pA = plane3.getA();
        pB = plane3.getB();
        pC = plane3.getC();
        pD = plane3.getD();
        m2.setElementAt(2, 0, pA * pA);
        m2.setElementAt(2, 1, pB * pB);
        m2.setElementAt(2, 2, pC * pC);
        m2.setElementAt(2, 3, 2.0 * pA * pB);
        m2.setElementAt(2, 4, 2.0 * pA * pC);
        m2.setElementAt(2, 5, 2.0 * pB * pC);
        m2.setElementAt(2, 6, 2.0 * pA * pD);
        m2.setElementAt(2, 7, 2.0 * pB * pD);
        m2.setElementAt(2, 8, 2.0 * pC * pD);
        m2.setElementAt(2, 9, pD * pD);
        pA = plane4.getA();
        pB = plane4.getB();
        pC = plane4.getC();
        pD = plane4.getD();
        m2.setElementAt(3, 0, pA * pA);
        m2.setElementAt(3, 1, pB * pB);
        m2.setElementAt(3, 2, pC * pC);
        m2.setElementAt(3, 3, 2.0 * pA * pB);
        m2.setElementAt(3, 4, 2.0 * pA * pC);
        m2.setElementAt(3, 5, 2.0 * pB * pC);
        m2.setElementAt(3, 6, 2.0 * pA * pD);
        m2.setElementAt(3, 7, 2.0 * pB * pD);
        m2.setElementAt(3, 8, 2.0 * pC * pD);
        m2.setElementAt(3, 9, pD * pD);
        pA = plane5.getA();
        pB = plane5.getB();
        pC = plane5.getC();
        pD = plane5.getD();
        m2.setElementAt(4, 0, pA * pA);
        m2.setElementAt(4, 1, pB * pB);
        m2.setElementAt(4, 2, pC * pC);
        m2.setElementAt(4, 3, 2.0 * pA * pB);
        m2.setElementAt(4, 4, 2.0 * pA * pC);
        m2.setElementAt(4, 5, 2.0 * pB * pC);
        m2.setElementAt(4, 6, 2.0 * pA * pD);
        m2.setElementAt(4, 7, 2.0 * pB * pD);
        m2.setElementAt(4, 8, 2.0 * pC * pD);
        m2.setElementAt(4, 9, pD * pD);
        pA = plane6.getA();
        pB = plane6.getB();
        pC = plane6.getC();
        pD = plane6.getD();
        m2.setElementAt(5, 0, pA * pA);
        m2.setElementAt(5, 1, pB * pB);
        m2.setElementAt(5, 2, pC * pC);
        m2.setElementAt(5, 3, 2.0 * pA * pB);
        m2.setElementAt(5, 4, 2.0 * pA * pC);
        m2.setElementAt(5, 5, 2.0 * pB * pC);
        m2.setElementAt(5, 6, 2.0 * pA * pD);
        m2.setElementAt(5, 7, 2.0 * pB * pD);
        m2.setElementAt(5, 8, 2.0 * pC * pD);
        m2.setElementAt(5, 9, pD * pD);
        pA = plane7.getA();
        pB = plane7.getB();
        pC = plane7.getC();
        pD = plane7.getD();
        m2.setElementAt(6, 0, pA * pA);
        m2.setElementAt(6, 1, pB * pB);
        m2.setElementAt(6, 2, pC * pC);
        m2.setElementAt(6, 3, 2.0 * pA * pB);
        m2.setElementAt(6, 4, 2.0 * pA * pC);
        m2.setElementAt(6, 5, 2.0 * pB * pC);
        m2.setElementAt(6, 6, 2.0 * pA * pD);
        m2.setElementAt(6, 7, 2.0 * pB * pD);
        m2.setElementAt(6, 8, 2.0 * pC * pD);
        m2.setElementAt(6, 9, pD * pD);
        pA = plane8.getA();
        pB = plane8.getB();
        pC = plane8.getC();
        pD = plane8.getD();
        m2.setElementAt(7, 0, pA * pA);
        m2.setElementAt(7, 1, pB * pB);
        m2.setElementAt(7, 2, pC * pC);
        m2.setElementAt(7, 3, 2.0 * pA * pB);
        m2.setElementAt(7, 4, 2.0 * pA * pC);
        m2.setElementAt(7, 5, 2.0 * pB * pC);
        m2.setElementAt(7, 6, 2.0 * pA * pD);
        m2.setElementAt(7, 7, 2.0 * pB * pD);
        m2.setElementAt(7, 8, 2.0 * pC * pD);
        m2.setElementAt(7, 9, pD * pD);
        pA = plane9.getA();
        pB = plane9.getB();
        pC = plane9.getC();
        pD = plane9.getD();
        m2.setElementAt(8, 0, pA * pA);
        m2.setElementAt(8, 1, pB * pB);
        m2.setElementAt(8, 2, pC * pC);
        m2.setElementAt(8, 3, 2.0 * pA * pB);
        m2.setElementAt(8, 4, 2.0 * pA * pC);
        m2.setElementAt(8, 5, 2.0 * pB * pC);
        m2.setElementAt(8, 6, 2.0 * pA * pD);
        m2.setElementAt(8, 7, 2.0 * pB * pD);
        m2.setElementAt(8, 8, 2.0 * pC * pD);
        m2.setElementAt(8, 9, pD * pD);

        while (com.irurueta.algebra.Utils.rank(m2) < 9) {
            m = Matrix.createWithUniformRandomValues(9, HOM_COORDS,
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            plane1 = new Plane(m.getElementAt(0, 0), m.getElementAt(0, 1),
                    m.getElementAt(0, 2), m.getElementAt(0, 3));
            plane2 = new Plane(m.getElementAt(1, 0), m.getElementAt(1, 1),
                    m.getElementAt(1, 2), m.getElementAt(1, 3));
            plane3 = new Plane(m.getElementAt(2, 0), m.getElementAt(2, 1),
                    m.getElementAt(2, 2), m.getElementAt(2, 3));
            plane4 = new Plane(m.getElementAt(3, 0), m.getElementAt(3, 1),
                    m.getElementAt(3, 2), m.getElementAt(3, 3));
            plane5 = new Plane(m.getElementAt(4, 0), m.getElementAt(4, 1),
                    m.getElementAt(4, 2), m.getElementAt(4, 3));
            plane6 = new Plane(m.getElementAt(5, 0), m.getElementAt(5, 1),
                    m.getElementAt(5, 2), m.getElementAt(5, 3));
            plane7 = new Plane(m.getElementAt(6, 0), m.getElementAt(6, 1),
                    m.getElementAt(6, 2), m.getElementAt(6, 3));
            plane8 = new Plane(m.getElementAt(7, 0), m.getElementAt(7, 1),
                    m.getElementAt(7, 2), m.getElementAt(7, 3));
            plane9 = new Plane(m.getElementAt(8, 0), m.getElementAt(8, 1),
                    m.getElementAt(8, 2), m.getElementAt(8, 3));

            plane1.normalize();
            plane2.normalize();
            plane3.normalize();
            plane4.normalize();
            plane5.normalize();
            plane6.normalize();
            plane7.normalize();
            plane8.normalize();
            plane9.normalize();

            // estimate dual quadric that lies inside of provided 9 planes

            m2 = new Matrix(9, 10);
            pA = plane1.getA();
            pB = plane1.getB();
            pC = plane1.getC();
            pD = plane1.getD();
            m2.setElementAt(0, 0, pA * pA);
            m2.setElementAt(0, 1, pB * pB);
            m2.setElementAt(0, 2, pC * pC);
            m2.setElementAt(0, 3, 2.0 * pA * pB);
            m2.setElementAt(0, 4, 2.0 * pA * pC);
            m2.setElementAt(0, 5, 2.0 * pB * pC);
            m2.setElementAt(0, 6, 2.0 * pA * pD);
            m2.setElementAt(0, 7, 2.0 * pB * pD);
            m2.setElementAt(0, 8, 2.0 * pC * pD);
            m2.setElementAt(0, 9, pD * pD);
            pA = plane2.getA();
            pB = plane2.getB();
            pC = plane2.getC();
            pD = plane2.getD();
            m2.setElementAt(1, 0, pA * pA);
            m2.setElementAt(1, 1, pB * pB);
            m2.setElementAt(1, 2, pC * pC);
            m2.setElementAt(1, 3, 2.0 * pA * pB);
            m2.setElementAt(1, 4, 2.0 * pA * pC);
            m2.setElementAt(1, 5, 2.0 * pB * pC);
            m2.setElementAt(1, 6, 2.0 * pA * pD);
            m2.setElementAt(1, 7, 2.0 * pB * pD);
            m2.setElementAt(1, 8, 2.0 * pC * pD);
            m2.setElementAt(1, 9, pD * pD);
            pA = plane3.getA();
            pB = plane3.getB();
            pC = plane3.getC();
            pD = plane3.getD();
            m2.setElementAt(2, 0, pA * pA);
            m2.setElementAt(2, 1, pB * pB);
            m2.setElementAt(2, 2, pC * pC);
            m2.setElementAt(2, 3, 2.0 * pA * pB);
            m2.setElementAt(2, 4, 2.0 * pA * pC);
            m2.setElementAt(2, 5, 2.0 * pB * pC);
            m2.setElementAt(2, 6, 2.0 * pA * pD);
            m2.setElementAt(2, 7, 2.0 * pB * pD);
            m2.setElementAt(2, 8, 2.0 * pC * pD);
            m2.setElementAt(2, 9, pD * pD);
            pA = plane4.getA();
            pB = plane4.getB();
            pC = plane4.getC();
            pD = plane4.getD();
            m2.setElementAt(3, 0, pA * pA);
            m2.setElementAt(3, 1, pB * pB);
            m2.setElementAt(3, 2, pC * pC);
            m2.setElementAt(3, 3, 2.0 * pA * pB);
            m2.setElementAt(3, 4, 2.0 * pA * pC);
            m2.setElementAt(3, 5, 2.0 * pB * pC);
            m2.setElementAt(3, 6, 2.0 * pA * pD);
            m2.setElementAt(3, 7, 2.0 * pB * pD);
            m2.setElementAt(3, 8, 2.0 * pC * pD);
            m2.setElementAt(3, 9, pD * pD);
            pA = plane5.getA();
            pB = plane5.getB();
            pC = plane5.getC();
            pD = plane5.getD();
            m2.setElementAt(4, 0, pA * pA);
            m2.setElementAt(4, 1, pB * pB);
            m2.setElementAt(4, 2, pC * pC);
            m2.setElementAt(4, 3, 2.0 * pA * pB);
            m2.setElementAt(4, 4, 2.0 * pA * pC);
            m2.setElementAt(4, 5, 2.0 * pB * pC);
            m2.setElementAt(4, 6, 2.0 * pA * pD);
            m2.setElementAt(4, 7, 2.0 * pB * pD);
            m2.setElementAt(4, 8, 2.0 * pC * pD);
            m2.setElementAt(4, 9, pD * pD);
            pA = plane6.getA();
            pB = plane6.getB();
            pC = plane6.getC();
            pD = plane6.getD();
            m2.setElementAt(5, 0, pA * pA);
            m2.setElementAt(5, 1, pB * pB);
            m2.setElementAt(5, 2, pC * pC);
            m2.setElementAt(5, 3, 2.0 * pA * pB);
            m2.setElementAt(5, 4, 2.0 * pA * pC);
            m2.setElementAt(5, 5, 2.0 * pB * pC);
            m2.setElementAt(5, 6, 2.0 * pA * pD);
            m2.setElementAt(5, 7, 2.0 * pB * pD);
            m2.setElementAt(5, 8, 2.0 * pC * pD);
            m2.setElementAt(5, 9, pD * pD);
            pA = plane7.getA();
            pB = plane7.getB();
            pC = plane7.getC();
            pD = plane7.getD();
            m2.setElementAt(6, 0, pA * pA);
            m2.setElementAt(6, 1, pB * pB);
            m2.setElementAt(6, 2, pC * pC);
            m2.setElementAt(6, 3, 2.0 * pA * pB);
            m2.setElementAt(6, 4, 2.0 * pA * pC);
            m2.setElementAt(6, 5, 2.0 * pB * pC);
            m2.setElementAt(6, 6, 2.0 * pA * pD);
            m2.setElementAt(6, 7, 2.0 * pB * pD);
            m2.setElementAt(6, 8, 2.0 * pC * pD);
            m2.setElementAt(6, 9, pD * pD);
            pA = plane8.getA();
            pB = plane8.getB();
            pC = plane8.getC();
            pD = plane8.getD();
            m2.setElementAt(7, 0, pA * pA);
            m2.setElementAt(7, 1, pB * pB);
            m2.setElementAt(7, 2, pC * pC);
            m2.setElementAt(7, 3, 2.0 * pA * pB);
            m2.setElementAt(7, 4, 2.0 * pA * pC);
            m2.setElementAt(7, 5, 2.0 * pB * pC);
            m2.setElementAt(7, 6, 2.0 * pA * pD);
            m2.setElementAt(7, 7, 2.0 * pB * pD);
            m2.setElementAt(7, 8, 2.0 * pC * pD);
            m2.setElementAt(7, 9, pD * pD);
            pA = plane9.getA();
            pB = plane9.getB();
            pC = plane9.getC();
            pD = plane9.getD();
            m2.setElementAt(8, 0, pA * pA);
            m2.setElementAt(8, 1, pB * pB);
            m2.setElementAt(8, 2, pC * pC);
            m2.setElementAt(8, 3, 2.0 * pA * pB);
            m2.setElementAt(8, 4, 2.0 * pA * pC);
            m2.setElementAt(8, 5, 2.0 * pB * pC);
            m2.setElementAt(8, 6, 2.0 * pA * pD);
            m2.setElementAt(8, 7, 2.0 * pB * pD);
            m2.setElementAt(8, 8, 2.0 * pC * pD);
            m2.setElementAt(8, 9, pD * pD);
        }

        final SingularValueDecomposer decomposer = new SingularValueDecomposer(m2);
        decomposer.decompose();

        final Matrix v = decomposer.getV();

        final double a = v.getElementAt(0, 9);
        final double b = v.getElementAt(1, 9);
        final double c = v.getElementAt(2, 9);
        final double d = v.getElementAt(3, 9);

        final double f = v.getElementAt(4, 9);
        final double e = v.getElementAt(5, 9);

        final double g = v.getElementAt(6, 9);
        final double h = v.getElementAt(7, 9);
        final double i = v.getElementAt(8, 9);
        final double j = v.getElementAt(9, 9);

        final Matrix dualQuadricPlane = new Matrix(HOM_COORDS, 1);
        dualQuadricPlane.setElementAt(0, 0, plane1.getA());
        dualQuadricPlane.setElementAt(1, 0, plane1.getB());
        dualQuadricPlane.setElementAt(2, 0, plane1.getC());

        double norm = com.irurueta.algebra.Utils.normF(dualQuadricPlane);
        dualQuadricPlane.multiplyByScalar(1.0 / norm);

        final Matrix dualQuadricMatrix = new Matrix(DUAL_QUADRIC_ROWS,
                DUAL_QUADRIC_COLS);
        dualQuadricMatrix.setElementAt(0, 0, a);
        dualQuadricMatrix.setElementAt(1, 1, b);
        dualQuadricMatrix.setElementAt(2, 2, c);
        dualQuadricMatrix.setElementAt(3, 3, j);
        dualQuadricMatrix.setElementAt(1, 0, d);
        dualQuadricMatrix.setElementAt(0, 1, d);
        dualQuadricMatrix.setElementAt(2, 1, e);
        dualQuadricMatrix.setElementAt(1, 2, e);
        dualQuadricMatrix.setElementAt(2, 0, f);
        dualQuadricMatrix.setElementAt(0, 2, f);
        dualQuadricMatrix.setElementAt(3, 0, g);
        dualQuadricMatrix.setElementAt(0, 3, g);
        dualQuadricMatrix.setElementAt(3, 1, h);
        dualQuadricMatrix.setElementAt(1, 3, h);
        dualQuadricMatrix.setElementAt(3, 2, i);
        dualQuadricMatrix.setElementAt(2, 3, i);

        norm = com.irurueta.algebra.Utils.normF(dualQuadricMatrix);
        dualQuadricMatrix.multiplyByScalar(1.0 / norm);

        // find point where line is tangent to quadric
        final Matrix homPointMatrix = dualQuadricMatrix.multiplyAndReturnNew(
                dualQuadricPlane);

        // add director vector of tangent plane to get a point outside of quadric
        double directVectorA = dualQuadricPlane.getElementAtIndex(0);
        double directVectorB = dualQuadricPlane.getElementAtIndex(1);
        double directVectorC = dualQuadricPlane.getElementAtIndex(2);
        final double directVectorNorm = Math.sqrt(directVectorA * directVectorA +
                directVectorB * directVectorB + directVectorC * directVectorC);
        directVectorA /= directVectorNorm;
        directVectorB /= directVectorNorm;
        directVectorC /= directVectorNorm;
        homPointMatrix.setElementAtIndex(0, homPointMatrix.getElementAtIndex(0)
                + directVectorA * homPointMatrix.getElementAtIndex(3));
        homPointMatrix.setElementAtIndex(0, homPointMatrix.getElementAtIndex(1)
                + directVectorB * homPointMatrix.getElementAtIndex(3));
        homPointMatrix.setElementAtIndex(0, homPointMatrix.getElementAtIndex(2)
                + directVectorC * homPointMatrix.getElementAtIndex(3));

        norm = com.irurueta.algebra.Utils.normF(homPointMatrix);
        //TODO: add divideByScalar
        homPointMatrix.multiplyByScalar(1.0 / norm);

        // get quadric matrix by inverting dual quadric matrix
        Matrix quadricMatrix = com.irurueta.algebra.Utils.inverse(
                dualQuadricMatrix);

        // find plane vector outside of dual quadric as the product of quadric
        // matrix and point outside of quadric
        final Matrix outsidePlaneMatrix = quadricMatrix.multiplyAndReturnNew(
                homPointMatrix);

        // instantiate plane outside of dual quadric using computed vector
        final Plane outsidePlane = new Plane(outsidePlaneMatrix.toArray());

        // instantiate new dual quadric instance
        final DualQuadric dualQuadric = new DualQuadric(dualQuadricMatrix);

        // check that initial 9 planes lie inside the dual quadric
        assertTrue(dualQuadric.isLocus(plane1, LOCUS_THRESHOLD));
        assertTrue(dualQuadric.isLocus(plane2, LOCUS_THRESHOLD));
        assertTrue(dualQuadric.isLocus(plane3, LOCUS_THRESHOLD));
        assertTrue(dualQuadric.isLocus(plane4, LOCUS_THRESHOLD));
        assertTrue(dualQuadric.isLocus(plane5, LOCUS_THRESHOLD));
        assertTrue(dualQuadric.isLocus(plane6, LOCUS_THRESHOLD));
        assertTrue(dualQuadric.isLocus(plane7, LOCUS_THRESHOLD));
        assertTrue(dualQuadric.isLocus(plane8, LOCUS_THRESHOLD));
        assertTrue(dualQuadric.isLocus(plane9, LOCUS_THRESHOLD));

        // check plane outside of dual quadric
        assertFalse(dualQuadric.isLocus(outsidePlane, LOCUS_THRESHOLD));
    }

    @Test
    public void testAngleBetweenPlanes() throws WrongSizeException,
            IllegalArgumentException, NonSymmetricMatrixException {

        // initial planes
        final Matrix planeMatrix1 = Matrix.createWithUniformRandomValues(HOM_COORDS,
                1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final Matrix planeMatrix2 = Matrix.createWithUniformRandomValues(HOM_COORDS,
                1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // transformation matrix
        final Matrix transform = Matrix.createWithUniformRandomValues(HOM_COORDS,
                HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // transform planes
        final Matrix tPlane1 = transform.multiplyAndReturnNew(planeMatrix1);
        final Matrix tPlane2 = transform.multiplyAndReturnNew(planeMatrix2);

        final double norm1 = com.irurueta.algebra.Utils.normF(tPlane1);
        final double norm2 = com.irurueta.algebra.Utils.normF(tPlane2);

        final double numerator = tPlane1.transposeAndReturnNew().multiplyAndReturnNew(
                tPlane2).getElementAt(0, 0);

        final double cosAngle = numerator / (norm1 * norm2);

        final double angle = Math.acos(cosAngle);

        // compute dual quadric matrix as the product of transposed transform
        // matrix with itself
        final Matrix transposedTransform = transform.transposeAndReturnNew();
        final Matrix dualQuadricMatrix = transposedTransform.multiplyAndReturnNew(
                transform);

        // normalize conic matrix
        final double normDualQuadric = com.irurueta.algebra.Utils.normF(
                dualQuadricMatrix);
        dualQuadricMatrix.multiplyByScalar(1.0 / normDualQuadric);

        final DualQuadric dualQuadric = new DualQuadric(dualQuadricMatrix);

        final Plane plane1 = new Plane(planeMatrix1.getElementAt(0, 0),
                planeMatrix1.getElementAt(1, 0),
                planeMatrix1.getElementAt(2, 0),
                planeMatrix1.getElementAt(3, 0));
        final Plane plane2 = new Plane(planeMatrix2.getElementAt(0, 0),
                planeMatrix2.getElementAt(1, 0),
                planeMatrix2.getElementAt(2, 0),
                planeMatrix2.getElementAt(3, 0));

        assertEquals(dualQuadric.angleBetweenPlanes(plane1, plane2), angle,
                PRECISION_ERROR);
    }

    @Test
    public void testArePerpendicularPlanes() throws WrongSizeException,
            DecomposerException, RankDeficientMatrixException,
            IllegalArgumentException, NonSymmetricMatrixException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            // trying perpendicular angle
            Matrix matrixPlane1 = Matrix.createWithUniformRandomValues(HOM_COORDS,
                    1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            double norm = com.irurueta.algebra.Utils.normF(matrixPlane1);
            matrixPlane1.multiplyByScalar(1.0 / norm);

            Matrix matrixPlane2 = new Matrix(HOM_COORDS, 1);
            matrixPlane2.setElementAt(0, 0, matrixPlane1.getElementAt(1, 0) +
                    matrixPlane1.getElementAt(2, 0));
            matrixPlane2.setElementAt(1, 0, -matrixPlane1.getElementAt(0, 0));
            matrixPlane2.setElementAt(2, 0, -matrixPlane1.getElementAt(0, 0));

            norm = com.irurueta.algebra.Utils.normF(matrixPlane2);
            matrixPlane2.multiplyByScalar(1.0 / norm);

            Matrix transform = Matrix.createWithUniformRandomValues(HOM_COORDS,
                    HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            while (com.irurueta.algebra.Utils.rank(transform) < 3) {
                transform = Matrix.createWithUniformRandomValues(HOM_COORDS,
                        HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            }

            final Matrix invTransform = com.irurueta.algebra.Utils.inverse(transform);

            Matrix transformPlaneMatrix1 = transform.multiplyAndReturnNew(
                    matrixPlane1);
            Matrix transformPlaneMatrix2 = transform.multiplyAndReturnNew(
                    matrixPlane2);

            final Matrix transInvTransform = invTransform.transposeAndReturnNew();

            final Matrix dualQuadricMatrix = transInvTransform.multiplyAndReturnNew(
                    invTransform);
            norm = com.irurueta.algebra.Utils.normF(dualQuadricMatrix);
            dualQuadricMatrix.multiplyByScalar(1.0 / norm);

            Plane transformPlane1 = new Plane(transformPlaneMatrix1.toArray());
            Plane transformPlane2 = new Plane(transformPlaneMatrix2.toArray());

            DualQuadric dualQuadric = new DualQuadric(dualQuadricMatrix);

            assertTrue(dualQuadric.arePerpendicularPlanes(transformPlane1,
                    transformPlane2, PERPENDICULAR_THRESHOLD));

            // trying non-perpendicular points
            double dotProduct;

            matrixPlane1 = Matrix.createWithUniformRandomValues(HOM_COORDS, 1,
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            norm = com.irurueta.algebra.Utils.normF(matrixPlane1);
            matrixPlane1.multiplyByScalar(1.0 / norm);

            matrixPlane2 = Matrix.createWithUniformRandomValues(HOM_COORDS, 1,
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            norm = com.irurueta.algebra.Utils.normF(matrixPlane2);
            matrixPlane2.multiplyByScalar(1.0 / norm);

            dotProduct = matrixPlane1.transposeAndReturnNew().multiplyAndReturnNew(
                    matrixPlane2).getElementAt(0, 0);

            // ensure lines are not perpendicular
            while (Math.abs(dotProduct) < PERPENDICULAR_THRESHOLD) {
                matrixPlane1 = Matrix.createWithUniformRandomValues(HOM_COORDS, 1,
                        MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                norm = com.irurueta.algebra.Utils.normF(matrixPlane1);
                matrixPlane1.multiplyByScalar(1.0 / norm);

                matrixPlane2 = Matrix.createWithUniformRandomValues(HOM_COORDS, 1,
                        MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                norm = com.irurueta.algebra.Utils.normF(matrixPlane2);
                matrixPlane2.multiplyByScalar(1.0 / norm);

                dotProduct = matrixPlane1.transposeAndReturnNew().
                        multiplyAndReturnNew(matrixPlane2).getElementAt(0, 0);
            }

            transformPlaneMatrix1 = transform.multiplyAndReturnNew(matrixPlane1);
            transformPlaneMatrix2 = transform.multiplyAndReturnNew(matrixPlane2);

            transformPlane1 = new Plane(transformPlaneMatrix1.toArray());
            transformPlane2 = new Plane(transformPlaneMatrix2.toArray());

            if (dualQuadric.arePerpendicularPlanes(transformPlane1, transformPlane2, PERPENDICULAR_THRESHOLD)) {
                continue;
            }
            assertFalse(dualQuadric.arePerpendicularPlanes(transformPlane1,
                    transformPlane2, PERPENDICULAR_THRESHOLD));

            dualQuadric = DualQuadric.createCanonicalDualAbsoluteQuadric();
            transformPlane1 = new Plane(1.0, 0.0, 0.0, 0.0);
            transformPlane2 = new Plane(0.0, 1.0, 0.0, 0.0);
            assertTrue(dualQuadric.arePerpendicularPlanes(transformPlane1, transformPlane2));

            numValid++;
            break;
        }
        assertTrue(numValid > 0);
    }

    @Test
    public void testGetQuadric() throws WrongSizeException, DecomposerException,
            RankDeficientMatrixException, IllegalArgumentException,
            NonSymmetricMatrixException, QuadricNotAvailableException {

        Matrix transformMatrix = Matrix.createWithUniformRandomValues(
                HOM_COORDS, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        while (com.irurueta.algebra.Utils.rank(transformMatrix) != 4) {
            transformMatrix = Matrix.createWithUniformRandomValues(HOM_COORDS,
                    HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        }

        final Matrix transformTransposedMatrix =
                transformMatrix.transposeAndReturnNew();

        final Matrix dualQuadricMatrix =
                transformTransposedMatrix.multiplyAndReturnNew(transformMatrix);

        final Matrix quadricMatrix1 = com.irurueta.algebra.Utils.inverse(
                dualQuadricMatrix);

        final DualQuadric dualQuadric = new DualQuadric(dualQuadricMatrix);

        final Quadric quadric = dualQuadric.getQuadric();
        final Quadric quadric2 = new Quadric();
        dualQuadric.quadric(quadric2);

        assertEquals(quadric.asMatrix(), quadric2.asMatrix());

        final Matrix quadricMatrix2 = quadric.asMatrix();

        // normalize quadric matrices
        double norm = com.irurueta.algebra.Utils.normF(quadricMatrix1);
        quadricMatrix1.multiplyByScalar(1.0 / norm);

        norm = com.irurueta.algebra.Utils.normF(quadricMatrix2);
        quadricMatrix2.multiplyByScalar(1.0 / norm);

        // compute difference of normalized quadric matrices
        final Matrix diffMatrix = quadricMatrix1.subtractAndReturnNew(quadricMatrix2);

        // ensure that difference matrix is almost zero by checking its norm
        norm = com.irurueta.algebra.Utils.normF(diffMatrix);
        assertEquals(norm, 0.0, PRECISION_ERROR);
    }

    @Test
    public void testSetParametersFromPlanes() throws WrongSizeException, DecomposerException, CoincidentPlanesException {
        Matrix m = Matrix.createWithUniformRandomValues(9, HOM_COORDS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        Plane plane1 = new Plane(m.getElementAt(0, 0), m.getElementAt(0, 1),
                m.getElementAt(0, 2), m.getElementAt(0, 3));
        Plane plane2 = new Plane(m.getElementAt(1, 0), m.getElementAt(1, 1),
                m.getElementAt(1, 2), m.getElementAt(1, 3));
        Plane plane3 = new Plane(m.getElementAt(2, 0), m.getElementAt(2, 1),
                m.getElementAt(2, 2), m.getElementAt(2, 3));
        Plane plane4 = new Plane(m.getElementAt(3, 0), m.getElementAt(3, 1),
                m.getElementAt(3, 2), m.getElementAt(3, 3));
        Plane plane5 = new Plane(m.getElementAt(4, 0), m.getElementAt(4, 1),
                m.getElementAt(4, 2), m.getElementAt(4, 3));
        Plane plane6 = new Plane(m.getElementAt(5, 0), m.getElementAt(5, 1),
                m.getElementAt(5, 2), m.getElementAt(5, 3));
        Plane plane7 = new Plane(m.getElementAt(6, 0), m.getElementAt(6, 1),
                m.getElementAt(6, 2), m.getElementAt(6, 3));
        Plane plane8 = new Plane(m.getElementAt(7, 0), m.getElementAt(7, 1),
                m.getElementAt(7, 2), m.getElementAt(7, 3));
        Plane plane9 = new Plane(m.getElementAt(8, 0), m.getElementAt(8, 1),
                m.getElementAt(8, 2), m.getElementAt(8, 3));

        plane1.normalize();
        plane2.normalize();
        plane3.normalize();
        plane4.normalize();
        plane5.normalize();
        plane6.normalize();
        plane7.normalize();
        plane8.normalize();
        plane9.normalize();

        // estimate dual quadric that lies inside of provided 9 planes
        Matrix m2 = new Matrix(9, 10);

        double pA = plane1.getA();
        double pB = plane1.getB();
        double pC = plane1.getC();
        double pD = plane1.getD();
        m2.setElementAt(0, 0, pA * pA);
        m2.setElementAt(0, 1, pB * pB);
        m2.setElementAt(0, 2, pC * pC);
        m2.setElementAt(0, 3, 2.0 * pA * pB);
        m2.setElementAt(0, 4, 2.0 * pA * pC);
        m2.setElementAt(0, 5, 2.0 * pB * pC);
        m2.setElementAt(0, 6, 2.0 * pA * pD);
        m2.setElementAt(0, 7, 2.0 * pB * pD);
        m2.setElementAt(0, 8, 2.0 * pC * pD);
        m2.setElementAt(0, 9, pD * pD);
        pA = plane2.getA();
        pB = plane2.getB();
        pC = plane2.getC();
        pD = plane2.getD();
        m2.setElementAt(1, 0, pA * pA);
        m2.setElementAt(1, 1, pB * pB);
        m2.setElementAt(1, 2, pC * pC);
        m2.setElementAt(1, 3, 2.0 * pA * pB);
        m2.setElementAt(1, 4, 2.0 * pA * pC);
        m2.setElementAt(1, 5, 2.0 * pB * pC);
        m2.setElementAt(1, 6, 2.0 * pA * pD);
        m2.setElementAt(1, 7, 2.0 * pB * pD);
        m2.setElementAt(1, 8, 2.0 * pC * pD);
        m2.setElementAt(1, 9, pD * pD);
        pA = plane3.getA();
        pB = plane3.getB();
        pC = plane3.getC();
        pD = plane3.getD();
        m2.setElementAt(2, 0, pA * pA);
        m2.setElementAt(2, 1, pB * pB);
        m2.setElementAt(2, 2, pC * pC);
        m2.setElementAt(2, 3, 2.0 * pA * pB);
        m2.setElementAt(2, 4, 2.0 * pA * pC);
        m2.setElementAt(2, 5, 2.0 * pB * pC);
        m2.setElementAt(2, 6, 2.0 * pA * pD);
        m2.setElementAt(2, 7, 2.0 * pB * pD);
        m2.setElementAt(2, 8, 2.0 * pC * pD);
        m2.setElementAt(2, 9, pD * pD);
        pA = plane4.getA();
        pB = plane4.getB();
        pC = plane4.getC();
        pD = plane4.getD();
        m2.setElementAt(3, 0, pA * pA);
        m2.setElementAt(3, 1, pB * pB);
        m2.setElementAt(3, 2, pC * pC);
        m2.setElementAt(3, 3, 2.0 * pA * pB);
        m2.setElementAt(3, 4, 2.0 * pA * pC);
        m2.setElementAt(3, 5, 2.0 * pB * pC);
        m2.setElementAt(3, 6, 2.0 * pA * pD);
        m2.setElementAt(3, 7, 2.0 * pB * pD);
        m2.setElementAt(3, 8, 2.0 * pC * pD);
        m2.setElementAt(3, 9, pD * pD);
        pA = plane5.getA();
        pB = plane5.getB();
        pC = plane5.getC();
        pD = plane5.getD();
        m2.setElementAt(4, 0, pA * pA);
        m2.setElementAt(4, 1, pB * pB);
        m2.setElementAt(4, 2, pC * pC);
        m2.setElementAt(4, 3, 2.0 * pA * pB);
        m2.setElementAt(4, 4, 2.0 * pA * pC);
        m2.setElementAt(4, 5, 2.0 * pB * pC);
        m2.setElementAt(4, 6, 2.0 * pA * pD);
        m2.setElementAt(4, 7, 2.0 * pB * pD);
        m2.setElementAt(4, 8, 2.0 * pC * pD);
        m2.setElementAt(4, 9, pD * pD);
        pA = plane6.getA();
        pB = plane6.getB();
        pC = plane6.getC();
        pD = plane6.getD();
        m2.setElementAt(5, 0, pA * pA);
        m2.setElementAt(5, 1, pB * pB);
        m2.setElementAt(5, 2, pC * pC);
        m2.setElementAt(5, 3, 2.0 * pA * pB);
        m2.setElementAt(5, 4, 2.0 * pA * pC);
        m2.setElementAt(5, 5, 2.0 * pB * pC);
        m2.setElementAt(5, 6, 2.0 * pA * pD);
        m2.setElementAt(5, 7, 2.0 * pB * pD);
        m2.setElementAt(5, 8, 2.0 * pC * pD);
        m2.setElementAt(5, 9, pD * pD);
        pA = plane7.getA();
        pB = plane7.getB();
        pC = plane7.getC();
        pD = plane7.getD();
        m2.setElementAt(6, 0, pA * pA);
        m2.setElementAt(6, 1, pB * pB);
        m2.setElementAt(6, 2, pC * pC);
        m2.setElementAt(6, 3, 2.0 * pA * pB);
        m2.setElementAt(6, 4, 2.0 * pA * pC);
        m2.setElementAt(6, 5, 2.0 * pB * pC);
        m2.setElementAt(6, 6, 2.0 * pA * pD);
        m2.setElementAt(6, 7, 2.0 * pB * pD);
        m2.setElementAt(6, 8, 2.0 * pC * pD);
        m2.setElementAt(6, 9, pD * pD);
        pA = plane8.getA();
        pB = plane8.getB();
        pC = plane8.getC();
        pD = plane8.getD();
        m2.setElementAt(7, 0, pA * pA);
        m2.setElementAt(7, 1, pB * pB);
        m2.setElementAt(7, 2, pC * pC);
        m2.setElementAt(7, 3, 2.0 * pA * pB);
        m2.setElementAt(7, 4, 2.0 * pA * pC);
        m2.setElementAt(7, 5, 2.0 * pB * pC);
        m2.setElementAt(7, 6, 2.0 * pA * pD);
        m2.setElementAt(7, 7, 2.0 * pB * pD);
        m2.setElementAt(7, 8, 2.0 * pC * pD);
        m2.setElementAt(7, 9, pD * pD);
        pA = plane9.getA();
        pB = plane9.getB();
        pC = plane9.getC();
        pD = plane9.getD();
        m2.setElementAt(8, 0, pA * pA);
        m2.setElementAt(8, 1, pB * pB);
        m2.setElementAt(8, 2, pC * pC);
        m2.setElementAt(8, 3, 2.0 * pA * pB);
        m2.setElementAt(8, 4, 2.0 * pA * pC);
        m2.setElementAt(8, 5, 2.0 * pB * pC);
        m2.setElementAt(8, 6, 2.0 * pA * pD);
        m2.setElementAt(8, 7, 2.0 * pB * pD);
        m2.setElementAt(8, 8, 2.0 * pC * pD);
        m2.setElementAt(8, 9, pD * pD);

        while (com.irurueta.algebra.Utils.rank(m2) < 9) {
            m = Matrix.createWithUniformRandomValues(9, HOM_COORDS,
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            plane1 = new Plane(m.getElementAt(0, 0), m.getElementAt(0, 1),
                    m.getElementAt(0, 2), m.getElementAt(0, 3));
            plane2 = new Plane(m.getElementAt(1, 0), m.getElementAt(1, 1),
                    m.getElementAt(1, 2), m.getElementAt(1, 3));
            plane3 = new Plane(m.getElementAt(2, 0), m.getElementAt(2, 1),
                    m.getElementAt(2, 2), m.getElementAt(2, 3));
            plane4 = new Plane(m.getElementAt(3, 0), m.getElementAt(3, 1),
                    m.getElementAt(3, 2), m.getElementAt(3, 3));
            plane5 = new Plane(m.getElementAt(4, 0), m.getElementAt(4, 1),
                    m.getElementAt(4, 2), m.getElementAt(4, 3));
            plane6 = new Plane(m.getElementAt(5, 0), m.getElementAt(5, 1),
                    m.getElementAt(5, 2), m.getElementAt(5, 3));
            plane7 = new Plane(m.getElementAt(6, 0), m.getElementAt(6, 1),
                    m.getElementAt(6, 2), m.getElementAt(6, 3));
            plane8 = new Plane(m.getElementAt(7, 0), m.getElementAt(7, 1),
                    m.getElementAt(7, 2), m.getElementAt(7, 3));
            plane9 = new Plane(m.getElementAt(8, 0), m.getElementAt(8, 1),
                    m.getElementAt(8, 2), m.getElementAt(8, 3));

            plane1.normalize();
            plane2.normalize();
            plane3.normalize();
            plane4.normalize();
            plane5.normalize();
            plane6.normalize();
            plane7.normalize();
            plane8.normalize();
            plane9.normalize();

            // estimate dual quadric that lies inside of provided 9 planes

            m2 = new Matrix(9, 10);
            pA = plane1.getA();
            pB = plane1.getB();
            pC = plane1.getC();
            pD = plane1.getD();
            m2.setElementAt(0, 0, pA * pA);
            m2.setElementAt(0, 1, pB * pB);
            m2.setElementAt(0, 2, pC * pC);
            m2.setElementAt(0, 3, 2.0 * pA * pB);
            m2.setElementAt(0, 4, 2.0 * pA * pC);
            m2.setElementAt(0, 5, 2.0 * pB * pC);
            m2.setElementAt(0, 6, 2.0 * pA * pD);
            m2.setElementAt(0, 7, 2.0 * pB * pD);
            m2.setElementAt(0, 8, 2.0 * pC * pD);
            m2.setElementAt(0, 9, pD * pD);
            pA = plane2.getA();
            pB = plane2.getB();
            pC = plane2.getC();
            pD = plane2.getD();
            m2.setElementAt(1, 0, pA * pA);
            m2.setElementAt(1, 1, pB * pB);
            m2.setElementAt(1, 2, pC * pC);
            m2.setElementAt(1, 3, 2.0 * pA * pB);
            m2.setElementAt(1, 4, 2.0 * pA * pC);
            m2.setElementAt(1, 5, 2.0 * pB * pC);
            m2.setElementAt(1, 6, 2.0 * pA * pD);
            m2.setElementAt(1, 7, 2.0 * pB * pD);
            m2.setElementAt(1, 8, 2.0 * pC * pD);
            m2.setElementAt(1, 9, pD * pD);
            pA = plane3.getA();
            pB = plane3.getB();
            pC = plane3.getC();
            pD = plane3.getD();
            m2.setElementAt(2, 0, pA * pA);
            m2.setElementAt(2, 1, pB * pB);
            m2.setElementAt(2, 2, pC * pC);
            m2.setElementAt(2, 3, 2.0 * pA * pB);
            m2.setElementAt(2, 4, 2.0 * pA * pC);
            m2.setElementAt(2, 5, 2.0 * pB * pC);
            m2.setElementAt(2, 6, 2.0 * pA * pD);
            m2.setElementAt(2, 7, 2.0 * pB * pD);
            m2.setElementAt(2, 8, 2.0 * pC * pD);
            m2.setElementAt(2, 9, pD * pD);
            pA = plane4.getA();
            pB = plane4.getB();
            pC = plane4.getC();
            pD = plane4.getD();
            m2.setElementAt(3, 0, pA * pA);
            m2.setElementAt(3, 1, pB * pB);
            m2.setElementAt(3, 2, pC * pC);
            m2.setElementAt(3, 3, 2.0 * pA * pB);
            m2.setElementAt(3, 4, 2.0 * pA * pC);
            m2.setElementAt(3, 5, 2.0 * pB * pC);
            m2.setElementAt(3, 6, 2.0 * pA * pD);
            m2.setElementAt(3, 7, 2.0 * pB * pD);
            m2.setElementAt(3, 8, 2.0 * pC * pD);
            m2.setElementAt(3, 9, pD * pD);
            pA = plane5.getA();
            pB = plane5.getB();
            pC = plane5.getC();
            pD = plane5.getD();
            m2.setElementAt(4, 0, pA * pA);
            m2.setElementAt(4, 1, pB * pB);
            m2.setElementAt(4, 2, pC * pC);
            m2.setElementAt(4, 3, 2.0 * pA * pB);
            m2.setElementAt(4, 4, 2.0 * pA * pC);
            m2.setElementAt(4, 5, 2.0 * pB * pC);
            m2.setElementAt(4, 6, 2.0 * pA * pD);
            m2.setElementAt(4, 7, 2.0 * pB * pD);
            m2.setElementAt(4, 8, 2.0 * pC * pD);
            m2.setElementAt(4, 9, pD * pD);
            pA = plane6.getA();
            pB = plane6.getB();
            pC = plane6.getC();
            pD = plane6.getD();
            m2.setElementAt(5, 0, pA * pA);
            m2.setElementAt(5, 1, pB * pB);
            m2.setElementAt(5, 2, pC * pC);
            m2.setElementAt(5, 3, 2.0 * pA * pB);
            m2.setElementAt(5, 4, 2.0 * pA * pC);
            m2.setElementAt(5, 5, 2.0 * pB * pC);
            m2.setElementAt(5, 6, 2.0 * pA * pD);
            m2.setElementAt(5, 7, 2.0 * pB * pD);
            m2.setElementAt(5, 8, 2.0 * pC * pD);
            m2.setElementAt(5, 9, pD * pD);
            pA = plane7.getA();
            pB = plane7.getB();
            pC = plane7.getC();
            pD = plane7.getD();
            m2.setElementAt(6, 0, pA * pA);
            m2.setElementAt(6, 1, pB * pB);
            m2.setElementAt(6, 2, pC * pC);
            m2.setElementAt(6, 3, 2.0 * pA * pB);
            m2.setElementAt(6, 4, 2.0 * pA * pC);
            m2.setElementAt(6, 5, 2.0 * pB * pC);
            m2.setElementAt(6, 6, 2.0 * pA * pD);
            m2.setElementAt(6, 7, 2.0 * pB * pD);
            m2.setElementAt(6, 8, 2.0 * pC * pD);
            m2.setElementAt(6, 9, pD * pD);
            pA = plane8.getA();
            pB = plane8.getB();
            pC = plane8.getC();
            pD = plane8.getD();
            m2.setElementAt(7, 0, pA * pA);
            m2.setElementAt(7, 1, pB * pB);
            m2.setElementAt(7, 2, pC * pC);
            m2.setElementAt(7, 3, 2.0 * pA * pB);
            m2.setElementAt(7, 4, 2.0 * pA * pC);
            m2.setElementAt(7, 5, 2.0 * pB * pC);
            m2.setElementAt(7, 6, 2.0 * pA * pD);
            m2.setElementAt(7, 7, 2.0 * pB * pD);
            m2.setElementAt(7, 8, 2.0 * pC * pD);
            m2.setElementAt(7, 9, pD * pD);
            pA = plane9.getA();
            pB = plane9.getB();
            pC = plane9.getC();
            pD = plane9.getD();
            m2.setElementAt(8, 0, pA * pA);
            m2.setElementAt(8, 1, pB * pB);
            m2.setElementAt(8, 2, pC * pC);
            m2.setElementAt(8, 3, 2.0 * pA * pB);
            m2.setElementAt(8, 4, 2.0 * pA * pC);
            m2.setElementAt(8, 5, 2.0 * pB * pC);
            m2.setElementAt(8, 6, 2.0 * pA * pD);
            m2.setElementAt(8, 7, 2.0 * pB * pD);
            m2.setElementAt(8, 8, 2.0 * pC * pD);
            m2.setElementAt(8, 9, pD * pD);
        }

        final DualQuadric dualQuadric = new DualQuadric(plane1, plane2, plane3, plane4, plane5,
                plane6, plane7, plane8, plane9);
        assertTrue(dualQuadric.isLocus(plane1, PRECISION_ERROR));
        assertTrue(dualQuadric.isLocus(plane2, PRECISION_ERROR));
        assertTrue(dualQuadric.isLocus(plane3, PRECISION_ERROR));
        assertTrue(dualQuadric.isLocus(plane4, PRECISION_ERROR));
        assertTrue(dualQuadric.isLocus(plane5, PRECISION_ERROR));
        assertTrue(dualQuadric.isLocus(plane6, PRECISION_ERROR));
        assertTrue(dualQuadric.isLocus(plane7, PRECISION_ERROR));
        assertTrue(dualQuadric.isLocus(plane8, PRECISION_ERROR));
        assertTrue(dualQuadric.isLocus(plane9, PRECISION_ERROR));

        // Force CoincidentPlanesException
        try {
            dualQuadric.setParametersFromPlanes(plane1, plane2, plane3, plane4,
                    plane5, plane6, plane7, plane8, plane8);
            fail("CoincidentPlanesException expected but not thrown");
        } catch (final CoincidentPlanesException ignore) {
        }
    }

    @Test
    public void testNormalize() throws WrongSizeException,
            IllegalArgumentException, NonSymmetricMatrixException {

        final Matrix t = Matrix.createWithUniformRandomValues(HOM_COORDS, HOM_COORDS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final Matrix transT = t.transposeAndReturnNew();

        // make symmetric matrix
        final Matrix dualQuadricMatrix = transT.multiplyAndReturnNew(t);

        final DualQuadric dualQuadric = new DualQuadric(dualQuadricMatrix);
        assertFalse(dualQuadric.isNormalized());

        // normalize dual quadric
        dualQuadric.normalize();
        assertTrue(dualQuadric.isNormalized());

        // return quadric as matrix
        final Matrix dualQuadricMatrix2 = dualQuadric.asMatrix();

        // compare that both matrices are equal up to scale, for that reason we
        // first normalize both matrices
        double norm = com.irurueta.algebra.Utils.normF(dualQuadricMatrix);
        dualQuadricMatrix.multiplyByScalar(1.0 / norm);

        norm = com.irurueta.algebra.Utils.normF(dualQuadricMatrix2);
        dualQuadricMatrix2.multiplyByScalar(1.0 / norm);

        // compute their difference
        final Matrix diffMatrix = dualQuadricMatrix.subtractAndReturnNew(
                dualQuadricMatrix2);

        // finally, ensure that the norm of the difference matrix is almost zero
        // up to machine precision
        norm = com.irurueta.algebra.Utils.normF(diffMatrix);
        assertEquals(norm, 0.0, PRECISION_ERROR);

        // check that when setting new values quadric becomes non-normalized
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double value = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        dualQuadric.setA(value);
        assertFalse(dualQuadric.isNormalized());

        dualQuadric.normalize();
        assertTrue(dualQuadric.isNormalized());
        dualQuadric.setB(value);
        assertFalse(dualQuadric.isNormalized());

        dualQuadric.normalize();
        assertTrue(dualQuadric.isNormalized());
        dualQuadric.setC(value);
        assertFalse(dualQuadric.isNormalized());

        dualQuadric.normalize();
        assertTrue(dualQuadric.isNormalized());
        dualQuadric.setD(value);
        assertFalse(dualQuadric.isNormalized());

        dualQuadric.normalize();
        assertTrue(dualQuadric.isNormalized());
        dualQuadric.setE(value);
        assertFalse(dualQuadric.isNormalized());

        dualQuadric.normalize();
        assertTrue(dualQuadric.isNormalized());
        dualQuadric.setF(value);
        assertFalse(dualQuadric.isNormalized());

        dualQuadric.normalize();
        assertTrue(dualQuadric.isNormalized());
        dualQuadric.setG(value);
        assertFalse(dualQuadric.isNormalized());

        dualQuadric.normalize();
        assertTrue(dualQuadric.isNormalized());
        dualQuadric.setH(value);
        assertFalse(dualQuadric.isNormalized());

        dualQuadric.normalize();
        assertTrue(dualQuadric.isNormalized());
        dualQuadric.setI(value);
        assertFalse(dualQuadric.isNormalized());

        dualQuadric.normalize();
        assertTrue(dualQuadric.isNormalized());
        dualQuadric.setJ(value);
        assertFalse(dualQuadric.isNormalized());

        dualQuadric.normalize();
        assertTrue(dualQuadric.isNormalized());
        dualQuadric.setParameters(value, value, value, value, value, value,
                value, value, value, value);
        assertFalse(dualQuadric.isNormalized());

        dualQuadric.normalize();
        assertTrue(dualQuadric.isNormalized());
        dualQuadric.setParameters(dualQuadricMatrix);
        assertFalse(dualQuadric.isNormalized());

        // when setting all values to zero, attempting to normalize has no effect
        dualQuadric.setParameters(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0);
        assertFalse(dualQuadric.isNormalized());
        dualQuadric.normalize();
        assertFalse(dualQuadric.isNormalized());
    }

    @Test
    public void testCreateCanonicalDualAbsoluteQuadric() throws WrongSizeException {
        final DualQuadric daq = DualQuadric.createCanonicalDualAbsoluteQuadric();

        assertEquals(daq.getA(), 1.0, 0.0);
        assertEquals(daq.getB(), 1.0, 0.0);
        assertEquals(daq.getC(), 1.0, 0.0);
        assertEquals(daq.getD(), 0.0, 0.0);
        assertEquals(daq.getE(), 0.0, 0.0);
        assertEquals(daq.getF(), 0.0, 0.0);
        assertEquals(daq.getG(), 0.0, 0.0);
        assertEquals(daq.getH(), 0.0, 0.0);
        assertEquals(daq.getI(), 0.0, 0.0);
        assertEquals(daq.getJ(), 0.0, 0.0);

        final Matrix m = Matrix.identity(4, 4);
        m.setElementAt(3, 3, 0.0);
        assertEquals(daq.asMatrix(), m);
    }

    @Test
    public void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final UniformRandomizer randomizer = new UniformRandomizer();

        double a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double e = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double f = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double g = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double h = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double i = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double j = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final DualQuadric dualQuadric1 = new DualQuadric(a, b, c, d, e, f, g, h, i, j);

        // check
        assertEquals(a, dualQuadric1.getA(), 0.0);
        assertEquals(b, dualQuadric1.getB(), 0.0);
        assertEquals(c, dualQuadric1.getC(), 0.0);
        assertEquals(d, dualQuadric1.getD(), 0.0);
        assertEquals(e, dualQuadric1.getE(), 0.0);
        assertEquals(f, dualQuadric1.getF(), 0.0);
        assertEquals(g, dualQuadric1.getG(), 0.0);
        assertEquals(h, dualQuadric1.getH(), 0.0);
        assertEquals(i, dualQuadric1.getI(), 0.0);
        assertEquals(j, dualQuadric1.getJ(), 0.0);
        assertFalse(dualQuadric1.isNormalized());

        // serialize and deserialize
        final byte[] bytes = SerializationHelper.serialize(dualQuadric1);
        final DualQuadric dualQuadric2 = SerializationHelper.deserialize(bytes);

        // check
        assertEquals(dualQuadric1.getA(), dualQuadric2.getA(), 0.0);
        assertEquals(dualQuadric1.getB(), dualQuadric2.getB(), 0.0);
        assertEquals(dualQuadric1.getC(), dualQuadric2.getC(), 0.0);
        assertEquals(dualQuadric1.getD(), dualQuadric2.getD(), 0.0);
        assertEquals(dualQuadric1.getE(), dualQuadric2.getE(), 0.0);
        assertEquals(dualQuadric1.getF(), dualQuadric2.getF(), 0.0);
        assertEquals(dualQuadric1.getG(), dualQuadric2.getG(), 0.0);
        assertEquals(dualQuadric1.getH(), dualQuadric2.getH(), 0.0);
        assertEquals(dualQuadric1.getI(), dualQuadric2.getI(), 0.0);
        assertEquals(dualQuadric1.getJ(), dualQuadric2.getJ(), 0.0);
        assertEquals(dualQuadric1.isNormalized(), dualQuadric2.isNormalized());
    }
}
