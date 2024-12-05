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

class DualQuadricTest {

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
    void testConstants() {
        assertEquals(4, BaseQuadric.BASEQUADRIC_MATRIX_ROW_SIZE);
        assertEquals(4, BaseQuadric.BASEQUADRIC_MATRIX_COLUMN_SIZE);
        assertEquals(10, BaseQuadric.N_PARAMS);
        assertEquals(1e-12, BaseQuadric.DEFAULT_LOCUS_THRESHOLD, 0.0);
        assertEquals(1e-12, BaseQuadric.DEFAULT_PERPENDICULAR_THRESHOLD, 0.0);
        assertEquals(0.0, BaseQuadric.MIN_THRESHOLD, 0.0);
    }

    @Test
    void testConstructor() throws WrongSizeException, IllegalArgumentException, NonSymmetricMatrixException,
            DecomposerException, CoincidentPlanesException {

        final var randomizer = new UniformRandomizer();

        // Constructor
        var dualQuadric = new DualQuadric();
        assertEquals(0.0, dualQuadric.getA(), 0.0);
        assertEquals(0.0, dualQuadric.getB(), 0.0);
        assertEquals(0.0, dualQuadric.getC(), 0.0);
        assertEquals(0.0, dualQuadric.getD(), 0.0);
        assertEquals(0.0, dualQuadric.getE(), 0.0);
        assertEquals(0.0, dualQuadric.getF(), 0.0);
        assertEquals(0.0, dualQuadric.getG(), 0.0);
        assertEquals(0.0, dualQuadric.getH(), 0.0);
        assertEquals(0.0, dualQuadric.getI(), 0.0);
        assertEquals(0.0, dualQuadric.getJ(), 0.0);
        assertFalse(dualQuadric.isNormalized());

        // Constructor with params
        var a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var e = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var f = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var g = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var h = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var i = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var j = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        dualQuadric = new DualQuadric(a, b, c, d, e, f, g, h, i, j);
        assertEquals(a, dualQuadric.getA(), 0.0);
        assertEquals(b, dualQuadric.getB(), 0.0);
        assertEquals(c, dualQuadric.getC(), 0.0);
        assertEquals(d, dualQuadric.getD(), 0.0);
        assertEquals(e, dualQuadric.getE(), 0.0);
        assertEquals(f, dualQuadric.getF(), 0.0);
        assertEquals(g, dualQuadric.getG(), 0.0);
        assertEquals(h, dualQuadric.getH(), 0.0);
        assertEquals(i, dualQuadric.getI(), 0.0);
        assertEquals(j, dualQuadric.getJ(), 0.0);
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

        assertEquals(m.getElementAt(0, 0), dualQuadric.getA(), 0.0);
        assertEquals(m.getElementAt(1, 1), dualQuadric.getB(), 0.0);
        assertEquals(m.getElementAt(2, 2), dualQuadric.getC(), 0.0);
        assertEquals(m.getElementAt(1, 0), dualQuadric.getD(), 0.0);
        assertEquals(m.getElementAt(2, 1), dualQuadric.getE(), 0.0);
        assertEquals(m.getElementAt(2, 0), dualQuadric.getF(), 0.0);
        assertEquals(m.getElementAt(3, 0), dualQuadric.getG(), 0.0);
        assertEquals(m.getElementAt(1, 3), dualQuadric.getH(), 0.0);
        assertEquals(m.getElementAt(3, 2), dualQuadric.getI(), 0.0);
        assertEquals(m.getElementAt(3, 3), dualQuadric.getJ(), 0.0);

        // Constructor using matrix with wrong size exception
        final var wrong1 = new Matrix(DUAL_QUADRIC_ROWS, DUAL_QUADRIC_COLS + 1);
        assertThrows(IllegalArgumentException.class, () -> new DualQuadric(wrong1));

        // Constructor using non-symmetric matrix
        final var wrong2 = new Matrix(DUAL_QUADRIC_ROWS, DUAL_QUADRIC_COLS);
        wrong2.setElementAt(0, 0, a);
        wrong2.setElementAt(1, 1, b);
        wrong2.setElementAt(2, 2, c);
        wrong2.setElementAt(3, 3, j);
        wrong2.setElementAt(1, 0, d);
        wrong2.setElementAt(0, 1, d + 1.0);
        wrong2.setElementAt(2, 1, e);
        wrong2.setElementAt(1, 2, e + 1.0);
        wrong2.setElementAt(2, 0, f);
        wrong2.setElementAt(0, 2, f + 1.0);
        wrong2.setElementAt(3, 0, g);
        wrong2.setElementAt(0, 3, g + 1.0);
        wrong2.setElementAt(3, 1, h);
        wrong2.setElementAt(1, 3, h + 1.0);
        wrong2.setElementAt(3, 2, i);
        wrong2.setElementAt(2, 3, i + 1.0);

        assertThrows(NonSymmetricMatrixException.class, () -> new DualQuadric(wrong2));

        // Constructor from 9 planes
        m = Matrix.createWithUniformRandomValues(9, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        var plane1 = new Plane(m.getElementAt(0, 0), m.getElementAt(0, 1),
                m.getElementAt(0, 2), m.getElementAt(0, 3));
        var plane2 = new Plane(m.getElementAt(1, 0), m.getElementAt(1, 1),
                m.getElementAt(1, 2), m.getElementAt(1, 3));
        var plane3 = new Plane(m.getElementAt(2, 0), m.getElementAt(2, 1),
                m.getElementAt(2, 2), m.getElementAt(2, 3));
        var plane4 = new Plane(m.getElementAt(3, 0), m.getElementAt(3, 1),
                m.getElementAt(3, 2), m.getElementAt(3, 3));
        var plane5 = new Plane(m.getElementAt(4, 0), m.getElementAt(4, 1),
                m.getElementAt(4, 2), m.getElementAt(4, 3));
        var plane6 = new Plane(m.getElementAt(5, 0), m.getElementAt(5, 1),
                m.getElementAt(5, 2), m.getElementAt(5, 3));
        var plane7 = new Plane(m.getElementAt(6, 0), m.getElementAt(6, 1),
                m.getElementAt(6, 2), m.getElementAt(6, 3));
        var plane8 = new Plane(m.getElementAt(7, 0), m.getElementAt(7, 1),
                m.getElementAt(7, 2), m.getElementAt(7, 3));
        var plane9 = new Plane(m.getElementAt(8, 0), m.getElementAt(8, 1),
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

        var m2 = new Matrix(9, 10);
        var pA = plane1.getA();
        var pB = plane1.getB();
        var pC = plane1.getC();
        var pD = plane1.getD();
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
            m = Matrix.createWithUniformRandomValues(9, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

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

        dualQuadric = new DualQuadric(plane1, plane2, plane3, plane4, plane5, plane6, plane7, plane8, plane9);
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
        final var finalPlane1 = plane1;
        final var finalPlane2 = plane2;
        final var finalPlane3 = plane3;
        final var finalPlane4 = plane4;
        final var finalPlane5 = plane5;
        final var finalPlane6 = plane6;
        final var finalPlane7 = plane7;
        final var finalPlane8 = plane8;
        assertThrows(CoincidentPlanesException.class, () -> new DualQuadric(finalPlane1, finalPlane2, finalPlane3,
                finalPlane4, finalPlane5, finalPlane6, finalPlane7, finalPlane8, finalPlane8));
    }

    @Test
    void testGettersAndSetters() throws WrongSizeException, IllegalArgumentException, NonSymmetricMatrixException {

        final var randomizer = new UniformRandomizer();

        var dualQuadric = new DualQuadric();
        var a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var e = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var f = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var g = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var h = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var i = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var j = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
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
        assertEquals(a, dualQuadric.getA(), 0.0);
        assertEquals(b, dualQuadric.getB(), 0.0);
        assertEquals(c, dualQuadric.getC(), 0.0);
        assertEquals(d, dualQuadric.getD(), 0.0);
        assertEquals(e, dualQuadric.getE(), 0.0);
        assertEquals(f, dualQuadric.getF(), 0.0);
        assertEquals(g, dualQuadric.getG(), 0.0);
        assertEquals(h, dualQuadric.getH(), 0.0);
        assertEquals(i, dualQuadric.getI(), 0.0);
        assertEquals(j, dualQuadric.getJ(), 0.0);

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
        assertEquals(a, dualQuadric.getA(), 0.0);
        assertEquals(b, dualQuadric.getB(), 0.0);
        assertEquals(c, dualQuadric.getC(), 0.0);
        assertEquals(d, dualQuadric.getD(), 0.0);
        assertEquals(e, dualQuadric.getE(), 0.0);
        assertEquals(f, dualQuadric.getF(), 0.0);
        assertEquals(g, dualQuadric.getG(), 0.0);
        assertEquals(h, dualQuadric.getH(), 0.0);
        assertEquals(i, dualQuadric.getI(), 0.0);
        assertEquals(j, dualQuadric.getJ(), 0.0);

        dualQuadric = new DualQuadric();
        final var m = new Matrix(DUAL_QUADRIC_ROWS, DUAL_QUADRIC_COLS);
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
        assertEquals(m.getElementAt(0, 0), dualQuadric.getA(), 0.0);
        assertEquals(m.getElementAt(1, 1), dualQuadric.getB(), 0.0);
        assertEquals(m.getElementAt(2, 2), dualQuadric.getC(), 0.0);
        assertEquals(m.getElementAt(1, 0), dualQuadric.getD(), 0.0);
        assertEquals(m.getElementAt(2, 1), dualQuadric.getE(), 0.0);
        assertEquals(m.getElementAt(2, 0), dualQuadric.getF(), 0.0);
        assertEquals(m.getElementAt(3, 0), dualQuadric.getG(), 0.0);
        assertEquals(m.getElementAt(3, 1), dualQuadric.getH(), 0.0);
        assertEquals(m.getElementAt(3, 2), dualQuadric.getI(), 0.0);
    }

    @Test
    void testAsMatrix() throws WrongSizeException, IllegalArgumentException, NonSymmetricMatrixException {

        final var randomizer = new UniformRandomizer();

        final var dualQuadric = new DualQuadric();
        final var m = new Matrix(DUAL_QUADRIC_ROWS, DUAL_QUADRIC_COLS);
        final var a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var e = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var f = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var g = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var h = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var i = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var j = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
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
        final var m2 = dualQuadric.asMatrix();

        assertTrue(m.equals(m2, PRECISION_ERROR));
    }

    @Test
    void testIsLocus() throws WrongSizeException, DecomposerException, NotReadyException, LockedException,
            com.irurueta.algebra.NotAvailableException, RankDeficientMatrixException, IllegalArgumentException,
            NonSymmetricMatrixException {

        var m = Matrix.createWithUniformRandomValues(9, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        var plane1 = new Plane(m.getElementAt(0, 0), m.getElementAt(0, 1),
                m.getElementAt(0, 2), m.getElementAt(0, 3));
        var plane2 = new Plane(m.getElementAt(1, 0), m.getElementAt(1, 1),
                m.getElementAt(1, 2), m.getElementAt(1, 3));
        var plane3 = new Plane(m.getElementAt(2, 0), m.getElementAt(2, 1),
                m.getElementAt(2, 2), m.getElementAt(2, 3));
        var plane4 = new Plane(m.getElementAt(3, 0), m.getElementAt(3, 1),
                m.getElementAt(3, 2), m.getElementAt(3, 3));
        var plane5 = new Plane(m.getElementAt(4, 0), m.getElementAt(4, 1),
                m.getElementAt(4, 2), m.getElementAt(4, 3));
        var plane6 = new Plane(m.getElementAt(5, 0), m.getElementAt(5, 1),
                m.getElementAt(5, 2), m.getElementAt(5, 3));
        var plane7 = new Plane(m.getElementAt(6, 0), m.getElementAt(6, 1),
                m.getElementAt(6, 2), m.getElementAt(6, 3));
        var plane8 = new Plane(m.getElementAt(7, 0), m.getElementAt(7, 1),
                m.getElementAt(7, 2), m.getElementAt(7, 3));
        var plane9 = new Plane(m.getElementAt(8, 0), m.getElementAt(8, 1),
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

        var m2 = new Matrix(9, 10);
        var pA = plane1.getA();
        var pB = plane1.getB();
        var pC = plane1.getC();
        var pD = plane1.getD();
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
            m = Matrix.createWithUniformRandomValues(9, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

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

        final var decomposer = new SingularValueDecomposer(m2);
        decomposer.decompose();

        final var v = decomposer.getV();

        final var a = v.getElementAt(0, 9);
        final var b = v.getElementAt(1, 9);
        final var c = v.getElementAt(2, 9);
        final var d = v.getElementAt(3, 9);

        final var f = v.getElementAt(4, 9);
        final var e = v.getElementAt(5, 9);

        final var g = v.getElementAt(6, 9);
        final var h = v.getElementAt(7, 9);
        final var i = v.getElementAt(8, 9);
        final var j = v.getElementAt(9, 9);

        final var dualQuadricPlane = new Matrix(HOM_COORDS, 1);
        dualQuadricPlane.setElementAt(0, 0, plane1.getA());
        dualQuadricPlane.setElementAt(1, 0, plane1.getB());
        dualQuadricPlane.setElementAt(2, 0, plane1.getC());

        var norm = com.irurueta.algebra.Utils.normF(dualQuadricPlane);
        dualQuadricPlane.multiplyByScalar(1.0 / norm);

        final var dualQuadricMatrix = new Matrix(DUAL_QUADRIC_ROWS, DUAL_QUADRIC_COLS);
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
        final var homPointMatrix = dualQuadricMatrix.multiplyAndReturnNew(dualQuadricPlane);

        // add director vector of tangent plane to get a point outside of quadric
        var directVectorA = dualQuadricPlane.getElementAtIndex(0);
        var directVectorB = dualQuadricPlane.getElementAtIndex(1);
        var directVectorC = dualQuadricPlane.getElementAtIndex(2);
        final var directVectorNorm = Math.sqrt(directVectorA * directVectorA + directVectorB * directVectorB
                + directVectorC * directVectorC);
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
        var quadricMatrix = com.irurueta.algebra.Utils.inverse(dualQuadricMatrix);

        // find plane vector outside dual quadric as the product of quadric
        // matrix and point outside of quadric
        final var outsidePlaneMatrix = quadricMatrix.multiplyAndReturnNew(homPointMatrix);

        // instantiate plane outside dual quadric using computed vector
        final var outsidePlane = new Plane(outsidePlaneMatrix.toArray());

        // instantiate new dual quadric instance
        final var dualQuadric = new DualQuadric(dualQuadricMatrix);

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
    void testAngleBetweenPlanes() throws WrongSizeException, IllegalArgumentException, NonSymmetricMatrixException {

        // initial planes
        final var planeMatrix1 = Matrix.createWithUniformRandomValues(HOM_COORDS,
                1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var planeMatrix2 = Matrix.createWithUniformRandomValues(HOM_COORDS,
                1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // transformation matrix
        final var transform = Matrix.createWithUniformRandomValues(HOM_COORDS, HOM_COORDS, MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        // transform planes
        final var tPlane1 = transform.multiplyAndReturnNew(planeMatrix1);
        final var tPlane2 = transform.multiplyAndReturnNew(planeMatrix2);

        final var norm1 = com.irurueta.algebra.Utils.normF(tPlane1);
        final var norm2 = com.irurueta.algebra.Utils.normF(tPlane2);

        final var numerator = tPlane1.transposeAndReturnNew().multiplyAndReturnNew(tPlane2)
                .getElementAt(0, 0);

        final var cosAngle = numerator / (norm1 * norm2);

        final var angle = Math.acos(cosAngle);

        // compute dual quadric matrix as the product of transposed transform
        // matrix with itself
        final var transposedTransform = transform.transposeAndReturnNew();
        final var dualQuadricMatrix = transposedTransform.multiplyAndReturnNew(transform);

        // normalize conic matrix
        final var normDualQuadric = com.irurueta.algebra.Utils.normF(dualQuadricMatrix);
        dualQuadricMatrix.multiplyByScalar(1.0 / normDualQuadric);

        final var dualQuadric = new DualQuadric(dualQuadricMatrix);

        final var plane1 = new Plane(planeMatrix1.getElementAt(0, 0),
                planeMatrix1.getElementAt(1, 0),
                planeMatrix1.getElementAt(2, 0),
                planeMatrix1.getElementAt(3, 0));
        final var plane2 = new Plane(planeMatrix2.getElementAt(0, 0),
                planeMatrix2.getElementAt(1, 0),
                planeMatrix2.getElementAt(2, 0),
                planeMatrix2.getElementAt(3, 0));

        assertEquals(dualQuadric.angleBetweenPlanes(plane1, plane2), angle, PRECISION_ERROR);
    }

    @Test
    void testArePerpendicularPlanes() throws WrongSizeException, DecomposerException, RankDeficientMatrixException,
            IllegalArgumentException, NonSymmetricMatrixException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            // trying perpendicular angle
            var matrixPlane1 = Matrix.createWithUniformRandomValues(HOM_COORDS, 1, MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);

            var norm = com.irurueta.algebra.Utils.normF(matrixPlane1);
            matrixPlane1.multiplyByScalar(1.0 / norm);

            var matrixPlane2 = new Matrix(HOM_COORDS, 1);
            matrixPlane2.setElementAt(0, 0, matrixPlane1.getElementAt(1, 0)
                    + matrixPlane1.getElementAt(2, 0));
            matrixPlane2.setElementAt(1, 0, -matrixPlane1.getElementAt(0, 0));
            matrixPlane2.setElementAt(2, 0, -matrixPlane1.getElementAt(0, 0));

            norm = com.irurueta.algebra.Utils.normF(matrixPlane2);
            matrixPlane2.multiplyByScalar(1.0 / norm);

            var transform = Matrix.createWithUniformRandomValues(HOM_COORDS, HOM_COORDS, MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            while (com.irurueta.algebra.Utils.rank(transform) < 3) {
                transform = Matrix.createWithUniformRandomValues(HOM_COORDS, HOM_COORDS, MIN_RANDOM_VALUE,
                        MAX_RANDOM_VALUE);
            }

            final var invTransform = com.irurueta.algebra.Utils.inverse(transform);

            var transformPlaneMatrix1 = transform.multiplyAndReturnNew(matrixPlane1);
            var transformPlaneMatrix2 = transform.multiplyAndReturnNew(matrixPlane2);

            final var transInvTransform = invTransform.transposeAndReturnNew();

            final var dualQuadricMatrix = transInvTransform.multiplyAndReturnNew(invTransform);
            norm = com.irurueta.algebra.Utils.normF(dualQuadricMatrix);
            dualQuadricMatrix.multiplyByScalar(1.0 / norm);

            var transformPlane1 = new Plane(transformPlaneMatrix1.toArray());
            var transformPlane2 = new Plane(transformPlaneMatrix2.toArray());

            var dualQuadric = new DualQuadric(dualQuadricMatrix);

            assertTrue(dualQuadric.arePerpendicularPlanes(transformPlane1, transformPlane2, PERPENDICULAR_THRESHOLD));

            // trying non-perpendicular points
            double dotProduct;

            matrixPlane1 = Matrix.createWithUniformRandomValues(HOM_COORDS, 1, MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            norm = com.irurueta.algebra.Utils.normF(matrixPlane1);
            matrixPlane1.multiplyByScalar(1.0 / norm);

            matrixPlane2 = Matrix.createWithUniformRandomValues(HOM_COORDS, 1, MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            norm = com.irurueta.algebra.Utils.normF(matrixPlane2);
            matrixPlane2.multiplyByScalar(1.0 / norm);

            dotProduct = matrixPlane1.transposeAndReturnNew().multiplyAndReturnNew(
                    matrixPlane2).getElementAt(0, 0);

            // ensure lines are not perpendicular
            while (Math.abs(dotProduct) < PERPENDICULAR_THRESHOLD) {
                matrixPlane1 = Matrix.createWithUniformRandomValues(HOM_COORDS, 1, MIN_RANDOM_VALUE,
                        MAX_RANDOM_VALUE);
                norm = com.irurueta.algebra.Utils.normF(matrixPlane1);
                matrixPlane1.multiplyByScalar(1.0 / norm);

                matrixPlane2 = Matrix.createWithUniformRandomValues(HOM_COORDS, 1, MIN_RANDOM_VALUE,
                        MAX_RANDOM_VALUE);
                norm = com.irurueta.algebra.Utils.normF(matrixPlane2);
                matrixPlane2.multiplyByScalar(1.0 / norm);

                dotProduct = matrixPlane1.transposeAndReturnNew().multiplyAndReturnNew(matrixPlane2)
                        .getElementAt(0, 0);
            }

            transformPlaneMatrix1 = transform.multiplyAndReturnNew(matrixPlane1);
            transformPlaneMatrix2 = transform.multiplyAndReturnNew(matrixPlane2);

            transformPlane1 = new Plane(transformPlaneMatrix1.toArray());
            transformPlane2 = new Plane(transformPlaneMatrix2.toArray());

            if (dualQuadric.arePerpendicularPlanes(transformPlane1, transformPlane2, PERPENDICULAR_THRESHOLD)) {
                continue;
            }
            assertFalse(dualQuadric.arePerpendicularPlanes(transformPlane1, transformPlane2, PERPENDICULAR_THRESHOLD));

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
    void testGetQuadric() throws WrongSizeException, DecomposerException, RankDeficientMatrixException,
            IllegalArgumentException, NonSymmetricMatrixException, QuadricNotAvailableException {

        var transformMatrix = Matrix.createWithUniformRandomValues(HOM_COORDS, HOM_COORDS, MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        while (com.irurueta.algebra.Utils.rank(transformMatrix) != 4) {
            transformMatrix = Matrix.createWithUniformRandomValues(HOM_COORDS, HOM_COORDS, MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
        }

        final var transformTransposedMatrix = transformMatrix.transposeAndReturnNew();

        final var dualQuadricMatrix = transformTransposedMatrix.multiplyAndReturnNew(transformMatrix);

        final var quadricMatrix1 = com.irurueta.algebra.Utils.inverse(dualQuadricMatrix);

        final var dualQuadric = new DualQuadric(dualQuadricMatrix);

        final var quadric = dualQuadric.getQuadric();
        final var quadric2 = new Quadric();
        dualQuadric.quadric(quadric2);

        assertEquals(quadric.asMatrix(), quadric2.asMatrix());

        final var quadricMatrix2 = quadric.asMatrix();

        // normalize quadric matrices
        var norm = com.irurueta.algebra.Utils.normF(quadricMatrix1);
        quadricMatrix1.multiplyByScalar(1.0 / norm);

        norm = com.irurueta.algebra.Utils.normF(quadricMatrix2);
        quadricMatrix2.multiplyByScalar(1.0 / norm);

        // compute difference of normalized quadric matrices
        final var diffMatrix = quadricMatrix1.subtractAndReturnNew(quadricMatrix2);

        // ensure that difference matrix is almost zero by checking its norm
        norm = com.irurueta.algebra.Utils.normF(diffMatrix);
        assertEquals(0.0, norm, PRECISION_ERROR);
    }

    @Test
    void testSetParametersFromPlanes() throws WrongSizeException, DecomposerException, CoincidentPlanesException {
        var m = Matrix.createWithUniformRandomValues(9, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        var plane1 = new Plane(m.getElementAt(0, 0), m.getElementAt(0, 1),
                m.getElementAt(0, 2), m.getElementAt(0, 3));
        var plane2 = new Plane(m.getElementAt(1, 0), m.getElementAt(1, 1),
                m.getElementAt(1, 2), m.getElementAt(1, 3));
        var plane3 = new Plane(m.getElementAt(2, 0), m.getElementAt(2, 1),
                m.getElementAt(2, 2), m.getElementAt(2, 3));
        var plane4 = new Plane(m.getElementAt(3, 0), m.getElementAt(3, 1),
                m.getElementAt(3, 2), m.getElementAt(3, 3));
        var plane5 = new Plane(m.getElementAt(4, 0), m.getElementAt(4, 1),
                m.getElementAt(4, 2), m.getElementAt(4, 3));
        var plane6 = new Plane(m.getElementAt(5, 0), m.getElementAt(5, 1),
                m.getElementAt(5, 2), m.getElementAt(5, 3));
        var plane7 = new Plane(m.getElementAt(6, 0), m.getElementAt(6, 1),
                m.getElementAt(6, 2), m.getElementAt(6, 3));
        var plane8 = new Plane(m.getElementAt(7, 0), m.getElementAt(7, 1),
                m.getElementAt(7, 2), m.getElementAt(7, 3));
        var plane9 = new Plane(m.getElementAt(8, 0), m.getElementAt(8, 1),
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
        var m2 = new Matrix(9, 10);

        var pA = plane1.getA();
        var pB = plane1.getB();
        var pC = plane1.getC();
        var pD = plane1.getD();
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
            m = Matrix.createWithUniformRandomValues(9, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

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

        final var dualQuadric = new DualQuadric(plane1, plane2, plane3, plane4, plane5, plane6, plane7, plane8, plane9);
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
        final var finalPlane1 = plane1;
        final var finalPlane2 = plane2;
        final var finalPlane3 = plane3;
        final var finalPlane4 = plane4;
        final var finalPlane5 = plane5;
        final var finalPlane6 = plane6;
        final var finalPlane7 = plane7;
        final var finalPlane8 = plane8;
        assertThrows(CoincidentPlanesException.class,
                () -> dualQuadric.setParametersFromPlanes(finalPlane1, finalPlane2, finalPlane3, finalPlane4,
                        finalPlane5, finalPlane6, finalPlane7, finalPlane8, finalPlane8));
    }

    @Test
    void testNormalize() throws WrongSizeException, IllegalArgumentException, NonSymmetricMatrixException {

        final var t = Matrix.createWithUniformRandomValues(HOM_COORDS, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var transT = t.transposeAndReturnNew();

        // make symmetric matrix
        final var dualQuadricMatrix = transT.multiplyAndReturnNew(t);

        final var dualQuadric = new DualQuadric(dualQuadricMatrix);
        assertFalse(dualQuadric.isNormalized());

        // normalize dual quadric
        dualQuadric.normalize();
        assertTrue(dualQuadric.isNormalized());

        // return quadric as matrix
        final var dualQuadricMatrix2 = dualQuadric.asMatrix();

        // compare that both matrices are equal up to scale, for that reason we
        // first normalize both matrices
        var norm = com.irurueta.algebra.Utils.normF(dualQuadricMatrix);
        dualQuadricMatrix.multiplyByScalar(1.0 / norm);

        norm = com.irurueta.algebra.Utils.normF(dualQuadricMatrix2);
        dualQuadricMatrix2.multiplyByScalar(1.0 / norm);

        // compute their difference
        final var diffMatrix = dualQuadricMatrix.subtractAndReturnNew(dualQuadricMatrix2);

        // finally, ensure that the norm of the difference matrix is almost zero
        // up to machine precision
        norm = com.irurueta.algebra.Utils.normF(diffMatrix);
        assertEquals(0.0, norm, PRECISION_ERROR);

        // check that when setting new values quadric becomes non-normalized
        final var randomizer = new UniformRandomizer();
        final var value = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
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
        dualQuadric.setParameters(value, value, value, value, value, value, value, value, value, value);
        assertFalse(dualQuadric.isNormalized());

        dualQuadric.normalize();
        assertTrue(dualQuadric.isNormalized());
        dualQuadric.setParameters(dualQuadricMatrix);
        assertFalse(dualQuadric.isNormalized());

        // when setting all values to zero, attempting to normalize has no effect
        dualQuadric.setParameters(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        assertFalse(dualQuadric.isNormalized());
        dualQuadric.normalize();
        assertFalse(dualQuadric.isNormalized());
    }

    @Test
    void testCreateCanonicalDualAbsoluteQuadric() throws WrongSizeException {
        final var daq = DualQuadric.createCanonicalDualAbsoluteQuadric();

        assertEquals(1.0, daq.getA(), 0.0);
        assertEquals(1.0, daq.getB(), 0.0);
        assertEquals(1.0, daq.getC(), 0.0);
        assertEquals(0.0, daq.getD(), 0.0);
        assertEquals(0.0, daq.getE(), 0.0);
        assertEquals(0.0, daq.getF(), 0.0);
        assertEquals(0.0, daq.getG(), 0.0);
        assertEquals(0.0, daq.getH(), 0.0);
        assertEquals(0.0, daq.getI(), 0.0);
        assertEquals(0.0, daq.getJ(), 0.0);

        final var m = Matrix.identity(4, 4);
        m.setElementAt(3, 3, 0.0);
        assertEquals(daq.asMatrix(), m);
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
        var g = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var h = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var i = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var j = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var dualQuadric1 = new DualQuadric(a, b, c, d, e, f, g, h, i, j);

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
        final var bytes = SerializationHelper.serialize(dualQuadric1);
        final var dualQuadric2 = SerializationHelper.<DualQuadric>deserialize(bytes);

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
