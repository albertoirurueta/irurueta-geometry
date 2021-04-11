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

import com.irurueta.algebra.Utils;
import com.irurueta.algebra.*;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class QuadricTest {

    private static final double MIN_RANDOM_VALUE = -10.0;
    private static final double MAX_RANDOM_VALUE = 10.0;
    private static final double PRECISION_ERROR = 1e-8;
    private static final double LOCUS_THRESHOLD = 1e-8;
    private static final double PERPENDICULAR_THRESHOLD = 1e-6;
    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final int QUADRIC_ROWS = 4;
    private static final int QUADRIC_COLS = 4;
    private static final int HOM_COORDS = 4;

    private static final double MIN_RANDOM_DEGREES = -180.0;
    private static final double MAX_RANDOM_DEGREES = 180.0;

    private static final int TIMES = 100;

    @Test
    public void testConstructor() throws WrongSizeException,
            IllegalArgumentException, NonSymmetricMatrixException,
            DecomposerException, CoincidentPointsException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // Constructor
        Quadric quadric = new Quadric();
        assertEquals(quadric.getA(), 0.0, 0.0);
        assertEquals(quadric.getB(), 0.0, 0.0);
        assertEquals(quadric.getC(), 0.0, 0.0);
        assertEquals(quadric.getD(), 0.0, 0.0);
        assertEquals(quadric.getE(), 0.0, 0.0);
        assertEquals(quadric.getF(), 0.0, 0.0);
        assertEquals(quadric.getG(), 0.0, 0.0);
        assertEquals(quadric.getH(), 0.0, 0.0);
        assertEquals(quadric.getI(), 0.0, 0.0);
        assertEquals(quadric.getJ(), 0.0, 0.0);

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

        quadric = new Quadric(a, b, c, d, e, f, g, h, i, j);
        assertEquals(quadric.getA(), a, 0.0);
        assertEquals(quadric.getB(), b, 0.0);
        assertEquals(quadric.getC(), c, 0.0);
        assertEquals(quadric.getD(), d, 0.0);
        assertEquals(quadric.getE(), e, 0.0);
        assertEquals(quadric.getF(), f, 0.0);
        assertEquals(quadric.getG(), g, 0.0);
        assertEquals(quadric.getH(), h, 0.0);
        assertEquals(quadric.getI(), i, 0.0);
        assertEquals(quadric.getJ(), j, 0.0);
        assertFalse(quadric.isNormalized());

        // Constructor using matrix
        Matrix m = new Matrix(QUADRIC_ROWS, QUADRIC_COLS);
        // get random values
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

        quadric = new Quadric(m);

        assertEquals(quadric.getA(), m.getElementAt(0, 0), 0.0);
        assertEquals(quadric.getB(), m.getElementAt(1, 1), 0.0);
        assertEquals(quadric.getC(), m.getElementAt(2, 2), 0.0);
        assertEquals(quadric.getD(), m.getElementAt(1, 0), 0.0);
        assertEquals(quadric.getE(), m.getElementAt(2, 1), 0.0);
        assertEquals(quadric.getF(), m.getElementAt(2, 0), 0.0);
        assertEquals(quadric.getG(), m.getElementAt(3, 0), 0.0);
        assertEquals(quadric.getH(), m.getElementAt(1, 3), 0.0);
        assertEquals(quadric.getI(), m.getElementAt(3, 2), 0.0);
        assertEquals(quadric.getJ(), m.getElementAt(3, 3), 0.0);

        // Constructor using matrix with wrong size exception
        m = new Matrix(QUADRIC_ROWS, QUADRIC_COLS + 1);
        quadric = null;
        try {
            quadric = new Quadric(m);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(quadric);

        // Constructor using non-symmetric matrix
        m = new Matrix(QUADRIC_ROWS, QUADRIC_COLS);
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

        quadric = null;
        try {
            quadric = new Quadric(m);
            fail("NonSymmetricMatrixException expected but not thrown");
        } catch (final NonSymmetricMatrixException ignore) {
        }
        assertNull(quadric);

        // Constructor from 9 points
        m = Matrix.createWithUniformRandomValues(9, HOM_COORDS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        HomogeneousPoint3D point1 = new HomogeneousPoint3D(m.getElementAt(0, 0),
                m.getElementAt(0, 1), m.getElementAt(0, 2), 1.0);
        HomogeneousPoint3D point2 = new HomogeneousPoint3D(m.getElementAt(1, 0),
                m.getElementAt(1, 1), m.getElementAt(1, 2), 1.0);
        HomogeneousPoint3D point3 = new HomogeneousPoint3D(m.getElementAt(2, 0),
                m.getElementAt(2, 1), m.getElementAt(2, 2), 1.0);
        HomogeneousPoint3D point4 = new HomogeneousPoint3D(m.getElementAt(3, 0),
                m.getElementAt(3, 1), m.getElementAt(3, 2), 1.0);
        HomogeneousPoint3D point5 = new HomogeneousPoint3D(m.getElementAt(4, 0),
                m.getElementAt(4, 1), m.getElementAt(4, 2), 1.0);
        HomogeneousPoint3D point6 = new HomogeneousPoint3D(m.getElementAt(5, 0),
                m.getElementAt(5, 1), m.getElementAt(5, 2), 1.0);
        HomogeneousPoint3D point7 = new HomogeneousPoint3D(m.getElementAt(6, 0),
                m.getElementAt(6, 1), m.getElementAt(6, 2), 1.0);
        HomogeneousPoint3D point8 = new HomogeneousPoint3D(m.getElementAt(7, 0),
                m.getElementAt(7, 1), m.getElementAt(7, 2), 1.0);
        HomogeneousPoint3D point9 = new HomogeneousPoint3D(m.getElementAt(8, 0),
                m.getElementAt(8, 1), m.getElementAt(8, 2), 1.0);

        // estimate quadric that lies inside of provided 9 points
        Matrix quadricMatrix = new Matrix(9, 10);
        double x = point1.getHomX();
        double y = point1.getHomY();
        double z = point1.getHomZ();
        double w = point1.getHomW();
        quadricMatrix.setElementAt(0, 0, x * x);
        quadricMatrix.setElementAt(0, 1, y * y);
        quadricMatrix.setElementAt(0, 2, z * z);
        quadricMatrix.setElementAt(0, 3, 2.0 * x * y);
        quadricMatrix.setElementAt(0, 4, 2.0 * x * z);
        quadricMatrix.setElementAt(0, 5, 2.0 * y * z);
        quadricMatrix.setElementAt(0, 6, 2.0 * x * w);
        quadricMatrix.setElementAt(0, 7, 2.0 * y * w);
        quadricMatrix.setElementAt(0, 8, 2.0 * z * w);
        quadricMatrix.setElementAt(0, 9, w * w);
        x = point2.getHomX();
        y = point2.getHomY();
        z = point2.getHomZ();
        w = point2.getHomW();
        quadricMatrix.setElementAt(1, 0, x * x);
        quadricMatrix.setElementAt(1, 1, y * y);
        quadricMatrix.setElementAt(1, 2, z * z);
        quadricMatrix.setElementAt(1, 3, 2.0 * x * y);
        quadricMatrix.setElementAt(1, 4, 2.0 * x * z);
        quadricMatrix.setElementAt(1, 5, 2.0 * y * z);
        quadricMatrix.setElementAt(1, 6, 2.0 * x * w);
        quadricMatrix.setElementAt(1, 7, 2.0 * y * w);
        quadricMatrix.setElementAt(1, 8, 2.0 * z * w);
        quadricMatrix.setElementAt(1, 9, w * w);
        x = point3.getHomX();
        y = point3.getHomY();
        z = point3.getHomZ();
        w = point3.getHomW();
        quadricMatrix.setElementAt(2, 0, x * x);
        quadricMatrix.setElementAt(2, 1, y * y);
        quadricMatrix.setElementAt(2, 2, z * z);
        quadricMatrix.setElementAt(2, 3, 2.0 * x * y);
        quadricMatrix.setElementAt(2, 4, 2.0 * x * z);
        quadricMatrix.setElementAt(2, 5, 2.0 * y * z);
        quadricMatrix.setElementAt(2, 6, 2.0 * x * w);
        quadricMatrix.setElementAt(2, 7, 2.0 * y * w);
        quadricMatrix.setElementAt(2, 8, 2.0 * z * w);
        quadricMatrix.setElementAt(2, 9, w * w);
        x = point4.getHomX();
        y = point4.getHomY();
        z = point4.getHomZ();
        w = point4.getHomW();
        quadricMatrix.setElementAt(3, 0, x * x);
        quadricMatrix.setElementAt(3, 1, y * y);
        quadricMatrix.setElementAt(3, 2, z * z);
        quadricMatrix.setElementAt(3, 3, 2.0 * x * y);
        quadricMatrix.setElementAt(3, 4, 2.0 * x * z);
        quadricMatrix.setElementAt(3, 5, 2.0 * y * z);
        quadricMatrix.setElementAt(3, 6, 2.0 * x * w);
        quadricMatrix.setElementAt(3, 7, 2.0 * y * w);
        quadricMatrix.setElementAt(3, 8, 2.0 * z * w);
        quadricMatrix.setElementAt(3, 9, w * w);
        x = point5.getHomX();
        y = point5.getHomY();
        z = point5.getHomZ();
        w = point5.getHomW();
        quadricMatrix.setElementAt(4, 0, x * x);
        quadricMatrix.setElementAt(4, 1, y * y);
        quadricMatrix.setElementAt(4, 2, z * z);
        quadricMatrix.setElementAt(4, 3, 2.0 * x * y);
        quadricMatrix.setElementAt(4, 4, 2.0 * x * z);
        quadricMatrix.setElementAt(4, 5, 2.0 * y * z);
        quadricMatrix.setElementAt(4, 6, 2.0 * x * w);
        quadricMatrix.setElementAt(4, 7, 2.0 * y * w);
        quadricMatrix.setElementAt(4, 8, 2.0 * z * w);
        quadricMatrix.setElementAt(4, 9, w * w);
        x = point6.getHomX();
        y = point6.getHomY();
        z = point6.getHomZ();
        w = point6.getHomW();
        quadricMatrix.setElementAt(5, 0, x * x);
        quadricMatrix.setElementAt(5, 1, y * y);
        quadricMatrix.setElementAt(5, 2, z * z);
        quadricMatrix.setElementAt(5, 3, 2.0 * x * y);
        quadricMatrix.setElementAt(5, 4, 2.0 * x * z);
        quadricMatrix.setElementAt(5, 5, 2.0 * y * z);
        quadricMatrix.setElementAt(5, 6, 2.0 * x * w);
        quadricMatrix.setElementAt(5, 7, 2.0 * y * w);
        quadricMatrix.setElementAt(5, 8, 2.0 * z * w);
        quadricMatrix.setElementAt(5, 9, w * w);
        x = point7.getHomX();
        y = point7.getHomY();
        z = point7.getHomZ();
        w = point7.getHomW();
        quadricMatrix.setElementAt(6, 0, x * x);
        quadricMatrix.setElementAt(6, 1, y * y);
        quadricMatrix.setElementAt(6, 2, z * z);
        quadricMatrix.setElementAt(6, 3, 2.0 * x * y);
        quadricMatrix.setElementAt(6, 4, 2.0 * x * z);
        quadricMatrix.setElementAt(6, 5, 2.0 * y * z);
        quadricMatrix.setElementAt(6, 6, 2.0 * x * w);
        quadricMatrix.setElementAt(6, 7, 2.0 * y * w);
        quadricMatrix.setElementAt(6, 8, 2.0 * z * w);
        quadricMatrix.setElementAt(6, 9, w * w);
        x = point8.getHomX();
        y = point8.getHomY();
        z = point8.getHomZ();
        w = point8.getHomW();
        quadricMatrix.setElementAt(7, 0, x * x);
        quadricMatrix.setElementAt(7, 1, y * y);
        quadricMatrix.setElementAt(7, 2, z * z);
        quadricMatrix.setElementAt(7, 3, 2.0 * x * y);
        quadricMatrix.setElementAt(7, 4, 2.0 * x * z);
        quadricMatrix.setElementAt(7, 5, 2.0 * y * z);
        quadricMatrix.setElementAt(7, 6, 2.0 * x * w);
        quadricMatrix.setElementAt(7, 7, 2.0 * y * w);
        quadricMatrix.setElementAt(7, 8, 2.0 * z * w);
        quadricMatrix.setElementAt(7, 9, w * w);
        x = point9.getHomX();
        y = point9.getHomY();
        z = point9.getHomZ();
        w = point9.getHomW();
        quadricMatrix.setElementAt(8, 0, x * x);
        quadricMatrix.setElementAt(8, 1, y * y);
        quadricMatrix.setElementAt(8, 2, z * z);
        quadricMatrix.setElementAt(8, 3, 2.0 * x * y);
        quadricMatrix.setElementAt(8, 4, 2.0 * x * z);
        quadricMatrix.setElementAt(8, 5, 2.0 * y * z);
        quadricMatrix.setElementAt(8, 6, 2.0 * x * w);
        quadricMatrix.setElementAt(8, 7, 2.0 * y * w);
        quadricMatrix.setElementAt(8, 8, 2.0 * z * w);
        quadricMatrix.setElementAt(8, 9, w * w);

        while (com.irurueta.algebra.Utils.rank(quadricMatrix) < 9) {
            m = Matrix.createWithUniformRandomValues(9, HOM_COORDS,
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            point1 = new HomogeneousPoint3D(m.getElementAt(0, 0),
                    m.getElementAt(0, 1), m.getElementAt(0, 2), 1.0);
            point2 = new HomogeneousPoint3D(m.getElementAt(1, 0),
                    m.getElementAt(1, 1), m.getElementAt(1, 2), 1.0);
            point3 = new HomogeneousPoint3D(m.getElementAt(2, 0),
                    m.getElementAt(2, 1), m.getElementAt(2, 2), 1.0);
            point4 = new HomogeneousPoint3D(m.getElementAt(3, 0),
                    m.getElementAt(3, 1), m.getElementAt(3, 2), 1.0);
            point5 = new HomogeneousPoint3D(m.getElementAt(4, 0),
                    m.getElementAt(4, 1), m.getElementAt(4, 2), 1.0);
            point6 = new HomogeneousPoint3D(m.getElementAt(5, 0),
                    m.getElementAt(5, 1), m.getElementAt(5, 2), 1.0);
            point7 = new HomogeneousPoint3D(m.getElementAt(6, 0),
                    m.getElementAt(6, 1), m.getElementAt(6, 2), 1.0);
            point8 = new HomogeneousPoint3D(m.getElementAt(7, 0),
                    m.getElementAt(7, 1), m.getElementAt(7, 2), 1.0);
            point9 = new HomogeneousPoint3D(m.getElementAt(8, 0),
                    m.getElementAt(8, 1), m.getElementAt(8, 2), 1.0);

            // estimate quadric that lies inside of provided 9 points
            quadricMatrix = new Matrix(9, 10);
            x = point1.getHomX();
            y = point1.getHomY();
            z = point1.getHomZ();
            w = point1.getHomW();
            quadricMatrix.setElementAt(0, 0, x * x);
            quadricMatrix.setElementAt(0, 1, y * y);
            quadricMatrix.setElementAt(0, 2, z * z);
            quadricMatrix.setElementAt(0, 3, 2.0 * x * y);
            quadricMatrix.setElementAt(0, 4, 2.0 * x * z);
            quadricMatrix.setElementAt(0, 5, 2.0 * y * z);
            quadricMatrix.setElementAt(0, 6, 2.0 * x * w);
            quadricMatrix.setElementAt(0, 7, 2.0 * y * w);
            quadricMatrix.setElementAt(0, 8, 2.0 * z * w);
            quadricMatrix.setElementAt(0, 9, w * w);
            x = point2.getHomX();
            y = point2.getHomY();
            z = point2.getHomZ();
            w = point2.getHomW();
            quadricMatrix.setElementAt(1, 0, x * x);
            quadricMatrix.setElementAt(1, 1, y * y);
            quadricMatrix.setElementAt(1, 2, z * z);
            quadricMatrix.setElementAt(1, 3, 2.0 * x * y);
            quadricMatrix.setElementAt(1, 4, 2.0 * x * z);
            quadricMatrix.setElementAt(1, 5, 2.0 * y * z);
            quadricMatrix.setElementAt(1, 6, 2.0 * x * w);
            quadricMatrix.setElementAt(1, 7, 2.0 * y * w);
            quadricMatrix.setElementAt(1, 8, 2.0 * z * w);
            quadricMatrix.setElementAt(1, 9, w * w);
            x = point3.getHomX();
            y = point3.getHomY();
            z = point3.getHomZ();
            w = point3.getHomW();
            quadricMatrix.setElementAt(2, 0, x * x);
            quadricMatrix.setElementAt(2, 1, y * y);
            quadricMatrix.setElementAt(2, 2, z * z);
            quadricMatrix.setElementAt(2, 3, 2.0 * x * y);
            quadricMatrix.setElementAt(2, 4, 2.0 * x * z);
            quadricMatrix.setElementAt(2, 5, 2.0 * y * z);
            quadricMatrix.setElementAt(2, 6, 2.0 * x * w);
            quadricMatrix.setElementAt(2, 7, 2.0 * y * w);
            quadricMatrix.setElementAt(2, 8, 2.0 * z * w);
            quadricMatrix.setElementAt(2, 9, w * w);
            x = point4.getHomX();
            y = point4.getHomY();
            z = point4.getHomZ();
            w = point4.getHomW();
            quadricMatrix.setElementAt(3, 0, x * x);
            quadricMatrix.setElementAt(3, 1, y * y);
            quadricMatrix.setElementAt(3, 2, z * z);
            quadricMatrix.setElementAt(3, 3, 2.0 * x * y);
            quadricMatrix.setElementAt(3, 4, 2.0 * x * z);
            quadricMatrix.setElementAt(3, 5, 2.0 * y * z);
            quadricMatrix.setElementAt(3, 6, 2.0 * x * w);
            quadricMatrix.setElementAt(3, 7, 2.0 * y * w);
            quadricMatrix.setElementAt(3, 8, 2.0 * z * w);
            quadricMatrix.setElementAt(3, 9, w * w);
            x = point5.getHomX();
            y = point5.getHomY();
            z = point5.getHomZ();
            w = point5.getHomW();
            quadricMatrix.setElementAt(4, 0, x * x);
            quadricMatrix.setElementAt(4, 1, y * y);
            quadricMatrix.setElementAt(4, 2, z * z);
            quadricMatrix.setElementAt(4, 3, 2.0 * x * y);
            quadricMatrix.setElementAt(4, 4, 2.0 * x * z);
            quadricMatrix.setElementAt(4, 5, 2.0 * y * z);
            quadricMatrix.setElementAt(4, 6, 2.0 * x * w);
            quadricMatrix.setElementAt(4, 7, 2.0 * y * w);
            quadricMatrix.setElementAt(4, 8, 2.0 * z * w);
            quadricMatrix.setElementAt(4, 9, w * w);
            x = point6.getHomX();
            y = point6.getHomY();
            z = point6.getHomZ();
            w = point6.getHomW();
            quadricMatrix.setElementAt(5, 0, x * x);
            quadricMatrix.setElementAt(5, 1, y * y);
            quadricMatrix.setElementAt(5, 2, z * z);
            quadricMatrix.setElementAt(5, 3, 2.0 * x * y);
            quadricMatrix.setElementAt(5, 4, 2.0 * x * z);
            quadricMatrix.setElementAt(5, 5, 2.0 * y * z);
            quadricMatrix.setElementAt(5, 6, 2.0 * x * w);
            quadricMatrix.setElementAt(5, 7, 2.0 * y * w);
            quadricMatrix.setElementAt(5, 8, 2.0 * z * w);
            quadricMatrix.setElementAt(5, 9, w * w);
            x = point7.getHomX();
            y = point7.getHomY();
            z = point7.getHomZ();
            w = point7.getHomW();
            quadricMatrix.setElementAt(6, 0, x * x);
            quadricMatrix.setElementAt(6, 1, y * y);
            quadricMatrix.setElementAt(6, 2, z * z);
            quadricMatrix.setElementAt(6, 3, 2.0 * x * y);
            quadricMatrix.setElementAt(6, 4, 2.0 * x * z);
            quadricMatrix.setElementAt(6, 5, 2.0 * y * z);
            quadricMatrix.setElementAt(6, 6, 2.0 * x * w);
            quadricMatrix.setElementAt(6, 7, 2.0 * y * w);
            quadricMatrix.setElementAt(6, 8, 2.0 * z * w);
            quadricMatrix.setElementAt(6, 9, w * w);
            x = point8.getHomX();
            y = point8.getHomY();
            z = point8.getHomZ();
            w = point8.getHomW();
            quadricMatrix.setElementAt(7, 0, x * x);
            quadricMatrix.setElementAt(7, 1, y * y);
            quadricMatrix.setElementAt(7, 2, z * z);
            quadricMatrix.setElementAt(7, 3, 2.0 * x * y);
            quadricMatrix.setElementAt(7, 4, 2.0 * x * z);
            quadricMatrix.setElementAt(7, 5, 2.0 * y * z);
            quadricMatrix.setElementAt(7, 6, 2.0 * x * w);
            quadricMatrix.setElementAt(7, 7, 2.0 * y * w);
            quadricMatrix.setElementAt(7, 8, 2.0 * z * w);
            quadricMatrix.setElementAt(7, 9, w * w);
            x = point9.getHomX();
            y = point9.getHomY();
            z = point9.getHomZ();
            w = point9.getHomW();
            quadricMatrix.setElementAt(8, 0, x * x);
            quadricMatrix.setElementAt(8, 1, y * y);
            quadricMatrix.setElementAt(8, 2, z * z);
            quadricMatrix.setElementAt(8, 3, 2.0 * x * y);
            quadricMatrix.setElementAt(8, 4, 2.0 * x * z);
            quadricMatrix.setElementAt(8, 5, 2.0 * y * z);
            quadricMatrix.setElementAt(8, 6, 2.0 * x * w);
            quadricMatrix.setElementAt(8, 7, 2.0 * y * w);
            quadricMatrix.setElementAt(8, 8, 2.0 * z * w);
            quadricMatrix.setElementAt(8, 9, w * w);
        }

        quadric = new Quadric(point1, point2, point3, point4, point5, point6,
                point7, point8, point9);
        assertTrue(quadric.isLocus(point1, PRECISION_ERROR));
        assertTrue(quadric.isLocus(point2, PRECISION_ERROR));
        assertTrue(quadric.isLocus(point3, PRECISION_ERROR));
        assertTrue(quadric.isLocus(point4, PRECISION_ERROR));
        assertTrue(quadric.isLocus(point5, PRECISION_ERROR));
        assertTrue(quadric.isLocus(point6, PRECISION_ERROR));
        assertTrue(quadric.isLocus(point7, PRECISION_ERROR));
        assertTrue(quadric.isLocus(point8, PRECISION_ERROR));
        assertTrue(quadric.isLocus(point9, PRECISION_ERROR));

        // Force CoincidentPointsException
        quadric = null;
        try {
            quadric = new Quadric(point1, point2, point3, point4, point5,
                    point6, point7, point8, point8);
            fail("CoincidentPointsException expected but not thrown");
        } catch (final CoincidentPointsException ignore) {
        }
        assertNull(quadric);
    }

    @Test
    public void testGettersAndSetters() throws WrongSizeException,
            IllegalArgumentException, NonSymmetricMatrixException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final Quadric quadric = new Quadric();
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
        quadric.setA(a);
        quadric.setB(b);
        quadric.setC(c);
        quadric.setD(d);
        quadric.setE(e);
        quadric.setF(f);
        quadric.setG(g);
        quadric.setH(h);
        quadric.setI(i);
        quadric.setJ(j);
        assertEquals(quadric.getA(), a, 0.0);
        assertEquals(quadric.getB(), b, 0.0);
        assertEquals(quadric.getC(), c, 0.0);
        assertEquals(quadric.getD(), d, 0.0);
        assertEquals(quadric.getE(), e, 0.0);
        assertEquals(quadric.getF(), f, 0.0);
        assertEquals(quadric.getG(), g, 0.0);
        assertEquals(quadric.getH(), h, 0.0);
        assertEquals(quadric.getI(), i, 0.0);
        assertEquals(quadric.getJ(), j, 0.0);

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
        quadric.setParameters(a, b, c, d, e, f, g, h, i, j);
        assertEquals(quadric.getA(), a, 0.0);
        assertEquals(quadric.getB(), b, 0.0);
        assertEquals(quadric.getC(), c, 0.0);
        assertEquals(quadric.getD(), d, 0.0);
        assertEquals(quadric.getE(), e, 0.0);
        assertEquals(quadric.getF(), f, 0.0);
        assertEquals(quadric.getG(), g, 0.0);
        assertEquals(quadric.getH(), h, 0.0);
        assertEquals(quadric.getI(), i, 0.0);
        assertEquals(quadric.getJ(), j, 0.0);

        Matrix m = new Matrix(QUADRIC_ROWS, QUADRIC_COLS);
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
        quadric.setParameters(m);
        assertEquals(quadric.getA(), m.getElementAt(0, 0), 0.0);
        assertEquals(quadric.getB(), m.getElementAt(1, 1), 0.0);
        assertEquals(quadric.getC(), m.getElementAt(2, 2), 0.0);
        assertEquals(quadric.getD(), m.getElementAt(1, 0), 0.0);
        assertEquals(quadric.getE(), m.getElementAt(2, 1), 0.0);
        assertEquals(quadric.getF(), m.getElementAt(2, 0), 0.0);
        assertEquals(quadric.getG(), m.getElementAt(3, 0), 0.0);
        assertEquals(quadric.getH(), m.getElementAt(3, 1), 0.0);
        assertEquals(quadric.getI(), m.getElementAt(3, 2), 0.0);

        // Force IllegalArgumentException
        m = new Matrix(QUADRIC_ROWS + 1, QUADRIC_COLS + 1);
        try {
            quadric.setParameters(m);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        // Force NonSymmetricMatrixException
        m = new Matrix(QUADRIC_ROWS, QUADRIC_COLS);
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
        try {
            quadric.setParameters(m);
            fail("NonSymmetricMatrixException expected but not thrown");
        } catch (final NonSymmetricMatrixException ignore) {
        }
        quadric.setParameters(m, 1.0);

    }

    @Test
    public void testAsMatrix() throws WrongSizeException,
            IllegalArgumentException, NonSymmetricMatrixException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Quadric quadric = new Quadric();
        final Matrix m = new Matrix(QUADRIC_ROWS, QUADRIC_COLS);
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
        quadric.setParameters(m);

        final Matrix m2 = quadric.asMatrix();

        assertTrue(m.equals(m2));
    }

    @Test
    public void testIsLocus() throws WrongSizeException, DecomposerException,
            NotReadyException, LockedException,
            com.irurueta.algebra.NotAvailableException, IllegalArgumentException,
            NonSymmetricMatrixException, CoincidentPointsException {

        Matrix m = Matrix.createWithUniformRandomValues(9, HOM_COORDS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        HomogeneousPoint3D point1 = new HomogeneousPoint3D(m.getElementAt(0, 0),
                m.getElementAt(0, 1), m.getElementAt(0, 2),
                m.getElementAt(0, 3));
        HomogeneousPoint3D point2 = new HomogeneousPoint3D(m.getElementAt(1, 0),
                m.getElementAt(1, 1), m.getElementAt(1, 2),
                m.getElementAt(1, 3));
        HomogeneousPoint3D point3 = new HomogeneousPoint3D(m.getElementAt(2, 0),
                m.getElementAt(2, 1), m.getElementAt(2, 2),
                m.getElementAt(2, 3));
        HomogeneousPoint3D point4 = new HomogeneousPoint3D(m.getElementAt(3, 0),
                m.getElementAt(3, 1), m.getElementAt(3, 2),
                m.getElementAt(3, 3));
        HomogeneousPoint3D point5 = new HomogeneousPoint3D(m.getElementAt(4, 0),
                m.getElementAt(4, 1), m.getElementAt(4, 2),
                m.getElementAt(4, 3));
        HomogeneousPoint3D point6 = new HomogeneousPoint3D(m.getElementAt(5, 0),
                m.getElementAt(5, 1), m.getElementAt(5, 2),
                m.getElementAt(5, 3));
        HomogeneousPoint3D point7 = new HomogeneousPoint3D(m.getElementAt(6, 0),
                m.getElementAt(6, 1), m.getElementAt(6, 2),
                m.getElementAt(6, 3));
        HomogeneousPoint3D point8 = new HomogeneousPoint3D(m.getElementAt(7, 0),
                m.getElementAt(7, 1), m.getElementAt(7, 2),
                m.getElementAt(7, 3));
        HomogeneousPoint3D point9 = new HomogeneousPoint3D(m.getElementAt(8, 0),
                m.getElementAt(8, 1), m.getElementAt(8, 2),
                m.getElementAt(8, 3));

        point1.normalize();
        point2.normalize();
        point3.normalize();
        point4.normalize();
        point5.normalize();
        point6.normalize();
        point7.normalize();
        point8.normalize();
        point9.normalize();

        // estimate quadric that lies inside of provided 9 points
        Matrix quadricMatrix = new Matrix(9, 10);
        double x = point1.getHomX();
        double y = point1.getHomY();
        double z = point1.getHomZ();
        double w = point1.getHomW();
        quadricMatrix.setElementAt(0, 0, x * x);
        quadricMatrix.setElementAt(0, 1, y * y);
        quadricMatrix.setElementAt(0, 2, z * z);
        quadricMatrix.setElementAt(0, 3, 2.0 * x * y);
        quadricMatrix.setElementAt(0, 4, 2.0 * x * z);
        quadricMatrix.setElementAt(0, 5, 2.0 * y * z);
        quadricMatrix.setElementAt(0, 6, 2.0 * x * w);
        quadricMatrix.setElementAt(0, 7, 2.0 * y * w);
        quadricMatrix.setElementAt(0, 8, 2.0 * z * w);
        quadricMatrix.setElementAt(0, 9, w * w);
        x = point2.getHomX();
        y = point2.getHomY();
        z = point2.getHomZ();
        w = point2.getHomW();
        quadricMatrix.setElementAt(1, 0, x * x);
        quadricMatrix.setElementAt(1, 1, y * y);
        quadricMatrix.setElementAt(1, 2, z * z);
        quadricMatrix.setElementAt(1, 3, 2.0 * x * y);
        quadricMatrix.setElementAt(1, 4, 2.0 * x * z);
        quadricMatrix.setElementAt(1, 5, 2.0 * y * z);
        quadricMatrix.setElementAt(1, 6, 2.0 * x * w);
        quadricMatrix.setElementAt(1, 7, 2.0 * y * w);
        quadricMatrix.setElementAt(1, 8, 2.0 * z * w);
        quadricMatrix.setElementAt(1, 9, w * w);
        x = point3.getHomX();
        y = point3.getHomY();
        z = point3.getHomZ();
        w = point3.getHomW();
        quadricMatrix.setElementAt(2, 0, x * x);
        quadricMatrix.setElementAt(2, 1, y * y);
        quadricMatrix.setElementAt(2, 2, z * z);
        quadricMatrix.setElementAt(2, 3, 2.0 * x * y);
        quadricMatrix.setElementAt(2, 4, 2.0 * x * z);
        quadricMatrix.setElementAt(2, 5, 2.0 * y * z);
        quadricMatrix.setElementAt(2, 6, 2.0 * x * w);
        quadricMatrix.setElementAt(2, 7, 2.0 * y * w);
        quadricMatrix.setElementAt(2, 8, 2.0 * z * w);
        quadricMatrix.setElementAt(2, 9, w * w);
        x = point4.getHomX();
        y = point4.getHomY();
        z = point4.getHomZ();
        w = point4.getHomW();
        quadricMatrix.setElementAt(3, 0, x * x);
        quadricMatrix.setElementAt(3, 1, y * y);
        quadricMatrix.setElementAt(3, 2, z * z);
        quadricMatrix.setElementAt(3, 3, 2.0 * x * y);
        quadricMatrix.setElementAt(3, 4, 2.0 * x * z);
        quadricMatrix.setElementAt(3, 5, 2.0 * y * z);
        quadricMatrix.setElementAt(3, 6, 2.0 * x * w);
        quadricMatrix.setElementAt(3, 7, 2.0 * y * w);
        quadricMatrix.setElementAt(3, 8, 2.0 * z * w);
        quadricMatrix.setElementAt(3, 9, w * w);
        x = point5.getHomX();
        y = point5.getHomY();
        z = point5.getHomZ();
        w = point5.getHomW();
        quadricMatrix.setElementAt(4, 0, x * x);
        quadricMatrix.setElementAt(4, 1, y * y);
        quadricMatrix.setElementAt(4, 2, z * z);
        quadricMatrix.setElementAt(4, 3, 2.0 * x * y);
        quadricMatrix.setElementAt(4, 4, 2.0 * x * z);
        quadricMatrix.setElementAt(4, 5, 2.0 * y * z);
        quadricMatrix.setElementAt(4, 6, 2.0 * x * w);
        quadricMatrix.setElementAt(4, 7, 2.0 * y * w);
        quadricMatrix.setElementAt(4, 8, 2.0 * z * w);
        quadricMatrix.setElementAt(4, 9, w * w);
        x = point6.getHomX();
        y = point6.getHomY();
        z = point6.getHomZ();
        w = point6.getHomW();
        quadricMatrix.setElementAt(5, 0, x * x);
        quadricMatrix.setElementAt(5, 1, y * y);
        quadricMatrix.setElementAt(5, 2, z * z);
        quadricMatrix.setElementAt(5, 3, 2.0 * x * y);
        quadricMatrix.setElementAt(5, 4, 2.0 * x * z);
        quadricMatrix.setElementAt(5, 5, 2.0 * y * z);
        quadricMatrix.setElementAt(5, 6, 2.0 * x * w);
        quadricMatrix.setElementAt(5, 7, 2.0 * y * w);
        quadricMatrix.setElementAt(5, 8, 2.0 * z * w);
        quadricMatrix.setElementAt(5, 9, w * w);
        x = point7.getHomX();
        y = point7.getHomY();
        z = point7.getHomZ();
        w = point7.getHomW();
        quadricMatrix.setElementAt(6, 0, x * x);
        quadricMatrix.setElementAt(6, 1, y * y);
        quadricMatrix.setElementAt(6, 2, z * z);
        quadricMatrix.setElementAt(6, 3, 2.0 * x * y);
        quadricMatrix.setElementAt(6, 4, 2.0 * x * z);
        quadricMatrix.setElementAt(6, 5, 2.0 * y * z);
        quadricMatrix.setElementAt(6, 6, 2.0 * x * w);
        quadricMatrix.setElementAt(6, 7, 2.0 * y * w);
        quadricMatrix.setElementAt(6, 8, 2.0 * z * w);
        quadricMatrix.setElementAt(6, 9, w * w);
        x = point8.getHomX();
        y = point8.getHomY();
        z = point8.getHomZ();
        w = point8.getHomW();
        quadricMatrix.setElementAt(7, 0, x * x);
        quadricMatrix.setElementAt(7, 1, y * y);
        quadricMatrix.setElementAt(7, 2, z * z);
        quadricMatrix.setElementAt(7, 3, 2.0 * x * y);
        quadricMatrix.setElementAt(7, 4, 2.0 * x * z);
        quadricMatrix.setElementAt(7, 5, 2.0 * y * z);
        quadricMatrix.setElementAt(7, 6, 2.0 * x * w);
        quadricMatrix.setElementAt(7, 7, 2.0 * y * w);
        quadricMatrix.setElementAt(7, 8, 2.0 * z * w);
        quadricMatrix.setElementAt(7, 9, w * w);
        x = point9.getHomX();
        y = point9.getHomY();
        z = point9.getHomZ();
        w = point9.getHomW();
        quadricMatrix.setElementAt(8, 0, x * x);
        quadricMatrix.setElementAt(8, 1, y * y);
        quadricMatrix.setElementAt(8, 2, z * z);
        quadricMatrix.setElementAt(8, 3, 2.0 * x * y);
        quadricMatrix.setElementAt(8, 4, 2.0 * x * z);
        quadricMatrix.setElementAt(8, 5, 2.0 * y * z);
        quadricMatrix.setElementAt(8, 6, 2.0 * x * w);
        quadricMatrix.setElementAt(8, 7, 2.0 * y * w);
        quadricMatrix.setElementAt(8, 8, 2.0 * z * w);
        quadricMatrix.setElementAt(8, 9, w * w);

        while (com.irurueta.algebra.Utils.rank(quadricMatrix) < 9) {
            m = Matrix.createWithUniformRandomValues(9, HOM_COORDS,
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            point1 = new HomogeneousPoint3D(m.getElementAt(0, 0),
                    m.getElementAt(0, 1), m.getElementAt(0, 2),
                    m.getElementAt(0, 3));
            point2 = new HomogeneousPoint3D(m.getElementAt(1, 0),
                    m.getElementAt(1, 1), m.getElementAt(1, 2),
                    m.getElementAt(1, 3));
            point3 = new HomogeneousPoint3D(m.getElementAt(2, 0),
                    m.getElementAt(2, 1), m.getElementAt(2, 2),
                    m.getElementAt(2, 3));
            point4 = new HomogeneousPoint3D(m.getElementAt(3, 0),
                    m.getElementAt(3, 1), m.getElementAt(3, 2),
                    m.getElementAt(3, 3));
            point5 = new HomogeneousPoint3D(m.getElementAt(4, 0),
                    m.getElementAt(4, 1), m.getElementAt(4, 2),
                    m.getElementAt(4, 3));
            point6 = new HomogeneousPoint3D(m.getElementAt(5, 0),
                    m.getElementAt(5, 1), m.getElementAt(5, 2),
                    m.getElementAt(5, 3));
            point7 = new HomogeneousPoint3D(m.getElementAt(6, 0),
                    m.getElementAt(6, 1), m.getElementAt(6, 2),
                    m.getElementAt(6, 3));
            point8 = new HomogeneousPoint3D(m.getElementAt(7, 0),
                    m.getElementAt(7, 1), m.getElementAt(7, 2),
                    m.getElementAt(7, 3));
            point9 = new HomogeneousPoint3D(m.getElementAt(8, 0),
                    m.getElementAt(8, 1), m.getElementAt(8, 2),
                    m.getElementAt(8, 3));

            point1.normalize();
            point2.normalize();
            point3.normalize();
            point4.normalize();
            point5.normalize();
            point6.normalize();
            point7.normalize();
            point8.normalize();
            point9.normalize();

            // estimate quadric that lies inside of provided 9 points
            quadricMatrix = new Matrix(9, 10);
            x = point1.getHomX();
            y = point1.getHomY();
            z = point1.getHomZ();
            w = point1.getHomW();
            quadricMatrix.setElementAt(0, 0, x * x);
            quadricMatrix.setElementAt(0, 1, y * y);
            quadricMatrix.setElementAt(0, 2, z * z);
            quadricMatrix.setElementAt(0, 3, 2.0 * x * y);
            quadricMatrix.setElementAt(0, 4, 2.0 * x * z);
            quadricMatrix.setElementAt(0, 5, 2.0 * y * z);
            quadricMatrix.setElementAt(0, 6, 2.0 * x * w);
            quadricMatrix.setElementAt(0, 7, 2.0 * y * w);
            quadricMatrix.setElementAt(0, 8, 2.0 * z * w);
            quadricMatrix.setElementAt(0, 9, w * w);
            x = point2.getHomX();
            y = point2.getHomY();
            z = point2.getHomZ();
            w = point2.getHomW();
            quadricMatrix.setElementAt(1, 0, x * x);
            quadricMatrix.setElementAt(1, 1, y * y);
            quadricMatrix.setElementAt(1, 2, z * z);
            quadricMatrix.setElementAt(1, 3, 2.0 * x * y);
            quadricMatrix.setElementAt(1, 4, 2.0 * x * z);
            quadricMatrix.setElementAt(1, 5, 2.0 * y * z);
            quadricMatrix.setElementAt(1, 6, 2.0 * x * w);
            quadricMatrix.setElementAt(1, 7, 2.0 * y * w);
            quadricMatrix.setElementAt(1, 8, 2.0 * z * w);
            quadricMatrix.setElementAt(1, 9, w * w);
            x = point3.getHomX();
            y = point3.getHomY();
            z = point3.getHomZ();
            w = point3.getHomW();
            quadricMatrix.setElementAt(2, 0, x * x);
            quadricMatrix.setElementAt(2, 1, y * y);
            quadricMatrix.setElementAt(2, 2, z * z);
            quadricMatrix.setElementAt(2, 3, 2.0 * x * y);
            quadricMatrix.setElementAt(2, 4, 2.0 * x * z);
            quadricMatrix.setElementAt(2, 5, 2.0 * y * z);
            quadricMatrix.setElementAt(2, 6, 2.0 * x * w);
            quadricMatrix.setElementAt(2, 7, 2.0 * y * w);
            quadricMatrix.setElementAt(2, 8, 2.0 * z * w);
            quadricMatrix.setElementAt(2, 9, w * w);
            x = point4.getHomX();
            y = point4.getHomY();
            z = point4.getHomZ();
            w = point4.getHomW();
            quadricMatrix.setElementAt(3, 0, x * x);
            quadricMatrix.setElementAt(3, 1, y * y);
            quadricMatrix.setElementAt(3, 2, z * z);
            quadricMatrix.setElementAt(3, 3, 2.0 * x * y);
            quadricMatrix.setElementAt(3, 4, 2.0 * x * z);
            quadricMatrix.setElementAt(3, 5, 2.0 * y * z);
            quadricMatrix.setElementAt(3, 6, 2.0 * x * w);
            quadricMatrix.setElementAt(3, 7, 2.0 * y * w);
            quadricMatrix.setElementAt(3, 8, 2.0 * z * w);
            quadricMatrix.setElementAt(3, 9, w * w);
            x = point5.getHomX();
            y = point5.getHomY();
            z = point5.getHomZ();
            w = point5.getHomW();
            quadricMatrix.setElementAt(4, 0, x * x);
            quadricMatrix.setElementAt(4, 1, y * y);
            quadricMatrix.setElementAt(4, 2, z * z);
            quadricMatrix.setElementAt(4, 3, 2.0 * x * y);
            quadricMatrix.setElementAt(4, 4, 2.0 * x * z);
            quadricMatrix.setElementAt(4, 5, 2.0 * y * z);
            quadricMatrix.setElementAt(4, 6, 2.0 * x * w);
            quadricMatrix.setElementAt(4, 7, 2.0 * y * w);
            quadricMatrix.setElementAt(4, 8, 2.0 * z * w);
            quadricMatrix.setElementAt(4, 9, w * w);
            x = point6.getHomX();
            y = point6.getHomY();
            z = point6.getHomZ();
            w = point6.getHomW();
            quadricMatrix.setElementAt(5, 0, x * x);
            quadricMatrix.setElementAt(5, 1, y * y);
            quadricMatrix.setElementAt(5, 2, z * z);
            quadricMatrix.setElementAt(5, 3, 2.0 * x * y);
            quadricMatrix.setElementAt(5, 4, 2.0 * x * z);
            quadricMatrix.setElementAt(5, 5, 2.0 * y * z);
            quadricMatrix.setElementAt(5, 6, 2.0 * x * w);
            quadricMatrix.setElementAt(5, 7, 2.0 * y * w);
            quadricMatrix.setElementAt(5, 8, 2.0 * z * w);
            quadricMatrix.setElementAt(5, 9, w * w);
            x = point7.getHomX();
            y = point7.getHomY();
            z = point7.getHomZ();
            w = point7.getHomW();
            quadricMatrix.setElementAt(6, 0, x * x);
            quadricMatrix.setElementAt(6, 1, y * y);
            quadricMatrix.setElementAt(6, 2, z * z);
            quadricMatrix.setElementAt(6, 3, 2.0 * x * y);
            quadricMatrix.setElementAt(6, 4, 2.0 * x * z);
            quadricMatrix.setElementAt(6, 5, 2.0 * y * z);
            quadricMatrix.setElementAt(6, 6, 2.0 * x * w);
            quadricMatrix.setElementAt(6, 7, 2.0 * y * w);
            quadricMatrix.setElementAt(6, 8, 2.0 * z * w);
            quadricMatrix.setElementAt(6, 9, w * w);
            x = point8.getHomX();
            y = point8.getHomY();
            z = point8.getHomZ();
            w = point8.getHomW();
            quadricMatrix.setElementAt(7, 0, x * x);
            quadricMatrix.setElementAt(7, 1, y * y);
            quadricMatrix.setElementAt(7, 2, z * z);
            quadricMatrix.setElementAt(7, 3, 2.0 * x * y);
            quadricMatrix.setElementAt(7, 4, 2.0 * x * z);
            quadricMatrix.setElementAt(7, 5, 2.0 * y * z);
            quadricMatrix.setElementAt(7, 6, 2.0 * x * w);
            quadricMatrix.setElementAt(7, 7, 2.0 * y * w);
            quadricMatrix.setElementAt(7, 8, 2.0 * z * w);
            quadricMatrix.setElementAt(7, 9, w * w);
            x = point9.getHomX();
            y = point9.getHomY();
            z = point9.getHomZ();
            w = point9.getHomW();
            quadricMatrix.setElementAt(8, 0, x * x);
            quadricMatrix.setElementAt(8, 1, y * y);
            quadricMatrix.setElementAt(8, 2, z * z);
            quadricMatrix.setElementAt(8, 3, 2.0 * x * y);
            quadricMatrix.setElementAt(8, 4, 2.0 * x * z);
            quadricMatrix.setElementAt(8, 5, 2.0 * y * z);
            quadricMatrix.setElementAt(8, 6, 2.0 * x * w);
            quadricMatrix.setElementAt(8, 7, 2.0 * y * w);
            quadricMatrix.setElementAt(8, 8, 2.0 * z * w);
            quadricMatrix.setElementAt(8, 9, w * w);
        }

        final SingularValueDecomposer decomposer = new SingularValueDecomposer(
                quadricMatrix);
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

        final Matrix quadricPoint = new Matrix(QUADRIC_ROWS, 1);
        quadricPoint.setElementAt(0, 0, point1.getHomX());
        quadricPoint.setElementAt(1, 0, point1.getHomY());
        quadricPoint.setElementAt(2, 0, point1.getHomZ());
        quadricPoint.setElementAt(3, 0, point1.getHomW());

        double norm = com.irurueta.algebra.Utils.normF(quadricPoint);
        quadricPoint.multiplyByScalar(1.0 / norm);

        final Matrix resultQuadricMatrix = new Matrix(QUADRIC_ROWS, QUADRIC_COLS);
        resultQuadricMatrix.setElementAt(0, 0, a);
        resultQuadricMatrix.setElementAt(1, 1, b);
        resultQuadricMatrix.setElementAt(2, 2, c);
        resultQuadricMatrix.setElementAt(3, 3, j);
        resultQuadricMatrix.setElementAt(1, 0, d);
        resultQuadricMatrix.setElementAt(0, 1, d);
        resultQuadricMatrix.setElementAt(2, 1, e);
        resultQuadricMatrix.setElementAt(1, 2, e);
        resultQuadricMatrix.setElementAt(2, 0, f);
        resultQuadricMatrix.setElementAt(0, 2, f);
        resultQuadricMatrix.setElementAt(3, 0, g);
        resultQuadricMatrix.setElementAt(0, 3, g);
        resultQuadricMatrix.setElementAt(3, 1, h);
        resultQuadricMatrix.setElementAt(1, 3, h);
        resultQuadricMatrix.setElementAt(3, 2, i);
        resultQuadricMatrix.setElementAt(2, 3, i);

        norm = com.irurueta.algebra.Utils.normF(resultQuadricMatrix);
        resultQuadricMatrix.multiplyByScalar(1.0 / norm);

        // find plane tangent to quadric resultQuadricMatrix at point
        // quadricPoint
        final Matrix planeParams = resultQuadricMatrix.multiplyAndReturnNew(
                quadricPoint);

        norm = com.irurueta.algebra.Utils.normF(planeParams);
        planeParams.multiplyByScalar(1.0 / norm);

        // use director vector of tangent plane to find a point outside quadric
        double directVectorA = planeParams.getElementAt(0, 0);
        double directVectorB = planeParams.getElementAt(1, 0);
        double directVectorC = planeParams.getElementAt(2, 0);
        final double directVectorNorm = Math.sqrt(directVectorA * directVectorA +
                directVectorB * directVectorB + directVectorC * directVectorC);
        directVectorA /= directVectorNorm;
        directVectorB /= directVectorNorm;
        directVectorC /= directVectorNorm;
        final InhomogeneousPoint3D outsidePoint = new InhomogeneousPoint3D(
                point1.getInhomX() + directVectorA,
                point1.getInhomY() + directVectorB,
                point1.getInhomZ() + directVectorC);

        // instantiate new quadric instance
        final Quadric quadric = new Quadric(resultQuadricMatrix);

        // check that initial 5 points lie inside the quadric
        assertTrue(quadric.isLocus(point1, LOCUS_THRESHOLD));
        assertTrue(quadric.isLocus(point2, LOCUS_THRESHOLD));
        assertTrue(quadric.isLocus(point3, LOCUS_THRESHOLD));
        assertTrue(quadric.isLocus(point4, LOCUS_THRESHOLD));
        assertTrue(quadric.isLocus(point5, LOCUS_THRESHOLD));
        assertTrue(quadric.isLocus(point6, LOCUS_THRESHOLD));
        assertTrue(quadric.isLocus(point7, LOCUS_THRESHOLD));
        assertTrue(quadric.isLocus(point8, LOCUS_THRESHOLD));
        assertTrue(quadric.isLocus(point9, LOCUS_THRESHOLD));

        // check point outside of quadric
        assertFalse(quadric.isLocus(outsidePoint, LOCUS_THRESHOLD));

        // test constructor from 9 points
        final Quadric quadric2 = new Quadric(point1, point2, point3, point4, point5,
                point6, point7, point8, point9);
        assertTrue(quadric2.isLocus(point1, LOCUS_THRESHOLD));
        assertTrue(quadric2.isLocus(point2, LOCUS_THRESHOLD));
        assertTrue(quadric2.isLocus(point3, LOCUS_THRESHOLD));
        assertTrue(quadric2.isLocus(point4, LOCUS_THRESHOLD));
        assertTrue(quadric2.isLocus(point5, LOCUS_THRESHOLD));
        assertTrue(quadric2.isLocus(point6, LOCUS_THRESHOLD));
        assertTrue(quadric2.isLocus(point7, LOCUS_THRESHOLD));
        assertTrue(quadric2.isLocus(point8, LOCUS_THRESHOLD));
        assertTrue(quadric2.isLocus(point9, LOCUS_THRESHOLD));
    }

    @Test
    public void testAngleBetweenPoints() throws WrongSizeException,
            IllegalArgumentException, NonSymmetricMatrixException {

        // initial 3D points
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

        // compute quadric matrix as the product of transposed transform matrix
        // with itself
        final Matrix transposedTransform = transform.transposeAndReturnNew();
        final Matrix quadricMatrix = transposedTransform.multiplyAndReturnNew(
                transform);

        // normalize conic matrix
        final double normQuadric = com.irurueta.algebra.Utils.normF(quadricMatrix);
        quadricMatrix.multiplyByScalar(1.0 / normQuadric);

        final Quadric quadric = new Quadric(quadricMatrix);

        final Point3D point1 = new HomogeneousPoint3D(point1Matrix.getElementAt(0, 0),
                point1Matrix.getElementAt(1, 0),
                point1Matrix.getElementAt(2, 0),
                point1Matrix.getElementAt(3, 0));
        final Point3D point2 = new HomogeneousPoint3D(point2Matrix.getElementAt(0, 0),
                point2Matrix.getElementAt(1, 0),
                point2Matrix.getElementAt(2, 0),
                point2Matrix.getElementAt(3, 0));

        assertEquals(quadric.angleBetweenPoints(point1, point2), angle,
                PRECISION_ERROR);
    }

    @Test
    public void testArePerpendicularPoints() throws WrongSizeException,
            DecomposerException, RankDeficientMatrixException,
            IllegalArgumentException, NonSymmetricMatrixException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            Matrix matrixPoint1 = Matrix.createWithUniformRandomValues(HOM_COORDS,
                    1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            double norm = com.irurueta.algebra.Utils.normF(matrixPoint1);
            matrixPoint1.multiplyByScalar(1.0 / norm);

            Matrix matrixPoint2 = Matrix.createWithUniformRandomValues(HOM_COORDS,
                    1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            matrixPoint2.setElementAt(0, 0, matrixPoint1.getElementAt(1, 0) +
                    matrixPoint1.getElementAt(2, 0) +
                    matrixPoint1.getElementAt(3, 0));
            matrixPoint2.setElementAt(1, 0, -matrixPoint1.getElementAt(0, 0));
            matrixPoint2.setElementAt(2, 0, -matrixPoint1.getElementAt(0, 0));
            matrixPoint2.setElementAt(3, 0, -matrixPoint1.getElementAt(0, 0));

            norm = com.irurueta.algebra.Utils.normF(matrixPoint2);
            matrixPoint2.multiplyByScalar(1.0 / norm);

            Matrix transform = Matrix.createWithUniformRandomValues(HOM_COORDS,
                    HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            while (com.irurueta.algebra.Utils.rank(transform) < 4) {
                transform = Matrix.createWithUniformRandomValues(HOM_COORDS,
                        HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            }

            final Matrix invTransform = com.irurueta.algebra.Utils.inverse(transform);

            Matrix transformPointMatrix1 = transform.multiplyAndReturnNew(
                    matrixPoint1);
            Matrix transformPointMatrix2 = transform.multiplyAndReturnNew(
                    matrixPoint2);

            final Matrix transInvTransform = invTransform.transposeAndReturnNew();

            final Matrix quadricMatrix = transInvTransform.multiplyAndReturnNew(
                    invTransform);
            norm = com.irurueta.algebra.Utils.normF(quadricMatrix);
            quadricMatrix.multiplyByScalar(1.0 / norm);

            Point3D transformPoint1 = new HomogeneousPoint3D(
                    transformPointMatrix1.toArray());
            Point3D transformPoint2 = new HomogeneousPoint3D(
                    transformPointMatrix2.toArray());

            final Quadric quadric = new Quadric(quadricMatrix);

            assertTrue(quadric.arePerpendicularPoints(transformPoint1,
                    transformPoint2, PERPENDICULAR_THRESHOLD));
            assertEquals(quadric.arePerpendicularPoints(transformPoint1,
                    transformPoint2, Quadric.DEFAULT_PERPENDICULAR_THRESHOLD),
                    quadric.arePerpendicularPoints(transformPoint1,
                            transformPoint2));

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

            transformPoint1 = new HomogeneousPoint3D(
                    transformPointMatrix1.toArray());
            transformPoint2 = new HomogeneousPoint3D(
                    transformPointMatrix2.toArray());

            if (quadric.arePerpendicularPoints(transformPoint1, transformPoint2,
                    5.0 * PERPENDICULAR_THRESHOLD)) {
                continue;
            }
            assertFalse(quadric.arePerpendicularPoints(transformPoint1,
                    transformPoint2, 5.0 * PERPENDICULAR_THRESHOLD));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testGetDualQuadric() throws WrongSizeException,
            DecomposerException, RankDeficientMatrixException,
            IllegalArgumentException, NonSymmetricMatrixException,
            DualQuadricNotAvailableException {

        Matrix transformMatrix = Matrix.createWithUniformRandomValues(
                HOM_COORDS, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        while (com.irurueta.algebra.Utils.rank(transformMatrix) != 4) {
            transformMatrix = Matrix.createWithUniformRandomValues(
                    HOM_COORDS, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        }

        final Matrix transformTransposedMatrix =
                transformMatrix.transposeAndReturnNew();

        final Matrix quadricMatrix = transformTransposedMatrix.multiplyAndReturnNew(
                transformMatrix);

        final Matrix dualQuadricMatrix = com.irurueta.algebra.Utils.inverse(
                quadricMatrix);

        final Quadric quadric = new Quadric(quadricMatrix);
        final DualQuadric dualQuadric = quadric.getDualQuadric();
        final Matrix dualQuadricMatrix2 = dualQuadric.asMatrix();

        // normalize dual conic matrices
        double norm = com.irurueta.algebra.Utils.normF(dualQuadricMatrix);
        dualQuadricMatrix.multiplyByScalar(1.0 / norm);

        norm = com.irurueta.algebra.Utils.normF(dualQuadricMatrix2);
        dualQuadricMatrix2.multiplyByScalar(1.0 / norm);

        // compute difference of normalized dual conic matrices
        final Matrix diffMatrix = dualQuadricMatrix.subtractAndReturnNew(
                dualQuadricMatrix2);

        // ensure that difference matrix is almost zero by checking its norm
        norm = com.irurueta.algebra.Utils.normF(diffMatrix);
        assertEquals(norm, 0.0, PRECISION_ERROR);
    }

    @Test
    public void testNormalize() throws WrongSizeException,
            IllegalArgumentException, NonSymmetricMatrixException {

        final Matrix t = Matrix.createWithUniformRandomValues(HOM_COORDS, HOM_COORDS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final Matrix transT = t.transposeAndReturnNew();

        // make symmetric matrix
        final Matrix quadricMatrix = transT.multiplyAndReturnNew(t);

        final Quadric quadric = new Quadric(quadricMatrix);
        assertFalse(quadric.isNormalized());

        // normalize quadric
        quadric.normalize();
        assertTrue(quadric.isNormalized());

        // return quadric as matrix
        final Matrix quadricMatrix2 = quadric.asMatrix();

        // compare that both matrices are equal up to scale, for that reason we
        // first normalize both matrices
        double norm = com.irurueta.algebra.Utils.normF(quadricMatrix);
        quadricMatrix.multiplyByScalar(1.0 / norm);

        norm = com.irurueta.algebra.Utils.normF(quadricMatrix2);
        quadricMatrix2.multiplyByScalar(1.0 / norm);

        // compute their difference
        final Matrix diffMatrix = quadricMatrix.subtractAndReturnNew(quadricMatrix2);

        // finally, ensure that the norm of the difference matrix is almost zero
        // up to machine precision
        norm = com.irurueta.algebra.Utils.normF(diffMatrix);
        assertEquals(norm, 0.0, PRECISION_ERROR);

        // check that when setting new values quadric becomes non-normalized
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double value = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        quadric.setA(value);
        assertFalse(quadric.isNormalized());

        quadric.normalize();
        assertTrue(quadric.isNormalized());
        quadric.setB(value);
        assertFalse(quadric.isNormalized());

        quadric.normalize();
        assertTrue(quadric.isNormalized());
        quadric.setC(value);
        assertFalse(quadric.isNormalized());

        quadric.normalize();
        assertTrue(quadric.isNormalized());
        quadric.setD(value);
        assertFalse(quadric.isNormalized());

        quadric.normalize();
        assertTrue(quadric.isNormalized());
        quadric.setE(value);
        assertFalse(quadric.isNormalized());

        quadric.normalize();
        assertTrue(quadric.isNormalized());
        quadric.setF(value);
        assertFalse(quadric.isNormalized());

        quadric.normalize();
        assertTrue(quadric.isNormalized());
        quadric.setG(value);
        assertFalse(quadric.isNormalized());

        quadric.normalize();
        assertTrue(quadric.isNormalized());
        quadric.setH(value);
        assertFalse(quadric.isNormalized());

        quadric.normalize();
        assertTrue(quadric.isNormalized());
        quadric.setI(value);
        assertFalse(quadric.isNormalized());

        quadric.normalize();
        assertTrue(quadric.isNormalized());
        quadric.setJ(value);
        assertFalse(quadric.isNormalized());

        quadric.normalize();
        assertTrue(quadric.isNormalized());
        quadric.setParameters(value, value, value, value, value, value, value,
                value, value, value);
        assertFalse(quadric.isNormalized());

        quadric.normalize();
        assertTrue(quadric.isNormalized());
        quadric.setParameters(quadricMatrix);
        assertFalse(quadric.isNormalized());

        // when setting all values to zero, attempting to normalize has no effect
        quadric.setParameters(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        assertFalse(quadric.isNormalized());
        quadric.normalize();
        assertFalse(quadric.isNormalized());
    }

    @Test
    public void testGetTangentPlaneAt() throws NotLocusException {
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final Point3D center = new InhomogeneousPoint3D(randomizer.nextDouble(
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final double radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE));

            final Sphere sphere = new Sphere(center, radius);
            final Quadric quadric = sphere.toQuadric();

            final double angle1 = randomizer.nextDouble(MIN_RANDOM_DEGREES,
                    MAX_RANDOM_DEGREES) * Math.PI / 180.0;
            final double angle2 = randomizer.nextDouble(MIN_RANDOM_DEGREES,
                    MAX_RANDOM_DEGREES) * Math.PI / 180.0;
            final Point3D point = new HomogeneousPoint3D(
                    center.getInhomX() + radius * Math.cos(angle1) *
                            Math.cos(angle2),
                    center.getInhomY() + radius * Math.sin(angle1) *
                            Math.cos(angle2),
                    center.getInhomZ() + radius * Math.sin(angle2),
                    1.0);
            point.normalize();

            assertTrue(sphere.isLocus(point));
            assertTrue(quadric.isLocus(point));

            // find tangent plane at locus point
            final Plane plane = sphere.getTangentPlaneAt(point);
            final Plane plane2 = quadric.getTangentPlaneAt(point);

            assertTrue(plane.equals(plane2, ABSOLUTE_ERROR));

            // check that point is also at plane's locus
            assertTrue(plane.isLocus(point, ABSOLUTE_ERROR));
            assertTrue(plane2.isLocus(point, ABSOLUTE_ERROR));

            final double[] directorVector = plane.getDirectorVector();
            final double[] directorVector2 = plane2.getDirectorVector();

            final double[] pointVector = new double[]{
                    point.getInhomX() - center.getInhomX(),
                    point.getInhomY() - center.getInhomY(),
                    point.getInhomZ() - center.getInhomZ()
            };

            // normalize both vectors
            final double norm1 = com.irurueta.algebra.Utils.normF(directorVector);
            final double norm1b = com.irurueta.algebra.Utils.normF(directorVector2);
            final double norm2 = com.irurueta.algebra.Utils.normF(pointVector);

            final double[] vector1 = new double[3];
            final double[] vector1b = new double[3];
            final double[] vector2 = new double[3];
            ArrayUtils.multiplyByScalar(directorVector, 1.0 / norm1, vector1);
            ArrayUtils.multiplyByScalar(directorVector2, 1.0 / norm1b, vector1b);
            ArrayUtils.multiplyByScalar(pointVector, 1.0 / norm2, vector2);
            // check that both normalized vectors are equal
            assertArrayEquals(vector1, vector2, ABSOLUTE_ERROR);
            assertArrayEquals(vector1b, vector2, ABSOLUTE_ERROR);
        }
    }

    @Test
    public void testIntersectWithSphereAndXYPlane() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Point3D center = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final double radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE));

        final Sphere sphere = new Sphere(center, radius);
        final Quadric quadric = sphere.toQuadric();

        // create xy plane at z value of sphere center
        final double[] directorVector = new double[]{0.0, 0.0, 1.0};
        ArrayUtils.normalize(directorVector);

        final Plane plane = new Plane(center, directorVector);

        assertTrue(plane.isLocus(center));

        final Conic conic1 = quadric.intersectWith(plane);
        final Conic conic2 = new Conic();
        quadric.intersectWith(plane, conic2);

        assertEquals(conic1.getA(), conic2.getA(), ABSOLUTE_ERROR);
        assertEquals(conic1.getB(), conic2.getB(), ABSOLUTE_ERROR);
        assertEquals(conic1.getC(), conic2.getC(), ABSOLUTE_ERROR);
        assertEquals(conic1.getD(), conic2.getD(), ABSOLUTE_ERROR);
        assertEquals(conic1.getE(), conic2.getE(), ABSOLUTE_ERROR);
        assertEquals(conic1.getF(), conic2.getF(), ABSOLUTE_ERROR);

        assertEquals(conic1.getConicType(), ConicType.CIRCLE_CONIC_TYPE);
        assertEquals(conic2.getConicType(), ConicType.CIRCLE_CONIC_TYPE);

        final Circle circle = new Circle(conic1);

        final Point2D circleCenter = circle.getCenter();
        final double circleRadius = circle.getRadius();

        assertEquals(circleCenter.getInhomX(), center.getInhomX(),
                ABSOLUTE_ERROR);
        assertEquals(circleCenter.getInhomY(), center.getInhomY(),
                ABSOLUTE_ERROR);
        assertEquals(radius, circleRadius, ABSOLUTE_ERROR);
    }

    @Test
    public void testIntersectWithSphereAndAnyPlane()
            throws AlgebraException, GeometryException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Point3D center = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final double radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE));

        final Sphere sphere = new Sphere(center, radius);
        final Quadric quadric = sphere.toQuadric();

        final double[] directorVector = new double[3];
        randomizer.fill(directorVector);
        ArrayUtils.normalize(directorVector);

        final Plane plane = new Plane(center, directorVector);

        assertTrue(plane.isLocus(center));

        final Conic conic1 = quadric.intersectWith(plane);
        final Conic conic2 = new Conic();
        quadric.intersectWith(plane, conic2);

        assertEquals(conic1.getA(), conic2.getA(), ABSOLUTE_ERROR);
        assertEquals(conic1.getB(), conic2.getB(), ABSOLUTE_ERROR);
        assertEquals(conic1.getC(), conic2.getC(), ABSOLUTE_ERROR);
        assertEquals(conic1.getD(), conic2.getD(), ABSOLUTE_ERROR);
        assertEquals(conic1.getE(), conic2.getE(), ABSOLUTE_ERROR);
        assertEquals(conic1.getF(), conic2.getF(), ABSOLUTE_ERROR);

        // conic is not a circle because it is expressed in terms of
        // quadric coordinates, not on plane coordinates
        assertEquals(conic1.getConicType(), ConicType.ELLIPSE_CONIC_TYPE);
        assertEquals(conic2.getConicType(), ConicType.ELLIPSE_CONIC_TYPE);

        // Rotate plane and quadric to make it an xy plane
        final double[] directorVector2 = new double[]{0.0, 0.0, 1.0};
        final double angle = ArrayUtils.angle(directorVector,
                directorVector2);
        final double[] axis = Utils.crossProduct(directorVector,
                directorVector2);

        final AxisRotation3D rotation = new AxisRotation3D(axis, angle);

        // translate quadric and plane to origin
        EuclideanTransformation3D transformation =
                new EuclideanTransformation3D(new double[]{
                        -center.getInhomX(),
                        -center.getInhomY(),
                        -center.getInhomZ()});

        Quadric transformedQuadric =
                transformation.transformAndReturnNew(quadric);
        Plane transformedPlane = transformation.transformAndReturnNew(plane);
        Point3D transformedCenter = transformation.transformAndReturnNew(center);

        // center is at the origin
        assertTrue(transformedCenter.equals(new InhomogeneousPoint3D(),
                ABSOLUTE_ERROR));

        // plane is at the origin
        assertTrue(transformedPlane.isLocus(transformedCenter, ABSOLUTE_ERROR));

        // sphere is at the origin and has same size
        final Sphere transformedSphere = new Sphere(transformedQuadric);

        assertTrue(transformedSphere.getCenter().equals(
                transformedCenter, ABSOLUTE_ERROR));
        assertEquals(transformedSphere.getRadius(), radius, ABSOLUTE_ERROR);

        // rotate plane and quadric
        transformation = new EuclideanTransformation3D(rotation);
        transformedQuadric = transformation.transformAndReturnNew(
                transformedQuadric);
        transformedPlane = transformation.transformAndReturnNew(
                transformedPlane);
        transformedCenter = transformation.transformAndReturnNew(
                transformedCenter);

        // center is at the origin
        assertTrue(transformedCenter.equals(new InhomogeneousPoint3D(),
                ABSOLUTE_ERROR));

        // plane is at the origin and rotated
        transformedPlane.normalize();
        assertTrue(transformedPlane.isLocus(transformedCenter, ABSOLUTE_ERROR));

        double[] transformedDirectorVector = transformedPlane.getDirectorVector();
        assertArrayEquals(transformedDirectorVector, directorVector2, ABSOLUTE_ERROR);

        // because of numerical accuracy, transformed quadric is an ellipsoid rather
        // than an ellipse, but having almost equal semi-axes.

        Conic conic3 = transformedQuadric.intersectWith(transformedPlane);

        // because of numerical accuracy, conic is considered an ellipse, but
        // the ellipse has almost equal semi-minor and semi-major axes
        Ellipse ellipse = new Ellipse(conic3);

        Point2D ellipseCenter = ellipse.getCenter();
        double semiMajorAxis = ellipse.getSemiMajorAxis();
        double semiMinorAxis = ellipse.getSemiMinorAxis();

        // check that conic is centered at origin and keeps radius
        assertEquals(ellipseCenter.getInhomX(), 0.0,
                ABSOLUTE_ERROR);
        assertEquals(ellipseCenter.getInhomY(), 0.0,
                ABSOLUTE_ERROR);
        assertEquals(radius, semiMajorAxis, ABSOLUTE_ERROR);
        assertEquals(radius, semiMinorAxis, ABSOLUTE_ERROR);

        assertTrue(conic3.getConicType() == ConicType.CIRCLE_CONIC_TYPE ||
                conic3.getConicType() == ConicType.ELLIPSE_CONIC_TYPE);

        // transform plane and quadric to their original position (but now they are
        // rotated so that plane points towards z axis)
        transformation = new EuclideanTransformation3D(new double[]{
                center.getInhomX(),
                center.getInhomY(),
                center.getInhomZ()});

        transformedQuadric = transformation.transformAndReturnNew(
                transformedQuadric);
        transformedPlane = transformation.transformAndReturnNew(
                transformedPlane);
        transformedCenter = transformation.transformAndReturnNew(
                transformedCenter);

        // center is at the original position
        assertTrue(transformedCenter.equals(center, ABSOLUTE_ERROR));

        // plane is at the original position and rotated
        transformedPlane.normalize();
        assertTrue(transformedPlane.isLocus(center, ABSOLUTE_ERROR));

        transformedDirectorVector = transformedPlane.getDirectorVector();
        assertArrayEquals(
                ArrayUtils.normalizeAndReturnNew(transformedDirectorVector),
                directorVector2, ABSOLUTE_ERROR);

        // because of numerical accuracy, transformed quadric is an ellipsoid rather
        // than an ellipse, but having almost equal semi-axes.

        conic3 = transformedQuadric.intersectWith(transformedPlane);

        ellipse = new Ellipse(conic3);

        ellipseCenter = ellipse.getCenter();
        semiMajorAxis = ellipse.getSemiMajorAxis();
        semiMinorAxis = ellipse.getSemiMinorAxis();

        // check that conic is centered at origin and keeps radius
        assertEquals(ellipseCenter.getInhomX(), center.getInhomX(),
                ABSOLUTE_ERROR);
        assertEquals(ellipseCenter.getInhomY(), center.getInhomY(),
                ABSOLUTE_ERROR);
        assertEquals(radius, semiMajorAxis, ABSOLUTE_ERROR);
        assertEquals(radius, semiMinorAxis, ABSOLUTE_ERROR);

        assertTrue(conic3.getConicType() == ConicType.CIRCLE_CONIC_TYPE ||
                conic3.getConicType() == ConicType.ELLIPSE_CONIC_TYPE);
    }
}
