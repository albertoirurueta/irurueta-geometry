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
import org.junit.jupiter.api.Test;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

class QuadricTest {

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
            DecomposerException, CoincidentPointsException {

        final var randomizer = new UniformRandomizer();

        // Constructor
        var quadric = new Quadric();
        assertEquals(0.0, quadric.getA(), 0.0);
        assertEquals(0.0, quadric.getB(), 0.0);
        assertEquals(0.0, quadric.getC(), 0.0);
        assertEquals(0.0, quadric.getD(), 0.0);
        assertEquals(0.0, quadric.getE(), 0.0);
        assertEquals(0.0, quadric.getF(), 0.0);
        assertEquals(0.0, quadric.getG(), 0.0);
        assertEquals(0.0, quadric.getH(), 0.0);
        assertEquals(0.0, quadric.getI(), 0.0);
        assertEquals(0.0, quadric.getJ(), 0.0);

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

        quadric = new Quadric(a, b, c, d, e, f, g, h, i, j);
        assertEquals(a, quadric.getA(), 0.0);
        assertEquals(b, quadric.getB(), 0.0);
        assertEquals(c, quadric.getC(), 0.0);
        assertEquals(d, quadric.getD(), 0.0);
        assertEquals(e, quadric.getE(), 0.0);
        assertEquals(f, quadric.getF(), 0.0);
        assertEquals(g, quadric.getG(), 0.0);
        assertEquals(h, quadric.getH(), 0.0);
        assertEquals(i, quadric.getI(), 0.0);
        assertEquals(j, quadric.getJ(), 0.0);
        assertFalse(quadric.isNormalized());

        // Constructor using matrix
        var m = new Matrix(QUADRIC_ROWS, QUADRIC_COLS);
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

        assertEquals(m.getElementAt(0, 0), quadric.getA(), 0.0);
        assertEquals(m.getElementAt(1, 1), quadric.getB(), 0.0);
        assertEquals(m.getElementAt(2, 2), quadric.getC(), 0.0);
        assertEquals(m.getElementAt(1, 0), quadric.getD(), 0.0);
        assertEquals(m.getElementAt(2, 1), quadric.getE(), 0.0);
        assertEquals(m.getElementAt(2, 0),quadric.getF(), 0.0);
        assertEquals(m.getElementAt(3, 0), quadric.getG(), 0.0);
        assertEquals(m.getElementAt(1, 3), quadric.getH(), 0.0);
        assertEquals(m.getElementAt(3, 2), quadric.getI(), 0.0);
        assertEquals(m.getElementAt(3, 3), quadric.getJ(), 0.0);

        // Constructor using matrix with wrong size exception
        final var wrongM1 = new Matrix(QUADRIC_ROWS, QUADRIC_COLS + 1);
        assertThrows(IllegalArgumentException.class, () -> new Quadric(wrongM1));

        // Constructor using non-symmetric matrix
        final var wrongM2 = new Matrix(QUADRIC_ROWS, QUADRIC_COLS);
        wrongM2.setElementAt(0, 0, a);
        wrongM2.setElementAt(1, 1, b);
        wrongM2.setElementAt(2, 2, c);
        wrongM2.setElementAt(3, 3, j);
        wrongM2.setElementAt(1, 0, d);
        wrongM2.setElementAt(0, 1, d + 1.0);
        wrongM2.setElementAt(2, 1, e);
        wrongM2.setElementAt(1, 2, e + 1.0);
        wrongM2.setElementAt(2, 0, f);
        wrongM2.setElementAt(0, 2, f + 1.0);
        wrongM2.setElementAt(3, 0, g);
        wrongM2.setElementAt(0, 3, g + 1.0);
        wrongM2.setElementAt(3, 1, h);
        wrongM2.setElementAt(1, 3, h + 1.0);
        wrongM2.setElementAt(3, 2, i);
        wrongM2.setElementAt(2, 3, i + 1.0);
        assertThrows(NonSymmetricMatrixException.class, () -> new Quadric(wrongM2));

        // Constructor from 9 points
        m = Matrix.createWithUniformRandomValues(9, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

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

        // estimate quadric that lies inside provided 9 points
        var quadricMatrix = new Matrix(9, 10);
        var x = point1.getHomX();
        var y = point1.getHomY();
        var z = point1.getHomZ();
        var w = point1.getHomW();
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
            m = Matrix.createWithUniformRandomValues(9, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

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

            // estimate quadric that lies inside provided 9 points
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

        quadric = new Quadric(point1, point2, point3, point4, point5, point6, point7, point8, point9);
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
        final var finalPoint1 = point1;
        final var finalPoint2 = point2;
        final var finalPoint3 = point3;
        final var finalPoint4 = point4;
        final var finalPoint5 = point5;
        final var finalPoint6 = point6;
        final var finalPoint7 = point7;
        final var finalPoint8 = point8;
        assertThrows(CoincidentPointsException.class, () -> new Quadric(finalPoint1, finalPoint2, finalPoint3,
                finalPoint4, finalPoint5, finalPoint6, finalPoint7, finalPoint8, finalPoint8));
    }

    @Test
    void testGettersAndSetters() throws WrongSizeException, IllegalArgumentException, NonSymmetricMatrixException {
        final var randomizer = new UniformRandomizer();

        final var quadric = new Quadric();
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
        assertEquals(a, quadric.getA(), 0.0);
        assertEquals(b, quadric.getB(), 0.0);
        assertEquals(c, quadric.getC(), 0.0);
        assertEquals(d, quadric.getD(), 0.0);
        assertEquals(e, quadric.getE(), 0.0);
        assertEquals(f, quadric.getF(), 0.0);
        assertEquals(g, quadric.getG(), 0.0);
        assertEquals(h, quadric.getH(), 0.0);
        assertEquals(i, quadric.getI(), 0.0);
        assertEquals(j, quadric.getJ(), 0.0);

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
        assertEquals(a, quadric.getA(), 0.0);
        assertEquals(b, quadric.getB(), 0.0);
        assertEquals(c, quadric.getC(), 0.0);
        assertEquals(d, quadric.getD(), 0.0);
        assertEquals(e, quadric.getE(), 0.0);
        assertEquals(f, quadric.getF(), 0.0);
        assertEquals(g, quadric.getG(), 0.0);
        assertEquals(h, quadric.getH(), 0.0);
        assertEquals(i, quadric.getI(), 0.0);
        assertEquals(j, quadric.getJ(), 0.0);

        var m = new Matrix(QUADRIC_ROWS, QUADRIC_COLS);
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
        assertEquals(m.getElementAt(0, 0), quadric.getA(), 0.0);
        assertEquals(m.getElementAt(1, 1), quadric.getB(), 0.0);
        assertEquals(m.getElementAt(2, 2), quadric.getC(), 0.0);
        assertEquals(m.getElementAt(1, 0), quadric.getD(), 0.0);
        assertEquals(m.getElementAt(2, 1), quadric.getE(), 0.0);
        assertEquals(m.getElementAt(2, 0), quadric.getF(), 0.0);
        assertEquals(m.getElementAt(3, 0), quadric.getG(), 0.0);
        assertEquals(m.getElementAt(3, 1), quadric.getH(), 0.0);
        assertEquals(m.getElementAt(3, 2), quadric.getI(), 0.0);

        // Force IllegalArgumentException
        final var wrongM3 = new Matrix(QUADRIC_ROWS + 1, QUADRIC_COLS + 1);
        assertThrows(IllegalArgumentException.class, () -> quadric.setParameters(wrongM3));

        // Force NonSymmetricMatrixException
        final var wrongM4 = new Matrix(QUADRIC_ROWS, QUADRIC_COLS);
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
        wrongM4.setElementAt(0, 0, a);
        wrongM4.setElementAt(1, 1, b);
        wrongM4.setElementAt(2, 2, c);
        wrongM4.setElementAt(3, 3, j);
        wrongM4.setElementAt(1, 0, d);
        wrongM4.setElementAt(0, 1, d + 1.0);
        wrongM4.setElementAt(2, 1, e);
        wrongM4.setElementAt(1, 2, e + 1.0);
        wrongM4.setElementAt(2, 0, f);
        wrongM4.setElementAt(0, 2, f + 1.0);
        wrongM4.setElementAt(3, 0, g);
        wrongM4.setElementAt(0, 3, g + 1.0);
        wrongM4.setElementAt(3, 1, h);
        wrongM4.setElementAt(1, 3, h + 1.0);
        wrongM4.setElementAt(3, 2, i);
        wrongM4.setElementAt(2, 3, i + 1.0);
        assertThrows(NonSymmetricMatrixException.class, () -> quadric.setParameters(wrongM4));
        assertDoesNotThrow(() -> quadric.setParameters(wrongM4, 1.0));
    }

    @Test
    void testAsMatrix() throws WrongSizeException, IllegalArgumentException, NonSymmetricMatrixException {

        final var randomizer = new UniformRandomizer();
        final var quadric = new Quadric();
        final var m = new Matrix(QUADRIC_ROWS, QUADRIC_COLS);
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
        quadric.setParameters(m);

        final var m2 = quadric.asMatrix();

        assertTrue(m.equals(m2));
    }

    @Test
    void testIsLocus() throws WrongSizeException, DecomposerException, NotReadyException, LockedException,
            com.irurueta.algebra.NotAvailableException, IllegalArgumentException,
            NonSymmetricMatrixException, CoincidentPointsException {

        var m = Matrix.createWithUniformRandomValues(9, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        var point1 = new HomogeneousPoint3D(m.getElementAt(0, 0), m.getElementAt(0, 1),
                m.getElementAt(0, 2), m.getElementAt(0, 3));
        var point2 = new HomogeneousPoint3D(m.getElementAt(1, 0), m.getElementAt(1, 1),
                m.getElementAt(1, 2), m.getElementAt(1, 3));
        var point3 = new HomogeneousPoint3D(m.getElementAt(2, 0), m.getElementAt(2, 1),
                m.getElementAt(2, 2), m.getElementAt(2, 3));
        var point4 = new HomogeneousPoint3D(m.getElementAt(3, 0), m.getElementAt(3, 1),
                m.getElementAt(3, 2), m.getElementAt(3, 3));
        var point5 = new HomogeneousPoint3D(m.getElementAt(4, 0), m.getElementAt(4, 1),
                m.getElementAt(4, 2), m.getElementAt(4, 3));
        var point6 = new HomogeneousPoint3D(m.getElementAt(5, 0), m.getElementAt(5, 1),
                m.getElementAt(5, 2), m.getElementAt(5, 3));
        var point7 = new HomogeneousPoint3D(m.getElementAt(6, 0), m.getElementAt(6, 1),
                m.getElementAt(6, 2), m.getElementAt(6, 3));
        var point8 = new HomogeneousPoint3D(m.getElementAt(7, 0), m.getElementAt(7, 1),
                m.getElementAt(7, 2), m.getElementAt(7, 3));
        var point9 = new HomogeneousPoint3D(m.getElementAt(8, 0), m.getElementAt(8, 1),
                m.getElementAt(8, 2), m.getElementAt(8, 3));

        point1.normalize();
        point2.normalize();
        point3.normalize();
        point4.normalize();
        point5.normalize();
        point6.normalize();
        point7.normalize();
        point8.normalize();
        point9.normalize();

        // estimate quadric that lies inside provided 9 points
        var quadricMatrix = new Matrix(9, 10);
        var x = point1.getHomX();
        var y = point1.getHomY();
        var z = point1.getHomZ();
        var w = point1.getHomW();
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
            m = Matrix.createWithUniformRandomValues(9, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            point1 = new HomogeneousPoint3D(m.getElementAt(0, 0), m.getElementAt(0, 1),
                    m.getElementAt(0, 2), m.getElementAt(0, 3));
            point2 = new HomogeneousPoint3D(m.getElementAt(1, 0), m.getElementAt(1, 1),
                    m.getElementAt(1, 2), m.getElementAt(1, 3));
            point3 = new HomogeneousPoint3D(m.getElementAt(2, 0), m.getElementAt(2, 1),
                    m.getElementAt(2, 2), m.getElementAt(2, 3));
            point4 = new HomogeneousPoint3D(m.getElementAt(3, 0), m.getElementAt(3, 1),
                    m.getElementAt(3, 2), m.getElementAt(3, 3));
            point5 = new HomogeneousPoint3D(m.getElementAt(4, 0), m.getElementAt(4, 1),
                    m.getElementAt(4, 2), m.getElementAt(4, 3));
            point6 = new HomogeneousPoint3D(m.getElementAt(5, 0), m.getElementAt(5, 1),
                    m.getElementAt(5, 2), m.getElementAt(5, 3));
            point7 = new HomogeneousPoint3D(m.getElementAt(6, 0), m.getElementAt(6, 1),
                    m.getElementAt(6, 2), m.getElementAt(6, 3));
            point8 = new HomogeneousPoint3D(m.getElementAt(7, 0), m.getElementAt(7, 1),
                    m.getElementAt(7, 2), m.getElementAt(7, 3));
            point9 = new HomogeneousPoint3D(m.getElementAt(8, 0), m.getElementAt(8, 1),
                    m.getElementAt(8, 2), m.getElementAt(8, 3));

            point1.normalize();
            point2.normalize();
            point3.normalize();
            point4.normalize();
            point5.normalize();
            point6.normalize();
            point7.normalize();
            point8.normalize();
            point9.normalize();

            // estimate quadric that lies inside provided 9 points
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

        final var decomposer = new SingularValueDecomposer(quadricMatrix);
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

        final var quadricPoint = new Matrix(QUADRIC_ROWS, 1);
        quadricPoint.setElementAt(0, 0, point1.getHomX());
        quadricPoint.setElementAt(1, 0, point1.getHomY());
        quadricPoint.setElementAt(2, 0, point1.getHomZ());
        quadricPoint.setElementAt(3, 0, point1.getHomW());

        var norm = com.irurueta.algebra.Utils.normF(quadricPoint);
        quadricPoint.multiplyByScalar(1.0 / norm);

        final var resultQuadricMatrix = new Matrix(QUADRIC_ROWS, QUADRIC_COLS);
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
        final var planeParams = resultQuadricMatrix.multiplyAndReturnNew(quadricPoint);

        norm = com.irurueta.algebra.Utils.normF(planeParams);
        planeParams.multiplyByScalar(1.0 / norm);

        // use director vector of tangent plane to find a point outside quadric
        var directVectorA = planeParams.getElementAt(0, 0);
        var directVectorB = planeParams.getElementAt(1, 0);
        var directVectorC = planeParams.getElementAt(2, 0);
        final var directVectorNorm = Math.sqrt(directVectorA * directVectorA + directVectorB * directVectorB
                + directVectorC * directVectorC);
        directVectorA /= directVectorNorm;
        directVectorB /= directVectorNorm;
        directVectorC /= directVectorNorm;
        final var outsidePoint = new InhomogeneousPoint3D(
                point1.getInhomX() + directVectorA,
                point1.getInhomY() + directVectorB,
                point1.getInhomZ() + directVectorC);

        // instantiate new quadric instance
        final var quadric = new Quadric(resultQuadricMatrix);

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
        final var quadric2 = new Quadric(point1, point2, point3, point4, point5, point6, point7, point8, point9);
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
    void testAngleBetweenPoints() throws WrongSizeException, IllegalArgumentException, NonSymmetricMatrixException {

        // initial 3D points
        final var point1Matrix = Matrix.createWithUniformRandomValues(HOM_COORDS,
                1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var point2Matrix = Matrix.createWithUniformRandomValues(HOM_COORDS,
                1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

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

        // compute quadric matrix as the product of transposed transform matrix
        // with itself
        final var transposedTransform = transform.transposeAndReturnNew();
        final var quadricMatrix = transposedTransform.multiplyAndReturnNew(transform);

        // normalize conic matrix
        final var normQuadric = com.irurueta.algebra.Utils.normF(quadricMatrix);
        quadricMatrix.multiplyByScalar(1.0 / normQuadric);

        final var quadric = new Quadric(quadricMatrix);

        final var point1 = new HomogeneousPoint3D(point1Matrix.getElementAt(0, 0),
                point1Matrix.getElementAt(1, 0),
                point1Matrix.getElementAt(2, 0),
                point1Matrix.getElementAt(3, 0));
        final var point2 = new HomogeneousPoint3D(point2Matrix.getElementAt(0, 0),
                point2Matrix.getElementAt(1, 0),
                point2Matrix.getElementAt(2, 0),
                point2Matrix.getElementAt(3, 0));

        assertEquals(quadric.angleBetweenPoints(point1, point2), angle, PRECISION_ERROR);
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
            matrixPoint2.setElementAt(0, 0, matrixPoint1.getElementAt(1, 0) +
                    matrixPoint1.getElementAt(2, 0) +
                    matrixPoint1.getElementAt(3, 0));
            matrixPoint2.setElementAt(1, 0, -matrixPoint1.getElementAt(0, 0));
            matrixPoint2.setElementAt(2, 0, -matrixPoint1.getElementAt(0, 0));
            matrixPoint2.setElementAt(3, 0, -matrixPoint1.getElementAt(0, 0));

            norm = com.irurueta.algebra.Utils.normF(matrixPoint2);
            matrixPoint2.multiplyByScalar(1.0 / norm);

            var transform = Matrix.createWithUniformRandomValues(HOM_COORDS, HOM_COORDS, MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            while (com.irurueta.algebra.Utils.rank(transform) < 4) {
                transform = Matrix.createWithUniformRandomValues(HOM_COORDS, HOM_COORDS, MIN_RANDOM_VALUE,
                        MAX_RANDOM_VALUE);
            }

            final var invTransform = com.irurueta.algebra.Utils.inverse(transform);

            var transformPointMatrix1 = transform.multiplyAndReturnNew(matrixPoint1);
            var transformPointMatrix2 = transform.multiplyAndReturnNew(matrixPoint2);

            final var transInvTransform = invTransform.transposeAndReturnNew();

            final var quadricMatrix = transInvTransform.multiplyAndReturnNew(invTransform);
            norm = com.irurueta.algebra.Utils.normF(quadricMatrix);
            quadricMatrix.multiplyByScalar(1.0 / norm);

            var transformPoint1 = new HomogeneousPoint3D(transformPointMatrix1.toArray());
            var transformPoint2 = new HomogeneousPoint3D(transformPointMatrix2.toArray());

            final var quadric = new Quadric(quadricMatrix);

            assertTrue(quadric.arePerpendicularPoints(transformPoint1, transformPoint2, PERPENDICULAR_THRESHOLD));
            assertEquals(quadric.arePerpendicularPoints(transformPoint1, transformPoint2,
                    Quadric.DEFAULT_PERPENDICULAR_THRESHOLD),
                    quadric.arePerpendicularPoints(transformPoint1, transformPoint2));

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

            transformPoint1 = new HomogeneousPoint3D(transformPointMatrix1.toArray());
            transformPoint2 = new HomogeneousPoint3D(transformPointMatrix2.toArray());

            if (quadric.arePerpendicularPoints(transformPoint1, transformPoint2,
                    5.0 * PERPENDICULAR_THRESHOLD)) {
                continue;
            }
            assertFalse(quadric.arePerpendicularPoints(transformPoint1, transformPoint2,
                    5.0 * PERPENDICULAR_THRESHOLD));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testGetDualQuadric() throws WrongSizeException, DecomposerException, RankDeficientMatrixException,
            IllegalArgumentException, NonSymmetricMatrixException, DualQuadricNotAvailableException {

        var transformMatrix = Matrix.createWithUniformRandomValues(HOM_COORDS, HOM_COORDS, MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        while (com.irurueta.algebra.Utils.rank(transformMatrix) != 4) {
            transformMatrix = Matrix.createWithUniformRandomValues(HOM_COORDS, HOM_COORDS, MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
        }

        final var transformTransposedMatrix = transformMatrix.transposeAndReturnNew();

        final var quadricMatrix = transformTransposedMatrix.multiplyAndReturnNew(transformMatrix);

        final var dualQuadricMatrix = com.irurueta.algebra.Utils.inverse(quadricMatrix);

        final var quadric = new Quadric(quadricMatrix);
        final var dualQuadric = quadric.getDualQuadric();
        final var dualQuadricMatrix2 = dualQuadric.asMatrix();

        // normalize dual conic matrices
        var norm = com.irurueta.algebra.Utils.normF(dualQuadricMatrix);
        dualQuadricMatrix.multiplyByScalar(1.0 / norm);

        norm = com.irurueta.algebra.Utils.normF(dualQuadricMatrix2);
        dualQuadricMatrix2.multiplyByScalar(1.0 / norm);

        // compute difference of normalized dual conic matrices
        final var diffMatrix = dualQuadricMatrix.subtractAndReturnNew(dualQuadricMatrix2);

        // ensure that difference matrix is almost zero by checking its norm
        norm = com.irurueta.algebra.Utils.normF(diffMatrix);
        assertEquals(0.0, norm, PRECISION_ERROR);
    }

    @Test
    void testSetParametersFromPoints() throws WrongSizeException, CoincidentPointsException {
        var m = Matrix.createWithUniformRandomValues(9, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        var point1 = new HomogeneousPoint3D(m.getElementAt(0, 0),
                m.getElementAt(0, 1), m.getElementAt(0, 2), 1.0);
        var point2 = new HomogeneousPoint3D(m.getElementAt(1, 0),
                m.getElementAt(1, 1), m.getElementAt(1, 2), 1.0);
        var point3 = new HomogeneousPoint3D(m.getElementAt(2, 0),
                m.getElementAt(2, 1), m.getElementAt(2, 2), 1.0);
        var point4 = new HomogeneousPoint3D(m.getElementAt(3, 0),
                m.getElementAt(3, 1), m.getElementAt(3, 2), 1.0);
        var point5 = new HomogeneousPoint3D(m.getElementAt(4, 0),
                m.getElementAt(4, 1), m.getElementAt(4, 2), 1.0);
        var point6 = new HomogeneousPoint3D(m.getElementAt(5, 0),
                m.getElementAt(5, 1), m.getElementAt(5, 2), 1.0);
        var point7 = new HomogeneousPoint3D(m.getElementAt(6, 0),
                m.getElementAt(6, 1), m.getElementAt(6, 2), 1.0);
        var point8 = new HomogeneousPoint3D(m.getElementAt(7, 0),
                m.getElementAt(7, 1), m.getElementAt(7, 2), 1.0);
        var point9 = new HomogeneousPoint3D(m.getElementAt(8, 0),
                m.getElementAt(8, 1), m.getElementAt(8, 2), 1.0);

        final var quadric = new Quadric();
        quadric.setParametersFromPoints(point1, point2, point3, point4, point5, point6, point7, point8, point9);

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
        assertThrows(CoincidentPointsException.class,
                () -> quadric.setParametersFromPoints(point1, point2, point3, point4, point5, point6, point7, point8,
                        point8));
    }

    @Test
    void testNormalize() throws WrongSizeException, IllegalArgumentException, NonSymmetricMatrixException {

        final var t = Matrix.createWithUniformRandomValues(HOM_COORDS, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var transT = t.transposeAndReturnNew();

        // make symmetric matrix
        final var quadricMatrix = transT.multiplyAndReturnNew(t);

        final var quadric = new Quadric(quadricMatrix);
        assertFalse(quadric.isNormalized());

        // normalize quadric
        quadric.normalize();
        assertTrue(quadric.isNormalized());

        // return quadric as matrix
        final var quadricMatrix2 = quadric.asMatrix();

        // compare that both matrices are equal up to scale, for that reason we
        // first normalize both matrices
        var norm = com.irurueta.algebra.Utils.normF(quadricMatrix);
        quadricMatrix.multiplyByScalar(1.0 / norm);

        norm = com.irurueta.algebra.Utils.normF(quadricMatrix2);
        quadricMatrix2.multiplyByScalar(1.0 / norm);

        // compute their difference
        final var diffMatrix = quadricMatrix.subtractAndReturnNew(quadricMatrix2);

        // finally, ensure that the norm of the difference matrix is almost zero
        // up to machine precision
        norm = com.irurueta.algebra.Utils.normF(diffMatrix);
        assertEquals(0.0, norm, PRECISION_ERROR);

        // check that when setting new values quadric becomes non-normalized
        final var randomizer = new UniformRandomizer();
        final var value = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
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
        quadric.setParameters(value, value, value, value, value, value, value, value, value, value);
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
    void testGetTangentPlaneAt() throws NotLocusException {
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var center = new InhomogeneousPoint3D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final var radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

            final var sphere = new Sphere(center, radius);
            final var quadric = sphere.toQuadric();

            final var angle1 = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
            final var angle2 = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
            final var point = new HomogeneousPoint3D(
                    center.getInhomX() + radius * Math.cos(angle1) * Math.cos(angle2),
                    center.getInhomY() + radius * Math.sin(angle1) * Math.cos(angle2),
                    center.getInhomZ() + radius * Math.sin(angle2),
                    1.0);
            point.normalize();

            assertTrue(sphere.isLocus(point));
            assertTrue(quadric.isLocus(point));

            // find tangent plane at locus point
            final var plane = sphere.getTangentPlaneAt(point);
            final var plane2 = quadric.getTangentPlaneAt(point);
            final var plane3 = new Plane();
            quadric.tangentPlaneAt(point, plane3, ABSOLUTE_ERROR);
            final var plane4 = new Plane();
            quadric.tangentPlaneAt(point, plane4, ABSOLUTE_ERROR);

            assertTrue(plane.equals(plane2, ABSOLUTE_ERROR));
            assertTrue(plane.equals(plane3, ABSOLUTE_ERROR));
            assertTrue(plane.equals(plane4, ABSOLUTE_ERROR));

            // check that point is also at plane's locus
            assertTrue(plane.isLocus(point, ABSOLUTE_ERROR));
            assertTrue(plane2.isLocus(point, ABSOLUTE_ERROR));

            final var directorVector = plane.getDirectorVector();
            final var directorVector2 = plane2.getDirectorVector();

            final var pointVector = new double[]{
                    point.getInhomX() - center.getInhomX(),
                    point.getInhomY() - center.getInhomY(),
                    point.getInhomZ() - center.getInhomZ()
            };

            // normalize both vectors
            final var norm1 = com.irurueta.algebra.Utils.normF(directorVector);
            final var norm1b = com.irurueta.algebra.Utils.normF(directorVector2);
            final var norm2 = com.irurueta.algebra.Utils.normF(pointVector);

            final var vector1 = new double[3];
            final var vector1b = new double[3];
            final var vector2 = new double[3];
            ArrayUtils.multiplyByScalar(directorVector, 1.0 / norm1, vector1);
            ArrayUtils.multiplyByScalar(directorVector2, 1.0 / norm1b, vector1b);
            ArrayUtils.multiplyByScalar(pointVector, 1.0 / norm2, vector2);
            // check that both normalized vectors are equal
            assertArrayEquals(vector1, vector2, ABSOLUTE_ERROR);
            assertArrayEquals(vector1b, vector2, ABSOLUTE_ERROR);
        }
    }

    @Test
    void testIntersectWithSphereAndXYPlane() {
        final var randomizer = new UniformRandomizer();
        final var center = new InhomogeneousPoint3D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final var sphere = new Sphere(center, radius);
        final var quadric = sphere.toQuadric();

        // create xy plane at z value of sphere center
        final var directorVector = new double[]{0.0, 0.0, 1.0};
        ArrayUtils.normalize(directorVector);

        final var plane = new Plane(center, directorVector);

        assertTrue(plane.isLocus(center));

        final var conic1 = quadric.intersectWith(plane);
        final var conic2 = new Conic();
        quadric.intersectWith(plane, conic2);

        assertEquals(conic1.getA(), conic2.getA(), ABSOLUTE_ERROR);
        assertEquals(conic1.getB(), conic2.getB(), ABSOLUTE_ERROR);
        assertEquals(conic1.getC(), conic2.getC(), ABSOLUTE_ERROR);
        assertEquals(conic1.getD(), conic2.getD(), ABSOLUTE_ERROR);
        assertEquals(conic1.getE(), conic2.getE(), ABSOLUTE_ERROR);
        assertEquals(conic1.getF(), conic2.getF(), ABSOLUTE_ERROR);

        assertEquals(ConicType.CIRCLE_CONIC_TYPE, conic1.getConicType());
        assertEquals(ConicType.CIRCLE_CONIC_TYPE, conic2.getConicType());

        final var circle = new Circle(conic1);

        final var circleCenter = circle.getCenter();
        final var circleRadius = circle.getRadius();

        assertEquals(circleCenter.getInhomX(), center.getInhomX(), ABSOLUTE_ERROR);
        assertEquals(circleCenter.getInhomY(), center.getInhomY(), ABSOLUTE_ERROR);
        assertEquals(radius, circleRadius, ABSOLUTE_ERROR);
    }

    @Test
    void testIntersectWithSphereAndAnyPlane() throws AlgebraException, GeometryException {
        final var randomizer = new UniformRandomizer();
        final var center = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final var sphere = new Sphere(center, radius);
        final var quadric = sphere.toQuadric();

        final var directorVector = new double[3];
        randomizer.fill(directorVector);
        ArrayUtils.normalize(directorVector);

        final var plane = new Plane(center, directorVector);

        assertTrue(plane.isLocus(center));

        final var conic1 = quadric.intersectWith(plane);
        final var conic2 = new Conic();
        quadric.intersectWith(plane, conic2);

        assertEquals(conic1.getA(), conic2.getA(), ABSOLUTE_ERROR);
        assertEquals(conic1.getB(), conic2.getB(), ABSOLUTE_ERROR);
        assertEquals(conic1.getC(), conic2.getC(), ABSOLUTE_ERROR);
        assertEquals(conic1.getD(), conic2.getD(), ABSOLUTE_ERROR);
        assertEquals(conic1.getE(), conic2.getE(), ABSOLUTE_ERROR);
        assertEquals(conic1.getF(), conic2.getF(), ABSOLUTE_ERROR);

        // conic is not a circle because it is expressed in terms of
        // quadric coordinates, not on plane coordinates
        assertEquals(ConicType.ELLIPSE_CONIC_TYPE, conic1.getConicType());
        assertEquals(ConicType.ELLIPSE_CONIC_TYPE, conic2.getConicType());

        // Rotate plane and quadric to make it an xy-plane
        final var directorVector2 = new double[]{0.0, 0.0, 1.0};
        final var angle = ArrayUtils.angle(directorVector, directorVector2);
        final var axis = Utils.crossProduct(directorVector, directorVector2);

        final var rotation = new AxisRotation3D(axis, angle);

        // translate quadric and plane to origin
        var transformation = new EuclideanTransformation3D(new double[]{
                        -center.getInhomX(),
                        -center.getInhomY(),
                        -center.getInhomZ()});

        var transformedQuadric = transformation.transformAndReturnNew(quadric);
        var transformedPlane = transformation.transformAndReturnNew(plane);
        var transformedCenter = transformation.transformAndReturnNew(center);

        // center is at the origin
        assertTrue(transformedCenter.equals(new InhomogeneousPoint3D(), ABSOLUTE_ERROR));

        // plane is at the origin
        assertTrue(transformedPlane.isLocus(transformedCenter, ABSOLUTE_ERROR));

        // sphere is at the origin and has same size
        final var transformedSphere = new Sphere(transformedQuadric);

        assertTrue(transformedSphere.getCenter().equals(transformedCenter, ABSOLUTE_ERROR));
        assertEquals(transformedSphere.getRadius(), radius, ABSOLUTE_ERROR);

        // rotate plane and quadric
        transformation = new EuclideanTransformation3D(rotation);
        transformedQuadric = transformation.transformAndReturnNew(transformedQuadric);
        transformedPlane = transformation.transformAndReturnNew(transformedPlane);
        transformedCenter = transformation.transformAndReturnNew(transformedCenter);

        // center is at the origin
        assertTrue(transformedCenter.equals(new InhomogeneousPoint3D(), ABSOLUTE_ERROR));

        // plane is at the origin and rotated
        transformedPlane.normalize();
        assertTrue(transformedPlane.isLocus(transformedCenter, ABSOLUTE_ERROR));

        var transformedDirectorVector = transformedPlane.getDirectorVector();
        assertArrayEquals(transformedDirectorVector, directorVector2, ABSOLUTE_ERROR);

        // because of numerical accuracy, transformed quadric is an ellipsoid rather
        // than an ellipse, but having almost equal semi-axes.

        var conic3 = transformedQuadric.intersectWith(transformedPlane);

        // because of numerical accuracy, conic is considered an ellipse, but
        // the ellipse has almost equal semi-minor and semi-major axes
        var ellipse = new Ellipse(conic3);

        var ellipseCenter = ellipse.getCenter();
        var semiMajorAxis = ellipse.getSemiMajorAxis();
        var semiMinorAxis = ellipse.getSemiMinorAxis();

        // check that conic is centered at origin and keeps radius
        assertEquals(0.0, ellipseCenter.getInhomX(), ABSOLUTE_ERROR);
        assertEquals(0.0, ellipseCenter.getInhomY(), ABSOLUTE_ERROR);
        assertEquals(radius, semiMajorAxis, ABSOLUTE_ERROR);
        assertEquals(radius, semiMinorAxis, ABSOLUTE_ERROR);

        assertTrue(conic3.getConicType() == ConicType.CIRCLE_CONIC_TYPE
                || conic3.getConicType() == ConicType.ELLIPSE_CONIC_TYPE);

        // transform plane and quadric to their original position (but now they are
        // rotated so that plane points towards z axis)
        transformation = new EuclideanTransformation3D(new double[]{
                center.getInhomX(),
                center.getInhomY(),
                center.getInhomZ()});

        transformedQuadric = transformation.transformAndReturnNew(transformedQuadric);
        transformedPlane = transformation.transformAndReturnNew(transformedPlane);
        transformedCenter = transformation.transformAndReturnNew(transformedCenter);

        // center is at the original position
        assertTrue(transformedCenter.equals(center, ABSOLUTE_ERROR));

        // plane is at the original position and rotated
        transformedPlane.normalize();
        assertTrue(transformedPlane.isLocus(center, ABSOLUTE_ERROR));

        transformedDirectorVector = transformedPlane.getDirectorVector();
        assertArrayEquals(ArrayUtils.normalizeAndReturnNew(transformedDirectorVector), directorVector2, ABSOLUTE_ERROR);

        // because of numerical accuracy, transformed quadric is an ellipsoid rather
        // than an ellipse, but having almost equal semi-axes.

        conic3 = transformedQuadric.intersectWith(transformedPlane);

        ellipse = new Ellipse(conic3);

        ellipseCenter = ellipse.getCenter();
        semiMajorAxis = ellipse.getSemiMajorAxis();
        semiMinorAxis = ellipse.getSemiMinorAxis();

        // check that conic is centered at origin and keeps radius
        assertEquals(ellipseCenter.getInhomX(), center.getInhomX(), ABSOLUTE_ERROR);
        assertEquals(ellipseCenter.getInhomY(), center.getInhomY(), ABSOLUTE_ERROR);
        assertEquals(radius, semiMajorAxis, ABSOLUTE_ERROR);
        assertEquals(radius, semiMinorAxis, ABSOLUTE_ERROR);

        assertTrue(conic3.getConicType() == ConicType.CIRCLE_CONIC_TYPE
                || conic3.getConicType() == ConicType.ELLIPSE_CONIC_TYPE);
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
        final var quadric1 = new Quadric(a, b, c, d, e, f, g, h, i, j);

        // check
        assertEquals(a, quadric1.getA(), 0.0);
        assertEquals(b, quadric1.getB(), 0.0);
        assertEquals(c, quadric1.getC(), 0.0);
        assertEquals(d, quadric1.getD(), 0.0);
        assertEquals(e, quadric1.getE(), 0.0);
        assertEquals(f, quadric1.getF(), 0.0);
        assertEquals(g, quadric1.getG(), 0.0);
        assertEquals(h, quadric1.getH(), 0.0);
        assertEquals(i, quadric1.getI(), 0.0);
        assertEquals(j, quadric1.getJ(), 0.0);
        assertFalse(quadric1.isNormalized());

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(quadric1);
        final var quadric2 = SerializationHelper.<Quadric>deserialize(bytes);

        // check
        assertEquals(quadric1.getA(), quadric2.getA(), 0.0);
        assertEquals(quadric1.getB(), quadric2.getB(), 0.0);
        assertEquals(quadric1.getC(), quadric2.getC(), 0.0);
        assertEquals(quadric1.getD(), quadric2.getD(), 0.0);
        assertEquals(quadric1.getE(), quadric2.getE(), 0.0);
        assertEquals(quadric1.getF(), quadric2.getF(), 0.0);
        assertEquals(quadric1.getG(), quadric2.getG(), 0.0);
        assertEquals(quadric1.getH(), quadric2.getH(), 0.0);
        assertEquals(quadric1.getI(), quadric2.getI(), 0.0);
        assertEquals(quadric1.getJ(), quadric2.getJ(), 0.0);
        assertEquals(quadric1.isNormalized(), quadric2.isNormalized());
    }
}
