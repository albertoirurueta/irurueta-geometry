/*
 * Copyright (C) 2015 Alberto Irurueta Carro (alberto@irurueta.com)
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

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.ArrayUtils;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.SingularValueDecomposer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.io.IOException;
import java.util.Random;

import static org.junit.Assert.*;

public class QuaternionTest {
    private static final int ROTATION_COLS = 3;
    private static final int INHOM_COORDS = 3;

    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    private static final double MIN_ANGLE_DEGREES = 0.0;
    private static final double MAX_ANGLE_DEGREES = 90.0;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final double JACOBIAN_ERROR = 1e-6;

    @Test
    public void testConstants() {
        assertEquals(4, Quaternion.N_PARAMS);
        assertEquals(3, Quaternion.N_ANGLES);
        assertEquals(1e-7, Quaternion.AXIS_NORM_THRESHOLD, 0.0);
        assertEquals(1e-6, Quaternion.LARGE_AXIS_NORM_THRESHOLD, 0.0);
        assertEquals(1e-8, Quaternion.TRACE_THRESHOLD, 0.0);
    }

    @Test
    public void testConstructors() throws AlgebraException, RotationException {
        // empty constructor
        Quaternion q = new Quaternion();

        // check correctness
        assertEquals(q.getA(), 1.0, 0.0);
        assertEquals(q.getB(), 0.0, 0.0);
        assertEquals(q.getC(), 0.0, 0.0);
        assertEquals(q.getD(), 0.0, 0.0);
        assertEquals(q.toMatrixRotation().getInternalMatrix(),
                Matrix.identity(MatrixRotation3D.INHOM_COORDS,
                        MatrixRotation3D.INHOM_COORDS));

        // constructor with values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        q = new Quaternion(a, b, c, d);

        // check correctness
        assertEquals(q.getA(), a, 0.0);
        assertEquals(q.getB(), b, 0.0);
        assertEquals(q.getC(), c, 0.0);
        assertEquals(q.getD(), d, 0.0);

        // constructor from another quaternion
        Quaternion q2 = new Quaternion(q);

        // check correctness
        assertEquals(q2.getA(), a, 0.0);
        assertEquals(q2.getB(), b, 0.0);
        assertEquals(q2.getC(), c, 0.0);
        assertEquals(q2.getD(), d, 0.0);

        // constructor from values
        double[] values = new double[]{a, b, c, d};
        q = new Quaternion(values);

        // check correctness
        assertEquals(q.getA(), a, 0.0);
        assertEquals(q.getB(), b, 0.0);
        assertEquals(q.getC(), c, 0.0);
        assertEquals(q.getD(), d, 0.0);

        // Force IllegalArgumentException
        q = null;
        values = new double[]{1, a, b, c, d};
        try {
            q = new Quaternion(values);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(q);

        // constructor from axis and rotation angle
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        // Find any 3 orthogonal vectors, 1st will be axis of rotation, and
        // the remaining two will lie on the rotation plane and will be used
        // to test theta angle

        // To find 3 orthogonal vectors, we use V matrix of a singular
        // decomposition of any Nx3 matrix
        final Matrix aM = Matrix.createWithUniformRandomValues(1, ROTATION_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final SingularValueDecomposer decomposer = new SingularValueDecomposer(aM);

        decomposer.decompose();

        final Matrix vMatrix = decomposer.getV();

        // axis of rotation
        final double[] axis = vMatrix.getSubmatrixAsArray(0, 0,
                2, 0);

        final AxisRotation3D axisRotation = new AxisRotation3D(axis, theta);

        q = new Quaternion(axis, theta);

        // check correctness
        final double[] axis2 = q.getRotationAxis();
        final double theta2 = q.getRotationAngle();
        assertArrayEquals(axis, axis2, ABSOLUTE_ERROR);
        assertEquals(theta, theta2, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        q = null;
        try {
            q = new Quaternion(new double[]{}, theta);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(q);

        // constructor from an axis rotation
        q = new Quaternion(axisRotation);

        // check correctness
        //noinspection AssertBetweenInconvertibleTypes
        assertEquals(q, axisRotation);

        // constructor from roll, pitch and yaw angles
        final double roll = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double yaw = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        q = new Quaternion(roll, pitch, yaw);
        MatrixRotation3D matrixRotation = new MatrixRotation3D();
        matrixRotation.setRollPitchYaw(roll, pitch, yaw);

        q2 = matrixRotation.toQuaternion();

        assertEquals(q, q2);

        final double[] angles = new double[Quaternion.N_ANGLES];
        q.toEulerAngles(angles);

        // check correctness
        //noinspection AssertBetweenInconvertibleTypes
        assertEquals(q, matrixRotation);
        assertEquals(roll, angles[0], ABSOLUTE_ERROR);
        assertEquals(pitch, angles[1], ABSOLUTE_ERROR);
        assertEquals(yaw, angles[2], ABSOLUTE_ERROR);

        // constructor from matrix rotation
        matrixRotation = new MatrixRotation3D(axis, theta);
        q = new Quaternion(matrixRotation);

        // check correctness
        //noinspection AssertBetweenInconvertibleTypes
        assertEquals(q, matrixRotation);
    }

    @Test
    public void testGetSetA() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final Quaternion q = new Quaternion();

        // check initial value
        assertEquals(q.getA(), 1.0, 0.0);

        // set new values
        q.setA(a);

        // check correctness
        assertEquals(q.getA(), a, 0.0);
    }

    @Test
    public void testGetSetB() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final Quaternion q = new Quaternion();

        // check initial value
        assertEquals(q.getB(), 0.0, 0.0);

        // set new value
        q.setB(b);

        // check correctness
        assertEquals(q.getB(), b, 0.0);
    }

    @Test
    public void testGetSetC() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final Quaternion q = new Quaternion();

        // check initial value
        assertEquals(q.getC(), 0.0, 0.0);

        // set new value
        q.setC(c);

        // check correctness
        assertEquals(q.getC(), c, 0.0);
    }

    @Test
    public void testGetSetD() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final Quaternion q = new Quaternion();

        // check initial value
        assertEquals(q.getD(), 0.0, 0.0);

        // set new value
        q.setD(d);

        // check correctness
        assertEquals(q.getD(), d, 0.0);
    }

    @Test
    public void testGetSetValues() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] values = new double[Quaternion.N_PARAMS];
        randomizer.fill(values, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final Quaternion q = new Quaternion();

        // set values
        q.setValues(values);

        // check correctness
        final double[] values2 = new double[Quaternion.N_PARAMS];
        final double[] values3 = q.getValues();
        q.values(values2);

        assertArrayEquals(values, values2, 0.0);
        assertArrayEquals(values, values3, 0.0);

        // Force IllegalArgumentException
        final double[] invalid = new double[Quaternion.N_PARAMS + 1];
        try {
            q.setValues(invalid);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFromQuaternion() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] values = new double[Quaternion.N_PARAMS];
        randomizer.fill(values, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final Quaternion q = new Quaternion(values);

        final Quaternion q2 = new Quaternion();
        q2.fromQuaternion(q);

        // check correctness
        assertArrayEquals(values, q.getValues(), 0.0);
        assertArrayEquals(values, q2.getValues(), 0.0);
        assertArrayEquals(q.getValues(), q2.getValues(), 0.0);
        assertEquals(q, q2);
    }

    @Test
    public void testClone() throws CloneNotSupportedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] values = new double[Quaternion.N_PARAMS];
        randomizer.fill(values, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final Quaternion q = new Quaternion(values);

        final Quaternion q2 = q.clone();

        // check correctness
        assertArrayEquals(values, q.getValues(), 0.0);
        assertArrayEquals(values, q2.getValues(), 0.0);
        assertArrayEquals(q.getValues(), q2.getValues(), 0.0);
        assertEquals(q, q2);
    }

    @Test
    public void testCopyTo() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] values = new double[Quaternion.N_PARAMS];
        randomizer.fill(values, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final Quaternion q = new Quaternion(values);

        final Quaternion q2 = new Quaternion();
        q.copyTo(q2);

        // check correctness
        assertArrayEquals(values, q.getValues(), 0.0);
        assertArrayEquals(values, q2.getValues(), 0.0);
        assertArrayEquals(q.getValues(), q2.getValues(), 0.0);
        assertEquals(q, q2);
    }

    @Test
    public void testGetSetFromAxisAndRotation() throws AlgebraException,
            RotationException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        // Find any 3 orthogonal vectors, 1st will be axis of rotation, and
        // the remaining two will lie on the rotation plane and will be used
        // to test theta angle

        // To find 3 orthogonal vectors, we use V matrix of a singular
        // decomposition of any Nx3 matrix
        final Matrix aM = Matrix.createWithUniformRandomValues(1, ROTATION_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final SingularValueDecomposer decomposer = new SingularValueDecomposer(aM);

        decomposer.decompose();

        final Matrix vMatrix = decomposer.getV();

        // axis of rotation
        final double[] axis = vMatrix.getSubmatrixAsArray(0, 0,
                2, 0);

        final AxisRotation3D axisRotation = new AxisRotation3D(axis, theta);

        Quaternion q = new Quaternion();
        // set values
        q.setFromAxisAndRotation(axis[0], axis[1], axis[2], theta);

        // check correctness
        assertArrayEquals(axis, q.getRotationAxis(), ABSOLUTE_ERROR);
        assertEquals(theta, q.getRotationAngle(), ABSOLUTE_ERROR);

        // set values with jacobians
        Matrix jacobianOfTheta = new Matrix(Quaternion.N_PARAMS, 1);
        Matrix jacobianOfAxis = new Matrix(Quaternion.N_PARAMS,
                Quaternion.N_ANGLES);

        q.setFromAxisAndRotation(axis[0], axis[1], axis[2], theta,
                jacobianOfTheta, jacobianOfAxis);

        // check correctness
        final double halfTheta = theta / 2.0;
        final double c = Math.cos(halfTheta);
        final double s = Math.sin(halfTheta);
        final double halfC = c / 2.0;
        final double halfS = s / 2.0;

        assertEquals(jacobianOfTheta.getElementAtIndex(0), -halfS,
                ABSOLUTE_ERROR);
        assertEquals(jacobianOfTheta.getElementAtIndex(1), axis[0] * halfC,
                ABSOLUTE_ERROR);
        assertEquals(jacobianOfTheta.getElementAtIndex(2), axis[1] * halfC,
                ABSOLUTE_ERROR);
        assertEquals(jacobianOfTheta.getElementAtIndex(3), axis[2] * halfC,
                ABSOLUTE_ERROR);

        assertEquals(jacobianOfAxis.getElementAt(0, 0), 0.0, 0.0);
        assertEquals(jacobianOfAxis.getElementAt(1, 0), s, 0.0);
        assertEquals(jacobianOfAxis.getElementAt(2, 0), 0.0, 0.0);
        assertEquals(jacobianOfAxis.getElementAt(3, 0), 0.0, 0.0);

        assertEquals(jacobianOfAxis.getElementAt(0, 1), 0.0, 0.0);
        assertEquals(jacobianOfAxis.getElementAt(1, 1), 0.0, 0.0);
        assertEquals(jacobianOfAxis.getElementAt(2, 1), s, 0.0);
        assertEquals(jacobianOfAxis.getElementAt(3, 0), 0.0, 0.0);

        assertEquals(jacobianOfAxis.getElementAt(0, 2), 0.0, 0.0);
        assertEquals(jacobianOfAxis.getElementAt(1, 2), 0.0, 0.0);
        assertEquals(jacobianOfAxis.getElementAt(2, 2), 0.0, 0.0);
        assertEquals(jacobianOfAxis.getElementAt(3, 2), s, 0.0);

        // Force IllegalArgumentException
        final Matrix invalid = new Matrix(1, 1);
        try {
            q.setFromAxisAndRotation(axis[0], axis[1], axis[2], theta,
                    invalid, jacobianOfAxis);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        try {
            q.setFromAxisAndRotation(axis[0], axis[1], axis[2], theta,
                    jacobianOfTheta, invalid);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        // set values
        q = new Quaternion();
        q.setFromAxisAndRotation(axis, theta);

        // check correctness
        assertArrayEquals(axis, q.getRotationAxis(), ABSOLUTE_ERROR);
        assertEquals(theta, q.getRotationAngle(), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            q.setFromAxisAndRotation(new double[1], theta);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        // set values with jacobians
        jacobianOfTheta = new Matrix(Quaternion.N_PARAMS, 1);
        jacobianOfAxis = new Matrix(Quaternion.N_PARAMS, Quaternion.N_ANGLES);

        q.setFromAxisAndRotation(axis, theta, jacobianOfTheta, jacobianOfAxis);

        // check correctness
        assertEquals(jacobianOfTheta.getElementAtIndex(0), -halfS,
                ABSOLUTE_ERROR);
        assertEquals(jacobianOfTheta.getElementAtIndex(1), axis[0] * halfC,
                ABSOLUTE_ERROR);
        assertEquals(jacobianOfTheta.getElementAtIndex(2), axis[1] * halfC,
                ABSOLUTE_ERROR);
        assertEquals(jacobianOfTheta.getElementAtIndex(3), axis[2] * halfC,
                ABSOLUTE_ERROR);

        assertEquals(jacobianOfAxis.getElementAt(0, 0), 0.0, 0.0);
        assertEquals(jacobianOfAxis.getElementAt(1, 0), s, 0.0);
        assertEquals(jacobianOfAxis.getElementAt(2, 0), 0.0, 0.0);
        assertEquals(jacobianOfAxis.getElementAt(3, 0), 0.0, 0.0);

        assertEquals(jacobianOfAxis.getElementAt(0, 1), 0.0, 0.0);
        assertEquals(jacobianOfAxis.getElementAt(1, 1), 0.0, 0.0);
        assertEquals(jacobianOfAxis.getElementAt(2, 1), s, 0.0);
        assertEquals(jacobianOfAxis.getElementAt(3, 0), 0.0, 0.0);

        assertEquals(jacobianOfAxis.getElementAt(0, 2), 0.0, 0.0);
        assertEquals(jacobianOfAxis.getElementAt(1, 2), 0.0, 0.0);
        assertEquals(jacobianOfAxis.getElementAt(2, 2), 0.0, 0.0);
        assertEquals(jacobianOfAxis.getElementAt(3, 2), s, 0.0);

        // Force IllegalArgumentException
        try {
            q.setFromAxisAndRotation(new double[1], theta, jacobianOfTheta,
                    jacobianOfAxis);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            q.setFromAxisAndRotation(axis, theta, invalid, jacobianOfAxis);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        try {
            q.setFromAxisAndRotation(axis, theta, jacobianOfTheta, invalid);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        // set values
        q = new Quaternion();
        q.setFromAxisAndRotation(axisRotation);

        // check correctness
        assertArrayEquals(axis, q.getRotationAxis(), ABSOLUTE_ERROR);
        assertEquals(theta, q.getRotationAngle(), ABSOLUTE_ERROR);

        // set values with jacobians
        jacobianOfTheta = new Matrix(Quaternion.N_PARAMS, 1);
        jacobianOfAxis = new Matrix(Quaternion.N_PARAMS, Quaternion.N_ANGLES);

        q = new Quaternion();
        q.setFromAxisAndRotation(axisRotation, jacobianOfTheta, jacobianOfAxis);

        // check correctness
        assertArrayEquals(axis, q.getRotationAxis(), ABSOLUTE_ERROR);
        assertEquals(theta, q.getRotationAngle(), ABSOLUTE_ERROR);

        assertEquals(jacobianOfTheta.getElementAtIndex(0), -halfS,
                ABSOLUTE_ERROR);
        assertEquals(jacobianOfTheta.getElementAtIndex(1), axis[0] * halfC,
                ABSOLUTE_ERROR);
        assertEquals(jacobianOfTheta.getElementAtIndex(2), axis[1] * halfC,
                ABSOLUTE_ERROR);
        assertEquals(jacobianOfTheta.getElementAtIndex(3), axis[2] * halfC,
                ABSOLUTE_ERROR);

        assertEquals(jacobianOfAxis.getElementAt(0, 0), 0.0, 0.0);
        assertEquals(jacobianOfAxis.getElementAt(1, 0), s, 0.0);
        assertEquals(jacobianOfAxis.getElementAt(2, 0), 0.0, 0.0);
        assertEquals(jacobianOfAxis.getElementAt(3, 0), 0.0, 0.0);

        assertEquals(jacobianOfAxis.getElementAt(0, 1), 0.0, 0.0);
        assertEquals(jacobianOfAxis.getElementAt(1, 1), 0.0, 0.0);
        assertEquals(jacobianOfAxis.getElementAt(2, 1), s, 0.0);
        assertEquals(jacobianOfAxis.getElementAt(3, 0), 0.0, 0.0);

        assertEquals(jacobianOfAxis.getElementAt(0, 2), 0.0, 0.0);
        assertEquals(jacobianOfAxis.getElementAt(1, 2), 0.0, 0.0);
        assertEquals(jacobianOfAxis.getElementAt(2, 2), 0.0, 0.0);
        assertEquals(jacobianOfAxis.getElementAt(3, 2), s, 0.0);

        // Force IllegalArgumentException
        try {
            q.setFromAxisAndRotation(axisRotation, invalid, jacobianOfAxis);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        try {
            q.setFromAxisAndRotation(axisRotation, jacobianOfTheta, invalid);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testMultiplyAndProduct() throws AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double a1 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double a2 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double b1 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double b2 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double c1 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double c2 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double d1 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double d2 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final Quaternion q1 = new Quaternion(a1, b1, c1, d1);
        final Quaternion q2 = new Quaternion(a2, b2, c2, d2);

        final double a = a1 * a2 - b1 * b2 - c1 * c2 - d1 * d2;
        final double b = a1 * b2 + b1 * a2 + c1 * d2 - d1 * c2;
        final double c = a1 * c2 - b1 * d2 + c1 * a2 + d1 * b2;
        final double d = a1 * d2 + b1 * c2 - c1 * b2 + d1 * a2;

        Quaternion q = new Quaternion(q1);
        q.multiply(q2);

        // check correctness
        assertEquals(q.getA(), a, ABSOLUTE_ERROR);
        assertEquals(q.getB(), b, ABSOLUTE_ERROR);
        assertEquals(q.getC(), c, ABSOLUTE_ERROR);
        assertEquals(q.getD(), d, ABSOLUTE_ERROR);

        q = q1.multiplyAndReturnNew(q2);

        // check correctness
        assertEquals(q.getA(), a, ABSOLUTE_ERROR);
        assertEquals(q.getB(), b, ABSOLUTE_ERROR);
        assertEquals(q.getC(), c, ABSOLUTE_ERROR);
        assertEquals(q.getD(), d, ABSOLUTE_ERROR);

        q = new Quaternion();
        q1.multiply(q2, q);

        // check correctness
        assertEquals(q.getA(), a, ABSOLUTE_ERROR);
        assertEquals(q.getB(), b, ABSOLUTE_ERROR);
        assertEquals(q.getC(), c, ABSOLUTE_ERROR);
        assertEquals(q.getD(), d, ABSOLUTE_ERROR);

        q = new Quaternion();
        Quaternion.product(q1, q2, q);

        // check correctness
        assertEquals(q.getA(), a, ABSOLUTE_ERROR);
        assertEquals(q.getB(), b, ABSOLUTE_ERROR);
        assertEquals(q.getC(), c, ABSOLUTE_ERROR);
        assertEquals(q.getD(), d, ABSOLUTE_ERROR);

        q = new Quaternion();
        Matrix jacobianQ1 = new Matrix(Quaternion.N_PARAMS,
                Quaternion.N_PARAMS);
        Matrix jacobianQ2 = new Matrix(Quaternion.N_PARAMS,
                Quaternion.N_PARAMS);
        Quaternion.product(q1, q2, q, jacobianQ1, jacobianQ2);

        // check correctness
        assertEquals(q.getA(), a, ABSOLUTE_ERROR);
        assertEquals(q.getB(), b, ABSOLUTE_ERROR);
        assertEquals(q.getC(), c, ABSOLUTE_ERROR);
        assertEquals(q.getD(), d, ABSOLUTE_ERROR);

        assertEquals(jacobianQ1.getElementAt(0, 0), a2, ABSOLUTE_ERROR);
        assertEquals(jacobianQ1.getElementAt(1, 0), b2, ABSOLUTE_ERROR);
        assertEquals(jacobianQ1.getElementAt(2, 0), c2, ABSOLUTE_ERROR);
        assertEquals(jacobianQ1.getElementAt(3, 0), d2, ABSOLUTE_ERROR);

        assertEquals(jacobianQ1.getElementAt(0, 1), -b2, ABSOLUTE_ERROR);
        assertEquals(jacobianQ1.getElementAt(1, 1), a2, ABSOLUTE_ERROR);
        assertEquals(jacobianQ1.getElementAt(2, 1), -d2, ABSOLUTE_ERROR);
        assertEquals(jacobianQ1.getElementAt(3, 1), c2, ABSOLUTE_ERROR);

        assertEquals(jacobianQ1.getElementAt(0, 2), -c2, ABSOLUTE_ERROR);
        assertEquals(jacobianQ1.getElementAt(1, 2), d2, ABSOLUTE_ERROR);
        assertEquals(jacobianQ1.getElementAt(2, 2), a2, ABSOLUTE_ERROR);
        assertEquals(jacobianQ1.getElementAt(3, 2), -b2, ABSOLUTE_ERROR);

        assertEquals(jacobianQ1.getElementAt(0, 3), -d2, ABSOLUTE_ERROR);
        assertEquals(jacobianQ1.getElementAt(1, 3), -c2, ABSOLUTE_ERROR);
        assertEquals(jacobianQ1.getElementAt(2, 3), b2, ABSOLUTE_ERROR);
        assertEquals(jacobianQ1.getElementAt(3, 3), a2, ABSOLUTE_ERROR);

        assertEquals(jacobianQ2.getElementAt(0, 0), a1, ABSOLUTE_ERROR);
        assertEquals(jacobianQ2.getElementAt(1, 0), b1, ABSOLUTE_ERROR);
        assertEquals(jacobianQ2.getElementAt(2, 0), c1, ABSOLUTE_ERROR);
        assertEquals(jacobianQ2.getElementAt(3, 0), d1, ABSOLUTE_ERROR);

        assertEquals(jacobianQ2.getElementAt(0, 1), -b1, ABSOLUTE_ERROR);
        assertEquals(jacobianQ2.getElementAt(1, 1), a1, ABSOLUTE_ERROR);
        assertEquals(jacobianQ2.getElementAt(2, 1), d1, ABSOLUTE_ERROR);
        assertEquals(jacobianQ2.getElementAt(3, 1), -c1, ABSOLUTE_ERROR);

        assertEquals(jacobianQ2.getElementAt(0, 2), -c1, ABSOLUTE_ERROR);
        assertEquals(jacobianQ2.getElementAt(1, 2), -d1, ABSOLUTE_ERROR);
        assertEquals(jacobianQ2.getElementAt(2, 2), a1, ABSOLUTE_ERROR);
        assertEquals(jacobianQ2.getElementAt(3, 2), b1, ABSOLUTE_ERROR);

        assertEquals(jacobianQ2.getElementAt(0, 3), -d1, ABSOLUTE_ERROR);
        assertEquals(jacobianQ2.getElementAt(1, 3), c1, ABSOLUTE_ERROR);
        assertEquals(jacobianQ2.getElementAt(2, 3), -b1, ABSOLUTE_ERROR);
        assertEquals(jacobianQ2.getElementAt(3, 3), a1, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final Matrix invalid = new Matrix(1, 1);
        try {
            Quaternion.product(q1, q2, q, invalid, jacobianQ2);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            Quaternion.product(q1, q2, q, jacobianQ1, invalid);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        // check correctness of jacobians
        q = new Quaternion();
        jacobianQ1 = new Matrix(Quaternion.N_PARAMS,
                Quaternion.N_PARAMS);
        jacobianQ2 = new Matrix(Quaternion.N_PARAMS,
                Quaternion.N_PARAMS);
        Quaternion.product(q1, q2, q, jacobianQ1, jacobianQ2);

        // check q1 variation
        double[] diff = new double[4];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        final Quaternion q1b = new Quaternion(q1.getA() + diff[0],
                q1.getB() + diff[1], q1.getC() + diff[2], q1.getD() + diff[3]);
        Quaternion qb = new Quaternion();
        Quaternion.product(q1b, q2, qb);

        double[] diffResult = new double[]{
                qb.getA() - q.getA(),
                qb.getB() - q.getB(),
                qb.getC() - q.getC(),
                qb.getD() - q.getD()
        };
        double[] diffResult2 = jacobianQ1.multiplyAndReturnNew(
                Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);

        // check q2 variation
        diff = new double[4];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        final Quaternion q2b = new Quaternion(q2.getA() + diff[0],
                q2.getB() + diff[1], q2.getC() + diff[2], q2.getD() + diff[3]);
        qb = new Quaternion();
        Quaternion.product(q1, q2b, qb);

        diffResult = new double[]{
                qb.getA() - q.getA(),
                qb.getB() - q.getB(),
                qb.getC() - q.getC(),
                qb.getD() - q.getD()
        };
        diffResult2 = jacobianQ2.multiplyAndReturnNew(
                Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);
    }

    @Test
    public void testSetFromEulerAngles() throws AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double roll = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double yaw = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        Matrix jacobian = new Matrix(Quaternion.N_PARAMS, Quaternion.N_ANGLES);

        Quaternion q = new Quaternion();
        q.setFromEulerAngles(roll, pitch, yaw, jacobian);

        // check correctness
        double[] angles = q.toEulerAngles();
        assertEquals(roll, angles[0], ABSOLUTE_ERROR);
        assertEquals(pitch, angles[1], ABSOLUTE_ERROR);
        assertEquals(yaw, angles[2], ABSOLUTE_ERROR);

        final double halfRoll = roll / 2.0;
        final double halfPitch = pitch / 2.0;
        final double halfYaw = yaw / 2.0;

        final double sr = Math.sin(halfRoll);
        final double sp = Math.sin(halfPitch);
        final double sy = Math.sin(halfYaw);

        final double cr = Math.cos(halfRoll);
        final double cp = Math.cos(halfPitch);
        final double cy = Math.cos(halfYaw);

        final double tmp1 = -cy * cp * sr + sy * sp * cr;
        final double tmp2 = 0.5 * (cy * cp * cr + sy * sp * sr);
        final double tmp3 = -cy * sp * sr + sy * cp * cr;
        final double tmp4 = -sy * cp * sr - cy * sp * cr;
        final double tmp5 = -cy * sp * cr + sy * cp * sr;
        final double tmp6 = -cy * sp * sr - sy * cp * cr;
        final double tmp7 = 0.5 * (cy * cp * cr - sy * sp * sr);
        final double tmp8 = -cy * cp * sr - sy * sp * cr;
        final double tmp9 = -sy * cp * cr + cy * sp * sr;
        final double tmp10 = -sy * sp * cr + cy * cp * sr;
        assertEquals(jacobian.getElementAt(0, 0), 0.5 * tmp1,
                ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(1, 0), tmp2,
                ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(2, 0), 0.5 * tmp3,
                ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(3, 0), 0.5 * tmp4,
                ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(0, 1), 0.5 * tmp5,
                ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(1, 1), 0.5 * tmp6,
                ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(2, 1), tmp7,
                ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(3, 1), 0.5 * tmp8,
                ABSOLUTE_ERROR);

        assertEquals(jacobian.getElementAt(0, 2), 0.5 * tmp9,
                ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(1, 2), 0.5 * tmp4,
                ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(2, 2), 0.5 * tmp10,
                ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(3, 2), tmp2,
                ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final Matrix invalid = new Matrix(1, 1);
        try {
            q.setFromEulerAngles(roll, pitch, yaw, invalid);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        // set values
        q = new Quaternion();
        q.setFromEulerAngles(roll, pitch, yaw);

        // check correctness
        angles = q.toEulerAngles();
        assertEquals(roll, angles[0], ABSOLUTE_ERROR);
        assertEquals(pitch, angles[1], ABSOLUTE_ERROR);
        assertEquals(yaw, angles[2], ABSOLUTE_ERROR);

        // set values and jacobian
        jacobian = new Matrix(Quaternion.N_PARAMS, Quaternion.N_ANGLES);
        q = new Quaternion();
        q.setFromEulerAngles(new double[]{roll, pitch, yaw}, jacobian);

        // check correctness
        angles = q.toEulerAngles();
        assertEquals(roll, angles[0], ABSOLUTE_ERROR);
        assertEquals(pitch, angles[1], ABSOLUTE_ERROR);
        assertEquals(yaw, angles[2], ABSOLUTE_ERROR);

        assertEquals(jacobian.getElementAt(0, 0), 0.5 * tmp1,
                ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(1, 0), tmp2,
                ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(2, 0), 0.5 * tmp3,
                ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(3, 0), 0.5 * tmp4,
                ABSOLUTE_ERROR);

        assertEquals(jacobian.getElementAt(0, 1), 0.5 * tmp5,
                ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(1, 1), 0.5 * tmp6,
                ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(2, 1), tmp7,
                ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(3, 1), 0.5 * tmp8,
                ABSOLUTE_ERROR);

        assertEquals(jacobian.getElementAt(0, 2), 0.5 * tmp9,
                ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(1, 2), 0.5 * tmp4,
                ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(2, 2), 0.5 * tmp10,
                ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(3, 2), tmp2,
                ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            q.setFromEulerAngles(new double[1], jacobian);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) {
        }
        try {
            q.setFromEulerAngles(new double[]{roll, pitch, yaw}, invalid);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) {
        }

        // set values
        q = new Quaternion();
        q.setFromEulerAngles(new double[]{roll, pitch, yaw});

        // check correctness
        angles = q.toEulerAngles();
        assertEquals(roll, angles[0], ABSOLUTE_ERROR);
        assertEquals(pitch, angles[1], ABSOLUTE_ERROR);
        assertEquals(yaw, angles[2], ABSOLUTE_ERROR);
    }

    @Test
    public void testEulerToMatrixRotation() throws AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double roll = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double yaw = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        final MatrixRotation3D rot = new MatrixRotation3D();
        rot.setRollPitchYaw(roll, pitch, yaw);

        MatrixRotation3D rot2 = new MatrixRotation3D();
        Matrix jacobian = new Matrix(9, 3);
        Quaternion.eulerToMatrixRotation(roll, pitch, yaw, rot2, jacobian);

        // check correctness
        assertEquals(rot, rot2);

        final double sr = Math.sin(roll);
        final double cr = Math.cos(roll);
        final double sp = Math.sin(pitch);
        final double cp = Math.cos(pitch);
        final double sy = Math.sin(yaw);
        final double cy = Math.cos(yaw);

        final double tmp1 = -cr * sy + sr * sp * cy;
        final double tmp2 = cr * cy + sr * sp * sy;
        final double tmp3 = sr * sy + cr * sp * cy;
        final double tmp4 = -sr * cy + cr * sp * sy;
        final double tmp5 = cr * sy - sr * sp * cy;
        final double tmp6 = -cr * cy - sr * sp * sy;
        final double tmp7 = sr * cy - cr * sp * sy;

        assertEquals(rot2.internalMatrix.getElementAt(0, 0), cp * cy,
                ABSOLUTE_ERROR);
        assertEquals(rot2.internalMatrix.getElementAt(1, 0), cp * sy,
                ABSOLUTE_ERROR);
        assertEquals(rot2.internalMatrix.getElementAt(2, 0), -sp,
                ABSOLUTE_ERROR);

        assertEquals(rot2.internalMatrix.getElementAt(0, 1), tmp1,
                ABSOLUTE_ERROR);
        assertEquals(rot2.internalMatrix.getElementAt(1, 1), tmp2,
                ABSOLUTE_ERROR);
        assertEquals(rot2.internalMatrix.getElementAt(2, 1), sr * cp,
                ABSOLUTE_ERROR);

        assertEquals(rot2.internalMatrix.getElementAt(0, 2), tmp3,
                ABSOLUTE_ERROR);
        assertEquals(rot2.internalMatrix.getElementAt(1, 2), tmp4,
                ABSOLUTE_ERROR);
        assertEquals(rot2.internalMatrix.getElementAt(2, 2), cr * cp,
                ABSOLUTE_ERROR);

        assertEquals(jacobian.getElementAt(0, 0), 0.0, 0.0);
        assertEquals(jacobian.getElementAt(1, 0), 0.0, 0.0);
        assertEquals(jacobian.getElementAt(2, 0), 0.0, 0.0);
        assertEquals(jacobian.getElementAt(3, 0), tmp3,
                ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(4, 0), tmp4,
                ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(5, 0), cr * cp, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(6, 0), tmp5,
                ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(7, 0), tmp6,
                ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(8, 0), -sr * cp, ABSOLUTE_ERROR);

        assertEquals(jacobian.getElementAt(0, 1), -sp * cy, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(1, 1), -sp * sy, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(2, 1), -cp, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(3, 1), sr * cp * cy, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(4, 1), sr * cp * sy, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(5, 1), -sr * sp, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(6, 1), cr * cp * cy, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(7, 1), cr * cp * sy, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(8, 1), -cr * sp, ABSOLUTE_ERROR);

        assertEquals(jacobian.getElementAt(0, 2), -cp * sy, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(1, 2), cp * cy, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(2, 2), 0.0, 0.0);
        assertEquals(jacobian.getElementAt(3, 2), tmp6,
                ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(4, 2), tmp1,
                ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(5, 2), 0.0, 0.0);
        assertEquals(jacobian.getElementAt(6, 2), tmp7,
                ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(7, 2), tmp3,
                ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(8, 2), 0.0, 0.0);

        // Force IllegalArgumentException
        final Matrix invalid = new Matrix(1, 1);
        try {
            Quaternion.eulerToMatrixRotation(roll, pitch, yaw, rot2, invalid);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        // set new values
        rot2 = new MatrixRotation3D();
        Quaternion.eulerToMatrixRotation(roll, pitch, yaw, rot2);

        // check correctness
        assertEquals(rot, rot2);

        // set new values
        rot2 = new MatrixRotation3D();
        jacobian = new Matrix(9, 3);
        Quaternion.eulerToMatrixRotation(new double[]{roll, pitch, yaw}, rot2,
                jacobian);

        // check correctness
        assertEquals(rot, rot2);

        assertEquals(rot2.internalMatrix.getElementAt(0, 0), cp * cy,
                ABSOLUTE_ERROR);
        assertEquals(rot2.internalMatrix.getElementAt(1, 0), cp * sy,
                ABSOLUTE_ERROR);
        assertEquals(rot2.internalMatrix.getElementAt(2, 0), -sp,
                ABSOLUTE_ERROR);

        assertEquals(rot2.internalMatrix.getElementAt(0, 1), tmp1,
                ABSOLUTE_ERROR);
        assertEquals(rot2.internalMatrix.getElementAt(1, 1), tmp2,
                ABSOLUTE_ERROR);
        assertEquals(rot2.internalMatrix.getElementAt(2, 1), sr * cp,
                ABSOLUTE_ERROR);

        assertEquals(rot2.internalMatrix.getElementAt(0, 2), tmp3,
                ABSOLUTE_ERROR);
        assertEquals(rot2.internalMatrix.getElementAt(1, 2), tmp4,
                ABSOLUTE_ERROR);
        assertEquals(rot2.internalMatrix.getElementAt(2, 2), cr * cp,
                ABSOLUTE_ERROR);

        assertEquals(jacobian.getElementAt(0, 0), 0.0, 0.0);
        assertEquals(jacobian.getElementAt(1, 0), 0.0, 0.0);
        assertEquals(jacobian.getElementAt(2, 0), 0.0, 0.0);
        assertEquals(jacobian.getElementAt(3, 0), tmp3,
                ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(4, 0), tmp4,
                ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(5, 0), cr * cp, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(6, 0), tmp5,
                ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(7, 0), tmp6,
                ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(8, 0), -sr * cp, ABSOLUTE_ERROR);

        assertEquals(jacobian.getElementAt(0, 1), -sp * cy, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(1, 1), -sp * sy, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(2, 1), -cp, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(3, 1), sr * cp * cy, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(4, 1), sr * cp * sy, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(5, 1), -sr * sp, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(6, 1), cr * cp * cy, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(7, 1), cr * cp * sy, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(8, 1), -cr * sp, ABSOLUTE_ERROR);

        assertEquals(jacobian.getElementAt(0, 2), -cp * sy, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(1, 2), cp * cy, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(2, 2), 0.0, 0.0);
        assertEquals(jacobian.getElementAt(3, 2), tmp6,
                ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(4, 2), tmp1,
                ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(5, 2), 0.0, 0.0);
        assertEquals(jacobian.getElementAt(6, 2), tmp7,
                ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(7, 2), tmp3,
                ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(8, 2), 0.0, 0.0);

        // Force IllegalArgumentException
        try {
            Quaternion.eulerToMatrixRotation(new double[1], rot2, jacobian);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            Quaternion.eulerToMatrixRotation(new double[]{roll, pitch, yaw},
                    rot2, invalid);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        // set new values
        rot2 = new MatrixRotation3D();
        Quaternion.eulerToMatrixRotation(new double[]{roll, pitch, yaw}, rot2);

        // check correctness
        assertEquals(rot, rot2);

        // Force IllegalArgumentException
        try {
            Quaternion.eulerToMatrixRotation(new double[1], rot2);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testToAxisAndRotationAngle() throws AlgebraException,
            RotationException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        // Find any 3 orthogonal vectors, 1st will be axis of rotation, and
        // the remaining two will lie on the rotation plane and will be used
        // to test theta angle

        // To find 3 orthogonal vectors, we use V matrix of a singular
        // decomposition of any Nx3 matrix
        final Matrix aM = Matrix.createWithUniformRandomValues(1, ROTATION_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final SingularValueDecomposer decomposer = new SingularValueDecomposer(aM);

        decomposer.decompose();

        final Matrix vMatrix = decomposer.getV();

        // axis of rotation
        final double[] axis = vMatrix.getSubmatrixAsArray(0, 0,
                2, 0);

        final Quaternion q = new Quaternion(axis, theta);

        final double[] axis2 = new double[Quaternion.N_ANGLES];
        final Matrix jacobianAngle = new Matrix(1, Quaternion.N_PARAMS);
        final Matrix jacobianAxis = new Matrix(Quaternion.N_ANGLES,
                Quaternion.N_PARAMS);
        double theta2 = q.toAxisAndRotationAngle(axis2, jacobianAngle,
                jacobianAxis);

        // check correctness
        assertArrayEquals(axis, axis2, ABSOLUTE_ERROR);
        assertEquals(theta, theta2, ABSOLUTE_ERROR);

        final double n = Math.sqrt(q.getB() * q.getB() + q.getC() * q.getC() +
                q.getD() * q.getD());
        final double s = q.getA();
        final double denom = n * n + s * s;
        final double aN = 2.0 * s / denom;
        final double aS = -2.0 * n / denom;

        assertEquals(jacobianAngle.getElementAtIndex(0), aS, ABSOLUTE_ERROR);
        assertEquals(jacobianAngle.getElementAtIndex(1), axis[0] * aN,
                ABSOLUTE_ERROR);
        assertEquals(jacobianAngle.getElementAtIndex(2), axis[1] * aN,
                ABSOLUTE_ERROR);
        assertEquals(jacobianAngle.getElementAtIndex(3), axis[2] * aN,
                ABSOLUTE_ERROR);

        final Matrix uV = Matrix.identity(3, 3);
        uV.multiplyByScalar(n);
        uV.subtract(Matrix.newFromArray(
                new double[]{q.getB(), q.getC(), q.getD()}, true).
                multiplyAndReturnNew(Matrix.newFromArray(axis2, false)));
        uV.multiplyByScalar(1.0 / (n * n));

        assertEquals(jacobianAxis.getSubmatrix(0, 1,
                2, 3), uV);
        assertArrayEquals(jacobianAxis.getSubmatrix(0, 0,
                2, 0).toArray(),
                new double[]{0.0, 0.0, 0.0}, 0.0);

        theta2 = q.toAxisAndRotationAngle(axis2);

        // check correctness
        assertArrayEquals(axis, axis2, ABSOLUTE_ERROR);
        assertEquals(theta, theta2, ABSOLUTE_ERROR);

        AxisRotation3D axisRotation = new AxisRotation3D();
        q.toAxisRotation(axisRotation);

        // check correctness
        assertArrayEquals(axis, axisRotation.getRotationAxis(), ABSOLUTE_ERROR);
        assertEquals(theta, axisRotation.getRotationAngle(), ABSOLUTE_ERROR);

        axisRotation = q.toAxisRotation();

        // check correctness
        assertArrayEquals(axis, axisRotation.getRotationAxis(), ABSOLUTE_ERROR);
        assertEquals(theta, axisRotation.getRotationAngle(), ABSOLUTE_ERROR);
    }

    @Test
    public void testToRotationVector() throws AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        // Find any 3 orthogonal vectors, 1st will be axis of rotation, and
        // the remaining two will lie on the rotation plane and will be used
        // to test theta angle

        // To find 3 orthogonal vectors, we use V matrix of a singular
        // decomposition of any Nx3 matrix
        final Matrix aM = Matrix.createWithUniformRandomValues(1, ROTATION_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final SingularValueDecomposer decomposer = new SingularValueDecomposer(aM);

        decomposer.decompose();

        final Matrix vMatrix = decomposer.getV();

        // axis of rotation
        final double[] axis = vMatrix.getSubmatrixAsArray(0, 0,
                2, 0);

        final Quaternion q = new Quaternion(axis, theta);

        double[] rotationVector = new double[axis.length];
        final Matrix jacobian = new Matrix(Quaternion.N_ANGLES, Quaternion.N_PARAMS);

        q.toRotationVector(rotationVector, jacobian);

        // check correctness
        final double[] axis2 = new double[axis.length];
        final Matrix jacobianAngle = new Matrix(1, Quaternion.N_PARAMS);
        final Matrix jacobianAxis = new Matrix(Quaternion.N_ANGLES,
                Quaternion.N_PARAMS);
        final double theta2 = q.toAxisAndRotationAngle(axis2, jacobianAngle,
                jacobianAxis);
        final Matrix vA = Matrix.newFromArray(axis2, true);
        final Matrix vQ = vA.multiplyAndReturnNew(jacobianAngle).addAndReturnNew(
                jacobianAxis.multiplyByScalarAndReturnNew(theta2));

        // rotation vector is proportional to rotation axis but having a norm equal
        // to the rotation angle
        assertEquals(ArrayUtils.dotProduct(rotationVector, axis),
                Math.abs(theta), ABSOLUTE_ERROR);

        assertTrue(jacobian.equals(vQ, ABSOLUTE_ERROR));

        // throw IllegalArgumentException
        try {
            q.toRotationVector(new double[1], jacobian);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            q.toRotationVector(rotationVector, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        rotationVector = new double[axis.length];
        q.toRotationVector(rotationVector);

        // rotation vector is proportional to rotation axis but having a norm equal
        // to the rotation angle
        assertEquals(ArrayUtils.dotProduct(rotationVector, axis),
                Math.abs(theta), ABSOLUTE_ERROR);
    }

    @Test
    public void testToEulerAngles() throws AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double roll = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double yaw = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        final MatrixRotation3D rot = new MatrixRotation3D();
        rot.setRollPitchYaw(roll, pitch, yaw);

        final Quaternion q = new Quaternion(roll, pitch, yaw);

        //noinspection AssertBetweenInconvertibleTypes
        assertEquals(rot, q);

        double[] angles = new double[Quaternion.N_ANGLES];
        final Matrix jacobian = new Matrix(Quaternion.N_ANGLES, Quaternion.N_PARAMS);

        q.toEulerAngles(angles, jacobian);

        // check correctness
        assertEquals(roll, angles[0], ABSOLUTE_ERROR);
        assertEquals(pitch, angles[1], ABSOLUTE_ERROR);
        assertEquals(yaw, angles[2], ABSOLUTE_ERROR);

        final double a = q.getA();
        final double b = q.getB();
        final double c = q.getC();
        final double d = q.getD();

        final double y1 = 2.0 * c * d + 2.0 * a * b;
        final double x1 = a * a - b * b - c * c + d * d;
        final double z2 = -2.0 * b * d + 2.0 * a * c;
        final double y3 = 2.0 * b * c + 2.0 * a * d;
        final double x3 = a * a + b * b - c * c - d * d;

        final double[] dx1dq = new double[]{2 * a, -2 * b, -2 * c, 2 * d};
        final double[] dy1dq = new double[]{2 * b, 2 * a, 2 * d, 2 * c};
        final double[] dz2dq = new double[]{2 * c, -2 * d, 2 * a, -2 * b};
        final double[] dx3dq = new double[]{2 * a, 2 * b, -2 * c, -2 * d};
        final double[] dy3dq = new double[]{2 * d, 2 * c, 2 * b, 2 * a};

        final double de1dx1 = -y1 / (x1 * x1 + y1 * y1);
        final double de1dy1 = x1 / (x1 * x1 + y1 * y1);
        final double de2dz2 = 1 / Math.sqrt(1 - z2 * z2);
        final double de3dx3 = -y3 / (x3 * x3 + y3 * y3);
        final double de3dy3 = x3 / (x3 * x3 + y3 * y3);

        // de1dq = de1dx1 * dx1dq + de1dy * dy1dq
        ArrayUtils.multiplyByScalar(dx1dq, de1dx1, dx1dq);
        ArrayUtils.multiplyByScalar(dy1dq, de1dy1, dy1dq);
        final double[] de1dq = ArrayUtils.sumAndReturnNew(dx1dq, dy1dq);

        // de2dq = de2dz2 * dz2dq
        final double[] de2dq = ArrayUtils.multiplyByScalarAndReturnNew(dz2dq,
                de2dz2);

        // de3dq = de3dx3 * dx3dq + de3dy3 * dy3dq
        ArrayUtils.multiplyByScalar(dx3dq, de3dx3, dx3dq);
        ArrayUtils.multiplyByScalar(dy3dq, de3dy3, dy3dq);
        final double[] de3dq = ArrayUtils.sumAndReturnNew(dx3dq, dy3dq);

        assertEquals(jacobian.getElementAt(0, 0), de1dq[0], ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(0, 1), de1dq[1], ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(0, 2), de1dq[2], ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(0, 3), de1dq[3], ABSOLUTE_ERROR);

        assertEquals(jacobian.getElementAt(1, 0), de2dq[0], ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(1, 1), de2dq[1], ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(1, 2), de2dq[2], ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(1, 3), de2dq[3], ABSOLUTE_ERROR);

        assertEquals(jacobian.getElementAt(2, 0), de3dq[0], ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(2, 1), de3dq[1], ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(2, 2), de3dq[2], ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(2, 3), de3dq[3], ABSOLUTE_ERROR);

        angles = new double[Quaternion.N_ANGLES];
        q.toEulerAngles(angles);

        // check correctness
        assertEquals(roll, angles[0], ABSOLUTE_ERROR);
        assertEquals(pitch, angles[1], ABSOLUTE_ERROR);
        assertEquals(yaw, angles[2], ABSOLUTE_ERROR);
    }

    @Test
    public void testQuaternionMatrix() throws AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double roll1 = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double pitch1 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double yaw1 = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        final double roll2 = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double pitch2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double yaw2 = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        final Quaternion q1 = new Quaternion(roll1, pitch1, yaw1);
        final Quaternion q2 = new Quaternion(roll2, pitch2, yaw2);

        final Quaternion q = q1.multiplyAndReturnNew(q2);

        // check that quaternion product can be expressed as the matrix
        // product m1 with quaternion q2 expressed as a column vector
        Matrix m1 = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
        q1.quaternionMatrix(m1);

        final Matrix m2 = Matrix.newFromArray(q2.getValues(), true);
        Matrix m = m1.multiplyAndReturnNew(m2);

        assertArrayEquals(q.getValues(), m.getBuffer(), ABSOLUTE_ERROR);

        m1 = q1.toQuaternionMatrix();
        m = m1.multiplyAndReturnNew(m2);
        assertArrayEquals(q.getValues(), m.getBuffer(), ABSOLUTE_ERROR);
    }

    @Test
    public void testConjugate() throws AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final Quaternion q = new Quaternion(a, b, c, d);

        // check correctness
        assertEquals(q.getA(), a, 0.0);
        assertEquals(q.getB(), b, 0.0);
        assertEquals(q.getC(), c, 0.0);
        assertEquals(q.getD(), d, 0.0);

        // conjugate
        Quaternion qc = new Quaternion();
        final Matrix jacobian = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
        q.conjugate(qc, jacobian);

        // check correctness
        assertEquals(q.getA(), qc.getA(), 0.0);
        assertEquals(q.getB(), -qc.getB(), 0.0);
        assertEquals(q.getC(), -qc.getC(), 0.0);
        assertEquals(q.getD(), -qc.getD(), 0.0);

        assertEquals(jacobian.getElementAt(0, 0), 1.0, 0.0);
        assertEquals(jacobian.getElementAt(1, 0), 0.0, 0.0);
        assertEquals(jacobian.getElementAt(2, 0), 0.0, 0.0);
        assertEquals(jacobian.getElementAt(3, 0), 0.0, 0.0);

        assertEquals(jacobian.getElementAt(0, 1), 0.0, 0.0);
        assertEquals(jacobian.getElementAt(1, 1), -1.0, 0.0);
        assertEquals(jacobian.getElementAt(2, 1), 0.0, 0.0);
        assertEquals(jacobian.getElementAt(3, 0), 0.0, 0.0);

        assertEquals(jacobian.getElementAt(0, 2), 0.0, 0.0);
        assertEquals(jacobian.getElementAt(1, 2), 0.0, 0.0);
        assertEquals(jacobian.getElementAt(2, 2), -1.0, 0.0);
        assertEquals(jacobian.getElementAt(3, 2), 0.0, 0.0);

        assertEquals(jacobian.getElementAt(0, 3), 0.0, 0.0);
        assertEquals(jacobian.getElementAt(1, 3), 0.0, 0.0);
        assertEquals(jacobian.getElementAt(2, 3), 0.0, 0.0);
        assertEquals(jacobian.getElementAt(3, 3), -1.0, 0.0);

        qc = new Quaternion();
        q.conjugate(qc);

        final Quaternion qcB = q.conjugateAndReturnNew();

        // check correctness
        assertEquals(q.getA(), qc.getA(), 0.0);
        assertEquals(q.getB(), -qc.getB(), 0.0);
        assertEquals(q.getC(), -qc.getC(), 0.0);
        assertEquals(q.getD(), -qc.getD(), 0.0);

        assertEquals(qc, qcB);

        // conjugate of conjugate is the same as the original quaternion
        qc.conjugate(qc);
        assertEquals(qc, q);

        try {
            q.conjugate(qc, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testQuaternionMatrixN() throws AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double roll1 = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double pitch1 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double yaw1 = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        final double roll2 = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double pitch2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double yaw2 = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        final Quaternion q1 = new Quaternion(roll1, pitch1, yaw1);
        final Quaternion q2 = new Quaternion(roll2, pitch2, yaw2);

        final Quaternion q = q1.multiplyAndReturnNew(q2);

        // check that quaternion product can be expressed as the matrix
        // product m1 with quaternion q2 expressed as a column vector
        Matrix m2 = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
        q2.quaternionMatrixN(m2);

        final Matrix m1 = Matrix.newFromArray(q1.getValues(), true);
        Matrix m = m2.multiplyAndReturnNew(m1);

        assertArrayEquals(q.getValues(), m.getBuffer(), ABSOLUTE_ERROR);

        m2 = q2.toQuaternionMatrixN();
        m = m2.multiplyAndReturnNew(m1);
        assertArrayEquals(q.getValues(), m.getBuffer(), ABSOLUTE_ERROR);
    }

    @Test
    public void testToMatrixRotation() throws AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double roll = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double yaw = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        final MatrixRotation3D matrixRot = new MatrixRotation3D();
        matrixRot.setRollPitchYaw(roll, pitch, yaw);

        final Quaternion q = new Quaternion(roll, pitch, yaw);

        Matrix m = new Matrix(MatrixRotation3D.ROTATION3D_INHOM_MATRIX_ROWS,
                MatrixRotation3D.ROTATION3D_INHOM_MATRIX_COLS);
        final Matrix jacobian = new Matrix(9, 4);
        q.toMatrixRotation(m, jacobian);

        assertTrue(m.equals(matrixRot.internalMatrix, ABSOLUTE_ERROR));

        final double a = q.getA();
        final double b = q.getB();
        final double c = q.getC();
        final double d = q.getD();

        assertEquals(jacobian.getElementAt(0, 0), 2 * a, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(1, 0), 2 * d, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(2, 0), -2 * c, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(3, 0), -2 * d, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(4, 0), 2 * a, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(5, 0), 2 * b, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(6, 0), 2 * c, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(7, 0), -2 * b, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(8, 0), 2 * a, ABSOLUTE_ERROR);

        assertEquals(jacobian.getElementAt(0, 1), 2 * b, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(1, 1), 2 * c, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(2, 1), 2 * d, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(3, 1), 2 * c, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(4, 1), -2 * b, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(5, 1), 2 * a, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(6, 1), 2 * d, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(7, 1), -2 * a, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(8, 1), -2 * b, ABSOLUTE_ERROR);

        assertEquals(jacobian.getElementAt(0, 2), -2 * c, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(1, 2), 2 * b, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(2, 2), -2 * a, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(3, 2), 2 * b, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(4, 2), 2 * c, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(5, 2), 2 * d, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(6, 2), 2 * a, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(7, 2), 2 * d, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(8, 2), -2 * c, ABSOLUTE_ERROR);

        assertEquals(jacobian.getElementAt(0, 3), -2 * d, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(1, 3), 2 * a, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(2, 3), 2 * b, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(3, 3), -2 * a, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(4, 3), -2 * d, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(5, 3), 2 * c, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(6, 3), 2 * b, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(7, 3), 2 * c, ABSOLUTE_ERROR);
        assertEquals(jacobian.getElementAt(8, 3), 2 * d, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            q.toMatrixRotation(m, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            q.toMatrixRotation(new Matrix(1, 1), jacobian);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        m = new Matrix(MatrixRotation3D.ROTATION3D_INHOM_MATRIX_ROWS,
                MatrixRotation3D.ROTATION3D_INHOM_MATRIX_COLS);
        q.toMatrixRotation(m);

        assertTrue(m.equals(matrixRot.internalMatrix, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        try {
            q.toMatrixRotation(new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        final MatrixRotation3D matrixRot2 = new MatrixRotation3D();
        q.toMatrixRotation(matrixRot2);

        assertEquals(matrixRot, matrixRot2);

        final MatrixRotation3D matrixRot3 = q.toMatrixRotation();

        assertEquals(matrixRot, matrixRot3);
    }

    @Test
    public void testRotate() throws ColinearPointsException, AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double roll = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double yaw = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        final MatrixRotation3D matrixRot = new MatrixRotation3D();
        matrixRot.setRollPitchYaw(roll, pitch, yaw);

        final Quaternion q = new Quaternion(roll, pitch, yaw);

        // create 3 random points
        final HomogeneousPoint3D point1 = new HomogeneousPoint3D();
        point1.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final HomogeneousPoint3D point2 = new HomogeneousPoint3D();
        point2.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final HomogeneousPoint3D point3 = new HomogeneousPoint3D();
        point3.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        // ensure that points are not co-linear

        while (Plane.areColinearPoints(point1, point2, point3)) {
            point2.setInhomogeneousCoordinates(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            point3.setInhomogeneousCoordinates(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        }

        // create plane passing through all three points
        final Plane plane = new Plane(point1, point2, point3);
        assertTrue(plane.isLocus(point1, ABSOLUTE_ERROR));
        assertTrue(plane.isLocus(point2, ABSOLUTE_ERROR));
        assertTrue(plane.isLocus(point3, ABSOLUTE_ERROR));

        // now rotate points
        final Matrix jacobianPoint1A = new Matrix(3, 3);
        final Matrix jacobianQuaternion1A = new Matrix(3, 4);
        final Point3D rotPoint1A = Point3D.create();
        Quaternion.rotate(q, point1, rotPoint1A, jacobianPoint1A,
                jacobianQuaternion1A);

        final Point3D rotPoint1B = Point3D.create();
        final Matrix jacobianPoint1B = new Matrix(3, 3);
        final Matrix jacobianQuaternion1B = new Matrix(3, 4);
        q.rotate(point1, rotPoint1B, jacobianPoint1B, jacobianQuaternion1B);

        final Point3D rotPoint1C = Point3D.create();
        q.rotate(point1, rotPoint1C);

        final Point3D rotPoint1D = q.rotate(point1);

        final Point3D rotPoint1E = matrixRot.rotate(point1);

        final Matrix jacobianPoint2A = new Matrix(3, 3);
        final Matrix jacobianQuaternion2A = new Matrix(3, 4);
        final Point3D rotPoint2A = Point3D.create();
        Quaternion.rotate(q, point2, rotPoint2A, jacobianPoint2A,
                jacobianQuaternion2A);

        final Point3D rotPoint2B = Point3D.create();
        final Matrix jacobianPoint2B = new Matrix(3, 3);
        final Matrix jacobianQuaternion2B = new Matrix(3, 4);
        q.rotate(point2, rotPoint2B, jacobianPoint2B, jacobianQuaternion2B);

        final Point3D rotPoint2C = Point3D.create();
        q.rotate(point2, rotPoint2C);

        final Point3D rotPoint2D = q.rotate(point2);

        final Point3D rotPoint2E = matrixRot.rotate(point2);

        final Matrix jacobianPoint3A = new Matrix(3, 3);
        final Matrix jacobianQuaternion3A = new Matrix(3, 4);
        final Point3D rotPoint3A = Point3D.create();
        Quaternion.rotate(q, point3, rotPoint3A, jacobianPoint3A,
                jacobianQuaternion3A);

        final Point3D rotPoint3B = Point3D.create();
        final Matrix jacobianPoint3B = new Matrix(3, 3);
        final Matrix jacobianQuaternion3B = new Matrix(3, 4);
        q.rotate(point3, rotPoint3B, jacobianPoint3B, jacobianQuaternion3B);

        final Point3D rotPoint3C = Point3D.create();
        q.rotate(point3, rotPoint3C);

        final Point3D rotPoint3D = q.rotate(point3);

        final Point3D rotPoint3E = matrixRot.rotate(point3);

        // check that rotated points A, B, C, D, E are equal
        assertTrue(rotPoint1A.equals(rotPoint1B, ABSOLUTE_ERROR));
        assertTrue(rotPoint1B.equals(rotPoint1C, ABSOLUTE_ERROR));
        assertTrue(rotPoint1C.equals(rotPoint1D, ABSOLUTE_ERROR));
        assertTrue(rotPoint1D.equals(rotPoint1E, ABSOLUTE_ERROR));
        assertTrue(rotPoint1E.equals(rotPoint1A, ABSOLUTE_ERROR));

        assertTrue(rotPoint2A.equals(rotPoint2B, ABSOLUTE_ERROR));
        assertTrue(rotPoint2B.equals(rotPoint2C, ABSOLUTE_ERROR));
        assertTrue(rotPoint2C.equals(rotPoint2D, ABSOLUTE_ERROR));
        assertTrue(rotPoint2D.equals(rotPoint2E, ABSOLUTE_ERROR));
        assertTrue(rotPoint2E.equals(rotPoint2A, ABSOLUTE_ERROR));

        assertTrue(rotPoint3A.equals(rotPoint3B, ABSOLUTE_ERROR));
        assertTrue(rotPoint3B.equals(rotPoint3C, ABSOLUTE_ERROR));
        assertTrue(rotPoint3C.equals(rotPoint3D, ABSOLUTE_ERROR));
        assertTrue(rotPoint3D.equals(rotPoint3E, ABSOLUTE_ERROR));
        assertTrue(rotPoint3D.equals(rotPoint3A, ABSOLUTE_ERROR));

        // check that points have been correctly rotated
        final Matrix point1Mat = Matrix.newFromArray(point1.asArray(), true);
        final Matrix point2Mat = Matrix.newFromArray(point2.asArray(), true);
        final Matrix point3Mat = Matrix.newFromArray(point3.asArray(), true);
        final Matrix R = q.asHomogeneousMatrix();
        final Matrix rotPoint1Mat = R.multiplyAndReturnNew(point1Mat);
        final Matrix rotPoint2Mat = R.multiplyAndReturnNew(point2Mat);
        final Matrix rotPoint3Mat = R.multiplyAndReturnNew(point3Mat);

        // check correctness
        double scaleX = rotPoint1A.getHomX() /
                rotPoint1Mat.getElementAtIndex(0);
        double scaleY = rotPoint1A.getHomY() /
                rotPoint1Mat.getElementAtIndex(1);
        double scaleZ = rotPoint1A.getHomZ() /
                rotPoint1Mat.getElementAtIndex(2);
        double scaleW = rotPoint1A.getHomW() /
                rotPoint1Mat.getElementAtIndex(3);
        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleZ, ABSOLUTE_ERROR);
        assertEquals(scaleZ, scaleW, ABSOLUTE_ERROR);
        assertEquals(scaleW, scaleX, ABSOLUTE_ERROR);

        scaleX = rotPoint2A.getHomX() / rotPoint2Mat.getElementAtIndex(0);
        scaleY = rotPoint2A.getHomY() / rotPoint2Mat.getElementAtIndex(1);
        scaleZ = rotPoint2A.getHomZ() / rotPoint2Mat.getElementAtIndex(2);
        scaleW = rotPoint2A.getHomW() / rotPoint2Mat.getElementAtIndex(3);
        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleZ, ABSOLUTE_ERROR);
        assertEquals(scaleZ, scaleW, ABSOLUTE_ERROR);
        assertEquals(scaleW, scaleX, ABSOLUTE_ERROR);

        scaleX = rotPoint3A.getHomX() / rotPoint3Mat.getElementAtIndex(0);
        scaleY = rotPoint3A.getHomY() / rotPoint3Mat.getElementAtIndex(1);
        scaleZ = rotPoint3A.getHomZ() / rotPoint3Mat.getElementAtIndex(2);
        scaleW = rotPoint3A.getHomW() / rotPoint3Mat.getElementAtIndex(3);
        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleZ, ABSOLUTE_ERROR);
        assertEquals(scaleZ, scaleW, ABSOLUTE_ERROR);
        assertEquals(scaleW, scaleX, ABSOLUTE_ERROR);

        // ensure that points where correctly rotated using inhomogeneous
        // coordinates
        final Matrix inhomPoint = new Matrix(INHOM_COORDS, 1);
        inhomPoint.setElementAtIndex(0, point1.getInhomX());
        inhomPoint.setElementAtIndex(1, point1.getInhomY());
        inhomPoint.setElementAtIndex(2, point1.getInhomZ());
        final Matrix inhomR = q.asInhomogeneousMatrix();
        final Matrix inhomRotPoint = inhomR.multiplyAndReturnNew(inhomPoint);
        final Point3D rotP = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES,
                inhomRotPoint.toArray());
        assertTrue(rotP.equals(rotPoint1A, ABSOLUTE_ERROR));

        final Plane rotPlaneA = q.rotate(plane);
        final Plane rotPlaneB = new Plane();
        q.rotate(plane, rotPlaneB);

        // check both rotated lines are equal
        double scaleA = rotPlaneA.getA() / rotPlaneB.getA();
        double scaleB = rotPlaneA.getB() / rotPlaneB.getB();
        double scaleC = rotPlaneA.getC() / rotPlaneB.getC();
        double scaleD = rotPlaneA.getD() / rotPlaneB.getD();

        assertEquals(scaleA, scaleB, ABSOLUTE_ERROR);
        assertEquals(scaleB, scaleC, ABSOLUTE_ERROR);
        assertEquals(scaleC, scaleD, ABSOLUTE_ERROR);
        assertEquals(scaleD, scaleA, ABSOLUTE_ERROR);

        // check plane has been correctly rotated by ensuring that rotated points
        // belong into rotated line
        assertTrue(rotPlaneA.isLocus(rotPoint1A, ABSOLUTE_ERROR));
        assertTrue(rotPlaneA.isLocus(rotPoint1B, ABSOLUTE_ERROR));
        assertTrue(rotPlaneA.isLocus(rotPoint2A, ABSOLUTE_ERROR));
        assertTrue(rotPlaneA.isLocus(rotPoint2B, ABSOLUTE_ERROR));
        assertTrue(rotPlaneA.isLocus(rotPoint3A, ABSOLUTE_ERROR));
        assertTrue(rotPlaneA.isLocus(rotPoint3B, ABSOLUTE_ERROR));

        // and by ensuring that rotated plane follow appropriate equation
        final Matrix planeMat = Matrix.newFromArray(plane.asArray(), true);
        final Matrix rotPlaneMat = R.multiplyAndReturnNew(planeMat);

        scaleA = rotPlaneA.getA() / rotPlaneMat.getElementAtIndex(0);
        scaleB = rotPlaneA.getB() / rotPlaneMat.getElementAtIndex(1);
        scaleC = rotPlaneA.getC() / rotPlaneMat.getElementAtIndex(2);
        scaleD = rotPlaneA.getD() / rotPlaneMat.getElementAtIndex(3);

        assertEquals(scaleA, scaleB, ABSOLUTE_ERROR);
        assertEquals(scaleB, scaleC, ABSOLUTE_ERROR);
        assertEquals(scaleC, scaleD, ABSOLUTE_ERROR);
        assertEquals(scaleD, scaleA, ABSOLUTE_ERROR);
    }

    @Test
    public void testMatrixRotationToQuaternionAndSetFromMatrixRotation()
            throws AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double roll = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double yaw = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        final MatrixRotation3D matrixRot = new MatrixRotation3D();
        matrixRot.setRollPitchYaw(roll, pitch, yaw);

        Quaternion q = new Quaternion();

        // test
        Quaternion.matrixRotationToQuaternion(matrixRot.internalMatrix, q);

        // check correctness
        //noinspection AssertBetweenInconvertibleTypes
        assertEquals(matrixRot, q);

        // Force IllegalArgumentException
        try {
            Quaternion.matrixRotationToQuaternion(new Matrix(1, 1), q);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        // test
        q = new Quaternion();
        Quaternion.matrixRotationToQuaternion(matrixRot, q);

        // check correctness
        //noinspection AssertBetweenInconvertibleTypes
        assertEquals(matrixRot, q);

        // test
        q = new Quaternion();
        q.setFromMatrixRotation(matrixRot.internalMatrix);

        // check correctness
        //noinspection AssertBetweenInconvertibleTypes
        assertEquals(matrixRot, q);

        // Force IllegalArgumentException
        try {
            q.setFromMatrixRotation(new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        // test
        q = new Quaternion();
        q.setFromMatrixRotation(matrixRot);

        // check correctness
        //noinspection AssertBetweenInconvertibleTypes
        assertEquals(matrixRot, q);
    }

    @Test
    public void testRotationVectorToRotationAxisAndAngle()
            throws AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        // Find any 3 orthogonal vectors, 1st will be axis of rotation, and
        // the remaining two will lie on the rotation plane and will be used
        // to test theta angle

        // To find 3 orthogonal vectors, we use V matrix of a singular
        // decomposition of any Nx3 matrix
        final Matrix aM = Matrix.createWithUniformRandomValues(1, ROTATION_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final SingularValueDecomposer decomposer = new SingularValueDecomposer(aM);

        decomposer.decompose();

        final Matrix vMatrix = decomposer.getV();

        // axis of rotation
        final double[] axis = vMatrix.getSubmatrixAsArray(
                0, 0, 2, 0);

        final double[] rotationVector = ArrayUtils.multiplyByScalarAndReturnNew(axis,
                theta);

        final Matrix jacobianAlpha = new Matrix(1, 3);
        final Matrix jacobianRotationVector = new Matrix(3, 3);

        final double[] axis2 = new double[axis.length];
        final double theta2 = Quaternion.rotationVectorToRotationAxisAndAngle(
                rotationVector, axis2, jacobianAlpha, jacobianRotationVector);

        assertArrayEquals(axis, axis2, ABSOLUTE_ERROR);
        assertEquals(theta, theta2, ABSOLUTE_ERROR);

        assertArrayEquals(jacobianAlpha.getBuffer(), axis, ABSOLUTE_ERROR);

        assertEquals(jacobianRotationVector.getElementAt(0, 0),
                1 / theta - axis[0] * axis[0] / theta, ABSOLUTE_ERROR);
        assertEquals(jacobianRotationVector.getElementAt(1, 0),
                -axis[0] / theta * axis[1], ABSOLUTE_ERROR);
        assertEquals(jacobianRotationVector.getElementAt(2, 0),
                -axis[0] / theta * axis[2], ABSOLUTE_ERROR);

        assertEquals(jacobianRotationVector.getElementAt(0, 1),
                -axis[0] / theta * axis[1], ABSOLUTE_ERROR);
        assertEquals(jacobianRotationVector.getElementAt(1, 1),
                1 / theta - axis[1] * axis[1] / theta, ABSOLUTE_ERROR);
        assertEquals(jacobianRotationVector.getElementAt(2, 1),
                -axis[1] / theta * axis[2], ABSOLUTE_ERROR);

        assertEquals(jacobianRotationVector.getElementAt(0, 2),
                -axis[0] / theta * axis[2], ABSOLUTE_ERROR);
        assertEquals(jacobianRotationVector.getElementAt(1, 2),
                -axis[1] / theta * axis[2], ABSOLUTE_ERROR);
        assertEquals(jacobianRotationVector.getElementAt(2, 2),
                1 / theta - axis[2] * axis[2] / theta, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            Quaternion.rotationVectorToRotationAxisAndAngle(new double[1],
                    axis2, jacobianAlpha, jacobianRotationVector);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            Quaternion.rotationVectorToRotationAxisAndAngle(rotationVector,
                    new double[1], jacobianAlpha, jacobianRotationVector);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            Quaternion.rotationVectorToRotationAxisAndAngle(rotationVector,
                    axis2, new Matrix(1, 1), jacobianRotationVector);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            Quaternion.rotationVectorToRotationAxisAndAngle(rotationVector,
                    axis2, jacobianAlpha, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        final double[] axis3 = new double[axis.length];
        final double theta3 = Quaternion.rotationVectorToRotationAxisAndAngle(
                rotationVector, axis3);

        assertArrayEquals(axis, axis3, ABSOLUTE_ERROR);
        assertEquals(theta, theta3, ABSOLUTE_ERROR);
    }

    @Test
    public void testRotationVectorToQuaternion() throws AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        // Find any 3 orthogonal vectors, 1st will be axis of rotation, and
        // the remaining two will lie on the rotation plane and will be used
        // to test theta angle

        // To find 3 orthogonal vectors, we use V matrix of a singular
        // decomposition of any Nx3 matrix
        final Matrix aM = Matrix.createWithUniformRandomValues(1, ROTATION_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final SingularValueDecomposer decomposer = new SingularValueDecomposer(aM);

        decomposer.decompose();

        final Matrix vMatrix = decomposer.getV();

        // axis of rotation
        final double[] axis = vMatrix.getSubmatrixAsArray(0, 0,
                2, 0);

        final double[] rotationVector = ArrayUtils.multiplyByScalarAndReturnNew(axis,
                theta);

        Quaternion q = new Quaternion();
        Matrix jacobian = new Matrix(Quaternion.N_PARAMS, Quaternion.N_ANGLES);

        Quaternion.rotationVectorToQuaternion(rotationVector, q, jacobian);

        // check correctness
        double[] rotationVector2 = new double[Quaternion.N_ANGLES];
        q.toRotationVector(rotationVector2);
        assertArrayEquals(rotationVector, rotationVector2, ABSOLUTE_ERROR);

        if (theta < Quaternion.LARGE_AXIS_NORM_THRESHOLD) {
            assertEquals(jacobian.getElementAt(0, 0), -0.25 * rotationVector[0],
                    ABSOLUTE_ERROR);
            assertEquals(jacobian.getElementAt(1, 0), -0.25 * rotationVector[1],
                    ABSOLUTE_ERROR);
            assertEquals(jacobian.getElementAt(2, 0), -0.25 * rotationVector[2],
                    ABSOLUTE_ERROR);

            assertEquals(jacobian.getElementAt(0, 1), 0.5, 0.0);
            assertEquals(jacobian.getElementAt(1, 1), 0.0, 0.0);
            assertEquals(jacobian.getElementAt(2, 1), 0.0, 0.0);

            assertEquals(jacobian.getElementAt(0, 2), 0.0, 0.0);
            assertEquals(jacobian.getElementAt(1, 2), 0.5, 0.0);
            assertEquals(jacobian.getElementAt(2, 2), 0.0, 0.0);

            assertEquals(jacobian.getElementAt(0, 3), 0.0, 0.0);
            assertEquals(jacobian.getElementAt(1, 3), 0.0, 0.0);
            assertEquals(jacobian.getElementAt(2, 3), 0.5, 0.0);
        } else {
            final Matrix jacobianAlpha = new Matrix(1, Quaternion.N_ANGLES);
            final Matrix jacobianRotationVector = new Matrix(Quaternion.N_ANGLES,
                    Quaternion.N_ANGLES);
            Quaternion.rotationVectorToRotationAxisAndAngle(rotationVector,
                    axis, jacobianAlpha, jacobianRotationVector);

            final Matrix jacobianOfTheta = new Matrix(Quaternion.N_PARAMS, 1);
            final Matrix jacobianOfAxis = new Matrix(Quaternion.N_PARAMS,
                    Quaternion.N_ANGLES);
            final Quaternion tmp = new Quaternion();
            tmp.setFromAxisAndRotation(axis, theta, jacobianOfTheta,
                    jacobianOfAxis);

            final Matrix jacobian2 = jacobianOfTheta.multiplyAndReturnNew(
                    jacobianAlpha).addAndReturnNew(
                    jacobianOfAxis.multiplyAndReturnNew(
                            jacobianRotationVector));

            assertTrue(jacobian.equals(jacobian2, ABSOLUTE_ERROR));
        }

        // Force IllegalArgumentException
        try {
            Quaternion.rotationVectorToQuaternion(new double[1], q, jacobian);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            Quaternion.rotationVectorToQuaternion(rotationVector, q,
                    new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        q = new Quaternion();
        Quaternion.rotationVectorToQuaternion(rotationVector, q);

        // check correctness
        rotationVector2 = new double[Quaternion.N_ANGLES];
        q.toRotationVector(rotationVector2);
        assertArrayEquals(rotationVector, rotationVector2, ABSOLUTE_ERROR);

        // check correctness of jacobian
        q = new Quaternion();
        jacobian = new Matrix(Quaternion.N_PARAMS, Quaternion.N_ANGLES);

        Quaternion.rotationVectorToQuaternion(rotationVector, q, jacobian);

        // check rotation vector variation
        final double[] diff = new double[3];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        rotationVector2 = ArrayUtils.sumAndReturnNew(rotationVector, diff);

        final Quaternion q2 = new Quaternion();
        Quaternion.rotationVectorToQuaternion(rotationVector2, q2);

        final double[] diffResult = new double[]{
                q2.getA() - q.getA(),
                q2.getB() - q.getB(),
                q2.getC() - q.getC(),
                q2.getD() - q.getD()
        };
        final double[] diffResult2 = jacobian.multiplyAndReturnNew(
                Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);
    }

    @Test
    public void testSetFromRotationVector() throws AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        // Find any 3 orthogonal vectors, 1st will be axis of rotation, and
        // the remaining two will lie on the rotation plane and will be used
        // to test theta angle

        // To find 3 orthogonal vectors, we use V matrix of a singular
        // decomposition of any Nx3 matrix
        final Matrix aM = Matrix.createWithUniformRandomValues(1, ROTATION_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final SingularValueDecomposer decomposer = new SingularValueDecomposer(aM);

        decomposer.decompose();

        final Matrix vMatrix = decomposer.getV();

        // axis of rotation
        final double[] axis = vMatrix.getSubmatrixAsArray(0, 0,
                2, 0);

        final double[] rotationVector = ArrayUtils.multiplyByScalarAndReturnNew(axis,
                theta);

        final Quaternion q = new Quaternion();

        // set value
        q.setFromRotationVector(rotationVector);

        // check correctness
        final double[] rotationVector2 = new double[Quaternion.N_ANGLES];
        q.toRotationVector(rotationVector2);
        assertArrayEquals(rotationVector, rotationVector2, ABSOLUTE_ERROR);
    }

    @Test
    public void testRotationVectorToMatrixRotation() throws AlgebraException,
            RotationException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        // Find any 3 orthogonal vectors, 1st will be axis of rotation, and
        // the remaining two will lie on the rotation plane and will be used
        // to test theta angle

        // To find 3 orthogonal vectors, we use V matrix of a singular
        // decomposition of any Nx3 matrix
        final Matrix aM = Matrix.createWithUniformRandomValues(1, ROTATION_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final SingularValueDecomposer decomposer = new SingularValueDecomposer(aM);

        decomposer.decompose();

        final Matrix vMatrix = decomposer.getV();

        // axis of rotation
        final double[] axis = vMatrix.getSubmatrixAsArray(
                0, 0, 2, 0);

        final double[] rotationVector = ArrayUtils.multiplyByScalarAndReturnNew(axis,
                theta);

        final Matrix m = new Matrix(MatrixRotation3D.ROTATION3D_INHOM_MATRIX_ROWS,
                MatrixRotation3D.ROTATION3D_INHOM_MATRIX_COLS);

        Quaternion.rotationVectorToMatrixRotation(rotationVector, m);

        final MatrixRotation3D matrixRot = new MatrixRotation3D();
        Quaternion.rotationVectorToMatrixRotation(rotationVector, matrixRot);

        // check correctness
        assertEquals(matrixRot.internalMatrix, m);

        final double[] axis2 = matrixRot.getRotationAxis();
        final double theta2 = matrixRot.getRotationAngle();

        for (int i = 0; i < axis.length; i++) {
            assertEquals(Math.abs(axis[i]), Math.abs(axis2[i]), ABSOLUTE_ERROR);
        }
        assertEquals(Math.abs(theta), Math.abs(theta2), ABSOLUTE_ERROR);
    }

    @Test
    public void testGetType() {
        final Quaternion q = new Quaternion();
        assertEquals(q.getType(), Rotation3DType.QUATERNION);
    }

    @Test
    public void testSetAxisAndRotationRotationAxisAndGetRotationAngle()
            throws AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        // Find any 3 orthogonal vectors, 1st will be axis of rotation, and
        // the remaining two will lie on the rotation plane and will be used
        // to test theta angle

        // To find 3 orthogonal vectors, we use V matrix of a singular
        // decomposition of any Nx3 matrix
        final Matrix aM = Matrix.createWithUniformRandomValues(1, ROTATION_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final SingularValueDecomposer decomposer = new SingularValueDecomposer(aM);

        decomposer.decompose();

        final Matrix vMatrix = decomposer.getV();

        // axis of rotation
        final double[] axis = vMatrix.getSubmatrixAsArray(0, 0,
                2, 0);

        final Quaternion q = new Quaternion();
        q.setAxisAndRotation(axis[0], axis[1], axis[2], theta);

        // check correctness
        final double[] axis2 = new double[Quaternion.N_ANGLES];
        q.rotationAxis(axis2);

        assertArrayEquals(axis, axis2, ABSOLUTE_ERROR);
        assertEquals(theta, q.getRotationAngle(), ABSOLUTE_ERROR);
    }

    @Test
    public void testAsInhomogeneousMatrix() throws AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        // Find any 3 orthogonal vectors, 1st will be axis of rotation, and
        // the remaining two will lie on the rotation plane and will be used
        // to test theta angle

        // To find 3 orthogonal vectors, we use V matrix of a singular
        // decomposition of any Nx3 matrix
        final Matrix aM = Matrix.createWithUniformRandomValues(1, ROTATION_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final SingularValueDecomposer decomposer = new SingularValueDecomposer(aM);

        decomposer.decompose();

        final Matrix vMatrix = decomposer.getV();

        // axis of rotation
        final double[] axis = vMatrix.getSubmatrixAsArray(0, 0,
                2, 0);

        final Quaternion q = new Quaternion(axis, theta);
        final MatrixRotation3D rot = new MatrixRotation3D(axis, theta);

        Matrix m1 = q.asInhomogeneousMatrix();
        Matrix m2 = rot.asInhomogeneousMatrix();
        assertTrue(m1.equals(m2, ABSOLUTE_ERROR));

        m1 = new Matrix(MatrixRotation3D.ROTATION3D_INHOM_MATRIX_ROWS,
                MatrixRotation3D.ROTATION3D_INHOM_MATRIX_COLS);
        m2 = new Matrix(MatrixRotation3D.ROTATION3D_INHOM_MATRIX_ROWS,
                MatrixRotation3D.ROTATION3D_INHOM_MATRIX_COLS);

        q.asInhomogeneousMatrix(m1);
        rot.asInhomogeneousMatrix(m2);

        assertTrue(m1.equals(m2, ABSOLUTE_ERROR));
    }

    @Test
    public void testAsHomogeneousMatrix() throws AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        // Find any 3 orthogonal vectors, 1st will be axis of rotation, and
        // the remaining two will lie on the rotation plane and will be used
        // to test theta angle

        // To find 3 orthogonal vectors, we use V matrix of a singular
        // decomposition of any Nx3 matrix
        final Matrix aM = Matrix.createWithUniformRandomValues(1, ROTATION_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final SingularValueDecomposer decomposer = new SingularValueDecomposer(aM);

        decomposer.decompose();

        final Matrix vMatrix = decomposer.getV();

        // axis of rotation
        final double[] axis = vMatrix.getSubmatrixAsArray(0, 0,
                2, 0);

        final Quaternion q = new Quaternion(axis, theta);
        final MatrixRotation3D rot = new MatrixRotation3D(axis, theta);

        Matrix m1 = q.asHomogeneousMatrix();
        Matrix m2 = rot.asHomogeneousMatrix();
        assertTrue(m1.equals(m2, ABSOLUTE_ERROR));

        m1 = new Matrix(MatrixRotation3D.ROTATION3D_HOM_MATRIX_ROWS,
                MatrixRotation3D.ROTATION3D_HOM_MATRIX_COLS);
        m2 = new Matrix(MatrixRotation3D.ROTATION3D_HOM_MATRIX_ROWS,
                MatrixRotation3D.ROTATION3D_HOM_MATRIX_COLS);

        q.asHomogeneousMatrix(m1);
        rot.asHomogeneousMatrix(m2);

        assertTrue(m1.equals(m2, ABSOLUTE_ERROR));
    }

    @Test
    public void testFromInhomogeneousMatrix() throws AlgebraException,
            InvalidRotationMatrixException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        // Find any 3 orthogonal vectors, 1st will be axis of rotation, and
        // the remaining two will lie on the rotation plane and will be used
        // to test theta angle

        // To find 3 orthogonal vectors, we use V matrix of a singular
        // decomposition of any Nx3 matrix
        final Matrix aM = Matrix.createWithUniformRandomValues(1, ROTATION_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final SingularValueDecomposer decomposer = new SingularValueDecomposer(aM);

        decomposer.decompose();

        final Matrix vMatrix = decomposer.getV();

        // axis of rotation
        final double[] axis = vMatrix.getSubmatrixAsArray(0, 0,
                2, 0);

        final Quaternion q1 = new Quaternion(axis, theta);
        final Matrix m1 = q1.asInhomogeneousMatrix();

        final Quaternion q2 = new Quaternion();
        q2.fromInhomogeneousMatrix(m1);

        // check correctness
        assertEquals(q1, q2);
        assertTrue(m1.equals(q2.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
    }

    @Test
    public void testFromHomogeneousMatrix() throws AlgebraException,
            InvalidRotationMatrixException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        // Find any 3 orthogonal vectors, 1st will be axis of rotation, and
        // the remaining two will lie on the rotation plane and will be used
        // to test theta angle

        // To find 3 orthogonal vectors, we use V matrix of a singular
        // decomposition of any Nx3 matrix
        final Matrix aM = Matrix.createWithUniformRandomValues(1, ROTATION_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final SingularValueDecomposer decomposer = new SingularValueDecomposer(aM);

        decomposer.decompose();

        final Matrix vMatrix = decomposer.getV();

        // axis of rotation
        final double[] axis = vMatrix.getSubmatrixAsArray(0, 0,
                2, 0);

        final Quaternion q1 = new Quaternion(axis, theta);
        final Matrix m1 = q1.asHomogeneousMatrix();

        final Quaternion q2 = new Quaternion();
        q2.fromHomogeneousMatrix(m1);

        // check correctness
        assertEquals(q1, q2);
        assertTrue(m1.equals(q2.asHomogeneousMatrix(), ABSOLUTE_ERROR));
    }

    @Test
    public void testInverse() throws AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        // Find any 3 orthogonal vectors, 1st will be axis of rotation, and
        // the remaining two will lie on the rotation plane and will be used
        // to test theta angle

        // To find 3 orthogonal vectors, we use V matrix of a singular
        // decomposition of any Nx3 matrix
        final Matrix aM = Matrix.createWithUniformRandomValues(1, ROTATION_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final SingularValueDecomposer decomposer = new SingularValueDecomposer(aM);

        decomposer.decompose();

        final Matrix vMatrix = decomposer.getV();

        // axis of rotation
        final double[] axis = vMatrix.getSubmatrixAsArray(0, 0,
                2, 0);

        final Quaternion q = new Quaternion(axis, theta);
        final Quaternion invQ1 = new Quaternion();
        Quaternion.inverse(q, invQ1);

        final Quaternion invQ2 = Quaternion.inverseAndReturnNew(q);

        final Quaternion invQ3 = new Quaternion();
        q.inverse(invQ3);

        final Quaternion invQ4 = q.inverseAndReturnNew();

        // check correctness
        assertEquals(invQ1, invQ2);
        assertEquals(invQ2, invQ3);
        assertEquals(invQ3, invQ4);
        assertEquals(invQ4, invQ1);

        final Quaternion prodQ = q.multiplyAndReturnNew(invQ1);

        assertEquals(prodQ.getA(), 1.0, ABSOLUTE_ERROR);
        assertEquals(prodQ.getB(), 0.0, ABSOLUTE_ERROR);
        assertEquals(prodQ.getC(), 0.0, ABSOLUTE_ERROR);
        assertEquals(prodQ.getD(), 0.0, ABSOLUTE_ERROR);

        final HomogeneousPoint3D point = new HomogeneousPoint3D();
        point.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        // rotate point
        final Point3D rotPoint = q.rotate(point);

        // rotate with inverse quaternion
        final Point3D point2 = invQ1.rotate(rotPoint);

        // check that both points are equal
        assertTrue(point.equals(point2, ABSOLUTE_ERROR));

        // inverse q
        q.inverse();

        // rotate again rotated point
        final Point3D point3 = q.rotate(rotPoint);

        // check that both points are equal
        assertTrue(point.equals(point3, ABSOLUTE_ERROR));
    }

    @Test
    public void testInverseRotation() throws AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        // Find any 3 orthogonal vectors, 1st will be axis of rotation, and
        // the remaining two will lie on the rotation plane and will be used
        // to test theta angle

        // To find 3 orthogonal vectors, we use V matrix of a singular
        // decomposition of any Nx3 matrix
        final Matrix aM = Matrix.createWithUniformRandomValues(1, ROTATION_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final SingularValueDecomposer decomposer = new SingularValueDecomposer(aM);

        decomposer.decompose();

        final Matrix vMatrix = decomposer.getV();

        // axis of rotation
        final double[] axis = vMatrix.getSubmatrixAsArray(0, 0,
                2, 0);

        final Quaternion q = new Quaternion(axis, theta);

        final Rotation3D invQ1 = q.inverseRotationAndReturnNew();
        final Quaternion invQ2 = new Quaternion();
        q.inverseRotation(invQ2);
        final MatrixRotation3D invRot = new MatrixRotation3D();
        q.inverseRotation(invRot);

        // check correctness
        assertEquals(invQ1, invQ2);
        //noinspection AssertBetweenInconvertibleTypes
        assertEquals(invQ2, invRot);
        assertEquals(invRot, invQ1);

        final Quaternion prodQ = q.multiplyAndReturnNew(invQ2);

        assertEquals(prodQ.getA(), 1.0, ABSOLUTE_ERROR);
        assertEquals(prodQ.getB(), 0.0, ABSOLUTE_ERROR);
        assertEquals(prodQ.getC(), 0.0, ABSOLUTE_ERROR);
        assertEquals(prodQ.getD(), 0.0, ABSOLUTE_ERROR);

        final HomogeneousPoint3D point = new HomogeneousPoint3D();
        point.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        // rotate point
        final Point3D rotPoint = q.rotate(point);

        // rotate with inverse quaternion
        final Point3D point2 = invQ2.rotate(rotPoint);

        // check that both points are equal
        assertTrue(point.equals(point2, ABSOLUTE_ERROR));

        // inverse q
        q.inverseRotation();

        // rotate again rotated point
        final Point3D point3 = q.rotate(rotPoint);

        // check that both points are equal
        assertTrue(point.equals(point3, ABSOLUTE_ERROR));
    }

    @Test
    public void testCombine() throws AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double roll1 = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double pitch1 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double yaw1 = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        final double roll2 = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double pitch2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double yaw2 = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        final Quaternion q1 = new Quaternion(roll1, pitch1, yaw1);
        final Quaternion q2 = new Quaternion(roll2, pitch2, yaw2);

        final Quaternion q = new Quaternion();
        Quaternion.combine(q1, q2, q);

        assertTrue(q.asInhomogeneousMatrix().equals(q1.asInhomogeneousMatrix().
                        multiplyAndReturnNew(q2.asInhomogeneousMatrix()),
                ABSOLUTE_ERROR));

        final Quaternion q3 = q.combineAndReturnNew(q2);
        assertTrue(q3.asInhomogeneousMatrix().equals(q.asInhomogeneousMatrix().
                        multiplyAndReturnNew(q2.asInhomogeneousMatrix()),
                ABSOLUTE_ERROR));

        final MatrixRotation3D rot2 = q2.toMatrixRotation();
        final Rotation3D rot3 = q.combineAndReturnNew(rot2);

        assertEquals(q3, rot3);

        q1.combine(q2);
        assertEquals(q1, q);

        q.combine(rot2);
        assertEquals(q3, q);
    }

    @Test
    public void testFromRotation() throws AlgebraException, RotationException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        // Find any 3 orthogonal vectors, 1st will be axis of rotation, and
        // the remaining two will lie on the rotation plane and will be used
        // to test theta angle

        // To find 3 orthogonal vectors, we use V matrix of a singular
        // decomposition of any Nx3 matrix
        final Matrix aM = Matrix.createWithUniformRandomValues(1, ROTATION_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final SingularValueDecomposer decomposer = new SingularValueDecomposer(aM);

        decomposer.decompose();

        final Matrix vMatrix = decomposer.getV();

        // axis of rotation
        final double[] axis = vMatrix.getSubmatrixAsArray(0, 0,
                2, 0);

        final MatrixRotation3D matrixRot = new MatrixRotation3D(axis, theta);
        final AxisRotation3D axisRot = new AxisRotation3D(axis, theta);
        final Quaternion qRot = new Quaternion(axis, theta);

        // from matrix rotation
        Quaternion q = new Quaternion();
        q.fromRotation(matrixRot);

        // check correctness
        //noinspection AssertBetweenInconvertibleTypes
        assertEquals(q, matrixRot);
        assertArrayEquals(q.getRotationAxis(), axis, ABSOLUTE_ERROR);
        assertEquals(q.getRotationAngle(), theta, ABSOLUTE_ERROR);

        // from axis rotation
        q = new Quaternion();
        q.fromRotation(axisRot);

        // check correctness
        //noinspection AssertBetweenInconvertibleTypes
        assertEquals(q, axisRot);
        assertArrayEquals(q.getRotationAxis(), axis, ABSOLUTE_ERROR);
        assertEquals(q.getRotationAngle(), theta, ABSOLUTE_ERROR);

        // from quaternion
        q = new Quaternion();
        q.fromRotation(qRot);

        // check correctness
        assertEquals(q, qRot);
        assertArrayEquals(q.getRotationAxis(), axis, ABSOLUTE_ERROR);
        assertEquals(q.getRotationAngle(), theta, ABSOLUTE_ERROR);
    }

    @Test
    public void testToQuaternion() throws AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        // Find any 3 orthogonal vectors, 1st will be axis of rotation, and
        // the remaining two will lie on the rotation plane and will be used
        // to test theta angle

        // To find 3 orthogonal vectors, we use V matrix of a singular
        // decomposition of any Nx3 matrix
        final Matrix aM = Matrix.createWithUniformRandomValues(1, ROTATION_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final SingularValueDecomposer decomposer = new SingularValueDecomposer(aM);

        decomposer.decompose();

        final Matrix vMatrix = decomposer.getV();

        // axis of rotation
        final double[] axis = vMatrix.getSubmatrixAsArray(0, 0,
                2, 0);

        final Quaternion q1 = new Quaternion(axis, theta);
        final Quaternion q2 = new Quaternion();
        q1.toQuaternion(q2);

        assertEquals(q1, q2);
    }

    @Test
    public void testNormalize() throws AlgebraException, RotationException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        // Find any 3 orthogonal vectors, 1st will be axis of rotation, and
        // the remaining two will lie on the rotation plane and will be used
        // to test theta angle

        // To find 3 orthogonal vectors, we use V matrix of a singular
        // decomposition of any Nx3 matrix
        final Matrix aM = Matrix.createWithUniformRandomValues(1, ROTATION_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final SingularValueDecomposer decomposer = new SingularValueDecomposer(aM);

        decomposer.decompose();

        final Matrix vMatrix = decomposer.getV();

        // axis of rotation
        final double[] axis = vMatrix.getSubmatrixAsArray(
                0, 0, 2, 0);

        Quaternion q1 = new Quaternion(axis, theta);
        final Quaternion q2 = new Quaternion(axis, theta);
        assertFalse(q1.isNormalized());
        assertFalse(q2.isNormalized());

        // normalize
        q1.normalize();
        q1.normalize();

        // after normalization both quaternions are equivalent
        assertTrue(q1.equals(q2, ABSOLUTE_ERROR));

        assertEquals(Math.sqrt(q1.getA() * q1.getA() + q1.getB() * q1.getB() +
                        q1.getC() * q1.getC() + q1.getD() * q1.getD()), 1.0,
                ABSOLUTE_ERROR);

        assertTrue(q1.isNormalized());
        assertFalse(q2.isNormalized());


        // normalize with jacobian
        q1 = new Quaternion(axis, theta);
        assertFalse(q1.isNormalized());
        final double a = q1.getA();
        final double b = q1.getB();
        final double c = q1.getC();
        final double d = q1.getD();
        final double norm = Math.sqrt(a * a + b * b + c * c + d * d);

        final Matrix jacobian = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
        q1.normalize(jacobian);

        // check correctness
        assertTrue(q1.equals(q2, ABSOLUTE_ERROR));

        assertEquals(Math.sqrt(q1.getA() * q1.getA() + q1.getB() * q1.getB() +
                        q1.getC() * q1.getC() + q1.getD() * q1.getD()), 1.0,
                ABSOLUTE_ERROR);

        assertTrue(q1.isNormalized());
        assertFalse(q2.isNormalized());

        // check jacobian
        final double norm3 = norm * norm * norm;

        final Matrix jacobian2 = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);

        jacobian2.setElementAtIndex(0, (b * b + c * c + d * d) / norm3);
        jacobian2.setElementAtIndex(1, -a / norm3 * b);
        jacobian2.setElementAtIndex(2, -a / norm3 * c);
        jacobian2.setElementAtIndex(3, -a / norm3 * d);

        jacobian2.setElementAtIndex(4, -a / norm3 * b);
        jacobian2.setElementAtIndex(5, (a * a + c * c + d * d) / norm3);
        jacobian2.setElementAtIndex(6, -b / norm3 * c);
        jacobian2.setElementAtIndex(7, -b / norm3 * d);

        jacobian2.setElementAtIndex(8, -a / norm3 * c);
        jacobian2.setElementAtIndex(9, -b / norm3 * c);
        jacobian2.setElementAtIndex(10, (a * a + b * b + d * d) / norm3);
        jacobian2.setElementAtIndex(11, -c / norm3 * d);

        jacobian2.setElementAtIndex(12, -a / norm3 * d);
        jacobian2.setElementAtIndex(13, -b / norm3 * d);
        jacobian2.setElementAtIndex(14, -c / norm3 * d);
        jacobian2.setElementAtIndex(15, (a * a + b * b + c * c) / norm3);

        assertEquals(jacobian, jacobian2);

        // Force IllegalArgumentException
        try {
            q1.normalize(new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testSlerp() throws RotationException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double roll1 = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double pitch1 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double yaw1 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        final double roll2 = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double pitch2 = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double yaw2 = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        final Quaternion q1 = new Quaternion(roll1, pitch1, yaw1);
        final Quaternion q2 = new Quaternion(roll2, pitch2, yaw2);


        final Quaternion q3a = q1.slerpAndReturnNew(q2, 0.0);
        final Quaternion q3b = new Quaternion();
        q1.slerp(q2, 0.0, q3b);
        final Quaternion q3c = Quaternion.slerpAndReturnNew(q1, q2, 0.0);
        final Quaternion q3d = new Quaternion();
        Quaternion.slerp(q1, q2, 0.0, q3d);

        assertTrue(q3a.equals(q1, ABSOLUTE_ERROR));
        assertTrue(q3b.equals(q1, ABSOLUTE_ERROR));
        assertTrue(q3c.equals(q1, ABSOLUTE_ERROR));
        assertTrue(q3d.equals(q1, ABSOLUTE_ERROR));

        final Quaternion q4a = q1.slerpAndReturnNew(q2, 1.0);
        final Quaternion q4b = new Quaternion();
        q1.slerp(q2, 1.0, q4b);
        final Quaternion q4c = Quaternion.slerpAndReturnNew(q1, q2, 1.0);
        final Quaternion q4d = new Quaternion();
        Quaternion.slerp(q1, q2, 1.0, q4d);

        assertTrue(q4a.equals(q2, ABSOLUTE_ERROR));
        assertTrue(q4b.equals(q2, ABSOLUTE_ERROR));
        assertTrue(q4c.equals(q2, ABSOLUTE_ERROR));
        assertTrue(q4d.equals(q2, ABSOLUTE_ERROR));

        final Quaternion q5a = q2.slerpAndReturnNew(q1, 0.0);
        final Quaternion q5b = new Quaternion();
        q2.slerp(q1, 0.0, q5b);
        final Quaternion q5c = Quaternion.slerpAndReturnNew(q2, q1, 0.0);
        final Quaternion q5d = new Quaternion();
        Quaternion.slerp(q2, q1, 0.0, q5d);

        assertTrue(q5a.equals(q2, ABSOLUTE_ERROR));
        assertTrue(q5b.equals(q2, ABSOLUTE_ERROR));
        assertTrue(q5c.equals(q2, ABSOLUTE_ERROR));
        assertTrue(q5d.equals(q2, ABSOLUTE_ERROR));

        final Quaternion q6a = q2.slerpAndReturnNew(q1, 1.0);
        final Quaternion q6b = new Quaternion();
        q2.slerp(q1, 1.0, q6b);
        final Quaternion q6c = Quaternion.slerpAndReturnNew(q2, q1, 1.0);
        final Quaternion q6d = new Quaternion();
        Quaternion.slerp(q2, q1, 1.0, q6d);

        assertTrue(q6a.equals(q1, ABSOLUTE_ERROR));
        assertTrue(q6b.equals(q1, ABSOLUTE_ERROR));
        assertTrue(q6c.equals(q1, ABSOLUTE_ERROR));
        assertTrue(q6d.equals(q1, ABSOLUTE_ERROR));

        Quaternion result = new Quaternion();

        // use equal quaternions
        Quaternion.slerp(q1, q1, 0.0, result);
        assertTrue(result.equals(q1));

        Quaternion.slerp(q1, q1, 1.0, result);
        assertTrue(result.equals(q1));

        // force IllegalArgumentException
        result = new Quaternion();
        try {
            Quaternion.slerp(q1, q2, -1.0, result);
            fail("IllegalArgumentException expected");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            Quaternion.slerp(q1, q2, 2.0, result);
            fail("IllegalArgumentException expected");
        } catch (final IllegalArgumentException ignore) {
        }
        assertTrue(result.equals(new Quaternion()));
    }

    @Test
    public void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final Quaternion q1 = new Quaternion(a, b, c, d);

        // check
        assertEquals(q1.getA(), a, 0.0);
        assertEquals(q1.getB(), b, 0.0);
        assertEquals(q1.getC(), c, 0.0);
        assertEquals(q1.getD(), d, 0.0);
        assertFalse(q1.isNormalized());

        // serialize and deserialize
        final byte[] bytes = SerializationHelper.serialize(q1);
        final Quaternion q2 = SerializationHelper.deserialize(bytes);

        // check
        assertEquals(q1, q2);
        assertNotSame(q1, q2);
    }
}
