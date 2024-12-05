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

import com.irurueta.algebra.*;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

class QuaternionTest {
    private static final int ROTATION_COLS = 3;
    private static final int INHOM_COORDS = 3;

    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    private static final double MIN_ANGLE_DEGREES = 0.0;
    private static final double MAX_ANGLE_DEGREES = 90.0;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final double JACOBIAN_ERROR = 1e-6;

    @Test
    void testConstants() {
        assertEquals(4, Quaternion.N_PARAMS);
        assertEquals(3, Quaternion.N_ANGLES);
        assertEquals(1e-7, Quaternion.AXIS_NORM_THRESHOLD, 0.0);
        assertEquals(1e-6, Quaternion.LARGE_AXIS_NORM_THRESHOLD, 0.0);
        assertEquals(1e-8, Quaternion.TRACE_THRESHOLD, 0.0);
    }

    @Test
    void testConstructors() throws AlgebraException, RotationException {
        // empty constructor
        var q = new Quaternion();

        // check correctness
        assertEquals(1.0, q.getA(), 0.0);
        assertEquals(0.0, q.getB(), 0.0);
        assertEquals(0.0, q.getC(), 0.0);
        assertEquals(0.0, q.getD(), 0.0);
        assertEquals(Matrix.identity(MatrixRotation3D.INHOM_COORDS, MatrixRotation3D.INHOM_COORDS),
                q.toMatrixRotation().getInternalMatrix());

        // constructor with values
        final var randomizer = new UniformRandomizer();
        final var a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        q = new Quaternion(a, b, c, d);

        // check correctness
        assertEquals(a, q.getA(), 0.0);
        assertEquals(b, q.getB(), 0.0);
        assertEquals(c, q.getC(), 0.0);
        assertEquals(d, q.getD(), 0.0);

        // constructor from another quaternion
        var q2 = new Quaternion(q);

        // check correctness
        assertEquals(a, q2.getA(), 0.0);
        assertEquals(b, q2.getB(), 0.0);
        assertEquals(c, q2.getC(), 0.0);
        assertEquals(d, q2.getD(), 0.0);

        // constructor from values
        final var values1 = new double[]{a, b, c, d};
        q = new Quaternion(values1);

        // check correctness
        assertEquals(a, q.getA(), 0.0);
        assertEquals(b, q.getB(), 0.0);
        assertEquals(c, q.getC(), 0.0);
        assertEquals(d, q.getD(), 0.0);

        // Force IllegalArgumentException
        final var values2 = new double[]{1, a, b, c, d};
        assertThrows(IllegalArgumentException.class, () ->  new Quaternion(values2));

        // constructor from axis and rotation angle
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // Find any 3 orthogonal vectors, 1st will be axis of rotation, and
        // the remaining two will lie on the rotation plane and will be used
        // to test theta angle

        // To find 3 orthogonal vectors, we use V matrix of a singular
        // decomposition of any Nx3 matrix
        final var aM = Matrix.createWithUniformRandomValues(1, ROTATION_COLS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var decomposer = new SingularValueDecomposer(aM);

        decomposer.decompose();

        final var vMatrix = decomposer.getV();

        // axis of rotation
        final var axis = vMatrix.getSubmatrixAsArray(0, 0, 2, 0);

        final var axisRotation = new AxisRotation3D(axis, theta);

        q = new Quaternion(axis, theta);

        // check correctness
        final var axis2 = q.getRotationAxis();
        final var theta2 = q.getRotationAngle();
        assertArrayEquals(axis, axis2, ABSOLUTE_ERROR);
        assertEquals(theta, theta2, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new Quaternion(new double[]{}, theta));

        // constructor from an axis rotation
        q = new Quaternion(axisRotation);

        // check correctness
        //noinspection AssertBetweenInconvertibleTypes
        assertEquals(q, axisRotation);

        // constructor from roll, pitch and yaw angles
        final var roll = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));

        q = new Quaternion(roll, pitch, yaw);
        var matrixRotation = new MatrixRotation3D();
        matrixRotation.setRollPitchYaw(roll, pitch, yaw);

        q2 = matrixRotation.toQuaternion();

        assertEquals(q, q2);

        final var angles = new double[Quaternion.N_ANGLES];
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
    void testGetSetA() {
        final var randomizer = new UniformRandomizer();
        final var a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var q = new Quaternion();

        // check initial value
        assertEquals(1.0, q.getA(), 0.0);

        // set new values
        q.setA(a);

        // check correctness
        assertEquals(q.getA(), a, 0.0);
    }

    @Test
    void testGetSetB() {
        final var randomizer = new UniformRandomizer();
        final var b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var q = new Quaternion();

        // check initial value
        assertEquals(0.0, q.getB(), 0.0);

        // set new value
        q.setB(b);

        // check correctness
        assertEquals(q.getB(), b, 0.0);
    }

    @Test
    void testGetSetC() {
        final var randomizer = new UniformRandomizer();
        final var c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var q = new Quaternion();

        // check initial value
        assertEquals(0.0, q.getC(), 0.0);

        // set new value
        q.setC(c);

        // check correctness
        assertEquals(q.getC(), c, 0.0);
    }

    @Test
    void testGetSetD() {
        final var randomizer = new UniformRandomizer();
        final var d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var q = new Quaternion();

        // check initial value
        assertEquals(0.0, q.getD(), 0.0);

        // set new value
        q.setD(d);

        // check correctness
        assertEquals(q.getD(), d, 0.0);
    }

    @Test
    void testGetSetValues() {
        final var randomizer = new UniformRandomizer();
        final var values = new double[Quaternion.N_PARAMS];
        randomizer.fill(values, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var q = new Quaternion();

        // set values
        q.setValues(values);

        // check correctness
        final var values2 = new double[Quaternion.N_PARAMS];
        final var values3 = q.getValues();
        q.values(values2);

        assertArrayEquals(values, values2, 0.0);
        assertArrayEquals(values, values3, 0.0);

        // Force IllegalArgumentException
        final var invalid = new double[Quaternion.N_PARAMS + 1];
        assertThrows(IllegalArgumentException.class, () -> q.setValues(invalid));
    }

    @Test
    void testFromQuaternion() {
        final var randomizer = new UniformRandomizer();
        final var values = new double[Quaternion.N_PARAMS];
        randomizer.fill(values, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var q = new Quaternion(values);

        final var q2 = new Quaternion();
        q2.fromQuaternion(q);

        // check correctness
        assertArrayEquals(values, q.getValues(), 0.0);
        assertArrayEquals(values, q2.getValues(), 0.0);
        assertArrayEquals(q.getValues(), q2.getValues(), 0.0);
        assertEquals(q, q2);
    }

    @Test
    void testClone() throws CloneNotSupportedException {
        final var randomizer = new UniformRandomizer();
        final var values = new double[Quaternion.N_PARAMS];
        randomizer.fill(values, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var q = new Quaternion(values);

        final var q2 = q.clone();

        // check correctness
        assertArrayEquals(values, q.getValues(), 0.0);
        assertArrayEquals(values, q2.getValues(), 0.0);
        assertArrayEquals(q.getValues(), q2.getValues(), 0.0);
        assertEquals(q, q2);
    }

    @Test
    void testCopyTo() {
        final var randomizer = new UniformRandomizer();
        final var values = new double[Quaternion.N_PARAMS];
        randomizer.fill(values, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var q = new Quaternion(values);

        final var q2 = new Quaternion();
        q.copyTo(q2);

        // check correctness
        assertArrayEquals(values, q.getValues(), 0.0);
        assertArrayEquals(values, q2.getValues(), 0.0);
        assertArrayEquals(q.getValues(), q2.getValues(), 0.0);
        assertEquals(q, q2);
    }

    @Test
    void testGetSetFromAxisAndRotation() throws AlgebraException, RotationException {
        final var randomizer = new UniformRandomizer();
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // Find any 3 orthogonal vectors, 1st will be axis of rotation, and
        // the remaining two will lie on the rotation plane and will be used
        // to test theta angle

        // To find 3 orthogonal vectors, we use V matrix of a singular
        // decomposition of any Nx3 matrix
        final var aM = Matrix.createWithUniformRandomValues(1, ROTATION_COLS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var decomposer = new SingularValueDecomposer(aM);

        decomposer.decompose();

        final var vMatrix = decomposer.getV();

        // axis of rotation
        final var axis = vMatrix.getSubmatrixAsArray(0, 0, 2, 0);

        final var axisRotation = new AxisRotation3D(axis, theta);

        final var q1 = new Quaternion();
        // set values
        q1.setFromAxisAndRotation(axis[0], axis[1], axis[2], theta);

        // check correctness
        assertArrayEquals(axis, q1.getRotationAxis(), ABSOLUTE_ERROR);
        assertEquals(theta, q1.getRotationAngle(), ABSOLUTE_ERROR);

        // set values with jacobians
        final var jacobianOfTheta1 = new Matrix(Quaternion.N_PARAMS, 1);
        final var jacobianOfAxis1 = new Matrix(Quaternion.N_PARAMS, Quaternion.N_ANGLES);

        q1.setFromAxisAndRotation(axis[0], axis[1], axis[2], theta, jacobianOfTheta1, jacobianOfAxis1);

        // check correctness
        final var halfTheta = theta / 2.0;
        final var c = Math.cos(halfTheta);
        final var s = Math.sin(halfTheta);
        final var halfC = c / 2.0;
        final var halfS = s / 2.0;

        assertEquals(-halfS, jacobianOfTheta1.getElementAtIndex(0), ABSOLUTE_ERROR);
        assertEquals(axis[0] * halfC, jacobianOfTheta1.getElementAtIndex(1), ABSOLUTE_ERROR);
        assertEquals(axis[1] * halfC, jacobianOfTheta1.getElementAtIndex(2), ABSOLUTE_ERROR);
        assertEquals(axis[2] * halfC, jacobianOfTheta1.getElementAtIndex(3), ABSOLUTE_ERROR);

        assertEquals(0.0, jacobianOfAxis1.getElementAt(0, 0), 0.0);
        assertEquals(s, jacobianOfAxis1.getElementAt(1, 0), 0.0);
        assertEquals(0.0, jacobianOfAxis1.getElementAt(2, 0), 0.0);
        assertEquals(0.0, jacobianOfAxis1.getElementAt(3, 0), 0.0);

        assertEquals(0.0, jacobianOfAxis1.getElementAt(0, 1), 0.0);
        assertEquals(0.0, jacobianOfAxis1.getElementAt(1, 1), 0.0);
        assertEquals(s, jacobianOfAxis1.getElementAt(2, 1), 0.0);
        assertEquals(0.0, jacobianOfAxis1.getElementAt(3, 0), 0.0);

        assertEquals(0.0, jacobianOfAxis1.getElementAt(0, 2), 0.0);
        assertEquals(0.0, jacobianOfAxis1.getElementAt(1, 2), 0.0);
        assertEquals(0.0, jacobianOfAxis1.getElementAt(2, 2), 0.0);
        assertEquals(s, jacobianOfAxis1.getElementAt(3, 2), 0.0);

        // Force IllegalArgumentException
        final var invalid = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> q1.setFromAxisAndRotation(axis[0], axis[1], axis[2], theta, invalid, jacobianOfAxis1));
        assertThrows(IllegalArgumentException.class,
                () -> q1.setFromAxisAndRotation(axis[0], axis[1], axis[2], theta, jacobianOfTheta1, invalid));

        // set values
        final var q2 = new Quaternion();
        q2.setFromAxisAndRotation(axis, theta);

        // check correctness
        assertArrayEquals(axis, q2.getRotationAxis(), ABSOLUTE_ERROR);
        assertEquals(theta, q2.getRotationAngle(), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> q2.setFromAxisAndRotation(new double[1], theta));

        // set values with jacobians
        final var jacobianOfTheta2 = new Matrix(Quaternion.N_PARAMS, 1);
        final var jacobianOfAxis2 = new Matrix(Quaternion.N_PARAMS, Quaternion.N_ANGLES);

        q2.setFromAxisAndRotation(axis, theta, jacobianOfTheta2, jacobianOfAxis2);

        // check correctness
        assertEquals(-halfS, jacobianOfTheta2.getElementAtIndex(0), ABSOLUTE_ERROR);
        assertEquals(axis[0] * halfC, jacobianOfTheta2.getElementAtIndex(1), ABSOLUTE_ERROR);
        assertEquals(axis[1] * halfC, jacobianOfTheta2.getElementAtIndex(2), ABSOLUTE_ERROR);
        assertEquals(axis[2] * halfC, jacobianOfTheta2.getElementAtIndex(3), ABSOLUTE_ERROR);

        assertEquals(0.0, jacobianOfAxis2.getElementAt(0, 0), 0.0);
        assertEquals(s, jacobianOfAxis2.getElementAt(1, 0), 0.0);
        assertEquals(0.0, jacobianOfAxis2.getElementAt(2, 0), 0.0);
        assertEquals(0.0, jacobianOfAxis2.getElementAt(3, 0), 0.0);

        assertEquals(0.0, jacobianOfAxis2.getElementAt(0, 1), 0.0);
        assertEquals(0.0, jacobianOfAxis2.getElementAt(1, 1), 0.0);
        assertEquals(s, jacobianOfAxis2.getElementAt(2, 1), 0.0);
        assertEquals(0.0, jacobianOfAxis2.getElementAt(3, 0), 0.0);

        assertEquals(0.0, jacobianOfAxis2.getElementAt(0, 2), 0.0);
        assertEquals(0.0, jacobianOfAxis2.getElementAt(1, 2), 0.0);
        assertEquals(0.0, jacobianOfAxis2.getElementAt(2, 2), 0.0);
        assertEquals(s, jacobianOfAxis2.getElementAt(3, 2), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> q2.setFromAxisAndRotation(new double[1], theta, jacobianOfTheta2, jacobianOfAxis2));
        assertThrows(IllegalArgumentException.class,
                () -> q2.setFromAxisAndRotation(axis, theta, invalid, jacobianOfAxis2));
        assertThrows(IllegalArgumentException.class,
                () -> q2.setFromAxisAndRotation(axis, theta, jacobianOfTheta2, invalid));

        // set values
        final var q3 = new Quaternion();
        q3.setFromAxisAndRotation(axisRotation);

        // check correctness
        assertArrayEquals(axis, q3.getRotationAxis(), ABSOLUTE_ERROR);
        assertEquals(theta, q3.getRotationAngle(), ABSOLUTE_ERROR);

        // set values with jacobians
        final var jacobianOfTheta3 = new Matrix(Quaternion.N_PARAMS, 1);
        final var jacobianOfAxis3 = new Matrix(Quaternion.N_PARAMS, Quaternion.N_ANGLES);

        final var q4 = new Quaternion();
        q4.setFromAxisAndRotation(axisRotation, jacobianOfTheta3, jacobianOfAxis3);

        // check correctness
        assertArrayEquals(axis, q4.getRotationAxis(), ABSOLUTE_ERROR);
        assertEquals(theta, q4.getRotationAngle(), ABSOLUTE_ERROR);

        assertEquals(-halfS, jacobianOfTheta3.getElementAtIndex(0), ABSOLUTE_ERROR);
        assertEquals(axis[0] * halfC, jacobianOfTheta3.getElementAtIndex(1), ABSOLUTE_ERROR);
        assertEquals(axis[1] * halfC, jacobianOfTheta3.getElementAtIndex(2), ABSOLUTE_ERROR);
        assertEquals(axis[2] * halfC, jacobianOfTheta3.getElementAtIndex(3), ABSOLUTE_ERROR);

        assertEquals(0.0, jacobianOfAxis3.getElementAt(0, 0), 0.0);
        assertEquals(s, jacobianOfAxis3.getElementAt(1, 0), 0.0);
        assertEquals(0.0, jacobianOfAxis3.getElementAt(2, 0), 0.0);
        assertEquals(0.0, jacobianOfAxis3.getElementAt(3, 0), 0.0);

        assertEquals(0.0, jacobianOfAxis3.getElementAt(0, 1), 0.0);
        assertEquals(0.0, jacobianOfAxis3.getElementAt(1, 1), 0.0);
        assertEquals(s, jacobianOfAxis3.getElementAt(2, 1), 0.0);
        assertEquals(0.0, jacobianOfAxis3.getElementAt(3, 0), 0.0);

        assertEquals(0.0, jacobianOfAxis3.getElementAt(0, 2), 0.0);
        assertEquals(0.0, jacobianOfAxis3.getElementAt(1, 2), 0.0);
        assertEquals(0.0, jacobianOfAxis3.getElementAt(2, 2), 0.0);
        assertEquals(s, jacobianOfAxis3.getElementAt(3, 2), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> q4.setFromAxisAndRotation(axisRotation, invalid, jacobianOfAxis3));
        assertThrows(IllegalArgumentException.class,
                () -> q4.setFromAxisAndRotation(axisRotation, jacobianOfTheta3, invalid));
    }

    @Test
    void testMultiplyAndProduct() throws AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var a1 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var a2 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var b1 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var b2 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var c1 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var c2 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var d1 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var d2 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var q1 = new Quaternion(a1, b1, c1, d1);
        final var q2 = new Quaternion(a2, b2, c2, d2);

        final var a = a1 * a2 - b1 * b2 - c1 * c2 - d1 * d2;
        final var b = a1 * b2 + b1 * a2 + c1 * d2 - d1 * c2;
        final var c = a1 * c2 - b1 * d2 + c1 * a2 + d1 * b2;
        final var d = a1 * d2 + b1 * c2 - c1 * b2 + d1 * a2;

        final var q3 = new Quaternion(q1);
        q3.multiply(q2);

        // check correctness
        assertEquals(q3.getA(), a, ABSOLUTE_ERROR);
        assertEquals(q3.getB(), b, ABSOLUTE_ERROR);
        assertEquals(q3.getC(), c, ABSOLUTE_ERROR);
        assertEquals(q3.getD(), d, ABSOLUTE_ERROR);

        final var q4 = q1.multiplyAndReturnNew(q2);

        // check correctness
        assertEquals(q4.getA(), a, ABSOLUTE_ERROR);
        assertEquals(q4.getB(), b, ABSOLUTE_ERROR);
        assertEquals(q4.getC(), c, ABSOLUTE_ERROR);
        assertEquals(q4.getD(), d, ABSOLUTE_ERROR);

        final var q5 = new Quaternion();
        q1.multiply(q2, q5);

        // check correctness
        assertEquals(q5.getA(), a, ABSOLUTE_ERROR);
        assertEquals(q5.getB(), b, ABSOLUTE_ERROR);
        assertEquals(q5.getC(), c, ABSOLUTE_ERROR);
        assertEquals(q5.getD(), d, ABSOLUTE_ERROR);

        final var q6 = new Quaternion();
        Quaternion.product(q1, q2, q6);

        // check correctness
        assertEquals(q6.getA(), a, ABSOLUTE_ERROR);
        assertEquals(q6.getB(), b, ABSOLUTE_ERROR);
        assertEquals(q6.getC(), c, ABSOLUTE_ERROR);
        assertEquals(q6.getD(), d, ABSOLUTE_ERROR);

        final var q7 = new Quaternion();
        final var jacobianQ1 = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
        final var jacobianQ2 = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
        Quaternion.product(q1, q2, q7, jacobianQ1, jacobianQ2);

        // check correctness
        assertEquals(q7.getA(), a, ABSOLUTE_ERROR);
        assertEquals(q7.getB(), b, ABSOLUTE_ERROR);
        assertEquals(q7.getC(), c, ABSOLUTE_ERROR);
        assertEquals(q7.getD(), d, ABSOLUTE_ERROR);

        assertEquals(a2, jacobianQ1.getElementAt(0, 0), ABSOLUTE_ERROR);
        assertEquals(b2, jacobianQ1.getElementAt(1, 0), ABSOLUTE_ERROR);
        assertEquals(c2, jacobianQ1.getElementAt(2, 0), ABSOLUTE_ERROR);
        assertEquals(d2, jacobianQ1.getElementAt(3, 0), ABSOLUTE_ERROR);

        assertEquals(-b2, jacobianQ1.getElementAt(0, 1), ABSOLUTE_ERROR);
        assertEquals(a2, jacobianQ1.getElementAt(1, 1), ABSOLUTE_ERROR);
        assertEquals(-d2, jacobianQ1.getElementAt(2, 1), ABSOLUTE_ERROR);
        assertEquals(c2, jacobianQ1.getElementAt(3, 1), ABSOLUTE_ERROR);

        assertEquals(-c2, jacobianQ1.getElementAt(0, 2), ABSOLUTE_ERROR);
        assertEquals(d2, jacobianQ1.getElementAt(1, 2), ABSOLUTE_ERROR);
        assertEquals(a2, jacobianQ1.getElementAt(2, 2), ABSOLUTE_ERROR);
        assertEquals(-b2, jacobianQ1.getElementAt(3, 2), ABSOLUTE_ERROR);

        assertEquals(-d2, jacobianQ1.getElementAt(0, 3), ABSOLUTE_ERROR);
        assertEquals(-c2, jacobianQ1.getElementAt(1, 3), ABSOLUTE_ERROR);
        assertEquals(b2, jacobianQ1.getElementAt(2, 3), ABSOLUTE_ERROR);
        assertEquals(a2, jacobianQ1.getElementAt(3, 3), ABSOLUTE_ERROR);

        assertEquals(a1, jacobianQ2.getElementAt(0, 0), ABSOLUTE_ERROR);
        assertEquals(b1, jacobianQ2.getElementAt(1, 0), ABSOLUTE_ERROR);
        assertEquals(c1, jacobianQ2.getElementAt(2, 0), ABSOLUTE_ERROR);
        assertEquals(d1, jacobianQ2.getElementAt(3, 0), ABSOLUTE_ERROR);

        assertEquals(-b1, jacobianQ2.getElementAt(0, 1), ABSOLUTE_ERROR);
        assertEquals(a1, jacobianQ2.getElementAt(1, 1), ABSOLUTE_ERROR);
        assertEquals(d1, jacobianQ2.getElementAt(2, 1), ABSOLUTE_ERROR);
        assertEquals(-c1, jacobianQ2.getElementAt(3, 1), ABSOLUTE_ERROR);

        assertEquals(-c1, jacobianQ2.getElementAt(0, 2), ABSOLUTE_ERROR);
        assertEquals(-d1, jacobianQ2.getElementAt(1, 2), ABSOLUTE_ERROR);
        assertEquals(a1, jacobianQ2.getElementAt(2, 2), ABSOLUTE_ERROR);
        assertEquals(b1, jacobianQ2.getElementAt(3, 2), ABSOLUTE_ERROR);

        assertEquals(-d1, jacobianQ2.getElementAt(0, 3), ABSOLUTE_ERROR);
        assertEquals(c1, jacobianQ2.getElementAt(1, 3), ABSOLUTE_ERROR);
        assertEquals(-b1, jacobianQ2.getElementAt(2, 3), ABSOLUTE_ERROR);
        assertEquals(a1, jacobianQ2.getElementAt(3, 3), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var invalid = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> Quaternion.product(q1, q2, q7, invalid, jacobianQ2));
        assertThrows(IllegalArgumentException.class, () -> Quaternion.product(q1, q2, q7, jacobianQ1, invalid));

        // check correctness of jacobians
        final var q8 = new Quaternion();
        final var jacobianQ3 = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
        final var jacobianQ4 = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
        Quaternion.product(q1, q2, q8, jacobianQ3, jacobianQ4);

        // check q1 variation
        var diff = new double[4];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        final var q1b = new Quaternion(q1.getA() + diff[0], q1.getB() + diff[1], q1.getC() + diff[2],
                q1.getD() + diff[3]);
        var qb = new Quaternion();
        Quaternion.product(q1b, q2, qb);

        var diffResult = new double[]{
                qb.getA() - q8.getA(),
                qb.getB() - q8.getB(),
                qb.getC() - q8.getC(),
                qb.getD() - q8.getD()
        };
        var diffResult2 = jacobianQ1.multiplyAndReturnNew(Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);

        // check q2 variation
        diff = new double[4];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        final var q2b = new Quaternion(q2.getA() + diff[0], q2.getB() + diff[1], q2.getC() + diff[2],
                q2.getD() + diff[3]);
        qb = new Quaternion();
        Quaternion.product(q1, q2b, qb);

        diffResult = new double[]{
                qb.getA() - q8.getA(),
                qb.getB() - q8.getB(),
                qb.getC() - q8.getC(),
                qb.getD() - q8.getD()
        };
        diffResult2 = jacobianQ2.multiplyAndReturnNew(Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);
    }

    @Test
    void testSetFromEulerAngles() throws AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var roll = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));

        final var jacobian1 = new Matrix(Quaternion.N_PARAMS, Quaternion.N_ANGLES);

        final var q1 = new Quaternion();
        q1.setFromEulerAngles(roll, pitch, yaw, jacobian1);

        // check correctness
        var angles = q1.toEulerAngles();
        assertEquals(roll, angles[0], ABSOLUTE_ERROR);
        assertEquals(pitch, angles[1], ABSOLUTE_ERROR);
        assertEquals(yaw, angles[2], ABSOLUTE_ERROR);

        final var halfRoll = roll / 2.0;
        final var halfPitch = pitch / 2.0;
        final var halfYaw = yaw / 2.0;

        final var sr = Math.sin(halfRoll);
        final var sp = Math.sin(halfPitch);
        final var sy = Math.sin(halfYaw);

        final var cr = Math.cos(halfRoll);
        final var cp = Math.cos(halfPitch);
        final var cy = Math.cos(halfYaw);

        final var tmp1 = -cy * cp * sr + sy * sp * cr;
        final var tmp2 = 0.5 * (cy * cp * cr + sy * sp * sr);
        final var tmp3 = -cy * sp * sr + sy * cp * cr;
        final var tmp4 = -sy * cp * sr - cy * sp * cr;
        final var tmp5 = -cy * sp * cr + sy * cp * sr;
        final var tmp6 = -cy * sp * sr - sy * cp * cr;
        final var tmp7 = 0.5 * (cy * cp * cr - sy * sp * sr);
        final var tmp8 = -cy * cp * sr - sy * sp * cr;
        final var tmp9 = -sy * cp * cr + cy * sp * sr;
        final var tmp10 = -sy * sp * cr + cy * cp * sr;
        assertEquals(0.5 * tmp1, jacobian1.getElementAt(0, 0), ABSOLUTE_ERROR);
        assertEquals(tmp2, jacobian1.getElementAt(1, 0), ABSOLUTE_ERROR);
        assertEquals(0.5 * tmp3, jacobian1.getElementAt(2, 0), ABSOLUTE_ERROR);
        assertEquals(0.5 * tmp4, jacobian1.getElementAt(3, 0), ABSOLUTE_ERROR);
        assertEquals(0.5 * tmp5, jacobian1.getElementAt(0, 1), ABSOLUTE_ERROR);
        assertEquals(0.5 * tmp6, jacobian1.getElementAt(1, 1), ABSOLUTE_ERROR);
        assertEquals(tmp7, jacobian1.getElementAt(2, 1), ABSOLUTE_ERROR);
        assertEquals(0.5 * tmp8, jacobian1.getElementAt(3, 1), ABSOLUTE_ERROR);

        assertEquals(0.5 * tmp9, jacobian1.getElementAt(0, 2), ABSOLUTE_ERROR);
        assertEquals(0.5 * tmp4, jacobian1.getElementAt(1, 2), ABSOLUTE_ERROR);
        assertEquals(0.5 * tmp10, jacobian1.getElementAt(2, 2), ABSOLUTE_ERROR);
        assertEquals(tmp2, jacobian1.getElementAt(3, 2), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var invalid = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> q1.setFromEulerAngles(roll, pitch, yaw, invalid));

        // set values
        final var q2 = new Quaternion();
        q2.setFromEulerAngles(roll, pitch, yaw);

        // check correctness
        angles = q2.toEulerAngles();
        assertEquals(roll, angles[0], ABSOLUTE_ERROR);
        assertEquals(pitch, angles[1], ABSOLUTE_ERROR);
        assertEquals(yaw, angles[2], ABSOLUTE_ERROR);

        // set values and jacobian
        final var jacobian2 = new Matrix(Quaternion.N_PARAMS, Quaternion.N_ANGLES);
        final var q3 = new Quaternion();
        q3.setFromEulerAngles(new double[]{roll, pitch, yaw}, jacobian2);

        // check correctness
        angles = q3.toEulerAngles();
        assertEquals(roll, angles[0], ABSOLUTE_ERROR);
        assertEquals(pitch, angles[1], ABSOLUTE_ERROR);
        assertEquals(yaw, angles[2], ABSOLUTE_ERROR);

        assertEquals(0.5 * tmp1, jacobian2.getElementAt(0, 0), ABSOLUTE_ERROR);
        assertEquals(tmp2, jacobian2.getElementAt(1, 0), ABSOLUTE_ERROR);
        assertEquals(0.5 * tmp3, jacobian2.getElementAt(2, 0), ABSOLUTE_ERROR);
        assertEquals(0.5 * tmp4, jacobian2.getElementAt(3, 0), ABSOLUTE_ERROR);

        assertEquals(0.5 * tmp5, jacobian2.getElementAt(0, 1), ABSOLUTE_ERROR);
        assertEquals(0.5 * tmp6, jacobian2.getElementAt(1, 1), ABSOLUTE_ERROR);
        assertEquals(tmp7, jacobian2.getElementAt(2, 1), ABSOLUTE_ERROR);
        assertEquals(0.5 * tmp8, jacobian2.getElementAt(3, 1), ABSOLUTE_ERROR);

        assertEquals(0.5 * tmp9, jacobian2.getElementAt(0, 2), ABSOLUTE_ERROR);
        assertEquals(0.5 * tmp4, jacobian2.getElementAt(1, 2), ABSOLUTE_ERROR);
        assertEquals(0.5 * tmp10, jacobian2.getElementAt(2, 2), ABSOLUTE_ERROR);
        assertEquals(tmp2, jacobian2.getElementAt(3, 2), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> q3.setFromEulerAngles(new double[1], jacobian2));
        assertThrows(IllegalArgumentException.class,
                () -> q3.setFromEulerAngles(new double[]{roll, pitch, yaw}, invalid));

        // set values
        final var q4 = new Quaternion();
        q4.setFromEulerAngles(new double[]{roll, pitch, yaw});

        // check correctness
        angles = q4.toEulerAngles();
        assertEquals(roll, angles[0], ABSOLUTE_ERROR);
        assertEquals(pitch, angles[1], ABSOLUTE_ERROR);
        assertEquals(yaw, angles[2], ABSOLUTE_ERROR);
    }

    @Test
    void testEulerToMatrixRotation() throws AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var roll = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));

        final var rot = new MatrixRotation3D();
        rot.setRollPitchYaw(roll, pitch, yaw);

        final var rot2 = new MatrixRotation3D();
        final var jacobian1 = new Matrix(9, 3);
        Quaternion.eulerToMatrixRotation(roll, pitch, yaw, rot2, jacobian1);

        // check correctness
        assertEquals(rot, rot2);

        final var sr = Math.sin(roll);
        final var cr = Math.cos(roll);
        final var sp = Math.sin(pitch);
        final var cp = Math.cos(pitch);
        final var sy = Math.sin(yaw);
        final var cy = Math.cos(yaw);

        final var tmp1 = -cr * sy + sr * sp * cy;
        final var tmp2 = cr * cy + sr * sp * sy;
        final var tmp3 = sr * sy + cr * sp * cy;
        final var tmp4 = -sr * cy + cr * sp * sy;
        final var tmp5 = cr * sy - sr * sp * cy;
        final var tmp6 = -cr * cy - sr * sp * sy;
        final var tmp7 = sr * cy - cr * sp * sy;

        assertEquals(cp * cy, rot2.internalMatrix.getElementAt(0, 0), ABSOLUTE_ERROR);
        assertEquals(cp * sy, rot2.internalMatrix.getElementAt(1, 0), ABSOLUTE_ERROR);
        assertEquals(-sp, rot2.internalMatrix.getElementAt(2, 0), ABSOLUTE_ERROR);

        assertEquals(tmp1, rot2.internalMatrix.getElementAt(0, 1), ABSOLUTE_ERROR);
        assertEquals(tmp2, rot2.internalMatrix.getElementAt(1, 1), ABSOLUTE_ERROR);
        assertEquals(sr * cp, rot2.internalMatrix.getElementAt(2, 1), ABSOLUTE_ERROR);

        assertEquals(tmp3, rot2.internalMatrix.getElementAt(0, 2), ABSOLUTE_ERROR);
        assertEquals(tmp4, rot2.internalMatrix.getElementAt(1, 2), ABSOLUTE_ERROR);
        assertEquals(cr * cp, rot2.internalMatrix.getElementAt(2, 2), ABSOLUTE_ERROR);

        assertEquals(0.0, jacobian1.getElementAt(0, 0), 0.0);
        assertEquals(0.0, jacobian1.getElementAt(1, 0), 0.0);
        assertEquals(0.0, jacobian1.getElementAt(2, 0), 0.0);
        assertEquals(tmp3, jacobian1.getElementAt(3, 0), ABSOLUTE_ERROR);
        assertEquals(tmp4, jacobian1.getElementAt(4, 0), ABSOLUTE_ERROR);
        assertEquals(cr * cp, jacobian1.getElementAt(5, 0), ABSOLUTE_ERROR);
        assertEquals(tmp5, jacobian1.getElementAt(6, 0), ABSOLUTE_ERROR);
        assertEquals(tmp6, jacobian1.getElementAt(7, 0), ABSOLUTE_ERROR);
        assertEquals(-sr * cp, jacobian1.getElementAt(8, 0), ABSOLUTE_ERROR);

        assertEquals(-sp * cy, jacobian1.getElementAt(0, 1), ABSOLUTE_ERROR);
        assertEquals(-sp * sy, jacobian1.getElementAt(1, 1), ABSOLUTE_ERROR);
        assertEquals(-cp, jacobian1.getElementAt(2, 1), ABSOLUTE_ERROR);
        assertEquals(sr * cp * cy, jacobian1.getElementAt(3, 1), ABSOLUTE_ERROR);
        assertEquals(sr * cp * sy, jacobian1.getElementAt(4, 1), ABSOLUTE_ERROR);
        assertEquals(-sr * sp, jacobian1.getElementAt(5, 1), ABSOLUTE_ERROR);
        assertEquals(cr * cp * cy, jacobian1.getElementAt(6, 1), ABSOLUTE_ERROR);
        assertEquals(cr * cp * sy, jacobian1.getElementAt(7, 1), ABSOLUTE_ERROR);
        assertEquals(-cr * sp, jacobian1.getElementAt(8, 1), ABSOLUTE_ERROR);

        assertEquals(-cp * sy, jacobian1.getElementAt(0, 2), ABSOLUTE_ERROR);
        assertEquals(cp * cy, jacobian1.getElementAt(1, 2), ABSOLUTE_ERROR);
        assertEquals(0.0, jacobian1.getElementAt(2, 2), 0.0);
        assertEquals(tmp6, jacobian1.getElementAt(3, 2), ABSOLUTE_ERROR);
        assertEquals(tmp1, jacobian1.getElementAt(4, 2), ABSOLUTE_ERROR);
        assertEquals(0.0, jacobian1.getElementAt(5, 2), 0.0);
        assertEquals(tmp7, jacobian1.getElementAt(6, 2), ABSOLUTE_ERROR);
        assertEquals(tmp3, jacobian1.getElementAt(7, 2), ABSOLUTE_ERROR);
        assertEquals(0.0, jacobian1.getElementAt(8, 2), 0.0);

        // Force IllegalArgumentException
        final var invalid = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> Quaternion.eulerToMatrixRotation(roll, pitch, yaw, rot2, invalid));

        // set new values
        final var rot3 = new MatrixRotation3D();
        Quaternion.eulerToMatrixRotation(roll, pitch, yaw, rot3);

        // check correctness
        assertEquals(rot, rot3);

        // set new values
        final var rot4 = new MatrixRotation3D();
        final var jacobian2 = new Matrix(9, 3);
        Quaternion.eulerToMatrixRotation(new double[]{roll, pitch, yaw}, rot4, jacobian2);

        // check correctness
        assertEquals(rot, rot4);

        assertEquals(cp * cy, rot4.internalMatrix.getElementAt(0, 0), ABSOLUTE_ERROR);
        assertEquals(cp * sy, rot4.internalMatrix.getElementAt(1, 0), ABSOLUTE_ERROR);
        assertEquals(-sp, rot4.internalMatrix.getElementAt(2, 0), ABSOLUTE_ERROR);

        assertEquals(tmp1, rot4.internalMatrix.getElementAt(0, 1), ABSOLUTE_ERROR);
        assertEquals(tmp2, rot4.internalMatrix.getElementAt(1, 1), ABSOLUTE_ERROR);
        assertEquals(sr * cp, rot4.internalMatrix.getElementAt(2, 1), ABSOLUTE_ERROR);

        assertEquals(tmp3, rot4.internalMatrix.getElementAt(0, 2), ABSOLUTE_ERROR);
        assertEquals(tmp4, rot4.internalMatrix.getElementAt(1, 2), ABSOLUTE_ERROR);
        assertEquals(cr * cp, rot4.internalMatrix.getElementAt(2, 2), ABSOLUTE_ERROR);

        assertEquals(0.0, jacobian2.getElementAt(0, 0), 0.0);
        assertEquals(0.0, jacobian2.getElementAt(1, 0), 0.0);
        assertEquals(0.0, jacobian2.getElementAt(2, 0), 0.0);
        assertEquals(tmp3, jacobian2.getElementAt(3, 0), ABSOLUTE_ERROR);
        assertEquals(tmp4, jacobian2.getElementAt(4, 0), ABSOLUTE_ERROR);
        assertEquals(cr * cp, jacobian2.getElementAt(5, 0),  ABSOLUTE_ERROR);
        assertEquals(tmp5, jacobian2.getElementAt(6, 0), ABSOLUTE_ERROR);
        assertEquals(tmp6, jacobian2.getElementAt(7, 0), ABSOLUTE_ERROR);
        assertEquals(-sr * cp, jacobian2.getElementAt(8, 0), ABSOLUTE_ERROR);

        assertEquals(-sp * cy, jacobian2.getElementAt(0, 1), ABSOLUTE_ERROR);
        assertEquals(-sp * sy, jacobian2.getElementAt(1, 1), ABSOLUTE_ERROR);
        assertEquals(-cp, jacobian2.getElementAt(2, 1), ABSOLUTE_ERROR);
        assertEquals(sr * cp * cy, jacobian2.getElementAt(3, 1), ABSOLUTE_ERROR);
        assertEquals(sr * cp * sy, jacobian2.getElementAt(4, 1), ABSOLUTE_ERROR);
        assertEquals(-sr * sp, jacobian2.getElementAt(5, 1), ABSOLUTE_ERROR);
        assertEquals(cr * cp * cy, jacobian2.getElementAt(6, 1), ABSOLUTE_ERROR);
        assertEquals(cr * cp * sy, jacobian2.getElementAt(7, 1), ABSOLUTE_ERROR);
        assertEquals(-cr * sp, jacobian2.getElementAt(8, 1), ABSOLUTE_ERROR);

        assertEquals(-cp * sy, jacobian2.getElementAt(0, 2), ABSOLUTE_ERROR);
        assertEquals(cp * cy, jacobian2.getElementAt(1, 2), ABSOLUTE_ERROR);
        assertEquals(0.0, jacobian2.getElementAt(2, 2), 0.0);
        assertEquals(tmp6, jacobian2.getElementAt(3, 2), ABSOLUTE_ERROR);
        assertEquals(tmp1, jacobian2.getElementAt(4, 2), ABSOLUTE_ERROR);
        assertEquals(0.0, jacobian2.getElementAt(5, 2), 0.0);
        assertEquals(tmp7, jacobian2.getElementAt(6, 2), ABSOLUTE_ERROR);
        assertEquals(tmp3, jacobian2.getElementAt(7, 2), ABSOLUTE_ERROR);
        assertEquals(0.0, jacobian2.getElementAt(8, 2), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> Quaternion.eulerToMatrixRotation(new double[1], rot2, jacobian2));
        assertThrows(IllegalArgumentException.class,
                () -> Quaternion.eulerToMatrixRotation(new double[]{roll, pitch, yaw}, rot2, invalid));

        // set new values
        final var rot5 = new MatrixRotation3D();
        Quaternion.eulerToMatrixRotation(new double[]{roll, pitch, yaw}, rot5);

        // check correctness
        assertEquals(rot, rot5);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> Quaternion.eulerToMatrixRotation(new double[1], rot2));
    }

    @Test
    void testToAxisAndRotationAngle() throws AlgebraException, RotationException {
        final var randomizer = new UniformRandomizer();
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // Find any 3 orthogonal vectors, 1st will be axis of rotation, and
        // the remaining two will lie on the rotation plane and will be used
        // to test theta angle

        // To find 3 orthogonal vectors, we use V matrix of a singular
        // decomposition of any Nx3 matrix
        final var aM = Matrix.createWithUniformRandomValues(1, ROTATION_COLS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var decomposer = new SingularValueDecomposer(aM);

        decomposer.decompose();

        final var vMatrix = decomposer.getV();

        // axis of rotation
        final var axis = vMatrix.getSubmatrixAsArray(0, 0, 2, 0);

        final var q = new Quaternion(axis, theta);

        final var axis2 = new double[Quaternion.N_ANGLES];
        final var jacobianAngle = new Matrix(1, Quaternion.N_PARAMS);
        final var jacobianAxis = new Matrix(Quaternion.N_ANGLES, Quaternion.N_PARAMS);
        var theta2 = q.toAxisAndRotationAngle(axis2, jacobianAngle, jacobianAxis);

        // check correctness
        assertArrayEquals(axis, axis2, ABSOLUTE_ERROR);
        assertEquals(theta, theta2, ABSOLUTE_ERROR);

        final var n = Math.sqrt(q.getB() * q.getB() + q.getC() * q.getC() + q.getD() * q.getD());
        final var s = q.getA();
        final var denom = n * n + s * s;
        final var aN = 2.0 * s / denom;
        final var aS = -2.0 * n / denom;

        assertEquals(jacobianAngle.getElementAtIndex(0), aS, ABSOLUTE_ERROR);
        assertEquals(jacobianAngle.getElementAtIndex(1), axis[0] * aN, ABSOLUTE_ERROR);
        assertEquals(jacobianAngle.getElementAtIndex(2), axis[1] * aN, ABSOLUTE_ERROR);
        assertEquals(jacobianAngle.getElementAtIndex(3), axis[2] * aN, ABSOLUTE_ERROR);

        final var uV = Matrix.identity(3, 3);
        uV.multiplyByScalar(n);
        uV.subtract(Matrix.newFromArray(new double[]{q.getB(), q.getC(), q.getD()}, true)
                .multiplyAndReturnNew(Matrix.newFromArray(axis2, false)));
        uV.multiplyByScalar(1.0 / (n * n));

        assertEquals(jacobianAxis.getSubmatrix(0, 1, 2, 3), uV);
        assertArrayEquals(new double[]{0.0, 0.0, 0.0},
                jacobianAxis.getSubmatrix(0, 0, 2, 0).toArray(),
                0.0);

        theta2 = q.toAxisAndRotationAngle(axis2);

        // check correctness
        assertArrayEquals(axis, axis2, ABSOLUTE_ERROR);
        assertEquals(theta, theta2, ABSOLUTE_ERROR);

        var axisRotation = new AxisRotation3D();
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
    void testToRotationVector() throws AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // Find any 3 orthogonal vectors, 1st will be axis of rotation, and
        // the remaining two will lie on the rotation plane and will be used
        // to test theta angle

        // To find 3 orthogonal vectors, we use V matrix of a singular
        // decomposition of any Nx3 matrix
        final var aM = Matrix.createWithUniformRandomValues(1, ROTATION_COLS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var decomposer = new SingularValueDecomposer(aM);

        decomposer.decompose();

        final var vMatrix = decomposer.getV();

        // axis of rotation
        final var axis = vMatrix.getSubmatrixAsArray(0, 0, 2, 0);

        final var q = new Quaternion(axis, theta);

        final var rotationVector1 = new double[axis.length];
        final var jacobian = new Matrix(Quaternion.N_ANGLES, Quaternion.N_PARAMS);

        q.toRotationVector(rotationVector1, jacobian);

        // check correctness
        final var axis2 = new double[axis.length];
        final var jacobianAngle = new Matrix(1, Quaternion.N_PARAMS);
        final var jacobianAxis = new Matrix(Quaternion.N_ANGLES, Quaternion.N_PARAMS);
        final var theta2 = q.toAxisAndRotationAngle(axis2, jacobianAngle, jacobianAxis);
        final var vA = Matrix.newFromArray(axis2, true);
        final var vQ = vA.multiplyAndReturnNew(jacobianAngle).addAndReturnNew(
                jacobianAxis.multiplyByScalarAndReturnNew(theta2));

        // rotation vector is proportional to rotation axis but having a norm equal
        // to the rotation angle
        assertEquals(ArrayUtils.dotProduct(rotationVector1, axis), Math.abs(theta), ABSOLUTE_ERROR);

        assertTrue(jacobian.equals(vQ, ABSOLUTE_ERROR));

        // throw IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> q.toRotationVector(new double[1], jacobian));
        final var m = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> q.toRotationVector(rotationVector1, m));

        final var rotationVector2 = new double[axis.length];
        q.toRotationVector(rotationVector2);

        // rotation vector is proportional to rotation axis but having a norm equal
        // to the rotation angle
        assertEquals(ArrayUtils.dotProduct(rotationVector2, axis), Math.abs(theta), ABSOLUTE_ERROR);
    }

    @Test
    void testToEulerAngles() throws AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var roll = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));

        final var rot = new MatrixRotation3D();
        rot.setRollPitchYaw(roll, pitch, yaw);

        final var q = new Quaternion(roll, pitch, yaw);

        //noinspection AssertBetweenInconvertibleTypes
        assertEquals(rot, q);

        var angles = new double[Quaternion.N_ANGLES];
        final var jacobian = new Matrix(Quaternion.N_ANGLES, Quaternion.N_PARAMS);

        q.toEulerAngles(angles, jacobian);

        // check correctness
        assertEquals(roll, angles[0], ABSOLUTE_ERROR);
        assertEquals(pitch, angles[1], ABSOLUTE_ERROR);
        assertEquals(yaw, angles[2], ABSOLUTE_ERROR);

        final var a = q.getA();
        final var b = q.getB();
        final var c = q.getC();
        final var d = q.getD();

        final var y1 = 2.0 * c * d + 2.0 * a * b;
        final var x1 = a * a - b * b - c * c + d * d;
        final var z2 = -2.0 * b * d + 2.0 * a * c;
        final var y3 = 2.0 * b * c + 2.0 * a * d;
        final var x3 = a * a + b * b - c * c - d * d;

        final var dx1dq = new double[]{2 * a, -2 * b, -2 * c, 2 * d};
        final var dy1dq = new double[]{2 * b, 2 * a, 2 * d, 2 * c};
        final var dz2dq = new double[]{2 * c, -2 * d, 2 * a, -2 * b};
        final var dx3dq = new double[]{2 * a, 2 * b, -2 * c, -2 * d};
        final var dy3dq = new double[]{2 * d, 2 * c, 2 * b, 2 * a};

        final var de1dx1 = -y1 / (x1 * x1 + y1 * y1);
        final var de1dy1 = x1 / (x1 * x1 + y1 * y1);
        final var de2dz2 = 1 / Math.sqrt(1 - z2 * z2);
        final var de3dx3 = -y3 / (x3 * x3 + y3 * y3);
        final var de3dy3 = x3 / (x3 * x3 + y3 * y3);

        // de1dq = de1dx1 * dx1dq + de1dy * dy1dq
        ArrayUtils.multiplyByScalar(dx1dq, de1dx1, dx1dq);
        ArrayUtils.multiplyByScalar(dy1dq, de1dy1, dy1dq);
        final var de1dq = ArrayUtils.sumAndReturnNew(dx1dq, dy1dq);

        // de2dq = de2dz2 * dz2dq
        final var de2dq = ArrayUtils.multiplyByScalarAndReturnNew(dz2dq, de2dz2);

        // de3dq = de3dx3 * dx3dq + de3dy3 * dy3dq
        ArrayUtils.multiplyByScalar(dx3dq, de3dx3, dx3dq);
        ArrayUtils.multiplyByScalar(dy3dq, de3dy3, dy3dq);
        final var de3dq = ArrayUtils.sumAndReturnNew(dx3dq, dy3dq);

        assertEquals(de1dq[0], jacobian.getElementAt(0, 0), ABSOLUTE_ERROR);
        assertEquals(de1dq[1], jacobian.getElementAt(0, 1), ABSOLUTE_ERROR);
        assertEquals(de1dq[2], jacobian.getElementAt(0, 2), ABSOLUTE_ERROR);
        assertEquals(de1dq[3], jacobian.getElementAt(0, 3), ABSOLUTE_ERROR);

        assertEquals(de2dq[0], jacobian.getElementAt(1, 0), ABSOLUTE_ERROR);
        assertEquals(de2dq[1], jacobian.getElementAt(1, 1), ABSOLUTE_ERROR);
        assertEquals(de2dq[2], jacobian.getElementAt(1, 2), ABSOLUTE_ERROR);
        assertEquals(de2dq[3], jacobian.getElementAt(1, 3), ABSOLUTE_ERROR);

        assertEquals(de3dq[0], jacobian.getElementAt(2, 0), ABSOLUTE_ERROR);
        assertEquals(de3dq[1], jacobian.getElementAt(2, 1), ABSOLUTE_ERROR);
        assertEquals(de3dq[2], jacobian.getElementAt(2, 2), ABSOLUTE_ERROR);
        assertEquals(de3dq[3], jacobian.getElementAt(2, 3), ABSOLUTE_ERROR);

        angles = new double[Quaternion.N_ANGLES];
        q.toEulerAngles(angles);

        // check correctness
        assertEquals(roll, angles[0], ABSOLUTE_ERROR);
        assertEquals(pitch, angles[1], ABSOLUTE_ERROR);
        assertEquals(yaw, angles[2], ABSOLUTE_ERROR);
    }

    @Test
    void testQuaternionMatrix() throws AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var roll1 = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));
        final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw1 = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));

        final var roll2 = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));
        final var pitch2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw2 = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));

        final var q1 = new Quaternion(roll1, pitch1, yaw1);
        final var q2 = new Quaternion(roll2, pitch2, yaw2);

        final var q = q1.multiplyAndReturnNew(q2);

        // check that quaternion product can be expressed as the matrix
        // product m1 with quaternion q2 expressed as a column vector
        var m1 = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
        q1.quaternionMatrix(m1);

        final var m2 = Matrix.newFromArray(q2.getValues(), true);
        var m = m1.multiplyAndReturnNew(m2);

        assertArrayEquals(q.getValues(), m.getBuffer(), ABSOLUTE_ERROR);

        m1 = q1.toQuaternionMatrix();
        m = m1.multiplyAndReturnNew(m2);
        assertArrayEquals(q.getValues(), m.getBuffer(), ABSOLUTE_ERROR);
    }

    @Test
    void testConjugate() throws AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var q = new Quaternion(a, b, c, d);

        // check correctness
        assertEquals(a, q.getA(), 0.0);
        assertEquals(b, q.getB(), 0.0);
        assertEquals(c, q.getC(), 0.0);
        assertEquals(d, q.getD(), 0.0);

        // conjugate
        final var qc1 = new Quaternion();
        final var jacobian = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
        q.conjugate(qc1, jacobian);

        // check correctness
        assertEquals(qc1.getA(), q.getA(), 0.0);
        assertEquals(-qc1.getB(), q.getB(), 0.0);
        assertEquals(-qc1.getC(), q.getC(), 0.0);
        assertEquals(-qc1.getD(), q.getD(), 0.0);

        assertEquals(1.0, jacobian.getElementAt(0, 0), 0.0);
        assertEquals(0.0, jacobian.getElementAt(1, 0), 0.0);
        assertEquals(0.0, jacobian.getElementAt(2, 0), 0.0);
        assertEquals(0.0, jacobian.getElementAt(3, 0), 0.0);

        assertEquals(0.0, jacobian.getElementAt(0, 1), 0.0);
        assertEquals(-1.0, jacobian.getElementAt(1, 1), 0.0);
        assertEquals(0.0, jacobian.getElementAt(2, 1), 0.0);
        assertEquals(0.0, jacobian.getElementAt(3, 0), 0.0);

        assertEquals(0.0, jacobian.getElementAt(0, 2), 0.0);
        assertEquals(0.0, jacobian.getElementAt(1, 2), 0.0);
        assertEquals(-1.0, jacobian.getElementAt(2, 2), 0.0);
        assertEquals(0.0, jacobian.getElementAt(3, 2), 0.0);

        assertEquals(0.0, jacobian.getElementAt(0, 3), 0.0);
        assertEquals(0.0, jacobian.getElementAt(1, 3), 0.0);
        assertEquals(0.0, jacobian.getElementAt(2, 3), 0.0);
        assertEquals(-1.0, jacobian.getElementAt(3, 3), 0.0);

        final var qc2 = new Quaternion();
        q.conjugate(qc2);

        final var qcB = q.conjugateAndReturnNew();

        // check correctness
        assertEquals(q.getA(), qc2.getA(), 0.0);
        assertEquals(-q.getB(), qc2.getB(), 0.0);
        assertEquals(-q.getC(), qc2.getC(), 0.0);
        assertEquals(-q.getD(), qc2.getD(), 0.0);

        assertEquals(qc2, qcB);

        // conjugate of conjugate is the same as the original quaternion
        qc2.conjugate(qc2);
        assertEquals(qc2, q);
        final var m = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> q.conjugate(qc2, m));
    }

    @Test
    void testQuaternionMatrixN() throws AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var roll1 = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw1 = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));

        final var roll2 = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));
        final var pitch2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw2 = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));

        final var q1 = new Quaternion(roll1, pitch1, yaw1);
        final var q2 = new Quaternion(roll2, pitch2, yaw2);

        final var q = q1.multiplyAndReturnNew(q2);

        // check that quaternion product can be expressed as the matrix
        // product m1 with quaternion q2 expressed as a column vector
        var m2 = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
        q2.quaternionMatrixN(m2);

        final var m1 = Matrix.newFromArray(q1.getValues(), true);
        var m = m2.multiplyAndReturnNew(m1);

        assertArrayEquals(q.getValues(), m.getBuffer(), ABSOLUTE_ERROR);

        m2 = q2.toQuaternionMatrixN();
        m = m2.multiplyAndReturnNew(m1);
        assertArrayEquals(q.getValues(), m.getBuffer(), ABSOLUTE_ERROR);
    }

    @Test
    void testToMatrixRotation() throws AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var roll = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));

        final var matrixRot = new MatrixRotation3D();
        matrixRot.setRollPitchYaw(roll, pitch, yaw);

        final var q = new Quaternion(roll, pitch, yaw);

        final var m1 = new Matrix(MatrixRotation3D.ROTATION3D_INHOM_MATRIX_ROWS,
                MatrixRotation3D.ROTATION3D_INHOM_MATRIX_COLS);
        final var jacobian = new Matrix(9, 4);
        q.toMatrixRotation(m1, jacobian);

        assertTrue(m1.equals(matrixRot.internalMatrix, ABSOLUTE_ERROR));

        final var a = q.getA();
        final var b = q.getB();
        final var c = q.getC();
        final var d = q.getD();

        assertEquals(2 * a, jacobian.getElementAt(0, 0), ABSOLUTE_ERROR);
        assertEquals(2 * d, jacobian.getElementAt(1, 0), ABSOLUTE_ERROR);
        assertEquals(-2 * c, jacobian.getElementAt(2, 0), ABSOLUTE_ERROR);
        assertEquals(-2 * d, jacobian.getElementAt(3, 0), ABSOLUTE_ERROR);
        assertEquals(2 * a, jacobian.getElementAt(4, 0), ABSOLUTE_ERROR);
        assertEquals(2 * b, jacobian.getElementAt(5, 0), ABSOLUTE_ERROR);
        assertEquals(2 * c, jacobian.getElementAt(6, 0), ABSOLUTE_ERROR);
        assertEquals(-2 * b, jacobian.getElementAt(7, 0), ABSOLUTE_ERROR);
        assertEquals(2 * a, jacobian.getElementAt(8, 0), ABSOLUTE_ERROR);

        assertEquals(2 * b, jacobian.getElementAt(0, 1), ABSOLUTE_ERROR);
        assertEquals(2 * c, jacobian.getElementAt(1, 1), ABSOLUTE_ERROR);
        assertEquals(2 * d, jacobian.getElementAt(2, 1), ABSOLUTE_ERROR);
        assertEquals(2 * c, jacobian.getElementAt(3, 1), ABSOLUTE_ERROR);
        assertEquals(-2 * b, jacobian.getElementAt(4, 1), ABSOLUTE_ERROR);
        assertEquals(2 * a, jacobian.getElementAt(5, 1), ABSOLUTE_ERROR);
        assertEquals(2 * d, jacobian.getElementAt(6, 1), ABSOLUTE_ERROR);
        assertEquals(-2 * a, jacobian.getElementAt(7, 1), ABSOLUTE_ERROR);
        assertEquals(-2 * b, jacobian.getElementAt(8, 1), ABSOLUTE_ERROR);

        assertEquals(-2 * c, jacobian.getElementAt(0, 2), ABSOLUTE_ERROR);
        assertEquals(2 * b, jacobian.getElementAt(1, 2), ABSOLUTE_ERROR);
        assertEquals(-2 * a, jacobian.getElementAt(2, 2), ABSOLUTE_ERROR);
        assertEquals(2 * b, jacobian.getElementAt(3, 2), ABSOLUTE_ERROR);
        assertEquals(2 * c, jacobian.getElementAt(4, 2), ABSOLUTE_ERROR);
        assertEquals(2 * d, jacobian.getElementAt(5, 2), ABSOLUTE_ERROR);
        assertEquals(2 * a, jacobian.getElementAt(6, 2), ABSOLUTE_ERROR);
        assertEquals(2 * d, jacobian.getElementAt(7, 2), ABSOLUTE_ERROR);
        assertEquals(-2 * c, jacobian.getElementAt(8, 2), ABSOLUTE_ERROR);

        assertEquals(-2 * d, jacobian.getElementAt(0, 3), ABSOLUTE_ERROR);
        assertEquals(2 * a, jacobian.getElementAt(1, 3), ABSOLUTE_ERROR);
        assertEquals(2 * b, jacobian.getElementAt(2, 3), ABSOLUTE_ERROR);
        assertEquals(-2 * a, jacobian.getElementAt(3, 3), ABSOLUTE_ERROR);
        assertEquals(-2 * d, jacobian.getElementAt(4, 3), ABSOLUTE_ERROR);
        assertEquals(2 * c, jacobian.getElementAt(5, 3), ABSOLUTE_ERROR);
        assertEquals(2 * b, jacobian.getElementAt(6, 3), ABSOLUTE_ERROR);
        assertEquals(2 * c, jacobian.getElementAt(7, 3), ABSOLUTE_ERROR);
        assertEquals(2 * d, jacobian.getElementAt(8, 3), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var m = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> q.toMatrixRotation(m1, m));
        assertThrows(IllegalArgumentException.class, () -> q.toMatrixRotation(m, jacobian));

        final var m2 = new Matrix(MatrixRotation3D.ROTATION3D_INHOM_MATRIX_ROWS,
                MatrixRotation3D.ROTATION3D_INHOM_MATRIX_COLS);
        q.toMatrixRotation(m2);

        assertTrue(m2.equals(matrixRot.internalMatrix, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        final var m3 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> q.toMatrixRotation(m3));

        final var matrixRot2 = new MatrixRotation3D();
        q.toMatrixRotation(matrixRot2);

        assertEquals(matrixRot, matrixRot2);

        final var matrixRot3 = q.toMatrixRotation();

        assertEquals(matrixRot, matrixRot3);
    }

    @Test
    void testRotate() throws ColinearPointsException, AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var roll = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));

        final var matrixRot = new MatrixRotation3D();
        matrixRot.setRollPitchYaw(roll, pitch, yaw);

        final var q = new Quaternion(roll, pitch, yaw);

        // create 3 random points
        final var point1 = new HomogeneousPoint3D();
        point1.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final var point2 = new HomogeneousPoint3D();
        point2.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final var point3 = new HomogeneousPoint3D();
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
        final var plane = new Plane(point1, point2, point3);
        assertTrue(plane.isLocus(point1, ABSOLUTE_ERROR));
        assertTrue(plane.isLocus(point2, ABSOLUTE_ERROR));
        assertTrue(plane.isLocus(point3, ABSOLUTE_ERROR));

        // now rotate points
        final var jacobianPoint1A = new Matrix(3, 3);
        final var jacobianQuaternion1A = new Matrix(3, 4);
        final var rotPoint1A = Point3D.create();
        Quaternion.rotate(q, point1, rotPoint1A, jacobianPoint1A, jacobianQuaternion1A);

        final var rotPoint1B = Point3D.create();
        final var jacobianPoint1B = new Matrix(3, 3);
        final var jacobianQuaternion1B = new Matrix(3, 4);
        q.rotate(point1, rotPoint1B, jacobianPoint1B, jacobianQuaternion1B);

        final var rotPoint1C = Point3D.create();
        q.rotate(point1, rotPoint1C);

        final var rotPoint1D = q.rotate(point1);

        final var rotPoint1E = matrixRot.rotate(point1);

        final var jacobianPoint2A = new Matrix(3, 3);
        final var jacobianQuaternion2A = new Matrix(3, 4);
        final var rotPoint2A = Point3D.create();
        Quaternion.rotate(q, point2, rotPoint2A, jacobianPoint2A, jacobianQuaternion2A);

        final var rotPoint2B = Point3D.create();
        final var jacobianPoint2B = new Matrix(3, 3);
        final var jacobianQuaternion2B = new Matrix(3, 4);
        q.rotate(point2, rotPoint2B, jacobianPoint2B, jacobianQuaternion2B);

        final var rotPoint2C = Point3D.create();
        q.rotate(point2, rotPoint2C);

        final var rotPoint2D = q.rotate(point2);

        final var rotPoint2E = matrixRot.rotate(point2);

        final var jacobianPoint3A = new Matrix(3, 3);
        final var jacobianQuaternion3A = new Matrix(3, 4);
        final var rotPoint3A = Point3D.create();
        Quaternion.rotate(q, point3, rotPoint3A, jacobianPoint3A, jacobianQuaternion3A);

        final var rotPoint3B = Point3D.create();
        final var jacobianPoint3B = new Matrix(3, 3);
        final var jacobianQuaternion3B = new Matrix(3, 4);
        q.rotate(point3, rotPoint3B, jacobianPoint3B, jacobianQuaternion3B);

        final var rotPoint3C = Point3D.create();
        q.rotate(point3, rotPoint3C);

        final var rotPoint3D = q.rotate(point3);

        final var rotPoint3E = matrixRot.rotate(point3);

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
        final var point1Mat = Matrix.newFromArray(point1.asArray(), true);
        final var point2Mat = Matrix.newFromArray(point2.asArray(), true);
        final var point3Mat = Matrix.newFromArray(point3.asArray(), true);
        final var r = q.asHomogeneousMatrix();
        final var rotPoint1Mat = r.multiplyAndReturnNew(point1Mat);
        final var rotPoint2Mat = r.multiplyAndReturnNew(point2Mat);
        final var rotPoint3Mat = r.multiplyAndReturnNew(point3Mat);

        // check correctness
        var scaleX = rotPoint1A.getHomX() / rotPoint1Mat.getElementAtIndex(0);
        var scaleY = rotPoint1A.getHomY() / rotPoint1Mat.getElementAtIndex(1);
        var scaleZ = rotPoint1A.getHomZ() / rotPoint1Mat.getElementAtIndex(2);
        var scaleW = rotPoint1A.getHomW() / rotPoint1Mat.getElementAtIndex(3);
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
        final var inhomPoint = new Matrix(INHOM_COORDS, 1);
        inhomPoint.setElementAtIndex(0, point1.getInhomX());
        inhomPoint.setElementAtIndex(1, point1.getInhomY());
        inhomPoint.setElementAtIndex(2, point1.getInhomZ());
        final var inhomR = q.asInhomogeneousMatrix();
        final var inhomRotPoint = inhomR.multiplyAndReturnNew(inhomPoint);
        final var rotP = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, inhomRotPoint.toArray());
        assertTrue(rotP.equals(rotPoint1A, ABSOLUTE_ERROR));

        final var rotPlaneA = q.rotate(plane);
        final var rotPlaneB = new Plane();
        q.rotate(plane, rotPlaneB);

        // check both rotated lines are equal
        var scaleA = rotPlaneA.getA() / rotPlaneB.getA();
        var scaleB = rotPlaneA.getB() / rotPlaneB.getB();
        var scaleC = rotPlaneA.getC() / rotPlaneB.getC();
        var scaleD = rotPlaneA.getD() / rotPlaneB.getD();

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
        final var planeMat = Matrix.newFromArray(plane.asArray(), true);
        final var rotPlaneMat = r.multiplyAndReturnNew(planeMat);

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
    void testMatrixRotationToQuaternionAndSetFromMatrixRotation() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var roll = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));

        final var matrixRot = new MatrixRotation3D();
        matrixRot.setRollPitchYaw(roll, pitch, yaw);

        final var q1 = new Quaternion();

        // test
        Quaternion.matrixRotationToQuaternion(matrixRot.internalMatrix, q1);

        // check correctness
        //noinspection AssertBetweenInconvertibleTypes
        assertEquals(matrixRot, q1);

        // Force IllegalArgumentException
        final var m = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> Quaternion.matrixRotationToQuaternion(m, q1));

        // test
        final var q2 = new Quaternion();
        Quaternion.matrixRotationToQuaternion(matrixRot, q2);

        // check correctness
        //noinspection AssertBetweenInconvertibleTypes
        assertEquals(matrixRot, q2);

        // test
        final var q3 = new Quaternion();
        q3.setFromMatrixRotation(matrixRot.internalMatrix);

        // check correctness
        //noinspection AssertBetweenInconvertibleTypes
        assertEquals(matrixRot, q3);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> q3.setFromMatrixRotation(m));

        // test
        final var q4 = new Quaternion();
        q4.setFromMatrixRotation(matrixRot);

        // check correctness
        //noinspection AssertBetweenInconvertibleTypes
        assertEquals(matrixRot, q4);
    }

    @Test
    void testRotationVectorToRotationAxisAndAngle() throws AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // Find any 3 orthogonal vectors, 1st will be axis of rotation, and
        // the remaining two will lie on the rotation plane and will be used
        // to test theta angle

        // To find 3 orthogonal vectors, we use V matrix of a singular
        // decomposition of any Nx3 matrix
        final var aM = Matrix.createWithUniformRandomValues(1, ROTATION_COLS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var decomposer = new SingularValueDecomposer(aM);

        decomposer.decompose();

        final var vMatrix = decomposer.getV();

        // axis of rotation
        final var axis = vMatrix.getSubmatrixAsArray(0, 0, 2, 0);

        final var rotationVector = ArrayUtils.multiplyByScalarAndReturnNew(axis, theta);

        final var jacobianAlpha = new Matrix(1, 3);
        final var jacobianRotationVector = new Matrix(3, 3);

        final var axis2 = new double[axis.length];
        final var theta2 = Quaternion.rotationVectorToRotationAxisAndAngle(rotationVector, axis2, jacobianAlpha,
                jacobianRotationVector);

        assertArrayEquals(axis, axis2, ABSOLUTE_ERROR);
        assertEquals(theta, theta2, ABSOLUTE_ERROR);

        assertArrayEquals(jacobianAlpha.getBuffer(), axis, ABSOLUTE_ERROR);

        assertEquals(1 / theta - axis[0] * axis[0] / theta,
                jacobianRotationVector.getElementAt(0, 0), ABSOLUTE_ERROR);
        assertEquals(-axis[0] / theta * axis[1],
                jacobianRotationVector.getElementAt(1, 0), ABSOLUTE_ERROR);
        assertEquals(-axis[0] / theta * axis[2],
                jacobianRotationVector.getElementAt(2, 0), ABSOLUTE_ERROR);

        assertEquals(-axis[0] / theta * axis[1],
                jacobianRotationVector.getElementAt(0, 1), ABSOLUTE_ERROR);
        assertEquals(1 / theta - axis[1] * axis[1] / theta,
                jacobianRotationVector.getElementAt(1, 1), ABSOLUTE_ERROR);
        assertEquals(-axis[1] / theta * axis[2],
                jacobianRotationVector.getElementAt(2, 1), ABSOLUTE_ERROR);

        assertEquals(-axis[0] / theta * axis[2],
                jacobianRotationVector.getElementAt(0, 2), ABSOLUTE_ERROR);
        assertEquals(-axis[1] / theta * axis[2],
                jacobianRotationVector.getElementAt(1, 2), ABSOLUTE_ERROR);
        assertEquals(1 / theta - axis[2] * axis[2] / theta,
                jacobianRotationVector.getElementAt(2, 2), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> Quaternion.rotationVectorToRotationAxisAndAngle(new double[1], axis2, jacobianAlpha,
                        jacobianRotationVector));
        assertThrows(IllegalArgumentException.class,
                () -> Quaternion.rotationVectorToRotationAxisAndAngle(rotationVector, new double[1], jacobianAlpha,
                        jacobianRotationVector));
        final var m = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> Quaternion.rotationVectorToRotationAxisAndAngle(rotationVector, axis2, m,
                        jacobianRotationVector));
        assertThrows(IllegalArgumentException.class,
                () -> Quaternion.rotationVectorToRotationAxisAndAngle(rotationVector, axis2, jacobianAlpha, m));

        final var axis3 = new double[axis.length];
        final var theta3 = Quaternion.rotationVectorToRotationAxisAndAngle(rotationVector, axis3);

        assertArrayEquals(axis, axis3, ABSOLUTE_ERROR);
        assertEquals(theta, theta3, ABSOLUTE_ERROR);
    }

    @Test
    void testRotationVectorToQuaternion() throws AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // Find any 3 orthogonal vectors, 1st will be axis of rotation, and
        // the remaining two will lie on the rotation plane and will be used
        // to test theta angle

        // To find 3 orthogonal vectors, we use V matrix of a singular
        // decomposition of any Nx3 matrix
        final var aM = Matrix.createWithUniformRandomValues(1, ROTATION_COLS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var decomposer = new SingularValueDecomposer(aM);

        decomposer.decompose();

        final var vMatrix = decomposer.getV();

        // axis of rotation
        final var axis = vMatrix.getSubmatrixAsArray(0, 0, 2, 0);

        final var rotationVector = ArrayUtils.multiplyByScalarAndReturnNew(axis, theta);

        final var q1 = new Quaternion();
        final var jacobian1 = new Matrix(Quaternion.N_PARAMS, Quaternion.N_ANGLES);

        Quaternion.rotationVectorToQuaternion(rotationVector, q1, jacobian1);

        // check correctness
        var rotationVector2 = new double[Quaternion.N_ANGLES];
        q1.toRotationVector(rotationVector2);
        assertArrayEquals(rotationVector, rotationVector2, ABSOLUTE_ERROR);

        if (theta < Quaternion.LARGE_AXIS_NORM_THRESHOLD) {
            assertEquals(-0.25 * rotationVector[0], jacobian1.getElementAt(0, 0), ABSOLUTE_ERROR);
            assertEquals(-0.25 * rotationVector[1], jacobian1.getElementAt(1, 0), ABSOLUTE_ERROR);
            assertEquals(-0.25 * rotationVector[2], jacobian1.getElementAt(2, 0), ABSOLUTE_ERROR);

            assertEquals(0.5, jacobian1.getElementAt(0, 1), 0.0);
            assertEquals(0.0, jacobian1.getElementAt(1, 1), 0.0);
            assertEquals(0.0, jacobian1.getElementAt(2, 1), 0.0);

            assertEquals(0.0, jacobian1.getElementAt(0, 2), 0.0);
            assertEquals(0.5, jacobian1.getElementAt(1, 2), 0.0);
            assertEquals(0.0, jacobian1.getElementAt(2, 2), 0.0);

            assertEquals(0.0, jacobian1.getElementAt(0, 3), 0.0);
            assertEquals(0.0, jacobian1.getElementAt(1, 3), 0.0);
            assertEquals(0.5, jacobian1.getElementAt(2, 3), 0.0);
        } else {
            final var jacobianAlpha = new Matrix(1, Quaternion.N_ANGLES);
            final var jacobianRotationVector = new Matrix(Quaternion.N_ANGLES, Quaternion.N_ANGLES);
            Quaternion.rotationVectorToRotationAxisAndAngle(rotationVector, axis, jacobianAlpha,
                    jacobianRotationVector);

            final var jacobianOfTheta = new Matrix(Quaternion.N_PARAMS, 1);
            final var jacobianOfAxis = new Matrix(Quaternion.N_PARAMS, Quaternion.N_ANGLES);
            final var tmp = new Quaternion();
            tmp.setFromAxisAndRotation(axis, theta, jacobianOfTheta, jacobianOfAxis);

            final var jacobian2 = jacobianOfTheta.multiplyAndReturnNew(
                    jacobianAlpha).addAndReturnNew(jacobianOfAxis.multiplyAndReturnNew(jacobianRotationVector));

            assertTrue(jacobian1.equals(jacobian2, ABSOLUTE_ERROR));
        }

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> Quaternion.rotationVectorToQuaternion(new double[1], q1, jacobian1));
        final var m = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> Quaternion.rotationVectorToQuaternion(rotationVector, q1,
                m));

        final var q2 = new Quaternion();
        Quaternion.rotationVectorToQuaternion(rotationVector, q2);

        // check correctness
        rotationVector2 = new double[Quaternion.N_ANGLES];
        q2.toRotationVector(rotationVector2);
        assertArrayEquals(rotationVector, rotationVector2, ABSOLUTE_ERROR);

        // check correctness of jacobian
        final var q3 = new Quaternion();
        final var jacobian2 = new Matrix(Quaternion.N_PARAMS, Quaternion.N_ANGLES);

        Quaternion.rotationVectorToQuaternion(rotationVector, q3, jacobian2);

        // check rotation vector variation
        final var diff = new double[3];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        rotationVector2 = ArrayUtils.sumAndReturnNew(rotationVector, diff);

        final var q4 = new Quaternion();
        Quaternion.rotationVectorToQuaternion(rotationVector2, q4);

        final var diffResult = new double[]{
                q4.getA() - q3.getA(),
                q4.getB() - q3.getB(),
                q4.getC() - q3.getC(),
                q4.getD() - q3.getD()
        };
        final var diffResult2 = jacobian2.multiplyAndReturnNew(Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);
    }

    @Test
    void testSetFromRotationVector() throws AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // Find any 3 orthogonal vectors, 1st will be axis of rotation, and
        // the remaining two will lie on the rotation plane and will be used
        // to test theta angle

        // To find 3 orthogonal vectors, we use V matrix of a singular
        // decomposition of any Nx3 matrix
        final var aM = Matrix.createWithUniformRandomValues(1, ROTATION_COLS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var decomposer = new SingularValueDecomposer(aM);

        decomposer.decompose();

        final var vMatrix = decomposer.getV();

        // axis of rotation
        final var axis = vMatrix.getSubmatrixAsArray(0, 0, 2, 0);

        final var rotationVector = ArrayUtils.multiplyByScalarAndReturnNew(axis, theta);

        final var q = new Quaternion();

        // set value
        q.setFromRotationVector(rotationVector);

        // check correctness
        final var rotationVector2 = new double[Quaternion.N_ANGLES];
        q.toRotationVector(rotationVector2);
        assertArrayEquals(rotationVector, rotationVector2, ABSOLUTE_ERROR);
    }

    @Test
    void testRotationVectorToMatrixRotation() throws AlgebraException, RotationException {
        final var randomizer = new UniformRandomizer();
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // Find any 3 orthogonal vectors, 1st will be axis of rotation, and
        // the remaining two will lie on the rotation plane and will be used
        // to test theta angle

        // To find 3 orthogonal vectors, we use V matrix of a singular
        // decomposition of any Nx3 matrix
        final var aM = Matrix.createWithUniformRandomValues(1, ROTATION_COLS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var decomposer = new SingularValueDecomposer(aM);

        decomposer.decompose();

        final var vMatrix = decomposer.getV();

        // axis of rotation
        final var axis = vMatrix.getSubmatrixAsArray(0, 0, 2, 0);

        final var rotationVector = ArrayUtils.multiplyByScalarAndReturnNew(axis, theta);

        final var m = new Matrix(MatrixRotation3D.ROTATION3D_INHOM_MATRIX_ROWS,
                MatrixRotation3D.ROTATION3D_INHOM_MATRIX_COLS);

        Quaternion.rotationVectorToMatrixRotation(rotationVector, m);

        final var matrixRot = new MatrixRotation3D();
        Quaternion.rotationVectorToMatrixRotation(rotationVector, matrixRot);

        // check correctness
        assertEquals(m, matrixRot.internalMatrix);

        final var axis2 = matrixRot.getRotationAxis();
        final var theta2 = matrixRot.getRotationAngle();

        for (var i = 0; i < axis.length; i++) {
            assertEquals(Math.abs(axis[i]), Math.abs(axis2[i]), ABSOLUTE_ERROR);
        }
        assertEquals(Math.abs(theta), Math.abs(theta2), ABSOLUTE_ERROR);
    }

    @Test
    void testGetType() {
        final var q = new Quaternion();
        assertEquals(Rotation3DType.QUATERNION, q.getType());
    }

    @Test
    void testSetAxisAndRotationRotationAxisAndGetRotationAngle() throws AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // Find any 3 orthogonal vectors, 1st will be axis of rotation, and
        // the remaining two will lie on the rotation plane and will be used
        // to test theta angle

        // To find 3 orthogonal vectors, we use V matrix of a singular
        // decomposition of any Nx3 matrix
        final var aM = Matrix.createWithUniformRandomValues(1, ROTATION_COLS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var decomposer = new SingularValueDecomposer(aM);

        decomposer.decompose();

        final var vMatrix = decomposer.getV();

        // axis of rotation
        final var axis = vMatrix.getSubmatrixAsArray(0, 0, 2, 0);

        final var q = new Quaternion();
        q.setAxisAndRotation(axis[0], axis[1], axis[2], theta);

        // check correctness
        final var axis2 = new double[Quaternion.N_ANGLES];
        q.rotationAxis(axis2);

        assertArrayEquals(axis, axis2, ABSOLUTE_ERROR);
        assertEquals(theta, q.getRotationAngle(), ABSOLUTE_ERROR);
    }

    @Test
    void testAsInhomogeneousMatrix() throws AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // Find any 3 orthogonal vectors, 1st will be axis of rotation, and
        // the remaining two will lie on the rotation plane and will be used
        // to test theta angle

        // To find 3 orthogonal vectors, we use V matrix of a singular
        // decomposition of any Nx3 matrix
        final var aM = Matrix.createWithUniformRandomValues(1, ROTATION_COLS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var decomposer = new SingularValueDecomposer(aM);

        decomposer.decompose();

        final var vMatrix = decomposer.getV();

        // axis of rotation
        final var axis = vMatrix.getSubmatrixAsArray(0, 0, 2, 0);

        final var q = new Quaternion(axis, theta);
        final var rot = new MatrixRotation3D(axis, theta);

        var m1 = q.asInhomogeneousMatrix();
        var m2 = rot.asInhomogeneousMatrix();
        assertTrue(m1.equals(m2, ABSOLUTE_ERROR));

        m1 = new Matrix(MatrixRotation3D.ROTATION3D_INHOM_MATRIX_ROWS, MatrixRotation3D.ROTATION3D_INHOM_MATRIX_COLS);
        m2 = new Matrix(MatrixRotation3D.ROTATION3D_INHOM_MATRIX_ROWS, MatrixRotation3D.ROTATION3D_INHOM_MATRIX_COLS);

        q.asInhomogeneousMatrix(m1);
        rot.asInhomogeneousMatrix(m2);

        assertTrue(m1.equals(m2, ABSOLUTE_ERROR));
    }

    @Test
    void testAsHomogeneousMatrix() throws AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // Find any 3 orthogonal vectors, 1st will be axis of rotation, and
        // the remaining two will lie on the rotation plane and will be used
        // to test theta angle

        // To find 3 orthogonal vectors, we use V matrix of a singular
        // decomposition of any Nx3 matrix
        final var aM = Matrix.createWithUniformRandomValues(1, ROTATION_COLS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var decomposer = new SingularValueDecomposer(aM);

        decomposer.decompose();

        final var vMatrix = decomposer.getV();

        // axis of rotation
        final var axis = vMatrix.getSubmatrixAsArray(0, 0, 2, 0);

        final var q = new Quaternion(axis, theta);
        final var rot = new MatrixRotation3D(axis, theta);

        var m1 = q.asHomogeneousMatrix();
        var m2 = rot.asHomogeneousMatrix();
        assertTrue(m1.equals(m2, ABSOLUTE_ERROR));

        m1 = new Matrix(MatrixRotation3D.ROTATION3D_HOM_MATRIX_ROWS, MatrixRotation3D.ROTATION3D_HOM_MATRIX_COLS);
        m2 = new Matrix(MatrixRotation3D.ROTATION3D_HOM_MATRIX_ROWS, MatrixRotation3D.ROTATION3D_HOM_MATRIX_COLS);

        q.asHomogeneousMatrix(m1);
        rot.asHomogeneousMatrix(m2);

        assertTrue(m1.equals(m2, ABSOLUTE_ERROR));
    }

    @Test
    void testFromInhomogeneousMatrix() throws AlgebraException, InvalidRotationMatrixException {
        final var randomizer = new UniformRandomizer();
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // Find any 3 orthogonal vectors, 1st will be axis of rotation, and
        // the remaining two will lie on the rotation plane and will be used
        // to test theta angle

        // To find 3 orthogonal vectors, we use V matrix of a singular
        // decomposition of any Nx3 matrix
        final var aM = Matrix.createWithUniformRandomValues(1, ROTATION_COLS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var decomposer = new SingularValueDecomposer(aM);

        decomposer.decompose();

        final var vMatrix = decomposer.getV();

        // axis of rotation
        final var axis = vMatrix.getSubmatrixAsArray(0, 0,
                2, 0);

        final var q1 = new Quaternion(axis, theta);
        final var m1 = q1.asInhomogeneousMatrix();

        final var q2 = new Quaternion();
        q2.fromInhomogeneousMatrix(m1);

        // check correctness
        assertEquals(q1, q2);
        assertTrue(m1.equals(q2.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
    }

    @Test
    void testFromHomogeneousMatrix() throws AlgebraException, InvalidRotationMatrixException {
        final var randomizer = new UniformRandomizer();
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // Find any 3 orthogonal vectors, 1st will be axis of rotation, and
        // the remaining two will lie on the rotation plane and will be used
        // to test theta angle

        // To find 3 orthogonal vectors, we use V matrix of a singular
        // decomposition of any Nx3 matrix
        final var aM = Matrix.createWithUniformRandomValues(1, ROTATION_COLS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var decomposer = new SingularValueDecomposer(aM);

        decomposer.decompose();

        final var vMatrix = decomposer.getV();

        // axis of rotation
        final var axis = vMatrix.getSubmatrixAsArray(0, 0, 2, 0);

        final var q1 = new Quaternion(axis, theta);
        final var m1 = q1.asHomogeneousMatrix();

        final var q2 = new Quaternion();
        q2.fromHomogeneousMatrix(m1);

        // check correctness
        assertEquals(q1, q2);
        assertTrue(m1.equals(q2.asHomogeneousMatrix(), ABSOLUTE_ERROR));
    }

    @Test
    void testInverse() throws AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // Find any 3 orthogonal vectors, 1st will be axis of rotation, and
        // the remaining two will lie on the rotation plane and will be used
        // to test theta angle

        // To find 3 orthogonal vectors, we use V matrix of a singular
        // decomposition of any Nx3 matrix
        final var aM = Matrix.createWithUniformRandomValues(1, ROTATION_COLS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var decomposer = new SingularValueDecomposer(aM);

        decomposer.decompose();

        final var vMatrix = decomposer.getV();

        // axis of rotation
        final var axis = vMatrix.getSubmatrixAsArray(0, 0, 2, 0);

        final var q = new Quaternion(axis, theta);
        final var invQ1 = new Quaternion();
        Quaternion.inverse(q, invQ1);

        final var invQ2 = Quaternion.inverseAndReturnNew(q);

        final var invQ3 = new Quaternion();
        q.inverse(invQ3);

        final var invQ4 = q.inverseAndReturnNew();

        // check correctness
        assertEquals(invQ1, invQ2);
        assertEquals(invQ2, invQ3);
        assertEquals(invQ3, invQ4);
        assertEquals(invQ4, invQ1);

        final var prodQ = q.multiplyAndReturnNew(invQ1);

        assertEquals(1.0, prodQ.getA(), ABSOLUTE_ERROR);
        assertEquals(0.0, prodQ.getB(), ABSOLUTE_ERROR);
        assertEquals(0.0, prodQ.getC(), ABSOLUTE_ERROR);
        assertEquals(0.0, prodQ.getD(), ABSOLUTE_ERROR);

        final var point = new HomogeneousPoint3D();
        point.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        // rotate point
        final var rotPoint = q.rotate(point);

        // rotate with inverse quaternion
        final var point2 = invQ1.rotate(rotPoint);

        // check that both points are equal
        assertTrue(point.equals(point2, ABSOLUTE_ERROR));

        // inverse q
        q.inverse();

        // rotate again rotated point
        final var point3 = q.rotate(rotPoint);

        // check that both points are equal
        assertTrue(point.equals(point3, ABSOLUTE_ERROR));
    }

    @Test
    void testInverseRotation() throws AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // Find any 3 orthogonal vectors, 1st will be axis of rotation, and
        // the remaining two will lie on the rotation plane and will be used
        // to test theta angle

        // To find 3 orthogonal vectors, we use V matrix of a singular
        // decomposition of any Nx3 matrix
        final var aM = Matrix.createWithUniformRandomValues(1, ROTATION_COLS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var decomposer = new SingularValueDecomposer(aM);

        decomposer.decompose();

        final var vMatrix = decomposer.getV();

        // axis of rotation
        final var axis = vMatrix.getSubmatrixAsArray(0, 0, 2, 0);

        final var q = new Quaternion(axis, theta);

        final var invQ1 = q.inverseRotationAndReturnNew();
        final var invQ2 = new Quaternion();
        q.inverseRotation(invQ2);
        final var invRot = new MatrixRotation3D();
        q.inverseRotation(invRot);

        // check correctness
        assertEquals(invQ1, invQ2);
        //noinspection AssertBetweenInconvertibleTypes
        assertEquals(invQ2, invRot);
        assertEquals(invRot, invQ1);

        final var prodQ = q.multiplyAndReturnNew(invQ2);

        assertEquals(1.0, prodQ.getA(), ABSOLUTE_ERROR);
        assertEquals(0.0, prodQ.getB(), ABSOLUTE_ERROR);
        assertEquals(0.0, prodQ.getC(), ABSOLUTE_ERROR);
        assertEquals(0.0, prodQ.getD(), ABSOLUTE_ERROR);

        final var point = new HomogeneousPoint3D();
        point.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        // rotate point
        final var rotPoint = q.rotate(point);

        // rotate with inverse quaternion
        final var point2 = invQ2.rotate(rotPoint);

        // check that both points are equal
        assertTrue(point.equals(point2, ABSOLUTE_ERROR));

        // inverse q
        q.inverseRotation();

        // rotate again rotated point
        final var point3 = q.rotate(rotPoint);

        // check that both points are equal
        assertTrue(point.equals(point3, ABSOLUTE_ERROR));
    }

    @Test
    void testCombine() throws AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var roll1 = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));
        final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw1 = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));

        final var roll2 = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));
        final var pitch2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw2 = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));

        final var q1 = new Quaternion(roll1, pitch1, yaw1);
        final var q2 = new Quaternion(roll2, pitch2, yaw2);

        final var q = new Quaternion();
        Quaternion.combine(q1, q2, q);

        assertTrue(q.asInhomogeneousMatrix().equals(q1.asInhomogeneousMatrix()
                        .multiplyAndReturnNew(q2.asInhomogeneousMatrix()), ABSOLUTE_ERROR));

        final var q3 = q.combineAndReturnNew(q2);
        assertTrue(q3.asInhomogeneousMatrix().equals(q.asInhomogeneousMatrix()
                        .multiplyAndReturnNew(q2.asInhomogeneousMatrix()), ABSOLUTE_ERROR));

        final var rot2 = q2.toMatrixRotation();
        final var rot3 = q.combineAndReturnNew(rot2);

        assertEquals(q3, rot3);

        q1.combine(q2);
        assertEquals(q1, q);

        q.combine(rot2);
        assertEquals(q3, q);
    }

    @Test
    void testFromRotation() throws AlgebraException, RotationException {
        final var randomizer = new UniformRandomizer();
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // Find any 3 orthogonal vectors, 1st will be axis of rotation, and
        // the remaining two will lie on the rotation plane and will be used
        // to test theta angle

        // To find 3 orthogonal vectors, we use V matrix of a singular
        // decomposition of any Nx3 matrix
        final var aM = Matrix.createWithUniformRandomValues(1, ROTATION_COLS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var decomposer = new SingularValueDecomposer(aM);

        decomposer.decompose();

        final var vMatrix = decomposer.getV();

        // axis of rotation
        final var axis = vMatrix.getSubmatrixAsArray(0, 0, 2, 0);

        final var matrixRot = new MatrixRotation3D(axis, theta);
        final var axisRot = new AxisRotation3D(axis, theta);
        final var qRot = new Quaternion(axis, theta);

        // from matrix rotation
        var q = new Quaternion();
        q.fromRotation(matrixRot);

        // check correctness
        //noinspection AssertBetweenInconvertibleTypes
        assertEquals(q, matrixRot);
        assertArrayEquals(axis, q.getRotationAxis(), ABSOLUTE_ERROR);
        assertEquals(theta, q.getRotationAngle(), ABSOLUTE_ERROR);

        // from axis rotation
        q = new Quaternion();
        q.fromRotation(axisRot);

        // check correctness
        //noinspection AssertBetweenInconvertibleTypes
        assertEquals(q, axisRot);
        assertArrayEquals(axis, q.getRotationAxis(), ABSOLUTE_ERROR);
        assertEquals(theta, q.getRotationAngle(), ABSOLUTE_ERROR);

        // from quaternion
        q = new Quaternion();
        q.fromRotation(qRot);

        // check correctness
        assertEquals(q, qRot);
        assertArrayEquals(axis, q.getRotationAxis(), ABSOLUTE_ERROR);
        assertEquals(theta, q.getRotationAngle(), ABSOLUTE_ERROR);
    }

    @Test
    void testToQuaternion() throws AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // Find any 3 orthogonal vectors, 1st will be axis of rotation, and
        // the remaining two will lie on the rotation plane and will be used
        // to test theta angle

        // To find 3 orthogonal vectors, we use V matrix of a singular
        // decomposition of any Nx3 matrix
        final var aM = Matrix.createWithUniformRandomValues(1, ROTATION_COLS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var decomposer = new SingularValueDecomposer(aM);

        decomposer.decompose();

        final var vMatrix = decomposer.getV();

        // axis of rotation
        final var axis = vMatrix.getSubmatrixAsArray(0, 0, 2, 0);

        final var q1 = new Quaternion(axis, theta);
        final var q2 = new Quaternion();
        q1.toQuaternion(q2);

        assertEquals(q1, q2);
    }

    @Test
    void testNormalize() throws AlgebraException, RotationException {
        final var randomizer = new UniformRandomizer();
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // Find any 3 orthogonal vectors, 1st will be axis of rotation, and
        // the remaining two will lie on the rotation plane and will be used
        // to test theta angle

        // To find 3 orthogonal vectors, we use V matrix of a singular
        // decomposition of any Nx3 matrix
        final var aM = Matrix.createWithUniformRandomValues(1, ROTATION_COLS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var decomposer = new SingularValueDecomposer(aM);

        decomposer.decompose();

        final var vMatrix = decomposer.getV();

        // axis of rotation
        final var axis = vMatrix.getSubmatrixAsArray(0, 0, 2, 0);

        final var q1 = new Quaternion(axis, theta);
        final var q2 = new Quaternion(axis, theta);
        assertFalse(q1.isNormalized());
        assertFalse(q2.isNormalized());

        // normalize
        q1.normalize();
        q1.normalize();

        // after normalization both quaternions are equivalent
        assertTrue(q1.equals(q2, ABSOLUTE_ERROR));

        assertEquals(1.0,
                Math.sqrt(q1.getA() * q1.getA() + q1.getB() * q1.getB() + q1.getC() * q1.getC()
                        + q1.getD() * q1.getD()), ABSOLUTE_ERROR);

        assertTrue(q1.isNormalized());
        assertFalse(q2.isNormalized());


        // normalize with jacobian
        final var q3 = new Quaternion(axis, theta);
        assertFalse(q3.isNormalized());
        final var a = q3.getA();
        final var b = q3.getB();
        final var c = q3.getC();
        final var d = q3.getD();
        final var norm = Math.sqrt(a * a + b * b + c * c + d * d);

        final var jacobian = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
        q3.normalize(jacobian);

        // check correctness
        assertTrue(q3.equals(q2, ABSOLUTE_ERROR));

        assertEquals(1.0,
                Math.sqrt(q3.getA() * q3.getA() + q3.getB() * q3.getB() + q3.getC() * q3.getC()
                        + q3.getD() * q3.getD()), ABSOLUTE_ERROR);

        assertTrue(q3.isNormalized());
        assertFalse(q2.isNormalized());

        // check jacobian
        final var norm3 = norm * norm * norm;

        final var jacobian2 = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);

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
        final var m = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> q3.normalize(m));
    }

    @Test
    void testSlerp() throws RotationException {
        final var randomizer = new UniformRandomizer();
        final var roll1 = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));
        final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));
        final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));

        final var roll2 = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));
        final var pitch2 = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));
        final var yaw2 = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));

        final var q1 = new Quaternion(roll1, pitch1, yaw1);
        final var q2 = new Quaternion(roll2, pitch2, yaw2);

        final var q3a = q1.slerpAndReturnNew(q2, 0.0);
        final var q3b = new Quaternion();
        q1.slerp(q2, 0.0, q3b);
        final var q3c = Quaternion.slerpAndReturnNew(q1, q2, 0.0);
        final var q3d = new Quaternion();
        Quaternion.slerp(q1, q2, 0.0, q3d);

        assertTrue(q3a.equals(q1, ABSOLUTE_ERROR));
        assertTrue(q3b.equals(q1, ABSOLUTE_ERROR));
        assertTrue(q3c.equals(q1, ABSOLUTE_ERROR));
        assertTrue(q3d.equals(q1, ABSOLUTE_ERROR));

        final var q4a = q1.slerpAndReturnNew(q2, 1.0);
        final var q4b = new Quaternion();
        q1.slerp(q2, 1.0, q4b);
        final var q4c = Quaternion.slerpAndReturnNew(q1, q2, 1.0);
        final var q4d = new Quaternion();
        Quaternion.slerp(q1, q2, 1.0, q4d);

        assertTrue(q4a.equals(q2, ABSOLUTE_ERROR));
        assertTrue(q4b.equals(q2, ABSOLUTE_ERROR));
        assertTrue(q4c.equals(q2, ABSOLUTE_ERROR));
        assertTrue(q4d.equals(q2, ABSOLUTE_ERROR));

        final var q5a = q2.slerpAndReturnNew(q1, 0.0);
        final var q5b = new Quaternion();
        q2.slerp(q1, 0.0, q5b);
        final var q5c = Quaternion.slerpAndReturnNew(q2, q1, 0.0);
        final var q5d = new Quaternion();
        Quaternion.slerp(q2, q1, 0.0, q5d);

        assertTrue(q5a.equals(q2, ABSOLUTE_ERROR));
        assertTrue(q5b.equals(q2, ABSOLUTE_ERROR));
        assertTrue(q5c.equals(q2, ABSOLUTE_ERROR));
        assertTrue(q5d.equals(q2, ABSOLUTE_ERROR));

        final var q6a = q2.slerpAndReturnNew(q1, 1.0);
        final var q6b = new Quaternion();
        q2.slerp(q1, 1.0, q6b);
        final var q6c = Quaternion.slerpAndReturnNew(q2, q1, 1.0);
        final var q6d = new Quaternion();
        Quaternion.slerp(q2, q1, 1.0, q6d);

        assertTrue(q6a.equals(q1, ABSOLUTE_ERROR));
        assertTrue(q6b.equals(q1, ABSOLUTE_ERROR));
        assertTrue(q6c.equals(q1, ABSOLUTE_ERROR));
        assertTrue(q6d.equals(q1, ABSOLUTE_ERROR));

        final var result1 = new Quaternion();

        // use equal quaternions
        Quaternion.slerp(q1, q1, 0.0, result1);
        assertTrue(result1.equals(q1));

        Quaternion.slerp(q1, q1, 1.0, result1);
        assertTrue(result1.equals(q1));

        // force IllegalArgumentException
        final var result2 = new Quaternion();
        assertThrows(IllegalArgumentException.class, () -> Quaternion.slerp(q1, q2, -1.0, result2));
        assertThrows(IllegalArgumentException.class, () -> Quaternion.slerp(q1, q2, 2.0, result2));
        assertTrue(result2.equals(new Quaternion()));
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var randomizer = new UniformRandomizer();
        final var a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var q1 = new Quaternion(a, b, c, d);

        // check
        assertEquals(q1.getA(), a, 0.0);
        assertEquals(q1.getB(), b, 0.0);
        assertEquals(q1.getC(), c, 0.0);
        assertEquals(q1.getD(), d, 0.0);
        assertFalse(q1.isNormalized());

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(q1);
        final var q2 = SerializationHelper.<Quaternion>deserialize(bytes);

        // check
        assertEquals(q1, q2);
        assertNotSame(q1, q2);
    }
}
