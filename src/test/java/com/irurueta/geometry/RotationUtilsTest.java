/*
 * Copyright (C) 2016 Alberto Irurueta Carro (alberto@irurueta.com)
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
import com.irurueta.algebra.Matrix;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class RotationUtilsTest {

    private static final double MIN_ANGLE_DEGREES = 0.0;
    private static final double MAX_ANGLE_DEGREES = 90.0;

    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    private static final double ABSOLUTE_ERROR = 1e-6;

    @Test
    void testConstants() {
        assertEquals(3, RotationUtils.N_ANGULAR_RATES);
        assertEquals(4, RotationUtils.SKEW_MATRIX_SIZE);
    }

    @Test
    void testW2omega() throws AlgebraException {
        final var randomizer = new UniformRandomizer();

        final var w1 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var w2 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var w3 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var result1 = new Matrix(4, 4);
        RotationUtils.w2omega(w1, w2, w3, result1);

        // check correctness
        final var resultB = new Matrix(4, 4);
        resultB.setSubmatrix(0, 0, 3, 3,
                new double[]{
                        0.0, w1, w2, w3,
                        -w1, 0.0, -w3, w2,
                        -w2, w3, 0.0, -w1,
                        -w3, -w2, w1, 0.0
                }, true);

        assertTrue(result1.equals(resultB, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        final var m = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> RotationUtils.w2omega(w1, w2, w3, m));

        // test with array
        final var w = new double[]{w1, w2, w3};
        RotationUtils.w2omega(w, result1);

        // check correctness
        assertTrue(result1.equals(resultB, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> RotationUtils.w2omega(new double[1], result1));
        assertThrows(IllegalArgumentException.class, () -> RotationUtils.w2omega(w, m1));

        // test with new instance
        final var result2 = RotationUtils.w2omega(w1, w2, w3);

        // check correctness
        assertTrue(result2.equals(resultB, ABSOLUTE_ERROR));

        // test with array
        final var result3 = RotationUtils.w2omega(w);

        // check correctness
        assertTrue(result3.equals(resultB, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> RotationUtils.w2omega(new double[1]));
    }

    @Test
    void testQuaternionToPiMatrix() throws AlgebraException {
        final var randomizer = new UniformRandomizer();

        final var roll = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));

        final var q = new Quaternion(roll, pitch, yaw);

        final var a = q.getA();
        final var b = q.getB();
        final var c = q.getC();
        final var d = q.getD();

        final var pi1 = new Matrix(Quaternion.N_PARAMS, Quaternion.N_ANGLES);
        RotationUtils.quaternionToPiMatrix(q, pi1);
        final var pi2 = RotationUtils.quaternionToPiMatrix(q);

        // check correctness
        assertEquals(pi1, pi2);

        assertEquals(-b, pi1.getElementAt(0, 0), 0.0);
        assertEquals(a, pi1.getElementAt(1, 0), 0.0);
        assertEquals(d, pi1.getElementAt(2, 0), 0.0);
        assertEquals(-c, pi1.getElementAt(3, 0), 0.0);

        assertEquals(-c, pi1.getElementAt(0, 1), 0.0);
        assertEquals(-d, pi1.getElementAt(1, 1), 0.0);
        assertEquals(a, pi1.getElementAt(2, 1), 0.0);
        assertEquals(b, pi1.getElementAt(3, 1), 0.0);

        assertEquals(-d, pi1.getElementAt(0, 2), 0.0);
        assertEquals(c, pi1.getElementAt(1, 2), 0.0);
        assertEquals(-b, pi1.getElementAt(2, 2), 0.0);
        assertEquals(a, pi1.getElementAt(3, 2), 0.0);
    }

    @Test
    void testQuaternionToConjugatedPiMatrix() throws AlgebraException {
        final var randomizer = new UniformRandomizer();

        final var roll = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));

        final var q = new Quaternion(roll, pitch, yaw);

        final var a = q.getA();
        final var b = q.getB();
        final var c = q.getC();
        final var d = q.getD();

        final var cpi1 = new Matrix(Quaternion.N_PARAMS, Quaternion.N_ANGLES);
        RotationUtils.quaternionToConjugatedPiMatrix(q, cpi1);
        final var cpi2 = RotationUtils.quaternionToConjugatedPiMatrix(q);

        // check correctness
        assertEquals(cpi1, cpi2);

        assertEquals(b, cpi1.getElementAt(0, 0), 0.0);
        assertEquals(a, cpi1.getElementAt(1, 0), 0.0);
        assertEquals(-d, cpi1.getElementAt(2, 0), 0.0);
        assertEquals(c, cpi1.getElementAt(3, 0), 0.0);

        assertEquals(c, cpi1.getElementAt(0, 1), 0.0);
        assertEquals(d, cpi1.getElementAt(1, 1), 0.0);
        assertEquals(a, cpi1.getElementAt(2, 1), 0.0);
        assertEquals(-b, cpi1.getElementAt(3, 1), 0.0);

        assertEquals(d, cpi1.getElementAt(0, 2), 0.0);
        assertEquals(-c, cpi1.getElementAt(1, 2), 0.0);
        assertEquals(b, cpi1.getElementAt(2, 2), 0.0);
        assertEquals(a, cpi1.getElementAt(3, 2), 0.0);
    }

    @Test
    void testPiMatrixToConjugatedPiMatrix() throws AlgebraException {
        final var randomizer = new UniformRandomizer();

        final var roll = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));

        final var q = new Quaternion(roll, pitch, yaw);

        final var pi = RotationUtils.quaternionToPiMatrix(q);
        final var cpi = RotationUtils.quaternionToConjugatedPiMatrix(q);

        final var cpi2 = new Matrix(Quaternion.N_PARAMS, Quaternion.N_ANGLES);
        RotationUtils.piMatrixToConjugatedPiMatrix(pi, cpi2);
        final var cpi3 = RotationUtils.piMatrixToConjugatedPiMatrix(pi);

        assertEquals(cpi, cpi2);
        assertEquals(cpi, cpi3);
        assertEquals(cpi2, cpi3);
    }

    @Test
    void testRotationMatrixTimesVector() throws AlgebraException {
        final var randomizer = new UniformRandomizer();

        final var roll = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));

        final var q = new Quaternion(roll, pitch, yaw);

        final var point = new double[Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
        final var result1 = new double[point.length];
        final var result2 = new double[point.length];
        final var jacobianQ = new Matrix(Quaternion.N_ANGLES, Quaternion.N_PARAMS);
        final var jacobianP = new Matrix(Quaternion.N_ANGLES, Quaternion.N_ANGLES);
        RotationUtils.rotationMatrixTimesVector(q, point, result1, jacobianQ, jacobianP);

        // check correctness
        var rotated = new InhomogeneousPoint3D(q.rotate(Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES,
                point)));
        rotated.asArray(result2);

        assertArrayEquals(result2, result1, ABSOLUTE_ERROR);

        assertTrue(jacobianP.equals(q.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

        // check jacobian
        var a = q.getA();
        var b = q.getB();
        var c = q.getC();
        var d = q.getD();

        var x = point[0];
        var y = point[1];
        var z = point[2];

        var axdycz = 2.0 * (a * x - d * y + c * z);
        var bxcydz = 2.0 * (b * x + c * y + d * z);
        var cxbyaz = 2.0 * (c * x - b * y - a * z);
        var dxaybz = 2.0 * (d * x + a * y - b * z);

        assertEquals(axdycz, jacobianQ.getElementAt(0, 0), ABSOLUTE_ERROR);
        assertEquals(dxaybz, jacobianQ.getElementAt(1, 0), ABSOLUTE_ERROR);
        assertEquals(-cxbyaz, jacobianQ.getElementAt(2, 0), ABSOLUTE_ERROR);

        assertEquals(bxcydz, jacobianQ.getElementAt(0, 1), ABSOLUTE_ERROR);
        assertEquals(cxbyaz, jacobianQ.getElementAt(1, 1), ABSOLUTE_ERROR);
        assertEquals(dxaybz, jacobianQ.getElementAt(2, 1), ABSOLUTE_ERROR);

        assertEquals(-cxbyaz, jacobianQ.getElementAt(0, 2), ABSOLUTE_ERROR);
        assertEquals(bxcydz, jacobianQ.getElementAt(1, 2), ABSOLUTE_ERROR);
        assertEquals(-axdycz, jacobianQ.getElementAt(2, 2), ABSOLUTE_ERROR);

        assertEquals(-dxaybz, jacobianQ.getElementAt(0, 3), ABSOLUTE_ERROR);
        assertEquals(axdycz, jacobianQ.getElementAt(1, 3), ABSOLUTE_ERROR);
        assertEquals(bxcydz, jacobianQ.getElementAt(2, 3), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var m = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> RotationUtils.rotationMatrixTimesVector(q, point, result1,
                m, jacobianP));
        assertThrows(IllegalArgumentException.class, () -> RotationUtils.rotationMatrixTimesVector(q, point, result1,
                jacobianQ, m));

        // test without result
        final var result3 = RotationUtils.rotationMatrixTimesVector(q, point, jacobianQ, jacobianP);

        // check correctness
        rotated = new InhomogeneousPoint3D(q.rotate(Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, point)));
        rotated.asArray(result2);

        assertArrayEquals(result2, result3, ABSOLUTE_ERROR);

        assertTrue(jacobianP.equals(q.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

        // check jacobian
        a = q.getA();
        b = q.getB();
        c = q.getC();
        d = q.getD();

        x = point[0];
        y = point[1];
        z = point[2];

        axdycz = 2.0 * (a * x - d * y + c * z);
        bxcydz = 2.0 * (b * x + c * y + d * z);
        cxbyaz = 2.0 * (c * x - b * y - a * z);
        dxaybz = 2.0 * (d * x + a * y - b * z);

        assertEquals(axdycz, jacobianQ.getElementAt(0, 0), ABSOLUTE_ERROR);
        assertEquals(dxaybz,  jacobianQ.getElementAt(1, 0), ABSOLUTE_ERROR);
        assertEquals(-cxbyaz, jacobianQ.getElementAt(2, 0), ABSOLUTE_ERROR);

        assertEquals(bxcydz, jacobianQ.getElementAt(0, 1), ABSOLUTE_ERROR);
        assertEquals(cxbyaz, jacobianQ.getElementAt(1, 1), ABSOLUTE_ERROR);
        assertEquals(dxaybz, jacobianQ.getElementAt(2, 1), ABSOLUTE_ERROR);

        assertEquals(-cxbyaz, jacobianQ.getElementAt(0, 2), ABSOLUTE_ERROR);
        assertEquals(bxcydz, jacobianQ.getElementAt(1, 2), ABSOLUTE_ERROR);
        assertEquals(-axdycz, jacobianQ.getElementAt(2, 2), ABSOLUTE_ERROR);

        assertEquals(-dxaybz, jacobianQ.getElementAt(0, 3), ABSOLUTE_ERROR);
        assertEquals(axdycz, jacobianQ.getElementAt(1, 3), ABSOLUTE_ERROR);
        assertEquals(bxcydz, jacobianQ.getElementAt(2, 3), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> RotationUtils.rotationMatrixTimesVector(q, point, m1,
                jacobianP));
        assertThrows(IllegalArgumentException.class, () -> RotationUtils.rotationMatrixTimesVector(q, point, jacobianQ,
                m1));

        // test without jacobian
        RotationUtils.rotationMatrixTimesVector(q, point, result3);

        // check correctness
        rotated = new InhomogeneousPoint3D(q.rotate(Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, point)));
        rotated.asArray(result2);

        assertArrayEquals(result2, result3, ABSOLUTE_ERROR);

        assertTrue(jacobianP.equals(q.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

        // test without jacobian and result
        final var result4 = RotationUtils.rotationMatrixTimesVector(q, point);

        // check correctness
        rotated = new InhomogeneousPoint3D(q.rotate(Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, point)));
        rotated.asArray(result2);

        assertArrayEquals(result2, result4, ABSOLUTE_ERROR);

        assertTrue(jacobianP.equals(q.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

        final var p = new InhomogeneousPoint3D(point);
        Point3D resultPoint = new InhomogeneousPoint3D();
        RotationUtils.rotationMatrixTimesVector(q, p, resultPoint, null, null);
        final var result5 = resultPoint.asArray();
        assertArrayEquals(result2, result5, ABSOLUTE_ERROR);

        resultPoint = RotationUtils.rotationMatrixTimesVector(q, p, null, null);
        new InhomogeneousPoint3D(resultPoint).asArray(result2);
        assertArrayEquals(result2, result5, ABSOLUTE_ERROR);

        resultPoint = new InhomogeneousPoint3D();
        RotationUtils.rotationMatrixTimesVector(q, p, resultPoint);
        final var result6 = resultPoint.asArray();
        assertArrayEquals(result2, result6, ABSOLUTE_ERROR);

        resultPoint = RotationUtils.rotationMatrixTimesVector(q, p);
        new InhomogeneousPoint3D(resultPoint).asArray(result2);
        assertArrayEquals(result2, result6, ABSOLUTE_ERROR);
    }

    @Test
    void testTransposedRotationMatrixTimesVector() throws AlgebraException {
        final var randomizer = new UniformRandomizer();

        final var roll = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));

        final var q = new Quaternion(roll, pitch, yaw);

        final var point = new double[Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
        final var result1 = new double[point.length];
        final var result2 = new double[point.length];
        final var jacobianQ = new Matrix(Quaternion.N_ANGLES, Quaternion.N_PARAMS);
        final var jacobianP = new Matrix(Quaternion.N_ANGLES, Quaternion.N_ANGLES);
        RotationUtils.transposedRotationMatrixTimesVector(q, point, result1, jacobianQ, jacobianP);

        // check correctness
        var rotated = new InhomogeneousPoint3D(q.inverseAndReturnNew().rotate(Point3D.create(
                CoordinatesType.INHOMOGENEOUS_COORDINATES, point)));
        rotated.asArray(result2);

        assertArrayEquals(result2, result1, ABSOLUTE_ERROR);

        assertTrue(jacobianP.equals(q.asInhomogeneousMatrix().transposeAndReturnNew(), ABSOLUTE_ERROR));

        // check jacobian
        var a = q.getA();
        var b = q.getB();
        var c = q.getC();
        var d = q.getD();

        var x = point[0];
        var y = point[1];
        var z = point[2];

        var axdycz = 2.0 * (a * x + d * y - c * z);
        var bxcydz = 2.0 * (b * x + c * y + d * z);
        var cxbyaz = 2.0 * (c * x - b * y + a * z);
        var dxaybz = 2.0 * (d * x - a * y - b * z);

        assertEquals(axdycz, jacobianQ.getElementAt(0, 0), ABSOLUTE_ERROR);
        assertEquals(-dxaybz, jacobianQ.getElementAt(1, 0), ABSOLUTE_ERROR);
        assertEquals(cxbyaz, jacobianQ.getElementAt(2, 0), ABSOLUTE_ERROR);

        assertEquals(bxcydz, jacobianQ.getElementAt(0, 1), ABSOLUTE_ERROR);
        assertEquals(cxbyaz, jacobianQ.getElementAt(1, 1), ABSOLUTE_ERROR);
        assertEquals(dxaybz, jacobianQ.getElementAt(2, 1), ABSOLUTE_ERROR);

        assertEquals(-cxbyaz, jacobianQ.getElementAt(0, 2), ABSOLUTE_ERROR);
        assertEquals(bxcydz, jacobianQ.getElementAt(1, 2), ABSOLUTE_ERROR);
        assertEquals(axdycz, jacobianQ.getElementAt(2, 2), ABSOLUTE_ERROR);

        assertEquals(-dxaybz, jacobianQ.getElementAt(0, 3), ABSOLUTE_ERROR);
        assertEquals(-axdycz, jacobianQ.getElementAt(1, 3), ABSOLUTE_ERROR);
        assertEquals(bxcydz, jacobianQ.getElementAt(2, 3), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var m = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> RotationUtils.transposedRotationMatrixTimesVector(q, point, result1, m, jacobianP));
        assertThrows(IllegalArgumentException.class,
                () -> RotationUtils.transposedRotationMatrixTimesVector(q, point, result1, jacobianQ, m));

        // test without result
        final var result3 = RotationUtils.transposedRotationMatrixTimesVector(q, point, jacobianQ, jacobianP);

        // check correctness
        rotated = new InhomogeneousPoint3D(q.rotate(Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, point)));
        rotated.asArray(result2);

        assertArrayEquals(result2, result3, ABSOLUTE_ERROR);

        assertTrue(jacobianP.equals(q.asInhomogeneousMatrix().transposeAndReturnNew(), ABSOLUTE_ERROR));

        // check jacobian
        a = q.getA();
        b = q.getB();
        c = q.getC();
        d = q.getD();

        x = point[0];
        y = point[1];
        z = point[2];

        axdycz = 2.0 * (a * x + d * y - c * z);
        bxcydz = 2.0 * (b * x + c * y + d * z);
        cxbyaz = 2.0 * (c * x - b * y + a * z);
        dxaybz = 2.0 * (d * x - a * y - b * z);

        assertEquals(axdycz, jacobianQ.getElementAt(0, 0), ABSOLUTE_ERROR);
        assertEquals(-dxaybz, jacobianQ.getElementAt(1, 0), ABSOLUTE_ERROR);
        assertEquals(cxbyaz, jacobianQ.getElementAt(2, 0), ABSOLUTE_ERROR);

        assertEquals(bxcydz, jacobianQ.getElementAt(0, 1), ABSOLUTE_ERROR);
        assertEquals(cxbyaz, jacobianQ.getElementAt(1, 1), ABSOLUTE_ERROR);
        assertEquals(dxaybz, jacobianQ.getElementAt(2, 1), ABSOLUTE_ERROR);

        assertEquals(-cxbyaz, jacobianQ.getElementAt(0, 2), ABSOLUTE_ERROR);
        assertEquals(bxcydz, jacobianQ.getElementAt(1, 2), ABSOLUTE_ERROR);
        assertEquals(axdycz, jacobianQ.getElementAt(2, 2), ABSOLUTE_ERROR);

        assertEquals(-dxaybz, jacobianQ.getElementAt(0, 3), ABSOLUTE_ERROR);
        assertEquals(-axdycz, jacobianQ.getElementAt(1, 3), ABSOLUTE_ERROR);
        assertEquals(bxcydz, jacobianQ.getElementAt(2, 3), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> RotationUtils.transposedRotationMatrixTimesVector(q,
                point, m1, jacobianP));
        assertThrows(IllegalArgumentException.class, () -> RotationUtils.transposedRotationMatrixTimesVector(q,
                point, jacobianQ, m1));

        // test without jacobian
        RotationUtils.transposedRotationMatrixTimesVector(q, point, result3);

        // check correctness
        rotated = new InhomogeneousPoint3D(q.rotate(Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, point)));
        rotated.asArray(result2);

        assertArrayEquals(result2, result3, ABSOLUTE_ERROR);

        assertTrue(jacobianP.equals(q.asInhomogeneousMatrix().transposeAndReturnNew(), ABSOLUTE_ERROR));

        // test without jacobian and result
        final var result4 = RotationUtils.transposedRotationMatrixTimesVector(q, point);

        // check correctness
        rotated = new InhomogeneousPoint3D(q.rotate(Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, point)));
        rotated.asArray(result2);

        assertArrayEquals(result2, result4, ABSOLUTE_ERROR);

        assertTrue(jacobianP.equals(q.asInhomogeneousMatrix().transposeAndReturnNew(), ABSOLUTE_ERROR));

        final var p = new InhomogeneousPoint3D(point);
        Point3D resultPoint = new InhomogeneousPoint3D();
        RotationUtils.transposedRotationMatrixTimesVector(q, p, resultPoint, null, null);
        final var result5 = resultPoint.asArray();
        assertArrayEquals(result2, result5, ABSOLUTE_ERROR);

        resultPoint = RotationUtils.transposedRotationMatrixTimesVector(q, p, null, null);
        final var result6 = new InhomogeneousPoint3D(resultPoint).asArray();
        assertArrayEquals(result2, result6, ABSOLUTE_ERROR);

        resultPoint = new InhomogeneousPoint3D();
        RotationUtils.transposedRotationMatrixTimesVector(q, p, resultPoint);
        final var result7 = resultPoint.asArray();
        assertArrayEquals(result2, result7, ABSOLUTE_ERROR);

        resultPoint = RotationUtils.transposedRotationMatrixTimesVector(q, p);
        final var result8 = new InhomogeneousPoint3D(resultPoint).asArray();
        assertArrayEquals(result2, result8, ABSOLUTE_ERROR);
    }

    @Test
    void testCameraBodyToCameraSensorRotation() throws RotationException {
        final var rot = new MatrixRotation3D();
        RotationUtils.cameraBodyToCameraSensorRotation(rot);

        final var rot2 = RotationUtils.cameraBodyToCameraSensorRotation();

        // check correctness
        assertTrue(rot.equals(rot2, ABSOLUTE_ERROR));

        assertEquals(-Math.PI / 2.0, rot.getRollAngle(), ABSOLUTE_ERROR);
        assertEquals(0.0, rot.getPitchAngle(), ABSOLUTE_ERROR);
        assertEquals(-Math.PI / 2.0, rot.getYawAngle(), ABSOLUTE_ERROR);

        assertEquals(-Math.PI / 2.0, rot2.getRollAngle(), ABSOLUTE_ERROR);
        assertEquals(0.0, rot2.getPitchAngle(), ABSOLUTE_ERROR);
        assertEquals(-Math.PI / 2.0, rot2.getYawAngle(), ABSOLUTE_ERROR);
    }

    @Test
    void testQuaternionToEulerGaussian() throws AlgebraException {
        final var randomizer = new UniformRandomizer();

        final var roll = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));

        final var q = new Quaternion(roll, pitch, yaw);

        final var covQ = Matrix.identity(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
        final var euler = new double[Quaternion.N_ANGLES];
        final var covE = new Matrix(Quaternion.N_ANGLES, Quaternion.N_ANGLES);
        RotationUtils.quaternionToEulerGaussian(q, covQ, euler, covE);

        // check correctness
        final var angles = q.toEulerAngles();
        assertArrayEquals(euler, angles, ABSOLUTE_ERROR);

        final var j = new Matrix(Quaternion.N_ANGLES, Quaternion.N_PARAMS);
        q.toEulerAngles(angles, j);
        final var covE2 = j.multiplyAndReturnNew(covQ).multiplyAndReturnNew(j.transposeAndReturnNew());

        assertEquals(covE, covE2);
    }

    @Test
    void testAngularRatesToSkew() throws AlgebraException {
        final var randomizer = new UniformRandomizer();

        final var wx = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var wy = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var wz = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var angularRates = new double[]{wx, wy, wz};

        final var skew1 = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
        RotationUtils.angularRatesToSkew(angularRates, skew1);
        var skew2 = RotationUtils.angularRatesToSkew(angularRates);

        assertEquals(skew1, skew2);

        final var skew3 = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
        skew3.setElementAtIndex(0, 0.0);
        skew3.setElementAtIndex(1, wx);
        skew3.setElementAtIndex(2, wy);
        skew3.setElementAtIndex(3, wz);

        skew3.setElementAtIndex(4, -wx);
        skew3.setElementAtIndex(5, 0.0);
        skew3.setElementAtIndex(6, -wz);
        skew3.setElementAtIndex(7, wy);

        skew3.setElementAtIndex(8, -wy);
        skew3.setElementAtIndex(9, wz);
        skew3.setElementAtIndex(10, 0.0);
        skew3.setElementAtIndex(11, -wx);

        skew3.setElementAtIndex(12, -wz);
        skew3.setElementAtIndex(13, -wy);
        skew3.setElementAtIndex(14, wx);
        skew3.setElementAtIndex(15, 0.0);

        assertEquals(skew1, skew3);
        assertEquals(skew2, skew3);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> RotationUtils.angularRatesToSkew(new double[1], skew1));
        final var m = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> RotationUtils.angularRatesToSkew(angularRates, m));
        assertThrows(IllegalArgumentException.class, () -> RotationUtils.angularRatesToSkew(new double[1]));
    }
}
