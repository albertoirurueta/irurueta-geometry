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

import com.irurueta.algebra.NotAvailableException;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.*;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class Rotation3DTest {
    private static final int ROTATION_COLS = 3;
    private static final int INHOM_COORDS = 3;

    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    private static final double MIN_ANGLE_DEGREES = -90.0;
    private static final double MAX_ANGLE_DEGREES = 90.0;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final int TIMES = 10;

    @Test
    public void testConstants() {
        assertEquals(1e-12, Rotation3D.DEFAULT_VALID_THRESHOLD, 0.0);
        assertEquals(0.0, Rotation3D.MIN_THRESHOLD, 0.0);
        assertEquals(3, Rotation3D.INHOM_COORDS);
        assertEquals(4, Rotation3D.HOM_COORDS);
        assertEquals(Rotation3DType.QUATERNION, Rotation3D.DEFAULT_TYPE);
        assertEquals(1e-9, Rotation3D.DEFAULT_COMPARISON_THRESHOLD, 0.0);
    }

    @Test
    public void testCreate() throws WrongSizeException, RotationException {
        Rotation3D rotation = Rotation3D.create();
        assertEquals(Rotation3D.DEFAULT_TYPE, rotation.getType());
        Matrix m = rotation.asInhomogeneousMatrix();
        assertTrue(m.equals(Matrix.identity(INHOM_COORDS, INHOM_COORDS), ABSOLUTE_ERROR));

        rotation = Rotation3D.create(Rotation3DType.AXIS_ROTATION3D);
        assertEquals(Rotation3DType.AXIS_ROTATION3D, rotation.getType());
        m = rotation.asInhomogeneousMatrix();
        assertTrue(m.equals(Matrix.identity(INHOM_COORDS, INHOM_COORDS), ABSOLUTE_ERROR));

        rotation = Rotation3D.create(Rotation3DType.MATRIX_ROTATION3D);
        assertEquals(Rotation3DType.MATRIX_ROTATION3D, rotation.getType());
        m = rotation.asInhomogeneousMatrix();
        assertTrue(m.equals(Matrix.identity(INHOM_COORDS, INHOM_COORDS), ABSOLUTE_ERROR));

        rotation = Rotation3D.create(Rotation3DType.QUATERNION);
        assertEquals(Rotation3DType.QUATERNION, rotation.getType());
        m = rotation.asInhomogeneousMatrix();
        assertTrue(m.equals(Matrix.identity(INHOM_COORDS, INHOM_COORDS), ABSOLUTE_ERROR));

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double axisX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double axisY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double axisZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis values to get better results
        final double norm = Math.sqrt(axisX * axisX + axisY * axisY + axisZ * axisZ);
        axisX /= norm;
        axisY /= norm;
        axisZ /= norm;
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        final double[] axis = new double[INHOM_COORDS];
        axis[0] = axisX;
        axis[1] = axisY;
        axis[2] = axisZ;

        rotation = Rotation3D.create(axis, theta);
        assertEquals(Rotation3D.DEFAULT_TYPE, rotation.getType());
        double[] axis2 = rotation.getRotationAxis();
        // check axis are equal up to scale
        double scaleX = axis[0] / axis2[0];
        double scaleY = axis[1] / axis2[1];
        double scaleZ = axis[2] / axis2[2];
        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleZ, ABSOLUTE_ERROR);
        assertEquals(scaleZ, scaleX, ABSOLUTE_ERROR);
        assertEquals(1.0, Math.abs(theta / rotation.getRotationAngle()), ABSOLUTE_ERROR);

        rotation = Rotation3D.create(axis, theta, Rotation3DType.AXIS_ROTATION3D);
        assertEquals(Rotation3DType.AXIS_ROTATION3D, rotation.getType());
        assertArrayEquals(axis, rotation.getRotationAxis(), ABSOLUTE_ERROR);
        assertEquals(theta, rotation.getRotationAngle(), ABSOLUTE_ERROR);

        rotation = Rotation3D.create(axis, theta, Rotation3DType.MATRIX_ROTATION3D);
        assertEquals(Rotation3DType.MATRIX_ROTATION3D, rotation.getType());
        axis2 = rotation.getRotationAxis();
        // check axis are equal up to scale
        scaleX = axis[0] / axis2[0];
        scaleY = axis[1] / axis2[1];
        scaleZ = axis[2] / axis2[2];
        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleZ, ABSOLUTE_ERROR);
        assertEquals(scaleZ, scaleX, ABSOLUTE_ERROR);
        assertEquals(1.0, Math.abs(theta / rotation.getRotationAngle()), ABSOLUTE_ERROR);

        rotation = Rotation3D.create(axis, theta, Rotation3DType.QUATERNION);
        assertEquals(Rotation3DType.QUATERNION, rotation.getType());
        axis2 = rotation.getRotationAxis();
        // check axis are equal up to scale
        scaleX = axis[0] / axis2[0];
        scaleY = axis[1] / axis2[1];
        scaleZ = axis[2] / axis2[2];
        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleZ, ABSOLUTE_ERROR);
        assertEquals(scaleZ, scaleX, ABSOLUTE_ERROR);
        assertEquals(1.0, Math.abs(theta / rotation.getRotationAngle()), ABSOLUTE_ERROR);

        rotation = Rotation3D.create(axisX, axisY, axisZ, theta);
        assertEquals(Rotation3D.DEFAULT_TYPE, rotation.getType());
        axis2 = rotation.getRotationAxis();
        scaleX = axisX / axis2[0];
        scaleY = axisY / axis2[1];
        scaleZ = axisZ / axis2[2];
        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleZ, ABSOLUTE_ERROR);
        assertEquals(scaleZ, scaleX, ABSOLUTE_ERROR);
        assertEquals(1.0, Math.abs(theta / rotation.getRotationAngle()), ABSOLUTE_ERROR);

        rotation = Rotation3D.create(axisX, axisY, axisZ, theta, Rotation3DType.AXIS_ROTATION3D);
        assertEquals(Rotation3DType.AXIS_ROTATION3D, rotation.getType());
        assertEquals(axisX, rotation.getRotationAxis()[0], ABSOLUTE_ERROR);
        assertEquals(axisY, rotation.getRotationAxis()[1], ABSOLUTE_ERROR);
        assertEquals(axisZ, rotation.getRotationAxis()[2], ABSOLUTE_ERROR);
        assertEquals(theta, rotation.getRotationAngle(), ABSOLUTE_ERROR);

        rotation = Rotation3D.create(axisX, axisY, axisZ, theta, Rotation3DType.MATRIX_ROTATION3D);
        assertEquals(Rotation3DType.MATRIX_ROTATION3D, rotation.getType());
        axis2 = rotation.getRotationAxis();
        scaleX = axisX / axis2[0];
        scaleY = axisY / axis2[1];
        scaleZ = axisZ / axis2[2];
        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleZ, ABSOLUTE_ERROR);
        assertEquals(scaleZ, scaleX, ABSOLUTE_ERROR);
        assertEquals(1.0, Math.abs(theta / rotation.getRotationAngle()), ABSOLUTE_ERROR);

        rotation = Rotation3D.create(axisX, axisY, axisZ, theta, Rotation3DType.QUATERNION);
        assertEquals(Rotation3DType.QUATERNION, rotation.getType());
        axis2 = rotation.getRotationAxis();
        scaleX = axisX / axis2[0];
        scaleY = axisY / axis2[1];
        scaleZ = axisZ / axis2[2];
        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleZ, ABSOLUTE_ERROR);
        assertEquals(scaleZ, scaleX, ABSOLUTE_ERROR);
        assertEquals(1.0, Math.abs(theta / rotation.getRotationAngle()), ABSOLUTE_ERROR);
    }

    @Test
    public void testGetSetAxisAndRotation() throws WrongSizeException, NotReadyException,
            LockedException, DecomposerException, NotAvailableException, RotationException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        final Rotation3D rotation = Rotation3D.create();

        // Force IllegalArgumentException
        double[] axis = new double[INHOM_COORDS + 1];

        try {
            rotation.setAxisAndRotation(axis, theta);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        // Find any 3 orthogonal vectors, 1st will be axis of rotation, and the
        // remaining two will lie on the rotation plane and will be used to test
        // theta angle

        // To find 3 orthogonal vectors, we use V matrix of a singular value
        // decomposition of any Nx3 matrix
        final Matrix a = Matrix.createWithUniformRandomValues(1, ROTATION_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final SingularValueDecomposer decomposer = new SingularValueDecomposer(a);

        decomposer.decompose();

        final Matrix v = decomposer.getV();

        // axis of rotation
        final Matrix axisMatrix = v.getSubmatrix(0, 0, 2,
                0);

        // inhomogeneous coordinates of point laying on rotation plane
        final Matrix pointMatrix = v.getSubmatrix(0, 1, 2,
                1);

        axis = axisMatrix.toArray();
        rotation.setAxisAndRotation(axis, theta);

        // To test correctness of rotation, axis should remain equal
        final Matrix rotationMatrix = rotation.asInhomogeneousMatrix();

        final Matrix axisMatrix2 = rotationMatrix.multiplyAndReturnNew(axisMatrix);

        assertTrue(axisMatrix.equals(axisMatrix2, ABSOLUTE_ERROR));

        final double[] axis2 = rotation.getRotationAxis();
        final double[] axis3 = new double[3];
        rotation.rotationAxis(axis3);

        final double scaleX = axis[0] / axis2[0];
        final double scaleY = axis[0] / axis2[0];
        final double scaleZ = axis[0] / axis2[0];

        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleZ, ABSOLUTE_ERROR);
        assertEquals(scaleZ, scaleX, ABSOLUTE_ERROR);
        assertArrayEquals(axis2, axis3, 0.0);

        // and point on rotated plane should be rotated exactly theta
        // radians
        final Matrix pointMatrix2 = rotationMatrix.multiplyAndReturnNew(pointMatrix);

        // because vPoint is already normalized (from SVD decomposition)
        // we only need to normalize rotated point to compute the rotation
        // angle as the arc-cosine of their dot product
        final double norm2 = Utils.normF(pointMatrix2);
        pointMatrix2.multiplyByScalar(1.0 / norm2);

        final double dotProduct = pointMatrix.transposeAndReturnNew().
                multiplyAndReturnNew(pointMatrix2).getElementAtIndex(0);

        final double theta2 = Math.acos(dotProduct);
        final double theta2b = rotation.getRotationAngle();

        // check correctness of angles (up to sign for this test)
        assertEquals(Math.abs(theta), Math.abs(theta2), ABSOLUTE_ERROR);

        // check correctness of angles (including sign) for method in class
        if (scaleX > 0.0) {
            assertEquals(theta2b, theta, ABSOLUTE_ERROR);
        } else {
            assertEquals(-theta2b, theta, ABSOLUTE_ERROR);
        }
    }

    @Test
    public void testGetSetAxisAndRotation2() throws WrongSizeException, NotReadyException,
            LockedException, DecomposerException, NotAvailableException, RotationException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        final Rotation3D rotation = Rotation3D.create();

        // Find any 3 orthogonal vectors, 1st will be axis of rotation, and the
        // remaining two will lie on the rotation plane and will be used to test
        // theta angle

        // To find 3 orthogonal vectors, we use V matrix of a singular value
        // decomposition of any Nx3 matrix
        final Matrix a = Matrix.createWithUniformRandomValues(1, ROTATION_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final SingularValueDecomposer decomposer = new SingularValueDecomposer(a);

        decomposer.decompose();

        final Matrix v = decomposer.getV();

        // axis of rotation
        final Matrix axisMatrix = v.getSubmatrix(0, 0, 2,
                0);

        // inhomogeneous coordinates of point laying on rotation plane
        final Matrix pointMatrix = v.getSubmatrix(0, 1, 2,
                1);

        final double axisX = axisMatrix.getElementAtIndex(0);
        final double axisY = axisMatrix.getElementAtIndex(1);
        final double axisZ = axisMatrix.getElementAtIndex(2);
        final double[] axis = axisMatrix.toArray();
        rotation.setAxisAndRotation(axisX, axisY, axisZ, theta);

        // To test correctness of rotation, axis should remain equal
        final Matrix rotationMatrix = rotation.asInhomogeneousMatrix();

        final Matrix axisMatrix2 = rotationMatrix.multiplyAndReturnNew(axisMatrix);

        assertTrue(axisMatrix.equals(axisMatrix2, ABSOLUTE_ERROR));

        final double[] axis2 = rotation.getRotationAxis();
        final double[] axis3 = new double[3];
        rotation.rotationAxis(axis3);

        final double scaleX = axis[0] / axis2[0];
        final double scaleY = axis[0] / axis2[0];
        final double scaleZ = axis[0] / axis2[0];

        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleZ, ABSOLUTE_ERROR);
        assertEquals(scaleZ, scaleX, ABSOLUTE_ERROR);
        assertArrayEquals(axis2, axis3, 0.0);

        // and point on rotated plane should be rotated exactly theta
        // radians
        final Matrix pointMatrix2 = rotationMatrix.multiplyAndReturnNew(pointMatrix);

        // because vPoint is already normalized (from SVD decomposition)
        // we only need to normalize rotated point to compute the rotation
        // angle as the arc-cosine of their dot product
        final double norm2 = Utils.normF(pointMatrix2);
        pointMatrix2.multiplyByScalar(1.0 / norm2);

        final double dotProduct = pointMatrix.transposeAndReturnNew().
                multiplyAndReturnNew(pointMatrix2).getElementAtIndex(0);

        final double theta2 = Math.acos(dotProduct);
        final double theta2b = rotation.getRotationAngle();

        // check correctness of angles (up to sign for this test)
        assertEquals(Math.abs(theta), Math.abs(theta2), ABSOLUTE_ERROR);

        // check correctness of angles (including sign) for method in class
        if (scaleX > 0.0) {
            assertEquals(theta2b, theta, ABSOLUTE_ERROR);
        } else {
            assertEquals(-theta2b, theta, ABSOLUTE_ERROR);
        }
    }

    @Test
    public void testEquals() throws RotationException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double axisX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            double axisY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            double axisZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            // normalize axis values to get better results
            final double norm = Math.sqrt(axisX * axisX + axisY * axisY + axisZ * axisZ);
            axisX /= norm;
            axisY /= norm;
            axisZ /= norm;
            final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double threshold = randomizer.nextDouble(
                    Rotation2D.DEFAULT_COMPARISON_THRESHOLD,
                    2.0 * Rotation2D.DEFAULT_COMPARISON_THRESHOLD);
            final double theta2 = theta + threshold;

            final double[] axis = new double[INHOM_COORDS];
            axis[0] = axisX;
            axis[1] = axisY;
            axis[2] = axisZ;

            final double[] axis2 = ArrayUtils.multiplyByScalarAndReturnNew(axis, -1.0);

            final Rotation3D rotation1 = Rotation3D.create(axis, theta);
            // with same axis and rotation angle
            final Rotation3D rotation2 = Rotation3D.create(axis, theta);
            // with reversed axis and rotation angle
            final Rotation3D rotation3 = Rotation3D.create(axis2, -theta);

            // with different angle
            final Rotation3D rotation4 = Rotation3D.create(axis, theta2);
            // with different axis
            final double[] axis3 = new double[INHOM_COORDS];
            axis3[0] = axisX + randomizer.nextDouble();
            axis3[1] = axisY + randomizer.nextDouble();
            axis3[2] = axisZ + randomizer.nextDouble();
            final Rotation3D rotation5 = Rotation3D.create(axis3, theta);

            // check equal-ness
            //noinspection EqualsWithItself
            assertEquals(rotation1, rotation1);
            assertTrue(rotation1.equals(rotation1, Rotation3D.DEFAULT_COMPARISON_THRESHOLD));
            assertEquals(rotation1, rotation2);
            assertEquals(rotation1.hashCode(), rotation2.hashCode());
            assertTrue(rotation1.equals(rotation2, Rotation3D.DEFAULT_COMPARISON_THRESHOLD));
            assertEquals(rotation1, rotation3);
            assertTrue(rotation1.equals(rotation3, Rotation3D.DEFAULT_COMPARISON_THRESHOLD));

            assertNotEquals(rotation1, rotation4);
            assertFalse(rotation1.equals(rotation4, Rotation3D.DEFAULT_COMPARISON_THRESHOLD));
            assertNotEquals(rotation1, rotation5);
            assertFalse(rotation1.equals(rotation5, Rotation3D.DEFAULT_COMPARISON_THRESHOLD));

            // check with larger threshold
            assertTrue(rotation1.equals(rotation4, 2.0 * Math.PI));
            final double largeThreshold = 5.0 * Utils.normF(axis3);
            if (!rotation1.equals(rotation5, largeThreshold)) {
                continue;
            }
            assertTrue(rotation1.equals(rotation5, largeThreshold));
            numValid++;
            break;
        }
        assertTrue(numValid > 0);
    }

    @Test
    public void testFromRotation() throws AlgebraException, RotationException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            // Find any 3 orthogonal vectors, 1st will be axis of rotation, and
            // the remaining two will lie on the rotation plane and will be used
            // to test theta angle

            // To find 3 orthogonal vectors, we use V matrix of a singular
            // decomposition of any Nx3 matrix
            final Matrix a = Matrix.createWithUniformRandomValues(1, ROTATION_COLS,
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final SingularValueDecomposer decomposer = new SingularValueDecomposer(a);

            decomposer.decompose();

            final Matrix vMatrix = decomposer.getV();

            // axis of rotation
            final double[] axis = vMatrix.getSubmatrixAsArray(
                    0, 0, 2, 0);

            final MatrixRotation3D matrixRotation = new MatrixRotation3D();
            final AxisRotation3D axisRotation = new AxisRotation3D();
            final Quaternion quaternion = new Quaternion();

            matrixRotation.setAxisAndRotation(axis, theta);
            axisRotation.setAxisAndRotation(axis, theta);
            quaternion.setAxisAndRotation(axis, theta);

            // test from matrix rotation
            final Rotation3D rotation1 = Rotation3D.create();
            rotation1.fromRotation(matrixRotation);

            // check correctness
            if (!matrixRotation.equals(rotation1, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(matrixRotation.equals(rotation1, ABSOLUTE_ERROR));

            // test from axis rotation
            final Rotation3D rotation2 = Rotation3D.create();
            rotation2.fromRotation(axisRotation);

            // check correctness
            if (!axisRotation.equals(rotation2, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(axisRotation.equals(rotation2, ABSOLUTE_ERROR));

            // test from quaternion
            final Rotation3D rotation3 = Rotation3D.create();
            rotation3.fromRotation(quaternion);

            // check correctness
            if (!quaternion.equals(rotation3, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(quaternion.equals(rotation3, ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testToMatrixRotation() throws AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        // Find any 3 orthogonal vectors, 1st will be axis of rotation, and
        // the remaining two will lie on the rotation plane and will be used
        // to test theta angle

        // To find 3 orthogonal vectors, we use V matrix of a singular
        // decomposition of any Nx3 matrix
        final Matrix a = Matrix.createWithUniformRandomValues(1, ROTATION_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final SingularValueDecomposer decomposer = new SingularValueDecomposer(a);

        decomposer.decompose();

        final Matrix vMatrix = decomposer.getV();

        // axis of rotation
        final double[] axis = vMatrix.getSubmatrixAsArray(0, 0,
                2, 0);

        final Rotation3D rotation = Rotation3D.create(axis, theta);

        // to matrix rotation
        final MatrixRotation3D matrixRotation1 = new MatrixRotation3D();
        rotation.toMatrixRotation(matrixRotation1);
        final MatrixRotation3D matrixRotation2 = rotation.toMatrixRotation();

        // check correctness
        assertEquals(rotation, matrixRotation1);
        assertEquals(rotation, matrixRotation2);

        // to axis rotation
        final AxisRotation3D axisRotation1 = new AxisRotation3D();
        rotation.toAxisRotation(axisRotation1);
        final AxisRotation3D axisRotation2 = rotation.toAxisRotation();

        // check correctness
        assertEquals(rotation, axisRotation1);
        assertEquals(rotation, axisRotation2);

        // to quaternion
        final Quaternion quaternion1 = new Quaternion();
        rotation.toQuaternion(quaternion1);
        final Quaternion quaternion2 = rotation.toQuaternion();

        // check correctness
        assertEquals(rotation, quaternion1);
        assertEquals(rotation, quaternion2);
    }
}
