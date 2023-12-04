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

public class AxisRotation3DTest {
    private static final int ROTATION_COLS = 3;
    private static final int INHOM_COORDS = 3;
    private static final int HOM_COORDS = 4;

    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    private static final double MIN_RANDOM_SCALE = 2.0;
    private static final double MAX_RANDOM_SCALE = 3.0;

    private static final double MIN_ANGLE_DEGREES = 0.0;
    private static final double MAX_ANGLE_DEGREES = 90.0;

    private static final double ABSOLUTE_ERROR = 5e-6;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-5;

    private static final int TIMES = 10;

    @Test
    public void testConstants() {
        assertEquals(3, AxisRotation3D.AXIS_PARAMS);
        assertEquals(1e-12, AxisRotation3D.EPS, 0.0);
    }

    @Test
    public void testConstructor() throws RotationException, WrongSizeException,
            NotReadyException, LockedException, DecomposerException,
            NotAvailableException {
        // Test empty constructor
        AxisRotation3D rotation = new AxisRotation3D();
        assertNotNull(rotation);

        assertEquals(0.0, rotation.getAxisX(), 0.0);
        assertEquals(0.0, rotation.getAxisY(), 0.0);
        assertEquals(1.0, rotation.getAxisZ(), 0.0);
        assertEquals(0.0, rotation.getRotationAngle(), 0.0);

        // check that empty constructor creates a rotation without effect (its
        // matrix representation is equal to the identity
        assertTrue(rotation.asInhomogeneousMatrix().equals(
                Matrix.identity(INHOM_COORDS, INHOM_COORDS), ABSOLUTE_ERROR));

        // test constructor using axis and angle
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        // Force WrongSizeException
        double[] axis = new double[INHOM_COORDS + 1];

        rotation = null;
        try {
            rotation = new AxisRotation3D(axis, theta);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(rotation);

        // Find any 3 orthogonal vectors, 1st will be axis of rotation, and
        // the remaining two will lie on the rotation plane and will be used
        // to test theta angle

        // To find 3 orthogonal vectors, we use V matrix of a singular
        // decomposition of any Nx3 matrix
        final Matrix a = Matrix.createWithUniformRandomValues(1, INHOM_COORDS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final SingularValueDecomposer decomposer = new SingularValueDecomposer(a);

        decomposer.decompose();

        final Matrix vMatrix = decomposer.getV();

        // axis of rotation
        final double[] vAxis = vMatrix.getSubmatrixAsArray(0, 0, 2, 0);
        final Matrix mAxis = Matrix.newFromArray(vAxis, true);

        // inhomogeneous coordinates of point laying on rotation plane
        final double[] vPoint = vMatrix.getSubmatrixAsArray(0, 1, 2, 1);

        rotation = new AxisRotation3D(vAxis, theta);

        // To test correctness of rotation, axis should remain equal
        final Matrix rotationMatrix = rotation.asInhomogeneousMatrix();

        final Matrix mAxis2 = rotationMatrix.multiplyAndReturnNew(mAxis);

        assertEquals(mAxis.getElementAtIndex(0),
                mAxis2.getElementAtIndex(0), ABSOLUTE_ERROR);
        assertEquals(mAxis.getElementAtIndex(1),
                mAxis2.getElementAtIndex(1), ABSOLUTE_ERROR);
        assertEquals(mAxis.getElementAtIndex(2),
                mAxis2.getElementAtIndex(2), ABSOLUTE_ERROR);

        axis = rotation.getRotationAxis();

        final double scaleX = axis[0] / vAxis[0];
        final double scaleY = axis[0] / vAxis[0];
        final double scaleZ = axis[0] / vAxis[0];

        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleZ, ABSOLUTE_ERROR);
        assertEquals(scaleZ, scaleX, ABSOLUTE_ERROR);

        // and point on rotated plane should be rotated exactly theta
        // radians
        final Matrix mPoint = Matrix.newFromArray(vPoint);
        // below: rotated point
        final Matrix mPoint2 = rotationMatrix.multiplyAndReturnNew(mPoint);

        // because vPoint is already normalized (from SVD decomposition)
        // we only need to normalize rotated point to compute the rotation
        // angle as the arc-cosine of their dot product
        final double norm2 = Utils.normF(mPoint2);
        mPoint2.multiplyByScalar(1.0 / norm2);

        final double dotProduct = mPoint.transposeAndReturnNew().
                multiplyAndReturnNew(mPoint2).getElementAtIndex(0);

        final double theta2 = Math.acos(dotProduct);
        final double theta2b = rotation.getRotationAngle();

        // check correctness of angles (up to sign for this test)
        assertEquals(Math.abs(theta), Math.abs(theta2), ABSOLUTE_ERROR);

        // check correctness of angles (including sign) for method in class
        if (scaleX > 0.0) {
            assertEquals(theta, theta2b, ABSOLUTE_ERROR);
        } else {
            assertEquals(theta, -theta2b, ABSOLUTE_ERROR);
        }

        // Test copy constructor
        AxisRotation3D rotation2 = new AxisRotation3D(
                rotation);
        assertEquals(rotation2.asInhomogeneousMatrix(),
                rotation.asInhomogeneousMatrix());

        rotation2 = new AxisRotation3D(rotation.toQuaternion());
        assertTrue(rotation2.equals(rotation, ABSOLUTE_ERROR));
    }

    @Test
    public void testGetSetAxisAndRotation() throws WrongSizeException,
            NotReadyException, LockedException, DecomposerException,
            NotAvailableException, RotationException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        final AxisRotation3D rotation = new AxisRotation3D();

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
        final Matrix a = Matrix.createWithUniformRandomValues(1, INHOM_COORDS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final SingularValueDecomposer decomposer = new SingularValueDecomposer(a);

        decomposer.decompose();

        final Matrix v = decomposer.getV();

        // axis of rotation
        final Matrix axisMatrix = v.getSubmatrix(0, 0, 2, 0);

        // inhomogeneous coordinates of point laying on rotation plane
        final Matrix pointMatrix = v.getSubmatrix(0, 1, 2, 1);

        axis = axisMatrix.toArray();
        rotation.setAxisAndRotation(axis, theta);

        // To test correctness of rotation, axis should remain equal
        final Matrix rotationMatrix = rotation.asInhomogeneousMatrix();

        final Matrix axisMatrix2 = rotationMatrix.multiplyAndReturnNew(axisMatrix);

        assertTrue(axisMatrix.equals(axisMatrix2, ABSOLUTE_ERROR));

        final double[] axis2 = rotation.getRotationAxis();

        final double scaleX = axis[0] / axis2[0];
        final double scaleY = axis[0] / axis2[0];
        final double scaleZ = axis[0] / axis2[0];

        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleZ, ABSOLUTE_ERROR);
        assertEquals(scaleZ, scaleX, ABSOLUTE_ERROR);

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
            assertEquals(theta, theta2b, ABSOLUTE_ERROR);
        } else {
            assertEquals(theta, -theta2b, ABSOLUTE_ERROR);
        }
    }

    @Test
    public void testSetAxis() throws WrongSizeException,
            NotReadyException, LockedException, DecomposerException,
            NotAvailableException, RotationException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        final AxisRotation3D rotation = new AxisRotation3D();

        // Find any 3 orthogonal vectors, 1st will be axis of rotation, and the
        // remaining two will lie on the rotation plane and will be used to test
        // theta angle

        // To find 3 orthogonal vectors, we use V matrix of a singular value
        // decomposition of any Nx3 matrix
        final Matrix a = Matrix.createWithUniformRandomValues(1, INHOM_COORDS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final SingularValueDecomposer decomposer = new SingularValueDecomposer(a);

        decomposer.decompose();

        final Matrix v = decomposer.getV();

        // axis of rotation
        final Matrix axisMatrix = v.getSubmatrix(0, 0, 2, 0);

        // inhomogeneous coordinates of point laying on rotation plane
        final Matrix pointMatrix = v.getSubmatrix(0, 1, 2, 1);

        double[] axis = axisMatrix.toArray();
        ArrayUtils.multiplyByScalar(axis, theta, axis);
        rotation.setAxis(axis[0], axis[1], axis[2]);

        // To test correctness of rotation, axis should remain equal
        final Matrix rotationMatrix = rotation.asInhomogeneousMatrix();

        final Matrix axisMatrix2 = rotationMatrix.multiplyAndReturnNew(axisMatrix);

        assertTrue(axisMatrix.equals(axisMatrix2, ABSOLUTE_ERROR));

        final double[] axis2 = rotation.getRotationAxis();

        final double scaleX = axis[0] / axis2[0];
        final double scaleY = axis[0] / axis2[0];
        final double scaleZ = axis[0] / axis2[0];

        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleZ, ABSOLUTE_ERROR);
        assertEquals(scaleZ, scaleX, ABSOLUTE_ERROR);

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
            assertEquals(theta, theta2b, ABSOLUTE_ERROR);
        } else {
            assertEquals(theta, -theta2b, ABSOLUTE_ERROR);
        }
    }

    @Test
    public void testIsValidRotationMatrix()
            throws WrongSizeException, NotReadyException, LockedException,
            DecomposerException, NotAvailableException {

        final Matrix a = Matrix.createWithUniformRandomValues(INHOM_COORDS,
                INHOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // Use SVD to obtain an orthonormal matrix from V matrix
        final SingularValueDecomposer decomposer = new SingularValueDecomposer(a);

        decomposer.decompose();

        final Matrix rotationMatrix = decomposer.getV();

        // rotation matrix shouldn't be valid for tiny thresholds
        assertFalse(AxisRotation3D.isValidRotationMatrix(
                rotationMatrix, 0.0));

        // for reasonable thresholds, matrix should be valid because it is
        // orthonormal with some imprecision due to machine precision
        assertTrue(AxisRotation3D.isValidRotationMatrix(
                rotationMatrix, ABSOLUTE_ERROR));
        assertTrue(AxisRotation3D.isValidRotationMatrix(
                rotationMatrix));

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double scale = randomizer.nextDouble(MIN_RANDOM_SCALE, MAX_RANDOM_SCALE);

        // change scale of matrix to make it orthogonal instead of orthonormal
        rotationMatrix.multiplyByScalar(scale);

        // now matrix shouldn't be valid even for reasonable thresholds
        assertFalse(AxisRotation3D.isValidRotationMatrix(
                rotationMatrix, ABSOLUTE_ERROR));
        assertFalse(AxisRotation3D.isValidRotationMatrix(
                rotationMatrix));
    }

    @Test
    public void testInverseRotation() throws WrongSizeException,
            NotReadyException, LockedException, DecomposerException,
            NotAvailableException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        // To find 3 orthogonal vectors, we use V matrix of a singular value
        // decomposition of any Nx3 matrix
        final Matrix a = Matrix.createWithUniformRandomValues(1, INHOM_COORDS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final SingularValueDecomposer decomposer = new SingularValueDecomposer(a);

        decomposer.decompose();

        final Matrix v = decomposer.getV();

        // axis of rotation
        final Matrix axisMatrix = v.getSubmatrix(0, 0, 2, 0);

        // inhomogeneous coordinates of point laying on rotation plane
        final double[] axis = axisMatrix.toArray();

        final AxisRotation3D rotation = new AxisRotation3D(axis,
                theta);

        final Matrix rotationMatrix = rotation.asInhomogeneousMatrix();

        // compute inverse rotation
        final AxisRotation3D invRotation =
                rotation.inverseRotationAndReturnNew();
        final AxisRotation3D invRotation2 = new AxisRotation3D();
        rotation.inverseRotation(invRotation2);

        // check correctness
        final Matrix invRotMatrix = invRotation.asInhomogeneousMatrix();
        final Matrix invRotMatrix2 = invRotation2.asInhomogeneousMatrix();

        final Matrix identity = Matrix.identity(INHOM_COORDS, INHOM_COORDS);

        assertTrue(rotationMatrix.multiplyAndReturnNew(invRotMatrix).equals(
                identity, ABSOLUTE_ERROR));
        assertTrue(rotationMatrix.multiplyAndReturnNew(invRotMatrix2).equals(
                identity, ABSOLUTE_ERROR));

        // we can also invert the original rotation
        rotation.inverseRotation();
        assertTrue(invRotMatrix.equals(rotation.asInhomogeneousMatrix(),
                ABSOLUTE_ERROR));
    }

    @Test
    public void testAsInhomogeneousMatrix() throws WrongSizeException,
            NotReadyException, LockedException, DecomposerException,
            NotAvailableException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        // To find 3 orthogonal vectors, we use V matrix of a singular value
        // decomposition of any Nx3 matrix
        final Matrix a = Matrix.createWithUniformRandomValues(1, INHOM_COORDS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final SingularValueDecomposer decomposer = new SingularValueDecomposer(a);

        decomposer.decompose();

        final Matrix v = decomposer.getV();

        // axis of rotation
        final Matrix axisMatrix = v.getSubmatrix(0, 0, 2, 0);

        // inhomogeneous coordinates of point laying on rotation plane
        final double[] axis = axisMatrix.toArray();

        final AxisRotation3D rotation = new AxisRotation3D(
                axis, theta);
        final MatrixRotation3D rotation2 = new MatrixRotation3D(axis, theta);


        final Matrix rotationMatrix = rotation.asInhomogeneousMatrix();
        Matrix rotationMatrix2 = rotation2.getInternalMatrix();

        assertTrue(rotationMatrix.equals(rotationMatrix2, ABSOLUTE_ERROR));

        rotationMatrix2 = new Matrix(INHOM_COORDS, INHOM_COORDS);
        rotation.asInhomogeneousMatrix(rotationMatrix2);
        assertTrue(rotationMatrix.equals(rotationMatrix2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        try {
            rotation.asInhomogeneousMatrix(new Matrix(HOM_COORDS,
                    HOM_COORDS));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testAsHomogeneousMatrix() throws WrongSizeException,
            NotReadyException, LockedException, DecomposerException,
            NotAvailableException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        // To find 3 orthogonal vectors, we use V matrix of a singular value
        // decomposition of any Nx3 matrix
        final Matrix a = Matrix.createWithUniformRandomValues(1, INHOM_COORDS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final SingularValueDecomposer decomposer = new SingularValueDecomposer(a);

        decomposer.decompose();

        final Matrix v = decomposer.getV();

        // axis of rotation
        final Matrix axisMatrix = v.getSubmatrix(0, 0, 2, 0);

        // inhomogeneous coordinates of point laying on rotation plane
        final double[] axis = axisMatrix.toArray();

        final AxisRotation3D rotation = new AxisRotation3D(
                axis, theta);
        final MatrixRotation3D rotation2 = new MatrixRotation3D(axis, theta);

        final Matrix rotationMatrix = rotation.asHomogeneousMatrix();
        final Matrix rotationMatrix2 = rotation2.asHomogeneousMatrix();

        assertTrue(rotationMatrix.equals(rotationMatrix2, ABSOLUTE_ERROR));

        final Matrix homRotationMatrix = Matrix.identity(HOM_COORDS, HOM_COORDS);
        // set top-left 3x3 sub-matrix
        homRotationMatrix.setSubmatrix(0, 0, 2, 2,
                rotation.asInhomogeneousMatrix());

        final Matrix rotationMatrix3 = new Matrix(HOM_COORDS, HOM_COORDS);
        rotation.asHomogeneousMatrix(rotationMatrix3);
        assertTrue(homRotationMatrix.equals(rotationMatrix3, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        try {
            rotation.asHomogeneousMatrix(new Matrix(INHOM_COORDS, INHOM_COORDS));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFromMatrix() throws WrongSizeException,
            InvalidRotationMatrixException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        // To find 3 orthogonal vectors, we use V matrix of a singular value
        // decomposition of any Nx3 matrix
        final Matrix a = Matrix.createWithUniformRandomValues(1, INHOM_COORDS,
                0.0, MAX_RANDOM_VALUE);
        a.setElementAt(0, 2, MAX_RANDOM_VALUE);
        // normalize matrix
        final double norm = Utils.normF(a);
        a.multiplyByScalar(1.0 / norm);
        final double[] axis = a.toArray();

        final double[] coords = new double[INHOM_COORDS];
        final InhomogeneousPoint3D inputPoint = new InhomogeneousPoint3D(coords);

        final MatrixRotation3D rotation2 = new MatrixRotation3D(axis, theta);
        final Matrix rotationMatrix = rotation2.getInternalMatrix();

        final Point3D outputPoint2 = rotation2.rotate(inputPoint);

        final Matrix homRotationMatrix = Matrix.identity(HOM_COORDS, HOM_COORDS);
        // set top-left 3x3 sub-matrix
        homRotationMatrix.setSubmatrix(0, 0, 2, 2, rotationMatrix);

        final AxisRotation3D rotation = new AxisRotation3D();
        // test with inhomogeneous matrix
        rotation.fromMatrix(rotationMatrix, LARGE_ABSOLUTE_ERROR);

        Point3D outputPoint = rotation.rotate(inputPoint);

        assertTrue(outputPoint.equals(outputPoint2, ABSOLUTE_ERROR));

        // test with homogeneous matrix
        rotation.fromMatrix(homRotationMatrix);

        outputPoint = rotation.rotate(inputPoint);

        assertTrue(outputPoint.equals(outputPoint2, ABSOLUTE_ERROR));

        // Attempt to force InvalidRotationMatrixException (using a tiny
        // threshold). This exception might not always be thrown even for
        // the smallest threshold
        try {
            rotation.fromMatrix(rotationMatrix, 0.0);
        } catch (final InvalidRotationMatrixException ignore) {
        }

        // Force IllegalArgumentException
        try {
            rotation.fromMatrix(rotationMatrix, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        // or using a non orthonormal matrix
        homRotationMatrix.setElementAt(3, 3, 0.0); //makes matrix singular
        try {
            rotation.fromMatrix(homRotationMatrix, ABSOLUTE_ERROR);
            fail("InvalidRotationMatrixException expected but not thrown");
        } catch (final InvalidRotationMatrixException ignore) {
        }
    }

    @Test
    public void testFromInhomogeneousMatrix() throws WrongSizeException,
            NotReadyException, LockedException, DecomposerException,
            NotAvailableException, InvalidRotationMatrixException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        // To find 3 orthogonal vectors, we use V matrix of a singular value
        // decomposition of any Nx3 matrix
        final Matrix a = Matrix.createWithUniformRandomValues(1, INHOM_COORDS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final SingularValueDecomposer decomposer = new SingularValueDecomposer(a);

        decomposer.decompose();

        final Matrix v = decomposer.getV();

        // axis of rotation
        final Matrix axisMatrix = v.getSubmatrix(0, 0, 2, 0);

        // inhomogeneous coordinates of point laying on rotation plane
        final double[] axis = axisMatrix.toArray();
        final MatrixRotation3D rotation2 = new MatrixRotation3D(axis, theta);
        final Matrix rotationMatrix = rotation2.getInternalMatrix();

        final AxisRotation3D rotation = new AxisRotation3D();
        // test with inhomogeneous matrix
        rotation.fromInhomogeneousMatrix(rotationMatrix);
        // Check correctness
        assertTrue(rotationMatrix.equals(rotation.asInhomogeneousMatrix(),
                LARGE_ABSOLUTE_ERROR));

        rotation.fromMatrix(rotationMatrix, ABSOLUTE_ERROR);
        // Check correctness
        assertTrue(rotationMatrix.equals(rotation.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

        int numValid = 0;
        for (int t = 0; t < 10 * TIMES; t++) {
            // Force InvalidRotationMatrixException (using a tiny threshold)
            try {
                rotation.fromInhomogeneousMatrix(rotationMatrix, 0.0);
                continue;
            } catch (final InvalidRotationMatrixException ignore) {
            }

            // or using a non orthonormal matrix
            // because matrix is orthonormal, it's enough to scale it to some value
            final double scale = randomizer.nextDouble(MIN_RANDOM_SCALE, MAX_RANDOM_SCALE);
            rotationMatrix.multiplyByScalar(scale);
            try {
                rotation.fromInhomogeneousMatrix(rotationMatrix, ABSOLUTE_ERROR);
                continue;
            } catch (final InvalidRotationMatrixException ignore) {
            }

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testFromHomogeneousMatrix() throws WrongSizeException,
            NotReadyException, LockedException, DecomposerException,
            NotAvailableException, InvalidRotationMatrixException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        // To find 3 orthogonal vectors, we use V matrix of a singular value
        // decomposition of any Nx3 matrix
        final Matrix a = Matrix.createWithUniformRandomValues(1, INHOM_COORDS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final SingularValueDecomposer decomposer = new SingularValueDecomposer(a);

        decomposer.decompose();

        final Matrix v = decomposer.getV();

        // axis of rotation
        final Matrix axisMatrix = v.getSubmatrix(0, 0, 2, 0);

        // inhomogeneous coordinates of point laying on rotation plane
        final double[] axis = axisMatrix.toArray();
        final MatrixRotation3D rotation2 = new MatrixRotation3D(axis, theta);
        final Matrix rotationMatrix = rotation2.getInternalMatrix();

        final Matrix homRotationMatrix = Matrix.identity(HOM_COORDS, HOM_COORDS);
        // set top-left 3x3 sub-matrix
        homRotationMatrix.setSubmatrix(0, 0, 2, 2, rotationMatrix);

        final AxisRotation3D rotation = new AxisRotation3D();
        // test with homogeneous matrix
        rotation.fromHomogeneousMatrix(homRotationMatrix);
        // check correctness
        assertTrue(rotationMatrix.equals(rotation.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
        assertTrue(homRotationMatrix.equals(rotation.asHomogeneousMatrix(), ABSOLUTE_ERROR));

        rotation.fromHomogeneousMatrix(homRotationMatrix, ABSOLUTE_ERROR);
        // check correctness
        assertTrue(rotationMatrix.equals(rotation.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
        assertTrue(homRotationMatrix.equals(rotation.asHomogeneousMatrix(), ABSOLUTE_ERROR));

        // Force InvalidRotationMatrixException (using a tiny threshold)
        try {
            rotation.fromHomogeneousMatrix(rotationMatrix, 0.0);
            fail("InvalidRotationMatrixException expected but not thrown");
        } catch (final InvalidRotationMatrixException ignore) {
        }

        // or using a non orthonormal matrix
        // makes matrix singular
        homRotationMatrix.setElementAt(3, 3, 0.0);
        try {
            rotation.fromHomogeneousMatrix(homRotationMatrix, ABSOLUTE_ERROR);
            fail("InvalidRotationMatrixException expected but not thrown");
        } catch (final InvalidRotationMatrixException ignore) {
        }
    }

    @Test
    public void testRotate() throws WrongSizeException, ColinearPointsException,
            NotReadyException, LockedException, DecomposerException,
            NotAvailableException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        // To find 3 orthogonal vectors, we use V matrix of a singular value
        // decomposition of any Nx3 matrix
        final Matrix a = Matrix.createWithUniformRandomValues(1, INHOM_COORDS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final SingularValueDecomposer decomposer = new SingularValueDecomposer(a);

        decomposer.decompose();

        final Matrix v = decomposer.getV();

        // axis of rotation
        final Matrix axisMatrix = v.getSubmatrix(0, 0, 2, 0);

        // inhomogeneous coordinates of point laying on rotation plane
        final double[] axis = axisMatrix.toArray();

        final AxisRotation3D rotation = new AxisRotation3D(axis,
                theta);

        // create 3 random points
        final HomogeneousPoint3D point1 = new HomogeneousPoint3D();
        point1.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        // to increase accuracy
        point1.normalize();

        final HomogeneousPoint3D point2 = new HomogeneousPoint3D();
        point2.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        // to increase accuracy
        point2.normalize();

        final HomogeneousPoint3D point3 = new HomogeneousPoint3D();
        point3.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        // to increase accuracy
        point3.normalize();

        // ensure that points are not co-linear

        while (Plane.areColinearPoints(point1, point2, point3)) {
            point2.setInhomogeneousCoordinates(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            // to increase accuracy
            point2.normalize();
            point3.setInhomogeneousCoordinates(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            // to increase accuracy
            point3.normalize();
        }

        // create plane passing through all three points
        final Plane plane = new Plane(point1, point2, point3);
        // to increase accuracy
        plane.normalize();
        assertTrue(plane.isLocus(point1, ABSOLUTE_ERROR));
        assertTrue(plane.isLocus(point2, ABSOLUTE_ERROR));
        assertTrue(plane.isLocus(point3, ABSOLUTE_ERROR));

        // now rotate points and plane
        final Point3D rotPoint1A = rotation.rotate(point1);
        final Point3D rotPoint1B = Point3D.create();
        rotation.rotate(point1, rotPoint1B);

        final Point3D rotPoint2A = rotation.rotate(point2);
        final Point3D rotPoint2B = Point3D.create();
        rotation.rotate(point2, rotPoint2B);

        final Point3D rotPoint3A = rotation.rotate(point3);
        final Point3D rotPoint3B = Point3D.create();
        rotation.rotate(point3, rotPoint3B);

        // check that rotated points A and B are equal
        assertTrue(rotPoint1A.equals(rotPoint1B, ABSOLUTE_ERROR));
        assertTrue(rotPoint2A.equals(rotPoint2B, ABSOLUTE_ERROR));
        assertTrue(rotPoint3A.equals(rotPoint3B, ABSOLUTE_ERROR));

        // check points have been correctly rotated
        final Matrix point1Mat = Matrix.newFromArray(point1.asArray(), true);
        final Matrix point2Mat = Matrix.newFromArray(point2.asArray(), true);
        final Matrix r = rotation.asHomogeneousMatrix();
        final Matrix rotPoint1Mat = r.multiplyAndReturnNew(point1Mat);
        final Matrix rotPoint2Mat = r.multiplyAndReturnNew(point2Mat);

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

        // ensure that points where correctly rotated using inhomogeneous
        // coordinates
        final Matrix inhomPoint = new Matrix(INHOM_COORDS, 1);
        inhomPoint.setElementAtIndex(0, point1.getInhomX());
        inhomPoint.setElementAtIndex(1, point1.getInhomY());
        inhomPoint.setElementAtIndex(2, point1.getInhomZ());
        final Matrix inhomR = rotation.asInhomogeneousMatrix();
        final Matrix inhomRotPoint = inhomR.multiplyAndReturnNew(inhomPoint);
        final Point3D rotP = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES,
                inhomRotPoint.toArray());
        assertTrue(rotP.equals(rotPoint1A, ABSOLUTE_ERROR));

        final Plane rotPlaneA = rotation.rotate(plane);
        final Plane rotPlaneB = new Plane();
        rotation.rotate(plane, rotPlaneB);

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
        final Matrix rotPlaneMat = r.multiplyAndReturnNew(planeMat);

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
    public void testCombine() throws WrongSizeException, NotReadyException,
            LockedException, DecomposerException, NotAvailableException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta1 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double theta2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        // To find 3 orthogonal vectors, we use V matrix of a singular value
        // decomposition of any Nx3 matrix
        final Matrix a1 = Matrix.createWithUniformRandomValues(1, INHOM_COORDS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final Matrix a2 = Matrix.createWithUniformRandomValues(1, INHOM_COORDS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final SingularValueDecomposer decomposer = new SingularValueDecomposer(a1);
        decomposer.decompose();
        final Matrix v1 = decomposer.getV();

        decomposer.setInputMatrix(a2);
        decomposer.decompose();
        final Matrix v2 = decomposer.getV();

        // axis of rotation
        final Matrix axisMatrix1 = v1.getSubmatrix(0, 0, 2, 0);
        final Matrix axisMatrix2 = v2.getSubmatrix(0, 0, 2, 0);

        // inhomogeneous coordinates of point laying on rotation plane
        final double[] axis1 = axisMatrix1.toArray();
        final double[] axis2 = axisMatrix2.toArray();

        final AxisRotation3D rot1 = new AxisRotation3D(axis1,
                theta1);
        final AxisRotation3D rot2 = new AxisRotation3D(axis2,
                theta2);

        final AxisRotation3D rot = new AxisRotation3D();
        AxisRotation3D.combine(rot1, rot2, rot);

        assertTrue(rot.asInhomogeneousMatrix().equals(
                rot1.asInhomogeneousMatrix().multiplyAndReturnNew(
                        rot2.asInhomogeneousMatrix()), ABSOLUTE_ERROR));

        final AxisRotation3D rot3 = rot.combineAndReturnNew(rot2);
        assertTrue(rot3.asInhomogeneousMatrix().equals(
                rot.asInhomogeneousMatrix().multiplyAndReturnNew(
                        rot2.asInhomogeneousMatrix()), LARGE_ABSOLUTE_ERROR));

        final Matrix rotationMatrix = rot.asInhomogeneousMatrix();
        rot.combine(rot1);
        assertTrue(rot.asInhomogeneousMatrix().equals(rotationMatrix.
                        multiplyAndReturnNew(rot1.asInhomogeneousMatrix()),
                5.0 * LARGE_ABSOLUTE_ERROR));
    }

    @Test
    public void testType() {
        final AxisRotation3D rotation = new AxisRotation3D();
        assertEquals(Rotation3DType.AXIS_ROTATION3D, rotation.getType());
    }

    @Test
    public void testFromRotation() throws AlgebraException {
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
        final double[] axis = vMatrix.getSubmatrixAsArray(0, 0, 2,
                0);

        final MatrixRotation3D matrixRotation = new MatrixRotation3D();
        final AxisRotation3D axisRotation = new AxisRotation3D();
        final Quaternion quaternion = new Quaternion();

        matrixRotation.setAxisAndRotation(axis, theta);
        axisRotation.setAxisAndRotation(axis, theta);
        quaternion.setAxisAndRotation(axis, theta);

        // test from matrix rotation
        final AxisRotation3D rotation1 = new AxisRotation3D();
        rotation1.fromRotation(matrixRotation);

        // check correctness
        //noinspection AssertBetweenInconvertibleTypes
        assertEquals(matrixRotation, rotation1);

        // test from axis rotation
        final AxisRotation3D rotation2 = new AxisRotation3D();
        rotation2.fromRotation(axisRotation);

        // check correctness
        assertEquals(axisRotation, rotation2);

        // test from quaternion
        final AxisRotation3D rotation3 = new AxisRotation3D();
        rotation3.fromRotation(quaternion);

        // check correctness
        //noinspection AssertBetweenInconvertibleTypes
        assertEquals(quaternion, rotation3);
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
        final double[] axis = vMatrix.getSubmatrixAsArray(0, 0, 2,
                0);

        final AxisRotation3D rotation = new AxisRotation3D(axis, theta);

        // to matrix rotation
        final MatrixRotation3D matrixRotation1 = new MatrixRotation3D();
        rotation.toMatrixRotation(matrixRotation1);
        final MatrixRotation3D matrixRotation2 = rotation.toMatrixRotation();

        // check correctness
        //noinspection AssertBetweenInconvertibleTypes
        assertEquals(rotation, matrixRotation1);
        //noinspection AssertBetweenInconvertibleTypes
        assertEquals(rotation, matrixRotation2);
    }

    @Test
    public void testToAxisRotation() throws AlgebraException {
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
        final double[] axis = vMatrix.getSubmatrixAsArray(0, 0, 2,
                0);

        final AxisRotation3D rotation = new AxisRotation3D(axis, theta);

        // to axis rotation
        final AxisRotation3D axisRotation1 = new AxisRotation3D();
        rotation.toAxisRotation(axisRotation1);
        final AxisRotation3D axisRotation2 = rotation.toAxisRotation();

        // check correctness
        assertEquals(rotation, axisRotation1);
        assertEquals(rotation, axisRotation2);
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
        final Matrix a = Matrix.createWithUniformRandomValues(1, ROTATION_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final SingularValueDecomposer decomposer = new SingularValueDecomposer(a);

        decomposer.decompose();

        final Matrix vMatrix = decomposer.getV();

        // axis of rotation
        final double[] axis = vMatrix.getSubmatrixAsArray(0, 0, 2,
                0);

        final AxisRotation3D rotation = new AxisRotation3D(axis, theta);

        // to quaternion
        final Quaternion quaternion1 = new Quaternion();
        rotation.toQuaternion(quaternion1);
        final Quaternion quaternion2 = rotation.toQuaternion();

        // check correctness
        //noinspection all
        assertEquals(rotation, quaternion1);
        //noinspection all
        assertEquals(rotation, quaternion2);
    }

    @Test
    public void testSerializeDeserialize() throws WrongSizeException,
            LockedException, NotReadyException, DecomposerException,
            NotAvailableException, RotationException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        final Matrix a = Matrix.createWithUniformRandomValues(1, INHOM_COORDS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final SingularValueDecomposer decomposer = new SingularValueDecomposer(a);

        decomposer.decompose();

        final Matrix vMatrix = decomposer.getV();

        // axis of rotation
        final double[] vAxis = vMatrix.getSubmatrixAsArray(0, 0, 2, 0);

        final AxisRotation3D rotation1 = new AxisRotation3D(vAxis, theta);

        assertArrayEquals(vAxis, rotation1.getRotationAxis(), 0.0);
        assertEquals(theta, rotation1.getRotationAngle(), 0.0);
    }
}
