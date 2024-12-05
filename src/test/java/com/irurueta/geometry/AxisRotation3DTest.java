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
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class AxisRotation3DTest {
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
    void testConstants() {
        assertEquals(3, AxisRotation3D.AXIS_PARAMS);
        assertEquals(1e-12, AxisRotation3D.EPS, 0.0);
    }

    @Test
    void testConstructor() throws RotationException, WrongSizeException, NotReadyException, LockedException,
            DecomposerException, NotAvailableException {
        // Test empty constructor
        var rotation = new AxisRotation3D();
        assertNotNull(rotation);

        assertEquals(0.0, rotation.getAxisX(), 0.0);
        assertEquals(0.0, rotation.getAxisY(), 0.0);
        assertEquals(1.0, rotation.getAxisZ(), 0.0);
        assertEquals(0.0, rotation.getRotationAngle(), 0.0);

        // check that empty constructor creates a rotation without effect (its
        // matrix representation is equal to the identity
        assertTrue(rotation.asInhomogeneousMatrix().equals(Matrix.identity(INHOM_COORDS, INHOM_COORDS),
                ABSOLUTE_ERROR));

        // test constructor using axis and angle
        final var randomizer = new UniformRandomizer();
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // Force WrongSizeException
        var wrongAxis = new double[INHOM_COORDS + 1];
        assertThrows(IllegalArgumentException.class, () -> new AxisRotation3D(wrongAxis, theta));

        // Find any 3 orthogonal vectors, 1st will be axis of rotation, and
        // the remaining two will lie on the rotation plane and will be used
        // to test theta angle

        // To find 3 orthogonal vectors, we use V matrix of a singular
        // decomposition of any Nx3 matrix
        final var a = Matrix.createWithUniformRandomValues(1, INHOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var decomposer = new SingularValueDecomposer(a);

        decomposer.decompose();

        final var vMatrix = decomposer.getV();

        // axis of rotation
        final var vAxis = vMatrix.getSubmatrixAsArray(
                0, 0, 2, 0);
        final var mAxis = Matrix.newFromArray(vAxis, true);

        // inhomogeneous coordinates of point laying on rotation plane
        final var vPoint = vMatrix.getSubmatrixAsArray(
                0, 1, 2, 1);

        rotation = new AxisRotation3D(vAxis, theta);

        // To test correctness of rotation, axis should remain equal
        final var rotationMatrix = rotation.asInhomogeneousMatrix();

        final var mAxis2 = rotationMatrix.multiplyAndReturnNew(mAxis);

        assertEquals(mAxis.getElementAtIndex(0), mAxis2.getElementAtIndex(0), ABSOLUTE_ERROR);
        assertEquals(mAxis.getElementAtIndex(1), mAxis2.getElementAtIndex(1), ABSOLUTE_ERROR);
        assertEquals(mAxis.getElementAtIndex(2), mAxis2.getElementAtIndex(2), ABSOLUTE_ERROR);

        final var axis = rotation.getRotationAxis();

        final var scaleX = axis[0] / vAxis[0];
        final var scaleY = axis[0] / vAxis[0];
        final var scaleZ = axis[0] / vAxis[0];

        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleZ, ABSOLUTE_ERROR);
        assertEquals(scaleZ, scaleX, ABSOLUTE_ERROR);

        // and point on rotated plane should be rotated exactly theta
        // radians
        final var mPoint = Matrix.newFromArray(vPoint);
        // below: rotated point
        final var mPoint2 = rotationMatrix.multiplyAndReturnNew(mPoint);

        // because vPoint is already normalized (from SVD decomposition)
        // we only need to normalize rotated point to compute the rotation
        // angle as the arc-cosine of their dot product
        final var norm2 = Utils.normF(mPoint2);
        mPoint2.multiplyByScalar(1.0 / norm2);

        final var dotProduct = mPoint.transposeAndReturnNew().multiplyAndReturnNew(mPoint2).getElementAtIndex(0);

        final var theta2 = Math.acos(dotProduct);
        final var theta2b = rotation.getRotationAngle();

        // check correctness of angles (up to sign for this test)
        assertEquals(Math.abs(theta), Math.abs(theta2), ABSOLUTE_ERROR);

        // check correctness of angles (including sign) for method in class
        if (scaleX > 0.0) {
            assertEquals(theta, theta2b, ABSOLUTE_ERROR);
        } else {
            assertEquals(theta, -theta2b, ABSOLUTE_ERROR);
        }

        // Test copy constructor
        var rotation2 = new AxisRotation3D(rotation);
        assertEquals(rotation2.asInhomogeneousMatrix(), rotation.asInhomogeneousMatrix());

        rotation2 = new AxisRotation3D(rotation.toQuaternion());
        assertTrue(rotation2.equals(rotation, ABSOLUTE_ERROR));
    }

    @Test
    void testGetSetAxisAndRotation() throws WrongSizeException, NotReadyException, LockedException, DecomposerException,
            NotAvailableException, RotationException {

        final var randomizer = new UniformRandomizer();
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var rotation = new AxisRotation3D();

        // Force IllegalArgumentException
        var wrongAxis = new double[INHOM_COORDS + 1];
        assertThrows(IllegalArgumentException.class, () -> rotation.setAxisAndRotation(wrongAxis, theta));

        // Find any 3 orthogonal vectors, 1st will be axis of rotation, and the
        // remaining two will lie on the rotation plane and will be used to test
        // theta angle

        // To find 3 orthogonal vectors, we use V matrix of a singular value
        // decomposition of any Nx3 matrix
        final var a = Matrix.createWithUniformRandomValues(1, INHOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var decomposer = new SingularValueDecomposer(a);

        decomposer.decompose();

        final var v = decomposer.getV();

        // axis of rotation
        final var axisMatrix = v.getSubmatrix(0, 0, 2, 0);

        // inhomogeneous coordinates of point laying on rotation plane
        final var pointMatrix = v.getSubmatrix(0, 1, 2, 1);

        final var axis = axisMatrix.toArray();
        rotation.setAxisAndRotation(axis, theta);

        // To test correctness of rotation, axis should remain equal
        final var rotationMatrix = rotation.asInhomogeneousMatrix();

        final var axisMatrix2 = rotationMatrix.multiplyAndReturnNew(axisMatrix);

        assertTrue(axisMatrix.equals(axisMatrix2, ABSOLUTE_ERROR));

        final var axis2 = rotation.getRotationAxis();

        final var scaleX = axis[0] / axis2[0];
        final var scaleY = axis[0] / axis2[0];
        final var scaleZ = axis[0] / axis2[0];

        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleZ, ABSOLUTE_ERROR);
        assertEquals(scaleZ, scaleX, ABSOLUTE_ERROR);

        // and point on rotated plane should be rotated exactly theta
        // radians
        final var pointMatrix2 = rotationMatrix.multiplyAndReturnNew(pointMatrix);

        // because vPoint is already normalized (from SVD decomposition)
        // we only need to normalize rotated point to compute the rotation
        // angle as the arc-cosine of their dot product
        final var norm2 = Utils.normF(pointMatrix2);
        pointMatrix2.multiplyByScalar(1.0 / norm2);

        final var dotProduct = pointMatrix.transposeAndReturnNew().multiplyAndReturnNew(pointMatrix2)
                .getElementAtIndex(0);

        final var theta2 = Math.acos(dotProduct);
        final var theta2b = rotation.getRotationAngle();

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
    void testSetAxis() throws WrongSizeException, NotReadyException, LockedException, DecomposerException,
            NotAvailableException, RotationException {

        final var randomizer = new UniformRandomizer();
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var rotation = new AxisRotation3D();

        // Find any 3 orthogonal vectors, 1st will be axis of rotation, and the
        // remaining two will lie on the rotation plane and will be used to test
        // theta angle

        // To find 3 orthogonal vectors, we use V matrix of a singular value
        // decomposition of any Nx3 matrix
        final var a = Matrix.createWithUniformRandomValues(1, INHOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var decomposer = new SingularValueDecomposer(a);

        decomposer.decompose();

        final var v = decomposer.getV();

        // axis of rotation
        final var axisMatrix = v.getSubmatrix(0, 0, 2, 0);

        // inhomogeneous coordinates of point laying on rotation plane
        final var pointMatrix = v.getSubmatrix(0, 1, 2, 1);

        var axis = axisMatrix.toArray();
        ArrayUtils.multiplyByScalar(axis, theta, axis);
        rotation.setAxis(axis[0], axis[1], axis[2]);

        // To test correctness of rotation, axis should remain equal
        final var rotationMatrix = rotation.asInhomogeneousMatrix();

        final var axisMatrix2 = rotationMatrix.multiplyAndReturnNew(axisMatrix);

        assertTrue(axisMatrix.equals(axisMatrix2, ABSOLUTE_ERROR));

        final var axis2 = rotation.getRotationAxis();

        final var scaleX = axis[0] / axis2[0];
        final var scaleY = axis[0] / axis2[0];
        final var scaleZ = axis[0] / axis2[0];

        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleZ, ABSOLUTE_ERROR);
        assertEquals(scaleZ, scaleX, ABSOLUTE_ERROR);

        // and point on rotated plane should be rotated exactly theta
        // radians
        final var pointMatrix2 = rotationMatrix.multiplyAndReturnNew(pointMatrix);

        // because vPoint is already normalized (from SVD decomposition)
        // we only need to normalize rotated point to compute the rotation
        // angle as the arc-cosine of their dot product
        final var norm2 = Utils.normF(pointMatrix2);
        pointMatrix2.multiplyByScalar(1.0 / norm2);

        final var dotProduct = pointMatrix.transposeAndReturnNew().multiplyAndReturnNew(
                pointMatrix2).getElementAtIndex(0);

        final var theta2 = Math.acos(dotProduct);
        final var theta2b = rotation.getRotationAngle();

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
    void testIsValidRotationMatrix() throws WrongSizeException, NotReadyException, LockedException, DecomposerException,
            NotAvailableException {

        final var a = Matrix.createWithUniformRandomValues(INHOM_COORDS, INHOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // Use SVD to obtain an orthonormal matrix from V matrix
        final var decomposer = new SingularValueDecomposer(a);

        decomposer.decompose();

        final var rotationMatrix = decomposer.getV();

        // rotation matrix shouldn't be valid for tiny thresholds
        assertFalse(AxisRotation3D.isValidRotationMatrix(rotationMatrix, 0.0));

        // for reasonable thresholds, matrix should be valid because it is
        // orthonormal with some imprecision due to machine precision
        assertTrue(AxisRotation3D.isValidRotationMatrix(rotationMatrix, ABSOLUTE_ERROR));
        assertTrue(AxisRotation3D.isValidRotationMatrix(rotationMatrix));

        final var randomizer = new UniformRandomizer();
        final var scale = randomizer.nextDouble(MIN_RANDOM_SCALE, MAX_RANDOM_SCALE);

        // change scale of matrix to make it orthogonal instead of orthonormal
        rotationMatrix.multiplyByScalar(scale);

        // now matrix shouldn't be valid even for reasonable thresholds
        assertFalse(AxisRotation3D.isValidRotationMatrix(rotationMatrix, ABSOLUTE_ERROR));
        assertFalse(AxisRotation3D.isValidRotationMatrix(rotationMatrix));
    }

    @Test
    void testInverseRotation() throws WrongSizeException, NotReadyException, LockedException, DecomposerException,
            NotAvailableException {

        final var randomizer = new UniformRandomizer();
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // To find 3 orthogonal vectors, we use V matrix of a singular value
        // decomposition of any Nx3 matrix
        final var a = Matrix.createWithUniformRandomValues(1, INHOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var decomposer = new SingularValueDecomposer(a);

        decomposer.decompose();

        final var v = decomposer.getV();

        // axis of rotation
        final var axisMatrix = v.getSubmatrix(0, 0, 2, 0);

        // inhomogeneous coordinates of point laying on rotation plane
        final var axis = axisMatrix.toArray();

        final var rotation = new AxisRotation3D(axis, theta);

        final var rotationMatrix = rotation.asInhomogeneousMatrix();

        // compute inverse rotation
        final var invRotation = rotation.inverseRotationAndReturnNew();
        final var invRotation2 = new AxisRotation3D();
        rotation.inverseRotation(invRotation2);

        // check correctness
        final var invRotMatrix = invRotation.asInhomogeneousMatrix();
        final var invRotMatrix2 = invRotation2.asInhomogeneousMatrix();

        final var identity = Matrix.identity(INHOM_COORDS, INHOM_COORDS);

        assertTrue(rotationMatrix.multiplyAndReturnNew(invRotMatrix).equals(identity, ABSOLUTE_ERROR));
        assertTrue(rotationMatrix.multiplyAndReturnNew(invRotMatrix2).equals(identity, ABSOLUTE_ERROR));

        // we can also invert the original rotation
        rotation.inverseRotation();
        assertTrue(invRotMatrix.equals(rotation.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
    }

    @Test
    void testAsInhomogeneousMatrix() throws WrongSizeException, NotReadyException, LockedException, DecomposerException,
            NotAvailableException {

        final var randomizer = new UniformRandomizer();
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // To find 3 orthogonal vectors, we use V matrix of a singular value
        // decomposition of any Nx3 matrix
        final var a = Matrix.createWithUniformRandomValues(1, INHOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var decomposer = new SingularValueDecomposer(a);

        decomposer.decompose();

        final var v = decomposer.getV();

        // axis of rotation
        final var axisMatrix = v.getSubmatrix(0, 0, 2, 0);

        // inhomogeneous coordinates of point laying on rotation plane
        final var axis = axisMatrix.toArray();

        final var rotation = new AxisRotation3D(axis, theta);
        final var rotation2 = new MatrixRotation3D(axis, theta);

        final var rotationMatrix = rotation.asInhomogeneousMatrix();
        var rotationMatrix2 = rotation2.getInternalMatrix();

        assertTrue(rotationMatrix.equals(rotationMatrix2, ABSOLUTE_ERROR));

        rotationMatrix2 = new Matrix(INHOM_COORDS, INHOM_COORDS);
        rotation.asInhomogeneousMatrix(rotationMatrix2);
        assertTrue(rotationMatrix.equals(rotationMatrix2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        final var m = new Matrix(HOM_COORDS, HOM_COORDS);
        assertThrows(IllegalArgumentException.class, () -> rotation.asInhomogeneousMatrix(m));
    }

    @Test
    void testAsHomogeneousMatrix() throws WrongSizeException, NotReadyException, LockedException, DecomposerException,
            NotAvailableException {

        final var randomizer = new UniformRandomizer();
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // To find 3 orthogonal vectors, we use V matrix of a singular value
        // decomposition of any Nx3 matrix
        final var a = Matrix.createWithUniformRandomValues(1, INHOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var decomposer = new SingularValueDecomposer(a);

        decomposer.decompose();

        final var v = decomposer.getV();

        // axis of rotation
        final var axisMatrix = v.getSubmatrix(0, 0, 2, 0);

        // inhomogeneous coordinates of point laying on rotation plane
        final var axis = axisMatrix.toArray();

        final var rotation = new AxisRotation3D(axis, theta);
        final var rotation2 = new MatrixRotation3D(axis, theta);

        final var rotationMatrix = rotation.asHomogeneousMatrix();
        final var rotationMatrix2 = rotation2.asHomogeneousMatrix();

        assertTrue(rotationMatrix.equals(rotationMatrix2, ABSOLUTE_ERROR));

        final var homRotationMatrix = Matrix.identity(HOM_COORDS, HOM_COORDS);
        // set top-left 3x3 sub-matrix
        homRotationMatrix.setSubmatrix(0, 0, 2, 2,
                rotation.asInhomogeneousMatrix());

        final var rotationMatrix3 = new Matrix(HOM_COORDS, HOM_COORDS);
        rotation.asHomogeneousMatrix(rotationMatrix3);
        assertTrue(homRotationMatrix.equals(rotationMatrix3, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        final var m = new Matrix(INHOM_COORDS, INHOM_COORDS);
        assertThrows(IllegalArgumentException.class, () -> rotation.asHomogeneousMatrix(m));
    }

    @Test
    void testFromMatrix() throws WrongSizeException, InvalidRotationMatrixException {

        final var randomizer = new UniformRandomizer();
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // To find 3 orthogonal vectors, we use V matrix of a singular value
        // decomposition of any Nx3 matrix
        final var a = Matrix.createWithUniformRandomValues(1, INHOM_COORDS, 0.0, MAX_RANDOM_VALUE);
        a.setElementAt(0, 2, MAX_RANDOM_VALUE);
        // normalize matrix
        final var norm = Utils.normF(a);
        a.multiplyByScalar(1.0 / norm);
        final var axis = a.toArray();

        final var coords = new double[INHOM_COORDS];
        final var inputPoint = new InhomogeneousPoint3D(coords);

        final var rotation2 = new MatrixRotation3D(axis, theta);
        final var rotationMatrix = rotation2.getInternalMatrix();

        final var outputPoint2 = rotation2.rotate(inputPoint);

        final var homRotationMatrix = Matrix.identity(HOM_COORDS, HOM_COORDS);
        // set top-left 3x3 sub-matrix
        homRotationMatrix.setSubmatrix(0, 0, 2, 2, rotationMatrix);

        final var rotation = new AxisRotation3D();
        // test with inhomogeneous matrix
        rotation.fromMatrix(rotationMatrix, LARGE_ABSOLUTE_ERROR);

        var outputPoint = rotation.rotate(inputPoint);

        assertTrue(outputPoint.equals(outputPoint2, ABSOLUTE_ERROR));

        // test with homogeneous matrix
        rotation.fromMatrix(homRotationMatrix);

        outputPoint = rotation.rotate(inputPoint);

        assertTrue(outputPoint.equals(outputPoint2, ABSOLUTE_ERROR));

        // Attempt to force InvalidRotationMatrixException (using a tiny
        // threshold). This exception might not always be thrown even for
        // the smallest threshold
        assertThrows(InvalidRotationMatrixException.class, () -> rotation.fromMatrix(rotationMatrix, 0.0));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> rotation.fromMatrix(rotationMatrix, -ABSOLUTE_ERROR));

        // or using a non orthonormal matrix
        homRotationMatrix.setElementAt(3, 3, 0.0); //makes matrix singular
        assertThrows(InvalidRotationMatrixException.class,
                () -> rotation.fromMatrix(homRotationMatrix, ABSOLUTE_ERROR));
    }

    @Test
    void testFromInhomogeneousMatrix() throws WrongSizeException, NotReadyException, LockedException,
            DecomposerException, NotAvailableException, InvalidRotationMatrixException {

        final var randomizer = new UniformRandomizer();
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // To find 3 orthogonal vectors, we use V matrix of a singular value
        // decomposition of any Nx3 matrix
        final var a = Matrix.createWithUniformRandomValues(1, INHOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var decomposer = new SingularValueDecomposer(a);

        decomposer.decompose();

        final var v = decomposer.getV();

        // axis of rotation
        final var axisMatrix = v.getSubmatrix(0, 0, 2, 0);

        // inhomogeneous coordinates of point laying on rotation plane
        final var axis = axisMatrix.toArray();
        final var rotation2 = new MatrixRotation3D(axis, theta);
        final var rotationMatrix = rotation2.getInternalMatrix();

        final var rotation = new AxisRotation3D();
        // test with inhomogeneous matrix
        rotation.fromInhomogeneousMatrix(rotationMatrix);
        // Check correctness
        assertTrue(rotationMatrix.equals(rotation.asInhomogeneousMatrix(), LARGE_ABSOLUTE_ERROR));

        rotation.fromMatrix(rotationMatrix, ABSOLUTE_ERROR);
        // Check correctness
        assertTrue(rotationMatrix.equals(rotation.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

        var numValid = 0;
        for (var t = 0; t < 10 * TIMES; t++) {
            // Force InvalidRotationMatrixException (using a tiny threshold)
            assertThrows(InvalidRotationMatrixException.class,
                    () -> rotation.fromInhomogeneousMatrix(rotationMatrix, 0.0));

            // or using a non orthonormal matrix
            // because matrix is orthonormal, it's enough to scale it to some value
            final var scale = randomizer.nextDouble(MIN_RANDOM_SCALE, MAX_RANDOM_SCALE);
            rotationMatrix.multiplyByScalar(scale);
            try {
                rotation.fromInhomogeneousMatrix(rotationMatrix, ABSOLUTE_ERROR);
                continue;
            } catch (final InvalidRotationMatrixException ignore) {
                // if provided rotation matrix is not orthonormal, an exception occurs
            }

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testFromHomogeneousMatrix() throws WrongSizeException, NotReadyException, LockedException, DecomposerException,
            NotAvailableException, InvalidRotationMatrixException {

        final var randomizer = new UniformRandomizer();
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // To find 3 orthogonal vectors, we use V matrix of a singular value
        // decomposition of any Nx3 matrix
        final var a = Matrix.createWithUniformRandomValues(1, INHOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var decomposer = new SingularValueDecomposer(a);

        decomposer.decompose();

        final var v = decomposer.getV();

        // axis of rotation
        final var axisMatrix = v.getSubmatrix(0, 0, 2, 0);

        // inhomogeneous coordinates of point laying on rotation plane
        final var axis = axisMatrix.toArray();
        final var rotation2 = new MatrixRotation3D(axis, theta);
        final var rotationMatrix = rotation2.getInternalMatrix();

        final var homRotationMatrix = Matrix.identity(HOM_COORDS, HOM_COORDS);
        // set top-left 3x3 sub-matrix
        homRotationMatrix.setSubmatrix(0, 0, 2, 2, rotationMatrix);

        final var rotation = new AxisRotation3D();
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
        assertThrows(InvalidRotationMatrixException.class,
                () -> rotation.fromHomogeneousMatrix(rotationMatrix, 0.0));

        // or using a non orthonormal matrix
        // makes matrix singular
        homRotationMatrix.setElementAt(3, 3, 0.0);
        assertThrows(InvalidRotationMatrixException.class,
                () -> rotation.fromHomogeneousMatrix(homRotationMatrix, ABSOLUTE_ERROR));
    }

    @Test
    void testRotate() throws WrongSizeException, ColinearPointsException, NotReadyException, LockedException,
            DecomposerException, NotAvailableException {

        final var randomizer = new UniformRandomizer();
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // To find 3 orthogonal vectors, we use V matrix of a singular value
        // decomposition of any Nx3 matrix
        final var a = Matrix.createWithUniformRandomValues(1, INHOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var decomposer = new SingularValueDecomposer(a);

        decomposer.decompose();

        final var v = decomposer.getV();

        // axis of rotation
        final var axisMatrix = v.getSubmatrix(0, 0, 2, 0);

        // inhomogeneous coordinates of point laying on rotation plane
        final var axis = axisMatrix.toArray();

        final var rotation = new AxisRotation3D(axis, theta);

        // create 3 random points
        final var point1 = new HomogeneousPoint3D();
        point1.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        // to increase accuracy
        point1.normalize();

        final var point2 = new HomogeneousPoint3D();
        point2.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        // to increase accuracy
        point2.normalize();

        final var point3 = new HomogeneousPoint3D();
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
        final var plane = new Plane(point1, point2, point3);
        // to increase accuracy
        plane.normalize();
        assertTrue(plane.isLocus(point1, ABSOLUTE_ERROR));
        assertTrue(plane.isLocus(point2, ABSOLUTE_ERROR));
        assertTrue(plane.isLocus(point3, ABSOLUTE_ERROR));

        // now rotate points and plane
        final var rotPoint1A = rotation.rotate(point1);
        final var rotPoint1B = Point3D.create();
        rotation.rotate(point1, rotPoint1B);

        final var rotPoint2A = rotation.rotate(point2);
        final var rotPoint2B = Point3D.create();
        rotation.rotate(point2, rotPoint2B);

        final var rotPoint3A = rotation.rotate(point3);
        final var rotPoint3B = Point3D.create();
        rotation.rotate(point3, rotPoint3B);

        // check that rotated points A and B are equal
        assertTrue(rotPoint1A.equals(rotPoint1B, ABSOLUTE_ERROR));
        assertTrue(rotPoint2A.equals(rotPoint2B, ABSOLUTE_ERROR));
        assertTrue(rotPoint3A.equals(rotPoint3B, ABSOLUTE_ERROR));

        // check points have been correctly rotated
        final var point1Mat = Matrix.newFromArray(point1.asArray(), true);
        final var point2Mat = Matrix.newFromArray(point2.asArray(), true);
        final var r = rotation.asHomogeneousMatrix();
        final var rotPoint1Mat = r.multiplyAndReturnNew(point1Mat);
        final var rotPoint2Mat = r.multiplyAndReturnNew(point2Mat);

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

        // ensure that points where correctly rotated using inhomogeneous
        // coordinates
        final var inhomPoint = new Matrix(INHOM_COORDS, 1);
        inhomPoint.setElementAtIndex(0, point1.getInhomX());
        inhomPoint.setElementAtIndex(1, point1.getInhomY());
        inhomPoint.setElementAtIndex(2, point1.getInhomZ());
        final var inhomR = rotation.asInhomogeneousMatrix();
        final var inhomRotPoint = inhomR.multiplyAndReturnNew(inhomPoint);
        final var rotP = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, inhomRotPoint.toArray());
        assertTrue(rotP.equals(rotPoint1A, ABSOLUTE_ERROR));

        final var rotPlaneA = rotation.rotate(plane);
        final var rotPlaneB = new Plane();
        rotation.rotate(plane, rotPlaneB);

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
    void testCombine() throws WrongSizeException, NotReadyException, LockedException, DecomposerException,
            NotAvailableException {

        final var randomizer = new UniformRandomizer();
        final var theta1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var theta2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // To find 3 orthogonal vectors, we use V matrix of a singular value
        // decomposition of any Nx3 matrix
        final var a1 = Matrix.createWithUniformRandomValues(1, INHOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var a2 = Matrix.createWithUniformRandomValues(1, INHOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var decomposer = new SingularValueDecomposer(a1);
        decomposer.decompose();
        final var v1 = decomposer.getV();

        decomposer.setInputMatrix(a2);
        decomposer.decompose();
        final var v2 = decomposer.getV();

        // axis of rotation
        final var axisMatrix1 = v1.getSubmatrix(0, 0, 2, 0);
        final var axisMatrix2 = v2.getSubmatrix(0, 0, 2, 0);

        // inhomogeneous coordinates of point laying on rotation plane
        final var axis1 = axisMatrix1.toArray();
        final var axis2 = axisMatrix2.toArray();

        final var rot1 = new AxisRotation3D(axis1, theta1);
        final var rot2 = new AxisRotation3D(axis2, theta2);

        final var rot = new AxisRotation3D();
        AxisRotation3D.combine(rot1, rot2, rot);

        assertTrue(rot.asInhomogeneousMatrix().equals(rot1.asInhomogeneousMatrix().multiplyAndReturnNew(
                rot2.asInhomogeneousMatrix()), ABSOLUTE_ERROR));

        final var rot3 = rot.combineAndReturnNew(rot2);
        assertTrue(rot3.asInhomogeneousMatrix().equals(rot.asInhomogeneousMatrix().multiplyAndReturnNew(
                rot2.asInhomogeneousMatrix()), LARGE_ABSOLUTE_ERROR));

        final var rotationMatrix = rot.asInhomogeneousMatrix();
        rot.combine(rot1);
        assertTrue(rot.asInhomogeneousMatrix().equals(rotationMatrix.multiplyAndReturnNew(rot1.asInhomogeneousMatrix()),
                5.0 * LARGE_ABSOLUTE_ERROR));
    }

    @Test
    void testType() {
        final var rotation = new AxisRotation3D();
        assertEquals(Rotation3DType.AXIS_ROTATION3D, rotation.getType());
    }

    @Test
    void testFromRotation() throws AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // Find any 3 orthogonal vectors, 1st will be axis of rotation, and
        // the remaining two will lie on the rotation plane and will be used
        // to test theta angle

        // To find 3 orthogonal vectors, we use V matrix of a singular
        // decomposition of any Nx3 matrix
        final var a = Matrix.createWithUniformRandomValues(1, ROTATION_COLS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var decomposer = new SingularValueDecomposer(a);

        decomposer.decompose();

        final var vMatrix = decomposer.getV();

        // axis of rotation
        final var axis = vMatrix.getSubmatrixAsArray(0, 0, 2, 0);

        final var matrixRotation = new MatrixRotation3D();
        final var axisRotation = new AxisRotation3D();
        final var quaternion = new Quaternion();

        matrixRotation.setAxisAndRotation(axis, theta);
        axisRotation.setAxisAndRotation(axis, theta);
        quaternion.setAxisAndRotation(axis, theta);

        // test from matrix rotation
        final var rotation1 = new AxisRotation3D();
        rotation1.fromRotation(matrixRotation);

        // check correctness
        //noinspection AssertBetweenInconvertibleTypes
        assertEquals(matrixRotation, rotation1);

        // test from axis rotation
        final var rotation2 = new AxisRotation3D();
        rotation2.fromRotation(axisRotation);

        // check correctness
        assertEquals(axisRotation, rotation2);

        // test from quaternion
        final var rotation3 = new AxisRotation3D();
        rotation3.fromRotation(quaternion);

        // check correctness
        //noinspection AssertBetweenInconvertibleTypes
        assertEquals(quaternion, rotation3);
    }

    @Test
    void testToMatrixRotation() throws AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // Find any 3 orthogonal vectors, 1st will be axis of rotation, and
        // the remaining two will lie on the rotation plane and will be used
        // to test theta angle

        // To find 3 orthogonal vectors, we use V matrix of a singular
        // decomposition of any Nx3 matrix
        final var a = Matrix.createWithUniformRandomValues(1, ROTATION_COLS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var decomposer = new SingularValueDecomposer(a);

        decomposer.decompose();

        final var vMatrix = decomposer.getV();

        // axis of rotation
        final var axis = vMatrix.getSubmatrixAsArray(0, 0, 2, 0);

        final var rotation = new AxisRotation3D(axis, theta);

        // to matrix rotation
        final var matrixRotation1 = new MatrixRotation3D();
        rotation.toMatrixRotation(matrixRotation1);
        final var matrixRotation2 = rotation.toMatrixRotation();

        // check correctness
        //noinspection AssertBetweenInconvertibleTypes
        assertEquals(rotation, matrixRotation1);
        //noinspection AssertBetweenInconvertibleTypes
        assertEquals(rotation, matrixRotation2);
    }

    @Test
    void testToAxisRotation() throws AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // Find any 3 orthogonal vectors, 1st will be axis of rotation, and
        // the remaining two will lie on the rotation plane and will be used
        // to test theta angle

        // To find 3 orthogonal vectors, we use V matrix of a singular
        // decomposition of any Nx3 matrix
        final var a = Matrix.createWithUniformRandomValues(1, ROTATION_COLS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var decomposer = new SingularValueDecomposer(a);

        decomposer.decompose();

        final var vMatrix = decomposer.getV();

        // axis of rotation
        final var axis = vMatrix.getSubmatrixAsArray(0, 0, 2, 0);

        final var rotation = new AxisRotation3D(axis, theta);

        // to axis rotation
        final var axisRotation1 = new AxisRotation3D();
        rotation.toAxisRotation(axisRotation1);
        final var axisRotation2 = rotation.toAxisRotation();

        // check correctness
        assertEquals(rotation, axisRotation1);
        assertEquals(rotation, axisRotation2);
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
        final var a = Matrix.createWithUniformRandomValues(1, ROTATION_COLS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var decomposer = new SingularValueDecomposer(a);

        decomposer.decompose();

        final var vMatrix = decomposer.getV();

        // axis of rotation
        final var axis = vMatrix.getSubmatrixAsArray(0, 0, 2, 0);

        final var rotation = new AxisRotation3D(axis, theta);

        // to quaternion
        final var quaternion1 = new Quaternion();
        rotation.toQuaternion(quaternion1);
        final var quaternion2 = rotation.toQuaternion();

        // check correctness
        //noinspection all
        assertEquals(rotation, quaternion1);
        //noinspection all
        assertEquals(rotation, quaternion2);
    }

    @Test
    void testSerializeDeserialize() throws WrongSizeException, LockedException, NotReadyException, DecomposerException,
            NotAvailableException, RotationException {
        final var randomizer = new UniformRandomizer();
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var a = Matrix.createWithUniformRandomValues(1, INHOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var decomposer = new SingularValueDecomposer(a);

        decomposer.decompose();

        final var vMatrix = decomposer.getV();

        // axis of rotation
        final var vAxis = vMatrix.getSubmatrixAsArray(
                0, 0, 2, 0);

        final var rotation1 = new AxisRotation3D(vAxis, theta);

        assertArrayEquals(vAxis, rotation1.getRotationAxis(), 0.0);
        assertEquals(theta, rotation1.getRotationAngle(), 0.0);
    }
}
