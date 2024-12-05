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
import com.irurueta.geometry.estimators.DLTLinePlaneCorrespondencePinholeCameraEstimator;
import com.irurueta.geometry.estimators.DLTPointCorrespondencePinholeCameraEstimator;
import com.irurueta.geometry.estimators.PinholeCameraEstimatorException;
import com.irurueta.geometry.estimators.WrongListSizesException;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class PinholeCameraTest {

    private static final int PINHOLE_CAMERA_ROWS = 3;
    private static final int PINHOLE_CAMERA_COLS = 4;

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double LARGE_ABSOLUTE_ERROR = 5e-5;
    private static final double VERY_LARGE_ABSOLUTE_ERROR = 1e-4;

    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    private static final double MIN_FOCAL_LENGTH = 1.0;
    private static final double MAX_FOCAL_LENGTH = 100.0;

    private static final double MIN_SKEWNESS = -1.0;
    private static final double MAX_SKEWNESS = 1.0;

    private static final double MIN_PRINCIPAL_POINT = 0.0;
    private static final double MAX_PRINCIPAL_POINT = 100.0;

    private static final double MIN_ANGLE_DEGREES = -90.0;
    private static final double MAX_ANGLE_DEGREES = 90.0;

    private static final int INHOM_2D_COORDS = 2;
    private static final int INHOM_3D_COORDS = 3;

    private static final int HOM_2D_COORDS = 3;
    private static final int HOM_3D_COORDS = 4;

    private static final int MIN_NUMBER_POINTS = 10;
    private static final int MAX_NUMBER_POINTS = 100;

    private static final double MIN_DEPTH = 0.5;
    private static final double MAX_DEPTH = 100.0;

    private static final int MIN_N_POINTS = 10;
    private static final int MAX_N_POINTS = 100;

    private static final double MIN_RANDOM_POINT_VALUE = 50.0;
    private static final double MAX_RANDOM_POINT_VALUE = 100.0;

    private static final double MIN_RANDOM_LINE_VALUE = 0.0;
    private static final double MAX_RANDOM_LINE_VALUE = 1.0;

    private static final double MIN_FOCAL_LENGTH2 = 110.0;
    private static final double MAX_FOCAL_LENGTH2 = 130.0;

    private static final double MIN_SKEWNESS2 = -0.001;
    private static final double MAX_SKEWNESS2 = 0.001;

    private static final double MIN_ANGLE_DEGREES2 = 10.0;
    private static final double MAX_ANGLE_DEGREES2 = 15.0;

    private static final int N_POINTS = 6;
    private static final int N_CORRESPONDENCES = 4;

    private static final int TIMES = 10;

    @Test
    void testConstants() {
        assertEquals(3, PinholeCamera.PINHOLE_CAMERA_MATRIX_ROWS);
        assertEquals(4, PinholeCamera.PINHOLE_CAMERA_MATRIX_COLS);
        assertEquals(3, PinholeCamera.INHOM_COORDS);
        assertEquals(1e-12, PinholeCamera.EPS, 0.0);
    }

    @Test
    void testConstructors() throws WrongSizeException, RotationException, CameraException, NotAvailableException,
            WrongListSizesException, com.irurueta.geometry.estimators.LockedException,
            com.irurueta.geometry.estimators.NotReadyException, PinholeCameraEstimatorException {

        // test default constructor
        var camera = new PinholeCamera();

        // test type
        assertEquals(CameraType.PINHOLE_CAMERA, camera.getType());

        // test that internal matrix is the 3x4 identity, which corresponds to
        // canonical camera located at origin of coordinates, pointing towards
        // z-axis and with retinal plane located at Z = 1 (unitary focal length)
        assertEquals(Matrix.identity(PINHOLE_CAMERA_ROWS, PINHOLE_CAMERA_COLS), camera.getInternalMatrix());

        // test constructor by providing internal matrix
        var cameraMatrix = Matrix.createWithUniformRandomValues(PINHOLE_CAMERA_ROWS, PINHOLE_CAMERA_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        camera = new PinholeCamera(cameraMatrix);
        assertEquals(cameraMatrix, camera.getInternalMatrix());

        // Force WrongSizeException
        cameraMatrix = Matrix.createWithUniformRandomValues(PINHOLE_CAMERA_ROWS + 1,
                PINHOLE_CAMERA_COLS + 1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var finalCameraMatrix = cameraMatrix;
        assertThrows(WrongSizeException.class, () -> new PinholeCamera(finalCameraMatrix));

        // create intrinsic parameters
        final var randomizer = new UniformRandomizer();
        var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
        var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        // rotation
        var alphaEuler = randomizer.nextDouble(
                MIN_ANGLE_DEGREES * Math.PI / 180.0, MAX_ANGLE_DEGREES * Math.PI / 180.0);
        var betaEuler = randomizer.nextDouble(
                MIN_ANGLE_DEGREES * Math.PI / 180.0, MAX_ANGLE_DEGREES * Math.PI / 180.0);
        var gammaEuler = randomizer.nextDouble(
                MIN_ANGLE_DEGREES * Math.PI / 180.0, MAX_ANGLE_DEGREES * Math.PI / 180.0);

        var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                horizontalPrincipalPoint, verticalPrincipalPoint, skewness);
        final var intrinsicMatrix = intrinsic.getInternalMatrix();

        var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);
        final var rotationMatrix = rotation.getInternalMatrix();

        final var axis = rotation.getRotationAxis();
        final var theta = rotation.getRotationAngle();

        // image of world origin
        final var originImageArray = new double[INHOM_2D_COORDS];
        randomizer.fill(originImageArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var originImage = new InhomogeneousPoint2D(originImageArray);

        // camera center
        var cameraCenterArray = new double[INHOM_3D_COORDS];
        randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

        // test constructor with intrinsic parameters, rotation and image or
        // origin
        camera = new PinholeCamera(intrinsic, rotation, originImage);
        assertFalse(camera.isCameraCenterAvailable());
        assertTrue(camera.isCameraRotationAvailable());
        assertTrue(camera.areIntrinsicParametersAvailable());

        // test type
        assertEquals(CameraType.PINHOLE_CAMERA, camera.getType());

        // build internal matrix
        var cameraMatrix2 = new Matrix(PINHOLE_CAMERA_ROWS, PINHOLE_CAMERA_COLS);
        var mp = intrinsicMatrix.multiplyAndReturnNew(rotationMatrix);
        for (var v = 0; v < PINHOLE_CAMERA_ROWS; v++) {
            for (var u = 0; u < PINHOLE_CAMERA_ROWS; u++) {
                cameraMatrix2.setElementAt(u, v, mp.getElementAt(u, v));
            }
        }

        // set last column of camera matrix with homogeneous coordinates of image
        // or world origin
        cameraMatrix2.setElementAt(0, 3, originImage.getHomX());
        cameraMatrix2.setElementAt(1, 3, originImage.getHomY());
        cameraMatrix2.setElementAt(2, 3, originImage.getHomW());

        cameraMatrix = camera.getInternalMatrix();

        // compare matrices cameraMatrix and cameraMatrix2
        assertTrue(cameraMatrix.equals(cameraMatrix2, ABSOLUTE_ERROR));

        // decompose created camera and check that obtained intrinsic parameters,
        // rotation and image of world origin are correct
        camera.decompose(true, true);

        assertTrue(camera.isCameraCenterAvailable());
        assertTrue(camera.isCameraRotationAvailable());
        assertTrue(camera.areIntrinsicParametersAvailable());

        var rotation2 = camera.getCameraRotation();
        var intrinsic2 = camera.getIntrinsicParameters();
        final var originImage2 = camera.getImageOfWorldOrigin();

        // Obtain rotation axis and angle from 2nd rotation
        var axis2 = rotation2.getRotationAxis();
        var theta2 = rotation2.getRotationAngle();

        // check correctness of axis and angle

        // axis can be equal up to sign
        var scaleAxisX = axis[0] / axis2[0];
        var scaleAxisY = axis[1] / axis2[1];
        var scaleAxisZ = axis[2] / axis2[2];

        assertEquals(scaleAxisX, scaleAxisY, ABSOLUTE_ERROR);
        assertEquals(scaleAxisY, scaleAxisZ, ABSOLUTE_ERROR);
        assertEquals(scaleAxisZ, scaleAxisX, ABSOLUTE_ERROR);

        // if sign of rotation axis is the opposite (-1.0), then theta will be
        // -theta (opposite sign)
        if (scaleAxisX > 0.0) {
            assertEquals(theta2, theta, ABSOLUTE_ERROR);
        } else {
            assertEquals(-theta2, theta, ABSOLUTE_ERROR);
        }

        // compare intrinsic parameters
        assertEquals(horizontalFocalLength, intrinsic2.getHorizontalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(verticalFocalLength, intrinsic2.getVerticalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(skewness, intrinsic2.getSkewness(), ABSOLUTE_ERROR);
        assertEquals(horizontalPrincipalPoint, intrinsic2.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
        assertEquals(verticalPrincipalPoint, intrinsic2.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

        // compare images of origin
        assertTrue(originImage.equals(originImage2, ABSOLUTE_ERROR));

        // test constructor with intrinsic parameters, rotation and camera center
        camera = new PinholeCamera(intrinsic, rotation, cameraCenter);
        assertTrue(camera.isCameraCenterAvailable());
        assertTrue(camera.isCameraRotationAvailable());
        assertTrue(camera.areIntrinsicParametersAvailable());

        // test type
        assertEquals(CameraType.PINHOLE_CAMERA, camera.getType());

        // build internal matrix
        cameraMatrix2 = new Matrix(PINHOLE_CAMERA_ROWS, PINHOLE_CAMERA_COLS);
        mp = intrinsicMatrix.multiplyAndReturnNew(rotationMatrix);
        for (var v = 0; v < PINHOLE_CAMERA_ROWS; v++) {
            for (var u = 0; u < PINHOLE_CAMERA_ROWS; u++) {
                cameraMatrix2.setElementAt(u, v, mp.getElementAt(u, v));
            }
        }

        final var inhomCenter = new Matrix(PINHOLE_CAMERA_ROWS, 1);
        inhomCenter.setElementAtIndex(0, cameraCenter.getInhomX());
        inhomCenter.setElementAtIndex(1, cameraCenter.getInhomY());
        inhomCenter.setElementAtIndex(2, cameraCenter.getInhomZ());
        final var p4 = mp.multiplyAndReturnNew(inhomCenter).multiplyByScalarAndReturnNew(-1.0);

        // set last column of camera matrix with homogeneous coordinates of image
        // of world origin
        cameraMatrix2.setElementAt(0, 3, p4.getElementAtIndex(0));
        cameraMatrix2.setElementAt(1, 3, p4.getElementAtIndex(1));
        cameraMatrix2.setElementAt(2, 3, p4.getElementAtIndex(2));

        cameraMatrix = camera.getInternalMatrix();

        // test that camera center is the null-space of camera matrix
        final var homCenter = new Matrix(PINHOLE_CAMERA_COLS, 1);
        var homCameraCenter = new HomogeneousPoint3D(cameraCenter);
        homCameraCenter.normalize();
        homCenter.setElementAtIndex(0, homCameraCenter.getHomX());
        homCenter.setElementAtIndex(1, homCameraCenter.getHomY());
        homCenter.setElementAtIndex(2, homCameraCenter.getHomZ());
        homCenter.setElementAtIndex(3, homCameraCenter.getHomW());
        var nullMatrix = cameraMatrix.multiplyAndReturnNew(homCenter);
        assertEquals(0.0, nullMatrix.getElementAtIndex(0), ABSOLUTE_ERROR);
        assertEquals(0.0, nullMatrix.getElementAtIndex(1), ABSOLUTE_ERROR);
        assertEquals(0.0, nullMatrix.getElementAtIndex(2), ABSOLUTE_ERROR);

        // compare matrix cameraMatrix and cameraMatrix2
        assertTrue(cameraMatrix.equals(cameraMatrix2, ABSOLUTE_ERROR));

        // decompose created camera and check that obtained intrinsic parameters,
        // rotation and image of world origin are correct
        camera.decompose(true, true);

        assertTrue(camera.isCameraCenterAvailable());
        assertTrue(camera.isCameraRotationAvailable());
        assertTrue(camera.areIntrinsicParametersAvailable());

        rotation2 = camera.getCameraRotation();
        intrinsic2 = camera.getIntrinsicParameters();
        var cameraCenter2 = camera.getCameraCenter();

        homCameraCenter = new HomogeneousPoint3D(cameraCenter2);
        homCameraCenter.normalize();
        homCenter.setElementAtIndex(0, homCameraCenter.getHomX());
        homCenter.setElementAtIndex(1, homCameraCenter.getHomY());
        homCenter.setElementAtIndex(2, homCameraCenter.getHomZ());
        homCenter.setElementAtIndex(3, homCameraCenter.getHomW());
        nullMatrix = cameraMatrix.multiplyAndReturnNew(homCenter);
        assertEquals(0.0, nullMatrix.getElementAtIndex(0), ABSOLUTE_ERROR);
        assertEquals(0.0, nullMatrix.getElementAtIndex(1), ABSOLUTE_ERROR);
        assertEquals(0.0, nullMatrix.getElementAtIndex(2), ABSOLUTE_ERROR);

        // Obtain rotation axis and angle from 2nd rotation
        axis2 = rotation2.getRotationAxis();
        theta2 = rotation2.getRotationAngle();

        // check correctness of axis and angle

        // axis can be equal up to sign
        scaleAxisX = axis[0] / axis2[0];
        scaleAxisY = axis[1] / axis2[1];
        scaleAxisZ = axis[2] / axis2[2];

        assertEquals(scaleAxisX, scaleAxisY, ABSOLUTE_ERROR);
        assertEquals(scaleAxisY, scaleAxisZ, ABSOLUTE_ERROR);
        assertEquals(scaleAxisZ, scaleAxisX, ABSOLUTE_ERROR);

        // if sign of rotation axis is the opposite (-1.0), then theta will be
        // -theta (opposite sign)
        if (scaleAxisX > 0.0) {
            assertEquals(theta, theta2, ABSOLUTE_ERROR);
        } else {
            assertEquals(theta, -theta2, ABSOLUTE_ERROR);
        }

        // compare intrinsic parameters
        assertEquals(horizontalFocalLength, intrinsic2.getHorizontalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(verticalFocalLength, intrinsic2.getVerticalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(skewness, intrinsic2.getSkewness(), ABSOLUTE_ERROR);
        assertEquals(horizontalPrincipalPoint, intrinsic2.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
        assertEquals(verticalPrincipalPoint, intrinsic2.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

        // compare camera centers
        assertTrue(cameraCenter.equals(cameraCenter2, LARGE_ABSOLUTE_ERROR));

        // test constructor with point correspondences

        // create intrinsic parameters
        horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH2, MAX_FOCAL_LENGTH2);
        verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH2, MAX_FOCAL_LENGTH2);
        skewness = randomizer.nextDouble(MIN_SKEWNESS2, MAX_SKEWNESS2);
        horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

        // create rotation parameters
        alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES2, MAX_ANGLE_DEGREES2));
        betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES2, MAX_ANGLE_DEGREES2));
        gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES2, MAX_ANGLE_DEGREES2));

        rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

        // create camera center
        cameraCenterArray = new double[INHOM_3D_COORDS];
        randomizer.fill(cameraCenterArray, MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE);
        cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

        // instantiate camera
        camera = new PinholeCamera(intrinsic, rotation, cameraCenter);

        // normalize the camera to improve accuracy
        camera.normalize();

        // create 6 point correspondences
        final var point3D1 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE));
        Point3D point3D2 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE));
        final Point3D point3D3 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE));
        final Point3D point3D4 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE));
        final Point3D point3D5 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE));
        final Point3D point3D6 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE));

        final var point2D1 = camera.project(point3D1);
        var point2D2 = camera.project(point3D2);
        final var point2D3 = camera.project(point3D3);
        final var point2D4 = camera.project(point3D4);
        final var point2D5 = camera.project(point3D5);
        final var point2D6 = camera.project(point3D6);

        final var points3D = new ArrayList<Point3D>(N_POINTS);
        points3D.add(point3D1);
        points3D.add(point3D2);
        points3D.add(point3D3);
        points3D.add(point3D4);
        points3D.add(point3D5);
        points3D.add(point3D6);

        final var points2D = camera.project(points3D);

        camera = new PinholeCamera(point3D1, point3D2, point3D3, point3D4, point3D5, point3D6, point2D1, point2D2,
                point2D3, point2D4, point2D5, point2D6);
        final var estimator = new DLTPointCorrespondencePinholeCameraEstimator(points3D, points2D);
        estimator.setLMSESolutionAllowed(false);
        var estimatedCamera = estimator.estimate();

        // check equal-ness of cameras
        camera.decompose();
        estimatedCamera.decompose();

        intrinsic2 = camera.getIntrinsicParameters();
        var intrinsic3 = estimatedCamera.getIntrinsicParameters();

        assertEquals(intrinsic2.getHorizontalFocalLength(), intrinsic3.getHorizontalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic2.getVerticalFocalLength(), intrinsic3.getVerticalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic2.getSkewness(), intrinsic3.getSkewness(), ABSOLUTE_ERROR);
        assertEquals(intrinsic2.getHorizontalPrincipalPoint(), intrinsic3.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
        assertEquals(intrinsic2.getVerticalPrincipalPoint(), intrinsic3.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

        rotation2 = camera.getCameraRotation();
        Rotation3D rotation3 = estimatedCamera.getCameraRotation();

        assertTrue(rotation2.equals(rotation3, ABSOLUTE_ERROR));

        cameraCenter2 = camera.getCameraCenter();
        var cameraCenter3 = estimatedCamera.getCameraCenter();

        assertTrue(cameraCenter2.equals(cameraCenter3, ABSOLUTE_ERROR));

        // Force CameraException by repeating correspondences and creating a
        // degeneracy
        point3D2 = point3D1;
        point2D2 = camera.project(point3D2);
        final var wrongPoint3D2 = point3D2;
        final var wrongPoint2D2 = point2D2;
        assertThrows(CameraException.class, () -> new PinholeCamera(point3D1, wrongPoint3D2, point3D3, point3D4,
                point3D5, point3D6, point2D1, wrongPoint2D2, point2D3, point2D4, point2D5, point2D6));

        // constructor with line/plane correspondence

        // instantiate camera
        camera = new PinholeCamera(intrinsic, rotation, cameraCenter);

        // create 4 2D lines
        final var line2D1 = new Line2D(randomizer.nextDouble(MIN_RANDOM_LINE_VALUE, MAX_RANDOM_LINE_VALUE),
                randomizer.nextDouble(MIN_RANDOM_LINE_VALUE, MAX_RANDOM_LINE_VALUE),
                randomizer.nextDouble(MIN_RANDOM_LINE_VALUE, MAX_RANDOM_LINE_VALUE));
        var line2D2 = new Line2D(randomizer.nextDouble(MIN_RANDOM_LINE_VALUE, MAX_RANDOM_LINE_VALUE),
                randomizer.nextDouble(MIN_RANDOM_LINE_VALUE, MAX_RANDOM_LINE_VALUE),
                randomizer.nextDouble(MIN_RANDOM_LINE_VALUE, MAX_RANDOM_LINE_VALUE));
        final var line2D3 = new Line2D(randomizer.nextDouble(MIN_RANDOM_LINE_VALUE, MAX_RANDOM_LINE_VALUE),
                randomizer.nextDouble(MIN_RANDOM_LINE_VALUE, MAX_RANDOM_LINE_VALUE),
                randomizer.nextDouble(MIN_RANDOM_LINE_VALUE, MAX_RANDOM_LINE_VALUE));
        final var line2D4 = new Line2D(randomizer.nextDouble(MIN_RANDOM_LINE_VALUE, MAX_RANDOM_LINE_VALUE),
                randomizer.nextDouble(MIN_RANDOM_LINE_VALUE, MAX_RANDOM_LINE_VALUE),
                randomizer.nextDouble(MIN_RANDOM_LINE_VALUE, MAX_RANDOM_LINE_VALUE));

        final var plane1 = camera.backProject(line2D1);
        var plane2 = camera.backProject(line2D2);
        final var plane3 = camera.backProject(line2D3);
        final var plane4 = camera.backProject(line2D4);

        final var lines2D = new ArrayList<Line2D>(N_CORRESPONDENCES);
        lines2D.add(line2D1);
        lines2D.add(line2D2);
        lines2D.add(line2D3);
        lines2D.add(line2D4);

        final var planes = camera.backProjectLines(lines2D);

        camera = new PinholeCamera(plane1, plane2, plane3, plane4, line2D1, line2D2, line2D3, line2D4);
        final var estimator2 = new DLTLinePlaneCorrespondencePinholeCameraEstimator(planes, lines2D);
        estimator2.setLMSESolutionAllowed(false);
        estimatedCamera = estimator2.estimate();

        // check equal-ness of cameras
        camera.decompose();
        estimatedCamera.decompose();

        intrinsic2 = camera.getIntrinsicParameters();
        intrinsic3 = estimatedCamera.getIntrinsicParameters();

        assertEquals(intrinsic2.getHorizontalFocalLength(), intrinsic3.getHorizontalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic2.getVerticalFocalLength(), intrinsic3.getVerticalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic2.getSkewness(), intrinsic3.getSkewness(), ABSOLUTE_ERROR);
        assertEquals(intrinsic2.getHorizontalPrincipalPoint(), intrinsic3.getHorizontalPrincipalPoint(),
                ABSOLUTE_ERROR);
        assertEquals(intrinsic2.getVerticalPrincipalPoint(), intrinsic3.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

        rotation2 = camera.getCameraRotation();
        rotation3 = estimatedCamera.getCameraRotation();

        assertTrue(rotation2.equals(rotation3, ABSOLUTE_ERROR));

        cameraCenter2 = camera.getCameraCenter();
        cameraCenter3 = estimatedCamera.getCameraCenter();

        assertTrue(cameraCenter2.equals(cameraCenter3, ABSOLUTE_ERROR));

        // Force CameraException by repeating correspondences and creating a
        // degeneracy
        line2D2 = line2D1;
        plane2 = camera.backProject(line2D2);
        final var wrongPlane2 = plane2;
        assertThrows(CameraException.class, () -> new PinholeCamera(plane1, wrongPlane2, plane3, plane4, line2D1,
                line2D4, line2D3, line2D4));
    }

    @Test
    void testProjectPoints() throws WrongSizeException {
        // create intrinsic parameters
        final var randomizer = new UniformRandomizer();
        final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

        final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        // rotation
        final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

        final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

        // camera center
        final var cameraCenterArray = new double[INHOM_3D_COORDS];
        randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

        // instantiate camera
        final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);

        final var internalMatrix = camera.getInternalMatrix();

        assertEquals(PINHOLE_CAMERA_ROWS, internalMatrix.getRows());
        assertEquals(PINHOLE_CAMERA_COLS, internalMatrix.getColumns());

        // project list of world points
        final var nPoints = randomizer.nextInt(MIN_NUMBER_POINTS, MAX_NUMBER_POINTS);

        final var worldPointList = new ArrayList<Point3D>(nPoints);
        final var worldPointListMatrix = new Matrix(nPoints, Point3D.POINT3D_HOMOGENEOUS_COORDINATES_LENGTH);
        for (var i = 0; i < nPoints; i++) {
            final var pointArray = new double[Point3D.POINT3D_HOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(pointArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var point = new HomogeneousPoint3D(pointArray);
            worldPointList.add(point);
            worldPointListMatrix.setSubmatrix(i, 0, i,
                    Point3D.POINT3D_HOMOGENEOUS_COORDINATES_LENGTH - 1, pointArray);
        }

        final var transWorldPointListMatrix = worldPointListMatrix.transposeAndReturnNew();

        final var transImagePointListMatrix = internalMatrix.multiplyAndReturnNew(transWorldPointListMatrix);

        final var imagePointListMatrix = transImagePointListMatrix.transposeAndReturnNew();

        // list of image points projected by ourselves
        final var imagePointList = new ArrayList<Point2D>(nPoints);
        for (var i = 0; i < nPoints; i++) {
            final var pointArray = new double[Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH];
            imagePointListMatrix.getSubmatrixAsArray(i, 0, i,
                    Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH - 1, pointArray);
            final var point = new HomogeneousPoint2D(pointArray);
            imagePointList.add(point);
        }

        // list of image points to be tested
        final var imagePointList2 = camera.project(worldPointList);
        final var imagePointList3 = new ArrayList<Point2D>();
        camera.project(worldPointList, imagePointList3);

        assertEquals(imagePointList2.size(), nPoints);
        assertEquals(imagePointList.size(), nPoints);

        final var iterator = imagePointList.iterator();
        final var iterator2 = imagePointList2.iterator();
        final var iteratorWorld = worldPointList.iterator();

        while (iterator.hasNext() && iterator2.hasNext() && iteratorWorld.hasNext()) {
            final var imagePoint = iterator.next();
            final var imagePoint2 = iterator2.next();

            assertTrue(imagePoint.equals(imagePoint2, ABSOLUTE_ERROR));
        }

        assertEquals(imagePointList2, imagePointList3);

        // project single world point
        final var homWorldPointArray = new double[Point3D.POINT3D_HOMOGENEOUS_COORDINATES_LENGTH];
        randomizer.fill(homWorldPointArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var homWorldPoint = new HomogeneousPoint3D(homWorldPointArray);

        var imagePoint = camera.project(homWorldPoint);

        var homImagePointMatrix = internalMatrix.multiplyAndReturnNew(
                Matrix.newFromArray(homWorldPointArray, true));

        var scaleX = imagePoint.getHomX() / homImagePointMatrix.getElementAtIndex(0);
        var scaleY = imagePoint.getHomY() / homImagePointMatrix.getElementAtIndex(1);
        var scaleW = imagePoint.getHomW() / homImagePointMatrix.getElementAtIndex(2);

        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleW, ABSOLUTE_ERROR);
        assertEquals(scaleW, scaleX, ABSOLUTE_ERROR);

        // project another single world point
        randomizer.fill(homWorldPointArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        homWorldPoint = new HomogeneousPoint3D(homWorldPointArray);

        imagePoint = new HomogeneousPoint2D();
        camera.project(homWorldPoint, imagePoint);

        homImagePointMatrix = internalMatrix.multiplyAndReturnNew(
                Matrix.newFromArray(homWorldPointArray, true));

        scaleX = imagePoint.getHomX() / homImagePointMatrix.getElementAtIndex(0);
        scaleY = imagePoint.getHomY() / homImagePointMatrix.getElementAtIndex(1);
        scaleW = imagePoint.getHomW() / homImagePointMatrix.getElementAtIndex(2);

        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleW, ABSOLUTE_ERROR);
        assertEquals(scaleW, scaleX, ABSOLUTE_ERROR);
    }

    @Test
    void testBackProjectLines() throws WrongSizeException, NotReadyException, LockedException, DecomposerException,
            com.irurueta.algebra.NotAvailableException {

        // create intrinsic parameters
        final var randomizer = new UniformRandomizer();
        final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

        final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        // rotation
        final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

        final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

        // camera center
        final var cameraCenterArray = new double[INHOM_3D_COORDS];
        randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

        // instantiate camera
        final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);

        final var cameraMatrix = camera.getInternalMatrix();

        // instantiate random line to back-project
        final var lineArray = new double[HOM_2D_COORDS];
        randomizer.fill(lineArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var lineMatrix = Matrix.newFromArray(lineArray, true);

        final var line = new Line2D(lineArray);

        final var transCameraMatrix = cameraMatrix.transposeAndReturnNew();

        final var planeMatrix = transCameraMatrix.multiplyAndReturnNew(lineMatrix);

        var plane = camera.backProject(line);
        final var plane2 = new Plane();
        camera.backProject(line, plane2);

        var scaleA = plane.getA() / planeMatrix.getElementAtIndex(0);
        var scaleB = plane.getB() / planeMatrix.getElementAtIndex(1);
        var scaleC = plane.getC() / planeMatrix.getElementAtIndex(2);
        var scaleD = plane.getD() / planeMatrix.getElementAtIndex(3);

        assertEquals(scaleA, scaleB, ABSOLUTE_ERROR);
        assertEquals(scaleB, scaleC, ABSOLUTE_ERROR);
        assertEquals(scaleC, scaleD, ABSOLUTE_ERROR);
        assertEquals(scaleD, scaleA, ABSOLUTE_ERROR);
        assertEquals(plane, plane2);

        // test again back projection
        plane = new Plane();
        camera.backProject(line, plane);

        scaleA = plane.getA() / planeMatrix.getElementAtIndex(0);
        scaleB = plane.getB() / planeMatrix.getElementAtIndex(1);
        scaleC = plane.getC() / planeMatrix.getElementAtIndex(2);
        scaleD = plane.getD() / planeMatrix.getElementAtIndex(3);

        assertEquals(scaleA, scaleB, ABSOLUTE_ERROR);
        assertEquals(scaleB, scaleC, ABSOLUTE_ERROR);
        assertEquals(scaleC, scaleD, ABSOLUTE_ERROR);
        assertEquals(scaleD, scaleA, ABSOLUTE_ERROR);

        // to test correctness, randomly choose a world point located on
        // back-projected plane and project it to ensure that it lies on
        // provided line

        // row matrix containing plane parameters
        final var a = planeMatrix.transposeAndReturnNew();
        final var decomposer = new SingularValueDecomposer(a);
        decomposer.decompose();

        // a row matrix 1x4 will have rank 1 and nullity 3 (1 + 3 = 4)
        final var v = decomposer.getV();
        assertEquals(1, decomposer.getRank());
        assertEquals(3, decomposer.getNullity());

        // the last three columns of V contain the null-space of "a".
        // These three columns correspond to homogeneous 3D point coordinates
        // that belong to back-projected plane. Any linear-combination of such
        // three 3D points will also belong to such plane
        final var worldPoint1 = new HomogeneousPoint3D(
                v.getElementAt(0, 1), v.getElementAt(1, 1),
                v.getElementAt(2, 1), v.getElementAt(3, 1));
        final var worldPoint2 = new HomogeneousPoint3D(
                v.getElementAt(0, 2), v.getElementAt(1, 2),
                v.getElementAt(2, 2), v.getElementAt(3, 2));
        final var worldPoint3 = new HomogeneousPoint3D(
                v.getElementAt(0, 3), v.getElementAt(1, 3),
                v.getElementAt(2, 3), v.getElementAt(3, 3));

        // check that world points above belong to back-projected plane
        assertTrue(plane.isLocus(worldPoint1, ABSOLUTE_ERROR));
        assertTrue(plane.isLocus(worldPoint2, ABSOLUTE_ERROR));
        assertTrue(plane.isLocus(worldPoint3, ABSOLUTE_ERROR));

        // project world points
        final var imagePoint1 = camera.project(worldPoint1);
        final var imagePoint2 = camera.project(worldPoint2);
        final var imagePoint3 = camera.project(worldPoint3);

        // check that projected image points belong to line
        assertTrue(line.isLocus(imagePoint1, ABSOLUTE_ERROR));
        assertTrue(line.isLocus(imagePoint2, ABSOLUTE_ERROR));
        assertTrue(line.isLocus(imagePoint3, ABSOLUTE_ERROR));
    }

    @Test
    void testBackprojectPoints() throws CameraException {
        // create intrinsic parameters
        final var randomizer = new UniformRandomizer();
        final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

        final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        // rotation
        final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

        final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

        // camera center
        final var cameraCenterArray = new double[INHOM_3D_COORDS];
        randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

        // instantiate camera
        final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);

        // instantiate random 3D point
        final var worldPointArray = new double[HOM_3D_COORDS];
        randomizer.fill(worldPointArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var worldPoint = new HomogeneousPoint3D(worldPointArray);

        // project it
        var imagePoint = camera.project(worldPoint);

        // test that any point on such ray of light at any random distance will
        // project on the same image point
        final var rayOfLight = camera.backProject(imagePoint);
        final var rayOfLight2 = Point3D.create();
        camera.backProject(imagePoint, rayOfLight2);

        var imagePoint2 = camera.project(rayOfLight);

        assertTrue(imagePoint.equals(imagePoint2, ABSOLUTE_ERROR));
        assertEquals(rayOfLight, rayOfLight2);

        // now make a list of world points
        final var nPoints = randomizer.nextInt(MIN_N_POINTS, MAX_N_POINTS);

        final var worldPointList = new ArrayList<Point3D>(nPoints);
        for (var i = 0; i < nPoints; i++) {
            final var point = new HomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            worldPointList.add(point);
        }

        // project list of world points into image points
        final var imagePointList = camera.project(worldPointList);

        // back project image points to obtain a list of world points containing
        // rays of light
        final var worldPointList2 = camera.backProjectPoints(imagePointList);
        final var worldPointList3 = new ArrayList<Point3D>();
        camera.backProjectPoints(imagePointList, worldPointList3);

        // project again to ensure that projected rays of light produce the same
        // image points
        final var imagePointList2 = camera.project(worldPointList2);
        final var imagePointList3 = camera.project(worldPointList3);

        // iterate over image lists and check that both are equal
        final var iterator1 = imagePointList.iterator();
        final var iterator2 = imagePointList2.iterator();
        final var iterator3 = imagePointList3.iterator();

        while (iterator1.hasNext() && iterator2.hasNext() && iterator3.hasNext()) {
            imagePoint = iterator1.next();
            imagePoint2 = iterator2.next();

            assertTrue(imagePoint.equals(imagePoint2, ABSOLUTE_ERROR));
            imagePoint2 = iterator3.next();
            assertTrue(imagePoint.equals(imagePoint2, ABSOLUTE_ERROR));
        }
    }

    @Test
    void testDecomposeGetCameraRotationIntrinsicsCenterAndImageOfWorldOrigin() throws RotationException,
            WrongSizeException, CameraException, NotAvailableException {

        // create intrinsic parameters
        final var randomizer = new UniformRandomizer();
        final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

        final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        // rotation
        final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

        final var intrinsicMatrix = intrinsic.getInternalMatrix();

        final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);
        final var rotationMatrix = rotation.asInhomogeneousMatrix();

        final var axis = rotation.getRotationAxis();
        final var theta = rotation.getRotationAngle();

        // image of world origin
        final var originImageArray = new double[INHOM_2D_COORDS];
        randomizer.fill(originImageArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var originImage = new InhomogeneousPoint2D(originImageArray);

        // camera center
        final var cameraCenterArray = new double[INHOM_3D_COORDS];
        final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

        // test constructor with intrinsic parameters, rotation and image of
        // origin
        var camera = new PinholeCamera(intrinsic, rotation, originImage);
        assertFalse(camera.isCameraCenterAvailable());
        assertTrue(camera.isCameraRotationAvailable());
        assertTrue(camera.areIntrinsicParametersAvailable());

        // build internal camera matrix
        var cameraMatrix2 = new Matrix(PINHOLE_CAMERA_ROWS, PINHOLE_CAMERA_COLS);
        var mp = intrinsicMatrix.multiplyAndReturnNew(rotationMatrix);
        for (var v = 0; v < PINHOLE_CAMERA_ROWS; v++) {
            for (var u = 0; u < PINHOLE_CAMERA_ROWS; u++) {
                cameraMatrix2.setElementAt(u, v, mp.getElementAt(u, v));
            }
        }

        // set last column of camera matrix with homogeneous coordinates of image of world origin
        cameraMatrix2.setElementAt(0, 3, originImage.getHomX());
        cameraMatrix2.setElementAt(1, 3, originImage.getHomY());
        cameraMatrix2.setElementAt(2, 3, originImage.getHomW());

        var cameraMatrix = camera.getInternalMatrix();

        // compare matrices cameraMatrix and cameraMatrix2
        assertTrue(cameraMatrix.equals(cameraMatrix2, ABSOLUTE_ERROR));

        // decompose created camera and check that obtained intrinsic parameters,
        // rotation and image of world origin are correct
        camera.decompose(true, true);

        assertTrue(camera.isCameraCenterAvailable());
        assertTrue(camera.isCameraRotationAvailable());
        assertTrue(camera.areIntrinsicParametersAvailable());

        var rotation2 = camera.getCameraRotation();
        var intrinsic2 = camera.getIntrinsicParameters();
        final var originImage2 = camera.getImageOfWorldOrigin();

        // obtain rotation axis and angle from 2nd rotation
        var axis2 = rotation2.getRotationAxis();
        var theta2 = rotation2.getRotationAngle();

        // check correctness of axis and angle

        // axis can be equal up to sign
        var scaleAxisX = axis[0] / axis2[0];
        var scaleAxisY = axis[1] / axis2[1];
        var scaleAxisZ = axis[2] / axis2[2];

        assertEquals(scaleAxisX, scaleAxisY, ABSOLUTE_ERROR);
        assertEquals(scaleAxisY, scaleAxisZ, ABSOLUTE_ERROR);
        assertEquals(scaleAxisZ, scaleAxisX, ABSOLUTE_ERROR);

        // if sign of rotation axis is the opposite (-1.0), then theta will be -theta (opposite sign)
        if (scaleAxisX > 0.0) {
            assertEquals(theta2, theta, ABSOLUTE_ERROR);
        } else {
            assertEquals(-theta2, theta, ABSOLUTE_ERROR);
        }

        // compare intrinsic parameters
        assertEquals(horizontalFocalLength, intrinsic2.getHorizontalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(verticalFocalLength, intrinsic2.getVerticalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(skewness, intrinsic2.getSkewness(), ABSOLUTE_ERROR);
        assertEquals(horizontalPrincipalPoint, intrinsic2.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
        assertEquals(verticalPrincipalPoint, intrinsic2.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

        // Compare images of origin
        assertTrue(originImage.equals(originImage2, ABSOLUTE_ERROR));

        // test constructor with intrinsic parameters, rotation and camera center
        camera = new PinholeCamera(intrinsic, rotation, cameraCenter);
        assertTrue(camera.isCameraCenterAvailable());
        assertTrue(camera.isCameraRotationAvailable());
        assertTrue(camera.areIntrinsicParametersAvailable());

        // build internal matrix
        cameraMatrix2 = new Matrix(PINHOLE_CAMERA_ROWS, PINHOLE_CAMERA_COLS);
        mp = intrinsicMatrix.multiplyAndReturnNew(rotationMatrix);
        for (var v = 0; v < PINHOLE_CAMERA_ROWS; v++) {
            for (var u = 0; u < PINHOLE_CAMERA_ROWS; u++) {
                cameraMatrix2.setElementAt(u, v, mp.getElementAt(u, v));
            }
        }

        var inhomCenter = new Matrix(PINHOLE_CAMERA_ROWS, 1);
        inhomCenter.setElementAtIndex(0, cameraCenter.getInhomX());
        inhomCenter.setElementAtIndex(1, cameraCenter.getInhomY());
        inhomCenter.setElementAtIndex(2, cameraCenter.getInhomZ());
        final var p4 = mp.multiplyAndReturnNew(inhomCenter).multiplyByScalarAndReturnNew(-1.0);

        // set last column of camera matrix with homogeneous coordinates of image of world origin
        cameraMatrix.setElementAt(0, 3, p4.getElementAtIndex(0));
        cameraMatrix.setElementAt(1, 3, p4.getElementAtIndex(1));
        cameraMatrix.setElementAt(2, 3, p4.getElementAtIndex(2));

        cameraMatrix = camera.getInternalMatrix();

        // test that camera center is the null-space of camera matrix
        final var homCenter = new Matrix(PINHOLE_CAMERA_COLS, 1);
        var homCameraCenter = new HomogeneousPoint3D(cameraCenter);
        homCameraCenter.normalize();
        homCenter.setElementAtIndex(0, homCameraCenter.getHomX());
        homCenter.setElementAtIndex(1, homCameraCenter.getHomY());
        homCenter.setElementAtIndex(2, homCameraCenter.getHomZ());
        homCenter.setElementAtIndex(3, homCameraCenter.getHomW());
        var nullMatrix = cameraMatrix.multiplyAndReturnNew(homCenter);
        assertEquals(0.0, nullMatrix.getElementAtIndex(0), ABSOLUTE_ERROR);
        assertEquals(0.0, nullMatrix.getElementAtIndex(1), ABSOLUTE_ERROR);
        assertEquals(0.0, nullMatrix.getElementAtIndex(2), ABSOLUTE_ERROR);

        // compare matrices cameraMatrix and cameraMatrix2
        assertTrue(cameraMatrix.equals(cameraMatrix2, ABSOLUTE_ERROR));

        // decompose created camera and check that obtained intrinsic parameters,
        // rotation and image of world origin are correct
        camera.decompose(true, true);

        assertTrue(camera.isCameraCenterAvailable());
        assertTrue(camera.isCameraRotationAvailable());
        assertTrue(camera.areIntrinsicParametersAvailable());

        rotation2 = camera.getCameraRotation();
        intrinsic2 = camera.getIntrinsicParameters();
        final var cameraCenter2 = camera.getCameraCenter();

        homCameraCenter = new HomogeneousPoint3D(cameraCenter2);
        homCameraCenter.normalize();
        homCenter.setElementAtIndex(0, homCameraCenter.getHomX());
        homCenter.setElementAtIndex(1, homCameraCenter.getHomY());
        homCenter.setElementAtIndex(2, homCameraCenter.getHomZ());
        homCenter.setElementAtIndex(3, homCameraCenter.getHomW());
        nullMatrix = cameraMatrix.multiplyAndReturnNew(homCenter);
        assertEquals(0.0, nullMatrix.getElementAtIndex(0), ABSOLUTE_ERROR);
        assertEquals(0.0, nullMatrix.getElementAtIndex(1), ABSOLUTE_ERROR);
        assertEquals(0.0, nullMatrix.getElementAtIndex(2), ABSOLUTE_ERROR);

        // obtain rotation axis and angle from 2nd rotation
        axis2 = rotation2.getRotationAxis();
        theta2 = rotation2.getRotationAngle();

        // check correctness of axis and angle

        // axis can be equal up to sign
        scaleAxisX = axis[0] / axis2[0];
        scaleAxisY = axis[1] / axis2[1];
        scaleAxisZ = axis[2] / axis2[2];

        assertEquals(scaleAxisX, scaleAxisY, ABSOLUTE_ERROR);
        assertEquals(scaleAxisY, scaleAxisZ, ABSOLUTE_ERROR);
        assertEquals(scaleAxisZ, scaleAxisX, ABSOLUTE_ERROR);

        // if sign of rotation axis is the opposite (-1.0), then theta will be
        // -theta (opposite sign)
        if (scaleAxisX > 0.0) {
            assertEquals(theta2, theta, ABSOLUTE_ERROR);
        } else {
            assertEquals(-theta2, theta, ABSOLUTE_ERROR);
        }

        // compare intrinsic parameters
        assertEquals(horizontalFocalLength, intrinsic2.getHorizontalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(verticalFocalLength, intrinsic2.getVerticalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(skewness, intrinsic2.getSkewness(), ABSOLUTE_ERROR);
        assertEquals(horizontalPrincipalPoint, intrinsic2.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
        assertEquals(verticalPrincipalPoint, intrinsic2.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

        // compare camera centers
        assertTrue(cameraCenter.equals(cameraCenter2, ABSOLUTE_ERROR));

        // test with intrinsic and rotation and camera center by efault (both
        // enabled)
        camera.decompose();

        assertTrue(camera.isCameraCenterAvailable());
        assertTrue(camera.isCameraRotationAvailable());
        assertTrue(camera.areIntrinsicParametersAvailable());

        // test with intrinsic and rotation enabled and camera center by default
        // (enabled)
        camera.decompose(true);

        assertTrue(camera.isCameraCenterAvailable());
        assertTrue(camera.isCameraRotationAvailable());
        assertTrue(camera.areIntrinsicParametersAvailable());

        // test with intrinsic and rotation disabled and camera center by default
        // (enabled)
        camera.decompose(false);

        assertTrue(camera.isCameraCenterAvailable());
        assertFalse(camera.isCameraRotationAvailable());
        assertFalse(camera.areIntrinsicParametersAvailable());

        // test with intrinsic and rotation enabled and camera center enabled
        camera.decompose(true, true);

        assertTrue(camera.isCameraCenterAvailable());
        assertTrue(camera.isCameraRotationAvailable());
        assertTrue(camera.areIntrinsicParametersAvailable());

        // test with intrinsic and rotation enabled and camera center disabled
        camera.decompose(true, false);

        assertFalse(camera.isCameraCenterAvailable());
        assertTrue(camera.isCameraRotationAvailable());
        assertTrue(camera.areIntrinsicParametersAvailable());

        // test with intrinsic and rotation disabled and camera center enabled
        camera.decompose(false, true);

        assertTrue(camera.isCameraCenterAvailable());
        assertFalse(camera.isCameraRotationAvailable());
        assertFalse(camera.areIntrinsicParametersAvailable());

        // test with intrinsic and rotation disabled and camera center disabled
        camera.decompose(false, false);

        assertFalse(camera.isCameraCenterAvailable());
        assertFalse(camera.isCameraRotationAvailable());
        assertFalse(camera.areIntrinsicParametersAvailable());
    }

    @Test
    void testNormalize() throws WrongSizeException {
        final var cameraMatrix = Matrix.createWithUniformRandomValues(PINHOLE_CAMERA_ROWS, PINHOLE_CAMERA_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var camera = new PinholeCamera(cameraMatrix);

        final var norm = Math.sqrt(Math.pow(cameraMatrix.getElementAt(2, 0), 2.0)
                + Math.pow(cameraMatrix.getElementAt(2, 1), 2.0)
                + Math.pow(cameraMatrix.getElementAt(2, 2), 2.0));

        // normalize and copy
        final var cameraMatrix2 = cameraMatrix.multiplyByScalarAndReturnNew(1.0 / norm);

        // normalize camera which also normalizes internal matrix passed by reference
        camera.normalize();

        // compare both matrices
        assertTrue(cameraMatrix.equals(cameraMatrix2, ABSOLUTE_ERROR));
    }

    @Test
    void testCameraSign() throws WrongSizeException, DecomposerException, CameraException {

        final var cameraMatrix = Matrix.createWithUniformRandomValues(PINHOLE_CAMERA_ROWS, PINHOLE_CAMERA_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var camera = new PinholeCamera(cameraMatrix);

        // obtain camera sign and check correctness
        final var mp = cameraMatrix.getSubmatrix(0, 0, 2, 2);

        final var detMp = com.irurueta.algebra.Utils.det(mp);

        final var cameraSign = (detMp > 0.0) ? 1.0 : -1.0;
        final var cameraSign2 = camera.getCameraSign();

        assertEquals(cameraSign, cameraSign2, ABSOLUTE_ERROR);

        // set threshold equal to detMp
        final var cameraSign3 = camera.getCameraSign(detMp);
        assertEquals(-1.0, cameraSign3, ABSOLUTE_ERROR);

        // camera hasn't fixed sign yet
        assertFalse(camera.isCameraSignFixed());

        // fix camera sign
        camera.fixCameraSign();
        assertTrue(camera.isCameraSignFixed());
        assertTrue(camera.getCameraSign() > 0.0);
    }

    @Test
    void testGetSetInternalMatrix() throws WrongSizeException {
        final var cameraMatrix = Matrix.createWithUniformRandomValues(PINHOLE_CAMERA_ROWS, PINHOLE_CAMERA_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var camera = new PinholeCamera(cameraMatrix);
        assertTrue(cameraMatrix.equals(camera.getInternalMatrix(), ABSOLUTE_ERROR));

        // set new camera matrix
        final var cameraMatrix2 = Matrix.createWithUniformRandomValues(PINHOLE_CAMERA_ROWS, PINHOLE_CAMERA_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        camera.setInternalMatrix(cameraMatrix2);
        assertTrue(cameraMatrix2.equals(camera.getInternalMatrix(), ABSOLUTE_ERROR));
    }

    @Test
    void testGetSetCameraRotation() throws CameraException, NotAvailableException {
        final var randomizer = new UniformRandomizer();
        final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

        // instantiate canonical camera
        final var camera = new PinholeCamera();
        assertFalse(camera.isCameraRotationAvailable());

        // Force NotAvailableException
        assertThrows(NotAvailableException.class, camera::getCameraRotation);

        // set new rotation
        camera.setCameraRotation(rotation);
        assertTrue(camera.isCameraRotationAvailable());

        // check correctness of rotation
        assertEquals(rotation, camera.getCameraRotation());

        // decompose
        camera.decompose();
        assertTrue(camera.isCameraRotationAvailable());

        // retrieve new rotation instance
        final var rotation2 = camera.getCameraRotation();

        // compare euler angles
        assertTrue(rotation2.asInhomogeneousMatrix().equals(rotation.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
    }

    @Test
    void testGetSetIntrinsicsAndAvailability() throws CameraException, NotAvailableException {
        final var randomizer = new UniformRandomizer();
        final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

        final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

        final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        final var k = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

        // instantiate canonical camera
        final var camera = new PinholeCamera();
        assertFalse(camera.areIntrinsicParametersAvailable());

        // Force NotAvailableException
        assertThrows(NotAvailableException.class, camera::getIntrinsicParameters);

        // set new intrinsic parameters
        camera.setIntrinsicParameters(k);
        assertTrue(camera.areIntrinsicParametersAvailable());

        // check correctness of intrinsic parameters
        assertEquals(k, camera.getIntrinsicParameters());

        // decompose
        camera.decompose();
        assertTrue(camera.areIntrinsicParametersAvailable());

        // retrieve new intrinsics instance
        final var k2 = camera.getIntrinsicParameters();

        // compare intrinsic parameters
        assertEquals(horizontalFocalLength, k2.getHorizontalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(verticalFocalLength, k2.getVerticalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(skewness, k2.getSkewness(), ABSOLUTE_ERROR);
        assertEquals(horizontalPrincipalPoint, k2.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
        assertEquals(verticalPrincipalPoint, k2.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);
    }

    @Test
    void testRotate() throws WrongSizeException, CameraException, NotAvailableException, RotationException {

        final var axisMatrix = Matrix.createWithUniformRandomValues(INHOM_3D_COORDS, 1, MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final var norm = com.irurueta.algebra.Utils.normF(axisMatrix);
        axisMatrix.multiplyByScalar(1.0 / norm);

        final var axisArray = axisMatrix.toArray();

        final var randomizer = new UniformRandomizer();
        final var theta1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var theta2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var rotation1 = new MatrixRotation3D(axisArray, theta1);
        final var rotation2 = new MatrixRotation3D(axisArray, theta2);

        // instantiate canonical camera
        final var camera = new PinholeCamera();

        // set initial rotation with computed axis and rotation angle theta1
        camera.setCameraRotation(rotation1);

        // from current rotation, rotate more using the same axis and theta2
        camera.rotate(rotation2);
        assertTrue(camera.isCameraRotationAvailable());

        // obtain current rotation
        final var rotation3 = camera.getCameraRotation();

        // obtain current axis and rotation angle
        final var axis3 = rotation3.getRotationAxis();
        final var theta3 = rotation3.getRotationAngle();

        // check that rotation axis is equal up to sign
        final var scaleX = axisArray[0] / axis3[0];
        final var scaleY = axisArray[1] / axis3[1];
        final var scaleZ = axisArray[2] / axis3[2];

        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleZ, ABSOLUTE_ERROR);
        assertEquals(scaleZ, scaleX, ABSOLUTE_ERROR);

        if (scaleX > 0.0) {
            // rotation angle remains equal
            assertEquals(theta1 + theta2, theta3, ABSOLUTE_ERROR);
        } else {
            // rotation axis has opposite sign, and so does rotation angle
            assertEquals(-(theta1 + theta2), theta3, ABSOLUTE_ERROR);
        }
    }

    @Test
    void testPointAt() throws WrongSizeException, CameraException, NotAvailableException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var centerMatrix = Matrix.createWithUniformRandomValues(INHOM_3D_COORDS, 1, MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            final var cameraCenter = new InhomogeneousPoint3D(
                    centerMatrix.getElementAtIndex(0),
                    centerMatrix.getElementAtIndex(1),
                    centerMatrix.getElementAtIndex(2));

            final var randomizer = new UniformRandomizer();
            final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

            final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

            final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            // random camera
            final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);

            // point to a new random location
            final var pointAtMatrix = Matrix.createWithUniformRandomValues(INHOM_3D_COORDS, 1,
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var pointAt = new InhomogeneousPoint3D(
                    pointAtMatrix.getElementAtIndex(0),
                    pointAtMatrix.getElementAtIndex(1),
                    pointAtMatrix.getElementAtIndex(2));

            camera.pointAt(pointAt);

            // after pointing the camera to a new location, we decompose it
            camera.decompose(true, true);
            // and retrieve new intrinsics, rotation and camera center, and ensure
            // that intrinsics and camera center have not been modified
            final var intrinsic2 = camera.getIntrinsicParameters();
            final var cameraCenter2 = camera.getCameraCenter();

            assertTrue(camera.areIntrinsicParametersAvailable());
            assertTrue(camera.isCameraCenterAvailable());

            assertEquals(intrinsic.getHorizontalFocalLength(), intrinsic2.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(intrinsic.getVerticalFocalLength(), intrinsic2.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(intrinsic.getSkewness(), intrinsic2.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(intrinsic.getHorizontalPrincipalPoint(), intrinsic2.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(intrinsic.getVerticalPrincipalPoint(), intrinsic2.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            if (Math.abs(cameraCenter.getInhomX() - cameraCenter2.getInhomX()) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(cameraCenter.getInhomX(), cameraCenter2.getInhomX(), LARGE_ABSOLUTE_ERROR);
            if (Math.abs(cameraCenter.getInhomY() - cameraCenter2.getInhomY()) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(cameraCenter.getInhomY(), cameraCenter2.getInhomY(), LARGE_ABSOLUTE_ERROR);
            if (Math.abs(cameraCenter.getInhomZ() - cameraCenter2.getInhomZ()) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(cameraCenter.getInhomZ(), cameraCenter2.getInhomZ(), LARGE_ABSOLUTE_ERROR);

            // principal axis array is a 3-component array indicating direction
            // where camera is looking at
            final var principalAxis = camera.getPrincipalAxisArray();

            // diff array contains the difference between provided 3D point to
            // look at and camera center. This array has to be proportional (up to
            // scale) to the principal axis array
            final var diffArray = new double[INHOM_3D_COORDS];
            diffArray[0] = pointAt.getInhomX() - cameraCenter2.getInhomX();
            diffArray[1] = pointAt.getInhomY() - cameraCenter2.getInhomY();
            diffArray[2] = pointAt.getInhomZ() - cameraCenter2.getInhomZ();
            final var norm = com.irurueta.algebra.Utils.normF(diffArray);
            ArrayUtils.multiplyByScalar(diffArray, 1.0 / norm, diffArray);

            final var scaleX = diffArray[0] / principalAxis[0];
            final var scaleY = diffArray[1] / principalAxis[1];
            final var scaleZ = diffArray[2] / principalAxis[2];

            if (Math.abs(scaleX - scaleY) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(scaleX, scaleY, LARGE_ABSOLUTE_ERROR);
            if (Math.abs(scaleY - scaleZ) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(scaleY, scaleZ, LARGE_ABSOLUTE_ERROR);
            if (Math.abs(scaleZ - scaleX) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(scaleZ, scaleX, LARGE_ABSOLUTE_ERROR);

            numValid++;
            break;
        }
        assertTrue(numValid > 0);
    }

    @Test
    void testGetSetCameraCenterAvailabilityAndComputeCameraCenterSVDDetAndFinite() throws WrongSizeException,
            CameraException, NotAvailableException {

        final var cameraCenterMatrix = Matrix.createWithUniformRandomValues(INHOM_3D_COORDS, 1,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var cameraCenter = new InhomogeneousPoint3D(
                cameraCenterMatrix.getElementAtIndex(0),
                cameraCenterMatrix.getElementAtIndex(1),
                cameraCenterMatrix.getElementAtIndex(2));

        final var randomizer = new UniformRandomizer();
        final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

        final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

        final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);
        final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

        // instantiate new pinhole camera
        final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);

        // set new camera center
        final var cameraCenterMatrix2 = Matrix.createWithUniformRandomValues(INHOM_3D_COORDS, 1,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var cameraCenter2 = new InhomogeneousPoint3D(
                cameraCenterMatrix2.getElementAtIndex(0),
                cameraCenterMatrix2.getElementAtIndex(1),
                cameraCenterMatrix2.getElementAtIndex(2));

        camera.setCameraCenter(cameraCenter2);

        assertTrue(camera.areIntrinsicParametersAvailable());
        assertTrue(camera.isCameraCenterAvailable());
        assertTrue(camera.isCameraRotationAvailable());

        // decompose matrix to retrieve center, intrinsics and rotation
        camera.decompose(true, true);
        assertTrue(camera.areIntrinsicParametersAvailable());
        assertTrue(camera.isCameraCenterAvailable());
        assertTrue(camera.isCameraRotationAvailable());

        final var rotation2 = camera.getCameraRotation();
        final var intrinsic2 = camera.getIntrinsicParameters();
        final var cameraCenter3 = camera.getCameraCenter();

        // also compute center without decomposition using different methods
        final var cameraCenterSVD = camera.computeCameraCenterSVD();
        final var cameraCenterSVD2 = Point3D.create();
        camera.computeCameraCenterSVD(cameraCenterSVD2);
        final var cameraCenterDet = camera.computeCameraCenterDet();
        final var cameraCenterDet2 = Point3D.create();
        camera.computeCameraCenterDet(cameraCenterDet2);
        final var cameraCenterFinite = camera.computeCameraCenterFiniteCamera();
        final var cameraCenterFinite2 = Point3D.create();
        camera.computeCameraCenterFiniteCamera(cameraCenterFinite2);

        // check that rotation and intrinsics haven't changed while
        // camera center has been correctly modified

        // center has been correctly set
        assertTrue(cameraCenter2.equals(cameraCenter3, VERY_LARGE_ABSOLUTE_ERROR));
        assertTrue(cameraCenterSVD.equals(cameraCenter3, LARGE_ABSOLUTE_ERROR));
        assertEquals(cameraCenterSVD, cameraCenterSVD2);
        assertTrue(cameraCenterDet.equals(cameraCenter3, ABSOLUTE_ERROR));
        assertEquals(cameraCenterDet, cameraCenterDet2);
        assertTrue(cameraCenterFinite.equals(cameraCenter3, ABSOLUTE_ERROR));
        assertEquals(cameraCenterFinite, cameraCenterFinite2);

        // rotation remains the same
        assertTrue(rotation.asInhomogeneousMatrix().equals(rotation2.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

        // intrinsics remains the same
        assertEquals(intrinsic.getHorizontalFocalLength(), intrinsic2.getHorizontalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getVerticalFocalLength(), intrinsic2.getVerticalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getSkewness(), intrinsic2.getSkewness(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getHorizontalPrincipalPoint(), intrinsic2.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getVerticalPrincipalPoint(), intrinsic2.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);
    }

    @Test
    void testSetIntrinsicParametersAndRotation() throws CameraException, NotAvailableException {

        final var randomizer = new UniformRandomizer();
        final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

        final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

        final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);
        final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

        // instantiate canonical pinhole camera (focal length equal to 1,
        // pointing towards z axis and located at origin)
        final var camera = new PinholeCamera();

        // set intrinsic parameters and rotation
        camera.setIntrinsicParametersAndRotation(intrinsic, rotation);

        // decompose camera to obtain new intrinsics, rotation and center
        camera.decompose(true, true);
        // decomposed elements are available
        assertTrue(camera.isCameraCenterAvailable());
        assertTrue(camera.isCameraRotationAvailable());
        assertTrue(camera.areIntrinsicParametersAvailable());

        final var cameraCenter2 = camera.getCameraCenter();
        final var rotation2 = camera.getCameraRotation();
        final var intrinsic2 = camera.getIntrinsicParameters();

        // check camera center remains at origin
        assertEquals(0.0, cameraCenter2.getInhomX(), ABSOLUTE_ERROR);
        assertEquals(0.0, cameraCenter2.getInhomY(), ABSOLUTE_ERROR);
        assertEquals(0.0, cameraCenter2.getInhomZ(), ABSOLUTE_ERROR);

        // check camera rotation correctness
        // are different instances
        assertNotSame(rotation, rotation2);
        assertTrue(rotation.asInhomogeneousMatrix().equals(rotation2.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

        // check camera intrinsic parameters
        // are different instances
        assertNotSame(intrinsic, intrinsic2);
        assertEquals(intrinsic.getHorizontalFocalLength(), intrinsic2.getHorizontalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getVerticalFocalLength(), intrinsic2.getVerticalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getSkewness(), intrinsic2.getSkewness(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getHorizontalPrincipalPoint(), intrinsic2.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getVerticalPrincipalPoint(), intrinsic2.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);
    }

    @Test
    void testSetIntrinsicAndExtrinsicParameters() throws WrongSizeException, CameraException, NotAvailableException {

        final var centerMatrix = Matrix.createWithUniformRandomValues(INHOM_3D_COORDS, 1, MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final var cameraCenter = new InhomogeneousPoint3D(
                centerMatrix.getElementAtIndex(0),
                centerMatrix.getElementAtIndex(1),
                centerMatrix.getElementAtIndex(2));

        final var worldOriginMatrix = Matrix.createWithUniformRandomValues(HOM_2D_COORDS, 1, MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final var imageOfWorldOrigin = new HomogeneousPoint2D(
                worldOriginMatrix.getElementAtIndex(0),
                worldOriginMatrix.getElementAtIndex(1),
                worldOriginMatrix.getElementAtIndex(2));

        final var randomizer = new UniformRandomizer();
        final var alphaEuler1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var betaEuler1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var gammaEuler1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var alphaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var betaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var gammaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var horizontalFocalLength1 = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final var verticalFocalLength1 = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

        final var horizontalFocalLength2 = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final var verticalFocalLength2 = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

        final var skewness1 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
        final var skewness2 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

        final var horizontalPrincipalPoint1 = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final var verticalPrincipalPoint1 = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        final var horizontalPrincipalPoint2 = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final var verticalPrincipalPoint2 = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
        final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

        final var intrinsic1 = new PinholeCameraIntrinsicParameters(horizontalFocalLength1, verticalFocalLength1,
                horizontalPrincipalPoint1, verticalPrincipalPoint1, skewness1);
        final var intrinsic2 = new PinholeCameraIntrinsicParameters(horizontalFocalLength2, verticalFocalLength2,
                horizontalPrincipalPoint2, verticalPrincipalPoint2, skewness2);

        // instantiate canonical pinhole camera
        final var camera = new PinholeCamera();

        // set intrinsic and extrinsic parameters using image of world origin
        camera.setIntrinsicAndExtrinsicParameters(intrinsic1, rotation1, imageOfWorldOrigin);
        assertFalse(camera.isCameraCenterAvailable());
        assertTrue(camera.isCameraRotationAvailable());
        assertTrue(camera.areIntrinsicParametersAvailable());

        // decompose camera to check correctness of parameters that have been set
        camera.decompose(true, true);
        assertTrue(camera.isCameraCenterAvailable());
        assertTrue(camera.isCameraRotationAvailable());
        assertTrue(camera.areIntrinsicParametersAvailable());

        // check correctness of image of world origin
        final var imageOfWorldOrigin2 = camera.getImageOfWorldOrigin();
        assertTrue(imageOfWorldOrigin.equals(imageOfWorldOrigin2, ABSOLUTE_ERROR));

        // check correctness of intrinsic parameters
        final var intrinsic3 = camera.getIntrinsicParameters();
        assertEquals(intrinsic3.getHorizontalFocalLength(), intrinsic1.getHorizontalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic3.getVerticalFocalLength(), intrinsic1.getVerticalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic3.getSkewness(), intrinsic1.getSkewness(), ABSOLUTE_ERROR);
        assertEquals(intrinsic3.getHorizontalPrincipalPoint(), intrinsic1.getHorizontalPrincipalPoint(),
                ABSOLUTE_ERROR);
        assertEquals(intrinsic3.getVerticalPrincipalPoint(), intrinsic1.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

        // check correctness of rotation
        final var rotation3 = camera.getCameraRotation();
        assertTrue(rotation3.asInhomogeneousMatrix().equals(rotation1.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

        // set new intrinsic and extrinsic parameters using camera center
        camera.setIntrinsicAndExtrinsicParameters(intrinsic2, rotation2, cameraCenter);
        assertTrue(camera.isCameraCenterAvailable());
        assertTrue(camera.isCameraRotationAvailable());
        assertTrue(camera.areIntrinsicParametersAvailable());

        // decompose camera to check correctness of parameters that have been set
        camera.decompose(true, true);
        assertTrue(camera.isCameraCenterAvailable());
        assertTrue(camera.isCameraRotationAvailable());
        assertTrue(camera.areIntrinsicParametersAvailable());

        // check correctness of camera center
        final var cameraCenter2 = camera.getCameraCenter();
        assertTrue(cameraCenter.equals(cameraCenter2, LARGE_ABSOLUTE_ERROR));

        // check correctness of intrinsic parameters
        final var intrinsic4 = camera.getIntrinsicParameters();
        assertEquals(intrinsic4.getHorizontalFocalLength(), intrinsic2.getHorizontalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic4.getVerticalFocalLength(), intrinsic2.getVerticalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic4.getSkewness(), intrinsic2.getSkewness(), ABSOLUTE_ERROR);
        assertEquals(intrinsic4.getHorizontalPrincipalPoint(), intrinsic2.getHorizontalPrincipalPoint(),
                ABSOLUTE_ERROR);
        assertEquals(intrinsic4.getVerticalPrincipalPoint(), intrinsic2.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

        // check correctness of rotation
        final var rotation4 = camera.getCameraRotation();
        assertTrue(rotation4.asInhomogeneousMatrix().equals(rotation2.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
    }

    @Test
    void testGetSetVanishingPointsAndImageOfWorldOrigin() throws WrongSizeException {
        var internalMatrix = Matrix.createWithUniformRandomValues(PINHOLE_CAMERA_ROWS, PINHOLE_CAMERA_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var camera = new PinholeCamera(internalMatrix);

        final var randomizer = new UniformRandomizer();

        // test get set x-axis vanishing point
        final var xAxisVanishingPoint = camera.getXAxisVanishingPoint();
        var scaleX = internalMatrix.getElementAt(0, 0) / xAxisVanishingPoint.getHomX();
        var scaleY = internalMatrix.getElementAt(1, 0) / xAxisVanishingPoint.getHomY();
        var scaleW = internalMatrix.getElementAt(2, 0) / xAxisVanishingPoint.getHomW();
        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleW, ABSOLUTE_ERROR);
        assertEquals(scaleW, scaleX, ABSOLUTE_ERROR);
        final var xAxisVanishingPoint2 = Point2D.create();
        camera.xAxisVanishingPoint(xAxisVanishingPoint2);
        assertEquals(xAxisVanishingPoint, xAxisVanishingPoint2);

        // set new x-axis vanishing point
        var x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var w = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        xAxisVanishingPoint.setHomogeneousCoordinates(x, y, w);
        camera.setXAxisVanishingPoint(xAxisVanishingPoint);
        // when setting x-axis vanishing point, internal matrix will be modified
        internalMatrix = camera.getInternalMatrix();
        scaleX = internalMatrix.getElementAt(0, 0) / xAxisVanishingPoint.getHomX();
        scaleY = internalMatrix.getElementAt(1, 0) / xAxisVanishingPoint.getHomY();
        scaleW = internalMatrix.getElementAt(2, 0) / xAxisVanishingPoint.getHomW();
        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleW, ABSOLUTE_ERROR);
        assertEquals(scaleW, scaleX, ABSOLUTE_ERROR);

        // test get set y-axis vanishing point
        final var yAxisVanishingPoint = camera.getYAxisVanishingPoint();
        scaleX = internalMatrix.getElementAt(0, 1) / yAxisVanishingPoint.getHomX();
        scaleY = internalMatrix.getElementAt(1, 1) / yAxisVanishingPoint.getHomY();
        scaleW = internalMatrix.getElementAt(2, 1) / yAxisVanishingPoint.getHomW();
        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleW, ABSOLUTE_ERROR);
        assertEquals(scaleW, scaleX, ABSOLUTE_ERROR);
        final var yAxisVanishingPoint2 = Point2D.create();
        camera.yAxisVanishingPoint(yAxisVanishingPoint2);
        assertEquals(yAxisVanishingPoint, yAxisVanishingPoint2);

        // set new y-axis vanishing point
        x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        w = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        yAxisVanishingPoint.setHomogeneousCoordinates(x, y, w);
        camera.setYAxisVanishingPoint(yAxisVanishingPoint);
        // when setting y-axis vanishing point, internal matrix will be modified
        internalMatrix = camera.getInternalMatrix();
        scaleX = internalMatrix.getElementAt(0, 1) / yAxisVanishingPoint.getHomX();
        scaleY = internalMatrix.getElementAt(1, 1) / yAxisVanishingPoint.getHomY();
        scaleW = internalMatrix.getElementAt(2, 1) / yAxisVanishingPoint.getHomW();
        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleW, ABSOLUTE_ERROR);
        assertEquals(scaleW, scaleX, ABSOLUTE_ERROR);

        // test get set z-axis vanishing point
        final var zAxisVanishingPoint = camera.getZAxisVanishingPoint();
        scaleX = internalMatrix.getElementAt(0, 2) / zAxisVanishingPoint.getHomX();
        scaleY = internalMatrix.getElementAt(1, 2) / zAxisVanishingPoint.getHomY();
        scaleW = internalMatrix.getElementAt(2, 2) / zAxisVanishingPoint.getHomW();
        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleW, ABSOLUTE_ERROR);
        assertEquals(scaleW, scaleX, ABSOLUTE_ERROR);
        final var zAxisVanishingPoint2 = Point2D.create();
        camera.zAxisVanishingPoint(zAxisVanishingPoint2);
        assertEquals(zAxisVanishingPoint, zAxisVanishingPoint2);

        // set new y-axis vanishing point
        x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        w = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        zAxisVanishingPoint.setHomogeneousCoordinates(x, y, w);
        camera.setZAxisVanishingPoint(zAxisVanishingPoint);
        // when setting z-axis vanishing point, internal matrix will be modified
        internalMatrix = camera.getInternalMatrix();
        scaleX = internalMatrix.getElementAt(0, 2) / zAxisVanishingPoint.getHomX();
        scaleY = internalMatrix.getElementAt(1, 2) / zAxisVanishingPoint.getHomY();
        scaleW = internalMatrix.getElementAt(2, 2) / zAxisVanishingPoint.getHomW();
        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleW, ABSOLUTE_ERROR);
        assertEquals(scaleW, scaleX, ABSOLUTE_ERROR);

        // test get set image of world origin
        final var imageOfWorldOrigin = camera.getImageOfWorldOrigin();
        scaleX = internalMatrix.getElementAt(0, 3) / imageOfWorldOrigin.getHomX();
        scaleY = internalMatrix.getElementAt(1, 3) / imageOfWorldOrigin.getHomY();
        scaleW = internalMatrix.getElementAt(2, 3) / imageOfWorldOrigin.getHomW();
        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleW, ABSOLUTE_ERROR);
        assertEquals(scaleW, scaleX, ABSOLUTE_ERROR);
        final var imageOfWorldOrigin2 = Point2D.create();
        camera.imageOfWorldOrigin(imageOfWorldOrigin2);
        assertEquals(imageOfWorldOrigin, imageOfWorldOrigin2);

        // set new image of world origin
        x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        w = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        imageOfWorldOrigin.setHomogeneousCoordinates(x, y, w);
        camera.setImageOfWorldOrigin(imageOfWorldOrigin);
        // when setting image of world origin, internal matrix will be modified
        internalMatrix = camera.getInternalMatrix();
        scaleX = internalMatrix.getElementAt(0, 3) / imageOfWorldOrigin.getHomX();
        scaleY = internalMatrix.getElementAt(1, 3) / imageOfWorldOrigin.getHomY();
        scaleW = internalMatrix.getElementAt(2, 3) / imageOfWorldOrigin.getHomW();
        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleW, ABSOLUTE_ERROR);
        assertEquals(scaleW, scaleX, ABSOLUTE_ERROR);
    }

    @Test
    void testGetSetHorizontalVerticalAndPrincipalPlanesAndAxis() throws WrongSizeException, CameraException {
        var internalMatrix = Matrix.createWithUniformRandomValues(PINHOLE_CAMERA_ROWS, PINHOLE_CAMERA_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var camera = new PinholeCamera(internalMatrix);

        final var randomizer = new UniformRandomizer();

        // test get set vertical axis plane
        final var verticalAxisPlane = camera.getVerticalAxisPlane();
        var scaleA = internalMatrix.getElementAt(0, 0) / verticalAxisPlane.getA();
        var scaleB = internalMatrix.getElementAt(0, 1) / verticalAxisPlane.getB();
        var scaleC = internalMatrix.getElementAt(0, 2) / verticalAxisPlane.getC();
        var scaleD = internalMatrix.getElementAt(0, 3) / verticalAxisPlane.getD();
        assertEquals(scaleA, scaleB, ABSOLUTE_ERROR);
        assertEquals(scaleB, scaleC, ABSOLUTE_ERROR);
        assertEquals(scaleC, scaleD, ABSOLUTE_ERROR);
        assertEquals(scaleD, scaleA, ABSOLUTE_ERROR);
        final var verticalAxisPlane2 = new Plane();
        camera.verticalAxisPlane(verticalAxisPlane2);
        assertEquals(verticalAxisPlane, verticalAxisPlane2);

        // set new vertical axis plane
        var a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        verticalAxisPlane.setParameters(a, b, c, d);
        camera.setVerticalAxisPlane(verticalAxisPlane);
        // when setting vertical axis plane, internal matrix, which has been
        // passed by reference, is also updated
        scaleA = internalMatrix.getElementAt(0, 0) / verticalAxisPlane.getA();
        scaleB = internalMatrix.getElementAt(0, 1) / verticalAxisPlane.getB();
        scaleC = internalMatrix.getElementAt(0, 2) / verticalAxisPlane.getC();
        scaleD = internalMatrix.getElementAt(0, 3) / verticalAxisPlane.getD();
        assertEquals(scaleA, scaleB, ABSOLUTE_ERROR);
        assertEquals(scaleB, scaleC, ABSOLUTE_ERROR);
        assertEquals(scaleC, scaleD, ABSOLUTE_ERROR);
        assertEquals(scaleD, scaleA, ABSOLUTE_ERROR);

        // test get set horizontal axis plane
        final var horizontalAxisPlane = camera.getHorizontalAxisPlane();
        scaleA = internalMatrix.getElementAt(1, 0) / horizontalAxisPlane.getA();
        scaleB = internalMatrix.getElementAt(1, 1) / horizontalAxisPlane.getB();
        scaleC = internalMatrix.getElementAt(1, 2) / horizontalAxisPlane.getC();
        scaleD = internalMatrix.getElementAt(1, 3) / horizontalAxisPlane.getD();
        assertEquals(scaleA, scaleB, ABSOLUTE_ERROR);
        assertEquals(scaleB, scaleC, ABSOLUTE_ERROR);
        assertEquals(scaleC, scaleD, ABSOLUTE_ERROR);
        assertEquals(scaleD, scaleA, ABSOLUTE_ERROR);
        final var horizontalAxisPlane2 = new Plane();
        camera.horizontalAxisPlane(horizontalAxisPlane2);
        assertEquals(horizontalAxisPlane, horizontalAxisPlane2);

        // set new horizontal axis plane
        a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        horizontalAxisPlane.setParameters(a, b, c, d);
        camera.setHorizontalAxisPlane(horizontalAxisPlane);
        // when setting horizontal axis plane, internal matrix will be modified
        internalMatrix = camera.getInternalMatrix();
        scaleA = internalMatrix.getElementAt(1, 0) / horizontalAxisPlane.getA();
        scaleB = internalMatrix.getElementAt(1, 1) / horizontalAxisPlane.getB();
        scaleC = internalMatrix.getElementAt(1, 2) / horizontalAxisPlane.getC();
        scaleD = internalMatrix.getElementAt(1, 3) / horizontalAxisPlane.getD();
        assertEquals(scaleA, scaleB, ABSOLUTE_ERROR);
        assertEquals(scaleB, scaleC, ABSOLUTE_ERROR);
        assertEquals(scaleC, scaleD, ABSOLUTE_ERROR);
        assertEquals(scaleD, scaleA, ABSOLUTE_ERROR);

        // test get set principal plane
        final var principalPlane = camera.getPrincipalPlane();
        scaleA = internalMatrix.getElementAt(2, 0) / principalPlane.getA();
        scaleB = internalMatrix.getElementAt(2, 1) / principalPlane.getB();
        scaleC = internalMatrix.getElementAt(2, 2) / principalPlane.getC();
        scaleD = internalMatrix.getElementAt(2, 3) / principalPlane.getD();
        assertEquals(scaleA, scaleB, ABSOLUTE_ERROR);
        assertEquals(scaleB, scaleC, ABSOLUTE_ERROR);
        assertEquals(scaleC, scaleD, ABSOLUTE_ERROR);
        assertEquals(scaleD, scaleA, ABSOLUTE_ERROR);
        final var principalPlane2 = new Plane();
        camera.principalPlane(principalPlane2);
        assertEquals(principalPlane, principalPlane2);

        // test that principal axis array is up to scale with principal plane
        // director vector (its first 3 components)
        var principalAxisArray = camera.getPrincipalAxisArray();
        scaleA = principalAxisArray[0] / principalPlane.getA();
        scaleB = principalAxisArray[1] / principalPlane.getB();
        scaleC = principalAxisArray[2] / principalPlane.getC();
        assertEquals(scaleA, scaleB, ABSOLUTE_ERROR);
        assertEquals(scaleB, scaleC, ABSOLUTE_ERROR);
        assertEquals(scaleC, scaleA, ABSOLUTE_ERROR);

        // set new principal plane
        a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        principalPlane.setParameters(a, b, c, d);
        camera.setPrincipalPlane(principalPlane);
        // when setting principal plane, internal matrix will be modified
        internalMatrix = camera.getInternalMatrix();
        scaleA = internalMatrix.getElementAt(2, 0) / principalPlane.getA();
        scaleB = internalMatrix.getElementAt(2, 1) / principalPlane.getB();
        scaleC = internalMatrix.getElementAt(2, 2) / principalPlane.getC();
        scaleD = internalMatrix.getElementAt(2, 3) / principalPlane.getD();
        assertEquals(scaleA, scaleB, ABSOLUTE_ERROR);
        assertEquals(scaleB, scaleC, ABSOLUTE_ERROR);
        assertEquals(scaleC, scaleD, ABSOLUTE_ERROR);
        assertEquals(scaleD, scaleA, ABSOLUTE_ERROR);

        // test principal axis array again
        principalAxisArray = camera.getPrincipalAxisArray();
        scaleA = principalAxisArray[0] / principalPlane.getA();
        scaleB = principalAxisArray[1] / principalPlane.getB();
        scaleC = principalAxisArray[2] / principalPlane.getC();
        assertEquals(scaleA, scaleB, ABSOLUTE_ERROR);
        assertEquals(scaleB, scaleC, ABSOLUTE_ERROR);
        assertEquals(scaleC, scaleA, ABSOLUTE_ERROR);
        final var principalAxisArray2 = new double[3];
        camera.principalAxisArray(principalAxisArray2);
        assertArrayEquals(principalAxisArray, principalAxisArray2, 0.0);
    }

    @Test
    void testGetPrincipalPoint() throws WrongSizeException {
        final var centerMatrix = Matrix.createWithUniformRandomValues(INHOM_3D_COORDS, 1, MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final var cameraCenter = new InhomogeneousPoint3D(
                centerMatrix.getElementAtIndex(0),
                centerMatrix.getElementAtIndex(1),
                centerMatrix.getElementAtIndex(2));

        final var randomizer = new UniformRandomizer();
        final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

        final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

        final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);
        final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

        final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);

        // get principal point
        final var principalPoint = camera.getPrincipalPoint();
        final var principalPoint2 = Point2D.create();
        camera.principalPoint(principalPoint2);

        // and check correctness
        assertEquals(principalPoint.getInhomX(), horizontalPrincipalPoint, ABSOLUTE_ERROR);
        assertEquals(principalPoint.getInhomY(), verticalPrincipalPoint, ABSOLUTE_ERROR);
        assertEquals(principalPoint, principalPoint2);
    }

    @Test
    void testGetAndFixCameraSign() throws WrongSizeException, DecomposerException, CameraException {
        final var internalMatrix = Matrix.createWithUniformRandomValues(PINHOLE_CAMERA_ROWS, PINHOLE_CAMERA_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var mp = internalMatrix.getSubmatrix(0, 0, 2, 2);
        final var detMp = com.irurueta.algebra.Utils.det(mp);
        final var cameraSign = (detMp > 0.0) ? 1.0 : -1.0;

        final var camera = new PinholeCamera(internalMatrix);

        assertEquals(camera.getCameraSign(), cameraSign, ABSOLUTE_ERROR);
        // if we set threshold to determinant value, then camera sign is negative
        assertTrue(camera.getCameraSign(detMp) < 0.0);
        assertFalse(camera.isCameraSignFixed());

        // fix camera sign
        camera.fixCameraSign();
        assertTrue(camera.isCameraSignFixed());

        // now camera sign has to be positive
        assertTrue(camera.getCameraSign() > 0.0);
    }

    @Test
    void testGetDepthCheiralityAndFrontOfCamera() throws WrongSizeException, CameraException, NotReadyException,
            LockedException, DecomposerException, com.irurueta.algebra.NotAvailableException {

        final var centerMatrix = Matrix.createWithUniformRandomValues(INHOM_3D_COORDS, 1, MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final var cameraCenter = new InhomogeneousPoint3D(
                centerMatrix.getElementAtIndex(0),
                centerMatrix.getElementAtIndex(1),
                centerMatrix.getElementAtIndex(2));

        final var randomizer = new UniformRandomizer();
        final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

        final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

        final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        final var positiveDepth = randomizer.nextDouble(MIN_DEPTH, MAX_DEPTH);
        final var negativeDepth = randomizer.nextDouble(-MAX_DEPTH, -MIN_DEPTH);

        final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);
        final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

        final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);

        final var principalAxisArray = camera.getPrincipalAxisArray();

        final var principalPlane = camera.getPrincipalPlane();
        final var principalPlaneMatrix = new Matrix(1, HOM_3D_COORDS);
        principalPlaneMatrix.setElementAtIndex(0, principalPlane.getA());
        principalPlaneMatrix.setElementAtIndex(1, principalPlane.getB());
        principalPlaneMatrix.setElementAtIndex(2, principalPlane.getC());
        principalPlaneMatrix.setElementAtIndex(3, principalPlane.getD());

        // use SVD decomposition of principalPlaneMatrix to find points on the
        // plane as any arbitrary linear combination of the null-space of
        // principalPlaneMatrix
        final var decomposer = new SingularValueDecomposer(principalPlaneMatrix);

        decomposer.decompose();

        final var v = decomposer.getV();

        // first column of V is the principal plane matrix itself. Remaining
        // three columns are three homogeneous world points defining such plane
        // (hence they are its null-space)
        final var planePoint1 = new HomogeneousPoint3D(
                v.getElementAt(0, 1), v.getElementAt(1, 1),
                v.getElementAt(2, 1), v.getElementAt(3, 1));
        final var planePoint2 = new HomogeneousPoint3D(
                v.getElementAt(0, 2), v.getElementAt(1, 2),
                v.getElementAt(2, 2), v.getElementAt(3, 2));
        final var planePoint3 = new HomogeneousPoint3D(
                v.getElementAt(0, 3), v.getElementAt(1, 3),
                v.getElementAt(2, 3), v.getElementAt(3, 3));

        // create three points in front of the camera (with positiveDepth) and
        // three points behind the camera (with negativeDepth) using the three
        // points on the principal plane and adding to their inhomogeneous
        // coordinates some positiveDepth or negativeDepth value on the principal
        // axis direction (which is normalized)
        final var frontPoint1 = new InhomogeneousPoint3D(
                planePoint1.getInhomX() + (positiveDepth * principalAxisArray[0]),
                planePoint1.getInhomY() + (positiveDepth * principalAxisArray[1]),
                planePoint1.getInhomZ() + (positiveDepth * principalAxisArray[2]));
        final var frontPoint2 = new InhomogeneousPoint3D(
                planePoint2.getInhomX() + (positiveDepth * principalAxisArray[0]),
                planePoint2.getInhomY() + (positiveDepth * principalAxisArray[1]),
                planePoint2.getInhomZ() + (positiveDepth * principalAxisArray[2]));
        final var frontPoint3 = new InhomogeneousPoint3D(
                planePoint3.getInhomX() + (positiveDepth * principalAxisArray[0]),
                planePoint3.getInhomY() + (positiveDepth * principalAxisArray[1]),
                planePoint3.getInhomZ() + (positiveDepth * principalAxisArray[2]));

        final var backPoint1 = new InhomogeneousPoint3D(
                planePoint1.getInhomX() + (negativeDepth * principalAxisArray[0]),
                planePoint1.getInhomY() + (negativeDepth * principalAxisArray[1]),
                planePoint1.getInhomZ() + (negativeDepth * principalAxisArray[2]));
        final var backPoint2 = new InhomogeneousPoint3D(
                planePoint2.getInhomX() + (negativeDepth * principalAxisArray[0]),
                planePoint2.getInhomY() + (negativeDepth * principalAxisArray[1]),
                planePoint2.getInhomZ() + (negativeDepth * principalAxisArray[2]));
        final var backPoint3 = new InhomogeneousPoint3D(
                planePoint3.getInhomX() + (negativeDepth * principalAxisArray[0]),
                planePoint3.getInhomY() + (negativeDepth * principalAxisArray[1]),
                planePoint3.getInhomZ() + (negativeDepth * principalAxisArray[2]));

        // now check correctness of depth for front points
        assertEquals(camera.getDepth(frontPoint1), positiveDepth, ABSOLUTE_ERROR);
        assertEquals(camera.getDepth(frontPoint2), positiveDepth, ABSOLUTE_ERROR);
        assertEquals(camera.getDepth(frontPoint3), positiveDepth, ABSOLUTE_ERROR);

        // check that points are indeed in front of the camrea by also checking that their
        // cheirality is positive
        assertTrue(camera.getCheirality(frontPoint1) > 0.0);
        assertTrue(camera.getCheirality(frontPoint2) > 0.0);
        assertTrue(camera.getCheirality(frontPoint3) > 0.0);

        assertTrue(camera.isPointInFrontOfCamera(frontPoint1));
        assertTrue(camera.isPointInFrontOfCamera(frontPoint2));
        assertTrue(camera.isPointInFrontOfCamera(frontPoint3));
        assertTrue(camera.isPointInFrontOfCamera(frontPoint1, 0.0));

        // now for back points
        assertEquals(camera.getDepth(backPoint1), negativeDepth, ABSOLUTE_ERROR);
        assertEquals(camera.getDepth(backPoint2), negativeDepth, ABSOLUTE_ERROR);
        assertEquals(camera.getDepth(backPoint3), negativeDepth, ABSOLUTE_ERROR);

        // check that points are indeed behind by checking also that their
        // cheirality is negative
        assertTrue(camera.getCheirality(backPoint1) < 0.0);
        assertTrue(camera.getCheirality(backPoint2) < 0.0);
        assertTrue(camera.getCheirality(backPoint3) < 0.0);

        assertFalse(camera.isPointInFrontOfCamera(backPoint1));
        assertFalse(camera.isPointInFrontOfCamera(backPoint2));
        assertFalse(camera.isPointInFrontOfCamera(backPoint3));
    }

    @Test
    void testGetDepthsCheiralitiesAndInFrontOfCamera() throws WrongSizeException, CameraException {

        final var randomizer = new UniformRandomizer();
        final var nPoints = randomizer.nextInt(MIN_N_POINTS, MAX_N_POINTS);

        // generate a random camera and a list of random world points, and then
        // compare depth, cheiral and if each point is in front of camera by
        // using the methods that have been previously tested

        // list of random points
        final var worldPointList = new ArrayList<Point3D>(nPoints);
        for (var i = 0; i < nPoints; i++) {
            final var point = new HomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            worldPointList.add(point);
        }
        assertEquals(worldPointList.size(), nPoints);

        // random camera
        final var centerMatrix = Matrix.createWithUniformRandomValues(INHOM_3D_COORDS, 1, MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final var cameraCenter = new InhomogeneousPoint3D(
                centerMatrix.getElementAtIndex(0),
                centerMatrix.getElementAtIndex(1),
                centerMatrix.getElementAtIndex(2));

        final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

        final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

        final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);
        final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

        final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);

        // get depths of all points in the list
        final var depths = camera.getDepths(worldPointList);
        final var depths2 = new ArrayList<Double>(worldPointList.size());
        camera.depths(worldPointList, depths2);
        assertEquals(depths, depths2);

        // get cheiralities for all points in the list
        final var cheiralities = camera.getCheiralities(worldPointList);
        final var fronts = camera.arePointsInFrontOfCamera(worldPointList);
        final var fronts2 = new ArrayList<Boolean>(worldPointList.size());
        camera.arePointsInFrontOfCamera(worldPointList, fronts2);
        final var fronts3 = camera.arePointsInFrontOfCamera(worldPointList, 0.0);
        final var fronts4 = new ArrayList<Boolean>(worldPointList.size());
        camera.arePointsInFrontOfCamera(worldPointList, fronts4, 0.0);

        final var cheiralities2 = new ArrayList<Double>(worldPointList.size());
        camera.cheiralities(worldPointList, cheiralities2);
        assertEquals(cheiralities, cheiralities2);

        // iterate over lists to test each points individually
        final var pointsIterator = worldPointList.iterator();
        final var depthsIterator = depths.iterator();
        final var cheiralitiesIterator = cheiralities.iterator();
        final var frontsIterator = fronts.iterator();
        final var fronts2Iterator = fronts2.iterator();
        while (pointsIterator.hasNext()) {
            final var point = pointsIterator.next();

            final var depth = depthsIterator.next();
            final var cheirality = cheiralitiesIterator.next();
            final var front = frontsIterator.next();
            final var front2 = fronts2Iterator.next();

            final var camDepth = camera.getDepth(point);
            final var camCheirality = camera.getCheirality(point);
            final var camFront = camera.isPointInFrontOfCamera(point);

            assertEquals(depth, camDepth, ABSOLUTE_ERROR);
            assertEquals(cheirality, camCheirality, ABSOLUTE_ERROR);
            assertEquals(front, camFront);
            assertEquals(front2, camFront);
        }

        assertEquals(fronts, fronts2);
        assertEquals(fronts, fronts3);
        assertEquals(fronts, fronts4);
    }

    @Test
    void testCreateCanonicalCamera() throws WrongSizeException {
        final var camera = PinholeCamera.createCanonicalCamera();
        final var internalMatrix = camera.getInternalMatrix();

        // check that internal matrix is the 3x4 identity matrix
        assertTrue(internalMatrix.equals(Matrix.identity(PINHOLE_CAMERA_ROWS, PINHOLE_CAMERA_COLS), ABSOLUTE_ERROR));
    }

    @Test
    void testIsNormalized() throws WrongSizeException, CameraException {
        // test that whenever a parameter is modified in a camera, it becomes non
        // normalized

        final var cameraMatrix = Matrix.createWithUniformRandomValues(PINHOLE_CAMERA_ROWS, PINHOLE_CAMERA_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // Testing isNormalized after setInternalMatrix
        final var camera = new PinholeCamera();
        assertFalse(camera.isNormalized());
        camera.setInternalMatrix(cameraMatrix);
        assertFalse(camera.isNormalized());
        camera.normalize();
        assertTrue(camera.isNormalized());
        camera.setInternalMatrix(cameraMatrix);
        assertFalse(camera.isNormalized());

        // testing isNormalized after setCameraRotation
        final var randomizer = new UniformRandomizer();
        final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);
        assertFalse(camera.isNormalized());
        camera.setCameraRotation(rotation);
        assertFalse(camera.isNormalized());
        camera.normalize();
        assertTrue(camera.isNormalized());
        camera.setCameraRotation(rotation);
        assertFalse(camera.isNormalized());

        // testing isNormalized after setCameraIntrinsicParameters
        final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
        final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        final var k = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

        assertFalse(camera.isNormalized());
        camera.setIntrinsicParameters(k);
        assertFalse(camera.isNormalized());
        camera.normalize();
        assertTrue(camera.isNormalized());
        camera.setIntrinsicParameters(k);
        assertFalse(camera.isNormalized());

        // testing isNormalized after setCameraCenter
        final var cameraCenterArray = new double[INHOM_3D_COORDS];
        randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

        assertFalse(camera.isNormalized());
        camera.setCameraCenter(cameraCenter);
        assertFalse(camera.isNormalized());
        camera.normalize();
        assertTrue(camera.isNormalized());
        camera.setCameraCenter(cameraCenter);
        assertFalse(camera.isNormalized());

        // testing isNormalized after setIntrinsicParametersAndRotation
        assertFalse(camera.isNormalized());
        camera.setIntrinsicParametersAndRotation(k, rotation);
        assertFalse(camera.isNormalized());
        camera.normalize();
        assertTrue(camera.isNormalized());
        camera.setIntrinsicParametersAndRotation(k, rotation);
        assertFalse(camera.isNormalized());

        // testing isNormalized after setIntrinsicAndExtrinsicParameters
        final var worldOriginArray = new double[HOM_2D_COORDS];
        randomizer.fill(worldOriginArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var imageOfWorldOrigin = new HomogeneousPoint2D(worldOriginArray);

        assertFalse(camera.isNormalized());
        camera.setIntrinsicAndExtrinsicParameters(k, rotation, imageOfWorldOrigin);
        assertFalse(camera.isNormalized());
        camera.normalize();
        assertTrue(camera.isNormalized());
        camera.setIntrinsicAndExtrinsicParameters(k, rotation, imageOfWorldOrigin);
        assertFalse(camera.isNormalized());

        // testing setImageOfWorldOrigin
        assertFalse(camera.isNormalized());
        camera.setImageOfWorldOrigin(imageOfWorldOrigin);
        assertFalse(camera.isNormalized());
        camera.normalize();
        assertTrue(camera.isNormalized());
        camera.setImageOfWorldOrigin(imageOfWorldOrigin);
        assertFalse(camera.isNormalized());

        // testing isNormalized after set X Y Z axis vanishing point
        var x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var w = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var xAxisVanishingPoint = new HomogeneousPoint2D(x, y, w);
        x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        w = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var yAxisVanishingPoint = new HomogeneousPoint2D(x, y, w);
        x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        w = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var zAxisVanishingPoint = new HomogeneousPoint2D(x, y, w);

        assertFalse(camera.isNormalized());
        camera.setXAxisVanishingPoint(xAxisVanishingPoint);
        assertFalse(camera.isNormalized());
        camera.normalize();
        assertTrue(camera.isNormalized());
        camera.setXAxisVanishingPoint(xAxisVanishingPoint);
        assertFalse(camera.isNormalized());

        assertFalse(camera.isNormalized());
        camera.setYAxisVanishingPoint(yAxisVanishingPoint);
        assertFalse(camera.isNormalized());
        camera.normalize();
        assertTrue(camera.isNormalized());
        camera.setYAxisVanishingPoint(yAxisVanishingPoint);
        assertFalse(camera.isNormalized());

        assertFalse(camera.isNormalized());
        camera.setZAxisVanishingPoint(zAxisVanishingPoint);
        assertFalse(camera.isNormalized());
        camera.normalize();
        assertTrue(camera.isNormalized());
        camera.setZAxisVanishingPoint(zAxisVanishingPoint);
        assertFalse(camera.isNormalized());

        // testing isNormalized after setting horizontal, vertical and principal plane
        var a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var verticalAxisPlane = new Plane(a, b, c, d);

        a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var horizontalAxisPlane = new Plane(a, b, c, d);

        a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var principalPlane = new Plane(a, b, c, d);

        assertFalse(camera.isNormalized());
        camera.setVerticalAxisPlane(verticalAxisPlane);
        assertFalse(camera.isNormalized());
        camera.normalize();
        assertTrue(camera.isNormalized());
        camera.setVerticalAxisPlane(verticalAxisPlane);
        assertFalse(camera.isNormalized());

        assertFalse(camera.isNormalized());
        camera.setHorizontalAxisPlane(horizontalAxisPlane);
        assertFalse(camera.isNormalized());
        camera.normalize();
        assertTrue(camera.isNormalized());
        camera.setHorizontalAxisPlane(horizontalAxisPlane);
        assertFalse(camera.isNormalized());

        assertFalse(camera.isNormalized());
        camera.setPrincipalPlane(principalPlane);
        assertFalse(camera.isNormalized());
        camera.normalize();
        assertTrue(camera.isNormalized());
        camera.setPrincipalPlane(principalPlane);
        assertFalse(camera.isNormalized());

        // testing isNormalized after rotate
        assertFalse(camera.isNormalized());
        camera.rotate(rotation);
        assertFalse(camera.isNormalized());
        camera.normalize();
        assertTrue(camera.isNormalized());
        camera.rotate(rotation);
        assertFalse(camera.isNormalized());

        // testing isNormalized after pointAt
        final var pointAtArray = new double[INHOM_3D_COORDS];
        randomizer.fill(pointAtArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var pointAt = new InhomogeneousPoint3D(pointAtArray);

        assertFalse(camera.isNormalized());
        camera.pointAt(pointAt);
        assertFalse(camera.isNormalized());
        camera.normalize();
        assertTrue(camera.isNormalized());
        camera.pointAt(pointAt);
        assertFalse(camera.isNormalized());
    }

    @Test
    void testProjectBackProjectDualQuadricAndDualConic() throws DecomposerException, WrongSizeException,
            CoincidentLinesException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            // instantiate dual conic from set of lines
            var m = Matrix.createWithUniformRandomValues(5, HOM_2D_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            var line1 = new Line2D(m.getElementAt(0, 0), m.getElementAt(0, 1),
                    m.getElementAt(0, 2));
            var line2 = new Line2D(m.getElementAt(1, 0), m.getElementAt(1, 1),
                    m.getElementAt(1, 2));
            var line3 = new Line2D(m.getElementAt(2, 0), m.getElementAt(2, 1),
                    m.getElementAt(2, 2));
            var line4 = new Line2D(m.getElementAt(3, 0), m.getElementAt(3, 1),
                    m.getElementAt(3, 2));
            var line5 = new Line2D(m.getElementAt(4, 0), m.getElementAt(4, 1),
                    m.getElementAt(4, 2));

            line1.normalize();
            line2.normalize();
            line3.normalize();
            line4.normalize();
            line5.normalize();

            // estimate dual conic that lies inside provided 5 lines (we need to
            // ensure that line configuration is not degenerate)
            final var m2 = new Matrix(5, 6);

            var l1 = line1.getA();
            var l2 = line1.getB();
            var l3 = line1.getC();
            m2.setElementAt(0, 0, l1 * l1);
            m2.setElementAt(0, 1, 2.0 * l1 * l2);
            m2.setElementAt(0, 2, l2 * l2);
            m2.setElementAt(0, 3, 2.0 * l1 * l3);
            m2.setElementAt(0, 4, 2.0 * l2 * l3);
            m2.setElementAt(0, 5, l3 * l3);

            l1 = line2.getA();
            l2 = line2.getB();
            l3 = line2.getC();
            m2.setElementAt(1, 0, l1 * l1);
            m2.setElementAt(1, 1, 2.0 * l1 * l2);
            m2.setElementAt(1, 2, l2 * l2);
            m2.setElementAt(1, 3, 2.0 * l1 * l3);
            m2.setElementAt(1, 4, 2.0 * l2 * l3);
            m2.setElementAt(1, 5, l3 * l3);

            l1 = line3.getA();
            l2 = line3.getB();
            l3 = line3.getC();
            m2.setElementAt(2, 0, l1 * l1);
            m2.setElementAt(2, 1, 2.0 * l1 * l2);
            m2.setElementAt(2, 2, l2 * l2);
            m2.setElementAt(2, 3, 2.0 * l1 * l3);
            m2.setElementAt(2, 4, 2.0 * l2 * l3);
            m2.setElementAt(2, 5, l3 * l3);

            l1 = line4.getA();
            l2 = line4.getB();
            l3 = line4.getC();
            m2.setElementAt(3, 0, l1 * l1);
            m2.setElementAt(3, 1, 2.0 * l1 * l2);
            m2.setElementAt(3, 2, l2 * l2);
            m2.setElementAt(3, 3, 2.0 * l1 * l3);
            m2.setElementAt(3, 4, 2.0 * l2 * l3);
            m2.setElementAt(3, 5, l3 * l3);

            l1 = line5.getA();
            l2 = line5.getB();
            l3 = line5.getC();
            m2.setElementAt(4, 0, l1 * l1);
            m2.setElementAt(4, 1, 2.0 * l1 * l2);
            m2.setElementAt(4, 2, l2 * l2);
            m2.setElementAt(4, 3, 2.0 * l1 * l3);
            m2.setElementAt(4, 4, 2.0 * l2 * l3);
            m2.setElementAt(4, 5, l3 * l3);

            while (com.irurueta.algebra.Utils.rank(m2) < 5) {
                m = Matrix.createWithUniformRandomValues(5, HOM_2D_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

                line1 = new Line2D(m.getElementAt(0, 0), m.getElementAt(0, 1),
                        m.getElementAt(0, 2));
                line2 = new Line2D(m.getElementAt(1, 0), m.getElementAt(1, 1),
                        m.getElementAt(1, 2));
                line3 = new Line2D(m.getElementAt(2, 0), m.getElementAt(2, 1),
                        m.getElementAt(2, 2));
                line4 = new Line2D(m.getElementAt(3, 0), m.getElementAt(3, 1),
                        m.getElementAt(3, 2));
                line5 = new Line2D(m.getElementAt(4, 0), m.getElementAt(4, 1),
                        m.getElementAt(4, 2));

                line1.normalize();
                line2.normalize();
                line3.normalize();
                line4.normalize();
                line5.normalize();

                l1 = line1.getA();
                l2 = line1.getB();
                l3 = line1.getC();
                m2.setElementAt(0, 0, l1 * l1);
                m2.setElementAt(0, 1, 2.0 * l1 * l2);
                m2.setElementAt(0, 2, l2 * l2);
                m2.setElementAt(0, 3, 2.0 * l1 * l3);
                m2.setElementAt(0, 4, 2.0 * l2 * l3);
                m2.setElementAt(0, 5, l3 * l3);

                l1 = line2.getA();
                l2 = line2.getB();
                l3 = line2.getC();
                m2.setElementAt(1, 0, l1 * l1);
                m2.setElementAt(1, 1, 2.0 * l1 * l2);
                m2.setElementAt(1, 2, l2 * l2);
                m2.setElementAt(1, 3, 2.0 * l1 * l3);
                m2.setElementAt(1, 4, 2.0 * l2 * l3);
                m2.setElementAt(1, 5, l3 * l3);

                l1 = line3.getA();
                l2 = line3.getB();
                l3 = line3.getC();
                m2.setElementAt(2, 0, l1 * l1);
                m2.setElementAt(2, 1, 2.0 * l1 * l2);
                m2.setElementAt(2, 2, l2 * l2);
                m2.setElementAt(2, 3, 2.0 * l1 * l3);
                m2.setElementAt(2, 4, 2.0 * l2 * l3);
                m2.setElementAt(2, 5, l3 * l3);

                l1 = line4.getA();
                l2 = line4.getB();
                l3 = line4.getC();
                m2.setElementAt(3, 0, l1 * l1);
                m2.setElementAt(3, 1, 2.0 * l1 * l2);
                m2.setElementAt(3, 2, l2 * l2);
                m2.setElementAt(3, 3, 2.0 * l1 * l3);
                m2.setElementAt(3, 4, 2.0 * l2 * l3);
                m2.setElementAt(3, 5, l3 * l3);

                l1 = line5.getA();
                l2 = line5.getB();
                l3 = line5.getC();
                m2.setElementAt(4, 0, l1 * l1);
                m2.setElementAt(4, 1, 2.0 * l1 * l2);
                m2.setElementAt(4, 2, l2 * l2);
                m2.setElementAt(4, 3, 2.0 * l1 * l3);
                m2.setElementAt(4, 4, 2.0 * l2 * l3);
                m2.setElementAt(4, 5, l3 * l3);
            }

            final var dualConic = new DualConic(line1, line2, line3, line4, line5);

            // check that lines are locus of dual conic
            assertTrue(dualConic.isLocus(line1, ABSOLUTE_ERROR));
            assertTrue(dualConic.isLocus(line2, ABSOLUTE_ERROR));
            assertTrue(dualConic.isLocus(line3, ABSOLUTE_ERROR));
            assertTrue(dualConic.isLocus(line4, ABSOLUTE_ERROR));
            assertTrue(dualConic.isLocus(line5, ABSOLUTE_ERROR));

            // instantiate random camera
            final var cameraMatrix = Matrix.createWithUniformRandomValues(PINHOLE_CAMERA_ROWS, PINHOLE_CAMERA_COLS,
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var camera = new PinholeCamera(cameraMatrix);

            try {
                // back-project dual conic into dual quadric
                final var dualQuadric = camera.backProject(dualConic);
                final var dualQuadric2 = new DualQuadric();
                camera.backProject(dualConic, dualQuadric2);

                assertEquals(dualQuadric.asMatrix(), dualQuadric2.asMatrix());

                // back-project planes of dual conic
                final var plane1 = camera.backProject(line1);
                final var plane2 = camera.backProject(line2);
                final var plane3 = camera.backProject(line3);
                final var plane4 = camera.backProject(line4);
                final var plane5 = camera.backProject(line5);

                // check that back-projected planes are locus of back-projected dual quadric
                assertTrue(dualQuadric.isLocus(plane1));
                assertTrue(dualQuadric.isLocus(plane2));
                assertTrue(dualQuadric.isLocus(plane3));
                assertTrue(dualQuadric.isLocus(plane4));
                assertTrue(dualQuadric.isLocus(plane5));

                // project dual quadric into dual conic
                final var dualConic2 = camera.project(dualQuadric);
                final var dualConic3 = new DualConic();
                camera.project(dualQuadric, dualConic3);

                assertEquals(dualConic2.asMatrix(), dualConic3.asMatrix());

                // check that lines are locus of projected dual conic
                assertTrue(dualConic2.isLocus(line1));
                assertTrue(dualConic2.isLocus(line2));
                assertTrue(dualConic2.isLocus(line3));
                assertTrue(dualConic2.isLocus(line4));
                assertTrue(dualConic2.isLocus(line5));
            } catch (final CameraException e) {
                continue;
            }

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testProjectBackProjectQuadricAndConic() throws WrongSizeException, DecomposerException,
            CoincidentPointsException, CameraException {
        // create conic
        var m = Matrix.createWithUniformRandomValues(5, HOM_2D_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        var point1 = new HomogeneousPoint2D(
                m.getElementAt(0, 0), m.getElementAt(0, 1), m.getElementAt(0, 2));
        var point2 = new HomogeneousPoint2D(
                m.getElementAt(1, 0), m.getElementAt(1, 1), m.getElementAt(1, 2));
        var point3 = new HomogeneousPoint2D(
                m.getElementAt(2, 0), m.getElementAt(2, 1), m.getElementAt(2, 2));
        var point4 = new HomogeneousPoint2D(
                m.getElementAt(3, 0), m.getElementAt(3, 1), m.getElementAt(3, 2));
        var point5 = new HomogeneousPoint2D(
                m.getElementAt(4, 0), m.getElementAt(4, 1), m.getElementAt(4, 2));

        // estimate conic that lies inside provided 5 homogeneous 2D
        // points
        var conicMatrix = new Matrix(5, 6);
        var x = point1.getHomX();
        var y = point1.getHomY();
        var w = point1.getHomW();
        conicMatrix.setElementAt(0, 0, x * x);
        conicMatrix.setElementAt(0, 1, x * y);
        conicMatrix.setElementAt(0, 2, y * y);
        conicMatrix.setElementAt(0, 3, x * w);
        conicMatrix.setElementAt(0, 4, y * w);
        conicMatrix.setElementAt(0, 5, w * w);
        x = point2.getHomX();
        y = point2.getHomY();
        w = point2.getHomW();
        conicMatrix.setElementAt(1, 0, x * x);
        conicMatrix.setElementAt(1, 1, x * y);
        conicMatrix.setElementAt(1, 2, y * y);
        conicMatrix.setElementAt(1, 3, x * w);
        conicMatrix.setElementAt(1, 4, y * w);
        conicMatrix.setElementAt(1, 5, w * w);
        x = point3.getHomX();
        y = point3.getHomY();
        w = point3.getHomW();
        conicMatrix.setElementAt(2, 0, x * x);
        conicMatrix.setElementAt(2, 1, x * y);
        conicMatrix.setElementAt(2, 2, y * y);
        conicMatrix.setElementAt(2, 3, x * w);
        conicMatrix.setElementAt(2, 4, y * w);
        conicMatrix.setElementAt(2, 5, w * w);
        x = point4.getHomX();
        y = point4.getHomY();
        w = point4.getHomW();
        conicMatrix.setElementAt(3, 0, x * x);
        conicMatrix.setElementAt(3, 1, x * y);
        conicMatrix.setElementAt(3, 2, y * y);
        conicMatrix.setElementAt(3, 3, x * w);
        conicMatrix.setElementAt(3, 4, y * w);
        conicMatrix.setElementAt(3, 5, w * w);
        x = point5.getHomX();
        y = point5.getHomY();
        w = point5.getHomW();
        conicMatrix.setElementAt(4, 0, x * x);
        conicMatrix.setElementAt(4, 1, x * y);
        conicMatrix.setElementAt(4, 2, y * y);
        conicMatrix.setElementAt(4, 3, x * w);
        conicMatrix.setElementAt(4, 4, y * w);
        conicMatrix.setElementAt(4, 5, w * w);

        while (com.irurueta.algebra.Utils.rank(conicMatrix) < 5) {
            m = Matrix.createWithUniformRandomValues(5, HOM_2D_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            point1 = new HomogeneousPoint2D(
                    m.getElementAt(0, 0), m.getElementAt(0, 1),
                    m.getElementAt(0, 2));
            point2 = new HomogeneousPoint2D(
                    m.getElementAt(1, 0), m.getElementAt(1, 1),
                    m.getElementAt(1, 2));
            point3 = new HomogeneousPoint2D(
                    m.getElementAt(2, 0), m.getElementAt(2, 1),
                    m.getElementAt(2, 2));
            point4 = new HomogeneousPoint2D(
                    m.getElementAt(3, 0), m.getElementAt(3, 1),
                    m.getElementAt(3, 2));
            point5 = new HomogeneousPoint2D(
                    m.getElementAt(4, 0), m.getElementAt(4, 1),
                    m.getElementAt(4, 2));

            conicMatrix = new Matrix(5, 6);
            x = point1.getHomX();
            y = point1.getHomY();
            w = point1.getHomW();
            conicMatrix.setElementAt(0, 0, x * x);
            conicMatrix.setElementAt(0, 1, x * y);
            conicMatrix.setElementAt(0, 2, y * y);
            conicMatrix.setElementAt(0, 3, x * w);
            conicMatrix.setElementAt(0, 4, y * w);
            conicMatrix.setElementAt(0, 5, w * w);
            x = point2.getHomX();
            y = point2.getHomY();
            w = point2.getHomW();
            conicMatrix.setElementAt(1, 0, x * x);
            conicMatrix.setElementAt(1, 1, x * y);
            conicMatrix.setElementAt(1, 2, y * y);
            conicMatrix.setElementAt(1, 3, x * w);
            conicMatrix.setElementAt(1, 4, y * w);
            conicMatrix.setElementAt(1, 5, w * w);
            x = point3.getHomX();
            y = point3.getHomY();
            w = point3.getHomW();
            conicMatrix.setElementAt(2, 0, x * x);
            conicMatrix.setElementAt(2, 1, x * y);
            conicMatrix.setElementAt(2, 2, y * y);
            conicMatrix.setElementAt(2, 3, x * w);
            conicMatrix.setElementAt(2, 4, y * w);
            conicMatrix.setElementAt(2, 5, w * w);
            x = point4.getHomX();
            y = point4.getHomY();
            w = point4.getHomW();
            conicMatrix.setElementAt(3, 0, x * x);
            conicMatrix.setElementAt(3, 1, x * y);
            conicMatrix.setElementAt(3, 2, y * y);
            conicMatrix.setElementAt(3, 3, x * w);
            conicMatrix.setElementAt(3, 4, y * w);
            conicMatrix.setElementAt(3, 5, w * w);
            x = point5.getHomX();
            y = point5.getHomY();
            w = point5.getHomW();
            conicMatrix.setElementAt(4, 0, x * x);
            conicMatrix.setElementAt(4, 1, x * y);
            conicMatrix.setElementAt(4, 2, y * y);
            conicMatrix.setElementAt(4, 3, x * w);
            conicMatrix.setElementAt(4, 4, y * w);
            conicMatrix.setElementAt(4, 5, w * w);
        }

        final var conic = new Conic(point1, point2, point3, point4, point5);

        // check that points are locus of conic
        assertTrue(conic.isLocus(point1, ABSOLUTE_ERROR));
        assertTrue(conic.isLocus(point2, ABSOLUTE_ERROR));
        assertTrue(conic.isLocus(point3, ABSOLUTE_ERROR));
        assertTrue(conic.isLocus(point4, ABSOLUTE_ERROR));
        assertTrue(conic.isLocus(point5, ABSOLUTE_ERROR));

        // instantiate random camera
        final var cameraMatrix = Matrix.createWithUniformRandomValues(PINHOLE_CAMERA_ROWS, PINHOLE_CAMERA_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var camera = new PinholeCamera(cameraMatrix);

        // back-project conic into quadric
        final var quadric = camera.backProject(conic);
        final var quadric2 = new Quadric();
        camera.backProject(conic, quadric2);

        assertEquals(quadric.asMatrix(), quadric2.asMatrix());

        // back-project planes of dual conic
        final var point1b = camera.backProject(point1);
        final var point2b = camera.backProject(point2);
        final var point3b = camera.backProject(point3);
        final var point4b = camera.backProject(point4);
        final var point5b = camera.backProject(point5);

        // check that back-projected points are locus of back-projected quadric
        assertTrue(quadric.isLocus(point1b, ABSOLUTE_ERROR));
        assertTrue(quadric.isLocus(point2b, ABSOLUTE_ERROR));
        assertTrue(quadric.isLocus(point3b, ABSOLUTE_ERROR));
        assertTrue(quadric.isLocus(point4b, ABSOLUTE_ERROR));
        assertTrue(quadric.isLocus(point5b, ABSOLUTE_ERROR));

        // project quadric into conic
        final var conic2 = camera.project(quadric);
        final var conic3 = new Conic();
        camera.project(quadric, conic3);

        conic.normalize();
        conic2.normalize();
        conic3.normalize();

        assertEquals(conic2.asMatrix(), conic3.asMatrix());
    }

    @Test
    void testSetFromPointCorrespondences() throws CameraException, WrongListSizesException,
            com.irurueta.geometry.estimators.LockedException, com.irurueta.geometry.estimators.NotReadyException,
            PinholeCameraEstimatorException, NotAvailableException, RotationException {
        final var randomizer = new UniformRandomizer();

        // create intrinsic parameters
        final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH2, MAX_FOCAL_LENGTH2);
        final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH2, MAX_FOCAL_LENGTH2);
        final var skewness = randomizer.nextDouble(MIN_SKEWNESS2, MAX_SKEWNESS2);
        final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

        // create rotation parameters
        final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES2, MAX_ANGLE_DEGREES2));
        final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES2, MAX_ANGLE_DEGREES2));
        final var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES2, MAX_ANGLE_DEGREES2));

        final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

        // create camera center
        final var cameraCenterArray = new double[INHOM_3D_COORDS];
        randomizer.fill(cameraCenterArray, MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE);
        final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

        // instantiate camera
        final var camera1 = new PinholeCamera(intrinsic, rotation, cameraCenter);

        // normalize the camera to improve accuracy
        camera1.normalize();

        // create 6 point correspondences
        final var point3D1 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE));
        var point3D2 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE));
        final var point3D3 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE));
        final var point3D4 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE));
        final var point3D5 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE));
        final var point3D6 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE));

        final var point2D1 = camera1.project(point3D1);
        var point2D2 = camera1.project(point3D2);
        final var point2D3 = camera1.project(point3D3);
        final var point2D4 = camera1.project(point3D4);
        final var point2D5 = camera1.project(point3D5);
        final var point2D6 = camera1.project(point3D6);

        final var points3D = new ArrayList<Point3D>(N_POINTS);
        points3D.add(point3D1);
        points3D.add(point3D2);
        points3D.add(point3D3);
        points3D.add(point3D4);
        points3D.add(point3D5);
        points3D.add(point3D6);

        final var points2D = camera1.project(points3D);

        final var camera2 = new PinholeCamera();
        camera2.setFromPointCorrespondences(point3D1, point3D2, point3D3, point3D4, point3D5, point3D6, point2D1,
                point2D2, point2D3, point2D4, point2D5, point2D6);
        camera2.decompose();

        final var estimator = new DLTPointCorrespondencePinholeCameraEstimator(points3D, points2D);
        estimator.setLMSESolutionAllowed(false);
        final var camera3 = estimator.estimate();
        camera3.decompose();

        final var intrinsic2 = camera2.getIntrinsicParameters();
        final var intrinsic3 = camera3.getIntrinsicParameters();

        assertEquals(intrinsic.getHorizontalFocalLength(), intrinsic2.getHorizontalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getVerticalFocalLength(), intrinsic2.getVerticalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getSkewness(), intrinsic2.getSkewness(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getHorizontalPrincipalPoint(), intrinsic2.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getVerticalPrincipalPoint(), intrinsic2.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

        assertEquals(intrinsic2.getHorizontalFocalLength(), intrinsic3.getHorizontalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic2.getVerticalFocalLength(), intrinsic3.getVerticalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic2.getSkewness(), intrinsic3.getSkewness(), ABSOLUTE_ERROR);
        assertEquals(intrinsic2.getHorizontalPrincipalPoint(), intrinsic3.getHorizontalPrincipalPoint(),
                ABSOLUTE_ERROR);
        assertEquals(intrinsic2.getVerticalPrincipalPoint(), intrinsic3.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

        final var rotation2 = camera2.getCameraRotation();
        final var rotation3 = camera3.getCameraRotation();

        assertTrue(rotation2.equals(rotation3, ABSOLUTE_ERROR));

        final var cameraCenter2 = camera2.getCameraCenter();
        final var cameraCenter3 = camera3.getCameraCenter();

        assertTrue(cameraCenter2.equals(cameraCenter3, ABSOLUTE_ERROR));
        assertTrue(cameraCenter2.equals(cameraCenter, ABSOLUTE_ERROR));

        // Force CameraException by repeating correspondences and creating a
        // degeneracy
        point3D2 = point3D1;
        point2D2 = camera1.project(point3D2);
        final var wrongPoint3D2 = point3D2;
        final var wrongPoint2D2 = point2D2;
        assertThrows(CameraException.class, () -> camera2.setFromPointCorrespondences(point3D1, wrongPoint3D2, point3D3,
                point3D4, point3D5, point3D6, point2D1, wrongPoint2D2, point2D3, point2D4, point2D5, point2D6));
    }

    @Test
    void testSetFromLineAndPlaneCorrespondences() throws CameraException, WrongListSizesException,
            com.irurueta.geometry.estimators.LockedException, com.irurueta.geometry.estimators.NotReadyException,
            PinholeCameraEstimatorException, NotAvailableException, RotationException {
        final var randomizer = new UniformRandomizer();

        // create intrinsic parameters
        final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH2, MAX_FOCAL_LENGTH2);
        final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH2, MAX_FOCAL_LENGTH2);
        final var skewness = randomizer.nextDouble(MIN_SKEWNESS2, MAX_SKEWNESS2);
        final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

        // create rotation parameters
        final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES2, MAX_ANGLE_DEGREES2));
        final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES2, MAX_ANGLE_DEGREES2));
        final var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES2, MAX_ANGLE_DEGREES2));

        final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

        // create camera center
        final var cameraCenterArray = new double[INHOM_3D_COORDS];
        randomizer.fill(cameraCenterArray, MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE);
        final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

        // instantiate camera
        final var camera1 = new PinholeCamera(intrinsic, rotation, cameraCenter);

        // normalize the camera to improve accuracy
        camera1.normalize();

        // create 4 2D lines
        final var line2D1 = new Line2D(randomizer.nextDouble(MIN_RANDOM_LINE_VALUE, MAX_RANDOM_LINE_VALUE),
                randomizer.nextDouble(MIN_RANDOM_LINE_VALUE, MAX_RANDOM_LINE_VALUE),
                randomizer.nextDouble(MIN_RANDOM_LINE_VALUE, MAX_RANDOM_LINE_VALUE));
        var line2D2 = new Line2D(randomizer.nextDouble(MIN_RANDOM_LINE_VALUE, MAX_RANDOM_LINE_VALUE),
                randomizer.nextDouble(MIN_RANDOM_LINE_VALUE, MAX_RANDOM_LINE_VALUE),
                randomizer.nextDouble(MIN_RANDOM_LINE_VALUE, MAX_RANDOM_LINE_VALUE));
        final var line2D3 = new Line2D(randomizer.nextDouble(MIN_RANDOM_LINE_VALUE, MAX_RANDOM_LINE_VALUE),
                randomizer.nextDouble(MIN_RANDOM_LINE_VALUE, MAX_RANDOM_LINE_VALUE),
                randomizer.nextDouble(MIN_RANDOM_LINE_VALUE, MAX_RANDOM_LINE_VALUE));
        final var line2D4 = new Line2D(randomizer.nextDouble(MIN_RANDOM_LINE_VALUE, MAX_RANDOM_LINE_VALUE),
                randomizer.nextDouble(MIN_RANDOM_LINE_VALUE, MAX_RANDOM_LINE_VALUE),
                randomizer.nextDouble(MIN_RANDOM_LINE_VALUE, MAX_RANDOM_LINE_VALUE));

        final var plane1 = camera1.backProject(line2D1);
        var plane2 = camera1.backProject(line2D2);
        final var plane3 = camera1.backProject(line2D3);
        final var plane4 = camera1.backProject(line2D4);

        final var lines2D = new ArrayList<Line2D>(N_CORRESPONDENCES);
        lines2D.add(line2D1);
        lines2D.add(line2D2);
        lines2D.add(line2D3);
        lines2D.add(line2D4);

        final var planes = camera1.backProjectLines(lines2D);
        final var planes2 = new ArrayList<Plane>();
        camera1.backProjectLines(lines2D, planes2);

        assertEquals(planes, planes2);

        final var camera2 = new PinholeCamera();
        camera2.setFromLineAndPlaneCorrespondences(plane1, plane2, plane3, plane4, line2D1, line2D2, line2D3, line2D4);
        camera2.decompose();

        final var estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator(planes, lines2D);
        estimator.setLMSESolutionAllowed(false);
        final var camera3 = estimator.estimate();
        camera3.decompose();

        final var intrinsic2 = camera2.getIntrinsicParameters();
        final var intrinsic3 = camera3.getIntrinsicParameters();

        assertEquals(intrinsic2.getHorizontalFocalLength(), intrinsic3.getHorizontalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic2.getVerticalFocalLength(), intrinsic3.getVerticalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic2.getSkewness(), intrinsic3.getSkewness(), ABSOLUTE_ERROR);
        assertEquals(intrinsic2.getHorizontalPrincipalPoint(), intrinsic3.getHorizontalPrincipalPoint(),
                ABSOLUTE_ERROR);
        assertEquals(intrinsic2.getVerticalPrincipalPoint(), intrinsic3.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

        final var rotation2 = camera2.getCameraRotation();
        final var rotation3 = camera3.getCameraRotation();

        assertTrue(rotation2.equals(rotation3, ABSOLUTE_ERROR));

        final var cameraCenter2 = camera2.getCameraCenter();
        final var cameraCenter3 = camera3.getCameraCenter();

        assertTrue(cameraCenter2.equals(cameraCenter3, ABSOLUTE_ERROR));

        // Force CameraException by repeating correspondences and creating a
        // degeneracy
        line2D2 = line2D1;
        plane2 = camera1.backProject(line2D2);
        final var wrongPlane2 = plane2;
        assertThrows(CameraException.class, () -> camera2.setFromLineAndPlaneCorrespondences(plane1, wrongPlane2,
                plane3, plane4, line2D1, line2D4, line2D3, line2D4));
    }

    @Test
    void testSerializeDeserialize() throws CameraException, IOException, ClassNotFoundException {
        // create camera
        final var randomizer = new UniformRandomizer();
        var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
        var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        // rotation
        var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

        final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

        // camera center
        var cameraCenterArray = new double[INHOM_3D_COORDS];
        randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

        // test constructor with intrinsic parameters, rotation and image or
        // origin
        final var camera1 = new PinholeCamera(intrinsic, rotation, cameraCenter);
        camera1.decompose();

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(camera1);
        final var camera2 = SerializationHelper.<PinholeCamera>deserialize(bytes);

        // check
        assertEquals(camera1.getInternalMatrix(), camera2.getInternalMatrix());
    }
}
