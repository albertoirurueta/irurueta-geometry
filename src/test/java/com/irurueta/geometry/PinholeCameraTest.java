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
import org.junit.Test;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class PinholeCameraTest {

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
    public void testConstants() {
        assertEquals(3, PinholeCamera.PINHOLE_CAMERA_MATRIX_ROWS);
        assertEquals(4, PinholeCamera.PINHOLE_CAMERA_MATRIX_COLS);
        assertEquals(3, PinholeCamera.INHOM_COORDS);
        assertEquals(1e-12, PinholeCamera.EPS, 0.0);
    }

    @Test
    public void testConstructors() throws WrongSizeException, RotationException,
            CameraException, NotAvailableException, WrongListSizesException,
            com.irurueta.geometry.estimators.LockedException,
            com.irurueta.geometry.estimators.NotReadyException,
            PinholeCameraEstimatorException {

        // test default constructor
        PinholeCamera camera = new PinholeCamera();

        // test type
        assertEquals(camera.getType(), CameraType.PINHOLE_CAMERA);

        // test that internal matrix is the 3x4 identity, which corresponds to
        // canonical camera located at origin of coordinates, pointing towards
        // z-axis and with retinal plane located at Z = 1 (unitary focal length)
        assertEquals(camera.getInternalMatrix(), Matrix.identity(
                PINHOLE_CAMERA_ROWS, PINHOLE_CAMERA_COLS));

        // test constructor by providing internal matrix
        Matrix cameraMatrix = Matrix.createWithUniformRandomValues(
                PINHOLE_CAMERA_ROWS, PINHOLE_CAMERA_COLS, MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        camera = new PinholeCamera(cameraMatrix);
        assertEquals(camera.getInternalMatrix(), cameraMatrix);

        // Force WrongSizeException
        cameraMatrix = Matrix.createWithUniformRandomValues(
                PINHOLE_CAMERA_ROWS + 1, PINHOLE_CAMERA_COLS + 1,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        camera = null;
        try {
            camera = new PinholeCamera(cameraMatrix);
            fail("WrongSizeException expected but not thrown");
        } catch (final WrongSizeException ignore) {
        }
        assertNull(camera);

        // create intrinsic parameters
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                MAX_FOCAL_LENGTH);
        double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                MAX_FOCAL_LENGTH);
        double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
        double horizontalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        double verticalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        // rotation
        double alphaEuler = randomizer.nextDouble(
                MIN_ANGLE_DEGREES * Math.PI / 180.0,
                MAX_ANGLE_DEGREES * Math.PI / 180.0);
        double betaEuler = randomizer.nextDouble(
                MIN_ANGLE_DEGREES * Math.PI / 180.0,
                MAX_ANGLE_DEGREES * Math.PI / 180.0);
        double gammaEuler = randomizer.nextDouble(
                MIN_ANGLE_DEGREES * Math.PI / 180.0,
                MAX_ANGLE_DEGREES * Math.PI / 180.0);

        PinholeCameraIntrinsicParameters intrinsic =
                new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                        verticalFocalLength, horizontalPrincipalPoint,
                        verticalPrincipalPoint, skewness);
        final Matrix intrinsicMatrix = intrinsic.getInternalMatrix();

        MatrixRotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler,
                gammaEuler);
        final Matrix rotationMatrix = rotation.getInternalMatrix();

        final double[] axis = rotation.getRotationAxis();
        final double theta = rotation.getRotationAngle();

        // image of world origin
        final double[] originImageArray = new double[INHOM_2D_COORDS];
        randomizer.fill(originImageArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final InhomogeneousPoint2D originImage = new InhomogeneousPoint2D(
                originImageArray);

        // camera center
        double[] cameraCenterArray = new double[INHOM_3D_COORDS];
        randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                cameraCenterArray);

        // test constructor with intrinsic parameters, rotation and image or
        // origin
        camera = new PinholeCamera(intrinsic, rotation, originImage);
        assertFalse(camera.isCameraCenterAvailable());
        assertTrue(camera.isCameraRotationAvailable());
        assertTrue(camera.areIntrinsicParametersAvailable());

        // test type
        assertEquals(camera.getType(), CameraType.PINHOLE_CAMERA);

        // build internal matrix
        Matrix cameraMatrix2 = new Matrix(PINHOLE_CAMERA_ROWS,
                PINHOLE_CAMERA_COLS);
        Matrix mp = intrinsicMatrix.multiplyAndReturnNew(rotationMatrix);
        for (int v = 0; v < PINHOLE_CAMERA_ROWS; v++) {
            for (int u = 0; u < PINHOLE_CAMERA_ROWS; u++) {
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

        Rotation3D rotation2 = camera.getCameraRotation();
        PinholeCameraIntrinsicParameters intrinsic2 =
                camera.getIntrinsicParameters();
        final Point2D originImage2 = camera.getImageOfWorldOrigin();

        // Obtain rotation axis and angle from 2nd rotation
        double[] axis2 = rotation2.getRotationAxis();
        double theta2 = rotation2.getRotationAngle();

        // check correctness of axis and angle

        // axis can be equal up to sign
        double scaleAxisX = axis[0] / axis2[0];
        double scaleAxisY = axis[1] / axis2[1];
        double scaleAxisZ = axis[2] / axis2[2];

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
        assertEquals(intrinsic2.getHorizontalFocalLength(),
                horizontalFocalLength, ABSOLUTE_ERROR);
        assertEquals(intrinsic2.getVerticalFocalLength(), verticalFocalLength,
                ABSOLUTE_ERROR);
        assertEquals(intrinsic2.getSkewness(), skewness, ABSOLUTE_ERROR);
        assertEquals(intrinsic2.getHorizontalPrincipalPoint(),
                horizontalPrincipalPoint, ABSOLUTE_ERROR);
        assertEquals(intrinsic2.getVerticalPrincipalPoint(),
                verticalPrincipalPoint, ABSOLUTE_ERROR);

        // compare images of origin
        assertTrue(originImage.equals(originImage2, ABSOLUTE_ERROR));

        // test constructor with intrinsic parameters, rotation and camera center
        camera = new PinholeCamera(intrinsic, rotation, cameraCenter);
        assertTrue(camera.isCameraCenterAvailable());
        assertTrue(camera.isCameraRotationAvailable());
        assertTrue(camera.areIntrinsicParametersAvailable());

        // test type
        assertEquals(camera.getType(), CameraType.PINHOLE_CAMERA);

        // build internal matrix
        cameraMatrix2 = new Matrix(PINHOLE_CAMERA_ROWS, PINHOLE_CAMERA_COLS);
        mp = intrinsicMatrix.multiplyAndReturnNew(rotationMatrix);
        for (int v = 0; v < PINHOLE_CAMERA_ROWS; v++) {
            for (int u = 0; u < PINHOLE_CAMERA_ROWS; u++) {
                cameraMatrix2.setElementAt(u, v, mp.getElementAt(u, v));
            }
        }

        final Matrix inhomCenter = new Matrix(PINHOLE_CAMERA_ROWS, 1);
        inhomCenter.setElementAtIndex(0, cameraCenter.getInhomX());
        inhomCenter.setElementAtIndex(1, cameraCenter.getInhomY());
        inhomCenter.setElementAtIndex(2, cameraCenter.getInhomZ());
        final Matrix p4 = mp.multiplyAndReturnNew(inhomCenter).
                multiplyByScalarAndReturnNew(-1.0);

        // set last column of camera matrix with homogeneous coordinates of image
        // of world origin
        cameraMatrix2.setElementAt(0, 3, p4.getElementAtIndex(0));
        cameraMatrix2.setElementAt(1, 3, p4.getElementAtIndex(1));
        cameraMatrix2.setElementAt(2, 3, p4.getElementAtIndex(2));

        cameraMatrix = camera.getInternalMatrix();

        // test that camera center is the null-space of camera matrix
        final Matrix homCenter = new Matrix(PINHOLE_CAMERA_COLS, 1);
        HomogeneousPoint3D homCameraCenter = new HomogeneousPoint3D(
                cameraCenter);
        homCameraCenter.normalize();
        homCenter.setElementAtIndex(0, homCameraCenter.getHomX());
        homCenter.setElementAtIndex(1, homCameraCenter.getHomY());
        homCenter.setElementAtIndex(2, homCameraCenter.getHomZ());
        homCenter.setElementAtIndex(3, homCameraCenter.getHomW());
        Matrix nullMatrix = cameraMatrix.multiplyAndReturnNew(homCenter);
        assertEquals(nullMatrix.getElementAtIndex(0), 0.0, ABSOLUTE_ERROR);
        assertEquals(nullMatrix.getElementAtIndex(1), 0.0, ABSOLUTE_ERROR);
        assertEquals(nullMatrix.getElementAtIndex(2), 0.0, ABSOLUTE_ERROR);

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
        Point3D cameraCenter2 = camera.getCameraCenter();

        homCameraCenter = new HomogeneousPoint3D(cameraCenter2);
        homCameraCenter.normalize();
        homCenter.setElementAtIndex(0, homCameraCenter.getHomX());
        homCenter.setElementAtIndex(1, homCameraCenter.getHomY());
        homCenter.setElementAtIndex(2, homCameraCenter.getHomZ());
        homCenter.setElementAtIndex(3, homCameraCenter.getHomW());
        nullMatrix = cameraMatrix.multiplyAndReturnNew(homCenter);
        assertEquals(nullMatrix.getElementAtIndex(0), 0.0, ABSOLUTE_ERROR);
        assertEquals(nullMatrix.getElementAtIndex(1), 0.0, ABSOLUTE_ERROR);
        assertEquals(nullMatrix.getElementAtIndex(2), 0.0, ABSOLUTE_ERROR);

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
        assertEquals(intrinsic2.getHorizontalFocalLength(),
                horizontalFocalLength, ABSOLUTE_ERROR);
        assertEquals(intrinsic2.getVerticalFocalLength(),
                verticalFocalLength, ABSOLUTE_ERROR);
        assertEquals(intrinsic2.getSkewness(), skewness, ABSOLUTE_ERROR);
        assertEquals(intrinsic2.getHorizontalPrincipalPoint(),
                horizontalPrincipalPoint, ABSOLUTE_ERROR);
        assertEquals(intrinsic2.getVerticalPrincipalPoint(),
                verticalPrincipalPoint, ABSOLUTE_ERROR);

        // compare camera centers
        assertTrue(cameraCenter.equals(cameraCenter2, LARGE_ABSOLUTE_ERROR));

        // test constructor with point correspondences

        // create intrinsic parameters
        horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH2,
                MAX_FOCAL_LENGTH2);
        verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH2,
                MAX_FOCAL_LENGTH2);
        skewness = randomizer.nextDouble(MIN_SKEWNESS2, MAX_SKEWNESS2);
        horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT,
                MAX_PRINCIPAL_POINT);
        verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT,
                MAX_PRINCIPAL_POINT);

        intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                verticalFocalLength, horizontalPrincipalPoint,
                verticalPrincipalPoint, skewness);

        // create rotation parameters
        alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES2,
                MAX_ANGLE_DEGREES2) * Math.PI / 180.0;
        betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES2,
                MAX_ANGLE_DEGREES2) * Math.PI / 180.0;
        gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES2,
                MAX_ANGLE_DEGREES2) * Math.PI / 180.0;

        rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

        // create camera center
        cameraCenterArray = new double[INHOM_3D_COORDS];
        randomizer.fill(cameraCenterArray, MIN_RANDOM_POINT_VALUE,
                MAX_RANDOM_POINT_VALUE);
        cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

        // instantiate camera
        camera = new PinholeCamera(intrinsic, rotation, cameraCenter);

        // normalize the camera to improve accuracy
        camera.normalize();

        // create 6 point correspondences
        final Point3D point3D1 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE,
                        MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE,
                        MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE,
                        MAX_RANDOM_POINT_VALUE));
        Point3D point3D2 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE,
                        MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE,
                        MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE,
                        MAX_RANDOM_POINT_VALUE));
        final Point3D point3D3 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE,
                        MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE,
                        MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE,
                        MAX_RANDOM_POINT_VALUE));
        final Point3D point3D4 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE,
                        MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE,
                        MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE,
                        MAX_RANDOM_POINT_VALUE));
        final Point3D point3D5 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE,
                        MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE,
                        MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE,
                        MAX_RANDOM_POINT_VALUE));
        final Point3D point3D6 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE,
                        MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE,
                        MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE,
                        MAX_RANDOM_POINT_VALUE));

        final Point2D point2D1 = camera.project(point3D1);
        Point2D point2D2 = camera.project(point3D2);
        final Point2D point2D3 = camera.project(point3D3);
        final Point2D point2D4 = camera.project(point3D4);
        final Point2D point2D5 = camera.project(point3D5);
        final Point2D point2D6 = camera.project(point3D6);

        final List<Point3D> points3D = new ArrayList<>(N_POINTS);
        points3D.add(point3D1);
        points3D.add(point3D2);
        points3D.add(point3D3);
        points3D.add(point3D4);
        points3D.add(point3D5);
        points3D.add(point3D6);

        final List<Point2D> points2D = camera.project(points3D);

        camera = new PinholeCamera(point3D1, point3D2, point3D3, point3D4,
                point3D5, point3D6, point2D1, point2D2, point2D3, point2D4,
                point2D5, point2D6);
        final DLTPointCorrespondencePinholeCameraEstimator estimator =
                new DLTPointCorrespondencePinholeCameraEstimator(points3D,
                        points2D);
        estimator.setLMSESolutionAllowed(false);
        PinholeCamera estimatedCamera = estimator.estimate();

        // check equal-ness of cameras
        camera.decompose();
        estimatedCamera.decompose();

        intrinsic2 = camera.getIntrinsicParameters();
        PinholeCameraIntrinsicParameters intrinsic3 =
                estimatedCamera.getIntrinsicParameters();

        assertEquals(intrinsic2.getHorizontalFocalLength(),
                intrinsic3.getHorizontalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic2.getVerticalFocalLength(),
                intrinsic3.getVerticalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic2.getSkewness(), intrinsic3.getSkewness(),
                ABSOLUTE_ERROR);
        assertEquals(intrinsic2.getHorizontalPrincipalPoint(),
                intrinsic3.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
        assertEquals(intrinsic2.getVerticalPrincipalPoint(),
                intrinsic3.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

        rotation2 = camera.getCameraRotation();
        Rotation3D rotation3 = estimatedCamera.getCameraRotation();

        assertTrue(rotation2.equals(rotation3, ABSOLUTE_ERROR));

        cameraCenter2 = camera.getCameraCenter();
        Point3D cameraCenter3 = estimatedCamera.getCameraCenter();

        assertTrue(cameraCenter2.equals(cameraCenter3, ABSOLUTE_ERROR));

        // Force CameraException by repeating correspondences and creating a
        // degeneracy
        point3D2 = point3D1;
        point2D2 = camera.project(point3D2);
        camera = null;
        try {
            camera = new PinholeCamera(point3D1, point3D2, point3D3, point3D4,
                    point3D5, point3D6, point2D1, point2D2, point2D3, point2D4,
                    point2D5, point2D6);
            fail("CameraException expected but not thrown");
        } catch (final CameraException ignore) {
        }
        assertNull(camera);


        // constructor with line/plane correspondence

        // instantiate camera
        camera = new PinholeCamera(intrinsic, rotation, cameraCenter);

        // create 4 2D lines
        final Line2D line2D1 = new Line2D(randomizer.nextDouble(MIN_RANDOM_LINE_VALUE,
                MAX_RANDOM_LINE_VALUE),
                randomizer.nextDouble(MIN_RANDOM_LINE_VALUE,
                        MAX_RANDOM_LINE_VALUE),
                randomizer.nextDouble(MIN_RANDOM_LINE_VALUE,
                        MAX_RANDOM_LINE_VALUE));
        Line2D line2D2 = new Line2D(randomizer.nextDouble(MIN_RANDOM_LINE_VALUE,
                MAX_RANDOM_LINE_VALUE),
                randomizer.nextDouble(MIN_RANDOM_LINE_VALUE,
                        MAX_RANDOM_LINE_VALUE),
                randomizer.nextDouble(MIN_RANDOM_LINE_VALUE,
                        MAX_RANDOM_LINE_VALUE));
        final Line2D line2D3 = new Line2D(randomizer.nextDouble(MIN_RANDOM_LINE_VALUE,
                MAX_RANDOM_LINE_VALUE),
                randomizer.nextDouble(MIN_RANDOM_LINE_VALUE,
                        MAX_RANDOM_LINE_VALUE),
                randomizer.nextDouble(MIN_RANDOM_LINE_VALUE,
                        MAX_RANDOM_LINE_VALUE));
        final Line2D line2D4 = new Line2D(randomizer.nextDouble(MIN_RANDOM_LINE_VALUE,
                MAX_RANDOM_LINE_VALUE),
                randomizer.nextDouble(MIN_RANDOM_LINE_VALUE,
                        MAX_RANDOM_LINE_VALUE),
                randomizer.nextDouble(MIN_RANDOM_LINE_VALUE,
                        MAX_RANDOM_LINE_VALUE));

        final Plane plane1 = camera.backProject(line2D1);
        Plane plane2 = camera.backProject(line2D2);
        final Plane plane3 = camera.backProject(line2D3);
        final Plane plane4 = camera.backProject(line2D4);

        final List<Line2D> lines2D = new ArrayList<>(N_CORRESPONDENCES);
        lines2D.add(line2D1);
        lines2D.add(line2D2);
        lines2D.add(line2D3);
        lines2D.add(line2D4);

        final List<Plane> planes = camera.backProjectLines(lines2D);

        camera = new PinholeCamera(plane1, plane2, plane3, plane4,
                line2D1, line2D2, line2D3, line2D4);
        final DLTLinePlaneCorrespondencePinholeCameraEstimator estimator2 =
                new DLTLinePlaneCorrespondencePinholeCameraEstimator(planes,
                        lines2D);
        estimator2.setLMSESolutionAllowed(false);
        estimatedCamera = estimator2.estimate();

        // check equal-ness of cameras
        camera.decompose();
        estimatedCamera.decompose();

        intrinsic2 = camera.getIntrinsicParameters();
        intrinsic3 = estimatedCamera.getIntrinsicParameters();

        assertEquals(intrinsic2.getHorizontalFocalLength(),
                intrinsic3.getHorizontalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic2.getVerticalFocalLength(),
                intrinsic3.getVerticalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic2.getSkewness(), intrinsic3.getSkewness(),
                ABSOLUTE_ERROR);
        assertEquals(intrinsic2.getHorizontalPrincipalPoint(),
                intrinsic3.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
        assertEquals(intrinsic2.getVerticalPrincipalPoint(),
                intrinsic3.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

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
        camera = null;
        try {
            camera = new PinholeCamera(plane1, plane2, plane3, plane4,
                    line2D1, line2D4, line2D3, line2D4);
            fail("CameraException expected but not thrown");
        } catch (final CameraException ignore) {
        }
        assertNull(camera);
    }

    @Test
    public void testProjectPoints() throws WrongSizeException {
        // create intrinsic parameters
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                MAX_FOCAL_LENGTH);
        final double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                MAX_FOCAL_LENGTH);
        final double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

        final double horizontalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final double verticalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        // rotation
        final double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        final PinholeCameraIntrinsicParameters intrinsic =
                new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                        verticalFocalLength, horizontalPrincipalPoint,
                        verticalPrincipalPoint, skewness);

        final Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler,
                gammaEuler);

        // camera center
        final double[] cameraCenterArray = new double[INHOM_3D_COORDS];
        randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                cameraCenterArray);

        // instantiate camera
        final PinholeCamera camera = new PinholeCamera(intrinsic, rotation,
                cameraCenter);

        final Matrix internalMatrix = camera.getInternalMatrix();

        assertEquals(internalMatrix.getRows(), PINHOLE_CAMERA_ROWS);
        assertEquals(internalMatrix.getColumns(), PINHOLE_CAMERA_COLS);

        // project list of world points
        final int nPoints = randomizer.nextInt(MIN_NUMBER_POINTS, MAX_NUMBER_POINTS);

        final List<Point3D> worldPointList = new ArrayList<>(nPoints);
        final Matrix worldPointListMatrix = new Matrix(nPoints,
                Point3D.POINT3D_HOMOGENEOUS_COORDINATES_LENGTH);
        for (int i = 0; i < nPoints; i++) {
            final double[] pointArray = new double[
                    Point3D.POINT3D_HOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(pointArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final Point3D point = new HomogeneousPoint3D(pointArray);
            worldPointList.add(point);
            worldPointListMatrix.setSubmatrix(i, 0, i,
                    Point3D.POINT3D_HOMOGENEOUS_COORDINATES_LENGTH - 1,
                    pointArray);
        }

        final Matrix transWorldPointListMatrix =
                worldPointListMatrix.transposeAndReturnNew();

        final Matrix transImagePointListMatrix =
                internalMatrix.multiplyAndReturnNew(transWorldPointListMatrix);

        final Matrix imagePointListMatrix =
                transImagePointListMatrix.transposeAndReturnNew();

        // list of image points projected by ourselves
        final List<Point2D> imagePointList = new ArrayList<>(nPoints);
        for (int i = 0; i < nPoints; i++) {
            final double[] pointArray = new double[
                    Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH];
            imagePointListMatrix.getSubmatrixAsArray(i, 0, i,
                    Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH - 1,
                    pointArray);
            final Point2D point = new HomogeneousPoint2D(pointArray);
            imagePointList.add(point);
        }

        // list of image points to be tested
        final List<Point2D> imagePointList2 = camera.project(worldPointList);
        final List<Point2D> imagePointList3 = new ArrayList<>();
        camera.project(worldPointList, imagePointList3);

        assertEquals(imagePointList2.size(), nPoints);
        assertEquals(imagePointList.size(), nPoints);

        final Iterator<Point2D> iterator = imagePointList.iterator();
        final Iterator<Point2D> iterator2 = imagePointList2.iterator();
        final Iterator<Point3D> iteratorWorld = worldPointList.iterator();

        Point2D imagePoint;
        Point2D imagePoint2;
        while (iterator.hasNext() && iterator2.hasNext() &&
                iteratorWorld.hasNext()) {
            imagePoint = iterator.next();
            imagePoint2 = iterator2.next();

            assertTrue(imagePoint.equals(imagePoint2, ABSOLUTE_ERROR));
        }

        assertEquals(imagePointList2, imagePointList3);

        // project single world point
        final double[] homWorldPointArray = new double[
                Point3D.POINT3D_HOMOGENEOUS_COORDINATES_LENGTH];
        randomizer.fill(homWorldPointArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        HomogeneousPoint3D homWorldPoint = new HomogeneousPoint3D(
                homWorldPointArray);

        imagePoint = camera.project(homWorldPoint);

        Matrix homImagePointMatrix = internalMatrix.multiplyAndReturnNew(
                Matrix.newFromArray(homWorldPointArray, true));

        double scaleX = imagePoint.getHomX() /
                homImagePointMatrix.getElementAtIndex(0);
        double scaleY = imagePoint.getHomY() /
                homImagePointMatrix.getElementAtIndex(1);
        double scaleW = imagePoint.getHomW() /
                homImagePointMatrix.getElementAtIndex(2);

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

        scaleX = imagePoint.getHomX() /
                homImagePointMatrix.getElementAtIndex(0);
        scaleY = imagePoint.getHomY() /
                homImagePointMatrix.getElementAtIndex(1);
        scaleW = imagePoint.getHomW() /
                homImagePointMatrix.getElementAtIndex(2);

        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleW, ABSOLUTE_ERROR);
        assertEquals(scaleW, scaleX, ABSOLUTE_ERROR);
    }

    @Test
    public void testBackProjectLines() throws WrongSizeException,
            NotReadyException, LockedException, DecomposerException,
            com.irurueta.algebra.NotAvailableException {

        // create intrinsic parameters
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                MAX_FOCAL_LENGTH);
        final double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                MAX_FOCAL_LENGTH);
        final double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

        final double horizontalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final double verticalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        // rotation
        final double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        final PinholeCameraIntrinsicParameters intrinsic =
                new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                        verticalFocalLength, horizontalPrincipalPoint,
                        verticalPrincipalPoint, skewness);

        final Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler,
                gammaEuler);

        // camera center
        final double[] cameraCenterArray = new double[INHOM_3D_COORDS];
        randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                cameraCenterArray);

        // instantiate camera
        final PinholeCamera camera = new PinholeCamera(intrinsic, rotation,
                cameraCenter);

        final Matrix cameraMatrix = camera.getInternalMatrix();

        // instantiate random line to back-project
        final double[] lineArray = new double[HOM_2D_COORDS];
        randomizer.fill(lineArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final Matrix lineMatrix = Matrix.newFromArray(lineArray, true);

        final Line2D line = new Line2D(lineArray);

        final Matrix transCameraMatrix = cameraMatrix.transposeAndReturnNew();

        final Matrix planeMatrix = transCameraMatrix.multiplyAndReturnNew(
                lineMatrix);

        Plane plane = camera.backProject(line);
        final Plane plane2 = new Plane();
        camera.backProject(line, plane2);

        double scaleA = plane.getA() / planeMatrix.getElementAtIndex(0);
        double scaleB = plane.getB() / planeMatrix.getElementAtIndex(1);
        double scaleC = plane.getC() / planeMatrix.getElementAtIndex(2);
        double scaleD = plane.getD() / planeMatrix.getElementAtIndex(3);

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
        final Matrix a = planeMatrix.transposeAndReturnNew();
        final SingularValueDecomposer decomposer = new SingularValueDecomposer(a);
        decomposer.decompose();

        // a row matrix 1x4 will have rank 1 and nullity 3 (1 + 3 = 4)
        final Matrix v = decomposer.getV();
        assertEquals(decomposer.getRank(), 1);
        assertEquals(decomposer.getNullity(), 3);

        // the last three columns of V contain the null-space of a.
        // These three columns correspond to homogeneous 3D point coordinates
        // that belong to back-projected plane. Any linear-combination of such
        // three 3D points will also belong to such plane
        final HomogeneousPoint3D worldPoint1 = new HomogeneousPoint3D(
                v.getElementAt(0, 1), v.getElementAt(1, 1),
                v.getElementAt(2, 1), v.getElementAt(3, 1));
        final HomogeneousPoint3D worldPoint2 = new HomogeneousPoint3D(
                v.getElementAt(0, 2), v.getElementAt(1, 2),
                v.getElementAt(2, 2), v.getElementAt(3, 2));
        final HomogeneousPoint3D worldPoint3 = new HomogeneousPoint3D(
                v.getElementAt(0, 3), v.getElementAt(1, 3),
                v.getElementAt(2, 3), v.getElementAt(3, 3));

        // check that world points above belong to back-projected plane
        assertTrue(plane.isLocus(worldPoint1, ABSOLUTE_ERROR));
        assertTrue(plane.isLocus(worldPoint2, ABSOLUTE_ERROR));
        assertTrue(plane.isLocus(worldPoint3, ABSOLUTE_ERROR));

        // project world points
        final Point2D imagePoint1 = camera.project(worldPoint1);
        final Point2D imagePoint2 = camera.project(worldPoint2);
        final Point2D imagePoint3 = camera.project(worldPoint3);

        // check that projected image points belong to line
        assertTrue(line.isLocus(imagePoint1, ABSOLUTE_ERROR));
        assertTrue(line.isLocus(imagePoint2, ABSOLUTE_ERROR));
        assertTrue(line.isLocus(imagePoint3, ABSOLUTE_ERROR));
    }

    @Test
    public void testBackprojectPoints() throws CameraException {
        // create intrinsic parameters
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                MAX_FOCAL_LENGTH);
        final double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                MAX_FOCAL_LENGTH);
        final double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

        final double horizontalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final double verticalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        // rotation
        final double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        final PinholeCameraIntrinsicParameters intrinsic =
                new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                        verticalFocalLength, horizontalPrincipalPoint,
                        verticalPrincipalPoint, skewness);

        final Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler,
                gammaEuler);

        // camera center
        final double[] cameraCenterArray = new double[INHOM_3D_COORDS];
        randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                cameraCenterArray);

        // instantiate camera
        final PinholeCamera camera = new PinholeCamera(intrinsic, rotation,
                cameraCenter);

        // instantiate random 3D point
        final double[] worldPointArray = new double[HOM_3D_COORDS];
        randomizer.fill(worldPointArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final HomogeneousPoint3D worldPoint = new HomogeneousPoint3D(worldPointArray);

        // project it
        Point2D imagePoint = camera.project(worldPoint);

        // test that any point on such ray of light at any random distance will
        // project on the same image point
        final Point3D rayOfLight = camera.backProject(imagePoint);
        final Point3D rayOfLight2 = Point3D.create();
        camera.backProject(imagePoint, rayOfLight2);

        Point2D imagePoint2 = camera.project(rayOfLight);

        assertTrue(imagePoint.equals(imagePoint2, ABSOLUTE_ERROR));
        assertEquals(rayOfLight, rayOfLight2);

        // now make a list of world points
        final int nPoints = randomizer.nextInt(MIN_N_POINTS, MAX_N_POINTS);

        final List<Point3D> worldPointList = new ArrayList<>(nPoints);
        for (int i = 0; i < nPoints; i++) {
            final HomogeneousPoint3D point = new HomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            worldPointList.add(point);
        }

        // project list of world points into image points
        final List<Point2D> imagePointList = camera.project(worldPointList);

        // back project image points to obtain a list of world points containing
        // rays of light
        final List<Point3D> worldPointList2 = camera.backProjectPoints(
                imagePointList);
        final List<Point3D> worldPointList3 = new ArrayList<>();
        camera.backProjectPoints(imagePointList, worldPointList3);

        // project again to ensure that projected rays of light produce the same
        // image points
        final List<Point2D> imagePointList2 = camera.project(worldPointList2);
        final List<Point2D> imagePointList3 = camera.project(worldPointList3);

        // iterate over image lists and check that both are equal
        final Iterator<Point2D> iterator1 = imagePointList.iterator();
        final Iterator<Point2D> iterator2 = imagePointList2.iterator();
        final Iterator<Point2D> iterator3 = imagePointList3.iterator();

        while (iterator1.hasNext() && iterator2.hasNext() && iterator3.hasNext()) {
            imagePoint = iterator1.next();
            imagePoint2 = iterator2.next();

            assertTrue(imagePoint.equals(imagePoint2, ABSOLUTE_ERROR));
            imagePoint2 = iterator3.next();
            assertTrue(imagePoint.equals(imagePoint2, ABSOLUTE_ERROR));
        }
    }

    @Test
    public void testDecomposeGetCameraRotationIntrinsicsCenterAndImageOfWorldOrigin()
            throws RotationException, WrongSizeException, CameraException,
            NotAvailableException {

        // create intrinsic parameters
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                MAX_FOCAL_LENGTH);
        final double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                MAX_FOCAL_LENGTH);
        final double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

        final double horizontalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final double verticalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        // rotation
        final double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        final PinholeCameraIntrinsicParameters intrinsic =
                new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                        verticalFocalLength, horizontalPrincipalPoint,
                        verticalPrincipalPoint, skewness);

        final Matrix intrinsicMatrix = intrinsic.getInternalMatrix();

        final Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler,
                gammaEuler);
        final Matrix rotationMatrix = rotation.asInhomogeneousMatrix();

        final double[] axis = rotation.getRotationAxis();
        final double theta = rotation.getRotationAngle();

        // image of world origin
        final double[] originImageArray = new double[INHOM_2D_COORDS];
        randomizer.fill(originImageArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final InhomogeneousPoint2D originImage = new InhomogeneousPoint2D(
                originImageArray);

        // camera center
        final double[] cameraCenterArray = new double[INHOM_3D_COORDS];
        final InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                cameraCenterArray);

        // test constructor with intrinsic parameters, rotation and image of
        // origin
        PinholeCamera camera = new PinholeCamera(intrinsic, rotation,
                originImage);
        assertFalse(camera.isCameraCenterAvailable());
        assertTrue(camera.isCameraRotationAvailable());
        assertTrue(camera.areIntrinsicParametersAvailable());

        // build internal camera matrix
        Matrix cameraMatrix2 = new Matrix(PINHOLE_CAMERA_ROWS,
                PINHOLE_CAMERA_COLS);
        Matrix mp = intrinsicMatrix.multiplyAndReturnNew(rotationMatrix);
        for (int v = 0; v < PINHOLE_CAMERA_ROWS; v++) {
            for (int u = 0; u < PINHOLE_CAMERA_ROWS; u++) {
                cameraMatrix2.setElementAt(u, v, mp.getElementAt(u, v));
            }
        }

        // set last column of camera matrix with homogeneous coordinates of image
        // of world origin
        cameraMatrix2.setElementAt(0, 3, originImage.getHomX());
        cameraMatrix2.setElementAt(1, 3, originImage.getHomY());
        cameraMatrix2.setElementAt(2, 3, originImage.getHomW());

        Matrix cameraMatrix = camera.getInternalMatrix();

        // compare matrices cameraMatrix and cameraMatrix2
        assertTrue(cameraMatrix.equals(cameraMatrix2, ABSOLUTE_ERROR));

        // decompose created camera and check that obtained intrinsic parameters,
        // rotation and image of world origin are correct
        camera.decompose(true, true);

        assertTrue(camera.isCameraCenterAvailable());
        assertTrue(camera.isCameraRotationAvailable());
        assertTrue(camera.areIntrinsicParametersAvailable());

        Rotation3D rotation2 = camera.getCameraRotation();
        PinholeCameraIntrinsicParameters intrinsic2 =
                camera.getIntrinsicParameters();
        final Point2D originImage2 = camera.getImageOfWorldOrigin();

        // obtain rotation axis and angle from 2nd rotation
        double[] axis2 = rotation2.getRotationAxis();
        double theta2 = rotation2.getRotationAngle();

        // check correctness of axis and angle

        // axis can be equal up to sign
        double scaleAxisX = axis[0] / axis2[0];
        double scaleAxisY = axis[1] / axis2[1];
        double scaleAxisZ = axis[2] / axis2[2];

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
        assertEquals(intrinsic2.getHorizontalFocalLength(),
                horizontalFocalLength, ABSOLUTE_ERROR);
        assertEquals(intrinsic2.getVerticalFocalLength(),
                verticalFocalLength, ABSOLUTE_ERROR);
        assertEquals(intrinsic2.getSkewness(), skewness, ABSOLUTE_ERROR);
        assertEquals(intrinsic2.getHorizontalPrincipalPoint(),
                horizontalPrincipalPoint, ABSOLUTE_ERROR);
        assertEquals(intrinsic2.getVerticalPrincipalPoint(),
                verticalPrincipalPoint, ABSOLUTE_ERROR);

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
        for (int v = 0; v < PINHOLE_CAMERA_ROWS; v++) {
            for (int u = 0; u < PINHOLE_CAMERA_ROWS; u++) {
                cameraMatrix2.setElementAt(u, v, mp.getElementAt(u, v));
            }
        }

        Matrix inhomCenter = new Matrix(PINHOLE_CAMERA_ROWS, 1);
        inhomCenter.setElementAtIndex(0, cameraCenter.getInhomX());
        inhomCenter.setElementAtIndex(1, cameraCenter.getInhomY());
        inhomCenter.setElementAtIndex(2, cameraCenter.getInhomZ());
        final Matrix p4 = mp.multiplyAndReturnNew(inhomCenter).
                multiplyByScalarAndReturnNew(-1.0);

        // set last column of camera matrix with homogeneous coordinates of image
        // of world origin
        cameraMatrix.setElementAt(0, 3, p4.getElementAtIndex(0));
        cameraMatrix.setElementAt(1, 3, p4.getElementAtIndex(1));
        cameraMatrix.setElementAt(2, 3, p4.getElementAtIndex(2));

        cameraMatrix = camera.getInternalMatrix();

        // test that camera center is the null-space of camera matrix
        final Matrix homCenter = new Matrix(PINHOLE_CAMERA_COLS, 1);
        HomogeneousPoint3D homCameraCenter = new HomogeneousPoint3D(
                cameraCenter);
        homCameraCenter.normalize();
        homCenter.setElementAtIndex(0, homCameraCenter.getHomX());
        homCenter.setElementAtIndex(1, homCameraCenter.getHomY());
        homCenter.setElementAtIndex(2, homCameraCenter.getHomZ());
        homCenter.setElementAtIndex(3, homCameraCenter.getHomW());
        Matrix nullMatrix = cameraMatrix.multiplyAndReturnNew(homCenter);
        assertEquals(nullMatrix.getElementAtIndex(0), 0.0, ABSOLUTE_ERROR);
        assertEquals(nullMatrix.getElementAtIndex(1), 0.0, ABSOLUTE_ERROR);
        assertEquals(nullMatrix.getElementAtIndex(2), 0.0, ABSOLUTE_ERROR);

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
        final Point3D cameraCenter2 = camera.getCameraCenter();

        homCameraCenter = new HomogeneousPoint3D(cameraCenter2);
        homCameraCenter.normalize();
        homCenter.setElementAtIndex(0, homCameraCenter.getHomX());
        homCenter.setElementAtIndex(1, homCameraCenter.getHomY());
        homCenter.setElementAtIndex(2, homCameraCenter.getHomZ());
        homCenter.setElementAtIndex(3, homCameraCenter.getHomW());
        nullMatrix = cameraMatrix.multiplyAndReturnNew(homCenter);
        assertEquals(nullMatrix.getElementAtIndex(0), 0.0, ABSOLUTE_ERROR);
        assertEquals(nullMatrix.getElementAtIndex(1), 0.0, ABSOLUTE_ERROR);
        assertEquals(nullMatrix.getElementAtIndex(2), 0.0, ABSOLUTE_ERROR);

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
            assertEquals(theta, theta2, ABSOLUTE_ERROR);
        } else {
            assertEquals(theta, -theta2, ABSOLUTE_ERROR);
        }

        // compare intrinsic parameters
        assertEquals(intrinsic2.getHorizontalFocalLength(),
                horizontalFocalLength, ABSOLUTE_ERROR);
        assertEquals(intrinsic2.getVerticalFocalLength(),
                verticalFocalLength, ABSOLUTE_ERROR);
        assertEquals(intrinsic2.getSkewness(), skewness, ABSOLUTE_ERROR);
        assertEquals(intrinsic2.getHorizontalPrincipalPoint(),
                horizontalPrincipalPoint, ABSOLUTE_ERROR);
        assertEquals(intrinsic2.getVerticalPrincipalPoint(),
                verticalPrincipalPoint, ABSOLUTE_ERROR);

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
    public void testNormalize() throws WrongSizeException {
        final Matrix cameraMatrix = Matrix.createWithUniformRandomValues(
                PINHOLE_CAMERA_ROWS, PINHOLE_CAMERA_COLS, MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final PinholeCamera camera = new PinholeCamera(cameraMatrix);

        final double norm = Math.sqrt(
                Math.pow(cameraMatrix.getElementAt(2, 0), 2.0) +
                        Math.pow(cameraMatrix.getElementAt(2, 1), 2.0) +
                        Math.pow(cameraMatrix.getElementAt(2, 2), 2.0));

        // normalize and copy
        final Matrix cameraMatrix2 = cameraMatrix.multiplyByScalarAndReturnNew(
                1.0 / norm);

        // normalize camera which also normalizes internal matrix passed by
        // reference
        camera.normalize();

        // compare both matrices
        assertTrue(cameraMatrix.equals(cameraMatrix2, ABSOLUTE_ERROR));
    }

    @Test
    public void testCameraSign() throws WrongSizeException, DecomposerException,
            CameraException {

        final Matrix cameraMatrix = Matrix.createWithUniformRandomValues(
                PINHOLE_CAMERA_ROWS, PINHOLE_CAMERA_COLS, MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final PinholeCamera camera = new PinholeCamera(cameraMatrix);

        // obtain camera sign and check correctness
        final Matrix mp = cameraMatrix.getSubmatrix(0, 0,
                2, 2);

        final double detMp = com.irurueta.algebra.Utils.det(mp);

        final double cameraSign = (detMp > 0.0) ? 1.0 : -1.0;
        final double cameraSign2 = camera.getCameraSign();

        assertEquals(cameraSign, cameraSign2, ABSOLUTE_ERROR);

        // set threshold equal to detMp
        final double cameraSign3 = camera.getCameraSign(detMp);
        assertEquals(cameraSign3, -1.0, ABSOLUTE_ERROR);

        // camera hasn't fixed sign yet
        assertFalse(camera.isCameraSignFixed());

        // fix camera sign
        camera.fixCameraSign();
        assertTrue(camera.isCameraSignFixed());
        assertTrue(camera.getCameraSign() > 0.0);
    }

    @Test
    public void testGetSetInternalMatrix() throws WrongSizeException {
        final Matrix cameraMatrix = Matrix.createWithUniformRandomValues(
                PINHOLE_CAMERA_ROWS, PINHOLE_CAMERA_COLS, MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final PinholeCamera camera = new PinholeCamera(cameraMatrix);
        assertTrue(cameraMatrix.equals(camera.getInternalMatrix(),
                ABSOLUTE_ERROR));

        // set new camera matrix
        final Matrix cameraMatrix2 = Matrix.createWithUniformRandomValues(
                PINHOLE_CAMERA_ROWS, PINHOLE_CAMERA_COLS, MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        camera.setInternalMatrix(cameraMatrix2);
        assertTrue(cameraMatrix2.equals(camera.getInternalMatrix(),
                ABSOLUTE_ERROR));
    }

    @Test
    public void testGetSetCameraRotation() throws CameraException, NotAvailableException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        final Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler,
                gammaEuler);

        // instantiate canonical camera
        final PinholeCamera camera = new PinholeCamera();
        assertFalse(camera.isCameraRotationAvailable());

        // Force NotAvailableException
        try {
            camera.getCameraRotation();
            fail("NotAvailableException expected but not thrown");
        } catch (final NotAvailableException ignore) {
        }

        // set new rotation
        camera.setCameraRotation(rotation);
        assertTrue(camera.isCameraRotationAvailable());

        // check correctness of rotation
        assertEquals(rotation, camera.getCameraRotation());

        // decompose
        camera.decompose();
        assertTrue(camera.isCameraRotationAvailable());

        // retrieve new rotation instance
        final Rotation3D rotation2 = camera.getCameraRotation();

        // compare euler angles
        assertTrue(rotation2.asInhomogeneousMatrix().equals(
                rotation.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
    }

    @Test
    public void testGetSetIntrinsicsAndAvailability() throws CameraException,
            NotAvailableException {
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                MAX_FOCAL_LENGTH);
        final double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                MAX_FOCAL_LENGTH);

        final double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

        final double horizontalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final double verticalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        final PinholeCameraIntrinsicParameters k =
                new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                        verticalFocalLength, horizontalPrincipalPoint,
                        verticalPrincipalPoint, skewness);

        // instantiate canonical camera
        final PinholeCamera camera = new PinholeCamera();
        assertFalse(camera.areIntrinsicParametersAvailable());

        // Force NotAvailableException
        try {
            camera.getIntrinsicParameters();
            fail("NotAvailableException expected but not thrown");
        } catch (final NotAvailableException ignore) {
        }

        // set new intrinsic parameters
        camera.setIntrinsicParameters(k);
        assertTrue(camera.areIntrinsicParametersAvailable());

        // check correctness of intrinsic parameters
        assertEquals(k, camera.getIntrinsicParameters());

        // decompose
        camera.decompose();
        assertTrue(camera.areIntrinsicParametersAvailable());

        // retrieve new intrinsics instance
        final PinholeCameraIntrinsicParameters k2 = camera.getIntrinsicParameters();

        // compare intrinsic parameters
        assertEquals(k2.getHorizontalFocalLength(), horizontalFocalLength,
                ABSOLUTE_ERROR);
        assertEquals(k2.getVerticalFocalLength(), verticalFocalLength,
                ABSOLUTE_ERROR);
        assertEquals(k2.getSkewness(), skewness, ABSOLUTE_ERROR);
        assertEquals(k2.getHorizontalPrincipalPoint(),
                horizontalPrincipalPoint, ABSOLUTE_ERROR);
        assertEquals(k2.getVerticalPrincipalPoint(),
                verticalPrincipalPoint, ABSOLUTE_ERROR);
    }

    @Test
    public void testRotate() throws WrongSizeException, CameraException,
            NotAvailableException, RotationException {

        final Matrix axisMatrix = Matrix.createWithUniformRandomValues(
                INHOM_3D_COORDS, 1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double norm = com.irurueta.algebra.Utils.normF(axisMatrix);
        axisMatrix.multiplyByScalar(1.0 / norm);

        final double[] axisArray = axisMatrix.toArray();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta1 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double theta2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        final Rotation3D rotation1 = new MatrixRotation3D(axisArray, theta1);
        final Rotation3D rotation2 = new MatrixRotation3D(axisArray, theta2);

        // instantiate canonical camera
        final PinholeCamera camera = new PinholeCamera();

        // set initial rotation with computed axis and rotation angle theta1
        camera.setCameraRotation(rotation1);

        // from current rotation, rotate more using the same axis and theta2
        camera.rotate(rotation2);
        assertTrue(camera.isCameraRotationAvailable());

        // obtain current rotation
        final Rotation3D rotation3 = camera.getCameraRotation();

        // obtain current axis and rotation angle
        final double[] axis3 = rotation3.getRotationAxis();
        final double theta3 = rotation3.getRotationAngle();

        // check that rotation axis is equal up to sign
        final double scaleX = axisArray[0] / axis3[0];
        final double scaleY = axisArray[1] / axis3[1];
        final double scaleZ = axisArray[2] / axis3[2];

        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleZ, ABSOLUTE_ERROR);
        assertEquals(scaleZ, scaleX, ABSOLUTE_ERROR);

        if (scaleX > 0.0) {
            // rotation angle remains equal
            assertEquals(theta3, theta1 + theta2, ABSOLUTE_ERROR);
        } else {
            // rotation axis has opposite sign, and so does rotation angle
            assertEquals(theta3, -(theta1 + theta2), ABSOLUTE_ERROR);
        }
    }

    @Test
    public void testPointAt() throws WrongSizeException, CameraException,
            NotAvailableException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Matrix centerMatrix = Matrix.createWithUniformRandomValues(
                    INHOM_3D_COORDS, 1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    centerMatrix.getElementAtIndex(0),
                    centerMatrix.getElementAtIndex(1),
                    centerMatrix.getElementAtIndex(2));

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final double horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                    MAX_FOCAL_LENGTH);
            final double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                    MAX_FOCAL_LENGTH);
            final double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

            final double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler,
                    gammaEuler);

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                            verticalFocalLength, horizontalPrincipalPoint,
                            verticalPrincipalPoint, skewness);

            // random camera
            final PinholeCamera camera = new PinholeCamera(intrinsic, rotation,
                    cameraCenter);

            // point to a new random location
            final Matrix pointAtMatrix = Matrix.createWithUniformRandomValues(
                    INHOM_3D_COORDS, 1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final InhomogeneousPoint3D pointAt = new InhomogeneousPoint3D(
                    pointAtMatrix.getElementAtIndex(0),
                    pointAtMatrix.getElementAtIndex(1),
                    pointAtMatrix.getElementAtIndex(2));

            camera.pointAt(pointAt);

            // after pointing the camera to a new location, we decompose it
            camera.decompose(true, true);
            // and retrieve new intrinsics, rotation and camera center, and ensure
            // that intrinsics and camera center have not been modified
            final PinholeCameraIntrinsicParameters intrinsic2 =
                    camera.getIntrinsicParameters();
            final Point3D cameraCenter2 = camera.getCameraCenter();

            assertTrue(camera.areIntrinsicParametersAvailable());
            assertTrue(camera.isCameraCenterAvailable());

            assertEquals(intrinsic.getHorizontalFocalLength(),
                    intrinsic2.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(intrinsic.getVerticalFocalLength(),
                    intrinsic2.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(intrinsic.getSkewness(), intrinsic2.getSkewness(),
                    ABSOLUTE_ERROR);
            assertEquals(intrinsic.getHorizontalPrincipalPoint(),
                    intrinsic2.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(intrinsic.getVerticalPrincipalPoint(),
                    intrinsic2.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            if (Math.abs(cameraCenter.getInhomX() - cameraCenter2.getInhomX()) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(cameraCenter.getInhomX(), cameraCenter2.getInhomX(),
                    LARGE_ABSOLUTE_ERROR);
            if (Math.abs(cameraCenter.getInhomY() - cameraCenter2.getInhomY()) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(cameraCenter.getInhomY(), cameraCenter2.getInhomY(),
                    LARGE_ABSOLUTE_ERROR);
            if (Math.abs(cameraCenter.getInhomZ() - cameraCenter2.getInhomZ()) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(cameraCenter.getInhomZ(), cameraCenter2.getInhomZ(),
                    LARGE_ABSOLUTE_ERROR);

            // principal axis array is a 3-component array indicating direction
            // where camera is looking at
            final double[] principalAxis = camera.getPrincipalAxisArray();

            // diff array contains the difference between provided 3D point to
            // look at and camera center. This array has to be proportional (up to
            // scale) to the principal axis array
            final double[] diffArray = new double[INHOM_3D_COORDS];
            diffArray[0] = pointAt.getInhomX() - cameraCenter2.getInhomX();
            diffArray[1] = pointAt.getInhomY() - cameraCenter2.getInhomY();
            diffArray[2] = pointAt.getInhomZ() - cameraCenter2.getInhomZ();
            final double norm = com.irurueta.algebra.Utils.normF(diffArray);
            ArrayUtils.multiplyByScalar(diffArray, 1.0 / norm, diffArray);

            final double scaleX = diffArray[0] / principalAxis[0];
            final double scaleY = diffArray[1] / principalAxis[1];
            final double scaleZ = diffArray[2] / principalAxis[2];

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
    public void testGetSetCameraCenterAvailabilityAndComputeCameraCenterSVDDetAndFinite()
            throws WrongSizeException, CameraException, NotAvailableException {

        final Matrix cameraCenterMatrix = Matrix.createWithUniformRandomValues(
                INHOM_3D_COORDS, 1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                cameraCenterMatrix.getElementAtIndex(0),
                cameraCenterMatrix.getElementAtIndex(1),
                cameraCenterMatrix.getElementAtIndex(2));

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        final double horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                MAX_FOCAL_LENGTH);
        final double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                MAX_FOCAL_LENGTH);

        final double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

        final double horizontalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final double verticalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        final Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler,
                gammaEuler);
        final PinholeCameraIntrinsicParameters intrinsic =
                new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                        verticalFocalLength, horizontalPrincipalPoint,
                        verticalPrincipalPoint, skewness);

        // instantiate new pinhole camera
        final PinholeCamera camera = new PinholeCamera(intrinsic, rotation,
                cameraCenter);

        // set new camera center
        final Matrix cameraCenterMatrix2 = Matrix.createWithUniformRandomValues(
                INHOM_3D_COORDS, 1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final InhomogeneousPoint3D cameraCenter2 = new InhomogeneousPoint3D(
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

        final Rotation3D rotation2 = camera.getCameraRotation();
        final PinholeCameraIntrinsicParameters intrinsic2 =
                camera.getIntrinsicParameters();
        final Point3D cameraCenter3 = camera.getCameraCenter();

        // also compute center without decomposition using different methods
        final Point3D cameraCenterSVD = camera.computeCameraCenterSVD();
        final Point3D cameraCenterSVD2 = Point3D.create();
        camera.computeCameraCenterSVD(cameraCenterSVD2);
        final Point3D cameraCenterDet = camera.computeCameraCenterDet();
        final Point3D cameraCenterDet2 = Point3D.create();
        camera.computeCameraCenterDet(cameraCenterDet2);
        final Point3D cameraCenterFinite = camera.computeCameraCenterFiniteCamera();
        final Point3D cameraCenterFinite2 = Point3D.create();
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
        assertTrue(rotation.asInhomogeneousMatrix().equals(
                rotation2.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

        // intrinsics remains the same
        assertEquals(intrinsic.getHorizontalFocalLength(),
                intrinsic2.getHorizontalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getVerticalFocalLength(),
                intrinsic2.getVerticalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getSkewness(), intrinsic2.getSkewness(),
                ABSOLUTE_ERROR);
        assertEquals(intrinsic.getHorizontalPrincipalPoint(),
                intrinsic2.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getVerticalPrincipalPoint(),
                intrinsic2.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);
    }

    @Test
    public void testSetIntrinsicParametersAndRotation() throws CameraException,
            NotAvailableException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        final double horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                MAX_FOCAL_LENGTH);
        final double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                MAX_FOCAL_LENGTH);

        final double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

        final double horizontalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final double verticalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        final Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler,
                gammaEuler);
        final PinholeCameraIntrinsicParameters intrinsic =
                new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                        verticalFocalLength, horizontalPrincipalPoint,
                        verticalPrincipalPoint, skewness);

        // instantiate canonical pinhole camera (focal length equal to 1,
        // pointing towards z axis and located at origin)
        final PinholeCamera camera = new PinholeCamera();

        // set intrinsic parameters and rotation
        camera.setIntrinsicParametersAndRotation(intrinsic, rotation);

        // decompose camera to obtain new intrinsics, rotation and center
        camera.decompose(true, true);
        // decomposed elements are available
        assertTrue(camera.isCameraCenterAvailable());
        assertTrue(camera.isCameraRotationAvailable());
        assertTrue(camera.areIntrinsicParametersAvailable());

        final Point3D cameraCenter2 = camera.getCameraCenter();
        final Rotation3D rotation2 = camera.getCameraRotation();
        final PinholeCameraIntrinsicParameters intrinsic2 =
                camera.getIntrinsicParameters();

        // check camera center remains at origin
        assertEquals(cameraCenter2.getInhomX(), 0.0, ABSOLUTE_ERROR);
        assertEquals(cameraCenter2.getInhomY(), 0.0, ABSOLUTE_ERROR);
        assertEquals(cameraCenter2.getInhomZ(), 0.0, ABSOLUTE_ERROR);

        // check camera rotation correctness
        // are different instances
        assertNotSame(rotation, rotation2);
        assertTrue(rotation.asInhomogeneousMatrix().equals(
                rotation2.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

        // check camera intrinsic parameters
        // are different instances
        assertNotSame(intrinsic, intrinsic2);
        assertEquals(intrinsic.getHorizontalFocalLength(),
                intrinsic2.getHorizontalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getVerticalFocalLength(),
                intrinsic2.getVerticalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getSkewness(), intrinsic2.getSkewness(),
                ABSOLUTE_ERROR);
        assertEquals(intrinsic.getHorizontalPrincipalPoint(),
                intrinsic2.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getVerticalPrincipalPoint(),
                intrinsic2.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);
    }

    @Test
    public void testSetIntrinsicAndExtrinsicParameters()
            throws WrongSizeException, CameraException, NotAvailableException {

        final Matrix centerMatrix = Matrix.createWithUniformRandomValues(
                INHOM_3D_COORDS, 1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                centerMatrix.getElementAtIndex(0),
                centerMatrix.getElementAtIndex(1),
                centerMatrix.getElementAtIndex(2));

        final Matrix worldOriginMatrix = Matrix.createWithUniformRandomValues(
                HOM_2D_COORDS, 1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final HomogeneousPoint2D imageOfWorldOrigin = new HomogeneousPoint2D(
                worldOriginMatrix.getElementAtIndex(0),
                worldOriginMatrix.getElementAtIndex(1),
                worldOriginMatrix.getElementAtIndex(2));

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double alphaEuler1 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double betaEuler1 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double gammaEuler1 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        final double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        final double horizontalFocalLength1 = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                MAX_FOCAL_LENGTH);
        final double verticalFocalLength1 = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                MAX_FOCAL_LENGTH);

        final double horizontalFocalLength2 = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                MAX_FOCAL_LENGTH);
        final double verticalFocalLength2 = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                MAX_FOCAL_LENGTH);

        final double skewness1 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
        final double skewness2 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

        final double horizontalPrincipalPoint1 = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final double verticalPrincipalPoint1 = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        final double horizontalPrincipalPoint2 = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final double verticalPrincipalPoint2 = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        final Rotation3D rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1,
                gammaEuler1);
        final Rotation3D rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2,
                gammaEuler2);

        final PinholeCameraIntrinsicParameters intrinsic1 =
                new PinholeCameraIntrinsicParameters(horizontalFocalLength1,
                        verticalFocalLength1, horizontalPrincipalPoint1,
                        verticalPrincipalPoint1, skewness1);
        final PinholeCameraIntrinsicParameters intrinsic2 =
                new PinholeCameraIntrinsicParameters(horizontalFocalLength2,
                        verticalFocalLength2, horizontalPrincipalPoint2,
                        verticalPrincipalPoint2, skewness2);

        // instantiate canonical pinhole camera
        final PinholeCamera camera = new PinholeCamera();

        // set intrinsic and extrinsic parameters using image of world origin
        camera.setIntrinsicAndExtrinsicParameters(intrinsic1, rotation1,
                imageOfWorldOrigin);
        assertFalse(camera.isCameraCenterAvailable());
        assertTrue(camera.isCameraRotationAvailable());
        assertTrue(camera.areIntrinsicParametersAvailable());

        // decompose camera to check correctness of parameters that have been set
        camera.decompose(true, true);
        assertTrue(camera.isCameraCenterAvailable());
        assertTrue(camera.isCameraRotationAvailable());
        assertTrue(camera.areIntrinsicParametersAvailable());

        // check correctness of image of world origin
        final Point2D imageOfWorldOrigin2 = camera.getImageOfWorldOrigin();
        assertTrue(imageOfWorldOrigin.equals(imageOfWorldOrigin2,
                ABSOLUTE_ERROR));

        // check correctness of intrinsic parameters
        final PinholeCameraIntrinsicParameters intrinsic3 =
                camera.getIntrinsicParameters();
        assertEquals(intrinsic3.getHorizontalFocalLength(),
                intrinsic1.getHorizontalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic3.getVerticalFocalLength(),
                intrinsic1.getVerticalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic3.getSkewness(), intrinsic1.getSkewness(),
                ABSOLUTE_ERROR);
        assertEquals(intrinsic3.getHorizontalPrincipalPoint(),
                intrinsic1.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
        assertEquals(intrinsic3.getVerticalPrincipalPoint(),
                intrinsic1.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

        // check correctness of rotation
        final Rotation3D rotation3 = camera.getCameraRotation();
        assertTrue(rotation3.asInhomogeneousMatrix().equals(
                rotation1.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

        // set new intrinsic and extrinsic parameters using camera center
        camera.setIntrinsicAndExtrinsicParameters(intrinsic2, rotation2,
                cameraCenter);
        assertTrue(camera.isCameraCenterAvailable());
        assertTrue(camera.isCameraRotationAvailable());
        assertTrue(camera.areIntrinsicParametersAvailable());

        // decompose camera to check correctness of parameters that have been set
        camera.decompose(true, true);
        assertTrue(camera.isCameraCenterAvailable());
        assertTrue(camera.isCameraRotationAvailable());
        assertTrue(camera.areIntrinsicParametersAvailable());

        // check correctness of camera center
        final Point3D cameraCenter2 = camera.getCameraCenter();
        assertTrue(cameraCenter.equals(cameraCenter2, LARGE_ABSOLUTE_ERROR));

        // check correctness of intrinsic parameters
        final PinholeCameraIntrinsicParameters intrinsic4 =
                camera.getIntrinsicParameters();
        assertEquals(intrinsic4.getHorizontalFocalLength(),
                intrinsic2.getHorizontalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic4.getVerticalFocalLength(),
                intrinsic2.getVerticalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic4.getSkewness(), intrinsic2.getSkewness(),
                ABSOLUTE_ERROR);
        assertEquals(intrinsic4.getHorizontalPrincipalPoint(),
                intrinsic2.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
        assertEquals(intrinsic4.getVerticalPrincipalPoint(),
                intrinsic2.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

        // check correctness of rotation
        final Rotation3D rotation4 = camera.getCameraRotation();
        assertTrue(rotation4.asInhomogeneousMatrix().equals(
                rotation2.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
    }

    @Test
    public void testGetSetVanishingPointsAndImageOfWorldOrigin()
            throws WrongSizeException {

        Matrix internalMatrix = Matrix.createWithUniformRandomValues(
                PINHOLE_CAMERA_ROWS, PINHOLE_CAMERA_COLS, MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final PinholeCamera camera = new PinholeCamera(internalMatrix);

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // test get set x-axis vanishing point
        final Point2D xAxisVanishingPoint = camera.getXAxisVanishingPoint();
        double scaleX = internalMatrix.getElementAt(0, 0) /
                xAxisVanishingPoint.getHomX();
        double scaleY = internalMatrix.getElementAt(1, 0) /
                xAxisVanishingPoint.getHomY();
        double scaleW = internalMatrix.getElementAt(2, 0) /
                xAxisVanishingPoint.getHomW();
        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleW, ABSOLUTE_ERROR);
        assertEquals(scaleW, scaleX, ABSOLUTE_ERROR);
        final Point2D xAxisVanishingPoint2 = Point2D.create();
        camera.xAxisVanishingPoint(xAxisVanishingPoint2);
        assertEquals(xAxisVanishingPoint, xAxisVanishingPoint2);

        // set new x-axis vanishing point
        double x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double w = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        xAxisVanishingPoint.setHomogeneousCoordinates(x, y, w);
        camera.setXAxisVanishingPoint(xAxisVanishingPoint);
        // when setting x-axis vanishing point, internal matrix will be modified
        internalMatrix = camera.getInternalMatrix();
        scaleX = internalMatrix.getElementAt(0, 0) /
                xAxisVanishingPoint.getHomX();
        scaleY = internalMatrix.getElementAt(1, 0) /
                xAxisVanishingPoint.getHomY();
        scaleW = internalMatrix.getElementAt(2, 0) /
                xAxisVanishingPoint.getHomW();
        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleW, ABSOLUTE_ERROR);
        assertEquals(scaleW, scaleX, ABSOLUTE_ERROR);

        // test get set y-axis vanishing point
        final Point2D yAxisVanishingPoint = camera.getYAxisVanishingPoint();
        scaleX = internalMatrix.getElementAt(0, 1) /
                yAxisVanishingPoint.getHomX();
        scaleY = internalMatrix.getElementAt(1, 1) /
                yAxisVanishingPoint.getHomY();
        scaleW = internalMatrix.getElementAt(2, 1) /
                yAxisVanishingPoint.getHomW();
        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleW, ABSOLUTE_ERROR);
        assertEquals(scaleW, scaleX, ABSOLUTE_ERROR);
        final Point2D yAxisVanishingPoint2 = Point2D.create();
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
        scaleX = internalMatrix.getElementAt(0, 1) /
                yAxisVanishingPoint.getHomX();
        scaleY = internalMatrix.getElementAt(1, 1) /
                yAxisVanishingPoint.getHomY();
        scaleW = internalMatrix.getElementAt(2, 1) /
                yAxisVanishingPoint.getHomW();
        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleW, ABSOLUTE_ERROR);
        assertEquals(scaleW, scaleX, ABSOLUTE_ERROR);

        // test get set z-axis vanishing point
        final Point2D zAxisVanishingPoint = camera.getZAxisVanishingPoint();
        scaleX = internalMatrix.getElementAt(0, 2) /
                zAxisVanishingPoint.getHomX();
        scaleY = internalMatrix.getElementAt(1, 2) /
                zAxisVanishingPoint.getHomY();
        scaleW = internalMatrix.getElementAt(2, 2) /
                zAxisVanishingPoint.getHomW();
        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleW, ABSOLUTE_ERROR);
        assertEquals(scaleW, scaleX, ABSOLUTE_ERROR);
        final Point2D zAxisVanishingPoint2 = Point2D.create();
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
        scaleX = internalMatrix.getElementAt(0, 2) /
                zAxisVanishingPoint.getHomX();
        scaleY = internalMatrix.getElementAt(1, 2) /
                zAxisVanishingPoint.getHomY();
        scaleW = internalMatrix.getElementAt(2, 2) /
                zAxisVanishingPoint.getHomW();
        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleW, ABSOLUTE_ERROR);
        assertEquals(scaleW, scaleX, ABSOLUTE_ERROR);

        // test get set image of world origin
        final Point2D imageOfWorldOrigin = camera.getImageOfWorldOrigin();
        scaleX = internalMatrix.getElementAt(0, 3) /
                imageOfWorldOrigin.getHomX();
        scaleY = internalMatrix.getElementAt(1, 3) /
                imageOfWorldOrigin.getHomY();
        scaleW = internalMatrix.getElementAt(2, 3) /
                imageOfWorldOrigin.getHomW();
        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleW, ABSOLUTE_ERROR);
        assertEquals(scaleW, scaleX, ABSOLUTE_ERROR);
        final Point2D imageOfWorldOrigin2 = Point2D.create();
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
        scaleX = internalMatrix.getElementAt(0, 3) /
                imageOfWorldOrigin.getHomX();
        scaleY = internalMatrix.getElementAt(1, 3) /
                imageOfWorldOrigin.getHomY();
        scaleW = internalMatrix.getElementAt(2, 3) /
                imageOfWorldOrigin.getHomW();
        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleW, ABSOLUTE_ERROR);
        assertEquals(scaleW, scaleX, ABSOLUTE_ERROR);
    }

    @Test
    public void testGetSetHorizontalVerticalAndPrincipalPlanesAndAxis()
            throws WrongSizeException, CameraException {

        Matrix internalMatrix = Matrix.createWithUniformRandomValues(
                PINHOLE_CAMERA_ROWS, PINHOLE_CAMERA_COLS, MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final PinholeCamera camera = new PinholeCamera(internalMatrix);

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // test get set vertical axis plane
        final Plane verticalAxisPlane = camera.getVerticalAxisPlane();
        double scaleA = internalMatrix.getElementAt(0, 0) /
                verticalAxisPlane.getA();
        double scaleB = internalMatrix.getElementAt(0, 1) /
                verticalAxisPlane.getB();
        double scaleC = internalMatrix.getElementAt(0, 2) /
                verticalAxisPlane.getC();
        double scaleD = internalMatrix.getElementAt(0, 3) /
                verticalAxisPlane.getD();
        assertEquals(scaleA, scaleB, ABSOLUTE_ERROR);
        assertEquals(scaleB, scaleC, ABSOLUTE_ERROR);
        assertEquals(scaleC, scaleD, ABSOLUTE_ERROR);
        assertEquals(scaleD, scaleA, ABSOLUTE_ERROR);
        final Plane verticalAxisPlane2 = new Plane();
        camera.verticalAxisPlane(verticalAxisPlane2);
        assertEquals(verticalAxisPlane, verticalAxisPlane2);

        // set new vertical axis plane
        double a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
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
        final Plane horizontalAxisPlane = camera.getHorizontalAxisPlane();
        scaleA = internalMatrix.getElementAt(1, 0) / horizontalAxisPlane.getA();
        scaleB = internalMatrix.getElementAt(1, 1) / horizontalAxisPlane.getB();
        scaleC = internalMatrix.getElementAt(1, 2) / horizontalAxisPlane.getC();
        scaleD = internalMatrix.getElementAt(1, 3) / horizontalAxisPlane.getD();
        assertEquals(scaleA, scaleB, ABSOLUTE_ERROR);
        assertEquals(scaleB, scaleC, ABSOLUTE_ERROR);
        assertEquals(scaleC, scaleD, ABSOLUTE_ERROR);
        assertEquals(scaleD, scaleA, ABSOLUTE_ERROR);
        final Plane horizontalAxisPlane2 = new Plane();
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
        final Plane principalPlane = camera.getPrincipalPlane();
        scaleA = internalMatrix.getElementAt(2, 0) / principalPlane.getA();
        scaleB = internalMatrix.getElementAt(2, 1) / principalPlane.getB();
        scaleC = internalMatrix.getElementAt(2, 2) / principalPlane.getC();
        scaleD = internalMatrix.getElementAt(2, 3) / principalPlane.getD();
        assertEquals(scaleA, scaleB, ABSOLUTE_ERROR);
        assertEquals(scaleB, scaleC, ABSOLUTE_ERROR);
        assertEquals(scaleC, scaleD, ABSOLUTE_ERROR);
        assertEquals(scaleD, scaleA, ABSOLUTE_ERROR);
        final Plane principalPlane2 = new Plane();
        camera.principalPlane(principalPlane2);
        assertEquals(principalPlane, principalPlane2);

        // test that principal axis array is up to scale with principal plane
        // director vector (its first 3 components)
        double[] principalAxisArray = camera.getPrincipalAxisArray();
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
        final double[] principalAxisArray2 = new double[3];
        camera.principalAxisArray(principalAxisArray2);
        assertArrayEquals(principalAxisArray, principalAxisArray2, 0.0);
    }

    @Test
    public void testGetPrincipalPoint() throws WrongSizeException {
        final Matrix centerMatrix = Matrix.createWithUniformRandomValues(
                INHOM_3D_COORDS, 1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                centerMatrix.getElementAtIndex(0),
                centerMatrix.getElementAtIndex(1),
                centerMatrix.getElementAtIndex(2));

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        final double horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                MAX_FOCAL_LENGTH);
        final double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                MAX_FOCAL_LENGTH);

        final double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

        final double horizontalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final double verticalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        final Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler,
                gammaEuler);
        final PinholeCameraIntrinsicParameters intrinsic =
                new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                        verticalFocalLength, horizontalPrincipalPoint,
                        verticalPrincipalPoint, skewness);

        final PinholeCamera camera = new PinholeCamera(intrinsic, rotation,
                cameraCenter);

        // get principal point
        final Point2D principalPoint = camera.getPrincipalPoint();
        final Point2D principalPoint2 = Point2D.create();
        camera.principalPoint(principalPoint2);

        // and check correctness
        assertEquals(principalPoint.getInhomX(), horizontalPrincipalPoint,
                ABSOLUTE_ERROR);
        assertEquals(principalPoint.getInhomY(), verticalPrincipalPoint,
                ABSOLUTE_ERROR);
        assertEquals(principalPoint, principalPoint2);
    }

    @Test
    public void testGetAndFixCameraSign() throws WrongSizeException,
            DecomposerException, CameraException {

        final Matrix internalMatrix = Matrix.createWithUniformRandomValues(
                PINHOLE_CAMERA_ROWS, PINHOLE_CAMERA_COLS, MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final Matrix Mp = internalMatrix.getSubmatrix(0, 0, 2, 2);
        final double detMp = com.irurueta.algebra.Utils.det(Mp);
        final double cameraSign = (detMp > 0.0) ? 1.0 : -1.0;

        final PinholeCamera camera = new PinholeCamera(internalMatrix);

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
    public void testGetDepthCheiralityAndFrontOfCamera()
            throws WrongSizeException, CameraException, NotReadyException,
            LockedException, DecomposerException,
            com.irurueta.algebra.NotAvailableException {

        final Matrix centerMatrix = Matrix.createWithUniformRandomValues(
                INHOM_3D_COORDS, 1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                centerMatrix.getElementAtIndex(0),
                centerMatrix.getElementAtIndex(1),
                centerMatrix.getElementAtIndex(2));

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        final double horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                MAX_FOCAL_LENGTH);
        final double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                MAX_FOCAL_LENGTH);

        final double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

        final double horizontalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final double verticalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        final double positiveDepth = randomizer.nextDouble(MIN_DEPTH, MAX_DEPTH);
        final double negativeDepth = randomizer.nextDouble(-MAX_DEPTH, -MIN_DEPTH);

        final Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler,
                gammaEuler);
        final PinholeCameraIntrinsicParameters intrinsic =
                new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                        verticalFocalLength, horizontalPrincipalPoint,
                        verticalPrincipalPoint, skewness);

        final PinholeCamera camera = new PinholeCamera(intrinsic, rotation,
                cameraCenter);

        final double[] principalAxisArray = camera.getPrincipalAxisArray();

        final Plane principalPlane = camera.getPrincipalPlane();
        final Matrix principalPlaneMatrix = new Matrix(1, HOM_3D_COORDS);
        principalPlaneMatrix.setElementAtIndex(0, principalPlane.getA());
        principalPlaneMatrix.setElementAtIndex(1, principalPlane.getB());
        principalPlaneMatrix.setElementAtIndex(2, principalPlane.getC());
        principalPlaneMatrix.setElementAtIndex(3, principalPlane.getD());

        // use SVD decomposition of principalPlaneMatrix to find points on the
        // plane as any arbitrary linear combination of the null-space of
        // principalPlaneMatrix
        final SingularValueDecomposer decomposer = new SingularValueDecomposer(
                principalPlaneMatrix);

        decomposer.decompose();

        final Matrix v = decomposer.getV();

        // first column of V is the principal plane matrix itself. Remaining
        // three columns are three homogeneous world points defining such plane
        // (hence they are its null-space)
        final HomogeneousPoint3D planePoint1 = new HomogeneousPoint3D(
                v.getElementAt(0, 1), v.getElementAt(1, 1),
                v.getElementAt(2, 1), v.getElementAt(3, 1));
        final HomogeneousPoint3D planePoint2 = new HomogeneousPoint3D(
                v.getElementAt(0, 2), v.getElementAt(1, 2),
                v.getElementAt(2, 2), v.getElementAt(3, 2));
        final HomogeneousPoint3D planePoint3 = new HomogeneousPoint3D(
                v.getElementAt(0, 3), v.getElementAt(1, 3),
                v.getElementAt(2, 3), v.getElementAt(3, 3));

        // create three points in front of the camera (with positiveDepth) and
        // three points behind the camera (with negativeDepth) using the three
        // points on the principal plane and adding to their inhomogeneous
        // coordinates some positiveDepth or negativeDepth value on the principal
        // axis direction (which is normalized)
        final InhomogeneousPoint3D frontPoint1 = new InhomogeneousPoint3D(
                planePoint1.getInhomX() + (positiveDepth * principalAxisArray[0]),
                planePoint1.getInhomY() + (positiveDepth * principalAxisArray[1]),
                planePoint1.getInhomZ() + (positiveDepth * principalAxisArray[2]));
        final InhomogeneousPoint3D frontPoint2 = new InhomogeneousPoint3D(
                planePoint2.getInhomX() + (positiveDepth * principalAxisArray[0]),
                planePoint2.getInhomY() + (positiveDepth * principalAxisArray[1]),
                planePoint2.getInhomZ() + (positiveDepth * principalAxisArray[2]));
        final InhomogeneousPoint3D frontPoint3 = new InhomogeneousPoint3D(
                planePoint3.getInhomX() + (positiveDepth * principalAxisArray[0]),
                planePoint3.getInhomY() + (positiveDepth * principalAxisArray[1]),
                planePoint3.getInhomZ() + (positiveDepth * principalAxisArray[2]));

        final InhomogeneousPoint3D backPoint1 = new InhomogeneousPoint3D(
                planePoint1.getInhomX() + (negativeDepth * principalAxisArray[0]),
                planePoint1.getInhomY() + (negativeDepth * principalAxisArray[1]),
                planePoint1.getInhomZ() + (negativeDepth * principalAxisArray[2]));
        final InhomogeneousPoint3D backPoint2 = new InhomogeneousPoint3D(
                planePoint2.getInhomX() + (negativeDepth * principalAxisArray[0]),
                planePoint2.getInhomY() + (negativeDepth * principalAxisArray[1]),
                planePoint2.getInhomZ() + (negativeDepth * principalAxisArray[2]));
        final InhomogeneousPoint3D backPoint3 = new InhomogeneousPoint3D(
                planePoint3.getInhomX() + (negativeDepth * principalAxisArray[0]),
                planePoint3.getInhomY() + (negativeDepth * principalAxisArray[1]),
                planePoint3.getInhomZ() + (negativeDepth * principalAxisArray[2]));

        // now check correctness of depth for front points
        assertEquals(camera.getDepth(frontPoint1), positiveDepth,
                ABSOLUTE_ERROR);
        assertEquals(camera.getDepth(frontPoint2), positiveDepth,
                ABSOLUTE_ERROR);
        assertEquals(camera.getDepth(frontPoint3), positiveDepth,
                ABSOLUTE_ERROR);

        // check that points are indeed in front by checking also that their
        // cheirality is positive
        assertTrue(camera.getCheirality(frontPoint1) > 0.0);
        assertTrue(camera.getCheirality(frontPoint2) > 0.0);
        assertTrue(camera.getCheirality(frontPoint3) > 0.0);

        assertTrue(camera.isPointInFrontOfCamera(frontPoint1));
        assertTrue(camera.isPointInFrontOfCamera(frontPoint2));
        assertTrue(camera.isPointInFrontOfCamera(frontPoint3));
        assertTrue(camera.isPointInFrontOfCamera(frontPoint1, 0.0));

        // now for back points
        assertEquals(camera.getDepth(backPoint1), negativeDepth,
                ABSOLUTE_ERROR);
        assertEquals(camera.getDepth(backPoint2), negativeDepth,
                ABSOLUTE_ERROR);
        assertEquals(camera.getDepth(backPoint3), negativeDepth,
                ABSOLUTE_ERROR);

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
    public void testGetDepthsCheiralitiesAndInFrontOfCamera()
            throws WrongSizeException, CameraException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int nPoints = randomizer.nextInt(MIN_N_POINTS, MAX_N_POINTS);

        // generate a random camera and a list of random world points, and then
        // compare depth, cheiral and if each point is in front of camera by
        // using the methods that have been previously tested

        // list of random points
        final List<Point3D> worldPointList = new ArrayList<>(nPoints);
        for (int i = 0; i < nPoints; i++) {
            final HomogeneousPoint3D point = new HomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            worldPointList.add(point);
        }
        assertEquals(worldPointList.size(), nPoints);

        // random camera
        final Matrix centerMatrix = Matrix.createWithUniformRandomValues(
                INHOM_3D_COORDS, 1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                centerMatrix.getElementAtIndex(0),
                centerMatrix.getElementAtIndex(1),
                centerMatrix.getElementAtIndex(2));

        final double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        final double horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                MAX_FOCAL_LENGTH);
        final double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                MAX_FOCAL_LENGTH);

        final double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

        final double horizontalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final double verticalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        final Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler,
                gammaEuler);
        final PinholeCameraIntrinsicParameters intrinsic =
                new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                        verticalFocalLength, horizontalPrincipalPoint,
                        verticalPrincipalPoint, skewness);

        final PinholeCamera camera = new PinholeCamera(intrinsic, rotation,
                cameraCenter);

        // get depths of all points in the list
        final List<Double> depths = camera.getDepths(worldPointList);
        final List<Double> depths2 = new ArrayList<>(worldPointList.size());
        camera.depths(worldPointList, depths2);
        assertEquals(depths, depths2);

        // get cheiralities for all points in the list
        final List<Double> cheiralities = camera.getCheiralities(worldPointList);
        final List<Boolean> fronts = camera.arePointsInFrontOfCamera(worldPointList);
        final List<Boolean> fronts2 = new ArrayList<>(worldPointList.size());
        camera.arePointsInFrontOfCamera(worldPointList, fronts2);
        final List<Boolean> fronts3 = camera.arePointsInFrontOfCamera(worldPointList, 0.0);
        final List<Boolean> fronts4 = new ArrayList<>(worldPointList.size());
        camera.arePointsInFrontOfCamera(worldPointList, fronts4, 0.0);

        final List<Double> cheiralities2 = new ArrayList<>(worldPointList.size());
        camera.cheiralities(worldPointList, cheiralities2);
        assertEquals(cheiralities, cheiralities2);

        // iterate over lists to test each points individually
        final Iterator<Point3D> pointsIterator = worldPointList.iterator();
        final Iterator<Double> depthsIterator = depths.iterator();
        final Iterator<Double> cheiralitiesIterator = cheiralities.iterator();
        final Iterator<Boolean> frontsIterator = fronts.iterator();
        final Iterator<Boolean> fronts2Iterator = fronts2.iterator();
        Point3D point;
        Double depth;
        Double cheirality;
        Boolean front;
        Boolean front2;
        double camDepth;
        double camCheirality;
        boolean camFront;
        while (pointsIterator.hasNext()) {
            point = pointsIterator.next();

            depth = depthsIterator.next();
            cheirality = cheiralitiesIterator.next();
            front = frontsIterator.next();
            front2 = fronts2Iterator.next();

            camDepth = camera.getDepth(point);
            camCheirality = camera.getCheirality(point);
            camFront = camera.isPointInFrontOfCamera(point);

            assertEquals(depth, camDepth, ABSOLUTE_ERROR);
            assertEquals(cheirality, camCheirality,
                    ABSOLUTE_ERROR);
            assertEquals(front, camFront);
            assertEquals(front2, camFront);
        }

        assertEquals(fronts, fronts2);
        assertEquals(fronts, fronts3);
        assertEquals(fronts, fronts4);
    }

    @Test
    public void testCreateCanonicalCamera() throws WrongSizeException {
        final PinholeCamera camera = PinholeCamera.createCanonicalCamera();
        final Matrix internalMatrix = camera.getInternalMatrix();

        // check that internal matrix is the 3x4 identity matrix
        assertTrue(internalMatrix.equals(Matrix.identity(PINHOLE_CAMERA_ROWS,
                PINHOLE_CAMERA_COLS), ABSOLUTE_ERROR));
    }

    @Test
    public void testIsNormalized() throws WrongSizeException, CameraException {
        // test that whenever a parameter is modified in a camera, it becomes non
        // normalized

        final Matrix cameraMatrix = Matrix.createWithUniformRandomValues(
                PINHOLE_CAMERA_ROWS, PINHOLE_CAMERA_COLS, MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        // Testing isNormalized after setInternalMatrix
        final PinholeCamera camera = new PinholeCamera();
        assertFalse(camera.isNormalized());
        camera.setInternalMatrix(cameraMatrix);
        assertFalse(camera.isNormalized());
        camera.normalize();
        assertTrue(camera.isNormalized());
        camera.setInternalMatrix(cameraMatrix);
        assertFalse(camera.isNormalized());

        // testing isNormalized after setCameraRotation
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        final Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler,
                gammaEuler);
        assertFalse(camera.isNormalized());
        camera.setCameraRotation(rotation);
        assertFalse(camera.isNormalized());
        camera.normalize();
        assertTrue(camera.isNormalized());
        camera.setCameraRotation(rotation);
        assertFalse(camera.isNormalized());

        // testing isNormalized after setCameraIntrinsicParameters
        final double horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                MAX_FOCAL_LENGTH);
        final double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                MAX_FOCAL_LENGTH);
        final double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
        final double horizontalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final double verticalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        final PinholeCameraIntrinsicParameters k =
                new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                        verticalFocalLength, horizontalPrincipalPoint,
                        verticalPrincipalPoint, skewness);

        assertFalse(camera.isNormalized());
        camera.setIntrinsicParameters(k);
        assertFalse(camera.isNormalized());
        camera.normalize();
        assertTrue(camera.isNormalized());
        camera.setIntrinsicParameters(k);
        assertFalse(camera.isNormalized());

        // testing isNormalized after setCameraCenter
        final double[] cameraCenterArray = new double[INHOM_3D_COORDS];
        randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                cameraCenterArray);

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
        final double[] worldOriginArray = new double[HOM_2D_COORDS];
        randomizer.fill(worldOriginArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final HomogeneousPoint2D imageOfWorldOrigin = new HomogeneousPoint2D(
                worldOriginArray);

        assertFalse(camera.isNormalized());
        camera.setIntrinsicAndExtrinsicParameters(k, rotation,
                imageOfWorldOrigin);
        assertFalse(camera.isNormalized());
        camera.normalize();
        assertTrue(camera.isNormalized());
        camera.setIntrinsicAndExtrinsicParameters(k, rotation,
                imageOfWorldOrigin);
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
        double x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double w = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final Point2D xAxisVanishingPoint = new HomogeneousPoint2D(x, y, w);
        x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        w = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final Point2D yAxisVanishingPoint = new HomogeneousPoint2D(x, y, w);
        x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        w = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final Point2D zAxisVanishingPoint = new HomogeneousPoint2D(x, y, w);

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

        // testing isNormalized after setting horizontal, vertical and principal
        // plane
        double a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final Plane verticalAxisPlane = new Plane(a, b, c, d);

        a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final Plane horizontalAxisPlane = new Plane(a, b, c, d);

        a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final Plane principalPlane = new Plane(a, b, c, d);

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
        final double[] pointAtArray = new double[INHOM_3D_COORDS];
        randomizer.fill(pointAtArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final InhomogeneousPoint3D pointAt = new InhomogeneousPoint3D(pointAtArray);

        assertFalse(camera.isNormalized());
        camera.pointAt(pointAt);
        assertFalse(camera.isNormalized());
        camera.normalize();
        assertTrue(camera.isNormalized());
        camera.pointAt(pointAt);
        assertFalse(camera.isNormalized());
    }

    @Test
    public void testProjectBackProjectDualQuadricAndDualConic()
            throws DecomposerException, WrongSizeException,
            CoincidentLinesException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            // instantiate dual conic from set of lines
            Matrix m = Matrix.createWithUniformRandomValues(5, HOM_2D_COORDS,
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            Line2D line1 = new Line2D(m.getElementAt(0, 0), m.getElementAt(0, 1),
                    m.getElementAt(0, 2));
            Line2D line2 = new Line2D(m.getElementAt(1, 0), m.getElementAt(1, 1),
                    m.getElementAt(1, 2));
            Line2D line3 = new Line2D(m.getElementAt(2, 0), m.getElementAt(2, 1),
                    m.getElementAt(2, 2));
            Line2D line4 = new Line2D(m.getElementAt(3, 0), m.getElementAt(3, 1),
                    m.getElementAt(3, 2));
            Line2D line5 = new Line2D(m.getElementAt(4, 0), m.getElementAt(4, 1),
                    m.getElementAt(4, 2));

            line1.normalize();
            line2.normalize();
            line3.normalize();
            line4.normalize();
            line5.normalize();

            // estimate dual conic that lies inside of provided 5 lines (we need to
            // ensure that line configuration is not degenerate)
            final Matrix m2 = new Matrix(5, 6);

            double l1 = line1.getA();
            double l2 = line1.getB();
            double l3 = line1.getC();
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
                m = Matrix.createWithUniformRandomValues(5, HOM_2D_COORDS,
                        MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

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

            final DualConic dualConic = new DualConic(line1, line2, line3, line4, line5);

            // check that lines are locus of dual conic
            assertTrue(dualConic.isLocus(line1, ABSOLUTE_ERROR));
            assertTrue(dualConic.isLocus(line2, ABSOLUTE_ERROR));
            assertTrue(dualConic.isLocus(line3, ABSOLUTE_ERROR));
            assertTrue(dualConic.isLocus(line4, ABSOLUTE_ERROR));
            assertTrue(dualConic.isLocus(line5, ABSOLUTE_ERROR));

            // instantiate random camera
            final Matrix cameraMatrix = Matrix.createWithUniformRandomValues(
                    PINHOLE_CAMERA_ROWS, PINHOLE_CAMERA_COLS, MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            final PinholeCamera camera = new PinholeCamera(cameraMatrix);

            try {
                // back-project dual conic into dual quadric
                final DualQuadric dualQuadric = camera.backProject(dualConic);
                final DualQuadric dualQuadric2 = new DualQuadric();
                camera.backProject(dualConic, dualQuadric2);

                assertEquals(dualQuadric.asMatrix(), dualQuadric2.asMatrix());

                // back-project planes of dual conic
                final Plane plane1 = camera.backProject(line1);
                final Plane plane2 = camera.backProject(line2);
                final Plane plane3 = camera.backProject(line3);
                final Plane plane4 = camera.backProject(line4);
                final Plane plane5 = camera.backProject(line5);

                // check that back-projected planes are locus of back-projected dual
                // quadric
                assertTrue(dualQuadric.isLocus(plane1));
                assertTrue(dualQuadric.isLocus(plane2));
                assertTrue(dualQuadric.isLocus(plane3));
                assertTrue(dualQuadric.isLocus(plane4));
                assertTrue(dualQuadric.isLocus(plane5));

                // project dual quadric into dual conic
                final DualConic dualConic2 = camera.project(dualQuadric);
                final DualConic dualConic3 = new DualConic();
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
    public void testProjectBackProjectQuadricAndConic()
            throws WrongSizeException, DecomposerException,
            CoincidentPointsException, CameraException {

        // create conic
        Matrix m = Matrix.createWithUniformRandomValues(5, HOM_2D_COORDS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        HomogeneousPoint2D point1 = new HomogeneousPoint2D(
                m.getElementAt(0, 0), m.getElementAt(0, 1),
                m.getElementAt(0, 2));
        HomogeneousPoint2D point2 = new HomogeneousPoint2D(
                m.getElementAt(1, 0), m.getElementAt(1, 1),
                m.getElementAt(1, 2));
        HomogeneousPoint2D point3 = new HomogeneousPoint2D(
                m.getElementAt(2, 0), m.getElementAt(2, 1),
                m.getElementAt(2, 2));
        HomogeneousPoint2D point4 = new HomogeneousPoint2D(
                m.getElementAt(3, 0), m.getElementAt(3, 1),
                m.getElementAt(3, 2));
        HomogeneousPoint2D point5 = new HomogeneousPoint2D(
                m.getElementAt(4, 0), m.getElementAt(4, 1),
                m.getElementAt(4, 2));

        // estimate conic that lies inside of provided 5 homogeneous 2D
        // points
        Matrix conicMatrix = new Matrix(5, 6);
        double x = point1.getHomX();
        double y = point1.getHomY();
        double w = point1.getHomW();
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
            m = Matrix.createWithUniformRandomValues(5, HOM_2D_COORDS,
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

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

        final Conic conic = new Conic(point1, point2, point3, point4, point5);

        // check that points are locus of conic
        assertTrue(conic.isLocus(point1, ABSOLUTE_ERROR));
        assertTrue(conic.isLocus(point2, ABSOLUTE_ERROR));
        assertTrue(conic.isLocus(point3, ABSOLUTE_ERROR));
        assertTrue(conic.isLocus(point4, ABSOLUTE_ERROR));
        assertTrue(conic.isLocus(point5, ABSOLUTE_ERROR));

        // instantiate random camera
        final Matrix cameraMatrix = Matrix.createWithUniformRandomValues(
                PINHOLE_CAMERA_ROWS, PINHOLE_CAMERA_COLS, MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final PinholeCamera camera = new PinholeCamera(cameraMatrix);

        // back-project conic into quadric
        final Quadric quadric = camera.backProject(conic);
        final Quadric quadric2 = new Quadric();
        camera.backProject(conic, quadric2);

        assertEquals(quadric.asMatrix(), quadric2.asMatrix());

        // back-project planes of dual conic
        final Point3D point1b = camera.backProject(point1);
        final Point3D point2b = camera.backProject(point2);
        final Point3D point3b = camera.backProject(point3);
        final Point3D point4b = camera.backProject(point4);
        final Point3D point5b = camera.backProject(point5);

        // check that back-projected points are locus of back-projected quadric
        assertTrue(quadric.isLocus(point1b, ABSOLUTE_ERROR));
        assertTrue(quadric.isLocus(point2b, ABSOLUTE_ERROR));
        assertTrue(quadric.isLocus(point3b, ABSOLUTE_ERROR));
        assertTrue(quadric.isLocus(point4b, ABSOLUTE_ERROR));
        assertTrue(quadric.isLocus(point5b, ABSOLUTE_ERROR));

        // project quadric into conic
        final Conic conic2 = camera.project(quadric);
        final Conic conic3 = new Conic();
        camera.project(quadric, conic3);

        conic.normalize();
        conic2.normalize();
        conic3.normalize();

        assertEquals(conic2.asMatrix(), conic3.asMatrix());
    }

    @Test
    public void testSetFromPointCorrespondences() throws CameraException,
            WrongListSizesException,
            com.irurueta.geometry.estimators.LockedException,
            com.irurueta.geometry.estimators.NotReadyException,
            PinholeCameraEstimatorException, NotAvailableException,
            RotationException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // create intrinsic parameters
        final double horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH2,
                MAX_FOCAL_LENGTH2);
        final double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH2,
                MAX_FOCAL_LENGTH2);
        final double skewness = randomizer.nextDouble(MIN_SKEWNESS2, MAX_SKEWNESS2);
        final double horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT,
                MAX_PRINCIPAL_POINT);
        final double verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT,
                MAX_PRINCIPAL_POINT);

        final PinholeCameraIntrinsicParameters intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                verticalFocalLength, horizontalPrincipalPoint,
                verticalPrincipalPoint, skewness);

        // create rotation parameters
        final double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES2,
                MAX_ANGLE_DEGREES2) * Math.PI / 180.0;
        final double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES2,
                MAX_ANGLE_DEGREES2) * Math.PI / 180.0;
        final double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES2,
                MAX_ANGLE_DEGREES2) * Math.PI / 180.0;

        final MatrixRotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

        // create camera center
        final double[] cameraCenterArray = new double[INHOM_3D_COORDS];
        randomizer.fill(cameraCenterArray, MIN_RANDOM_POINT_VALUE,
                MAX_RANDOM_POINT_VALUE);
        final InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

        // instantiate camera
        final PinholeCamera camera1 = new PinholeCamera(intrinsic, rotation, cameraCenter);

        // normalize the camera to improve accuracy
        camera1.normalize();

        // create 6 point correspondences
        final Point3D point3D1 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE,
                        MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE,
                        MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE,
                        MAX_RANDOM_POINT_VALUE));
        Point3D point3D2 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE,
                        MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE,
                        MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE,
                        MAX_RANDOM_POINT_VALUE));
        final Point3D point3D3 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE,
                        MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE,
                        MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE,
                        MAX_RANDOM_POINT_VALUE));
        final Point3D point3D4 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE,
                        MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE,
                        MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE,
                        MAX_RANDOM_POINT_VALUE));
        final Point3D point3D5 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE,
                        MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE,
                        MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE,
                        MAX_RANDOM_POINT_VALUE));
        final Point3D point3D6 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE,
                        MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE,
                        MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE,
                        MAX_RANDOM_POINT_VALUE));

        final Point2D point2D1 = camera1.project(point3D1);
        Point2D point2D2 = camera1.project(point3D2);
        final Point2D point2D3 = camera1.project(point3D3);
        final Point2D point2D4 = camera1.project(point3D4);
        final Point2D point2D5 = camera1.project(point3D5);
        final Point2D point2D6 = camera1.project(point3D6);

        final List<Point3D> points3D = new ArrayList<>(N_POINTS);
        points3D.add(point3D1);
        points3D.add(point3D2);
        points3D.add(point3D3);
        points3D.add(point3D4);
        points3D.add(point3D5);
        points3D.add(point3D6);

        final List<Point2D> points2D = camera1.project(points3D);

        final PinholeCamera camera2 = new PinholeCamera();
        camera2.setFromPointCorrespondences(point3D1, point3D2, point3D3, point3D4,
                point3D5, point3D6, point2D1, point2D2, point2D3, point2D4,
                point2D5, point2D6);
        camera2.decompose();

        final DLTPointCorrespondencePinholeCameraEstimator estimator =
                new DLTPointCorrespondencePinholeCameraEstimator(points3D,
                        points2D);
        estimator.setLMSESolutionAllowed(false);
        final PinholeCamera camera3 = estimator.estimate();
        camera3.decompose();

        final PinholeCameraIntrinsicParameters intrinsic2 = camera2.getIntrinsicParameters();
        final PinholeCameraIntrinsicParameters intrinsic3 = camera3.getIntrinsicParameters();

        assertEquals(intrinsic2.getHorizontalFocalLength(),
                intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic2.getVerticalFocalLength(),
                intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic2.getSkewness(), intrinsic.getSkewness(),
                ABSOLUTE_ERROR);
        assertEquals(intrinsic2.getHorizontalPrincipalPoint(),
                intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
        assertEquals(intrinsic2.getVerticalPrincipalPoint(),
                intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

        assertEquals(intrinsic2.getHorizontalFocalLength(),
                intrinsic3.getHorizontalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic2.getVerticalFocalLength(),
                intrinsic3.getVerticalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic2.getSkewness(), intrinsic3.getSkewness(),
                ABSOLUTE_ERROR);
        assertEquals(intrinsic2.getHorizontalPrincipalPoint(),
                intrinsic3.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
        assertEquals(intrinsic2.getVerticalPrincipalPoint(),
                intrinsic3.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

        final Rotation3D rotation2 = camera2.getCameraRotation();
        final Rotation3D rotation3 = camera3.getCameraRotation();

        assertTrue(rotation2.equals(rotation3, ABSOLUTE_ERROR));

        final Point3D cameraCenter2 = camera2.getCameraCenter();
        final Point3D cameraCenter3 = camera3.getCameraCenter();

        assertTrue(cameraCenter2.equals(cameraCenter3, ABSOLUTE_ERROR));
        assertTrue(cameraCenter2.equals(cameraCenter, ABSOLUTE_ERROR));

        // Force CameraException by repeating correspondences and creating a
        // degeneracy
        point3D2 = point3D1;
        point2D2 = camera1.project(point3D2);
        try {
            camera2.setFromPointCorrespondences(point3D1, point3D2, point3D3, point3D4,
                    point3D5, point3D6, point2D1, point2D2, point2D3, point2D4,
                    point2D5, point2D6);
            fail("CameraException expected but not thrown");
        } catch (final CameraException ignore) {
        }
    }

    @Test
    public void testSetFromLineAndPlaneCorrespondences() throws CameraException,
            WrongListSizesException,
            com.irurueta.geometry.estimators.LockedException,
            com.irurueta.geometry.estimators.NotReadyException,
            PinholeCameraEstimatorException, NotAvailableException,
            RotationException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // create intrinsic parameters
        final double horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH2,
                MAX_FOCAL_LENGTH2);
        final double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH2,
                MAX_FOCAL_LENGTH2);
        final double skewness = randomizer.nextDouble(MIN_SKEWNESS2, MAX_SKEWNESS2);
        final double horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT,
                MAX_PRINCIPAL_POINT);
        final double verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT,
                MAX_PRINCIPAL_POINT);

        final PinholeCameraIntrinsicParameters intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                verticalFocalLength, horizontalPrincipalPoint,
                verticalPrincipalPoint, skewness);

        // create rotation parameters
        final double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES2,
                MAX_ANGLE_DEGREES2) * Math.PI / 180.0;
        final double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES2,
                MAX_ANGLE_DEGREES2) * Math.PI / 180.0;
        final double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES2,
                MAX_ANGLE_DEGREES2) * Math.PI / 180.0;

        final MatrixRotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

        // create camera center
        final double[] cameraCenterArray = new double[INHOM_3D_COORDS];
        randomizer.fill(cameraCenterArray, MIN_RANDOM_POINT_VALUE,
                MAX_RANDOM_POINT_VALUE);
        final InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

        // instantiate camera
        final PinholeCamera camera1 = new PinholeCamera(intrinsic, rotation, cameraCenter);

        // normalize the camera to improve accuracy
        camera1.normalize();

        // create 4 2D lines
        final Line2D line2D1 = new Line2D(randomizer.nextDouble(MIN_RANDOM_LINE_VALUE,
                MAX_RANDOM_LINE_VALUE),
                randomizer.nextDouble(MIN_RANDOM_LINE_VALUE,
                        MAX_RANDOM_LINE_VALUE),
                randomizer.nextDouble(MIN_RANDOM_LINE_VALUE,
                        MAX_RANDOM_LINE_VALUE));
        Line2D line2D2 = new Line2D(randomizer.nextDouble(MIN_RANDOM_LINE_VALUE,
                MAX_RANDOM_LINE_VALUE),
                randomizer.nextDouble(MIN_RANDOM_LINE_VALUE,
                        MAX_RANDOM_LINE_VALUE),
                randomizer.nextDouble(MIN_RANDOM_LINE_VALUE,
                        MAX_RANDOM_LINE_VALUE));
        final Line2D line2D3 = new Line2D(randomizer.nextDouble(MIN_RANDOM_LINE_VALUE,
                MAX_RANDOM_LINE_VALUE),
                randomizer.nextDouble(MIN_RANDOM_LINE_VALUE,
                        MAX_RANDOM_LINE_VALUE),
                randomizer.nextDouble(MIN_RANDOM_LINE_VALUE,
                        MAX_RANDOM_LINE_VALUE));
        final Line2D line2D4 = new Line2D(randomizer.nextDouble(MIN_RANDOM_LINE_VALUE,
                MAX_RANDOM_LINE_VALUE),
                randomizer.nextDouble(MIN_RANDOM_LINE_VALUE,
                        MAX_RANDOM_LINE_VALUE),
                randomizer.nextDouble(MIN_RANDOM_LINE_VALUE,
                        MAX_RANDOM_LINE_VALUE));

        final Plane plane1 = camera1.backProject(line2D1);
        Plane plane2 = camera1.backProject(line2D2);
        final Plane plane3 = camera1.backProject(line2D3);
        final Plane plane4 = camera1.backProject(line2D4);

        final List<Line2D> lines2D = new ArrayList<>(N_CORRESPONDENCES);
        lines2D.add(line2D1);
        lines2D.add(line2D2);
        lines2D.add(line2D3);
        lines2D.add(line2D4);

        final List<Plane> planes = camera1.backProjectLines(lines2D);
        final List<Plane> planes2 = new ArrayList<>();
        camera1.backProjectLines(lines2D, planes2);

        assertEquals(planes, planes2);

        final PinholeCamera camera2 = new PinholeCamera();
        camera2.setFromLineAndPlaneCorrespondences(plane1, plane2, plane3, plane4,
                line2D1, line2D2, line2D3, line2D4);
        camera2.decompose();

        final DLTLinePlaneCorrespondencePinholeCameraEstimator estimator =
                new DLTLinePlaneCorrespondencePinholeCameraEstimator(planes,
                        lines2D);
        estimator.setLMSESolutionAllowed(false);
        final PinholeCamera camera3 = estimator.estimate();
        camera3.decompose();

        final PinholeCameraIntrinsicParameters intrinsic2 = camera2.getIntrinsicParameters();
        final PinholeCameraIntrinsicParameters intrinsic3 = camera3.getIntrinsicParameters();

        assertEquals(intrinsic2.getHorizontalFocalLength(),
                intrinsic3.getHorizontalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic2.getVerticalFocalLength(),
                intrinsic3.getVerticalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic2.getSkewness(), intrinsic3.getSkewness(),
                ABSOLUTE_ERROR);
        assertEquals(intrinsic2.getHorizontalPrincipalPoint(),
                intrinsic3.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
        assertEquals(intrinsic2.getVerticalPrincipalPoint(),
                intrinsic3.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

        final Rotation3D rotation2 = camera2.getCameraRotation();
        final Rotation3D rotation3 = camera3.getCameraRotation();

        assertTrue(rotation2.equals(rotation3, ABSOLUTE_ERROR));

        final Point3D cameraCenter2 = camera2.getCameraCenter();
        final Point3D cameraCenter3 = camera3.getCameraCenter();

        assertTrue(cameraCenter2.equals(cameraCenter3, ABSOLUTE_ERROR));

        // Force CameraException by repeating correspondences and creating a
        // degeneracy
        line2D2 = line2D1;
        plane2 = camera1.backProject(line2D2);
        try {
            camera2.setFromLineAndPlaneCorrespondences(plane1, plane2, plane3, plane4,
                    line2D1, line2D4, line2D3, line2D4);
            fail("CameraException expected but not thrown");
        } catch (final CameraException ignore) {
        }
    }
}
