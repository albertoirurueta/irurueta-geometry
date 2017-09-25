/*
 * Copyright (C) 2017 Alberto Irurueta Carro (alberto@irurueta.com)
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

package com.irurueta.geometry.sfm;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.SingularValueDecomposer;
import com.irurueta.geometry.*;
import com.irurueta.geometry.epipolar.FundamentalMatrix;
import com.irurueta.geometry.epipolar.InvalidPairOfCamerasException;
import com.irurueta.geometry.slam.*;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class AbsoluteOrientationConstantVelocityModelSlamSparseReconstructorTest {

    private static final double MIN_FOCAL_LENGTH_ESSENTIAL = 750.0;
    private static final double MAX_FOCAL_LENGTH_ESSENTIAL = 1500.0;

    private static final double MIN_ANGLE_DEGREES = -30.0;
    private static final double MAX_ANGLE_DEGREES = -15.0;

    private static final double MIN_CAMERA_SEPARATION_ESSENTIAL = 500.0;
    private static final double MAX_CAMERA_SEPARATION_ESSENTIAL = 1000.0;

    private static final int MIN_NUM_POINTS = 25;
    private static final int MAX_NUM_POINTS = 50;

    private static final double MIN_LAMBDA_ESSENTIAL = -1000.0;
    private static final double MAX_LAMBDA_ESSENTIAL = 1000.0;

    private static final int TIMES = 500;
    private static final int MAX_TRIES = 5000;

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-3;

    private static final int MAX_CALIBRATION_SAMPLES = 10000;

    //conversion from milliseconds to nanoseconds
    private static final int MILLIS_TO_NANOS = 1000000;

    //time between samples expressed in nanoseconds (a typical sensor in Android
    //delivers a sample every 20ms)
    private static final int DELTA_NANOS = 20000000; //0.02 seconds

    private static final float MIN_CALIBRATION_OFFSET = -1e-4f;
    private static final float MAX_CALIBRATION_OFFSET = 1e-4f;

    private static final double ACCELERATION_NOISE_STANDARD_DEVIATION = 1e-4;
    private static final double ANGULAR_SPEED_NOISE_STANDARD_DEVIATION = 1e-4;

    private static final int N_SENSOR_SAMPLES = 50;

    private int mViewCount = 0;
    private EstimatedFundamentalMatrix mEstimatedFundamentalMatrix;
    private EstimatedFundamentalMatrix mEstimatedFundamentalMatrix2;
    private EstimatedCamera mEstimatedMetricCamera1;
    private EstimatedCamera mEstimatedMetricCamera2;
    private EstimatedCamera mEstimatedMetricCamera3;
    private EstimatedCamera mEstimatedEuclideanCamera1;
    private EstimatedCamera mEstimatedEuclideanCamera2;
    private EstimatedCamera mEstimatedEuclideanCamera3;
    private List<ReconstructedPoint3D> mMetricReconstructedPoints;
    private List<ReconstructedPoint3D> mEuclideanReconstructedPoints;

    private double mScale;
    private double mScale2;

    private boolean mStarted;
    private boolean mFinished;
    private boolean mFailed;
    private boolean mCancelled;

    private long mTimestamp;

    public AbsoluteOrientationConstantVelocityModelSlamSparseReconstructorTest() { }

    @BeforeClass
    public static void setUpClass() { }

    @AfterClass
    public static void tearDownClass() { }

    @Before
    public void setUp() {
        mViewCount = 0;
        mEstimatedFundamentalMatrix = mEstimatedFundamentalMatrix2 = null;
        mEstimatedMetricCamera1 = mEstimatedMetricCamera2 =
                mEstimatedMetricCamera3 = null;
        mEstimatedEuclideanCamera1 = mEstimatedEuclideanCamera2 =
                mEstimatedEuclideanCamera3 = null;
        mMetricReconstructedPoints = null;
        mEuclideanReconstructedPoints = null;
        mStarted = mFinished = mFailed = mCancelled = false;
        mTimestamp = 0;
    }

    @After
    public void tearDown() { }

    @Test
    public void testConstructor() {
        AbsoluteOrientationConstantVelocityModelSlamSparseReconstructorConfiguration configuration =
                new AbsoluteOrientationConstantVelocityModelSlamSparseReconstructorConfiguration();
        AbsoluteOrientationConstantVelocityModelSlamSparseReconstructorListener listener =
                new AbsoluteOrientationConstantVelocityModelSlamSparseReconstructorListener() {
                    @Override
                    public boolean hasMoreViewsAvailable(
                            AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor) {
                        return false;
                    }

                    @Override
                    public void onRequestSamplesForCurrentView(
                            AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor,
                            int viewId, List<Sample2D> samples) { }

                    @Override
                    public void onSamplesAccepted(
                            AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor, int viewId,
                            List<Sample2D> samples) { }

                    @Override
                    public void onSamplesRejected(
                            AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor, int viewId,
                            List<Sample2D> samples) { }

                    @Override
                    public void onRequestMatches(
                            AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor,
                            List<Sample2D> samples1, List<Sample2D> samples2,
                            int viewId1, int viewId2,
                            List<MatchedSamples> matches) { }

                    @Override
                    public void onFundamentalMatrixEstimated(
                            AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor,
                            EstimatedFundamentalMatrix estimatedFundamentalMatrix) { }

                    @Override
                    public void onMetricCameraEstimated(
                            AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor,
                            int previousViewId, int currentViewId,
                            EstimatedCamera previousCamera,
                            EstimatedCamera currentCamera) { }

                    @Override
                    public void onMetricReconstructedPointsEstimated(
                            AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor,
                            List<MatchedSamples> matches,
                            List<ReconstructedPoint3D> points) { }

                    @Override
                    public void onEuclideanCameraEstimated(
                            AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor,
                            int previousViewId, int currentViewId, double scale,
                            EstimatedCamera previousCamera,
                            EstimatedCamera currentCamera) { }

                    @Override
                    public void onEuclideanReconstructedPointsEstimated(
                            AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor,
                            double scale, List<ReconstructedPoint3D> points) { }

                    @Override
                    public void onStart(
                            AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor) { }

                    @Override
                    public void onFinish(
                            AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor) { }

                    @Override
                    public void onCancel(
                            AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor) { }

                    @Override
                    public void onFail(
                            AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor) { }
                };

        //constructor with listener
        AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor =
                new AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor(listener);

        //check default values
        assertNotNull(reconstructor.getConfiguration());
        assertSame(reconstructor.getListener(), listener);
        assertFalse(reconstructor.isRunning());
        assertFalse(reconstructor.isCancelled());
        assertFalse(reconstructor.hasFailed());
        assertFalse(reconstructor.isFinished());
        assertEquals(reconstructor.getViewCount(), 0);
        assertNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
        assertNull(reconstructor.getCurrentMetricEstimatedCamera());
        assertNull(reconstructor.getPreviousMetricEstimatedCamera());
        assertNull(reconstructor.getCurrentEuclideanEstimatedCamera());
        assertNull(reconstructor.getPreviousEuclideanEstimatedCamera());
        assertNull(reconstructor.getActiveMetricReconstructedPoints());
        assertNull(reconstructor.getActiveEuclideanReconstructedPoints());

        //constructor with configuration and listener
        reconstructor = new AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor(configuration, listener);

        //check default values
        assertSame(reconstructor.getConfiguration(), configuration);
        assertSame(reconstructor.getListener(), listener);
        assertFalse(reconstructor.isRunning());
        assertFalse(reconstructor.isCancelled());
        assertFalse(reconstructor.hasFailed());
        assertFalse(reconstructor.isFinished());
        assertEquals(reconstructor.getViewCount(), 0);
        assertNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
        assertNull(reconstructor.getCurrentMetricEstimatedCamera());
        assertNull(reconstructor.getPreviousMetricEstimatedCamera());
        assertNull(reconstructor.getCurrentEuclideanEstimatedCamera());
        assertNull(reconstructor.getPreviousEuclideanEstimatedCamera());
        assertNull(reconstructor.getActiveMetricReconstructedPoints());
        assertNull(reconstructor.getActiveEuclideanReconstructedPoints());
    }

    @Test
    public void testGeneralPointsEssentialWithConstantAccelerationAndRotationWithoutNoiseTwoViews()
            throws InvalidPairOfCamerasException, AlgebraException,
            CameraException,
            com.irurueta.geometry.estimators.NotReadyException,
            com.irurueta.geometry.NotAvailableException, RotationException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);

            AbsoluteOrientationConstantVelocityModelSlamSparseReconstructorConfiguration configuration =
                    new AbsoluteOrientationConstantVelocityModelSlamSparseReconstructorConfiguration();
            configuration.setInitialCamerasEstimatorMethod(
                    InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);

            float accelerationOffsetX = 0.0f;
            float accelerationOffsetY = 0.0f;
            float accelerationOffsetZ = 0.0f;

            float angularOffsetX = 0.0f;
            float angularOffsetY = 0.0f;
            float angularOffsetZ = 0.0f;

            AbsoluteOrientationConstantVelocityModelSlamCalibrator calibrator = createFinishedCalibrator(
                    accelerationOffsetX, accelerationOffsetY,
                    accelerationOffsetZ, angularOffsetX, angularOffsetY,
                    angularOffsetZ, noiseRandomizer);
            AbsoluteOrientationConstantVelocityModelSlamCalibrationData calibrationData
                    = calibrator.getCalibrationData();
            configuration.setCalibrationData(calibrationData);

            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH_ESSENTIAL,
                    MAX_FOCAL_LENGTH_ESSENTIAL);
            double aspectRatio = configuration.getInitialCamerasAspectRatio();
            double skewness = 0.0;
            double principalPoint = 0.0;

            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength, focalLength,
                            principalPoint, principalPoint, skewness);
            intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);

            configuration.setInitialIntrinsic1(intrinsic);
            configuration.setInitialIntrinsic2(intrinsic);

            double alphaEuler1 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler1 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler1 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1,
                    betaEuler1, gammaEuler1);
            MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2,
                    betaEuler2, gammaEuler2);
            AxisRotation3D axisRotation2 = new AxisRotation3D(
                    rotation1.inverseRotationAndReturnNew().
                            combineAndReturnNew(rotation2));

            double axisX = axisRotation2.getAxisX();
            double axisY = axisRotation2.getAxisY();
            double axisZ = axisRotation2.getAxisZ();
            double angle = axisRotation2.getRotationAngle();

            AxisRotation3D diffRotation = new AxisRotation3D(axisX, axisY,
                    axisZ, angle / N_SENSOR_SAMPLES);
            final Quaternion diffQuaternion = new Quaternion(diffRotation);

            //angular speeds (roll, pitch, yaw) on x, y, z axes
            double[] angularSpeeds = diffQuaternion.toEulerAngles();
            final double angularSpeedX = angularSpeeds[0];
            final double angularSpeedY = angularSpeeds[1];
            final double angularSpeedZ = angularSpeeds[2];
            Quaternion diffRotation2 = new Quaternion(angularSpeedX,
                    angularSpeedY, angularSpeedZ);

            //number of samples (50 samples * 0.02 s/sample = 1 second)
            MatrixRotation3D rotation2b = new MatrixRotation3D(rotation1);
            MatrixRotation3D rotation2c = new MatrixRotation3D(rotation1);
            for (int s = 0; s < N_SENSOR_SAMPLES; s++) {
                rotation2b.combine(diffRotation);
                rotation2c.combine(diffRotation2);
            }

            //check that rotations created by composing sensor samples are
            //equal to the original one
            assertTrue(rotation2.equals(rotation2b, ABSOLUTE_ERROR));
            assertTrue(rotation2.equals(rotation2c, ABSOLUTE_ERROR));

            double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);

            EuclideanTransformation3D rotationTransformation =
                    new EuclideanTransformation3D(rotation1);
            Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);
            center1 = rotationTransformation.transformAndReturnNew(center1);
            center2 = rotationTransformation.transformAndReturnNew(center2);

            double baseline = center1.distanceTo(center2);

            final double accelerationX, accelerationY, accelerationZ;

            //s = 0.5*a*t^2 --> a = 2*s/t^2
            //assuming t = 1 second (50 samples * 0.02 s/sample = 1 second)
            accelerationX = accelerationY = accelerationZ
                    = 2 * cameraSeparation;

            PinholeCamera camera1 = new PinholeCamera(intrinsic, rotation1,
                    center1);
            PinholeCamera camera2 = new PinholeCamera(intrinsic, rotation2,
                    center2);

            FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(
                    camera1, camera2);

            //create 3D points laying in front of both cameras

            //1st find an approximate central point by intersecting the axis
            //planes of both cameras
            Plane horizontalPlane1 = camera1.getHorizontalAxisPlane();
            Plane verticalPlane1 = camera1.getVerticalAxisPlane();
            Plane horizontalPlane2 = camera2.getHorizontalAxisPlane();
            Plane verticalPlane2 = camera2.getVerticalAxisPlane();
            Matrix planesIntersectionMatrix = new Matrix(
                    Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
            planesIntersectionMatrix.setElementAt(0, 0, verticalPlane1.getA());
            planesIntersectionMatrix.setElementAt(0, 1, verticalPlane1.getB());
            planesIntersectionMatrix.setElementAt(0, 2, verticalPlane1.getC());
            planesIntersectionMatrix.setElementAt(0, 3, verticalPlane1.getD());

            planesIntersectionMatrix.setElementAt(1, 0,
                    horizontalPlane1.getA());
            planesIntersectionMatrix.setElementAt(1, 1,
                    horizontalPlane1.getB());
            planesIntersectionMatrix.setElementAt(1, 2,
                    horizontalPlane1.getC());
            planesIntersectionMatrix.setElementAt(1, 3,
                    horizontalPlane1.getD());

            planesIntersectionMatrix.setElementAt(2, 0, verticalPlane2.getA());
            planesIntersectionMatrix.setElementAt(2, 1, verticalPlane2.getB());
            planesIntersectionMatrix.setElementAt(2, 2, verticalPlane2.getC());
            planesIntersectionMatrix.setElementAt(2, 3, verticalPlane2.getD());

            planesIntersectionMatrix.setElementAt(3, 0,
                    horizontalPlane2.getA());
            planesIntersectionMatrix.setElementAt(3, 1,
                    horizontalPlane2.getB());
            planesIntersectionMatrix.setElementAt(3, 2,
                    horizontalPlane2.getC());
            planesIntersectionMatrix.setElementAt(3, 3,
                    horizontalPlane2.getD());

            SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    planesIntersectionMatrix);
            decomposer.decompose();
            Matrix v = decomposer.getV();
            HomogeneousPoint3D centralCommonPoint = new HomogeneousPoint3D(
                    v.getElementAt(0, 3),
                    v.getElementAt(1, 3),
                    v.getElementAt(2, 3),
                    v.getElementAt(3, 3));

            double lambdaX, lambdaY, lambdaZ;

            final int numPoints = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);

            InhomogeneousPoint3D point3D;
            List<InhomogeneousPoint3D> points3D =
                    new ArrayList<>();
            Point2D projectedPoint1, projectedPoint2;
            final List<Point2D> projectedPoints1 = new ArrayList<>();
            final List<Point2D> projectedPoints2 = new ArrayList<>();
            boolean front1, front2, maxTriesReached = false;
            for (int i = 0; i < numPoints; i++) {
                //generate points and ensure they lie in front of both cameras
                int numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(
                            MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaY = randomizer.nextDouble(
                            MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaZ = randomizer.nextDouble(
                            MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);

                    point3D = new InhomogeneousPoint3D(
                            centralCommonPoint.getInhomX() + lambdaX,
                            centralCommonPoint.getInhomY() + lambdaY,
                            centralCommonPoint.getInhomZ() + lambdaZ);

                    front1 = camera1.isPointInFrontOfCamera(point3D);
                    front2 = camera2.isPointInFrontOfCamera(point3D);
                    if (numTry > MAX_TRIES) {
                        maxTriesReached = true;
                        break;
                    }
                    numTry++;
                } while(!front1 || !front2);

                if (maxTriesReached) break;

                points3D.add(point3D);

                //check that 3D point is in front of both cameras
                //noinspection all
                assertTrue(front1);
                //noinspection all
                assertTrue(front2);

                //project 3D point into both cameras
                projectedPoint1 = new InhomogeneousPoint2D();
                camera1.project(point3D, projectedPoint1);
                projectedPoints1.add(projectedPoint1);

                projectedPoint2 = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2);
                projectedPoints2.add(projectedPoint2);
            }

            if (maxTriesReached) continue;

            AbsoluteOrientationConstantVelocityModelSlamSparseReconstructorListener listener =
                    new AbsoluteOrientationConstantVelocityModelSlamSparseReconstructorListener() {
                        @Override
                        public boolean hasMoreViewsAvailable(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor) {
                            return mViewCount < 2;
                        }

                        @Override
                        public void onRequestSamplesForCurrentView(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor,
                                int viewId, List<Sample2D> samples) {

                            samples.clear();

                            Sample2D sample;
                            if (mViewCount == 0) {
                                //first view
                                for (int i = 0; i < numPoints; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints1.get(i));
                                    sample.setViewId(viewId);
                                    samples.add(sample);
                                }

                            } else {
                                //second view
                                for (int i = 0; i < numPoints; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints2.get(i));
                                    sample.setViewId(viewId);
                                    samples.add(sample);
                                }

                                //assume the following accelerator and gyroscope samples
                                //are obtained during a period of 1 second between 1st
                                //and 2nd view (50 samples * 0.02 s/sample = 1 second)
                                mTimestamp = 0;
                                Quaternion orientation = new Quaternion(rotation1);
                                for (int s = 0; s < N_SENSOR_SAMPLES; s++) {
                                    reconstructor.updateAccelerometerSample(mTimestamp,
                                            (float) accelerationX, (float) accelerationY,
                                            (float) accelerationZ);
                                    reconstructor.updateGyroscopeSample(mTimestamp,
                                            (float) angularSpeedX, (float) angularSpeedY,
                                            (float) angularSpeedZ);
                                    reconstructor.updateOrientationSample(mTimestamp,
                                            orientation);
                                    //update orientation
                                    orientation.combine(diffQuaternion);
                                    mTimestamp += DELTA_NANOS;
                                }
                            }
                        }

                        @Override
                        public void onSamplesAccepted(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor,
                                int viewId, List<Sample2D> samples) {
                            mViewCount++;
                        }

                        @Override
                        public void onSamplesRejected(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor,
                                int viewId, List<Sample2D> samples) { }

                        @Override
                        public void onRequestMatches(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor,
                                List<Sample2D> samples1,
                                List<Sample2D> samples2, int viewId1, int viewId2,
                                List<MatchedSamples> matches) {
                            matches.clear();

                            MatchedSamples match;
                            for (int i = 0; i < numPoints; i++) {
                                match = new MatchedSamples();
                                match.setSamples(new Sample2D[]{
                                        samples1.get(i), samples2.get(i)
                                });
                                match.setViewIds(new int[]{viewId1, viewId2});
                                matches.add(match);
                            }
                        }

                        @Override
                        public void onFundamentalMatrixEstimated(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor,
                                EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                            mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                        }

                        @Override
                        public void onMetricCameraEstimated(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor,
                                int previousViewId, int currentViewId,
                                EstimatedCamera previousCamera,
                                EstimatedCamera currentCamera) {
                            mEstimatedMetricCamera1 = previousCamera;
                            mEstimatedMetricCamera2 = currentCamera;
                        }

                        @Override
                        public void onMetricReconstructedPointsEstimated(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor,
                                List<MatchedSamples> matches,
                                List<ReconstructedPoint3D> points) {
                            mMetricReconstructedPoints = points;
                        }

                        @Override
                        public void onEuclideanCameraEstimated(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor,
                                int previousViewId, int currentViewId, double scale,
                                EstimatedCamera previousCamera,
                                EstimatedCamera currentCamera) {
                            mEstimatedEuclideanCamera1 = previousCamera;
                            mEstimatedEuclideanCamera2 = currentCamera;
                            mScale = scale;
                        }

                        @Override
                        public void onEuclideanReconstructedPointsEstimated(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor,
                                double scale, List<ReconstructedPoint3D> points) {
                            mEuclideanReconstructedPoints = points;
                            mScale = scale;
                        }

                        @Override
                        public void onStart(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor) {
                            mStarted = true;
                        }

                        @Override
                        public void onFinish(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor) {
                            mFinished = true;
                        }

                        @Override
                        public void onCancel(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor) {
                            mCancelled = true;
                        }

                        @Override
                        public void onFail(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor) {
                            mFailed = true;
                        }
                    };

            AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor =
                    new AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor(configuration, listener);

            //check initial values
            reset();
            assertFalse(mStarted);
            assertFalse(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertFalse(reconstructor.isFinished());

            reconstructor.start();

            //check correctness
            assertTrue(mStarted);
            assertTrue(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertTrue(reconstructor.isFinished());

            //check that estimated fundamental matrix is correct
            fundamentalMatrix.normalize();
            mEstimatedFundamentalMatrix.getFundamentalMatrix().normalize();

            //matrices are equal up to scale
            if (!fundamentalMatrix.getInternalMatrix().equals(
                    mEstimatedFundamentalMatrix.getFundamentalMatrix().
                            getInternalMatrix(), ABSOLUTE_ERROR) &&
                    !fundamentalMatrix.getInternalMatrix().
                            multiplyByScalarAndReturnNew(-1).equals(
                            mEstimatedFundamentalMatrix.getFundamentalMatrix().
                                    getInternalMatrix(), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundamentalMatrix.getInternalMatrix().equals(
                    mEstimatedFundamentalMatrix.getFundamentalMatrix().
                            getInternalMatrix(), ABSOLUTE_ERROR) ||
                    fundamentalMatrix.getInternalMatrix().
                            multiplyByScalarAndReturnNew(-1).equals(
                            mEstimatedFundamentalMatrix.getFundamentalMatrix().
                                    getInternalMatrix(), ABSOLUTE_ERROR));

            //check that reconstructed points are in a metric stratum (up to a
            //certain scale)
            PinholeCamera estimatedMetricCamera1 = mEstimatedMetricCamera1.getCamera();
            PinholeCamera estimatedMetricCamera2 = mEstimatedMetricCamera2.getCamera();
            assertNotSame(mEstimatedMetricCamera1, mEstimatedEuclideanCamera1);
            assertNotSame(mEstimatedMetricCamera2, mEstimatedEuclideanCamera2);

            PinholeCamera estimatedEuclideanCamera1 = mEstimatedEuclideanCamera1.getCamera();
            PinholeCamera estimatedEuclideanCamera2 = mEstimatedEuclideanCamera2.getCamera();

            estimatedMetricCamera1.decompose();
            estimatedMetricCamera2.decompose();

            estimatedEuclideanCamera1.decompose();
            estimatedEuclideanCamera2.decompose();

            assertNotSame(mMetricReconstructedPoints, mEuclideanReconstructedPoints);

            List<Point3D> metricReconstructedPoints3D = new ArrayList<>();
            List<Point3D> euclideanReconstructedPoints3D = new ArrayList<>();
            for (int i = 0; i < numPoints; i++) {
                metricReconstructedPoints3D.add(
                        mMetricReconstructedPoints.get(i).getPoint());
                euclideanReconstructedPoints3D.add(
                        mEuclideanReconstructedPoints.get(i).getPoint());
            }

            //check that all points are in front of both cameras
            for (int i = 0; i < numPoints; i++) {
                Point3D p = metricReconstructedPoints3D.get(i);
                Point3D pe = euclideanReconstructedPoints3D.get(i);

                assertTrue(estimatedMetricCamera1.isPointInFrontOfCamera(p));
                assertTrue(estimatedMetricCamera2.isPointInFrontOfCamera(p));

                assertTrue(estimatedEuclideanCamera1.isPointInFrontOfCamera(pe));
                assertTrue(estimatedEuclideanCamera2.isPointInFrontOfCamera(pe));
            }

            PinholeCameraIntrinsicParameters euclideanIntrinsic1 =
                    estimatedEuclideanCamera1.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters euclideanIntrinsic2 =
                    estimatedEuclideanCamera2.getIntrinsicParameters();

            Rotation3D euclideanRotation1 = estimatedEuclideanCamera1.getCameraRotation();
            Rotation3D euclideanRotation2 = estimatedEuclideanCamera2.getCameraRotation();

            assertEquals(euclideanIntrinsic1.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertEquals(euclideanIntrinsic2.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertTrue(euclideanRotation1.asInhomogeneousMatrix().equals(
                    rotation1.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
            assertTrue(euclideanRotation2.asInhomogeneousMatrix().equals(
                    rotation2.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

            //check that points are correct (after scale correction)

            MetricTransformation3D scaleAndOrientationTransformation
                    = new MetricTransformation3D(mScale);
            scaleAndOrientationTransformation.setRotation(rotation1.inverseRotationAndReturnNew());

            int numValidPoints = 0;
            for (int i = 0; i < numPoints; i++) {
                Point3D point = points3D.get(i);
                Point3D euclideanPoint = euclideanReconstructedPoints3D.get(i);

                //check metric points
                Point3D rescaledPoint = Point3D.create();
                scaleAndOrientationTransformation.transform(metricReconstructedPoints3D.get(i),
                        rescaledPoint);

                //euclidean and rescaled points match
                assertTrue(euclideanPoint.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR));

                rescaledPoint.setInhomogeneousCoordinates(
                        rescaledPoint.getInhomX() * baseline / mScale,
                        rescaledPoint.getInhomY() * baseline / mScale,
                        rescaledPoint.getInhomZ() * baseline / mScale);
                if (point.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR)) {
                    numValidPoints++;
                }
            }

            if (numValidPoints == 0) {
                continue;
            }

            numValid++;

            if (numValid > 0) {
                break;
            }
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testGeneralPointsEssentialWithConstantAccelerationAndRotationWithNoiseTwoViews()
            throws InvalidPairOfCamerasException, AlgebraException,
            CameraException,
            com.irurueta.geometry.estimators.NotReadyException,
            com.irurueta.geometry.NotAvailableException, RotationException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer offsetRandomizer = new UniformRandomizer(
                    new Random());
            GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);

            AbsoluteOrientationConstantVelocityModelSlamSparseReconstructorConfiguration configuration =
                    new AbsoluteOrientationConstantVelocityModelSlamSparseReconstructorConfiguration();
            configuration.setInitialCamerasEstimatorMethod(
                    InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);

            float accelerationOffsetX = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            float accelerationOffsetY = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            float accelerationOffsetZ = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);

            float angularOffsetX = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            float angularOffsetY = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            float angularOffsetZ = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);

            AbsoluteOrientationConstantVelocityModelSlamCalibrator calibrator = createFinishedCalibrator(
                    accelerationOffsetX, accelerationOffsetY,
                    accelerationOffsetZ, angularOffsetX, angularOffsetY,
                    angularOffsetZ, noiseRandomizer);
            AbsoluteOrientationConstantVelocityModelSlamCalibrationData calibrationData
                    = calibrator.getCalibrationData();
            configuration.setCalibrationData(calibrationData);

            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH_ESSENTIAL,
                    MAX_FOCAL_LENGTH_ESSENTIAL);
            double aspectRatio = configuration.getInitialCamerasAspectRatio();
            double skewness = 0.0;
            double principalPoint = 0.0;

            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength, focalLength,
                            principalPoint, principalPoint, skewness);
            intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);

            configuration.setInitialIntrinsic1(intrinsic);
            configuration.setInitialIntrinsic2(intrinsic);

            double alphaEuler1 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler1 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler1 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1,
                    betaEuler1, gammaEuler1);
            MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2,
                    betaEuler2, gammaEuler2);
            AxisRotation3D axisRotation2 = new AxisRotation3D(
                    rotation1.inverseRotationAndReturnNew().
                            combineAndReturnNew(rotation2));

            double axisX = axisRotation2.getAxisX();
            double axisY = axisRotation2.getAxisY();
            double axisZ = axisRotation2.getAxisZ();
            double angle = axisRotation2.getRotationAngle();

            AxisRotation3D diffRotation = new AxisRotation3D(axisX, axisY,
                    axisZ, angle / N_SENSOR_SAMPLES);
            final Quaternion diffQuaternion = new Quaternion(diffRotation);

            //angular speeds (roll, pitch, yaw) on x, y, z axes
            double[] angularSpeeds = diffQuaternion.toEulerAngles();
            final double angularSpeedX = angularSpeeds[0];
            final double angularSpeedY = angularSpeeds[1];
            final double angularSpeedZ = angularSpeeds[2];
            Quaternion diffRotation2 = new Quaternion(angularSpeedX,
                    angularSpeedY, angularSpeedZ);

            //number of samples (50 samples * 0.02 s/sample = 1 second)
            MatrixRotation3D rotation2b = new MatrixRotation3D(rotation1);
            MatrixRotation3D rotation2c = new MatrixRotation3D(rotation1);
            for (int s = 0; s < N_SENSOR_SAMPLES; s++) {
                rotation2b.combine(diffRotation);
                rotation2c.combine(diffRotation2);
            }

            //check that rotations created by composing sensor samples are
            //equal to the original one
            assertTrue(rotation2.equals(rotation2b, ABSOLUTE_ERROR));
            assertTrue(rotation2.equals(rotation2c, ABSOLUTE_ERROR));

            double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);

            EuclideanTransformation3D rotationTransformation =
                    new EuclideanTransformation3D(rotation1);
            Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);
            center1 = rotationTransformation.transformAndReturnNew(center1);
            center2 = rotationTransformation.transformAndReturnNew(center2);

            double baseline = center1.distanceTo(center2);

            final double accelerationX, accelerationY, accelerationZ;

            //s = 0.5*a*t^2 --> a = 2*s/t^2
            //assuming t = 1 second (50 samples * 0.02 s/sample = 1 second)
            accelerationX = accelerationY = accelerationZ
                    = 2 * cameraSeparation;

            PinholeCamera camera1 = new PinholeCamera(intrinsic, rotation1,
                    center1);
            PinholeCamera camera2 = new PinholeCamera(intrinsic, rotation2,
                    center2);

            FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(
                    camera1, camera2);

            //create 3D points laying in front of both cameras

            //1st find an approximate central point by intersecting the axis
            //planes of both cameras
            Plane horizontalPlane1 = camera1.getHorizontalAxisPlane();
            Plane verticalPlane1 = camera1.getVerticalAxisPlane();
            Plane horizontalPlane2 = camera2.getHorizontalAxisPlane();
            Plane verticalPlane2 = camera2.getVerticalAxisPlane();
            Matrix planesIntersectionMatrix = new Matrix(
                    Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
            planesIntersectionMatrix.setElementAt(0, 0, verticalPlane1.getA());
            planesIntersectionMatrix.setElementAt(0, 1, verticalPlane1.getB());
            planesIntersectionMatrix.setElementAt(0, 2, verticalPlane1.getC());
            planesIntersectionMatrix.setElementAt(0, 3, verticalPlane1.getD());

            planesIntersectionMatrix.setElementAt(1, 0,
                    horizontalPlane1.getA());
            planesIntersectionMatrix.setElementAt(1, 1,
                    horizontalPlane1.getB());
            planesIntersectionMatrix.setElementAt(1, 2,
                    horizontalPlane1.getC());
            planesIntersectionMatrix.setElementAt(1, 3,
                    horizontalPlane1.getD());

            planesIntersectionMatrix.setElementAt(2, 0, verticalPlane2.getA());
            planesIntersectionMatrix.setElementAt(2, 1, verticalPlane2.getB());
            planesIntersectionMatrix.setElementAt(2, 2, verticalPlane2.getC());
            planesIntersectionMatrix.setElementAt(2, 3, verticalPlane2.getD());

            planesIntersectionMatrix.setElementAt(3, 0,
                    horizontalPlane2.getA());
            planesIntersectionMatrix.setElementAt(3, 1,
                    horizontalPlane2.getB());
            planesIntersectionMatrix.setElementAt(3, 2,
                    horizontalPlane2.getC());
            planesIntersectionMatrix.setElementAt(3, 3,
                    horizontalPlane2.getD());

            SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    planesIntersectionMatrix);
            decomposer.decompose();
            Matrix v = decomposer.getV();
            HomogeneousPoint3D centralCommonPoint = new HomogeneousPoint3D(
                    v.getElementAt(0, 3),
                    v.getElementAt(1, 3),
                    v.getElementAt(2, 3),
                    v.getElementAt(3, 3));

            double lambdaX, lambdaY, lambdaZ;

            final int numPoints = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);

            InhomogeneousPoint3D point3D;
            List<InhomogeneousPoint3D> points3D =
                    new ArrayList<>();
            Point2D projectedPoint1, projectedPoint2;
            final List<Point2D> projectedPoints1 = new ArrayList<>();
            final List<Point2D> projectedPoints2 = new ArrayList<>();
            boolean front1, front2, maxTriesReached = false;
            for (int i = 0; i < numPoints; i++) {
                //generate points and ensure they lie in front of both cameras
                int numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(
                            MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaY = randomizer.nextDouble(
                            MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaZ = randomizer.nextDouble(
                            MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);

                    point3D = new InhomogeneousPoint3D(
                            centralCommonPoint.getInhomX() + lambdaX,
                            centralCommonPoint.getInhomY() + lambdaY,
                            centralCommonPoint.getInhomZ() + lambdaZ);

                    front1 = camera1.isPointInFrontOfCamera(point3D);
                    front2 = camera2.isPointInFrontOfCamera(point3D);
                    if (numTry > MAX_TRIES) {
                        maxTriesReached = true;
                        break;
                    }
                    numTry++;
                } while(!front1 || !front2);

                if (maxTriesReached) break;

                points3D.add(point3D);

                //check that 3D point is in front of both cameras
                //noinspection all
                assertTrue(front1);
                //noinspection all
                assertTrue(front2);

                //project 3D point into both cameras
                projectedPoint1 = new InhomogeneousPoint2D();
                camera1.project(point3D, projectedPoint1);
                projectedPoints1.add(projectedPoint1);

                projectedPoint2 = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2);
                projectedPoints2.add(projectedPoint2);
            }

            if (maxTriesReached) continue;

            final GaussianRandomizer accelerationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);

            AbsoluteOrientationConstantVelocityModelSlamSparseReconstructorListener listener =
                    new AbsoluteOrientationConstantVelocityModelSlamSparseReconstructorListener() {
                        @Override
                        public boolean hasMoreViewsAvailable(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor) {
                            return mViewCount < 2;
                        }

                        @Override
                        public void onRequestSamplesForCurrentView(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor,
                                int viewId, List<Sample2D> samples) {

                            samples.clear();

                            Sample2D sample;
                            if (mViewCount == 0) {
                                //first view
                                for (int i = 0; i < numPoints; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints1.get(i));
                                    sample.setViewId(viewId);
                                    samples.add(sample);
                                }

                            } else {
                                //second view
                                for (int i = 0; i < numPoints; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints2.get(i));
                                    sample.setViewId(viewId);
                                    samples.add(sample);
                                }

                                //assume the following accelerator and gyroscope samples
                                //are obtained during a period of 1 second between 1st
                                //and 2nd view (50 samples * 0.02 s/sample = 1 second)
                                mTimestamp = 0;
                                float noiseAccelerationX, noiseAccelerationY,
                                        noiseAccelerationZ;
                                float noiseAngularSpeedX, noiseAngularSpeedY,
                                        noiseAngularSpeedZ;

                                float accelerationWithNoiseX, accelerationWithNoiseY,
                                        accelerationWithNoiseZ;
                                float angularSpeedWithNoiseX, angularSpeedWithNoiseY,
                                        angularSpeedWithNoiseZ;

                                float[] accelerationWithNoise = new float[3];
                                float[] angularSpeedWithNoise = new float[3];

                                Quaternion orientation = new Quaternion(rotation1);
                                for (int s = 0; s < N_SENSOR_SAMPLES; s++) {
                                    noiseAccelerationX =
                                            accelerationRandomizer.nextFloat();
                                    noiseAccelerationY =
                                            accelerationRandomizer.nextFloat();
                                    noiseAccelerationZ =
                                            accelerationRandomizer.nextFloat();

                                    noiseAngularSpeedX =
                                            angularSpeedRandomizer.nextFloat();
                                    noiseAngularSpeedY =
                                            angularSpeedRandomizer.nextFloat();
                                    noiseAngularSpeedZ =
                                            angularSpeedRandomizer.nextFloat();

                                    accelerationWithNoiseX = (float)accelerationX +
                                            noiseAccelerationX;
                                    accelerationWithNoiseY = (float)accelerationY +
                                            noiseAccelerationY;
                                    accelerationWithNoiseZ = (float)accelerationZ +
                                            noiseAccelerationZ;
                                    accelerationWithNoise[0] = accelerationWithNoiseX;
                                    accelerationWithNoise[1] = accelerationWithNoiseY;
                                    accelerationWithNoise[2] = accelerationWithNoiseZ;

                                    angularSpeedWithNoiseX = (float)angularSpeedX +
                                            noiseAngularSpeedX;
                                    angularSpeedWithNoiseY = (float)angularSpeedY +
                                            noiseAngularSpeedY;
                                    angularSpeedWithNoiseZ = (float)angularSpeedZ +
                                            noiseAngularSpeedZ;
                                    angularSpeedWithNoise[0] = angularSpeedWithNoiseX;
                                    angularSpeedWithNoise[1] = angularSpeedWithNoiseY;
                                    angularSpeedWithNoise[2] = angularSpeedWithNoiseZ;

                                    reconstructor.updateAccelerometerSample(mTimestamp,
                                            accelerationWithNoise);
                                    reconstructor.updateGyroscopeSample(mTimestamp,
                                            angularSpeedWithNoise);
                                    reconstructor.updateOrientationSample(mTimestamp,
                                            orientation);
                                    //update orientation
                                    orientation.combine(diffQuaternion);
                                    mTimestamp += DELTA_NANOS;
                                }
                            }
                        }

                        @Override
                        public void onSamplesAccepted(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor, int viewId,
                                List<Sample2D> samples) {
                            mViewCount++;
                        }

                        @Override
                        public void onSamplesRejected(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor, int viewId,
                                List<Sample2D> samples) { }

                        @Override
                        public void onRequestMatches(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor,
                                List<Sample2D> samples1,
                                List<Sample2D> samples2, int viewId1, int viewId2,
                                List<MatchedSamples> matches) {
                            matches.clear();

                            MatchedSamples match;
                            for (int i = 0; i < numPoints; i++) {
                                match = new MatchedSamples();
                                match.setSamples(new Sample2D[]{
                                        samples1.get(i), samples2.get(i)
                                });
                                match.setViewIds(new int[]{viewId1, viewId2});
                                matches.add(match);
                            }
                        }

                        @Override
                        public void onFundamentalMatrixEstimated(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor,
                                EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                            mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                        }

                        @Override
                        public void onMetricCameraEstimated(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor,
                                int previousViewId, int currentViewId,
                                EstimatedCamera previousCamera,
                                EstimatedCamera currentCamera) {
                            mEstimatedMetricCamera1 = previousCamera;
                            mEstimatedMetricCamera2 = currentCamera;
                        }

                        @Override
                        public void onMetricReconstructedPointsEstimated(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor,
                                List<MatchedSamples> matches,
                                List<ReconstructedPoint3D> points) {
                            mMetricReconstructedPoints = points;
                        }

                        @Override
                        public void onEuclideanCameraEstimated(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor,
                                int previousViewId, int currentViewId, double scale,
                                EstimatedCamera previousCamera,
                                EstimatedCamera currentCamera) {
                            mEstimatedEuclideanCamera1 = previousCamera;
                            mEstimatedEuclideanCamera2 = currentCamera;
                            mScale = scale;
                        }

                        @Override
                        public void onEuclideanReconstructedPointsEstimated(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor,
                                double scale, List<ReconstructedPoint3D> points) {
                            mEuclideanReconstructedPoints = points;
                            mScale = scale;
                        }

                        @Override
                        public void onStart(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor) {
                            mStarted = true;
                        }

                        @Override
                        public void onFinish(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor) {
                            mFinished = true;
                        }

                        @Override
                        public void onCancel(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor) {
                            mCancelled = true;
                        }

                        @Override
                        public void onFail(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor) {
                            mFailed = true;
                        }
                    };

            AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor =
                    new AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor(configuration, listener);

            //check initial values
            reset();
            assertFalse(mStarted);
            assertFalse(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertFalse(reconstructor.isFinished());

            reconstructor.start();

            //check correctness
            assertTrue(mStarted);
            assertTrue(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertTrue(reconstructor.isFinished());

            //check that estimated fundamental matrix is correct
            fundamentalMatrix.normalize();
            mEstimatedFundamentalMatrix.getFundamentalMatrix().normalize();

            //matrices are equal up to scale
            if (!fundamentalMatrix.getInternalMatrix().equals(
                    mEstimatedFundamentalMatrix.getFundamentalMatrix().
                            getInternalMatrix(), ABSOLUTE_ERROR) &&
                    !fundamentalMatrix.getInternalMatrix().
                            multiplyByScalarAndReturnNew(-1).equals(
                            mEstimatedFundamentalMatrix.getFundamentalMatrix().
                                    getInternalMatrix(), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundamentalMatrix.getInternalMatrix().equals(
                    mEstimatedFundamentalMatrix.getFundamentalMatrix().
                            getInternalMatrix(), ABSOLUTE_ERROR) ||
                    fundamentalMatrix.getInternalMatrix().
                            multiplyByScalarAndReturnNew(-1).equals(
                            mEstimatedFundamentalMatrix.getFundamentalMatrix().
                                    getInternalMatrix(), ABSOLUTE_ERROR));

            //check that reconstructed points are in a metric stratum (up to a
            //certain scale)
            PinholeCamera estimatedMetricCamera1 = mEstimatedMetricCamera1.getCamera();
            PinholeCamera estimatedMetricCamera2 = mEstimatedMetricCamera2.getCamera();
            assertNotSame(mEstimatedMetricCamera1, mEstimatedEuclideanCamera1);
            assertNotSame(mEstimatedMetricCamera2, mEstimatedEuclideanCamera2);

            PinholeCamera estimatedEuclideanCamera1 = mEstimatedEuclideanCamera1.getCamera();
            PinholeCamera estimatedEuclideanCamera2 = mEstimatedEuclideanCamera2.getCamera();

            estimatedMetricCamera1.decompose();
            estimatedMetricCamera2.decompose();

            estimatedEuclideanCamera1.decompose();
            estimatedEuclideanCamera2.decompose();

            assertNotSame(mMetricReconstructedPoints, mEuclideanReconstructedPoints);

            List<Point3D> metricReconstructedPoints3D = new ArrayList<>();
            List<Point3D> euclideanReconstructedPoints3D = new ArrayList<>();
            for (int i = 0; i < numPoints; i++) {
                metricReconstructedPoints3D.add(
                        mMetricReconstructedPoints.get(i).getPoint());
                euclideanReconstructedPoints3D.add(
                        mEuclideanReconstructedPoints.get(i).getPoint());
            }

            //check that all points are in front of both cameras
            for (int i = 0; i < numPoints; i++) {
                Point3D p = metricReconstructedPoints3D.get(i);
                Point3D pe = euclideanReconstructedPoints3D.get(i);

                assertTrue(estimatedMetricCamera1.isPointInFrontOfCamera(p));
                assertTrue(estimatedMetricCamera2.isPointInFrontOfCamera(p));

                assertTrue(estimatedEuclideanCamera1.isPointInFrontOfCamera(pe));
                assertTrue(estimatedEuclideanCamera2.isPointInFrontOfCamera(pe));
            }

            PinholeCameraIntrinsicParameters euclideanIntrinsic1 =
                    estimatedEuclideanCamera1.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters euclideanIntrinsic2 =
                    estimatedEuclideanCamera2.getIntrinsicParameters();

            Rotation3D euclideanRotation1 = estimatedEuclideanCamera1.getCameraRotation();
            Rotation3D euclideanRotation2 = estimatedEuclideanCamera2.getCameraRotation();

            assertEquals(euclideanIntrinsic1.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertEquals(euclideanIntrinsic2.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertTrue(euclideanRotation1.asInhomogeneousMatrix().equals(
                    rotation1.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
            assertTrue(euclideanRotation2.asInhomogeneousMatrix().equals(
                    rotation2.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

            //check that points are correct (after scale correction)
            MetricTransformation3D scaleAndOrientationTransformation
                    = new MetricTransformation3D(mScale);
            scaleAndOrientationTransformation.setRotation(rotation1.inverseRotationAndReturnNew());

            int numValidPoints = 0;
            for (int i = 0; i < numPoints; i++) {
                Point3D point = points3D.get(i);
                Point3D euclideanPoint = euclideanReconstructedPoints3D.get(i);

                //check metric points
                Point3D rescaledPoint = Point3D.create();
                scaleAndOrientationTransformation.transform(metricReconstructedPoints3D.get(i),
                        rescaledPoint);

                //euclidean and rescaled points match
                assertTrue(euclideanPoint.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR));

                rescaledPoint.setInhomogeneousCoordinates(
                        rescaledPoint.getInhomX() * baseline / mScale,
                        rescaledPoint.getInhomY() * baseline / mScale,
                        rescaledPoint.getInhomZ() * baseline / mScale);
                if (point.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR)) {
                    numValidPoints++;
                }
            }

            if (numValidPoints == 0) {
                continue;
            }

            numValid++;

            if (numValid > 0) {
                break;
            }
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testGeneralPointsEssentialWithConstantAccelerationAndRotationWithoutNoiseThreeViews()
            throws InvalidPairOfCamerasException, AlgebraException,
            CameraException,
            com.irurueta.geometry.estimators.NotReadyException,
            com.irurueta.geometry.NotAvailableException, RotationException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);

            AbsoluteOrientationConstantVelocityModelSlamSparseReconstructorConfiguration configuration =
                    new AbsoluteOrientationConstantVelocityModelSlamSparseReconstructorConfiguration();
            configuration.setInitialCamerasEstimatorMethod(
                    InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);

            float accelerationOffsetX = 0.0f;
            float accelerationOffsetY = 0.0f;
            float accelerationOffsetZ = 0.0f;

            float angularOffsetX = 0.0f;
            float angularOffsetY = 0.0f;
            float angularOffsetZ = 0.0f;

            AbsoluteOrientationConstantVelocityModelSlamCalibrator calibrator = createFinishedCalibrator(
                    accelerationOffsetX, accelerationOffsetY,
                    accelerationOffsetZ, angularOffsetX, angularOffsetY,
                    angularOffsetZ, noiseRandomizer);
            AbsoluteOrientationConstantVelocityModelSlamCalibrationData calibrationData
                    = calibrator.getCalibrationData();
            configuration.setCalibrationData(calibrationData);

            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH_ESSENTIAL,
                    MAX_FOCAL_LENGTH_ESSENTIAL);
            double aspectRatio = configuration.getInitialCamerasAspectRatio();
            double skewness = 0.0;
            double principalPoint = 0.0;

            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength, focalLength,
                            principalPoint, principalPoint, skewness);
            intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);

            configuration.setInitialIntrinsic1(intrinsic);
            configuration.setInitialIntrinsic2(intrinsic);
            configuration.setAdditionalCamerasIntrinsics(intrinsic);
            configuration.setUseEPnPForAdditionalCamerasEstimation(true);
            configuration.setUseUPnPForAdditionalCamerasEstimation(false);
            configuration.setUseDAQForAdditionalCamerasIntrinics(false);
            configuration.setUseDIACForAdditionalCamerasIntrinsics(false);

            double alphaEuler1 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler1 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler1 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double alphaEuler3 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler3 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler3 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1,
                    betaEuler1, gammaEuler1);
            final MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2,
                    betaEuler2, gammaEuler2);
            AxisRotation3D axisRotation2 = new AxisRotation3D(
                    rotation1.inverseRotationAndReturnNew().
                            combineAndReturnNew(rotation2));

            MatrixRotation3D rotation3 = new MatrixRotation3D(alphaEuler3,
                    betaEuler3, gammaEuler3);

            double axis2X = axisRotation2.getAxisX();
            double axis2Y = axisRotation2.getAxisY();
            double axis2Z = axisRotation2.getAxisZ();
            double angle2 = axisRotation2.getRotationAngle();


            AxisRotation3D diffRotation = new AxisRotation3D(axis2X, axis2Y,
                    axis2Z, angle2 / N_SENSOR_SAMPLES);
            Quaternion diffQuaternion = new Quaternion(diffRotation);

            //angular speeds (roll, pitch, yaw) on x, y, z axes
            double[] angularSpeeds = diffQuaternion.toEulerAngles();
            final double angularSpeed2X = angularSpeeds[0];
            final double angularSpeed2Y = angularSpeeds[1];
            final double angularSpeed2Z = angularSpeeds[2];
            final Quaternion diffRotation2 = new Quaternion(angularSpeed2X,
                    angularSpeed2Y, angularSpeed2Z);

            //number of samples (50 samples * 0.02 s/sample = 1 second)
            MatrixRotation3D rotation2b = new MatrixRotation3D(rotation1);
            MatrixRotation3D rotation2c = new MatrixRotation3D(rotation1);
            for (int s = 0; s < N_SENSOR_SAMPLES; s++) {
                rotation2b.combine(diffRotation);
                rotation2c.combine(diffRotation2);
            }

            //check that rotations created by composing sensor samples are
            //equal to the original one
            assertTrue(rotation2.equals(rotation2b, ABSOLUTE_ERROR));
            assertTrue(rotation2.equals(rotation2c, ABSOLUTE_ERROR));

            AxisRotation3D accumDiffRotation = rotation2.inverseRotationAndReturnNew().
                    combineAndReturnNew(rotation3).toAxisRotation();
            double axis3X = accumDiffRotation.getAxisX();
            double axis3Y = accumDiffRotation.getAxisY();
            double axis3Z = accumDiffRotation.getAxisZ();
            double angle3 = accumDiffRotation.getRotationAngle();

            diffRotation = new AxisRotation3D(axis3X, axis3Y, axis3Z, angle3 / N_SENSOR_SAMPLES);
            diffQuaternion = new Quaternion(diffRotation);

            //angular speeds (roll, pitch, yaw) on x, y, z axes
            angularSpeeds = diffQuaternion.toEulerAngles();
            final double angularSpeed3X = angularSpeeds[0];
            final double angularSpeed3Y = angularSpeeds[1];
            final double angularSpeed3Z = angularSpeeds[2];
            final Quaternion diffRotation3 =
                    new Quaternion(angularSpeed3X, angularSpeed3Y, angularSpeed3Z);

            //number of samples (50 samples * 0.02 s/sample = 1 second), starting from
            //previously sampled rotation
            MatrixRotation3D rotation3b = new MatrixRotation3D(rotation2b);
            MatrixRotation3D rotation3c = new MatrixRotation3D(rotation2c);
            for (int s = 0; s < N_SENSOR_SAMPLES; s++) {
                rotation3b.combine(diffRotation);
                rotation3c.combine(diffRotation3);
            }

            //check that rotations created by composing sensor samples are equal
            //to the original one
            assertTrue(rotation3.equals(rotation3b, ABSOLUTE_ERROR));
            assertTrue(rotation3.equals(rotation3c, ABSOLUTE_ERROR));

            double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);
            double cameraSeparation2 = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);

            EuclideanTransformation3D rotationTransformation =
                    new EuclideanTransformation3D(rotation1);
            Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);
            Point3D center3 = new InhomogeneousPoint3D(
                    center2.getInhomX() + cameraSeparation2,
                    center2.getInhomY() + cameraSeparation2,
                    center2.getInhomZ() + cameraSeparation2);
            center1 = rotationTransformation.transformAndReturnNew(center1);
            center2 = rotationTransformation.transformAndReturnNew(center2);
            center3 = rotationTransformation.transformAndReturnNew(center3);

            double baseline = center1.distanceTo(center2);

            final double accelerationX, accelerationY, accelerationZ;

            //s = 0.5*a*t^2 --> a = 2*s/t^2
            //assuming t = 1 second (50 samples * 0.02 s/sample = 1 second)
            accelerationX = accelerationY = accelerationZ
                    = 2 * cameraSeparation;

            PinholeCamera camera1 = new PinholeCamera(intrinsic, rotation1,
                    center1);
            PinholeCamera camera2 = new PinholeCamera(intrinsic, rotation2,
                    center2);
            PinholeCamera camera3 = new PinholeCamera(intrinsic, rotation3,
                    center3);

            FundamentalMatrix fundamentalMatrix1 = new FundamentalMatrix(
                    camera1, camera2);

            //create 3D points laying in front of both cameras

            //1st find an approximate central point by intersecting the axis
            //planes of both cameras
            Plane horizontalPlane1 = camera1.getHorizontalAxisPlane();
            Plane verticalPlane1 = camera1.getVerticalAxisPlane();
            Plane horizontalPlane2 = camera2.getHorizontalAxisPlane();
            Plane verticalPlane2 = camera2.getVerticalAxisPlane();
            Matrix planesIntersectionMatrix = new Matrix(
                    Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
            planesIntersectionMatrix.setElementAt(0, 0, verticalPlane1.getA());
            planesIntersectionMatrix.setElementAt(0, 1, verticalPlane1.getB());
            planesIntersectionMatrix.setElementAt(0, 2, verticalPlane1.getC());
            planesIntersectionMatrix.setElementAt(0, 3, verticalPlane1.getD());

            planesIntersectionMatrix.setElementAt(1, 0,
                    horizontalPlane1.getA());
            planesIntersectionMatrix.setElementAt(1, 1,
                    horizontalPlane1.getB());
            planesIntersectionMatrix.setElementAt(1, 2,
                    horizontalPlane1.getC());
            planesIntersectionMatrix.setElementAt(1, 3,
                    horizontalPlane1.getD());

            planesIntersectionMatrix.setElementAt(2, 0, verticalPlane2.getA());
            planesIntersectionMatrix.setElementAt(2, 1, verticalPlane2.getB());
            planesIntersectionMatrix.setElementAt(2, 2, verticalPlane2.getC());
            planesIntersectionMatrix.setElementAt(2, 3, verticalPlane2.getD());

            planesIntersectionMatrix.setElementAt(3, 0,
                    horizontalPlane2.getA());
            planesIntersectionMatrix.setElementAt(3, 1,
                    horizontalPlane2.getB());
            planesIntersectionMatrix.setElementAt(3, 2,
                    horizontalPlane2.getC());
            planesIntersectionMatrix.setElementAt(3, 3,
                    horizontalPlane2.getD());

            SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    planesIntersectionMatrix);
            decomposer.decompose();
            Matrix v = decomposer.getV();
            HomogeneousPoint3D centralCommonPoint = new HomogeneousPoint3D(
                    v.getElementAt(0, 3),
                    v.getElementAt(1, 3),
                    v.getElementAt(2, 3),
                    v.getElementAt(3, 3));

            double lambdaX, lambdaY, lambdaZ;

            final int numPoints = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);

            InhomogeneousPoint3D point3D;
            List<InhomogeneousPoint3D> points3D =
                    new ArrayList<>();
            Point2D projectedPoint1, projectedPoint2, projectedPoint3;
            final List<Point2D> projectedPoints1 = new ArrayList<>();
            final List<Point2D> projectedPoints2 = new ArrayList<>();
            final List<Point2D> projectedPoints3 = new ArrayList<>();
            boolean front1, front2, maxTriesReached = false;
            for (int i = 0; i < numPoints; i++) {
                //generate points and ensure they lie in front of both cameras
                int numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(
                            MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaY = randomizer.nextDouble(
                            MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaZ = randomizer.nextDouble(
                            MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);

                    point3D = new InhomogeneousPoint3D(
                            centralCommonPoint.getInhomX() + lambdaX,
                            centralCommonPoint.getInhomY() + lambdaY,
                            centralCommonPoint.getInhomZ() + lambdaZ);

                    front1 = camera1.isPointInFrontOfCamera(point3D);
                    front2 = camera2.isPointInFrontOfCamera(point3D);
                    if (numTry > MAX_TRIES) {
                        maxTriesReached = true;
                        break;
                    }
                    numTry++;
                } while(!front1 || !front2);

                if (maxTriesReached) break;

                points3D.add(point3D);

                //check that 3D point is in front of both cameras
                //noinspection all
                assertTrue(front1);
                //noinspection all
                assertTrue(front2);

                //project 3D point into both cameras
                projectedPoint1 = new InhomogeneousPoint2D();
                camera1.project(point3D, projectedPoint1);
                projectedPoints1.add(projectedPoint1);

                projectedPoint2 = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2);
                projectedPoints2.add(projectedPoint2);

                projectedPoint3 = new InhomogeneousPoint2D();
                camera3.project(point3D, projectedPoint3);
                projectedPoints3.add(projectedPoint3);
            }

            if (maxTriesReached) continue;

            AbsoluteOrientationConstantVelocityModelSlamSparseReconstructorListener listener =
                    new AbsoluteOrientationConstantVelocityModelSlamSparseReconstructorListener() {
                        @Override
                        public boolean hasMoreViewsAvailable(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor) {
                            return mViewCount < 3;
                        }

                        @Override
                        public void onRequestSamplesForCurrentView(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor,
                                int viewId, List<Sample2D> samples) {

                            samples.clear();

                            Sample2D sample;
                            if (mViewCount == 0) {
                                //first view
                                for (int i = 0; i < numPoints; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints1.get(i));
                                    sample.setViewId(viewId);
                                    samples.add(sample);
                                }

                            } else if (mEstimatedFundamentalMatrix == null) {
                                //second view
                                for (int i = 0; i < numPoints; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints2.get(i));
                                    sample.setViewId(viewId);
                                    samples.add(sample);
                                }

                                //assume the following accelerator and gyroscope samples
                                //are obtained during a period of 1 second between 1st
                                //and 2nd view (50 samples * 0.02 s/sample = 1 second)
                                mTimestamp = 0;
                                Quaternion orientation = new Quaternion(rotation1);
                                for (int s = 0; s < N_SENSOR_SAMPLES; s++) {
                                    reconstructor.updateAccelerometerSample(mTimestamp,
                                            (float) accelerationX, (float) accelerationY,
                                            (float) accelerationZ);
                                    reconstructor.updateGyroscopeSample(mTimestamp,
                                            (float) angularSpeed2X, (float) angularSpeed2Y,
                                            (float) angularSpeed2Z);
                                    reconstructor.updateOrientationSample(mTimestamp,
                                            orientation);
                                    //update orientation
                                    orientation.combine(diffRotation2);
                                    mTimestamp += DELTA_NANOS;
                                }

                            } else {
                                //third view
                                for (int i = 0; i < numPoints; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints3.get(i));
                                    sample.setViewId(viewId);
                                    samples.add(sample);
                                }

                                //assume the following accelerator and gyroscope samples
                                //are obtained during a period of 1 second between 2nd
                                //and 3rd view (50 samples * 0.02 s/sample = 1 second)
                                Quaternion orientation = new Quaternion(rotation2);
                                for (int s = 0; s < N_SENSOR_SAMPLES; s++) {
                                    reconstructor.updateAccelerometerSample(mTimestamp,
                                            (float) accelerationX, (float) accelerationY,
                                            (float) accelerationZ);
                                    reconstructor.updateGyroscopeSample(mTimestamp,
                                            (float) angularSpeed3X, (float) angularSpeed3Y,
                                            (float) angularSpeed3Z);
                                    reconstructor.updateOrientationSample(mTimestamp,
                                            orientation);
                                    //update orientation
                                    orientation.combine(diffRotation3);
                                    mTimestamp += DELTA_NANOS;
                                }
                            }
                        }

                        @Override
                        public void onSamplesAccepted(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor, int viewId,
                                List<Sample2D> samples) {
                            mViewCount++;
                        }

                        @Override
                        public void onSamplesRejected(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor, int viewId,
                                List<Sample2D> samples) { }

                        @Override
                        public void onRequestMatches(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor,
                                List<Sample2D> samples1,
                                List<Sample2D> samples2, int viewId1, int viewId2,
                                List<MatchedSamples> matches) {
                            matches.clear();

                            int numCameras = 0;
                            if (mEstimatedMetricCamera1 != null &&
                                    (mEstimatedMetricCamera1.getViewId() == viewId1 ||
                                            mEstimatedMetricCamera1.getViewId() == viewId2)) {
                                numCameras++;
                            }
                            if (mEstimatedMetricCamera2 != null &&
                                    (mEstimatedMetricCamera2.getViewId() == viewId1 ||
                                            mEstimatedMetricCamera2.getViewId() == viewId2)) {
                                numCameras++;
                            }

                            EstimatedCamera[] estimatedCameras = null;
                            if (numCameras > 0) {
                                estimatedCameras = new EstimatedCamera[numCameras];


                                int pos = 0;
                                if (mEstimatedMetricCamera1 != null &&
                                        (mEstimatedMetricCamera1.getViewId() == viewId1 ||
                                                mEstimatedMetricCamera1.getViewId() == viewId2)) {
                                    estimatedCameras[pos] = mEstimatedMetricCamera1;
                                    pos++;
                                }
                                if (mEstimatedMetricCamera2 != null &&
                                        (mEstimatedMetricCamera2.getViewId() == viewId1 ||
                                                mEstimatedMetricCamera2.getViewId() == viewId2)) {
                                    estimatedCameras[pos] = mEstimatedMetricCamera2;
                                }
                            }

                            MatchedSamples match;
                            for (int i = 0; i < numPoints; i++) {
                                match = new MatchedSamples();
                                match.setSamples(new Sample2D[]{
                                        samples1.get(i), samples2.get(i)
                                });
                                match.setViewIds(new int[]{viewId1, viewId2});

                                if (mMetricReconstructedPoints != null) {
                                    match.setReconstructedPoint(
                                            mMetricReconstructedPoints.get(i));
                                }

                                if (estimatedCameras != null) {
                                    match.setCameras(estimatedCameras);
                                }

                                matches.add(match);
                            }
                        }

                        @Override
                        public void onFundamentalMatrixEstimated(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor,
                                EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                            if (mEstimatedFundamentalMatrix == null) {
                                mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                            } else if (mEstimatedFundamentalMatrix2 == null) {
                                mEstimatedFundamentalMatrix2 = estimatedFundamentalMatrix;
                            }
                        }

                        @Override
                        public void onMetricCameraEstimated(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor,
                                int previousViewId, int currentViewId,
                                EstimatedCamera previousCamera,
                                EstimatedCamera currentCamera) {
                            if (mEstimatedMetricCamera2 == null) {
                                mEstimatedMetricCamera1 = previousCamera;
                                mEstimatedMetricCamera2 = currentCamera;
                            } else if (mEstimatedMetricCamera3 == null){
                                mEstimatedMetricCamera2 = previousCamera;
                                mEstimatedMetricCamera3 = currentCamera;
                            }
                        }

                        @Override
                        public void onMetricReconstructedPointsEstimated(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor,
                                List<MatchedSamples> matches,
                                List<ReconstructedPoint3D> points) {
                            mMetricReconstructedPoints = points;
                        }

                        @Override
                        public void onEuclideanCameraEstimated(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor,
                                int previousViewId, int currentViewId, double scale,
                                EstimatedCamera previousCamera,
                                EstimatedCamera currentCamera) {
                            if (mEstimatedEuclideanCamera2 == null) {
                                mEstimatedEuclideanCamera1 = previousCamera;
                                mEstimatedEuclideanCamera2 = currentCamera;
                                mScale = scale;
                            } else if (mEstimatedEuclideanCamera3 == null) {
                                mEstimatedEuclideanCamera2 = previousCamera;
                                mEstimatedEuclideanCamera3 = currentCamera;
                                mScale2 = scale;
                            }
                        }

                        @Override
                        public void onEuclideanReconstructedPointsEstimated(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor,
                                double scale, List<ReconstructedPoint3D> points) {
                            if (mEuclideanReconstructedPoints == null) {
                                mScale = scale;
                            } else {
                                mScale2 = scale;
                            }

                            mEuclideanReconstructedPoints = points;
                        }

                        @Override
                        public void onStart(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor) {
                            mStarted = true;
                        }

                        @Override
                        public void onFinish(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor) {
                            mFinished = true;
                        }

                        @Override
                        public void onCancel(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor) {
                            mCancelled = true;
                        }

                        @Override
                        public void onFail(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor) {
                            mFailed = true;
                        }
                    };

            AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor =
                    new AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor(configuration, listener);

            //check initial values
            reset();
            assertFalse(mStarted);
            assertFalse(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertFalse(reconstructor.isFinished());

            reconstructor.start();

            //check correctness
            assertTrue(mStarted);
            assertTrue(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertTrue(reconstructor.isFinished());

            //check that estimated fundamental matrix is correct
            fundamentalMatrix1.normalize();
            mEstimatedFundamentalMatrix.getFundamentalMatrix().normalize();

            assertNull(mEstimatedFundamentalMatrix2);

            //matrices are equal up to scale
            if (!fundamentalMatrix1.getInternalMatrix().equals(
                    mEstimatedFundamentalMatrix.getFundamentalMatrix().
                            getInternalMatrix(), ABSOLUTE_ERROR) &&
                    !fundamentalMatrix1.getInternalMatrix().
                            multiplyByScalarAndReturnNew(-1).equals(
                            mEstimatedFundamentalMatrix.getFundamentalMatrix().
                                    getInternalMatrix(), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundamentalMatrix1.getInternalMatrix().equals(
                    mEstimatedFundamentalMatrix.getFundamentalMatrix().
                            getInternalMatrix(), ABSOLUTE_ERROR) ||
                    fundamentalMatrix1.getInternalMatrix().
                            multiplyByScalarAndReturnNew(-1).equals(
                            mEstimatedFundamentalMatrix.getFundamentalMatrix().
                                    getInternalMatrix(), ABSOLUTE_ERROR));

            //check that reconstructed points are in a metric stratum (up to a
            //certain scale)
            PinholeCamera estimatedMetricCamera1 = mEstimatedMetricCamera1.getCamera();
            PinholeCamera estimatedMetricCamera2 = mEstimatedMetricCamera2.getCamera();
            PinholeCamera estimatedMetricCamera3 = mEstimatedMetricCamera3.getCamera();
            assertNotSame(mEstimatedMetricCamera1, mEstimatedEuclideanCamera1);
            assertNotSame(mEstimatedMetricCamera2, mEstimatedEuclideanCamera2);
            assertNotSame(mEstimatedMetricCamera3, mEstimatedEuclideanCamera3);

            PinholeCamera estimatedEuclideanCamera1 = mEstimatedEuclideanCamera1.getCamera();
            PinholeCamera estimatedEuclideanCamera2 = mEstimatedEuclideanCamera2.getCamera();
            PinholeCamera estimatedEuclideanCamera3 = mEstimatedEuclideanCamera3.getCamera();

            estimatedMetricCamera1.decompose();
            estimatedMetricCamera2.decompose();
            estimatedMetricCamera3.decompose();

            estimatedEuclideanCamera1.decompose();
            estimatedEuclideanCamera2.decompose();
            estimatedEuclideanCamera3.decompose();

            assertNotSame(mMetricReconstructedPoints, mEuclideanReconstructedPoints);

            if (mMetricReconstructedPoints.size() != numPoints) {
                continue;
            }

            List<Point3D> metricReconstructedPoints3D = new ArrayList<>();
            List<Point3D> euclideanReconstructedPoints3D = new ArrayList<>();
            for (int i = 0; i < numPoints; i++) {
                metricReconstructedPoints3D.add(
                        mMetricReconstructedPoints.get(i).getPoint());
                euclideanReconstructedPoints3D.add(
                        mEuclideanReconstructedPoints.get(i).getPoint());
            }

            //check that all points are in front of both cameras
            for (int i = 0; i < numPoints; i++) {
                Point3D p = metricReconstructedPoints3D.get(i);
                Point3D pe = euclideanReconstructedPoints3D.get(i);

                assertTrue(estimatedMetricCamera1.isPointInFrontOfCamera(p));
                assertTrue(estimatedMetricCamera2.isPointInFrontOfCamera(p));

                assertTrue(estimatedEuclideanCamera1.isPointInFrontOfCamera(pe));
                assertTrue(estimatedEuclideanCamera2.isPointInFrontOfCamera(pe));
            }

            PinholeCameraIntrinsicParameters euclideanIntrinsic1 =
                    estimatedEuclideanCamera1.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters euclideanIntrinsic2 =
                    estimatedEuclideanCamera2.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters euclideanIntrinsic3 =
                    estimatedEuclideanCamera3.getIntrinsicParameters();

            Rotation3D euclideanRotation1 = estimatedEuclideanCamera1.getCameraRotation();
            Rotation3D euclideanRotation2 = estimatedEuclideanCamera2.getCameraRotation();
            Rotation3D euclideanRotation3 = estimatedEuclideanCamera3.getCameraRotation();

            assertEquals(euclideanIntrinsic1.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertEquals(euclideanIntrinsic2.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertEquals(euclideanIntrinsic3.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic3.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic3.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic3.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic3.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertTrue(euclideanRotation1.asInhomogeneousMatrix().equals(
                    rotation1.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
            assertTrue(euclideanRotation2.asInhomogeneousMatrix().equals(
                    rotation2.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
            assertTrue(euclideanRotation3.asInhomogeneousMatrix().equals(
                    rotation3.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

            //check that points are correct (after scale correction)

            MetricTransformation3D scaleAndOrientationTransformation
                    = new MetricTransformation3D(mScale2);
            scaleAndOrientationTransformation.setRotation(rotation1.inverseRotationAndReturnNew());


            int numValidPoints = 0;
            for (int i = 0; i < numPoints; i++) {
                Point3D point = points3D.get(i);
                Point3D euclideanPoint = euclideanReconstructedPoints3D.get(i);

                //check metric points
                Point3D rescaledPoint = Point3D.create();
                scaleAndOrientationTransformation.transform(metricReconstructedPoints3D.get(i),
                        rescaledPoint);

                assertTrue(euclideanPoint.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR));

                rescaledPoint.setInhomogeneousCoordinates(
                        rescaledPoint.getInhomX() * baseline / mScale2,
                        rescaledPoint.getInhomY() * baseline / mScale2,
                        rescaledPoint.getInhomZ() * baseline / mScale2);
                if (point.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR)) {
                    numValidPoints++;
                }
            }

            if (numValidPoints == 0) {
                continue;
            }

            numValid++;

            if (numValid > 0) {
                break;
            }
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testGeneralPointsEssentialWithConstantAccelerationAndRotationWithNoiseThreeViews()
            throws InvalidPairOfCamerasException, AlgebraException,
            CameraException,
            com.irurueta.geometry.estimators.NotReadyException,
            com.irurueta.geometry.NotAvailableException, RotationException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer offsetRandomizer = new UniformRandomizer(
                    new Random());
            GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);

            AbsoluteOrientationConstantVelocityModelSlamSparseReconstructorConfiguration configuration =
                    new AbsoluteOrientationConstantVelocityModelSlamSparseReconstructorConfiguration();
            configuration.setInitialCamerasEstimatorMethod(
                    InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);

            float accelerationOffsetX = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            float accelerationOffsetY = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            float accelerationOffsetZ = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);

            float angularOffsetX = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            float angularOffsetY = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            float angularOffsetZ = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);

            AbsoluteOrientationConstantVelocityModelSlamCalibrator calibrator = createFinishedCalibrator(
                    accelerationOffsetX, accelerationOffsetY,
                    accelerationOffsetZ, angularOffsetX, angularOffsetY,
                    angularOffsetZ, noiseRandomizer);
            AbsoluteOrientationConstantVelocityModelSlamCalibrationData calibrationData
                    = calibrator.getCalibrationData();
            configuration.setCalibrationData(calibrationData);

            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH_ESSENTIAL,
                    MAX_FOCAL_LENGTH_ESSENTIAL);
            double aspectRatio = configuration.getInitialCamerasAspectRatio();
            double skewness = 0.0;
            double principalPoint = 0.0;

            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength, focalLength,
                            principalPoint, principalPoint, skewness);
            intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);

            configuration.setInitialIntrinsic1(intrinsic);
            configuration.setInitialIntrinsic2(intrinsic);
            configuration.setAdditionalCamerasIntrinsics(intrinsic);
            configuration.setUseEPnPForAdditionalCamerasEstimation(true);
            configuration.setUseUPnPForAdditionalCamerasEstimation(false);
            configuration.setUseDAQForAdditionalCamerasIntrinics(false);
            configuration.setUseDIACForAdditionalCamerasIntrinsics(false);

            double alphaEuler1 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler1 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler1 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double alphaEuler3 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler3 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler3 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1,
                    betaEuler1, gammaEuler1);
            final MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2,
                    betaEuler2, gammaEuler2);
            AxisRotation3D axisRotation2 = new AxisRotation3D(
                    rotation1.inverseRotationAndReturnNew().
                            combineAndReturnNew(rotation2));

            MatrixRotation3D rotation3 = new MatrixRotation3D(alphaEuler3,
                    betaEuler3, gammaEuler3);

            double axis2X = axisRotation2.getAxisX();
            double axis2Y = axisRotation2.getAxisY();
            double axis2Z = axisRotation2.getAxisZ();
            double angle2 = axisRotation2.getRotationAngle();


            AxisRotation3D diffRotation = new AxisRotation3D(axis2X, axis2Y,
                    axis2Z, angle2 / N_SENSOR_SAMPLES);
            Quaternion diffQuaternion = new Quaternion(diffRotation);

            //angular speeds (roll, pitch, yaw) on x, y, z axes
            double[] angularSpeeds = diffQuaternion.toEulerAngles();
            final double angularSpeed2X = angularSpeeds[0];
            final double angularSpeed2Y = angularSpeeds[1];
            final double angularSpeed2Z = angularSpeeds[2];
            final Quaternion diffRotation2 = new Quaternion(angularSpeed2X,
                    angularSpeed2Y, angularSpeed2Z);

            //number of samples (50 samples * 0.02 s/sample = 1 second)
            MatrixRotation3D rotation2b = new MatrixRotation3D(rotation1);
            MatrixRotation3D rotation2c = new MatrixRotation3D(rotation1);
            for (int s = 0; s < N_SENSOR_SAMPLES; s++) {
                rotation2b.combine(diffRotation);
                rotation2c.combine(diffRotation2);
            }

            //check that rotations created by composing sensor samples are
            //equal to the original one
            assertTrue(rotation2.equals(rotation2b, ABSOLUTE_ERROR));
            assertTrue(rotation2.equals(rotation2c, ABSOLUTE_ERROR));

            AxisRotation3D accumDiffRotation = rotation2.inverseRotationAndReturnNew().
                    combineAndReturnNew(rotation3).toAxisRotation();
            double axis3X = accumDiffRotation.getAxisX();
            double axis3Y = accumDiffRotation.getAxisY();
            double axis3Z = accumDiffRotation.getAxisZ();
            double angle3 = accumDiffRotation.getRotationAngle();

            diffRotation = new AxisRotation3D(axis3X, axis3Y, axis3Z, angle3 / N_SENSOR_SAMPLES);
            diffQuaternion = new Quaternion(diffRotation);

            //angular speeds (roll, pitch, yaw) on x, y, z axes
            angularSpeeds = diffQuaternion.toEulerAngles();
            final double angularSpeed3X = angularSpeeds[0];
            final double angularSpeed3Y = angularSpeeds[1];
            final double angularSpeed3Z = angularSpeeds[2];
            final Quaternion diffRotation3 =
                    new Quaternion(angularSpeed3X, angularSpeed3Y, angularSpeed3Z);

            //number of samples (50 samples * 0.02 s/sample = 1 second), starting from
            //previously sampled rotation
            MatrixRotation3D rotation3b = new MatrixRotation3D(rotation2b);
            MatrixRotation3D rotation3c = new MatrixRotation3D(rotation2c);
            for (int s = 0; s < N_SENSOR_SAMPLES; s++) {
                rotation3b.combine(diffRotation);
                rotation3c.combine(diffRotation3);
            }

            //check that rotations created by composing sensor samples are equal
            //to the original one
            assertTrue(rotation3.equals(rotation3b, ABSOLUTE_ERROR));
            assertTrue(rotation3.equals(rotation3c, ABSOLUTE_ERROR));

            double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);
            double cameraSeparation2 = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);

            EuclideanTransformation3D rotationTransformation =
                    new EuclideanTransformation3D(rotation1);
            Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);
            Point3D center3 = new InhomogeneousPoint3D(
                    center2.getInhomX() + cameraSeparation2,
                    center2.getInhomY() + cameraSeparation2,
                    center2.getInhomZ() + cameraSeparation2);
            center1 = rotationTransformation.transformAndReturnNew(center1);
            center2 = rotationTransformation.transformAndReturnNew(center2);
            center3 = rotationTransformation.transformAndReturnNew(center3);

            double baseline = center1.distanceTo(center2);

            final double accelerationX, accelerationY, accelerationZ;

            //s = 0.5*a*t^2 --> a = 2*s/t^2
            //assuming t = 1 second (50 samples * 0.02 s/sample = 1 second)
            accelerationX = accelerationY = accelerationZ
                    = 2 * cameraSeparation;

            PinholeCamera camera1 = new PinholeCamera(intrinsic, rotation1,
                    center1);
            PinholeCamera camera2 = new PinholeCamera(intrinsic, rotation2,
                    center2);
            PinholeCamera camera3 = new PinholeCamera(intrinsic, rotation3,
                    center3);

            FundamentalMatrix fundamentalMatrix1 = new FundamentalMatrix(
                    camera1, camera2);

            //create 3D points laying in front of both cameras

            //1st find an approximate central point by intersecting the axis
            //planes of both cameras
            Plane horizontalPlane1 = camera1.getHorizontalAxisPlane();
            Plane verticalPlane1 = camera1.getVerticalAxisPlane();
            Plane horizontalPlane2 = camera2.getHorizontalAxisPlane();
            Plane verticalPlane2 = camera2.getVerticalAxisPlane();
            Matrix planesIntersectionMatrix = new Matrix(
                    Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
            planesIntersectionMatrix.setElementAt(0, 0, verticalPlane1.getA());
            planesIntersectionMatrix.setElementAt(0, 1, verticalPlane1.getB());
            planesIntersectionMatrix.setElementAt(0, 2, verticalPlane1.getC());
            planesIntersectionMatrix.setElementAt(0, 3, verticalPlane1.getD());

            planesIntersectionMatrix.setElementAt(1, 0,
                    horizontalPlane1.getA());
            planesIntersectionMatrix.setElementAt(1, 1,
                    horizontalPlane1.getB());
            planesIntersectionMatrix.setElementAt(1, 2,
                    horizontalPlane1.getC());
            planesIntersectionMatrix.setElementAt(1, 3,
                    horizontalPlane1.getD());

            planesIntersectionMatrix.setElementAt(2, 0, verticalPlane2.getA());
            planesIntersectionMatrix.setElementAt(2, 1, verticalPlane2.getB());
            planesIntersectionMatrix.setElementAt(2, 2, verticalPlane2.getC());
            planesIntersectionMatrix.setElementAt(2, 3, verticalPlane2.getD());

            planesIntersectionMatrix.setElementAt(3, 0,
                    horizontalPlane2.getA());
            planesIntersectionMatrix.setElementAt(3, 1,
                    horizontalPlane2.getB());
            planesIntersectionMatrix.setElementAt(3, 2,
                    horizontalPlane2.getC());
            planesIntersectionMatrix.setElementAt(3, 3,
                    horizontalPlane2.getD());

            SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    planesIntersectionMatrix);
            decomposer.decompose();
            Matrix v = decomposer.getV();
            HomogeneousPoint3D centralCommonPoint = new HomogeneousPoint3D(
                    v.getElementAt(0, 3),
                    v.getElementAt(1, 3),
                    v.getElementAt(2, 3),
                    v.getElementAt(3, 3));

            double lambdaX, lambdaY, lambdaZ;

            final int numPoints = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);

            InhomogeneousPoint3D point3D;
            List<InhomogeneousPoint3D> points3D =
                    new ArrayList<>();
            Point2D projectedPoint1, projectedPoint2, projectedPoint3;
            final List<Point2D> projectedPoints1 = new ArrayList<>();
            final List<Point2D> projectedPoints2 = new ArrayList<>();
            final List<Point2D> projectedPoints3 = new ArrayList<>();
            boolean front1, front2, maxTriesReached = false;
            for (int i = 0; i < numPoints; i++) {
                //generate points and ensure they lie in front of both cameras
                int numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(
                            MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaY = randomizer.nextDouble(
                            MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaZ = randomizer.nextDouble(
                            MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);

                    point3D = new InhomogeneousPoint3D(
                            centralCommonPoint.getInhomX() + lambdaX,
                            centralCommonPoint.getInhomY() + lambdaY,
                            centralCommonPoint.getInhomZ() + lambdaZ);

                    front1 = camera1.isPointInFrontOfCamera(point3D);
                    front2 = camera2.isPointInFrontOfCamera(point3D);
                    if (numTry > MAX_TRIES) {
                        maxTriesReached = true;
                        break;
                    }
                    numTry++;
                } while(!front1 || !front2);

                if (maxTriesReached) break;

                points3D.add(point3D);

                //check that 3D point is in front of both cameras
                //noinspection all
                assertTrue(front1);
                //noinspection all
                assertTrue(front2);

                //project 3D point into both cameras
                projectedPoint1 = new InhomogeneousPoint2D();
                camera1.project(point3D, projectedPoint1);
                projectedPoints1.add(projectedPoint1);

                projectedPoint2 = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2);
                projectedPoints2.add(projectedPoint2);

                projectedPoint3 = new InhomogeneousPoint2D();
                camera3.project(point3D, projectedPoint3);
                projectedPoints3.add(projectedPoint3);
            }

            if (maxTriesReached) continue;

            final GaussianRandomizer accelerationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);

            AbsoluteOrientationConstantVelocityModelSlamSparseReconstructorListener listener =
                    new AbsoluteOrientationConstantVelocityModelSlamSparseReconstructorListener() {
                        @Override
                        public boolean hasMoreViewsAvailable(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor) {
                            return mViewCount < 3;
                        }

                        @Override
                        public void onRequestSamplesForCurrentView(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor,
                                int viewId, List<Sample2D> samples) {

                            samples.clear();

                            Sample2D sample;
                            if (mViewCount == 0) {
                                //first view
                                for (int i = 0; i < numPoints; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints1.get(i));
                                    sample.setViewId(viewId);
                                    samples.add(sample);
                                }

                            } else if (mEstimatedFundamentalMatrix == null) {
                                //second view
                                for (int i = 0; i < numPoints; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints2.get(i));
                                    sample.setViewId(viewId);
                                    samples.add(sample);
                                }

                                //assume the following accelerator and gyroscope samples
                                //are obtained during a period of 1 second between 1st
                                //and 2nd view (50 samples * 0.02 s/sample = 1 second)
                                mTimestamp = 0;
                                float noiseAccelerationX, noiseAccelerationY,
                                        noiseAccelerationZ;
                                float noiseAngularSpeedX, noiseAngularSpeedY,
                                        noiseAngularSpeedZ;

                                float accelerationWithNoiseX, accelerationWithNoiseY,
                                        accelerationWithNoiseZ;
                                float angularSpeedWithNoiseX, angularSpeedWithNoiseY,
                                        angularSpeedWithNoiseZ;

                                float[] accelerationWithNoise = new float[3];
                                float[] angularSpeedWithNoise = new float[3];

                                Quaternion orientation = new Quaternion(rotation1);
                                for (int s = 0; s < N_SENSOR_SAMPLES; s++) {
                                    noiseAccelerationX =
                                            accelerationRandomizer.nextFloat();
                                    noiseAccelerationY =
                                            accelerationRandomizer.nextFloat();
                                    noiseAccelerationZ =
                                            accelerationRandomizer.nextFloat();

                                    noiseAngularSpeedX =
                                            angularSpeedRandomizer.nextFloat();
                                    noiseAngularSpeedY =
                                            angularSpeedRandomizer.nextFloat();
                                    noiseAngularSpeedZ =
                                            angularSpeedRandomizer.nextFloat();

                                    accelerationWithNoiseX = (float)accelerationX +
                                            noiseAccelerationX;
                                    accelerationWithNoiseY = (float)accelerationY +
                                            noiseAccelerationY;
                                    accelerationWithNoiseZ = (float)accelerationZ +
                                            noiseAccelerationZ;
                                    accelerationWithNoise[0] = accelerationWithNoiseX;
                                    accelerationWithNoise[1] = accelerationWithNoiseY;
                                    accelerationWithNoise[2] = accelerationWithNoiseZ;

                                    angularSpeedWithNoiseX = (float)angularSpeed2X +
                                            noiseAngularSpeedX;
                                    angularSpeedWithNoiseY = (float)angularSpeed2Y +
                                            noiseAngularSpeedY;
                                    angularSpeedWithNoiseZ = (float)angularSpeed2Z +
                                            noiseAngularSpeedZ;
                                    angularSpeedWithNoise[0] = angularSpeedWithNoiseX;
                                    angularSpeedWithNoise[1] = angularSpeedWithNoiseY;
                                    angularSpeedWithNoise[2] = angularSpeedWithNoiseZ;

                                    reconstructor.updateAccelerometerSample(mTimestamp,
                                            accelerationWithNoise);
                                    reconstructor.updateGyroscopeSample(mTimestamp,
                                            angularSpeedWithNoise);
                                    reconstructor.updateOrientationSample(mTimestamp,
                                            orientation);
                                    //update orientation
                                    orientation.combine(diffRotation2);
                                    mTimestamp += DELTA_NANOS;
                                }

                            } else {
                                //third view
                                for (int i = 0; i < numPoints; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints3.get(i));
                                    sample.setViewId(viewId);
                                    samples.add(sample);
                                }

                                //assume the following accelerator and gyroscope samples
                                //are obtained during a period of 1 second between 2nd
                                //and 3rd view (50 samples * 0.02 s/sample = 1 second)
                                float noiseAccelerationX, noiseAccelerationY,
                                        noiseAccelerationZ;
                                float noiseAngularSpeedX, noiseAngularSpeedY,
                                        noiseAngularSpeedZ;

                                float accelerationWithNoiseX, accelerationWithNoiseY,
                                        accelerationWithNoiseZ;
                                float angularSpeedWithNoiseX, angularSpeedWithNoiseY,
                                        angularSpeedWithNoiseZ;

                                float[] accelerationWithNoise = new float[3];
                                float[] angularSpeedWithNoise = new float[3];

                                Quaternion orientation = new Quaternion(rotation2);
                                for (int s = 0; s < N_SENSOR_SAMPLES; s++) {
                                    noiseAccelerationX =
                                            accelerationRandomizer.nextFloat();
                                    noiseAccelerationY =
                                            accelerationRandomizer.nextFloat();
                                    noiseAccelerationZ =
                                            accelerationRandomizer.nextFloat();

                                    noiseAngularSpeedX =
                                            angularSpeedRandomizer.nextFloat();
                                    noiseAngularSpeedY =
                                            angularSpeedRandomizer.nextFloat();
                                    noiseAngularSpeedZ =
                                            angularSpeedRandomizer.nextFloat();

                                    accelerationWithNoiseX = (float)accelerationX +
                                            noiseAccelerationX;
                                    accelerationWithNoiseY = (float)accelerationY +
                                            noiseAccelerationY;
                                    accelerationWithNoiseZ = (float)accelerationZ +
                                            noiseAccelerationZ;
                                    accelerationWithNoise[0] = accelerationWithNoiseX;
                                    accelerationWithNoise[1] = accelerationWithNoiseY;
                                    accelerationWithNoise[2] = accelerationWithNoiseZ;

                                    angularSpeedWithNoiseX = (float)angularSpeed3X +
                                            noiseAngularSpeedX;
                                    angularSpeedWithNoiseY = (float)angularSpeed3Y +
                                            noiseAngularSpeedY;
                                    angularSpeedWithNoiseZ = (float)angularSpeed3Z +
                                            noiseAngularSpeedZ;
                                    angularSpeedWithNoise[0] = angularSpeedWithNoiseX;
                                    angularSpeedWithNoise[1] = angularSpeedWithNoiseY;
                                    angularSpeedWithNoise[2] = angularSpeedWithNoiseZ;

                                    reconstructor.updateAccelerometerSample(mTimestamp,
                                            accelerationWithNoise);
                                    reconstructor.updateGyroscopeSample(mTimestamp,
                                            angularSpeedWithNoise);
                                    reconstructor.updateOrientationSample(mTimestamp,
                                            orientation);
                                    //update orientation
                                    orientation.combine(diffRotation3);
                                    mTimestamp += DELTA_NANOS;
                                }
                            }
                        }

                        @Override
                        public void onSamplesAccepted(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor, int viewId,
                                List<Sample2D> samples) {
                            mViewCount++;
                        }

                        @Override
                        public void onSamplesRejected(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor, int viewId,
                                List<Sample2D> samples) { }

                        @Override
                        public void onRequestMatches(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor,
                                List<Sample2D> samples1,
                                List<Sample2D> samples2, int viewId1, int viewId2,
                                List<MatchedSamples> matches) {
                            matches.clear();

                            int numCameras = 0;
                            if (mEstimatedMetricCamera1 != null &&
                                    (mEstimatedMetricCamera1.getViewId() == viewId1 ||
                                            mEstimatedMetricCamera1.getViewId() == viewId2)) {
                                numCameras++;
                            }
                            if (mEstimatedMetricCamera2 != null &&
                                    (mEstimatedMetricCamera2.getViewId() == viewId1 ||
                                            mEstimatedMetricCamera2.getViewId() == viewId2)) {
                                numCameras++;
                            }

                            EstimatedCamera[] estimatedCameras = null;
                            if (numCameras > 0) {
                                estimatedCameras = new EstimatedCamera[numCameras];


                                int pos = 0;
                                if (mEstimatedMetricCamera1 != null &&
                                        (mEstimatedMetricCamera1.getViewId() == viewId1 ||
                                                mEstimatedMetricCamera1.getViewId() == viewId2)) {
                                    estimatedCameras[pos] = mEstimatedMetricCamera1;
                                    pos++;
                                }
                                if (mEstimatedMetricCamera2 != null &&
                                        (mEstimatedMetricCamera2.getViewId() == viewId1 ||
                                                mEstimatedMetricCamera2.getViewId() == viewId2)) {
                                    estimatedCameras[pos] = mEstimatedMetricCamera2;
                                }
                            }

                            MatchedSamples match;
                            for (int i = 0; i < numPoints; i++) {
                                match = new MatchedSamples();
                                match.setSamples(new Sample2D[]{
                                        samples1.get(i), samples2.get(i)
                                });
                                match.setViewIds(new int[]{viewId1, viewId2});

                                if (mMetricReconstructedPoints != null) {
                                    match.setReconstructedPoint(
                                            mMetricReconstructedPoints.get(i));
                                }

                                if (estimatedCameras != null) {
                                    match.setCameras(estimatedCameras);
                                }

                                matches.add(match);
                            }
                        }

                        @Override
                        public void onFundamentalMatrixEstimated(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor,
                                EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                            if (mEstimatedFundamentalMatrix == null) {
                                mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                            } else if (mEstimatedFundamentalMatrix2 == null) {
                                mEstimatedFundamentalMatrix2 = estimatedFundamentalMatrix;
                            }
                        }

                        @Override
                        public void onMetricCameraEstimated(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor,
                                int previousViewId, int currentViewId,
                                EstimatedCamera previousCamera,
                                EstimatedCamera currentCamera) {
                            if (mEstimatedMetricCamera2 == null) {
                                mEstimatedMetricCamera1 = previousCamera;
                                mEstimatedMetricCamera2 = currentCamera;
                            } else if (mEstimatedMetricCamera3 == null){
                                mEstimatedMetricCamera2 = previousCamera;
                                mEstimatedMetricCamera3 = currentCamera;
                            }
                        }

                        @Override
                        public void onMetricReconstructedPointsEstimated(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor,
                                List<MatchedSamples> matches,
                                List<ReconstructedPoint3D> points) {
                            mMetricReconstructedPoints = points;
                        }

                        @Override
                        public void onEuclideanCameraEstimated(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor,
                                int previousViewId, int currentViewId, double scale,
                                EstimatedCamera previousCamera,
                                EstimatedCamera currentCamera) {
                            if (mEstimatedEuclideanCamera2 == null) {
                                mEstimatedEuclideanCamera1 = previousCamera;
                                mEstimatedEuclideanCamera2 = currentCamera;
                                mScale = scale;
                            } else if (mEstimatedEuclideanCamera3 == null) {
                                mEstimatedEuclideanCamera2 = previousCamera;
                                mEstimatedEuclideanCamera3 = currentCamera;
                                mScale2 = scale;
                            }
                        }

                        @Override
                        public void onEuclideanReconstructedPointsEstimated(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor,
                                double scale, List<ReconstructedPoint3D> points) {
                            if (mEuclideanReconstructedPoints == null) {
                                mScale = scale;
                            } else {
                                mScale2 = scale;
                            }

                            mEuclideanReconstructedPoints = points;
                        }

                        @Override
                        public void onStart(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor) {
                            mStarted = true;
                        }

                        @Override
                        public void onFinish(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor) {
                            mFinished = true;
                        }

                        @Override
                        public void onCancel(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor) {
                            mCancelled = true;
                        }

                        @Override
                        public void onFail(
                                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor) {
                            mFailed = true;
                        }
                    };

            AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor reconstructor =
                    new AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor(configuration, listener);

            //check initial values
            reset();
            assertFalse(mStarted);
            assertFalse(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertFalse(reconstructor.isFinished());

            reconstructor.start();

            //check correctness
            assertTrue(mStarted);
            assertTrue(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertTrue(reconstructor.isFinished());

            //check that estimated fundamental matrix is correct
            fundamentalMatrix1.normalize();
            mEstimatedFundamentalMatrix.getFundamentalMatrix().normalize();

            assertNull(mEstimatedFundamentalMatrix2);

            //matrices are equal up to scale
            if (!fundamentalMatrix1.getInternalMatrix().equals(
                    mEstimatedFundamentalMatrix.getFundamentalMatrix().
                            getInternalMatrix(), ABSOLUTE_ERROR) &&
                    !fundamentalMatrix1.getInternalMatrix().
                            multiplyByScalarAndReturnNew(-1).equals(
                            mEstimatedFundamentalMatrix.getFundamentalMatrix().
                                    getInternalMatrix(), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundamentalMatrix1.getInternalMatrix().equals(
                    mEstimatedFundamentalMatrix.getFundamentalMatrix().
                            getInternalMatrix(), ABSOLUTE_ERROR) ||
                    fundamentalMatrix1.getInternalMatrix().
                            multiplyByScalarAndReturnNew(-1).equals(
                            mEstimatedFundamentalMatrix.getFundamentalMatrix().
                                    getInternalMatrix(), ABSOLUTE_ERROR));

            //check that reconstructed points are in a metric stratum (up to a
            //certain scale)
            PinholeCamera estimatedMetricCamera1 = mEstimatedMetricCamera1.getCamera();
            PinholeCamera estimatedMetricCamera2 = mEstimatedMetricCamera2.getCamera();
            PinholeCamera estimatedMetricCamera3 = mEstimatedMetricCamera3.getCamera();
            assertNotSame(mEstimatedMetricCamera1, mEstimatedEuclideanCamera1);
            assertNotSame(mEstimatedMetricCamera2, mEstimatedEuclideanCamera2);
            assertNotSame(mEstimatedMetricCamera3, mEstimatedEuclideanCamera3);

            PinholeCamera estimatedEuclideanCamera1 = mEstimatedEuclideanCamera1.getCamera();
            PinholeCamera estimatedEuclideanCamera2 = mEstimatedEuclideanCamera2.getCamera();
            PinholeCamera estimatedEuclideanCamera3 = mEstimatedEuclideanCamera3.getCamera();

            estimatedMetricCamera1.decompose();
            estimatedMetricCamera2.decompose();
            estimatedMetricCamera3.decompose();

            estimatedEuclideanCamera1.decompose();
            estimatedEuclideanCamera2.decompose();
            estimatedEuclideanCamera3.decompose();

            assertNotSame(mMetricReconstructedPoints, mEuclideanReconstructedPoints);

            if (mMetricReconstructedPoints.size() != numPoints) {
                continue;
            }

            List<Point3D> metricReconstructedPoints3D = new ArrayList<>();
            List<Point3D> euclideanReconstructedPoints3D = new ArrayList<>();
            for (int i = 0; i < numPoints; i++) {
                metricReconstructedPoints3D.add(
                        mMetricReconstructedPoints.get(i).getPoint());
                euclideanReconstructedPoints3D.add(
                        mEuclideanReconstructedPoints.get(i).getPoint());
            }

            //check that all points are in front of both cameras
            for (int i = 0; i < numPoints; i++) {
                Point3D p = metricReconstructedPoints3D.get(i);
                Point3D pe = euclideanReconstructedPoints3D.get(i);

                assertTrue(estimatedMetricCamera1.isPointInFrontOfCamera(p));
                assertTrue(estimatedMetricCamera2.isPointInFrontOfCamera(p));

                assertTrue(estimatedEuclideanCamera1.isPointInFrontOfCamera(pe));
                assertTrue(estimatedEuclideanCamera2.isPointInFrontOfCamera(pe));
            }

            PinholeCameraIntrinsicParameters euclideanIntrinsic1 =
                    estimatedEuclideanCamera1.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters euclideanIntrinsic2 =
                    estimatedEuclideanCamera2.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters euclideanIntrinsic3 =
                    estimatedEuclideanCamera3.getIntrinsicParameters();

            Rotation3D euclideanRotation1 = estimatedEuclideanCamera1.getCameraRotation();
            Rotation3D euclideanRotation2 = estimatedEuclideanCamera2.getCameraRotation();
            Rotation3D euclideanRotation3 = estimatedEuclideanCamera3.getCameraRotation();

            assertEquals(euclideanIntrinsic1.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertEquals(euclideanIntrinsic2.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertEquals(euclideanIntrinsic3.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic3.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic3.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic3.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic3.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertTrue(euclideanRotation1.asInhomogeneousMatrix().equals(
                    rotation1.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
            assertTrue(euclideanRotation2.asInhomogeneousMatrix().equals(
                    rotation2.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
            assertTrue(euclideanRotation3.asInhomogeneousMatrix().equals(
                    rotation3.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

            //check that points are correct (after scale correction)

            MetricTransformation3D scaleAndOrientationTransformation
                    = new MetricTransformation3D(mScale2);
            scaleAndOrientationTransformation.setRotation(rotation1.inverseRotationAndReturnNew());


            int numValidPoints = 0;
            for (int i = 0; i < numPoints; i++) {
                Point3D point = points3D.get(i);
                Point3D euclideanPoint = euclideanReconstructedPoints3D.get(i);

                //check metric points
                Point3D rescaledPoint = Point3D.create();
                scaleAndOrientationTransformation.transform(metricReconstructedPoints3D.get(i),
                        rescaledPoint);

                assertTrue(euclideanPoint.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR));

                rescaledPoint.setInhomogeneousCoordinates(
                        rescaledPoint.getInhomX() * baseline / mScale2,
                        rescaledPoint.getInhomY() * baseline / mScale2,
                        rescaledPoint.getInhomZ() * baseline / mScale2);
                if (point.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR)) {
                    numValidPoints++;
                }
            }

            if (numValidPoints == 0) {
                continue;
            }

            numValid++;

            if (numValid > 0) {
                break;
            }
        }

        assertTrue(numValid > 0);
    }

    private void reset() {
        mViewCount = 0;
        mEstimatedFundamentalMatrix = mEstimatedFundamentalMatrix2 = null;
        mEstimatedMetricCamera1 = mEstimatedMetricCamera2 =
                mEstimatedMetricCamera3 = null;
        mEstimatedEuclideanCamera1 = mEstimatedEuclideanCamera2 =
                mEstimatedEuclideanCamera3 = null;
        mMetricReconstructedPoints = null;
        mEuclideanReconstructedPoints = null;
        mStarted = mFinished = mFailed = mCancelled = false;
        mScale = 0.0;
        mTimestamp = 0;
    }

    private AbsoluteOrientationConstantVelocityModelSlamCalibrator createFinishedCalibrator(
            float accelerationOffsetX, float accelerationOffsetY,
            float accelerationOffsetZ, float angularOffsetX,
            float angularOffsetY, float angularOffsetZ,
            GaussianRandomizer noiseRandomizer) {
        AbsoluteOrientationConstantVelocityModelSlamCalibrator calibrator =
                AbsoluteOrientationConstantVelocityModelSlamEstimator.createCalibrator();
        calibrator.setConvergenceThreshold(ABSOLUTE_ERROR);
        calibrator.setMaxNumSamples(MAX_CALIBRATION_SAMPLES);

        long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;

        float accelerationNoiseX, accelerationNoiseY, accelerationNoiseZ;
        float angularNoiseX, angularNoiseY, angularNoiseZ;

        double accelerationX, accelerationY, accelerationZ;
        double angularX, angularY, angularZ;
        Quaternion orientation = new Quaternion();

        for(int i = 0; i < MAX_CALIBRATION_SAMPLES; i++) {
            accelerationNoiseX = noiseRandomizer.nextFloat();
            accelerationNoiseY = noiseRandomizer.nextFloat();
            accelerationNoiseZ = noiseRandomizer.nextFloat();

            angularNoiseX = noiseRandomizer.nextFloat();
            angularNoiseY = noiseRandomizer.nextFloat();
            angularNoiseZ = noiseRandomizer.nextFloat();

            accelerationX = accelerationOffsetX + accelerationNoiseX;
            accelerationY = accelerationOffsetY + accelerationNoiseY;
            accelerationZ = accelerationOffsetZ + accelerationNoiseZ;

            angularX = angularOffsetX + angularNoiseX;
            angularY = angularOffsetY + angularNoiseY;
            angularZ = angularOffsetZ + angularNoiseZ;

            calibrator.updateAccelerometerSample(timestamp, (float)accelerationX,
                    (float)accelerationY, (float)accelerationZ);
            calibrator.updateGyroscopeSample(timestamp, (float)angularX, (float)angularY,
                    (float)angularZ);
            calibrator.updateOrientationSample(timestamp, orientation);

            if(calibrator.isFinished()) break;

            timestamp += DELTA_NANOS;
        }

        return calibrator;
    }

}
