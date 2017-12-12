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
import com.irurueta.geometry.slam.SlamCalibrationData;
import com.irurueta.geometry.slam.SlamCalibrator;
import com.irurueta.geometry.slam.SlamEstimator;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.logging.Level;
import java.util.logging.Logger;

import static org.junit.Assert.*;

public class SlamSparseReconstructorTest {

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

    private static final int MIN_TRACKED_POINTS = 10;
    private static final double NEAREST_THRESHOLD = 1e-6;

    //5% of relative error in scale estimation
    private static final double RELATIVE_ERROR = 0.05;

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

    private static final Logger LOGGER = Logger.getLogger(
            SlamSparseReconstructorTest.class.getSimpleName());

    private int mViewCount = 0;
    private EstimatedFundamentalMatrix mEstimatedFundamentalMatrix;
    private EstimatedFundamentalMatrix mEstimatedFundamentalMatrix2;
    private EstimatedFundamentalMatrix mEstimatedFundamentalMatrix3;
    private EstimatedCamera mEstimatedMetricCamera1;
    private EstimatedCamera mEstimatedMetricCamera2;
    private EstimatedCamera mEstimatedMetricCamera3;
    private EstimatedCamera mEstimatedMetricCamera4;
    private EstimatedCamera mEstimatedEuclideanCamera1;
    private EstimatedCamera mEstimatedEuclideanCamera2;
    private EstimatedCamera mEstimatedEuclideanCamera3;
    private EstimatedCamera mEstimatedEuclideanCamera4;
    private List<ReconstructedPoint3D> mMetricReconstructedPoints;
    private List<ReconstructedPoint3D> mEuclideanReconstructedPoints;

    private double mScale;
    private double mScale2;
    private double mScale3;

    private boolean mStarted;
    private boolean mFinished;
    private boolean mFailed;
    private boolean mCancelled;

    private long mTimestamp;

    private int mSlamDataAvailable;
    private int mSlamCameraEstimated;

    private PinholeCamera mSlamCamera;
    private Matrix mSlamCovariance;

    public SlamSparseReconstructorTest() { }

    @BeforeClass
    public static void setUpClass() { }

    @AfterClass
    public static void tearDownClass() { }

    @Before
    public void setUp() {
        mViewCount = 0;
        mEstimatedFundamentalMatrix = mEstimatedFundamentalMatrix2 =
                mEstimatedFundamentalMatrix3 = null;
        mEstimatedMetricCamera1 = mEstimatedMetricCamera2 =
                mEstimatedMetricCamera3 = mEstimatedMetricCamera4 = null;
        mEstimatedEuclideanCamera1 = mEstimatedEuclideanCamera2 =
                mEstimatedEuclideanCamera3 = mEstimatedEuclideanCamera4 = null;
        mMetricReconstructedPoints = null;
        mEuclideanReconstructedPoints = null;
        mStarted = mFinished = mFailed = mCancelled = false;
        mTimestamp = 0;
        mSlamDataAvailable = 0;
        mSlamCameraEstimated = 0;
        mSlamCamera = null;
        mSlamCovariance = null;
    }

    @After
    public void tearDown() { }

    @Test
    public void testConstructor() {
        assertEquals(SlamSparseReconstructor.MIN_NUMBER_OF_VIEWS, 2);

        SlamSparseReconstructorConfiguration configuration =
                new SlamSparseReconstructorConfiguration();
        SlamSparseReconstructorListener listener =
                new SlamSparseReconstructorListener() {
                    @Override
                    public void onSlamDataAvailable(SlamSparseReconstructor reconstructor,
                                                    double positionX, double positionY, double positionZ,
                                                    double velocityX, double velocityY, double velocityZ,
                                                    double accelerationX, double accelerationY, double accelerationZ,
                                                    double quaternionA, double quaternionB, double quaternionC, double quaternionD,
                                                    double angularSpeedX, double angularSpeedY, double angularSpeedZ,
                                                    Matrix covariance) { }

                    @Override
                    public void onSlamCameraEstimated(SlamSparseReconstructor reconstructor, PinholeCamera camera) { }

                    @Override
                    public boolean hasMoreViewsAvailable(SlamSparseReconstructor reconstructor) {
                        return false;
                    }

                    @Override
                    public void onRequestSamples(SlamSparseReconstructor reconstructor,
                                                 int previousViewId, int currentViewId,
                                                 List<Sample2D> previousViewTrackedSamples,
                                                 List<Sample2D> currentViewTrackedSamples,
                                                 List<Sample2D> currentViewNewlySpawnedSamples) { }

                    @Override
                    public void onSamplesAccepted(SlamSparseReconstructor reconstructor, int viewId,
                                                  List<Sample2D> previousViewTrackedSamples,
                                                  List<Sample2D> currentViewTrackedSamples) { }

                    @Override
                    public void onSamplesRejected(SlamSparseReconstructor reconstructor, int viewId,
                                                  List<Sample2D> previousViewTrackedSamples,
                                                  List<Sample2D> currentViewTrackedSamples) { }

                    @Override
                    public void onRequestMatches(SlamSparseReconstructor reconstructor,
                                                 List<Sample2D> allPreviousViewSamples,
                                                 List<Sample2D> previousViewTrackedSamples,
                                                 List<Sample2D> currentViewTrackedSamples,
                                                 int previousViewId, int currentViewId,
                                                 List<MatchedSamples> matches) { }

                    @Override
                    public void onFundamentalMatrixEstimated(SlamSparseReconstructor reconstructor,
                                                             EstimatedFundamentalMatrix estimatedFundamentalMatrix) { }

                    @Override
                    public void onMetricCameraEstimated(SlamSparseReconstructor reconstructor,
                                                        int previousViewId, int currentViewId,
                                                        EstimatedCamera previousCamera,
                                                        EstimatedCamera currentCamera) { }

                    @Override
                    public void onMetricReconstructedPointsEstimated(SlamSparseReconstructor reconstructor,
                                                                     List<MatchedSamples> matches,
                                                                     List<ReconstructedPoint3D> points) { }

                    @Override
                    public void onEuclideanCameraEstimated(SlamSparseReconstructor reconstructor,
                                                           int previousViewId, int currentViewId, double scale,
                                                           EstimatedCamera previousCamera,
                                                           EstimatedCamera currentCamera) { }

                    @Override
                    public void onEuclideanReconstructedPointsEstimated(SlamSparseReconstructor reconstructor,
                                                                        double scale,
                                                                        List<ReconstructedPoint3D> points) { }

                    @Override
                    public void onStart(SlamSparseReconstructor reconstructor) { }

                    @Override
                    public void onFinish(SlamSparseReconstructor reconstructor) { }

                    @Override
                    public void onCancel(SlamSparseReconstructor reconstructor) { }

                    @Override
                    public void onFail(SlamSparseReconstructor reconstructor) { }
                };

        //constructor with listener
        SlamSparseReconstructor reconstructor = new SlamSparseReconstructor(listener);

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
        assertEquals(reconstructor.getCurrentScale(), BaseSparseReconstructor.DEFAULT_SCALE, 0.0);
        assertNull(reconstructor.getPreviousViewTrackedSamples());
        assertNull(reconstructor.getCurrentViewTrackedSamples());
        assertNull(reconstructor.getCurrentViewNewlySpawnedSamples());
        assertTrue(reconstructor.isFirstView());
        assertFalse(reconstructor.isSecondView());
        assertFalse(reconstructor.isAdditionalView());

        //constructor with configuration and listener
        reconstructor = new SlamSparseReconstructor(configuration, listener);

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
        assertEquals(reconstructor.getCurrentScale(), BaseSparseReconstructor.DEFAULT_SCALE, 0.0);
        assertNull(reconstructor.getPreviousViewTrackedSamples());
        assertNull(reconstructor.getCurrentViewTrackedSamples());
        assertNull(reconstructor.getCurrentViewNewlySpawnedSamples());
        assertTrue(reconstructor.isFirstView());
        assertFalse(reconstructor.isSecondView());
        assertFalse(reconstructor.isAdditionalView());
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

            SlamSparseReconstructorConfiguration configuration =
                    new SlamSparseReconstructorConfiguration();
            configuration.setInitialCamerasEstimatorMethod(
                    InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);

            float accelerationOffsetX = 0.0f;
            float accelerationOffsetY = 0.0f;
            float accelerationOffsetZ = 0.0f;

            float angularOffsetX = 0.0f;
            float angularOffsetY = 0.0f;
            float angularOffsetZ = 0.0f;

            SlamCalibrator calibrator = createFinishedCalibrator(
                    accelerationOffsetX, accelerationOffsetY,
                    accelerationOffsetZ, angularOffsetX, angularOffsetY,
                    angularOffsetZ, noiseRandomizer);
            SlamCalibrationData calibrationData
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

            double alphaEuler1 = 0.0;
            double betaEuler1 = 0.0;
            double gammaEuler1 = 0.0;
            double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1,
                    betaEuler1, gammaEuler1);
            MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2,
                    betaEuler2, gammaEuler2);
            AxisRotation3D axisRotation2 = new AxisRotation3D(rotation2);

            double axisX = axisRotation2.getAxisX();
            double axisY = axisRotation2.getAxisY();
            double axisZ = axisRotation2.getAxisZ();
            double angle = axisRotation2.getRotationAngle();

            AxisRotation3D diffRotation = new AxisRotation3D(axisX, axisY,
                    axisZ, angle / N_SENSOR_SAMPLES);
            Quaternion diffQuaternion = new Quaternion(diffRotation);

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

            Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);

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

            final int numPoints1 = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);
            final int numPoints2 = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);

            InhomogeneousPoint3D point3D;
            List<InhomogeneousPoint3D> points3D1 = new ArrayList<>();
            Point2D projectedPoint1, projectedPoint2;
            final List<Point2D> projectedPoints1 = new ArrayList<>();
            final List<Point2D> projectedPoints2 = new ArrayList<>();
            boolean front1, front2;
            for (int i = 0; i < numPoints1; i++) {
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
                        fail("max tries reached");
                    }
                    numTry++;
                } while(!front1 || !front2);
                points3D1.add(point3D);

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

            Point2D projectedPoint2b;
            final List<Point2D> projectedPoints2b = new ArrayList<>();
            for (int i = 0; i < numPoints2; i++) {
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

                    front2 = camera2.isPointInFrontOfCamera(point3D);
                    if (numTry > MAX_TRIES) {
                        fail("max tries reached");
                    }
                    numTry++;
                } while(!front2);
                points3D1.add(point3D);

                //check that 3D point is in front of both cameras
                //noinspection all
                assertTrue(front2);

                projectedPoint2b = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2b);
                projectedPoints2b.add(projectedPoint2b);
            }

            SlamSparseReconstructorListener listener =
                    new SlamSparseReconstructorListener() {
                        @Override
                        public void onSlamDataAvailable(SlamSparseReconstructor reconstructor,
                                                        double positionX, double positionY, double positionZ,
                                                        double velocityX, double velocityY, double velocityZ,
                                                        double accelerationX, double accelerationY, double accelerationZ,
                                                        double quaternionA, double quaternionB, double quaternionC, double quaternionD,
                                                        double angularSpeedX, double angularSpeedY, double angularSpeedZ,
                                                        Matrix covariance) {
                            mSlamDataAvailable++;
                            mSlamCovariance = covariance;
                        }

                        @Override
                        public void onSlamCameraEstimated(SlamSparseReconstructor reconstructor,
                                                          PinholeCamera camera) {
                            mSlamCameraEstimated++;
                            mSlamCamera = camera;
                        }

                        @Override
                        public boolean hasMoreViewsAvailable(SlamSparseReconstructor reconstructor) {
                            return mViewCount < 2;
                        }

                        @Override
                        public void onRequestSamples(SlamSparseReconstructor reconstructor,
                                                     int previousViewId, int currentViewId,
                                                     List<Sample2D> previousViewTrackedSamples,
                                                     List<Sample2D> currentViewTrackedSamples,
                                                     List<Sample2D> currentViewNewlySpawnedSamples) {

                            previousViewTrackedSamples.clear();
                            currentViewTrackedSamples.clear();
                            currentViewNewlySpawnedSamples.clear();

                            Sample2D sample;
                            if (mViewCount == 0) {
                                //first view
                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints1.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewTrackedSamples.add(sample);
                                }
                            } else {
                                //second view
                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints1.get(i));
                                    sample.setViewId(previousViewId);
                                    previousViewTrackedSamples.add(sample);
                                }

                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints2.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewTrackedSamples.add(sample);
                                }

                                //spawned samples
                                for (int i = 0; i < numPoints2; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints2b.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewNewlySpawnedSamples.add(sample);
                                }

                                //assume the following accelerator and gyroscope samples
                                //are obtained during a period of 1 second between 1st
                                //and 2nd view (50 samples * 0.02 s/sample = 1 second)
                                mTimestamp = 0;
                                for (int s = 0; s < N_SENSOR_SAMPLES; s++) {
                                    reconstructor.updateAccelerometerSample(mTimestamp,
                                            (float) accelerationX, (float) accelerationY,
                                            (float) accelerationZ);
                                    reconstructor.updateGyroscopeSample(mTimestamp,
                                            (float) angularSpeedX, (float) angularSpeedY,
                                            (float) angularSpeedZ);
                                    mTimestamp += DELTA_NANOS;
                                }
                            }
                        }

                        @Override
                        public void onSamplesAccepted(SlamSparseReconstructor reconstructor, int viewId,
                                                      List<Sample2D> previousViewTrackedSamples,
                                                      List<Sample2D> currentViewTrackedSamples) {
                            mViewCount++;
                        }

                        @Override
                        public void onSamplesRejected(SlamSparseReconstructor reconstructor, int viewId,
                                                      List<Sample2D> previousViewTrackedSamples,
                                                      List<Sample2D> currentViewTrackedSamples) {
                            mViewCount++;
                        }

                        @Override
                        public void onRequestMatches(SlamSparseReconstructor reconstructor,
                                                     List<Sample2D> allPreviousViewSamples,
                                                     List<Sample2D> previousViewTrackedSamples,
                                                     List<Sample2D> currentViewTrackedSamples,
                                                     int previousViewId, int currentViewId,
                                                     List<MatchedSamples> matches) {
                            matches.clear();

                            MatchedSamples match;
                            for (int i = 0; i < numPoints1; i++) {
                                match = new MatchedSamples();
                                match.setSamples(new Sample2D[]{
                                        previousViewTrackedSamples.get(i), currentViewTrackedSamples.get(i)
                                });
                                match.setViewIds(new int[]{previousViewId, currentViewId});
                                matches.add(match);
                            }
                        }

                        @Override
                        public void onFundamentalMatrixEstimated(SlamSparseReconstructor reconstructor,
                                                                 EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                            mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                        }

                        @Override
                        public void onMetricCameraEstimated(SlamSparseReconstructor reconstructor,
                                                            int previousViewId, int currentViewId,
                                                            EstimatedCamera previousCamera,
                                                            EstimatedCamera currentCamera) {
                            mEstimatedMetricCamera1 = previousCamera;
                            mEstimatedMetricCamera2 = currentCamera;
                        }

                        @Override
                        public void onMetricReconstructedPointsEstimated(SlamSparseReconstructor reconstructor,
                                                                         List<MatchedSamples> matches,
                                                                         List<ReconstructedPoint3D> points) {
                            mMetricReconstructedPoints = points;
                        }

                        @Override
                        public void onEuclideanCameraEstimated(SlamSparseReconstructor reconstructor,
                                                               int previousViewId, int currentViewId, double scale,
                                                               EstimatedCamera previousCamera,
                                                               EstimatedCamera currentCamera) {
                            mEstimatedEuclideanCamera1 = previousCamera;
                            mEstimatedEuclideanCamera2 = currentCamera;
                            mScale = scale;
                        }

                        @Override
                        public void onEuclideanReconstructedPointsEstimated(
                                SlamSparseReconstructor reconstructor,
                                double scale, List<ReconstructedPoint3D> points) {
                            mEuclideanReconstructedPoints = points;
                            mScale = scale;
                        }

                        @Override
                        public void onStart(SlamSparseReconstructor reconstructor) {
                            mStarted = true;
                        }

                        @Override
                        public void onFinish(SlamSparseReconstructor reconstructor) {
                            mFinished = true;
                        }

                        @Override
                        public void onCancel(SlamSparseReconstructor reconstructor) {
                            mCancelled = true;
                        }

                        @Override
                        public void onFail(SlamSparseReconstructor reconstructor) {
                            mFailed = true;
                        }
                    };

            SlamSparseReconstructor reconstructor =
                    new SlamSparseReconstructor(configuration, listener);

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
            assertTrue(mSlamDataAvailable > 0);
            assertTrue(mSlamCameraEstimated > 0);
            assertNotNull(mSlamCamera);
            assertNotNull(mSlamCovariance);
            assertFalse(reconstructor.isFirstView());
            assertFalse(reconstructor.isSecondView());
            assertTrue(reconstructor.isAdditionalView());
            assertTrue(reconstructor.getViewCount() > 0);
            assertNotNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertSame(reconstructor.getCurrentEstimatedFundamentalMatrix(), mEstimatedFundamentalMatrix);
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertSame(reconstructor.getCurrentMetricEstimatedCamera(), mEstimatedMetricCamera2);
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertSame(reconstructor.getPreviousMetricEstimatedCamera(), mEstimatedMetricCamera1);
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(reconstructor.getCurrentEuclideanEstimatedCamera(), mEstimatedEuclideanCamera2);
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(reconstructor.getPreviousEuclideanEstimatedCamera(), mEstimatedEuclideanCamera1);
            assertNotNull(reconstructor.getActiveMetricReconstructedPoints());
            assertSame(reconstructor.getActiveMetricReconstructedPoints(), mMetricReconstructedPoints);
            assertNotNull(reconstructor.getActiveEuclideanReconstructedPoints());
            assertSame(reconstructor.getActiveEuclideanReconstructedPoints(), mEuclideanReconstructedPoints);
            assertEquals(reconstructor.getCurrentScale(), mScale, 0.0);
            assertNotNull(reconstructor.getPreviousViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewNewlySpawnedSamples());

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
            for (int i = 0; i < numPoints1; i++) {
                metricReconstructedPoints3D.add(
                        mMetricReconstructedPoints.get(i).getPoint());
                euclideanReconstructedPoints3D.add(
                        mEuclideanReconstructedPoints.get(i).getPoint());
            }

            //check that all points are in front of both cameras
            for (int i = 0; i < numPoints1; i++) {
                Point3D p = metricReconstructedPoints3D.get(i);
                Point3D pe = euclideanReconstructedPoints3D.get(i);

                assertTrue(estimatedMetricCamera1.isPointInFrontOfCamera(p));
                assertTrue(estimatedMetricCamera2.isPointInFrontOfCamera(p));

                assertTrue(estimatedEuclideanCamera1.isPointInFrontOfCamera(pe));
                assertTrue(estimatedEuclideanCamera2.isPointInFrontOfCamera(pe));
            }

            Point3D euclideanCenter1 = estimatedEuclideanCamera1.getCameraCenter();
            Point3D euclideanCenter2 = estimatedEuclideanCamera2.getCameraCenter();

            PinholeCameraIntrinsicParameters euclideanIntrinsic1 =
                    estimatedEuclideanCamera1.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters euclideanIntrinsic2 =
                    estimatedEuclideanCamera2.getIntrinsicParameters();

            Rotation3D euclideanRotation1 = estimatedEuclideanCamera1.getCameraRotation();
            Rotation3D euclideanRotation2 = estimatedEuclideanCamera2.getCameraRotation();

            double estimatedBaseline = euclideanCenter1.distanceTo(euclideanCenter2);

            //check cameras are correct
            double maxBaseline = Math.max(estimatedBaseline, baseline);
            double absoluteScaleError = RELATIVE_ERROR * maxBaseline;
            if (Math.abs(estimatedBaseline - baseline) > absoluteScaleError) {
                continue;
            }
            assertEquals(estimatedBaseline, baseline, absoluteScaleError);

            assertTrue(center1.equals(euclideanCenter1, ABSOLUTE_ERROR));
            if (!center2.equals(euclideanCenter2, absoluteScaleError)) {
                continue;
            }
            assertTrue(center2.equals(euclideanCenter2, absoluteScaleError));

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

            //check that scale error is less than 5%
            assertTrue(Math.abs(baseline / mScale - 1.0) < RELATIVE_ERROR);

            MetricTransformation3D scaleTransformation
                    = new MetricTransformation3D(mScale);

            int numValidPoints = 0;
            double scaleX, scaleY, scaleZ;
            for (int i = 0; i < numPoints1; i++) {
                Point3D point = points3D1.get(i);
                Point3D euclideanPoint = euclideanReconstructedPoints3D.get(i);

                //check metric points
                Point3D rescaledPoint = Point3D.create();
                scaleTransformation.transform(metricReconstructedPoints3D.get(i),
                        rescaledPoint);

                //euclidean and rescaled points match
                assertTrue(euclideanPoint.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR));

                scaleX = point.getInhomX() / rescaledPoint.getInhomX();
                scaleY = point.getInhomY() / rescaledPoint.getInhomY();
                scaleZ = point.getInhomZ() / rescaledPoint.getInhomZ();

                //check that scale error is less than 5%
                assertEquals(scaleX, baseline / mScale, LARGE_ABSOLUTE_ERROR);
                assertEquals(scaleY, baseline / mScale, LARGE_ABSOLUTE_ERROR);
                assertEquals(scaleZ, baseline / mScale, LARGE_ABSOLUTE_ERROR);
                if (Math.abs(scaleX - 1.0) > RELATIVE_ERROR ||
                        Math.abs(scaleY - 1.0) > RELATIVE_ERROR ||
                        Math.abs(scaleZ - 1.0) > RELATIVE_ERROR) {
                    continue;
                }
                rescaledPoint.setInhomogeneousCoordinates(
                        rescaledPoint.getInhomX() * baseline / mScale,
                        rescaledPoint.getInhomY() * baseline / mScale,
                        rescaledPoint.getInhomZ() * baseline / mScale);
                if (point.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR)) {
                    numValidPoints++;
                }

                //check euclidean points
                scaleX = point.getInhomX() / euclideanPoint.getInhomX();
                scaleY = point.getInhomY() / euclideanPoint.getInhomY();
                scaleZ = point.getInhomZ() / euclideanPoint.getInhomZ();

                //check that scale error is less than 5%
                assertEquals(scaleX, baseline / mScale, LARGE_ABSOLUTE_ERROR);
                assertEquals(scaleY, baseline / mScale, LARGE_ABSOLUTE_ERROR);
                assertEquals(scaleZ, baseline / mScale, LARGE_ABSOLUTE_ERROR);
                assertTrue(Math.abs(scaleX - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleY - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleZ - 1.0) < RELATIVE_ERROR);
            }

            if (numValidPoints == 0) {
                continue;
            }

            double scaleRelativeError = Math.abs(baseline / mScale - 1.0);
            LOGGER.log(Level.INFO,
                    "Baseline relative error without noise: {0,number,0.000%}",
                    scaleRelativeError);

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

            SlamSparseReconstructorConfiguration configuration =
                    new SlamSparseReconstructorConfiguration();
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

            SlamCalibrator calibrator = createFinishedCalibrator(
                    accelerationOffsetX, accelerationOffsetY,
                    accelerationOffsetZ, angularOffsetX, angularOffsetY,
                    angularOffsetZ, noiseRandomizer);
            SlamCalibrationData calibrationData
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

            double alphaEuler1 = 0.0;
            double betaEuler1 = 0.0;
            double gammaEuler1 = 0.0;
            double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1,
                    betaEuler1, gammaEuler1);
            MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2,
                    betaEuler2, gammaEuler2);
            AxisRotation3D axisRotation2 = new AxisRotation3D(rotation2);

            double axisX = axisRotation2.getAxisX();
            double axisY = axisRotation2.getAxisY();
            double axisZ = axisRotation2.getAxisZ();
            double angle = axisRotation2.getRotationAngle();

            AxisRotation3D diffRotation = new AxisRotation3D(axisX, axisY,
                    axisZ, angle / N_SENSOR_SAMPLES);
            Quaternion diffQuaternion = new Quaternion(diffRotation);

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

            Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);

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

            final int numPoints1 = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);
            final int numPoints2 = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);

            InhomogeneousPoint3D point3D;
            List<InhomogeneousPoint3D> points3D1 = new ArrayList<>();
            Point2D projectedPoint1, projectedPoint2;
            final List<Point2D> projectedPoints1 = new ArrayList<>();
            final List<Point2D> projectedPoints2 = new ArrayList<>();
            boolean front1, front2;
            for (int i = 0; i < numPoints1; i++) {
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
                        fail("max tries reached");
                    }
                    numTry++;
                } while(!front1 || !front2);
                points3D1.add(point3D);

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

            Point2D projectedPoint2b;
            final List<Point2D> projectedPoints2b = new ArrayList<>();
            for (int i = 0; i < numPoints2; i++) {
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

                    front2 = camera2.isPointInFrontOfCamera(point3D);
                    if (numTry > MAX_TRIES) {
                        fail("max tries reached");
                    }
                    numTry++;
                } while(!front2);

                //check that 3D point is in front of both cameras
                //noinspection all
                assertTrue(front2);

                projectedPoint2b = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2b);
                projectedPoints2b.add(projectedPoint2b);
            }

            final GaussianRandomizer accelerationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);

            SlamSparseReconstructorListener listener =
                    new SlamSparseReconstructorListener() {
                        @Override
                        public void onSlamDataAvailable(SlamSparseReconstructor reconstructor,
                                                        double positionX, double positionY, double positionZ,
                                                        double velocityX, double velocityY, double velocityZ,
                                                        double accelerationX, double accelerationY, double accelerationZ,
                                                        double quaternionA, double quaternionB, double quaternionC, double quaternionD,
                                                        double angularSpeedX, double angularSpeedY, double angularSpeedZ,
                                                        Matrix covariance) {
                            mSlamDataAvailable++;
                            mSlamCovariance = covariance;
                        }

                        @Override
                        public void onSlamCameraEstimated(SlamSparseReconstructor reconstructor,
                                                          PinholeCamera camera) {
                            mSlamCameraEstimated++;
                            mSlamCamera = camera;
                        }

                        @Override
                        public boolean hasMoreViewsAvailable(SlamSparseReconstructor reconstructor) {
                            return mViewCount < 2;
                        }

                        @Override
                        public void onRequestSamples(SlamSparseReconstructor reconstructor,
                                                     int previousViewId, int currentViewId,
                                                     List<Sample2D> previousViewTrackedSamples,
                                                     List<Sample2D> currentViewTrackedSamples,
                                                     List<Sample2D> currentViewNewlySpawnedSamples) {

                            previousViewTrackedSamples.clear();
                            currentViewTrackedSamples.clear();
                            currentViewNewlySpawnedSamples.clear();

                            Sample2D sample;
                            if (mViewCount == 0) {
                                //first view
                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints1.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewTrackedSamples.add(sample);
                                }
                            } else {
                                //second view
                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints1.get(i));
                                    sample.setViewId(previousViewId);
                                    previousViewTrackedSamples.add(sample);
                                }

                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints2.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewTrackedSamples.add(sample);
                                }

                                //spawned samples
                                for (int i = 0; i < numPoints2; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints2b.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewNewlySpawnedSamples.add(sample);
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
                                    mTimestamp += DELTA_NANOS;
                                }
                            }
                        }

                        @Override
                        public void onSamplesAccepted(SlamSparseReconstructor reconstructor, int viewId,
                                                      List<Sample2D> previousViewTrackedSamples,
                                                      List<Sample2D> currentViewTrackedSamples) {
                            mViewCount++;
                        }

                        @Override
                        public void onSamplesRejected(SlamSparseReconstructor reconstructor, int viewId,
                                                      List<Sample2D> previousViewTrackedSamples,
                                                      List<Sample2D> currentViewTrackedSamples) {
                            mViewCount++;
                        }

                        @Override
                        public void onRequestMatches(SlamSparseReconstructor reconstructor,
                                                     List<Sample2D> allPreviousViewSamples,
                                                     List<Sample2D> previousViewTrackedSamples,
                                                     List<Sample2D> currentViewTrackedSamples,
                                                     int previousViewId, int currentViewId,
                                                     List<MatchedSamples> matches) {
                            matches.clear();

                            MatchedSamples match;
                            for (int i = 0; i < numPoints1; i++) {
                                match = new MatchedSamples();
                                match.setSamples(new Sample2D[]{
                                        previousViewTrackedSamples.get(i), currentViewTrackedSamples.get(i)
                                });
                                match.setViewIds(new int[]{previousViewId, currentViewId});
                                matches.add(match);
                            }
                        }

                        @Override
                        public void onFundamentalMatrixEstimated(SlamSparseReconstructor reconstructor,
                                                                 EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                            mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                        }

                        @Override
                        public void onMetricCameraEstimated(SlamSparseReconstructor reconstructor,
                                                            int previousViewId, int currentViewId,
                                                            EstimatedCamera previousCamera,
                                                            EstimatedCamera currentCamera) {
                            mEstimatedMetricCamera1 = previousCamera;
                            mEstimatedMetricCamera2 = currentCamera;
                        }

                        @Override
                        public void onMetricReconstructedPointsEstimated(SlamSparseReconstructor reconstructor,
                                                                         List<MatchedSamples> matches,
                                                                         List<ReconstructedPoint3D> points) {
                            mMetricReconstructedPoints = points;
                        }

                        @Override
                        public void onEuclideanCameraEstimated(SlamSparseReconstructor reconstructor,
                                                               int previousViewId, int currentViewId, double scale,
                                                               EstimatedCamera previousCamera,
                                                               EstimatedCamera currentCamera) {
                            mEstimatedEuclideanCamera1 = previousCamera;
                            mEstimatedEuclideanCamera2 = currentCamera;
                            mScale = scale;
                        }

                        @Override
                        public void onEuclideanReconstructedPointsEstimated(
                                SlamSparseReconstructor reconstructor,
                                double scale, List<ReconstructedPoint3D> points) {
                            mEuclideanReconstructedPoints = points;
                            mScale = scale;
                        }

                        @Override
                        public void onStart(SlamSparseReconstructor reconstructor) {
                            mStarted = true;
                        }

                        @Override
                        public void onFinish(SlamSparseReconstructor reconstructor) {
                            mFinished = true;
                        }

                        @Override
                        public void onCancel(SlamSparseReconstructor reconstructor) {
                            mCancelled = true;
                        }

                        @Override
                        public void onFail(SlamSparseReconstructor reconstructor) {
                            mFailed = true;
                        }
                    };

            SlamSparseReconstructor reconstructor =
                    new SlamSparseReconstructor(configuration, listener);

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
            assertTrue(mSlamDataAvailable > 0);
            assertTrue(mSlamCameraEstimated > 0);
            assertNotNull(mSlamCamera);
            assertNotNull(mSlamCovariance);
            assertFalse(reconstructor.isFirstView());
            assertFalse(reconstructor.isSecondView());
            assertTrue(reconstructor.isAdditionalView());
            assertTrue(reconstructor.getViewCount() > 0);
            assertNotNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertSame(reconstructor.getCurrentEstimatedFundamentalMatrix(), mEstimatedFundamentalMatrix);
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertSame(reconstructor.getCurrentMetricEstimatedCamera(), mEstimatedMetricCamera2);
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertSame(reconstructor.getPreviousMetricEstimatedCamera(), mEstimatedMetricCamera1);
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(reconstructor.getCurrentEuclideanEstimatedCamera(), mEstimatedEuclideanCamera2);
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(reconstructor.getPreviousEuclideanEstimatedCamera(), mEstimatedEuclideanCamera1);
            assertNotNull(reconstructor.getActiveMetricReconstructedPoints());
            assertSame(reconstructor.getActiveMetricReconstructedPoints(), mMetricReconstructedPoints);
            assertNotNull(reconstructor.getActiveEuclideanReconstructedPoints());
            assertSame(reconstructor.getActiveEuclideanReconstructedPoints(), mEuclideanReconstructedPoints);
            assertEquals(reconstructor.getCurrentScale(), mScale, 0.0);
            assertNotNull(reconstructor.getPreviousViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewNewlySpawnedSamples());

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
            for (int i = 0; i < numPoints1; i++) {
                metricReconstructedPoints3D.add(
                        mMetricReconstructedPoints.get(i).getPoint());
                euclideanReconstructedPoints3D.add(
                        mEuclideanReconstructedPoints.get(i).getPoint());
            }

            //check that all points are in front of both cameras
            for (int i = 0; i < numPoints1; i++) {
                Point3D p = metricReconstructedPoints3D.get(i);
                Point3D pe = euclideanReconstructedPoints3D.get(i);

                assertTrue(estimatedMetricCamera1.isPointInFrontOfCamera(p));
                assertTrue(estimatedMetricCamera2.isPointInFrontOfCamera(p));

                assertTrue(estimatedEuclideanCamera1.isPointInFrontOfCamera(pe));
                assertTrue(estimatedEuclideanCamera2.isPointInFrontOfCamera(pe));
            }

            Point3D euclideanCenter1 = estimatedEuclideanCamera1.getCameraCenter();
            Point3D euclideanCenter2 = estimatedEuclideanCamera2.getCameraCenter();

            PinholeCameraIntrinsicParameters euclideanIntrinsic1 =
                    estimatedEuclideanCamera1.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters euclideanIntrinsic2 =
                    estimatedEuclideanCamera2.getIntrinsicParameters();

            Rotation3D euclideanRotation1 = estimatedEuclideanCamera1.getCameraRotation();
            Rotation3D euclideanRotation2 = estimatedEuclideanCamera2.getCameraRotation();

            double estimatedBaseline = euclideanCenter1.distanceTo(euclideanCenter2);

            //check cameras are correct
            double maxBaseline = Math.max(estimatedBaseline, baseline);
            double absoluteScaleError = RELATIVE_ERROR * maxBaseline;
            if (Math.abs(estimatedBaseline - baseline) > absoluteScaleError) {
                continue;
            }
            assertEquals(estimatedBaseline, baseline, absoluteScaleError);

            assertTrue(center1.equals(euclideanCenter1, ABSOLUTE_ERROR));
            if (!center2.equals(euclideanCenter2, absoluteScaleError)) {
                continue;
            }
            assertTrue(center2.equals(euclideanCenter2, absoluteScaleError));

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

            //check that scale error is less than 5%
            assertTrue(Math.abs(baseline / mScale - 1.0) < RELATIVE_ERROR);

            MetricTransformation3D scaleTransformation
                    = new MetricTransformation3D(mScale);

            int numValidPoints = 0;
            double scaleX, scaleY, scaleZ;
            for (int i = 0; i < numPoints1; i++) {
                Point3D point = points3D1.get(i);
                Point3D euclideanPoint = euclideanReconstructedPoints3D.get(i);

                //check metric points
                Point3D rescaledPoint = Point3D.create();
                scaleTransformation.transform(metricReconstructedPoints3D.get(i),
                        rescaledPoint);

                //euclidean and rescaled points match
                assertTrue(euclideanPoint.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR));

                scaleX = point.getInhomX() / rescaledPoint.getInhomX();
                scaleY = point.getInhomY() / rescaledPoint.getInhomY();
                scaleZ = point.getInhomZ() / rescaledPoint.getInhomZ();

                //check that scale error is less than 5%
                assertEquals(scaleX, baseline / mScale, LARGE_ABSOLUTE_ERROR);
                assertEquals(scaleY, baseline / mScale, LARGE_ABSOLUTE_ERROR);
                assertEquals(scaleZ, baseline / mScale, LARGE_ABSOLUTE_ERROR);
                if (Math.abs(scaleX - 1.0) > RELATIVE_ERROR ||
                        Math.abs(scaleY - 1.0) > RELATIVE_ERROR ||
                        Math.abs(scaleZ - 1.0) > RELATIVE_ERROR) {
                    continue;
                }
                rescaledPoint.setInhomogeneousCoordinates(
                        rescaledPoint.getInhomX() * baseline / mScale,
                        rescaledPoint.getInhomY() * baseline / mScale,
                        rescaledPoint.getInhomZ() * baseline / mScale);
                if (point.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR)) {
                    numValidPoints++;
                }

                //check euclidean points
                scaleX = point.getInhomX() / euclideanPoint.getInhomX();
                scaleY = point.getInhomY() / euclideanPoint.getInhomY();
                scaleZ = point.getInhomZ() / euclideanPoint.getInhomZ();

                //check that scale error is less than 5%
                assertEquals(scaleX, baseline / mScale, ABSOLUTE_ERROR);
                assertEquals(scaleY, baseline / mScale, ABSOLUTE_ERROR);
                assertEquals(scaleZ, baseline / mScale, ABSOLUTE_ERROR);
                assertTrue(Math.abs(scaleX - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleY - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleZ - 1.0) < RELATIVE_ERROR);
            }

            if (numValidPoints == 0) {
                continue;
            }

            double scaleRelativeError = Math.abs(baseline / mScale - 1.0);
            LOGGER.log(Level.INFO,
                    "Baseline relative error without noise: {0,number,0.000%}",
                    scaleRelativeError);

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

            SlamSparseReconstructorConfiguration configuration =
                    new SlamSparseReconstructorConfiguration();
            configuration.setInitialCamerasEstimatorMethod(
                    InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);

            float accelerationOffsetX = 0.0f;
            float accelerationOffsetY = 0.0f;
            float accelerationOffsetZ = 0.0f;

            float angularOffsetX = 0.0f;
            float angularOffsetY = 0.0f;
            float angularOffsetZ = 0.0f;

            SlamCalibrator calibrator = createFinishedCalibrator(
                    accelerationOffsetX, accelerationOffsetY,
                    accelerationOffsetZ, angularOffsetX, angularOffsetY,
                    angularOffsetZ, noiseRandomizer);
            SlamCalibrationData calibrationData
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

            double alphaEuler1 = 0.0;
            double betaEuler1 = 0.0;
            double gammaEuler1 = 0.0;
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

            MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1,
                    betaEuler1, gammaEuler1);
            MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2,
                    betaEuler2, gammaEuler2);
            AxisRotation3D axisRotation2 = new AxisRotation3D(rotation2);
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
            Quaternion diffRotation2 = new Quaternion(angularSpeed2X,
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
            diffRotation2 = new Quaternion(angularSpeed3X, angularSpeed3Y,
                    angularSpeed3Z);

            //number of samples (50 samples * 0.02 s/sample = 1 second), starting from
            //previously sampled rotation
            MatrixRotation3D rotation3b = new MatrixRotation3D(rotation2b);
            MatrixRotation3D rotation3c = new MatrixRotation3D(rotation2c);
            for (int s = 0; s < N_SENSOR_SAMPLES; s++) {
                rotation3b.combine(diffRotation);
                rotation3c.combine(diffRotation2);
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

            Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);
            Point3D center3 = new InhomogeneousPoint3D(
                    center2.getInhomX() + cameraSeparation2,
                    center2.getInhomY() + cameraSeparation2,
                    center2.getInhomZ() + cameraSeparation2);

            double baseline = center1.distanceTo(center2);
            double baseline2 = center2.distanceTo(center3);

            final double accelerationX, accelerationY, accelerationZ;
            final double accelerationX2, accelerationY2, accelerationZ2;

            //s = 0.5*a*t^2 --> a = 2*s/t^2
            //assuming t = 1 second (50 samples * 0.02 s/sample = 1 second)
            accelerationX = accelerationY = accelerationZ
                    = 2 * cameraSeparation;
            accelerationX2 = accelerationY2 = accelerationZ2
                    = 2 * cameraSeparation2;

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

            final int numPoints1 = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);
            final int numPoints2 = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);
            final int start = randomizer.nextInt(0,
                    numPoints1 - MIN_TRACKED_POINTS);

            InhomogeneousPoint3D point3D;
            List<InhomogeneousPoint3D> points3D1 = new ArrayList<>();
            Point2D projectedPoint1, projectedPoint2, projectedPoint3;
            final List<Point2D> projectedPoints1 = new ArrayList<>();
            final List<Point2D> projectedPoints2 = new ArrayList<>();
            final List<Point2D> projectedPoints3 = new ArrayList<>();
            boolean front1, front2;
            for (int i = 0; i < numPoints1; i++) {
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
                        fail("max tries reached");
                    }
                    numTry++;
                } while(!front1 || !front2);
                points3D1.add(point3D);

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

            List<InhomogeneousPoint3D> points3D2 = new ArrayList<>();
            Point2D projectedPoint2b, projectedPoint3b;
            final List<Point2D> projectedPoints2b = new ArrayList<>();
            final List<Point2D> projectedPoints3b = new ArrayList<>();
            for (int i = 0; i < numPoints2; i++) {
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

                    front2 = camera2.isPointInFrontOfCamera(point3D);
                    if (numTry > MAX_TRIES) {
                        fail("max tries reached");
                    }
                    numTry++;
                } while(!front2);
                points3D2.add(point3D);

                //check that 3D point is in front of both cameras
                //noinspection all
                assertTrue(front2);

                projectedPoint2b = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2b);
                projectedPoints2b.add(projectedPoint2b);

                projectedPoint3b = new InhomogeneousPoint2D();
                camera3.project(point3D, projectedPoint3b);
                projectedPoints3b.add(projectedPoint3b);
            }

            SlamSparseReconstructorListener listener =
                    new SlamSparseReconstructorListener() {
                        @Override
                        public void onSlamDataAvailable(SlamSparseReconstructor reconstructor,
                                                        double positionX, double positionY, double positionZ,
                                                        double velocityX, double velocityY, double velocityZ,
                                                        double accelerationX, double accelerationY, double accelerationZ,
                                                        double quaternionA, double quaternionB, double quaternionC, double quaternionD,
                                                        double angularSpeedX, double angularSpeedY, double angularSpeedZ,
                                                        Matrix covariance) {
                            mSlamDataAvailable++;
                            mSlamCovariance = covariance;
                        }

                        @Override
                        public void onSlamCameraEstimated(SlamSparseReconstructor reconstructor,
                                                          PinholeCamera camera) {
                            mSlamCameraEstimated++;
                            mSlamCamera = camera;
                        }

                        @Override
                        public boolean hasMoreViewsAvailable(SlamSparseReconstructor reconstructor) {
                            return mViewCount < 3;
                        }

                        @Override
                        public void onRequestSamples(SlamSparseReconstructor reconstructor,
                                                     int previousViewId, int currentViewId,
                                                     List<Sample2D> previousViewTrackedSamples,
                                                     List<Sample2D> currentViewTrackedSamples,
                                                     List<Sample2D> currentViewNewlySpawnedSamples) {

                            previousViewTrackedSamples.clear();
                            currentViewTrackedSamples.clear();
                            currentViewNewlySpawnedSamples.clear();

                            Sample2D sample;
                            if (mViewCount == 0) {
                                //first view
                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints1.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewTrackedSamples.add(sample);
                                }
                            } else if (mEstimatedFundamentalMatrix == null) {
                                //second view
                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints1.get(i));
                                    sample.setViewId(previousViewId);
                                    previousViewTrackedSamples.add(sample);
                                }

                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints2.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewTrackedSamples.add(sample);
                                }

                                //spawned samples
                                for (int i = 0; i < numPoints2; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints2b.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewNewlySpawnedSamples.add(sample);
                                }

                                //assume the following accelerator and gyroscope samples
                                //are obtained during a period of 1 second between 1st
                                //and 2nd view (50 samples * 0.02 s/sample = 1 second)
                                mTimestamp = 0;
                                for (int s = 0; s < N_SENSOR_SAMPLES; s++) {
                                    reconstructor.updateAccelerometerSample(mTimestamp,
                                            (float) accelerationX, (float) accelerationY,
                                            (float) accelerationZ);
                                    reconstructor.updateGyroscopeSample(mTimestamp,
                                            (float) angularSpeed2X, (float) angularSpeed2Y,
                                            (float) angularSpeed2Z);
                                    mTimestamp += DELTA_NANOS;
                                }

                            } else {
                                //third view
                                for (int i = start; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints2.get(i));
                                    sample.setViewId(previousViewId);
                                    previousViewTrackedSamples.add(sample);
                                }

                                for (int i = 0; i < numPoints2; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints2b.get(i));
                                    sample.setViewId(previousViewId);
                                    previousViewTrackedSamples.add(sample);
                                }


                                for (int i = start; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints3.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewTrackedSamples.add(sample);
                                }

                                for (int i = 0; i < numPoints2; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints3b.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewTrackedSamples.add(sample);
                                }

                                //assume the following accelerator and gyroscope samples
                                //are obtained during a period of 1 second between 2nd
                                //and 3rd view (50 samples * 0.02 s/sample = 1 second)
                                for (int s = 0; s < N_SENSOR_SAMPLES; s++) {
                                    reconstructor.updateAccelerometerSample(mTimestamp,
                                            (float) accelerationX2, (float) accelerationY2,
                                            (float) accelerationZ2);
                                    reconstructor.updateGyroscopeSample(mTimestamp,
                                            (float) angularSpeed3X, (float) angularSpeed3Y,
                                            (float) angularSpeed3Z);
                                    mTimestamp += DELTA_NANOS;
                                }
                            }
                        }

                        @Override
                        public void onSamplesAccepted(SlamSparseReconstructor reconstructor, int viewId,
                                                      List<Sample2D> previousViewTrackedSamples,
                                                      List<Sample2D> currentViewTrackedSamples) {
                            mViewCount++;
                        }

                        @Override
                        public void onSamplesRejected(SlamSparseReconstructor reconstructor, int viewId,
                                                      List<Sample2D> previousViewTrackedSamples,
                                                      List<Sample2D> currentViewTrackedSamples) {
                            mViewCount++;
                        }

                        @Override
                        public void onRequestMatches(SlamSparseReconstructor reconstructor,
                                                     List<Sample2D> allPreviousViewSamples,
                                                     List<Sample2D> previousViewTrackedSamples,
                                                     List<Sample2D> currentViewTrackedSamples,
                                                     int previousViewId, int currentViewId,
                                                     List<MatchedSamples> matches) {
                            matches.clear();

                            int numCameras = 0;
                            if (mEstimatedMetricCamera1 != null &&
                                    (mEstimatedMetricCamera1.getViewId() == previousViewId ||
                                            mEstimatedMetricCamera1.getViewId() == currentViewId)) {
                                numCameras++;
                            }
                            if (mEstimatedMetricCamera2 != null &&
                                    (mEstimatedMetricCamera2.getViewId() == previousViewId ||
                                            mEstimatedMetricCamera2.getViewId() == currentViewId)) {
                                numCameras++;
                            }

                            EstimatedCamera[] estimatedCameras = null;
                            if (numCameras > 0) {
                                estimatedCameras = new EstimatedCamera[numCameras];


                                int pos = 0;
                                if (mEstimatedMetricCamera1 != null &&
                                        (mEstimatedMetricCamera1.getViewId() == previousViewId ||
                                                mEstimatedMetricCamera1.getViewId() == currentViewId)) {
                                    estimatedCameras[pos] = mEstimatedMetricCamera1;
                                    pos++;
                                }
                                if (mEstimatedMetricCamera2 != null &&
                                        (mEstimatedMetricCamera2.getViewId() == previousViewId ||
                                                mEstimatedMetricCamera2.getViewId() == currentViewId)) {
                                    estimatedCameras[pos] = mEstimatedMetricCamera2;
                                }
                            }

                            List<Point2D> allPreviousPoints = new ArrayList<>();
                            for (Sample2D sample : allPreviousViewSamples) {
                                allPreviousPoints.add(sample.getPoint());
                            }
                            KDTree2D tree = new KDTree2D(allPreviousPoints);

                            //search previous view tracked samples within tree
                            int numTrackedSamples = previousViewTrackedSamples.size();
                            Point2D point, nearestPoint;
                            int nearestIndex;
                            MatchedSamples match;
                            for (int i = 0; i < numTrackedSamples; i++) {
                                Sample2D previousSample = previousViewTrackedSamples.get(i);
                                point = previousSample.getPoint();
                                nearestIndex = tree.nearestIndex(point);
                                nearestPoint = allPreviousPoints.get(nearestIndex);
                                Sample2D nearestSample = allPreviousViewSamples.get(nearestIndex);

                                if (point.distanceTo(nearestPoint) > NEAREST_THRESHOLD) {
                                    continue;
                                }

                                Sample2D currentSample = currentViewTrackedSamples.get(i);

                                match = new MatchedSamples();
                                match.setSamples(new Sample2D[]{
                                        previousSample, currentSample
                                });
                                match.setViewIds(new int[]{previousViewId, currentViewId});

                                match.setReconstructedPoint(nearestSample.getReconstructedPoint());

                                if (estimatedCameras != null) {
                                    match.setCameras(estimatedCameras);
                                }

                                matches.add(match);
                            }
                        }

                        @Override
                        public void onFundamentalMatrixEstimated(SlamSparseReconstructor reconstructor,
                                                                 EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                            if (mEstimatedFundamentalMatrix == null) {
                                mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                            } else if (mEstimatedFundamentalMatrix2 == null) {
                                mEstimatedFundamentalMatrix2 = estimatedFundamentalMatrix;
                            }
                        }

                        @Override
                        public void onMetricCameraEstimated(SlamSparseReconstructor reconstructor,
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
                        public void onMetricReconstructedPointsEstimated(SlamSparseReconstructor reconstructor,
                                                                         List<MatchedSamples> matches,
                                                                         List<ReconstructedPoint3D> points) {
                            mMetricReconstructedPoints = points;
                        }

                        @Override
                        public void onEuclideanCameraEstimated(SlamSparseReconstructor reconstructor,
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
                                SlamSparseReconstructor reconstructor,
                                double scale, List<ReconstructedPoint3D> points) {
                            if (mEuclideanReconstructedPoints == null) {
                                mScale = scale;
                            } else {
                                mScale2 = scale;
                            }

                            mEuclideanReconstructedPoints = points;
                        }

                        @Override
                        public void onStart(SlamSparseReconstructor reconstructor) {
                            mStarted = true;
                        }

                        @Override
                        public void onFinish(SlamSparseReconstructor reconstructor) {
                            mFinished = true;
                        }

                        @Override
                        public void onCancel(SlamSparseReconstructor reconstructor) {
                            mCancelled = true;
                        }

                        @Override
                        public void onFail(SlamSparseReconstructor reconstructor) {
                            mFailed = true;
                        }
                    };

            SlamSparseReconstructor reconstructor =
                    new SlamSparseReconstructor(configuration, listener);

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
            assertTrue(mSlamDataAvailable > 0);
            assertTrue(mSlamCameraEstimated > 0);
            assertNotNull(mSlamCamera);
            assertNotNull(mSlamCovariance);
            assertFalse(reconstructor.isFirstView());
            assertFalse(reconstructor.isSecondView());
            assertTrue(reconstructor.isAdditionalView());
            assertTrue(reconstructor.isAdditionalView());
            assertTrue(reconstructor.getViewCount() > 0);
            assertNotNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertSame(reconstructor.getCurrentEstimatedFundamentalMatrix(), mEstimatedFundamentalMatrix);
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertSame(reconstructor.getCurrentMetricEstimatedCamera(), mEstimatedMetricCamera3);
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertSame(reconstructor.getPreviousMetricEstimatedCamera(), mEstimatedMetricCamera2);
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(reconstructor.getCurrentEuclideanEstimatedCamera(), mEstimatedEuclideanCamera3);
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(reconstructor.getPreviousEuclideanEstimatedCamera(), mEstimatedEuclideanCamera2);
            assertNotNull(reconstructor.getActiveMetricReconstructedPoints());
            assertSame(reconstructor.getActiveMetricReconstructedPoints(), mMetricReconstructedPoints);
            assertNotNull(reconstructor.getActiveEuclideanReconstructedPoints());
            assertSame(reconstructor.getActiveEuclideanReconstructedPoints(), mEuclideanReconstructedPoints);
            assertEquals(reconstructor.getCurrentScale(), mScale2, 0.0);
            assertNotNull(reconstructor.getPreviousViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewNewlySpawnedSamples());

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

            int numReconstructedPoints = numPoints1 - start + numPoints2;

            if (mMetricReconstructedPoints.size() != numReconstructedPoints) {
                continue;
            }

            List<Point3D> metricReconstructedPoints3D = new ArrayList<>();
            List<Point3D> euclideanReconstructedPoints3D = new ArrayList<>();
            for (int i = 0; i < numReconstructedPoints; i++) {
                metricReconstructedPoints3D.add(
                        mMetricReconstructedPoints.get(i).getPoint());
                euclideanReconstructedPoints3D.add(
                        mEuclideanReconstructedPoints.get(i).getPoint());
            }

            //check that all points are in front of both cameras
            for (int i = 0; i < numReconstructedPoints; i++) {
                Point3D p = metricReconstructedPoints3D.get(i);
                Point3D pe = euclideanReconstructedPoints3D.get(i);

                assertTrue(estimatedMetricCamera1.isPointInFrontOfCamera(p));
                assertTrue(estimatedMetricCamera2.isPointInFrontOfCamera(p));

                assertTrue(estimatedEuclideanCamera1.isPointInFrontOfCamera(pe));
                assertTrue(estimatedEuclideanCamera2.isPointInFrontOfCamera(pe));
            }

            Point3D euclideanCenter1 = estimatedEuclideanCamera1.getCameraCenter();
            Point3D euclideanCenter2 = estimatedEuclideanCamera2.getCameraCenter();
            Point3D euclideanCenter3 = estimatedEuclideanCamera3.getCameraCenter();

            PinholeCameraIntrinsicParameters euclideanIntrinsic1 =
                    estimatedEuclideanCamera1.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters euclideanIntrinsic2 =
                    estimatedEuclideanCamera2.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters euclideanIntrinsic3 =
                    estimatedEuclideanCamera3.getIntrinsicParameters();

            Rotation3D euclideanRotation1 = estimatedEuclideanCamera1.getCameraRotation();
            Rotation3D euclideanRotation2 = estimatedEuclideanCamera2.getCameraRotation();
            Rotation3D euclideanRotation3 = estimatedEuclideanCamera3.getCameraRotation();

            double estimatedBaseline = euclideanCenter1.distanceTo(euclideanCenter2);
            double estimatedBaseline2 = euclideanCenter2.distanceTo(euclideanCenter3);

            //check cameras are correct
            double maxBaseline = Math.max(estimatedBaseline, baseline);
            double absoluteScaleError = RELATIVE_ERROR * maxBaseline;
            if (Math.abs(estimatedBaseline - baseline) > absoluteScaleError) {
                continue;
            }
            assertEquals(estimatedBaseline, baseline, absoluteScaleError);

            double maxBaseline2 = Math.max(estimatedBaseline2, baseline2);
            double absoluteScaleError2 = RELATIVE_ERROR * maxBaseline2;
            if (Math.abs(estimatedBaseline2 - baseline2) > absoluteScaleError2) {
                continue;
            }
            assertEquals(estimatedBaseline2, baseline2, absoluteScaleError2);

            assertTrue(center1.equals(euclideanCenter1, ABSOLUTE_ERROR));
            if (!center2.equals(euclideanCenter2, absoluteScaleError)) {
                continue;
            }
            assertTrue(center2.equals(euclideanCenter2, absoluteScaleError));
            if(!center3.equals(euclideanCenter3, absoluteScaleError2)) {
                continue;
            }
            assertTrue(center3.equals(euclideanCenter3, absoluteScaleError2));

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

            //check that points are correct (up to 5% scale error)

            //check that scale error is less than 5%
            assertTrue(Math.abs(baseline / mScale - 1.0) < RELATIVE_ERROR);
            assertTrue(Math.abs(baseline / mScale2 - 1.0) < RELATIVE_ERROR);
            MetricTransformation3D scaleTransformation
                    = new MetricTransformation3D(mScale2);

            int numValidPoints = 0;
            double scaleX, scaleY, scaleZ;
            for (int i = start; i < numPoints1; i++) {
                Point3D point = points3D1.get(i);
                Point3D euclideanPoint = euclideanReconstructedPoints3D.get(i - start);

                //check metric points
                Point3D rescaledPoint = Point3D.create();
                scaleTransformation.transform(metricReconstructedPoints3D.get(i - start),
                        rescaledPoint);

                assertTrue(euclideanPoint.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR));

                scaleX = point.getInhomX() / rescaledPoint.getInhomX();
                scaleY = point.getInhomY() / rescaledPoint.getInhomY();
                scaleZ = point.getInhomZ() / rescaledPoint.getInhomZ();

                //check that scale error is less than 5%
                if (Math.abs(scaleX - baseline / mScale2) > ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleX, baseline / mScale2, ABSOLUTE_ERROR);
                if (Math.abs(scaleY - baseline / mScale2) > ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleY, baseline / mScale2, ABSOLUTE_ERROR);
                if (Math.abs(scaleZ - baseline / mScale2) > ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleZ, baseline / mScale2, ABSOLUTE_ERROR);
                if (Math.abs(scaleX - 1.0) > RELATIVE_ERROR ||
                        Math.abs(scaleY - 1.0) > RELATIVE_ERROR ||
                        Math.abs(scaleZ - 1.0) > RELATIVE_ERROR) {
                    continue;
                }
                rescaledPoint.setInhomogeneousCoordinates(
                        rescaledPoint.getInhomX() * baseline / mScale2,
                        rescaledPoint.getInhomY() * baseline / mScale2,
                        rescaledPoint.getInhomZ() * baseline / mScale2);
                if (point.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR)) {
                    numValidPoints++;
                }

                //check euclidean points
                scaleX = point.getInhomX() / euclideanPoint.getInhomX();
                scaleY = point.getInhomY() / euclideanPoint.getInhomY();
                scaleZ = point.getInhomZ() / euclideanPoint.getInhomZ();

                //check that scale error is less than 5%
                assertEquals(scaleX, baseline / mScale2, ABSOLUTE_ERROR);
                assertEquals(scaleY, baseline / mScale2, ABSOLUTE_ERROR);
                assertEquals(scaleZ, baseline / mScale2, ABSOLUTE_ERROR);
                assertTrue(Math.abs(scaleX - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleY - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleZ - 1.0) < RELATIVE_ERROR);
            }

            for (int i = 0; i < numPoints2; i++) {
                Point3D point = points3D2.get(i);
                Point3D euclideanPoint = euclideanReconstructedPoints3D.get(i + numPoints1 - start);

                //check metric points
                Point3D rescaledPoint = Point3D.create();
                scaleTransformation.transform(metricReconstructedPoints3D.get(i + numPoints1 - start),
                        rescaledPoint);

                assertTrue(euclideanPoint.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR));

                scaleX = point.getInhomX() / rescaledPoint.getInhomX();
                scaleY = point.getInhomY() / rescaledPoint.getInhomY();
                scaleZ = point.getInhomZ() / rescaledPoint.getInhomZ();

                //check that scale error is less than 5%
                if (Math.abs(scaleX - baseline / mScale2) > ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleX, baseline / mScale2, ABSOLUTE_ERROR);
                if (Math.abs(scaleY - baseline / mScale2) > ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleY, baseline / mScale2, ABSOLUTE_ERROR);
                if (Math.abs(scaleZ - baseline / mScale2) > ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleZ, baseline / mScale2, ABSOLUTE_ERROR);
                if (Math.abs(scaleX - 1.0) > RELATIVE_ERROR ||
                        Math.abs(scaleY - 1.0) > RELATIVE_ERROR ||
                        Math.abs(scaleZ - 1.0) > RELATIVE_ERROR) {
                    continue;
                }
                rescaledPoint.setInhomogeneousCoordinates(
                        rescaledPoint.getInhomX() * baseline / mScale2,
                        rescaledPoint.getInhomY() * baseline / mScale2,
                        rescaledPoint.getInhomZ() * baseline / mScale2);
                if (point.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR)) {
                    numValidPoints++;
                }

                //check euclidean points
                scaleX = point.getInhomX() / euclideanPoint.getInhomX();
                scaleY = point.getInhomY() / euclideanPoint.getInhomY();
                scaleZ = point.getInhomZ() / euclideanPoint.getInhomZ();

                //check that scale error is less than 5%
                assertEquals(scaleX, baseline / mScale2, ABSOLUTE_ERROR);
                assertEquals(scaleY, baseline / mScale2, ABSOLUTE_ERROR);
                assertEquals(scaleZ, baseline / mScale2, ABSOLUTE_ERROR);
                assertTrue(Math.abs(scaleX - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleY - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleZ - 1.0) < RELATIVE_ERROR);
            }

            if (numValidPoints == 0) {
                continue;
            }

            double scaleRelativeError = Math.abs(baseline / mScale2 - 1.0);
            LOGGER.log(Level.INFO,
                    "Baseline relative error without noise: {0,number,0.000%}",
                    scaleRelativeError);

            //check scales
            double maxScale = Math.max(mScale, mScale2);
            scaleRelativeError = RELATIVE_ERROR * maxScale;
            if (Math.abs(mScale - mScale2) > scaleRelativeError) {
                continue;
            }
            assertEquals(mScale, mScale2, scaleRelativeError);

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

            SlamSparseReconstructorConfiguration configuration =
                    new SlamSparseReconstructorConfiguration();
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

            SlamCalibrator calibrator = createFinishedCalibrator(
                    accelerationOffsetX, accelerationOffsetY,
                    accelerationOffsetZ, angularOffsetX, angularOffsetY,
                    angularOffsetZ, noiseRandomizer);
            SlamCalibrationData calibrationData
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

            double alphaEuler1 = 0.0;
            double betaEuler1 = 0.0;
            double gammaEuler1 = 0.0;
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

            MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1,
                    betaEuler1, gammaEuler1);
            MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2,
                    betaEuler2, gammaEuler2);
            AxisRotation3D axisRotation2 = new AxisRotation3D(rotation2);
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
            Quaternion diffRotation2 = new Quaternion(angularSpeed2X,
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
            diffRotation2 = new Quaternion(angularSpeed3X, angularSpeed3Y,
                    angularSpeed3Z);

            //number of samples (50 samples * 0.02 s/sample = 1 second), starting from
            //previously sampled rotation
            MatrixRotation3D rotation3b = new MatrixRotation3D(rotation2b);
            MatrixRotation3D rotation3c = new MatrixRotation3D(rotation2c);
            for (int s = 0; s < N_SENSOR_SAMPLES; s++) {
                rotation3b.combine(diffRotation);
                rotation3c.combine(diffRotation2);
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

            Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);
            Point3D center3 = new InhomogeneousPoint3D(
                    center2.getInhomX() + cameraSeparation2,
                    center2.getInhomY() + cameraSeparation2,
                    center2.getInhomZ() + cameraSeparation2);

            double baseline = center1.distanceTo(center2);
            double baseline2 = center2.distanceTo(center3);

            final double accelerationX, accelerationY, accelerationZ;
            final double accelerationX2, accelerationY2, accelerationZ2;

            //s = 0.5*a*t^2 --> a = 2*s/t^2
            //assuming t = 1 second (50 samples * 0.02 s/sample = 1 second)
            accelerationX = accelerationY = accelerationZ
                    = 2 * cameraSeparation;
            accelerationX2 = accelerationY2 = accelerationZ2
                    = 2 * cameraSeparation2;

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

            final int numPoints1 = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);
            final int numPoints2 = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);
            final int start = randomizer.nextInt(0,
                    numPoints1 - MIN_TRACKED_POINTS);

            InhomogeneousPoint3D point3D;
            List<InhomogeneousPoint3D> points3D1 = new ArrayList<>();
            Point2D projectedPoint1, projectedPoint2, projectedPoint3;
            final List<Point2D> projectedPoints1 = new ArrayList<>();
            final List<Point2D> projectedPoints2 = new ArrayList<>();
            final List<Point2D> projectedPoints3 = new ArrayList<>();
            boolean front1, front2;
            for (int i = 0; i < numPoints1; i++) {
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
                        fail("max tries reached");
                    }
                    numTry++;
                } while(!front1 || !front2);
                points3D1.add(point3D);

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

            List<InhomogeneousPoint3D> points3D2 = new ArrayList<>();
            Point2D projectedPoint2b, projectedPoint3b;
            final List<Point2D> projectedPoints2b = new ArrayList<>();
            final List<Point2D> projectedPoints3b = new ArrayList<>();
            for (int i = 0; i < numPoints2; i++) {
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

                    front2 = camera2.isPointInFrontOfCamera(point3D);
                    if (numTry > MAX_TRIES) {
                        fail("max tries reached");
                    }
                    numTry++;
                } while(!front2);
                points3D2.add(point3D);

                //check that 3D point is in front of both cameras
                //noinspection all
                assertTrue(front2);

                projectedPoint2b = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2b);
                projectedPoints2b.add(projectedPoint2b);

                projectedPoint3b = new InhomogeneousPoint2D();
                camera3.project(point3D, projectedPoint3b);
                projectedPoints3b.add(projectedPoint3b);
            }

            final GaussianRandomizer accelerationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);

            SlamSparseReconstructorListener listener =
                    new SlamSparseReconstructorListener() {
                        @Override
                        public void onSlamDataAvailable(SlamSparseReconstructor reconstructor,
                                                        double positionX, double positionY, double positionZ,
                                                        double velocityX, double velocityY, double velocityZ,
                                                        double accelerationX, double accelerationY, double accelerationZ,
                                                        double quaternionA, double quaternionB, double quaternionC, double quaternionD,
                                                        double angularSpeedX, double angularSpeedY, double angularSpeedZ,
                                                        Matrix covariance) {
                            mSlamDataAvailable++;
                            mSlamCovariance = covariance;
                        }

                        @Override
                        public void onSlamCameraEstimated(SlamSparseReconstructor reconstructor,
                                                          PinholeCamera camera) {
                            mSlamCameraEstimated++;
                            mSlamCamera = camera;
                        }

                        @Override
                        public boolean hasMoreViewsAvailable(SlamSparseReconstructor reconstructor) {
                            return mViewCount < 3;
                        }

                        @Override
                        public void onRequestSamples(SlamSparseReconstructor reconstructor,
                                                     int previousViewId, int currentViewId,
                                                     List<Sample2D> previousViewTrackedSamples,
                                                     List<Sample2D> currentViewTrackedSamples,
                                                     List<Sample2D> currentViewNewlySpawnedSamples) {

                            previousViewTrackedSamples.clear();
                            currentViewTrackedSamples.clear();
                            currentViewNewlySpawnedSamples.clear();

                            Sample2D sample;
                            if (mViewCount == 0) {
                                //first view
                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints1.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewTrackedSamples.add(sample);
                                }
                            } else if (mEstimatedFundamentalMatrix == null) {
                                //second view
                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints1.get(i));
                                    sample.setViewId(previousViewId);
                                    previousViewTrackedSamples.add(sample);
                                }

                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints2.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewTrackedSamples.add(sample);
                                }

                                //spawned samples
                                for (int i = 0; i < numPoints2; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints2b.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewNewlySpawnedSamples.add(sample);
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
                                    mTimestamp += DELTA_NANOS;
                                }

                            } else {
                                //third view
                                for (int i = start; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints2.get(i));
                                    sample.setViewId(previousViewId);
                                    previousViewTrackedSamples.add(sample);
                                }

                                for (int i = 0; i < numPoints2; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints2b.get(i));
                                    sample.setViewId(previousViewId);
                                    previousViewTrackedSamples.add(sample);
                                }


                                for (int i = start; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints3.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewTrackedSamples.add(sample);
                                }

                                for (int i = 0; i < numPoints2; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints3b.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewTrackedSamples.add(sample);
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

                                    accelerationWithNoiseX = (float)accelerationX2 +
                                            noiseAccelerationX;
                                    accelerationWithNoiseY = (float)accelerationY2 +
                                            noiseAccelerationY;
                                    accelerationWithNoiseZ = (float)accelerationZ2 +
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
                                    mTimestamp += DELTA_NANOS;
                                }
                            }
                        }

                        @Override
                        public void onSamplesAccepted(SlamSparseReconstructor reconstructor, int viewId,
                                                      List<Sample2D> previousViewTrackedSamples,
                                                      List<Sample2D> currentViewTrackedSamples) {
                            mViewCount++;
                        }

                        @Override
                        public void onSamplesRejected(SlamSparseReconstructor reconstructor, int viewId,
                                                      List<Sample2D> previousViewTrackedSamples,
                                                      List<Sample2D> currentViewTrackedSamples) {
                            mViewCount++;
                        }

                        @Override
                        public void onRequestMatches(SlamSparseReconstructor reconstructor,
                                                     List<Sample2D> allPreviousViewSamples,
                                                     List<Sample2D> previousViewTrackedSamples,
                                                     List<Sample2D> currentViewTrackedSamples,
                                                     int previousViewId, int currentViewId,
                                                     List<MatchedSamples> matches) {
                            matches.clear();

                            int numCameras = 0;
                            if (mEstimatedMetricCamera1 != null &&
                                    (mEstimatedMetricCamera1.getViewId() == previousViewId ||
                                            mEstimatedMetricCamera1.getViewId() == currentViewId)) {
                                numCameras++;
                            }
                            if (mEstimatedMetricCamera2 != null &&
                                    (mEstimatedMetricCamera2.getViewId() == previousViewId ||
                                            mEstimatedMetricCamera2.getViewId() == currentViewId)) {
                                numCameras++;
                            }

                            EstimatedCamera[] estimatedCameras = null;
                            if (numCameras > 0) {
                                estimatedCameras = new EstimatedCamera[numCameras];


                                int pos = 0;
                                if (mEstimatedMetricCamera1 != null &&
                                        (mEstimatedMetricCamera1.getViewId() == previousViewId ||
                                                mEstimatedMetricCamera1.getViewId() == currentViewId)) {
                                    estimatedCameras[pos] = mEstimatedMetricCamera1;
                                    pos++;
                                }
                                if (mEstimatedMetricCamera2 != null &&
                                        (mEstimatedMetricCamera2.getViewId() == previousViewId ||
                                                mEstimatedMetricCamera2.getViewId() == currentViewId)) {
                                    estimatedCameras[pos] = mEstimatedMetricCamera2;
                                }
                            }

                            List<Point2D> allPreviousPoints = new ArrayList<>();
                            for (Sample2D sample : allPreviousViewSamples) {
                                allPreviousPoints.add(sample.getPoint());
                            }
                            KDTree2D tree = new KDTree2D(allPreviousPoints);

                            //search previous view tracked samples within tree
                            int numTrackedSamples = previousViewTrackedSamples.size();
                            Point2D point, nearestPoint;
                            int nearestIndex;
                            MatchedSamples match;
                            for (int i = 0; i < numTrackedSamples; i++) {
                                Sample2D previousSample = previousViewTrackedSamples.get(i);
                                point = previousSample.getPoint();
                                nearestIndex = tree.nearestIndex(point);
                                nearestPoint = allPreviousPoints.get(nearestIndex);
                                Sample2D nearestSample = allPreviousViewSamples.get(nearestIndex);

                                if (point.distanceTo(nearestPoint) > NEAREST_THRESHOLD) {
                                    continue;
                                }

                                Sample2D currentSample = currentViewTrackedSamples.get(i);

                                match = new MatchedSamples();
                                match.setSamples(new Sample2D[]{
                                        previousSample, currentSample
                                });
                                match.setViewIds(new int[]{previousViewId, currentViewId});

                                match.setReconstructedPoint(nearestSample.getReconstructedPoint());

                                if (estimatedCameras != null) {
                                    match.setCameras(estimatedCameras);
                                }

                                matches.add(match);
                            }
                        }

                        @Override
                        public void onFundamentalMatrixEstimated(SlamSparseReconstructor reconstructor,
                                                                 EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                            if (mEstimatedFundamentalMatrix == null) {
                                mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                            } else if (mEstimatedFundamentalMatrix2 == null) {
                                mEstimatedFundamentalMatrix2 = estimatedFundamentalMatrix;
                            }
                        }

                        @Override
                        public void onMetricCameraEstimated(SlamSparseReconstructor reconstructor,
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
                        public void onMetricReconstructedPointsEstimated(SlamSparseReconstructor reconstructor,
                                                                         List<MatchedSamples> matches,
                                                                         List<ReconstructedPoint3D> points) {
                            mMetricReconstructedPoints = points;
                        }

                        @Override
                        public void onEuclideanCameraEstimated(SlamSparseReconstructor reconstructor,
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
                                SlamSparseReconstructor reconstructor,
                                double scale, List<ReconstructedPoint3D> points) {
                            if (mEuclideanReconstructedPoints == null) {
                                mScale = scale;
                            } else {
                                mScale2 = scale;
                            }

                            mEuclideanReconstructedPoints = points;
                        }

                        @Override
                        public void onStart(SlamSparseReconstructor reconstructor) {
                            mStarted = true;
                        }

                        @Override
                        public void onFinish(SlamSparseReconstructor reconstructor) {
                            mFinished = true;
                        }

                        @Override
                        public void onCancel(SlamSparseReconstructor reconstructor) {
                            mCancelled = true;
                        }

                        @Override
                        public void onFail(SlamSparseReconstructor reconstructor) {
                            mFailed = true;
                        }
                    };

            SlamSparseReconstructor reconstructor =
                    new SlamSparseReconstructor(configuration, listener);

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
            assertTrue(mSlamDataAvailable > 0);
            assertTrue(mSlamCameraEstimated > 0);
            assertNotNull(mSlamCamera);
            assertNotNull(mSlamCovariance);
            assertFalse(reconstructor.isFirstView());
            assertFalse(reconstructor.isSecondView());
            assertTrue(reconstructor.isAdditionalView());
            assertTrue(reconstructor.isAdditionalView());
            assertTrue(reconstructor.getViewCount() > 0);
            assertNotNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertSame(reconstructor.getCurrentEstimatedFundamentalMatrix(), mEstimatedFundamentalMatrix);
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertSame(reconstructor.getCurrentMetricEstimatedCamera(), mEstimatedMetricCamera3);
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertSame(reconstructor.getPreviousMetricEstimatedCamera(), mEstimatedMetricCamera2);
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(reconstructor.getCurrentEuclideanEstimatedCamera(), mEstimatedEuclideanCamera3);
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(reconstructor.getPreviousEuclideanEstimatedCamera(), mEstimatedEuclideanCamera2);
            assertNotNull(reconstructor.getActiveMetricReconstructedPoints());
            assertSame(reconstructor.getActiveMetricReconstructedPoints(), mMetricReconstructedPoints);
            assertNotNull(reconstructor.getActiveEuclideanReconstructedPoints());
            assertSame(reconstructor.getActiveEuclideanReconstructedPoints(), mEuclideanReconstructedPoints);
            assertEquals(reconstructor.getCurrentScale(), mScale2, 0.0);
            assertNotNull(reconstructor.getPreviousViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewNewlySpawnedSamples());

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

            int numReconstructedPoints = numPoints1 - start + numPoints2;

            if (mMetricReconstructedPoints.size() != numReconstructedPoints) {
                continue;
            }

            List<Point3D> metricReconstructedPoints3D = new ArrayList<>();
            List<Point3D> euclideanReconstructedPoints3D = new ArrayList<>();
            for (int i = 0; i < numReconstructedPoints; i++) {
                metricReconstructedPoints3D.add(
                        mMetricReconstructedPoints.get(i).getPoint());
                euclideanReconstructedPoints3D.add(
                        mEuclideanReconstructedPoints.get(i).getPoint());
            }

            //check that all points are in front of both cameras
            for (int i = 0; i < numReconstructedPoints; i++) {
                Point3D p = metricReconstructedPoints3D.get(i);
                Point3D pe = euclideanReconstructedPoints3D.get(i);

                assertTrue(estimatedMetricCamera1.isPointInFrontOfCamera(p));
                assertTrue(estimatedMetricCamera2.isPointInFrontOfCamera(p));

                assertTrue(estimatedEuclideanCamera1.isPointInFrontOfCamera(pe));
                assertTrue(estimatedEuclideanCamera2.isPointInFrontOfCamera(pe));
            }

            Point3D euclideanCenter1 = estimatedEuclideanCamera1.getCameraCenter();
            Point3D euclideanCenter2 = estimatedEuclideanCamera2.getCameraCenter();
            Point3D euclideanCenter3 = estimatedEuclideanCamera3.getCameraCenter();

            PinholeCameraIntrinsicParameters euclideanIntrinsic1 =
                    estimatedEuclideanCamera1.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters euclideanIntrinsic2 =
                    estimatedEuclideanCamera2.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters euclideanIntrinsic3 =
                    estimatedEuclideanCamera3.getIntrinsicParameters();

            Rotation3D euclideanRotation1 = estimatedEuclideanCamera1.getCameraRotation();
            Rotation3D euclideanRotation2 = estimatedEuclideanCamera2.getCameraRotation();
            Rotation3D euclideanRotation3 = estimatedEuclideanCamera3.getCameraRotation();

            double estimatedBaseline = euclideanCenter1.distanceTo(euclideanCenter2);
            double estimatedBaseline2 = euclideanCenter2.distanceTo(euclideanCenter3);

            //check cameras are correct
            double maxBaseline = Math.max(estimatedBaseline, baseline);
            double absoluteScaleError = RELATIVE_ERROR * maxBaseline;
            if (Math.abs(estimatedBaseline - baseline) > absoluteScaleError) {
                continue;
            }
            assertEquals(estimatedBaseline, baseline, absoluteScaleError);

            double maxBaseline2 = Math.max(estimatedBaseline2, baseline2);
            double absoluteScaleError2 = RELATIVE_ERROR * maxBaseline2;
            if (Math.abs(estimatedBaseline2 - baseline2) > absoluteScaleError2) {
                continue;
            }
            assertEquals(estimatedBaseline2, baseline2, absoluteScaleError2);

            assertTrue(center1.equals(euclideanCenter1, ABSOLUTE_ERROR));
            if (!center2.equals(euclideanCenter2, absoluteScaleError)) {
                continue;
            }
            assertTrue(center2.equals(euclideanCenter2, absoluteScaleError));
            if(!center3.equals(euclideanCenter3, absoluteScaleError2)) {
                continue;
            }
            assertTrue(center3.equals(euclideanCenter3, absoluteScaleError2));

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

            //check that scale error is less than 5%
            assertTrue(Math.abs(baseline / mScale - 1.0) < RELATIVE_ERROR);
            assertTrue(Math.abs(baseline / mScale2 - 1.0) < RELATIVE_ERROR);
            MetricTransformation3D scaleTransformation
                    = new MetricTransformation3D(mScale2);

            int numValidPoints = 0;
            double scaleX, scaleY, scaleZ;
            for (int i = start; i < numPoints1; i++) {
                Point3D point = points3D1.get(i);
                Point3D euclideanPoint = euclideanReconstructedPoints3D.get(i - start);

                //check metric points
                Point3D rescaledPoint = Point3D.create();
                scaleTransformation.transform(metricReconstructedPoints3D.get(i - start),
                        rescaledPoint);

                assertTrue(euclideanPoint.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR));

                scaleX = point.getInhomX() / rescaledPoint.getInhomX();
                scaleY = point.getInhomY() / rescaledPoint.getInhomY();
                scaleZ = point.getInhomZ() / rescaledPoint.getInhomZ();

                //check that scale error is less than 5%
                assertEquals(scaleX, baseline / mScale2, LARGE_ABSOLUTE_ERROR);
                assertEquals(scaleY, baseline / mScale2, LARGE_ABSOLUTE_ERROR);
                assertEquals(scaleZ, baseline / mScale2, LARGE_ABSOLUTE_ERROR);
                if (Math.abs(scaleX - 1.0) > RELATIVE_ERROR ||
                        Math.abs(scaleY - 1.0) > RELATIVE_ERROR ||
                        Math.abs(scaleZ - 1.0) > RELATIVE_ERROR) {
                    continue;
                }
                rescaledPoint.setInhomogeneousCoordinates(
                        rescaledPoint.getInhomX() * baseline / mScale2,
                        rescaledPoint.getInhomY() * baseline / mScale2,
                        rescaledPoint.getInhomZ() * baseline / mScale2);
                if (point.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR)) {
                    numValidPoints++;
                }

                //check euclidean points
                scaleX = point.getInhomX() / euclideanPoint.getInhomX();
                scaleY = point.getInhomY() / euclideanPoint.getInhomY();
                scaleZ = point.getInhomZ() / euclideanPoint.getInhomZ();

                //check that scale error is less than 5%
                if (Math.abs(scaleX - baseline / mScale2) > ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleX, baseline / mScale2, ABSOLUTE_ERROR);
                if (Math.abs(scaleY - baseline / mScale2) > ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleY, baseline / mScale2, ABSOLUTE_ERROR);
                if (Math.abs(scaleZ - baseline / mScale2) > ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleZ, baseline / mScale2, ABSOLUTE_ERROR);
                assertTrue(Math.abs(scaleX - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleY - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleZ - 1.0) < RELATIVE_ERROR);
            }

            for (int i = 0; i < numPoints2; i++) {
                Point3D point = points3D2.get(i);
                Point3D euclideanPoint = euclideanReconstructedPoints3D.get(i + numPoints1 - start);

                //check metric points
                Point3D rescaledPoint = Point3D.create();
                scaleTransformation.transform(metricReconstructedPoints3D.get(i + numPoints1 - start),
                        rescaledPoint);

                assertTrue(euclideanPoint.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR));

                scaleX = point.getInhomX() / rescaledPoint.getInhomX();
                scaleY = point.getInhomY() / rescaledPoint.getInhomY();
                scaleZ = point.getInhomZ() / rescaledPoint.getInhomZ();

                //check that scale error is less than 5%
                assertEquals(scaleX, baseline / mScale2, LARGE_ABSOLUTE_ERROR);
                assertEquals(scaleY, baseline / mScale2, LARGE_ABSOLUTE_ERROR);
                assertEquals(scaleZ, baseline / mScale2, LARGE_ABSOLUTE_ERROR);
                if (Math.abs(scaleX - 1.0) > RELATIVE_ERROR ||
                        Math.abs(scaleY - 1.0) > RELATIVE_ERROR ||
                        Math.abs(scaleZ - 1.0) > RELATIVE_ERROR) {
                    continue;
                }
                rescaledPoint.setInhomogeneousCoordinates(
                        rescaledPoint.getInhomX() * baseline / mScale2,
                        rescaledPoint.getInhomY() * baseline / mScale2,
                        rescaledPoint.getInhomZ() * baseline / mScale2);
                if (point.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR)) {
                    numValidPoints++;
                }

                //check euclidean points
                scaleX = point.getInhomX() / euclideanPoint.getInhomX();
                scaleY = point.getInhomY() / euclideanPoint.getInhomY();
                scaleZ = point.getInhomZ() / euclideanPoint.getInhomZ();

                //check that scale error is less than 5%
                if (Math.abs(scaleX - baseline / mScale2) > ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleX, baseline / mScale2, ABSOLUTE_ERROR);
                if (Math.abs(scaleY - baseline / mScale2) > ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleY, baseline / mScale2, ABSOLUTE_ERROR);
                if (Math.abs(scaleZ - baseline / mScale2) > ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleZ, baseline / mScale2, ABSOLUTE_ERROR);
                assertTrue(Math.abs(scaleX - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleY - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleZ - 1.0) < RELATIVE_ERROR);
            }

            if (numValidPoints == 0) {
                continue;
            }

            double scaleRelativeError = Math.abs(baseline / mScale2 - 1.0);
            LOGGER.log(Level.INFO,
                    "Baseline relative error without noise: {0,number,0.000%}",
                    scaleRelativeError);

            //check scales
            double maxScale = Math.max(mScale, mScale2);
            scaleRelativeError = RELATIVE_ERROR * maxScale;
            if (Math.abs(mScale - mScale2) > scaleRelativeError) {
                continue;
            }
            assertEquals(mScale, mScale2, scaleRelativeError);

            numValid++;

            if (numValid > 0) {
                break;
            }
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testGeneralPointsEssentialWithConstantAccelerationAndRotationWithoutNoiseFourViews()
            throws InvalidPairOfCamerasException, AlgebraException,
            CameraException,
            com.irurueta.geometry.estimators.NotReadyException,
            com.irurueta.geometry.NotAvailableException, RotationException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);

            SlamSparseReconstructorConfiguration configuration =
                    new SlamSparseReconstructorConfiguration();
            configuration.setInitialCamerasEstimatorMethod(
                    InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);

            float accelerationOffsetX = 0.0f;
            float accelerationOffsetY = 0.0f;
            float accelerationOffsetZ = 0.0f;

            float angularOffsetX = 0.0f;
            float angularOffsetY = 0.0f;
            float angularOffsetZ = 0.0f;

            SlamCalibrator calibrator = createFinishedCalibrator(
                    accelerationOffsetX, accelerationOffsetY,
                    accelerationOffsetZ, angularOffsetX, angularOffsetY,
                    angularOffsetZ, noiseRandomizer);
            SlamCalibrationData calibrationData
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

            double alphaEuler1 = 0.0;
            double betaEuler1 = 0.0;
            double gammaEuler1 = 0.0;
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
            double alphaEuler4 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler4 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler4 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1,
                    betaEuler1, gammaEuler1);
            MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2,
                    betaEuler2, gammaEuler2);
            AxisRotation3D axisRotation2 = new AxisRotation3D(rotation2);
            MatrixRotation3D rotation3 = new MatrixRotation3D(alphaEuler3,
                    betaEuler3, gammaEuler3);
            MatrixRotation3D rotation4 = new MatrixRotation3D(alphaEuler4,
                    betaEuler4, gammaEuler4);

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
            Quaternion diffRotation2 = new Quaternion(angularSpeed2X,
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
            diffRotation2 = new Quaternion(angularSpeed3X, angularSpeed3Y,
                    angularSpeed3Z);

            //number of samples (50 samples * 0.02 s/sample = 1 second), starting from
            //previously sampled rotation
            MatrixRotation3D rotation3b = new MatrixRotation3D(rotation2b);
            MatrixRotation3D rotation3c = new MatrixRotation3D(rotation2c);
            for (int s = 0; s < N_SENSOR_SAMPLES; s++) {
                rotation3b.combine(diffRotation);
                rotation3c.combine(diffRotation2);
            }

            //check that rotations created by composing sensor samples are equal
            //to the original one
            assertTrue(rotation3.equals(rotation3b, ABSOLUTE_ERROR));
            assertTrue(rotation3.equals(rotation3c, ABSOLUTE_ERROR));

            accumDiffRotation = rotation3.inverseRotationAndReturnNew().
                    combineAndReturnNew(rotation4).toAxisRotation();
            double axis4X = accumDiffRotation.getAxisX();
            double axis4Y = accumDiffRotation.getAxisY();
            double axis4Z = accumDiffRotation.getAxisZ();
            double angle4 = accumDiffRotation.getRotationAngle();

            diffRotation = new AxisRotation3D(axis4X, axis4Y, axis4Z, angle4 / N_SENSOR_SAMPLES);
            diffQuaternion = new Quaternion(diffRotation);

            //angular speeds (roll, pitch, yaw) on x, y, z axes
            angularSpeeds = diffQuaternion.toEulerAngles();
            final double angularSpeed4X = angularSpeeds[0];
            final double angularSpeed4Y = angularSpeeds[1];
            final double angularSpeed4Z = angularSpeeds[2];
            diffRotation2 = new Quaternion(angularSpeed4X, angularSpeed4Y,
                    angularSpeed4Z);

            //number of samples (50 samples * 0.02 s/sample = 1 second), starting from
            //previously sampled rotation
            MatrixRotation3D rotation4b = new MatrixRotation3D(rotation3b);
            MatrixRotation3D rotation4c = new MatrixRotation3D(rotation3c);
            for (int s = 0; s < N_SENSOR_SAMPLES; s++) {
                rotation4b.combine(diffRotation);
                rotation4c.combine(diffRotation2);
            }

            //check that rotations created by composing sensor samples are equal
            //to the original one
            assertTrue(rotation4.equals(rotation4b, ABSOLUTE_ERROR));
            assertTrue(rotation4.equals(rotation4c, ABSOLUTE_ERROR));

            double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);
            double cameraSeparation2 = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);
            double cameraSeparation3 = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);

            Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);
            Point3D center3 = new InhomogeneousPoint3D(
                    center2.getInhomX() + cameraSeparation2,
                    center2.getInhomY() + cameraSeparation2,
                    center2.getInhomZ() + cameraSeparation2);
            Point3D center4 = new InhomogeneousPoint3D(
                    center3.getInhomX() + cameraSeparation3,
                    center3.getInhomY() + cameraSeparation3,
                    center3.getInhomZ() + cameraSeparation3);

            double baseline = center1.distanceTo(center2);
            double baseline2 = center2.distanceTo(center3);
            double baseline3 = center3.distanceTo(center4);

            final double accelerationX, accelerationY, accelerationZ;
            final double accelerationX2, accelerationY2, accelerationZ2;
            final double accelerationX3, accelerationY3, accelerationZ3;

            //s = 0.5*a*t^2 --> a = 2*s/t^2
            //assuming t = 1 second (50 samples * 0.02 s/sample = 1 second)
            accelerationX = accelerationY = accelerationZ
                    = 2 * cameraSeparation;
            accelerationX2 = accelerationY2 = accelerationZ2
                    = 2 * cameraSeparation2;
            accelerationX3 = accelerationY3 = accelerationZ3
                    = 2 * cameraSeparation3;

            PinholeCamera camera1 = new PinholeCamera(intrinsic, rotation1,
                    center1);
            PinholeCamera camera2 = new PinholeCamera(intrinsic, rotation2,
                    center2);
            PinholeCamera camera3 = new PinholeCamera(intrinsic, rotation3,
                    center3);
            PinholeCamera camera4 = new PinholeCamera(intrinsic, rotation4,
                    center4);

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

            final int numPoints1 = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);
            final int numPoints2 = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);
            final int numPoints3 = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);
            final int numPoints4 = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);
            final int start1 = randomizer.nextInt(0,
                    numPoints1 - MIN_TRACKED_POINTS);
            final int start2 = randomizer.nextInt(0, numPoints2 - MIN_TRACKED_POINTS);

            InhomogeneousPoint3D point3D;
            Point2D projectedPoint1, projectedPoint2, projectedPoint3;
            final List<Point2D> projectedPoints1 = new ArrayList<>();
            final List<Point2D> projectedPoints2 = new ArrayList<>();
            final List<Point2D> projectedPoints3 = new ArrayList<>();
            boolean front1, front2;
            for (int i = 0; i < numPoints1; i++) {
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
                        fail("max tries reached");
                    }
                    numTry++;
                } while(!front1 || !front2);

                //check that 3D point is in front of both cameras
                //noinspection all
                assertTrue(front1);
                //noinspection all
                assertTrue(front2);

                //project 3D point into first pair
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

            List<InhomogeneousPoint3D> points3D2 = new ArrayList<>();
            Point2D projectedPoint2b, projectedPoint3b, projectedPoint4;
            final List<Point2D> projectedPoints2b = new ArrayList<>();
            final List<Point2D> projectedPoints3b = new ArrayList<>();
            final List<Point2D> projectedPoints4 = new ArrayList<>();
            for (int i = 0; i < numPoints2; i++) {
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

                    front2 = camera2.isPointInFrontOfCamera(point3D);
                    if (numTry > MAX_TRIES) {
                        fail("max tries reached");
                    }
                    numTry++;
                } while(!front2);
                points3D2.add(point3D);

                //check that 3D point is in front of 2nd camera
                //noinspection all
                assertTrue(front2);

                //project 3D point into second pair
                projectedPoint2b = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2b);
                projectedPoints2b.add(projectedPoint2b);

                projectedPoint3b = new InhomogeneousPoint2D();
                camera3.project(point3D, projectedPoint3b);
                projectedPoints3b.add(projectedPoint3b);

                projectedPoint4 = new InhomogeneousPoint2D();
                camera4.project(point3D, projectedPoint4);
                projectedPoints4.add(projectedPoint4);
            }

            List<InhomogeneousPoint3D> points3D3 = new ArrayList<>();
            Point2D projectedPoint3c, projectedPoint4b;
            final List<Point2D> projectedPoints3c = new ArrayList<>();
            final List<Point2D> projectedPoints4b = new ArrayList<>();
            for (int i = 0; i < numPoints3; i++) {
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

                    front2 = camera2.isPointInFrontOfCamera(point3D);
                    if (numTry > MAX_TRIES) {
                        fail("max tries reached");
                    }
                    numTry++;
                } while(!front2);
                points3D3.add(point3D);

                //check that 3D point is in front of 2nd camera
                //noinspection all
                assertTrue(front2);

                //project 3D point into second pair
                projectedPoint3c = new InhomogeneousPoint2D();
                camera3.project(point3D, projectedPoint3c);
                projectedPoints3c.add(projectedPoint3c);

                projectedPoint4b = new InhomogeneousPoint2D();
                camera4.project(point3D, projectedPoint4b);
                projectedPoints4b.add(projectedPoint4b);
            }

            Point2D projectedPoint4c;
            final List<Point2D> projectedPoints4c = new ArrayList<>();
            for (int i = 0; i < numPoints4; i++) {
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

                    front2 = camera2.isPointInFrontOfCamera(point3D);
                    if (numTry > MAX_TRIES) {
                        fail("max tries reached");
                    }
                    numTry++;
                } while(!front2);

                //check that 3D point is in front of 2nd camera
                //noinspection all
                assertTrue(front2);

                //project 3D point into second pair
                projectedPoint4c = new InhomogeneousPoint2D();
                camera4.project(point3D, projectedPoint4c);
                projectedPoints4c.add(projectedPoint4c);
            }

            SlamSparseReconstructorListener listener =
                    new SlamSparseReconstructorListener() {
                        @Override
                        public void onSlamDataAvailable(SlamSparseReconstructor reconstructor,
                                                        double positionX, double positionY, double positionZ,
                                                        double velocityX, double velocityY, double velocityZ,
                                                        double accelerationX, double accelerationY, double accelerationZ,
                                                        double quaternionA, double quaternionB, double quaternionC, double quaternionD,
                                                        double angularSpeedX, double angularSpeedY, double angularSpeedZ,
                                                        Matrix covariance) {
                            mSlamDataAvailable++;
                            mSlamCovariance = covariance;
                        }

                        @Override
                        public void onSlamCameraEstimated(SlamSparseReconstructor reconstructor,
                                                          PinholeCamera camera) {
                            mSlamCameraEstimated++;
                            mSlamCamera = camera;
                        }

                        @Override
                        public boolean hasMoreViewsAvailable(SlamSparseReconstructor reconstructor) {
                            return mViewCount < 4;
                        }

                        @Override
                        public void onRequestSamples(SlamSparseReconstructor reconstructor,
                                                     int previousViewId, int currentViewId,
                                                     List<Sample2D> previousViewTrackedSamples,
                                                     List<Sample2D> currentViewTrackedSamples,
                                                     List<Sample2D> currentViewNewlySpawnedSamples) {

                            previousViewTrackedSamples.clear();
                            currentViewTrackedSamples.clear();
                            currentViewNewlySpawnedSamples.clear();

                            Sample2D sample;
                            if (mViewCount == 0) {
                                //first view
                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints1.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewTrackedSamples.add(sample);
                                }
                            } else if (mViewCount == 1) {
                                //second view
                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints1.get(i));
                                    sample.setViewId(previousViewId);
                                    previousViewTrackedSamples.add(sample);
                                }

                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints2.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewTrackedSamples.add(sample);
                                }

                                //spawned samples
                                for (int i = 0; i < numPoints2; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints2b.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewNewlySpawnedSamples.add(sample);
                                }

                                //assume the following accelerator and gyroscope samples
                                //are obtained during a period of 1 second between 1st
                                //and 2nd view (50 samples * 0.02 s/sample = 1 second)
                                mTimestamp = 0;
                                for (int s = 0; s < N_SENSOR_SAMPLES; s++) {
                                    reconstructor.updateAccelerometerSample(mTimestamp,
                                            (float) accelerationX, (float) accelerationY,
                                            (float) accelerationZ);
                                    reconstructor.updateGyroscopeSample(mTimestamp,
                                            (float) angularSpeed2X, (float) angularSpeed2Y,
                                            (float) angularSpeed2Z);
                                    mTimestamp += DELTA_NANOS;
                                }

                            } else if (mViewCount == 2){
                                //third view
                                for (int i = start1; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints2.get(i));
                                    sample.setViewId(previousViewId);
                                    previousViewTrackedSamples.add(sample);
                                }

                                for (int i = 0; i < numPoints2; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints2b.get(i));
                                    sample.setViewId(previousViewId);
                                    previousViewTrackedSamples.add(sample);
                                }

                                for (int i = start1; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints3.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewTrackedSamples.add(sample);
                                }

                                for (int i = 0; i < numPoints2; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints3b.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewTrackedSamples.add(sample);
                                }

                                //spawned samples
                                for (int i = 0; i < numPoints3; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints3c.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewNewlySpawnedSamples.add(sample);
                                }

                                //assume the following accelerator and gyroscope samples
                                //are obtained during a period of 1 second between 2nd
                                //and 3rd view (50 samples * 0.02 s/sample = 1 second)
                                for (int s = 0; s < N_SENSOR_SAMPLES; s++) {
                                    reconstructor.updateAccelerometerSample(mTimestamp,
                                            (float) accelerationX2, (float) accelerationY2,
                                            (float) accelerationZ2);
                                    reconstructor.updateGyroscopeSample(mTimestamp,
                                            (float) angularSpeed3X, (float) angularSpeed3Y,
                                            (float) angularSpeed3Z);
                                    mTimestamp += DELTA_NANOS;
                                }

                            } else {
                                //4th view
                                for (int i = start2; i < numPoints2; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints3b.get(i));
                                    sample.setViewId(previousViewId);
                                    previousViewTrackedSamples.add(sample);
                                }

                                for (int i = 0; i < numPoints3; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints3c.get(i));
                                    sample.setViewId(previousViewId);
                                    previousViewTrackedSamples.add(sample);
                                }

                                for (int i = start2; i < numPoints2; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints4.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewTrackedSamples.add(sample);
                                }

                                for (int i = 0; i < numPoints3; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints4b.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewTrackedSamples.add(sample);
                                }

                                //spawned samples
                                for (int i = 0; i < numPoints4; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints4c.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewNewlySpawnedSamples.add(sample);
                                }

                                //assume the following accelerator and gyroscope samples
                                //are obtained during a period of 1 second between 2nd
                                //and 3rd view (50 samples * 0.02 s/sample = 1 second)
                                for (int s = 0; s < N_SENSOR_SAMPLES; s++) {
                                    reconstructor.updateAccelerometerSample(mTimestamp,
                                            (float) accelerationX3, (float) accelerationY3,
                                            (float) accelerationZ3);
                                    reconstructor.updateGyroscopeSample(mTimestamp,
                                            (float) angularSpeed4X, (float) angularSpeed4Y,
                                            (float) angularSpeed4Z);
                                    mTimestamp += DELTA_NANOS;
                                }
                            }
                        }

                        @Override
                        public void onSamplesAccepted(SlamSparseReconstructor reconstructor, int viewId,
                                                      List<Sample2D> previousViewTrackedSamples,
                                                      List<Sample2D> currentViewTrackedSamples) {
                            mViewCount++;
                        }

                        @Override
                        public void onSamplesRejected(SlamSparseReconstructor reconstructor, int viewId,
                                                      List<Sample2D> previousViewTrackedSamples,
                                                      List<Sample2D> currentViewTrackedSamples) {
                            mViewCount++;
                        }

                        @Override
                        public void onRequestMatches(SlamSparseReconstructor reconstructor,
                                                     List<Sample2D> allPreviousViewSamples,
                                                     List<Sample2D> previousViewTrackedSamples,
                                                     List<Sample2D> currentViewTrackedSamples,
                                                     int previousViewId, int currentViewId,
                                                     List<MatchedSamples> matches) {
                            matches.clear();

                            int numCameras = 0;
                            if (mEstimatedMetricCamera1 != null &&
                                    (mEstimatedMetricCamera1.getViewId() == previousViewId ||
                                            mEstimatedMetricCamera1.getViewId() == currentViewId)) {
                                numCameras++;
                            }
                            if (mEstimatedMetricCamera2 != null &&
                                    (mEstimatedMetricCamera2.getViewId() == previousViewId ||
                                            mEstimatedMetricCamera2.getViewId() == currentViewId)) {
                                numCameras++;
                            }
                            if (mEstimatedMetricCamera3 != null &&
                                    (mEstimatedMetricCamera3.getViewId() == previousViewId ||
                                    mEstimatedMetricCamera3.getViewId() == currentViewId)) {
                                numCameras++;
                            }

                            EstimatedCamera[] estimatedCameras = null;
                            if (numCameras > 0) {
                                estimatedCameras = new EstimatedCamera[numCameras];


                                int pos = 0;
                                if (mEstimatedMetricCamera1 != null &&
                                        (mEstimatedMetricCamera1.getViewId() == previousViewId ||
                                                mEstimatedMetricCamera1.getViewId() == currentViewId)) {
                                    estimatedCameras[pos] = mEstimatedMetricCamera1;
                                    pos++;
                                }
                                if (mEstimatedMetricCamera2 != null &&
                                        (mEstimatedMetricCamera2.getViewId() == previousViewId ||
                                                mEstimatedMetricCamera2.getViewId() == currentViewId)) {
                                    estimatedCameras[pos] = mEstimatedMetricCamera2;
                                }
                                if (mEstimatedMetricCamera3 != null &&
                                        (mEstimatedMetricCamera3.getViewId() == previousViewId ||
                                                mEstimatedMetricCamera3.getViewId() == currentViewId)) {
                                    estimatedCameras[pos] = mEstimatedMetricCamera3;
                                }
                            }

                            List<Point2D> allPreviousPoints = new ArrayList<>();
                            for (Sample2D sample : allPreviousViewSamples) {
                                allPreviousPoints.add(sample.getPoint());
                            }
                            KDTree2D tree = new KDTree2D(allPreviousPoints);

                            //search previous view tracked samples within tree
                            int numTrackedSamples = previousViewTrackedSamples.size();
                            Point2D point, nearestPoint;
                            int nearestIndex;
                            MatchedSamples match;
                            for (int i = 0; i < numTrackedSamples; i++) {
                                Sample2D previousSample = previousViewTrackedSamples.get(i);
                                point = previousSample.getPoint();
                                nearestIndex = tree.nearestIndex(point);
                                nearestPoint = allPreviousPoints.get(nearestIndex);
                                Sample2D nearestSample = allPreviousViewSamples.get(nearestIndex);

                                if (point.distanceTo(nearestPoint) > NEAREST_THRESHOLD) {
                                    continue;
                                }

                                Sample2D currentSample = currentViewTrackedSamples.get(i);

                                match = new MatchedSamples();
                                match.setSamples(new Sample2D[]{
                                        previousSample, currentSample
                                });
                                match.setViewIds(new int[]{previousViewId, currentViewId});

                                match.setReconstructedPoint(nearestSample.getReconstructedPoint());

                                if (estimatedCameras != null) {
                                    match.setCameras(estimatedCameras);
                                }

                                matches.add(match);
                            }
                        }

                        @Override
                        public void onFundamentalMatrixEstimated(SlamSparseReconstructor reconstructor,
                                                                 EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                            if (mEstimatedFundamentalMatrix == null) {
                                mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                            } else if (mEstimatedFundamentalMatrix2 == null) {
                                mEstimatedFundamentalMatrix2 = estimatedFundamentalMatrix;
                            } else if (mEstimatedFundamentalMatrix3 == null) {
                                mEstimatedFundamentalMatrix3 = estimatedFundamentalMatrix;
                            }
                        }

                        @Override
                        public void onMetricCameraEstimated(SlamSparseReconstructor reconstructor,
                                                            int previousViewId, int currentViewId,
                                                            EstimatedCamera previousCamera,
                                                            EstimatedCamera currentCamera) {
                            if (mEstimatedMetricCamera2 == null) {
                                mEstimatedMetricCamera1 = previousCamera;
                                mEstimatedMetricCamera2 = currentCamera;
                            } else if (mEstimatedMetricCamera3 == null){
                                mEstimatedMetricCamera2 = previousCamera;
                                mEstimatedMetricCamera3 = currentCamera;
                            } else if (mEstimatedMetricCamera4 == null) {
                                mEstimatedMetricCamera3 = previousCamera;
                                mEstimatedMetricCamera4 = currentCamera;
                            }
                        }

                        @Override
                        public void onMetricReconstructedPointsEstimated(SlamSparseReconstructor reconstructor,
                                                                         List<MatchedSamples> matches,
                                                                         List<ReconstructedPoint3D> points) {
                            mMetricReconstructedPoints = points;
                        }

                        @Override
                        public void onEuclideanCameraEstimated(SlamSparseReconstructor reconstructor,
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
                            } else if (mEstimatedEuclideanCamera4 == null) {
                                mEstimatedEuclideanCamera3 = previousCamera;
                                mEstimatedEuclideanCamera4 = currentCamera;
                                mScale3 = scale;
                            }
                        }

                        @Override
                        public void onEuclideanReconstructedPointsEstimated(
                                SlamSparseReconstructor reconstructor,
                                double scale, List<ReconstructedPoint3D> points) {
                            if (mEuclideanReconstructedPoints == null) {
                                mScale = scale;
                            } else if (mEstimatedMetricCamera4 == null){
                                mScale2 = scale;
                            } else {
                                mScale3 = scale;
                            }

                            mEuclideanReconstructedPoints = points;
                        }

                        @Override
                        public void onStart(SlamSparseReconstructor reconstructor) {
                            mStarted = true;
                        }

                        @Override
                        public void onFinish(SlamSparseReconstructor reconstructor) {
                            mFinished = true;
                        }

                        @Override
                        public void onCancel(SlamSparseReconstructor reconstructor) {
                            mCancelled = true;
                        }

                        @Override
                        public void onFail(SlamSparseReconstructor reconstructor) {
                            mFailed = true;
                        }
                    };

            SlamSparseReconstructor reconstructor =
                    new SlamSparseReconstructor(configuration, listener);

            //check initial values
            reset();
            assertFalse(mStarted);
            assertFalse(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertFalse(reconstructor.isFinished());

            reconstructor.start();

            if (mFailed) {
                continue;
            }

            //check correctness
            assertTrue(mStarted);
            assertTrue(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertTrue(reconstructor.isFinished());
            assertTrue(mSlamDataAvailable > 0);
            assertTrue(mSlamCameraEstimated > 0);
            assertNotNull(mSlamCamera);
            assertNotNull(mSlamCovariance);
            assertFalse(reconstructor.isFirstView());
            assertFalse(reconstructor.isSecondView());
            assertTrue(reconstructor.isAdditionalView());
            assertTrue(reconstructor.isAdditionalView());
            assertTrue(reconstructor.getViewCount() > 0);
            assertNotNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertSame(reconstructor.getCurrentEstimatedFundamentalMatrix(), mEstimatedFundamentalMatrix);
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertSame(reconstructor.getCurrentMetricEstimatedCamera(), mEstimatedMetricCamera4);
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertSame(reconstructor.getPreviousMetricEstimatedCamera(), mEstimatedMetricCamera3);
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(reconstructor.getCurrentEuclideanEstimatedCamera(), mEstimatedEuclideanCamera4);
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(reconstructor.getPreviousEuclideanEstimatedCamera(), mEstimatedEuclideanCamera3);
            assertNotNull(reconstructor.getActiveMetricReconstructedPoints());
            assertSame(reconstructor.getActiveMetricReconstructedPoints(), mMetricReconstructedPoints);
            assertNotNull(reconstructor.getActiveEuclideanReconstructedPoints());
            assertSame(reconstructor.getActiveEuclideanReconstructedPoints(), mEuclideanReconstructedPoints);
            assertEquals(reconstructor.getCurrentScale(), mScale3, 0.0);
            assertNotNull(reconstructor.getPreviousViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewNewlySpawnedSamples());

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
            PinholeCamera estimatedMetricCamera4 = mEstimatedMetricCamera4.getCamera();
            assertNotSame(mEstimatedMetricCamera1, mEstimatedEuclideanCamera1);
            assertNotSame(mEstimatedMetricCamera2, mEstimatedEuclideanCamera2);
            assertNotSame(mEstimatedMetricCamera3, mEstimatedEuclideanCamera3);
            assertNotSame(mEstimatedMetricCamera4, mEstimatedEuclideanCamera4);

            PinholeCamera estimatedEuclideanCamera1 = mEstimatedEuclideanCamera1.getCamera();
            PinholeCamera estimatedEuclideanCamera2 = mEstimatedEuclideanCamera2.getCamera();
            PinholeCamera estimatedEuclideanCamera3 = mEstimatedEuclideanCamera3.getCamera();
            PinholeCamera estimatedEuclideanCamera4 = mEstimatedEuclideanCamera4.getCamera();

            estimatedMetricCamera1.decompose();
            estimatedMetricCamera2.decompose();
            estimatedMetricCamera3.decompose();
            estimatedMetricCamera4.decompose();

            estimatedEuclideanCamera1.decompose();
            estimatedEuclideanCamera2.decompose();
            estimatedEuclideanCamera3.decompose();
            estimatedEuclideanCamera4.decompose();

            assertNotSame(mMetricReconstructedPoints, mEuclideanReconstructedPoints);

            int numReconstructedPoints = numPoints2 - start2 + numPoints3;

            if (mMetricReconstructedPoints.size() != numReconstructedPoints) {
                continue;
            }

            List<Point3D> metricReconstructedPoints3D = new ArrayList<>();
            List<Point3D> euclideanReconstructedPoints3D = new ArrayList<>();
            for (int i = 0; i < numReconstructedPoints; i++) {
                metricReconstructedPoints3D.add(
                        mMetricReconstructedPoints.get(i).getPoint());
                euclideanReconstructedPoints3D.add(
                        mEuclideanReconstructedPoints.get(i).getPoint());
            }

            //check that all points are in front of both cameras
            boolean failed = false;
            for (int i = 0; i < numReconstructedPoints; i++) {
                Point3D p = metricReconstructedPoints3D.get(i);
                Point3D pe = euclideanReconstructedPoints3D.get(i);

                if (!estimatedMetricCamera1.isPointInFrontOfCamera(p)) {
                    failed = true;
                    break;
                }
                assertTrue(estimatedMetricCamera1.isPointInFrontOfCamera(p));

                if (!estimatedMetricCamera2.isPointInFrontOfCamera(p)) {
                    failed = true;
                    break;
                }
                assertTrue(estimatedMetricCamera2.isPointInFrontOfCamera(p));

                if (!estimatedEuclideanCamera1.isPointInFrontOfCamera(pe)) {
                    failed = true;
                    break;
                }
                assertTrue(estimatedEuclideanCamera1.isPointInFrontOfCamera(pe));

                if (!estimatedEuclideanCamera2.isPointInFrontOfCamera(pe)) {
                    failed = true;
                    break;
                }
                assertTrue(estimatedEuclideanCamera2.isPointInFrontOfCamera(pe));
            }

            if (failed) {
                continue;
            }

            Point3D metricCenter1 = estimatedMetricCamera1.getCameraCenter();
            Point3D metricCenter2 = estimatedMetricCamera2.getCameraCenter();
            Point3D metricCenter3 = estimatedMetricCamera3.getCameraCenter();
            Point3D metricCenter4 = estimatedMetricCamera4.getCameraCenter();

            Point3D euclideanCenter1 = estimatedEuclideanCamera1.getCameraCenter();
            Point3D euclideanCenter2 = estimatedEuclideanCamera2.getCameraCenter();
            Point3D euclideanCenter3 = estimatedEuclideanCamera3.getCameraCenter();
            Point3D euclideanCenter4 = estimatedEuclideanCamera4.getCameraCenter();

            PinholeCameraIntrinsicParameters euclideanIntrinsic1 =
                    estimatedEuclideanCamera1.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters euclideanIntrinsic2 =
                    estimatedEuclideanCamera2.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters euclideanIntrinsic3 =
                    estimatedEuclideanCamera3.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters euclideanIntrinsic4 =
                    estimatedEuclideanCamera4.getIntrinsicParameters();

            Rotation3D euclideanRotation1 = estimatedEuclideanCamera1.getCameraRotation();
            Rotation3D euclideanRotation2 = estimatedEuclideanCamera2.getCameraRotation();
            Rotation3D euclideanRotation3 = estimatedEuclideanCamera3.getCameraRotation();
            Rotation3D euclideanRotation4 = estimatedEuclideanCamera4.getCameraRotation();

            double estimatedBaseline = euclideanCenter1.distanceTo(euclideanCenter2);
            double estimatedBaseline2 = euclideanCenter2.distanceTo(euclideanCenter3);
            double estimatedBaseline3 = euclideanCenter3.distanceTo(euclideanCenter4);

            //check cameras are correct
            double maxBaseline = Math.max(estimatedBaseline, baseline);
            double absoluteScaleError = RELATIVE_ERROR * maxBaseline;
            if (Math.abs(estimatedBaseline - baseline) > absoluteScaleError) {
                continue;
            }
            assertEquals(estimatedBaseline, baseline, absoluteScaleError);

            double maxBaseline2 = Math.max(estimatedBaseline2, baseline2);
            double absoluteScaleError2 = RELATIVE_ERROR * maxBaseline2;
            if (Math.abs(estimatedBaseline2 - baseline2) > absoluteScaleError2) {
                continue;
            }
            assertEquals(estimatedBaseline2, baseline2, absoluteScaleError2);

            double maxBaseline3 = Math.max(estimatedBaseline3, baseline3);
            double absoluteScaleError3 = RELATIVE_ERROR * maxBaseline3;
            if (Math.abs(estimatedBaseline3 - baseline3) > absoluteScaleError3) {
                continue;
            }
            assertEquals(estimatedBaseline3, baseline3, absoluteScaleError3);


            MetricTransformation3D scaleTransformation
                    = new MetricTransformation3D(baseline);
            Point3D rescaledCenter1 = scaleTransformation.transformAndReturnNew(
                    metricCenter1);
            Point3D rescaledCenter2 = scaleTransformation.transformAndReturnNew(
                    metricCenter2);
            Point3D rescaledCenter3 = scaleTransformation.transformAndReturnNew(
                    metricCenter3);
            Point3D rescaledCenter4 = scaleTransformation.transformAndReturnNew(
                    metricCenter4);


            assertTrue(center1.equals(rescaledCenter1, LARGE_ABSOLUTE_ERROR));
            assertTrue(center2.equals(rescaledCenter2, LARGE_ABSOLUTE_ERROR));
            assertTrue(center3.equals(rescaledCenter3, LARGE_ABSOLUTE_ERROR));
            assertTrue(center4.equals(rescaledCenter4, LARGE_ABSOLUTE_ERROR));

            assertTrue(center1.equals(euclideanCenter1, ABSOLUTE_ERROR));
            if (!center2.equals(euclideanCenter2, absoluteScaleError)) {
                continue;
            }
            assertTrue(center2.equals(euclideanCenter2, absoluteScaleError));
            if(!center3.equals(euclideanCenter3, absoluteScaleError2)) {
                continue;
            }
            assertTrue(center3.equals(euclideanCenter3, absoluteScaleError2));
            if(!center4.equals(euclideanCenter4, 3*absoluteScaleError3)) {
                continue;
            }
            assertTrue(center4.equals(euclideanCenter4, 3*absoluteScaleError3));

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

            assertEquals(euclideanIntrinsic4.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic4.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic4.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic4.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic4.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertTrue(euclideanRotation1.asInhomogeneousMatrix().equals(
                    rotation1.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
            assertTrue(euclideanRotation2.asInhomogeneousMatrix().equals(
                    rotation2.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
            assertTrue(euclideanRotation3.asInhomogeneousMatrix().equals(
                    rotation3.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
            assertTrue(euclideanRotation4.asInhomogeneousMatrix().equals(
                    rotation4.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

            //check that points are correct (after scale correction)

            //check that scale error is less than 5%
            assertTrue(Math.abs(baseline / mScale - 1.0) < RELATIVE_ERROR);
            assertTrue(Math.abs(baseline / mScale2 - 1.0) < RELATIVE_ERROR);
            assertTrue(Math.abs(baseline / mScale3 - 1.0) < RELATIVE_ERROR);
            scaleTransformation = new MetricTransformation3D(mScale3);

            int numValidPoints = 0;
            double scaleX, scaleY, scaleZ;
            for (int i = start2; i < numPoints2; i++) {
                Point3D point = points3D2.get(i);
                Point3D euclideanPoint = euclideanReconstructedPoints3D.get(i - start2);

                //check metric points
                Point3D rescaledPoint = Point3D.create();
                scaleTransformation.transform(metricReconstructedPoints3D.get(i - start2),
                        rescaledPoint);

                assertTrue(euclideanPoint.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR));

                scaleX = point.getInhomX() / rescaledPoint.getInhomX();
                scaleY = point.getInhomY() / rescaledPoint.getInhomY();
                scaleZ = point.getInhomZ() / rescaledPoint.getInhomZ();

                //check that scale error is less than 5%
                if (Math.abs(scaleX - baseline / mScale3) > ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleX, baseline / mScale3, ABSOLUTE_ERROR);
                if (Math.abs(scaleY - baseline / mScale3) > ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleY, baseline / mScale3, ABSOLUTE_ERROR);
                if (Math.abs(scaleZ - baseline / mScale3) > ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleZ, baseline / mScale3, ABSOLUTE_ERROR);
                if (Math.abs(scaleX - 1.0) > RELATIVE_ERROR ||
                        Math.abs(scaleY - 1.0) > RELATIVE_ERROR ||
                        Math.abs(scaleZ - 1.0) > RELATIVE_ERROR) {
                    continue;
                }
                rescaledPoint.setInhomogeneousCoordinates(
                        rescaledPoint.getInhomX() * baseline / mScale3,
                        rescaledPoint.getInhomY() * baseline / mScale3,
                        rescaledPoint.getInhomZ() * baseline / mScale3);
                if (point.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR)) {
                    numValidPoints++;
                }

                //check euclidean points
                scaleX = point.getInhomX() / euclideanPoint.getInhomX();
                scaleY = point.getInhomY() / euclideanPoint.getInhomY();
                scaleZ = point.getInhomZ() / euclideanPoint.getInhomZ();

                //check that scale error is less than 5%
                assertEquals(scaleX, baseline / mScale3, ABSOLUTE_ERROR);
                assertEquals(scaleY, baseline / mScale3, ABSOLUTE_ERROR);
                assertEquals(scaleZ, baseline / mScale3, ABSOLUTE_ERROR);
                assertTrue(Math.abs(scaleX - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleY - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleZ - 1.0) < RELATIVE_ERROR);
            }

            for (int i = 0; i < numPoints3; i++) {
                Point3D point = points3D3.get(i);
                Point3D euclideanPoint = euclideanReconstructedPoints3D.get(i + numPoints2 - start2);

                //check metric points
                Point3D rescaledPoint = Point3D.create();
                scaleTransformation.transform(metricReconstructedPoints3D.get(i + numPoints2 - start2),
                        rescaledPoint);

                assertTrue(euclideanPoint.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR));

                scaleX = point.getInhomX() / rescaledPoint.getInhomX();
                scaleY = point.getInhomY() / rescaledPoint.getInhomY();
                scaleZ = point.getInhomZ() / rescaledPoint.getInhomZ();

                //check that scale error is less than 5%
                if (Math.abs(scaleX - baseline / mScale3) > ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleX, baseline / mScale3, ABSOLUTE_ERROR);
                if (Math.abs(scaleY - baseline / mScale3) > ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleY, baseline / mScale3, ABSOLUTE_ERROR);
                if (Math.abs(scaleZ - baseline / mScale3) > ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleZ, baseline / mScale3, ABSOLUTE_ERROR);
                if (Math.abs(scaleX - 1.0) > RELATIVE_ERROR ||
                        Math.abs(scaleY - 1.0) > RELATIVE_ERROR ||
                        Math.abs(scaleZ - 1.0) > RELATIVE_ERROR) {
                    continue;
                }
                rescaledPoint.setInhomogeneousCoordinates(
                        rescaledPoint.getInhomX() * baseline / mScale3,
                        rescaledPoint.getInhomY() * baseline / mScale3,
                        rescaledPoint.getInhomZ() * baseline / mScale3);
                if (point.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR)) {
                    numValidPoints++;
                }

                //check euclidean points
                scaleX = point.getInhomX() / euclideanPoint.getInhomX();
                scaleY = point.getInhomY() / euclideanPoint.getInhomY();
                scaleZ = point.getInhomZ() / euclideanPoint.getInhomZ();

                //check that scale error is less than 5%
                assertEquals(scaleX, baseline / mScale3, ABSOLUTE_ERROR);
                assertEquals(scaleY, baseline / mScale3, ABSOLUTE_ERROR);
                assertEquals(scaleZ, baseline / mScale3, ABSOLUTE_ERROR);
                assertTrue(Math.abs(scaleX - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleY - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleZ - 1.0) < RELATIVE_ERROR);
            }

            if (numValidPoints == 0) {
                continue;
            }

            double scaleRelativeError = Math.abs(baseline / mScale3 - 1.0);
            LOGGER.log(Level.INFO,
                    "Baseline relative error without noise: {0,number,0.000%}",
                    scaleRelativeError);

            //check scales
            double maxScale = Math.max(Math.max(mScale, mScale2), mScale3);
            scaleRelativeError = RELATIVE_ERROR * maxScale;
            if (Math.abs(mScale - mScale2) > scaleRelativeError) {
                continue;
            }
            assertEquals(mScale, mScale2, scaleRelativeError);

            numValid++;

            if (numValid > 0) {
                break;
            }
        }

        assertTrue(numValid > 0);
    }

    private void reset() {
        mViewCount = 0;
        mEstimatedFundamentalMatrix = mEstimatedFundamentalMatrix2 =
                mEstimatedFundamentalMatrix3 = null;
        mEstimatedMetricCamera1 = mEstimatedMetricCamera2 =
                mEstimatedMetricCamera3 = mEstimatedMetricCamera4 = null;
        mEstimatedEuclideanCamera1 = mEstimatedEuclideanCamera2 =
                mEstimatedEuclideanCamera3 = mEstimatedEuclideanCamera4 = null;
        mMetricReconstructedPoints = null;
        mEuclideanReconstructedPoints = null;
        mStarted = mFinished = mFailed = mCancelled = false;
        mScale = 0.0;
        mTimestamp = 0;
        mSlamDataAvailable = 0;
        mSlamCameraEstimated = 0;
        mSlamCamera = null;
    }

    private SlamCalibrator createFinishedCalibrator(float accelerationOffsetX,
                                                    float accelerationOffsetY, float accelerationOffsetZ,
                                                    float angularOffsetX, float angularOffsetY, float angularOffsetZ,
                                                    GaussianRandomizer noiseRandomizer) {
        SlamCalibrator calibrator = SlamEstimator.createCalibrator();
        calibrator.setConvergenceThreshold(ABSOLUTE_ERROR);
        calibrator.setMaxNumSamples(MAX_CALIBRATION_SAMPLES);

        long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;

        float accelerationNoiseX, accelerationNoiseY, accelerationNoiseZ;
        float angularNoiseX, angularNoiseY, angularNoiseZ;

        double accelerationX, accelerationY, accelerationZ;
        double angularX, angularY, angularZ;

        for (int i = 0; i < MAX_CALIBRATION_SAMPLES; i++) {
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

            calibrator.updateAccelerometerSample(timestamp, (float) accelerationX,
                    (float) accelerationY, (float) accelerationZ);
            calibrator.updateGyroscopeSample(timestamp, (float) angularX, (float) angularY,
                    (float) angularZ);

            if (calibrator.isFinished()) {
                break;
            }

            timestamp += DELTA_NANOS;
        }

        return calibrator;
    }
}
