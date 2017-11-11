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

public class SlamPairedViewsSparseReconstructorTest {

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
    private EstimatedCamera mEstimatedEuclideanCamera1;
    private EstimatedCamera mEstimatedEuclideanCamera2;
    private EstimatedCamera mEstimatedEuclideanCamera3;
    private EstimatedCamera mEstimatedEuclideanCamera4;
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

    public SlamPairedViewsSparseReconstructorTest() { }

    @BeforeClass
    public static void setUpClass() { }

    @AfterClass
    public static void tearDownClass() { }

    @Before
    public void setUp() {
        mViewCount = 0;
        mEstimatedFundamentalMatrix = mEstimatedFundamentalMatrix2 =
                mEstimatedFundamentalMatrix3 = null;
        mEstimatedEuclideanCamera1 = mEstimatedEuclideanCamera2 =
                mEstimatedEuclideanCamera3 = mEstimatedEuclideanCamera4 = null;
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
        assertEquals(SlamPairedViewsSparseReconstructor.MIN_NUMBER_OF_VIEWS, 2);

        SlamPairedViewsSparseReconstructorConfiguration configuration =
                new SlamPairedViewsSparseReconstructorConfiguration();
        SlamPairedViewsSparseReconstructorListener listener =
                new SlamPairedViewsSparseReconstructorListener() {
                    @Override
                    public void onSlamDataAvailable(SlamPairedViewsSparseReconstructor reconstructor, double positionX,
                            double positionY, double positionZ, double velocityX, double velocityY, double velocityZ,
                            double accelerationX, double accelerationY, double accelerationZ, double quaternionA,
                            double quaternionB, double quaternionC, double quaternionD, double angularSpeedX,
                            double angularSpeedY, double angularSpeedZ, Matrix covariance) { }

                    @Override
                    public void onSlamCameraEstimated(SlamPairedViewsSparseReconstructor reconstructor,
                            PinholeCamera camera) { }

                    @Override
                    public boolean hasMoreViewsAvailable(SlamPairedViewsSparseReconstructor reconstructor) {
                        return false;
                    }

                    @Override
                    public void onRequestSamplesForCurrentViewPair(SlamPairedViewsSparseReconstructor reconstructor,
                            int viewId1, int viewId2, List<Sample2D> samples1, List<Sample2D> samples2) { }

                    @Override
                    public void onSamplesAccepted(SlamPairedViewsSparseReconstructor reconstructor, int viewId1,
                            int viewId2, List<Sample2D> samples1, List<Sample2D> samples2) { }

                    @Override
                    public void onSamplesRejected(SlamPairedViewsSparseReconstructor reconstructor, int viewId1,
                            int viewId2, List<Sample2D> samples1, List<Sample2D> samples2) { }

                    @Override
                    public void onRequestMatches(SlamPairedViewsSparseReconstructor reconstructor, int viewId1,
                            int viewId2, List<Sample2D> samples1, List<Sample2D> samples2,
                            List<MatchedSamples> matches) { }

                    @Override
                    public void onFundamentalMatrixEstimated(SlamPairedViewsSparseReconstructor reconstructor,
                            int viewId1, int viewId2, EstimatedFundamentalMatrix estimatedFundamentalMatrix) { }

                    @Override
                    public void onEuclideanCameraPairEstimated(SlamPairedViewsSparseReconstructor reconstructor,
                            int viewId1, int viewId2, double scale, EstimatedCamera camera1,
                            EstimatedCamera camera2) { }

                    @Override
                    public void onEuclideanReconstructedPointsEstimated(
                            SlamPairedViewsSparseReconstructor reconstructor, int viewId1, int viewId2, double scale,
                            List<ReconstructedPoint3D> points) { }

                    @Override
                    public PinholeCameraIntrinsicParameters onIntrinsicParametersRequested(
                            SlamPairedViewsSparseReconstructor reconstructor, int viewId) {
                        return null;
                    }

                    @Override
                    public void onStart(SlamPairedViewsSparseReconstructor reconstructor) { }

                    @Override
                    public void onFinish(SlamPairedViewsSparseReconstructor reconstructor) { }

                    @Override
                    public void onCancel(SlamPairedViewsSparseReconstructor reconstructor) { }

                    @Override
                    public void onFail(SlamPairedViewsSparseReconstructor reconstructor) { }
                };

        SlamPairedViewsSparseReconstructor reconstructor = new SlamPairedViewsSparseReconstructor(listener);

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
        assertNull(reconstructor.getMetricReconstructedPoints());
        assertNull(reconstructor.getEuclideanReconstructedPoints());
        assertEquals(reconstructor.getCurrentScale(), BaseSparseReconstructor.DEFAULT_SCALE, 0.0);
        assertNull(reconstructor.getPreviousViewSamples());
        assertNull(reconstructor.getCurrentViewSamples());
        assertTrue(reconstructor.isFirstViewPair());
        assertFalse(reconstructor.isAdditionalViewPair());

        //constructor with configuration and listener
        reconstructor = new SlamPairedViewsSparseReconstructor(configuration, listener);

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
        assertNull(reconstructor.getMetricReconstructedPoints());
        assertNull(reconstructor.getEuclideanReconstructedPoints());
        assertEquals(reconstructor.getCurrentScale(), BaseSparseReconstructor.DEFAULT_SCALE, 0.0);
        assertNull(reconstructor.getPreviousViewSamples());
        assertNull(reconstructor.getCurrentViewSamples());
        assertTrue(reconstructor.isFirstViewPair());
        assertFalse(reconstructor.isAdditionalViewPair());
    }

    @Test
    public void testGeneralPointsEssentialWithConstantAccelerationAndRotationWithoutNoiseTwoViews()
            throws InvalidPairOfCamerasException, AlgebraException, CameraException, RotationException,
            com.irurueta.geometry.estimators.NotReadyException,
            com.irurueta.geometry.NotAvailableException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);

            SlamPairedViewsSparseReconstructorConfiguration configuration =
                    new SlamPairedViewsSparseReconstructorConfiguration();
            configuration.setPairedCamerasEstimatorMethod(
                    InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);
            configuration.setIntrinsicParametersKnown(true);

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
            double aspectRatio = configuration.getPairedCamerasAspectRatio();
            double skewness = 0.0;
            double principalPoint = 0.0;

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength, focalLength,
                            principalPoint, principalPoint, skewness);
            intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);


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

            final int numPoints = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);

            InhomogeneousPoint3D point3D;
            List<InhomogeneousPoint3D> points3D =
                    new ArrayList<>();
            Point2D projectedPoint1, projectedPoint2;
            final List<Point2D> projectedPoints1 = new ArrayList<>();
            final List<Point2D> projectedPoints2 = new ArrayList<>();
            boolean front1, front2;
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
                        fail("max tries reached");
                    }
                    numTry++;
                } while(!front1 || !front2);
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

            SlamPairedViewsSparseReconstructorListener listener =
                    new SlamPairedViewsSparseReconstructorListener() {
                        @Override
                        public void onSlamDataAvailable(SlamPairedViewsSparseReconstructor reconstructor,
                                double positionX, double positionY, double positionZ, double velocityX,
                                double velocityY, double velocityZ, double accelerationX, double accelerationY,
                                double accelerationZ, double quaternionA, double quaternionB, double quaternionC,
                                double quaternionD, double angularSpeedX, double angularSpeedY, double angularSpeedZ,
                                Matrix covariance) {
                            mSlamDataAvailable++;
                            mSlamCovariance = covariance;
                        }

                        @Override
                        public void onSlamCameraEstimated(SlamPairedViewsSparseReconstructor reconstructor,
                                                          PinholeCamera camera) {
                            mSlamCameraEstimated++;
                            mSlamCamera = camera;
                        }

                        @Override
                        public boolean hasMoreViewsAvailable(SlamPairedViewsSparseReconstructor reconstructor) {
                            return mViewCount < 2;
                        }

                        @Override
                        public void onRequestSamplesForCurrentViewPair(SlamPairedViewsSparseReconstructor reconstructor,
                                int viewId1, int viewId2, List<Sample2D> samples1, List<Sample2D> samples2) {

                            samples1.clear();
                            samples2.clear();

                            Sample2D sample1, sample2;
                            for (int i = 0; i < numPoints; i++) {
                                sample1 = new Sample2D();
                                sample1.setPoint(projectedPoints1.get(i));
                                sample1.setViewId(viewId1);
                                samples1.add(sample1);

                                sample2 = new Sample2D();
                                sample2.setPoint(projectedPoints2.get(i));
                                sample2.setViewId(viewId2);
                                samples2.add(sample2);
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

                        @Override
                        public void onSamplesAccepted(SlamPairedViewsSparseReconstructor reconstructor, int viewId1,
                                int viewId2, List<Sample2D> samples1, List<Sample2D> samples2) {
                            mViewCount += 2;
                        }

                        @Override
                        public void onSamplesRejected(SlamPairedViewsSparseReconstructor reconstructor, int viewId1,
                                int viewId2, List<Sample2D> samples1, List<Sample2D> samples2) {
                            mViewCount += 2;
                        }

                        @Override
                        public void onRequestMatches(SlamPairedViewsSparseReconstructor reconstructor, int viewId1,
                                int viewId2, List<Sample2D> samples1, List<Sample2D> samples2,
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
                        public void onFundamentalMatrixEstimated(SlamPairedViewsSparseReconstructor reconstructor,
                                int viewId1, int viewId2, EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                            mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                        }

                        @Override
                        public void onEuclideanCameraPairEstimated(SlamPairedViewsSparseReconstructor reconstructor,
                                int viewId1, int viewId2, double scale, EstimatedCamera camera1,
                                EstimatedCamera camera2) {
                            mEstimatedEuclideanCamera1 = camera1;
                            mEstimatedEuclideanCamera2 = camera2;
                            mScale = scale;
                        }

                        @Override
                        public void onEuclideanReconstructedPointsEstimated(
                                SlamPairedViewsSparseReconstructor reconstructor, int viewId1, int viewId2,
                                double scale, List<ReconstructedPoint3D> points) {
                            mEuclideanReconstructedPoints = points;
                            mScale = scale;
                        }

                        @Override
                        public PinholeCameraIntrinsicParameters onIntrinsicParametersRequested(SlamPairedViewsSparseReconstructor reconstructor, int viewId) {
                            return intrinsic;
                        }

                        @Override
                        public void onStart(SlamPairedViewsSparseReconstructor reconstructor) {
                            mStarted = true;
                        }

                        @Override
                        public void onFinish(SlamPairedViewsSparseReconstructor reconstructor) {
                            mFinished = true;
                        }

                        @Override
                        public void onCancel(SlamPairedViewsSparseReconstructor reconstructor) {
                            mCancelled = true;
                        }

                        @Override
                        public void onFail(SlamPairedViewsSparseReconstructor reconstructor) {
                            mFailed = true;
                        }
                    };

            SlamPairedViewsSparseReconstructor reconstructor =
                    new SlamPairedViewsSparseReconstructor(configuration, listener);

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
            assertFalse(reconstructor.isFirstViewPair());
            assertTrue(reconstructor.isAdditionalViewPair());
            assertTrue(reconstructor.getViewCount() > 0);
            assertNotNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertSame(reconstructor.getCurrentEstimatedFundamentalMatrix(), mEstimatedFundamentalMatrix);
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(reconstructor.getCurrentEuclideanEstimatedCamera(), mEstimatedEuclideanCamera2);
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(reconstructor.getPreviousEuclideanEstimatedCamera(), mEstimatedEuclideanCamera1);
            assertNotNull(reconstructor.getMetricReconstructedPoints());
            assertNotNull(reconstructor.getEuclideanReconstructedPoints());
            assertSame(reconstructor.getEuclideanReconstructedPoints(), mEuclideanReconstructedPoints);
            assertEquals(reconstructor.getCurrentScale(), mScale, 0.0);
            assertNotNull(reconstructor.getPreviousViewSamples());
            assertNotNull(reconstructor.getCurrentViewSamples());

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

            PinholeCamera estimatedEuclideanCamera1 = mEstimatedEuclideanCamera1.getCamera();
            PinholeCamera estimatedEuclideanCamera2 = mEstimatedEuclideanCamera2.getCamera();

            estimatedEuclideanCamera1.decompose();
            estimatedEuclideanCamera2.decompose();

            List<Point3D> euclideanReconstructedPoints3D = new ArrayList<>();
            for (int i = 0; i < numPoints; i++) {
                euclideanReconstructedPoints3D.add(
                        mEuclideanReconstructedPoints.get(i).getPoint());
            }

            //check that all points are in front of both cameras
            for (int i = 0; i < numPoints; i++) {
                Point3D p = euclideanReconstructedPoints3D.get(i);
                assertTrue(estimatedEuclideanCamera1.isPointInFrontOfCamera(p));
                assertTrue(estimatedEuclideanCamera2.isPointInFrontOfCamera(p));
            }

            Point3D euclideanCenter1 = estimatedEuclideanCamera1.getCameraCenter();
            Point3D euclideanCenter2 = estimatedEuclideanCamera2.getCameraCenter();

            PinholeCameraIntrinsicParameters euclideanIntrinsic1 =
                    estimatedEuclideanCamera1.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters euclideanIntrinsic2 =
                    estimatedEuclideanCamera2.getIntrinsicParameters();

            Rotation3D euclideanRotation1 = estimatedEuclideanCamera1.getCameraRotation();
            Rotation3D euclideanRotation2 = estimatedEuclideanCamera2.getCameraRotation();

            //check scale
            double estimatedBaseline = euclideanCenter1.distanceTo(euclideanCenter2);

            //check cameras are correct
            double maxBaseline = Math.max(estimatedBaseline, baseline);
            double absoluteScaleError = RELATIVE_ERROR * maxBaseline;
            if (Math.abs(estimatedBaseline - baseline) > absoluteScaleError) {
                continue;
            }
            assertEquals(estimatedBaseline, baseline, absoluteScaleError);
            assertEquals(mScale, estimatedBaseline, LARGE_ABSOLUTE_ERROR);

            //check cameras
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
                    = new MetricTransformation3D(baseline / mScale);

            int numValidPoints = 0;
            double scaleX, scaleY, scaleZ;
            for (int i = 0; i < numPoints; i++) {
                Point3D point = points3D.get(i);
                Point3D euclideanPoint = euclideanReconstructedPoints3D.get(i);

                //check metric points
                Point3D rescaledPoint = Point3D.create();
                scaleTransformation.transform(euclideanPoint, rescaledPoint);

                //euclidean and rescaled points match
                assertTrue(euclideanPoint.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR));

                scaleX = point.getInhomX() / rescaledPoint.getInhomX();
                scaleY = point.getInhomY() / rescaledPoint.getInhomY();
                scaleZ = point.getInhomZ() / rescaledPoint.getInhomZ();

                //check that scale error is less than 5%
                if (Math.abs(scaleX - 1.0) > LARGE_ABSOLUTE_ERROR ||
                        Math.abs(scaleY - 1.0) > LARGE_ABSOLUTE_ERROR ||
                        Math.abs(scaleZ - 1.0) > LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleX, 1.0, LARGE_ABSOLUTE_ERROR);
                assertEquals(scaleY, 1.0, LARGE_ABSOLUTE_ERROR);
                assertEquals(scaleZ, 1.0, LARGE_ABSOLUTE_ERROR);
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
            throws InvalidPairOfCamerasException, AlgebraException, CameraException, RotationException,
            com.irurueta.geometry.estimators.NotReadyException,
            com.irurueta.geometry.NotAvailableException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer offsetRandomizer = new UniformRandomizer(
                    new Random());
            GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);

            SlamPairedViewsSparseReconstructorConfiguration configuration =
                    new SlamPairedViewsSparseReconstructorConfiguration();
            configuration.setPairedCamerasEstimatorMethod(
                    InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);
            configuration.setIntrinsicParametersKnown(true);

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
            double aspectRatio = configuration.getPairedCamerasAspectRatio();
            double skewness = 0.0;
            double principalPoint = 0.0;

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength, focalLength,
                            principalPoint, principalPoint, skewness);
            intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);


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

            final int numPoints = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);

            InhomogeneousPoint3D point3D;
            List<InhomogeneousPoint3D> points3D =
                    new ArrayList<>();
            Point2D projectedPoint1, projectedPoint2;
            final List<Point2D> projectedPoints1 = new ArrayList<>();
            final List<Point2D> projectedPoints2 = new ArrayList<>();
            boolean front1, front2;
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
                        fail("max tries reached");
                    }
                    numTry++;
                } while(!front1 || !front2);
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

            SlamPairedViewsSparseReconstructorListener listener =
                    new SlamPairedViewsSparseReconstructorListener() {
                        @Override
                        public void onSlamDataAvailable(SlamPairedViewsSparseReconstructor reconstructor,
                                                        double positionX, double positionY, double positionZ, double velocityX,
                                                        double velocityY, double velocityZ, double accelerationX, double accelerationY,
                                                        double accelerationZ, double quaternionA, double quaternionB, double quaternionC,
                                                        double quaternionD, double angularSpeedX, double angularSpeedY, double angularSpeedZ,
                                                        Matrix covariance) {
                            mSlamDataAvailable++;
                            mSlamCovariance = covariance;
                        }

                        @Override
                        public void onSlamCameraEstimated(SlamPairedViewsSparseReconstructor reconstructor,
                                                          PinholeCamera camera) {
                            mSlamCameraEstimated++;
                            mSlamCamera = camera;
                        }

                        @Override
                        public boolean hasMoreViewsAvailable(SlamPairedViewsSparseReconstructor reconstructor) {
                            return mViewCount < 2;
                        }

                        @Override
                        public void onRequestSamplesForCurrentViewPair(SlamPairedViewsSparseReconstructor reconstructor,
                                                                       int viewId1, int viewId2, List<Sample2D> samples1, List<Sample2D> samples2) {

                            samples1.clear();
                            samples2.clear();

                            Sample2D sample1, sample2;
                            for (int i = 0; i < numPoints; i++) {
                                sample1 = new Sample2D();
                                sample1.setPoint(projectedPoints1.get(i));
                                sample1.setViewId(viewId1);
                                samples1.add(sample1);

                                sample2 = new Sample2D();
                                sample2.setPoint(projectedPoints2.get(i));
                                sample2.setViewId(viewId2);
                                samples2.add(sample2);
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

                        @Override
                        public void onSamplesAccepted(SlamPairedViewsSparseReconstructor reconstructor, int viewId1,
                                                      int viewId2, List<Sample2D> samples1, List<Sample2D> samples2) {
                            mViewCount += 2;
                        }

                        @Override
                        public void onSamplesRejected(SlamPairedViewsSparseReconstructor reconstructor, int viewId1,
                                                      int viewId2, List<Sample2D> samples1, List<Sample2D> samples2) {
                            mViewCount += 2;
                        }

                        @Override
                        public void onRequestMatches(SlamPairedViewsSparseReconstructor reconstructor, int viewId1,
                                                     int viewId2, List<Sample2D> samples1, List<Sample2D> samples2,
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
                        public void onFundamentalMatrixEstimated(SlamPairedViewsSparseReconstructor reconstructor,
                                                                 int viewId1, int viewId2, EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                            mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                        }

                        @Override
                        public void onEuclideanCameraPairEstimated(SlamPairedViewsSparseReconstructor reconstructor,
                                                                   int viewId1, int viewId2, double scale, EstimatedCamera camera1,
                                                                   EstimatedCamera camera2) {
                            mEstimatedEuclideanCamera1 = camera1;
                            mEstimatedEuclideanCamera2 = camera2;
                            mScale = scale;
                        }

                        @Override
                        public void onEuclideanReconstructedPointsEstimated(
                                SlamPairedViewsSparseReconstructor reconstructor, int viewId1, int viewId2,
                                double scale, List<ReconstructedPoint3D> points) {
                            mEuclideanReconstructedPoints = points;
                            mScale = scale;
                        }

                        @Override
                        public PinholeCameraIntrinsicParameters onIntrinsicParametersRequested(SlamPairedViewsSparseReconstructor reconstructor, int viewId) {
                            return intrinsic;
                        }

                        @Override
                        public void onStart(SlamPairedViewsSparseReconstructor reconstructor) {
                            mStarted = true;
                        }

                        @Override
                        public void onFinish(SlamPairedViewsSparseReconstructor reconstructor) {
                            mFinished = true;
                        }

                        @Override
                        public void onCancel(SlamPairedViewsSparseReconstructor reconstructor) {
                            mCancelled = true;
                        }

                        @Override
                        public void onFail(SlamPairedViewsSparseReconstructor reconstructor) {
                            mFailed = true;
                        }
                    };

            SlamPairedViewsSparseReconstructor reconstructor =
                    new SlamPairedViewsSparseReconstructor(configuration, listener);

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
            assertFalse(reconstructor.isFirstViewPair());
            assertTrue(reconstructor.isAdditionalViewPair());
            assertTrue(reconstructor.getViewCount() > 0);
            assertNotNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertSame(reconstructor.getCurrentEstimatedFundamentalMatrix(), mEstimatedFundamentalMatrix);
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(reconstructor.getCurrentEuclideanEstimatedCamera(), mEstimatedEuclideanCamera2);
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(reconstructor.getPreviousEuclideanEstimatedCamera(), mEstimatedEuclideanCamera1);
            assertNotNull(reconstructor.getMetricReconstructedPoints());
            assertNotNull(reconstructor.getEuclideanReconstructedPoints());
            assertSame(reconstructor.getEuclideanReconstructedPoints(), mEuclideanReconstructedPoints);
            assertEquals(reconstructor.getCurrentScale(), mScale, 0.0);
            assertNotNull(reconstructor.getPreviousViewSamples());
            assertNotNull(reconstructor.getCurrentViewSamples());

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

            PinholeCamera estimatedEuclideanCamera1 = mEstimatedEuclideanCamera1.getCamera();
            PinholeCamera estimatedEuclideanCamera2 = mEstimatedEuclideanCamera2.getCamera();

            estimatedEuclideanCamera1.decompose();
            estimatedEuclideanCamera2.decompose();

            List<Point3D> euclideanReconstructedPoints3D = new ArrayList<>();
            for (int i = 0; i < numPoints; i++) {
                euclideanReconstructedPoints3D.add(
                        mEuclideanReconstructedPoints.get(i).getPoint());
            }

            //check that all points are in front of both cameras
            for (int i = 0; i < numPoints; i++) {
                Point3D p = euclideanReconstructedPoints3D.get(i);
                assertTrue(estimatedEuclideanCamera1.isPointInFrontOfCamera(p));
                assertTrue(estimatedEuclideanCamera2.isPointInFrontOfCamera(p));
            }

            Point3D euclideanCenter1 = estimatedEuclideanCamera1.getCameraCenter();
            Point3D euclideanCenter2 = estimatedEuclideanCamera2.getCameraCenter();

            PinholeCameraIntrinsicParameters euclideanIntrinsic1 =
                    estimatedEuclideanCamera1.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters euclideanIntrinsic2 =
                    estimatedEuclideanCamera2.getIntrinsicParameters();

            Rotation3D euclideanRotation1 = estimatedEuclideanCamera1.getCameraRotation();
            Rotation3D euclideanRotation2 = estimatedEuclideanCamera2.getCameraRotation();

            //check scale
            double estimatedBaseline = euclideanCenter1.distanceTo(euclideanCenter2);

            //check cameras are correct
            double maxBaseline = Math.max(estimatedBaseline, baseline);
            double absoluteScaleError = RELATIVE_ERROR * maxBaseline;
            if (Math.abs(estimatedBaseline - baseline) > absoluteScaleError) {
                continue;
            }
            assertEquals(estimatedBaseline, baseline, absoluteScaleError);
            assertEquals(mScale, estimatedBaseline, LARGE_ABSOLUTE_ERROR);

            //check cameras
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
                    = new MetricTransformation3D(baseline / mScale);

            int numValidPoints = 0;
            double scaleX, scaleY, scaleZ;
            for (int i = 0; i < numPoints; i++) {
                Point3D point = points3D.get(i);
                Point3D euclideanPoint = euclideanReconstructedPoints3D.get(i);

                //check metric points
                Point3D rescaledPoint = Point3D.create();
                scaleTransformation.transform(euclideanPoint, rescaledPoint);

                //euclidean and rescaled points match
                assertTrue(euclideanPoint.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR));

                scaleX = point.getInhomX() / rescaledPoint.getInhomX();
                scaleY = point.getInhomY() / rescaledPoint.getInhomY();
                scaleZ = point.getInhomZ() / rescaledPoint.getInhomZ();

                //check that scale error is less than 5%
                if (Math.abs(scaleX - 1.0) > LARGE_ABSOLUTE_ERROR ||
                        Math.abs(scaleY - 1.0) > LARGE_ABSOLUTE_ERROR ||
                        Math.abs(scaleZ - 1.0) > LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleX, 1.0, LARGE_ABSOLUTE_ERROR);
                assertEquals(scaleY, 1.0, LARGE_ABSOLUTE_ERROR);
                assertEquals(scaleZ, 1.0, LARGE_ABSOLUTE_ERROR);
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

    private void reset() {
        mViewCount = 0;
        mEstimatedFundamentalMatrix = mEstimatedFundamentalMatrix2 =
                mEstimatedFundamentalMatrix3 = null;
        mEstimatedEuclideanCamera1 = mEstimatedEuclideanCamera2 =
                mEstimatedEuclideanCamera3 = mEstimatedEuclideanCamera4 = null;
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
