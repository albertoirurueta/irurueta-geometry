/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.sfm.ConstantVelocityModelSlamTwoViewsSparseReconstructor
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date June 21, 2017.
 */
package com.irurueta.geometry.sfm;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.SingularValueDecomposer;
import com.irurueta.geometry.AxisRotation3D;
import com.irurueta.geometry.CameraException;
import com.irurueta.geometry.HomogeneousPoint3D;
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.MatrixRotation3D;
import com.irurueta.geometry.MetricTransformation3D;
import com.irurueta.geometry.NotAvailableException;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.Plane;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.Quaternion;
import com.irurueta.geometry.Rotation3D;
import com.irurueta.geometry.RotationException;
import com.irurueta.geometry.epipolar.FundamentalMatrix;
import com.irurueta.geometry.epipolar.InvalidPairOfCamerasException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.geometry.slam.ConstantVelocityModelSlamCalibrationData;
import com.irurueta.geometry.slam.ConstantVelocityModelSlamCalibrator;
import com.irurueta.geometry.slam.ConstantVelocityModelSlamEstimator;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.logging.Level;
import java.util.logging.Logger;
import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class ConstantVelocityModelSlamTwoViewsSparseReconstructorTest {
    
    public static final double MIN_RANDOM_VALUE = -100.0;
    public static final double MAX_RANDOM_VALUE = 100.0;

    public static final double MIN_FOCAL_LENGTH_ESSENTIAL = 750.0;
    public static final double MAX_FOCAL_LENGTH_ESSENTIAL = 1500.0;

    public static final double MIN_ANGLE_DEGREES = -30.0;
    public static final double MAX_ANGLE_DEGREES = -15.0;

    public static final double MIN_CAMERA_SEPARATION_ESSENTIAL = 500.0;
    public static final double MAX_CAMERA_SEPARATION_ESSENTIAL = 1000.0;

    public static final int MIN_NUM_POINTS = 25;
    public static final int MAX_NUM_POINTS = 50;

    public static final double MIN_LAMBDA_ESSENTIAL = -1000.0;
    public static final double MAX_LAMBDA_ESSENTIAL = 1000.0;

    public static final int TIMES = 500;
    public static final int MAX_TRIES = 5000;

    public static final double ABSOLUTE_ERROR = 1e-6;
    public static final double LARGE_ABSOLUTE_ERROR = 1e-3;

    //5% of relative error in scale estimation
    public static final double RELATIVE_ERROR = 0.1;

    public static final int MAX_CALIBRATION_SAMPLES = 10000;

    //conversion from milliseconds to nanoseconds
    public static final int MILLIS_TO_NANOS = 1000000;

    //time between samples expressed in nanoseconds (a typical sensor in Android 
    //delivers a sample every 20ms)
    public static final int DELTA_NANOS = 20000000; //0.02 seconds

    public static final float MIN_CALIBRATION_OFFSET = -1e-4f;
    public static final float MAX_CALIBRATION_OFFSET = 1e-4f;

    public static final double ACCELERATION_NOISE_STANDARD_DEVIATION = 1e-4;
    public static final double ANGULAR_SPEED_NOISE_STANDARD_DEVIATION = 1e-4;    

    public static final int N_SENSOR_SAMPLES = 50;
    
    public static final Logger LOGGER = Logger.getLogger(
            ConstantVelocityModelSlamTwoViewsSparseReconstructorTest.class.getSimpleName());

    private int mViewCount = 0;
    private EstimatedFundamentalMatrix mEstimatedFundamentalMatrix;
    private EstimatedCamera mEstimatedCamera1;
    private EstimatedCamera mEstimatedCamera2;
    private List<ReconstructedPoint3D> mReconstructedPoints;

    private boolean mStarted;
    private boolean mFinished;
    private boolean mFailed;
    private boolean mCancelled;

    private int mSlamDataAvailable;
    private int mSlamCameraEstimated;

    private PinholeCamera mSlamCamera;
    private Matrix mSlamCovariance;
    
    public ConstantVelocityModelSlamTwoViewsSparseReconstructorTest() { }
    
    @BeforeClass
    public static void setUpClass() { }
    
    @AfterClass
    public static void tearDownClass() { }
    
    @Before
    public void setUp() {
        mViewCount = 0;
        mEstimatedFundamentalMatrix = null;
        mEstimatedCamera1 = mEstimatedCamera2 = null;
        mReconstructedPoints = null;
        mStarted = mFinished = mFailed = mCancelled = false;
        mSlamDataAvailable = 0;
        mSlamCameraEstimated = 0;
        mSlamCamera = null;
        mSlamCovariance = null;
    }
    
    @After
    public void tearDown() { }

    @Test
    public void testConstructor() {
        ConstantVelocityModelSlamTwoViewsSparseReconstructorConfiguration configuration
                = new ConstantVelocityModelSlamTwoViewsSparseReconstructorConfiguration();
        ConstantVelocityModelSlamTwoViewsSparseReconstructorListener listener =
                new ConstantVelocityModelSlamTwoViewsSparseReconstructorListener() {
            @Override
            public void onSlamDataAvailable(ConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor,
                                            double positionX, double positionY, double positionZ,
                                            double velocityX, double velocityY, double velocityZ,
                                            double accelerationX, double accelerationY, double accelerationZ,
                                            double quaternionA, double quaternionB, double quaternionC, double quaternionD,
                                            double angularSpeedX, double angularSpeedY, double angularSpeedZ,
                                            Matrix covariance) { }

            @Override
            public void onSlamCameraEstimated(ConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor,
                                              PinholeCamera camera) { }

            @Override
            public boolean hasMoreViewsAvailable(
                    ConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor) {
                return false;
            }

            @Override
            public void onRequestSamplesForCurrentView(
                    ConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor, 
                    int viewId, List<Sample2D> samples) { }

            @Override
            public void onSamplesAccepted(
                    ConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor, 
                    int viewId, List<Sample2D> samples) { }

            @Override
            public void onSamplesRejected(
                    ConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor, 
                    int viewId, List<Sample2D> samples) { }

            @Override
            public void onRequestMatches(
                    ConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor, 
                    List<Sample2D> samples1, List<Sample2D> samples2, 
                    int viewId1, int viewId2, List<MatchedSamples> matches) { }

            @Override
            public void onFundamentalMatrixEstimated(
                    ConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor, 
                    EstimatedFundamentalMatrix estimatedFundamentalMatrix) { }

            @Override
            public void onCamerasEstimated(
                    ConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor, 
                    int viewId1, int viewId2, EstimatedCamera camera1, 
                    EstimatedCamera camera2) { }

            @Override
            public void onReconstructedPointsEstimated(
                    ConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor, 
                    List<MatchedSamples> matches, 
                    List<ReconstructedPoint3D> points) { }

            @Override
            public void onStart(
                    ConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor) { }

            @Override
            public void onFinish(
                    ConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor) { }

            @Override
            public void onCancel(
                    ConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor) { }

            @Override
            public void onFail(
                    ConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor) { }
        };
        
        //constructor with listener
        ConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor =
                new ConstantVelocityModelSlamTwoViewsSparseReconstructor(
                        listener);
        
        //check default values
        assertNotNull(reconstructor.getConfiguration());
        assertSame(reconstructor.getListener(), listener);
        assertFalse(reconstructor.isRunning());
        assertFalse(reconstructor.isCancelled());
        assertFalse(reconstructor.hasFailed());
        assertFalse(reconstructor.isFinished());
        assertEquals(reconstructor.getViewCount(), 0);
        assertNull(reconstructor.getEstimatedFundamentalMatrix());
        assertNull(reconstructor.getEstimatedCamera1());
        assertNull(reconstructor.getEstimatedCamera2());
        assertNull(reconstructor.getReconstructedPoints());

        //constructor with configuration and listener
        reconstructor = new ConstantVelocityModelSlamTwoViewsSparseReconstructor(
                configuration, listener);
        
        //check default values
        assertSame(reconstructor.getConfiguration(), configuration);
        assertSame(reconstructor.getListener(), listener);
        assertFalse(reconstructor.isRunning());
        assertFalse(reconstructor.isCancelled());
        assertFalse(reconstructor.hasFailed());
        assertEquals(reconstructor.getViewCount(), 0);
        assertNull(reconstructor.getEstimatedFundamentalMatrix());
        assertNull(reconstructor.getEstimatedCamera1());
        assertNull(reconstructor.getEstimatedCamera2());
        assertNull(reconstructor.getReconstructedPoints());        
    }
    
    @Test
    public void testGeneralPointsEssentialWithConstantAccelerationAndRotationWithoutNoise()
            throws InvalidPairOfCamerasException, AlgebraException,
            CameraException, RotationException, FailedReconstructionException,
            CancelledReconstructionException, NotReadyException,
            NotAvailableException {
        
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);
            
            ConstantVelocityModelSlamTwoViewsSparseReconstructorConfiguration configuration =
                    new ConstantVelocityModelSlamTwoViewsSparseReconstructorConfiguration();
            configuration.setInitialCamerasEstimatorMethod(
                    InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);
            
            float accelerationOffsetX = 0.0f;
            float accelerationOffsetY = 0.0f;
            float accelerationOffsetZ = 0.0f;

            float angularOffsetX = 0.0f;
            float angularOffsetY = 0.0f;
            float angularOffsetZ = 0.0f;

            ConstantVelocityModelSlamCalibrator calibrator = createFinishedCalibrator(
                    accelerationOffsetX, accelerationOffsetY,
                    accelerationOffsetZ, angularOffsetX, angularOffsetY,
                    angularOffsetZ, noiseRandomizer);
            ConstantVelocityModelSlamCalibrationData calibrationData
                    = calibrator.getCalibrationData();
            configuration.setCalibrationData(calibrationData);

            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double focalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH_ESSENTIAL,
                    MAX_FOCAL_LENGTH_ESSENTIAL);
            double aspectRatio = configuration.getInitialCamerasAspectRatio();
            double skewness = 0.0;
            double principalPoint = 0.0;

            PinholeCameraIntrinsicParameters intrinsic
                    = new PinholeCameraIntrinsicParameters(focalLength,
                            focalLength, principalPoint, principalPoint, skewness);
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

            final int numPoints = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);

            InhomogeneousPoint3D point3D;
            List<InhomogeneousPoint3D> points3D
                    = new ArrayList<InhomogeneousPoint3D>();
            Point2D projectedPoint1, projectedPoint2;
            final List<Point2D> projectedPoints1 = new ArrayList<Point2D>();
            final List<Point2D> projectedPoints2 = new ArrayList<Point2D>();
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
                } while (!front1 || !front2);
                points3D.add(point3D);

                //check that 3D point is in front of both cameras
                assertTrue(front1);
                assertTrue(front2);

                //project 3D point into both cameras
                projectedPoint1 = new InhomogeneousPoint2D();
                camera1.project(point3D, projectedPoint1);
                projectedPoints1.add(projectedPoint1);

                projectedPoint2 = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2);
                projectedPoints2.add(projectedPoint2);
            }

            ConstantVelocityModelSlamTwoViewsSparseReconstructorListener listener =
                    new ConstantVelocityModelSlamTwoViewsSparseReconstructorListener() {
                @Override
                public void onSlamDataAvailable(ConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor,
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
                public void onSlamCameraEstimated(ConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor,
                                                  PinholeCamera camera) {
                    mSlamCameraEstimated++;
                    mSlamCamera = camera;
                }

                @Override
                public boolean hasMoreViewsAvailable(
                        ConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor) {
                    return mViewCount < 2;
                }

                @Override
                public void onRequestSamplesForCurrentView(
                        ConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor, 
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

                        //assume the following accelerator and gyroscope samples
                        //are obtained during a period of 1 second between 1st
                        //and 2nd view (50 samples * 0.02 s/sample = 1 second)
                        long timestamp = 0;
                        for (int s = 0; s < N_SENSOR_SAMPLES; s++) {
                            reconstructor.updateAccelerometerSample(timestamp,
                                    (float) accelerationX, (float) accelerationY,
                                    (float) accelerationZ);
                            reconstructor.updateGyroscopeSample(timestamp,
                                    (float) angularSpeedX, (float) angularSpeedY,
                                    (float) angularSpeedZ);
                            timestamp += DELTA_NANOS;
                        }

                    } else {
                        //second view
                        for (int i = 0; i < numPoints; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints2.get(i));
                            sample.setViewId(viewId);
                            samples.add(sample);
                        }
                    }
                }

                @Override
                public void onSamplesAccepted(
                        ConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor, 
                        int viewId, List<Sample2D> samples) {
                    mViewCount++;
                }

                @Override
                public void onSamplesRejected(
                        ConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor, 
                        int viewId, List<Sample2D> samples) { }

                @Override
                public void onRequestMatches(
                        ConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor, 
                        List<Sample2D> samples1, List<Sample2D> samples2, 
                        int viewId1, int viewId2, List<MatchedSamples> matches) {
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
                        ConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor, 
                        EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                }

                @Override
                public void onCamerasEstimated(
                        ConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor, 
                        int viewId1, int viewId2, EstimatedCamera camera1, 
                        EstimatedCamera camera2) {
                    mEstimatedCamera1 = camera1;
                    mEstimatedCamera2 = camera2;
                }

                @Override
                public void onReconstructedPointsEstimated(
                        ConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor, 
                        List<MatchedSamples> matches, 
                        List<ReconstructedPoint3D> points) {
                    mReconstructedPoints = points;
                }

                @Override
                public void onStart(
                        ConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor) {
                    mStarted = true;
                }

                @Override
                public void onFinish(
                        ConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor) {
                    mFinished = true;
                }

                @Override
                public void onCancel(
                        ConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor) {
                    mCancelled = true;
                }

                @Override
                public void onFail(
                        ConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor) {
                    mFailed = true;
                }
            };
            
            ConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor =
                    new ConstantVelocityModelSlamTwoViewsSparseReconstructor(
                            configuration, listener);
            
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

            //check that estimated fundamental matrix is correct
            fundamentalMatrix.normalize();
            mEstimatedFundamentalMatrix.getFundamentalMatrix().normalize();

            //matrices are equal up to scale
            if (!fundamentalMatrix.getInternalMatrix().equals(
                    mEstimatedFundamentalMatrix.getFundamentalMatrix().
                    getInternalMatrix(), ABSOLUTE_ERROR)
                    && !fundamentalMatrix.getInternalMatrix().
                    multiplyByScalarAndReturnNew(-1).equals(
                    mEstimatedFundamentalMatrix.getFundamentalMatrix().
                    getInternalMatrix(), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundamentalMatrix.getInternalMatrix().equals(
                    mEstimatedFundamentalMatrix.getFundamentalMatrix().
                    getInternalMatrix(), ABSOLUTE_ERROR)
                    || fundamentalMatrix.getInternalMatrix().
                    multiplyByScalarAndReturnNew(-1).equals(
                    mEstimatedFundamentalMatrix.getFundamentalMatrix().
                    getInternalMatrix(), ABSOLUTE_ERROR));
            
            //check that reconstructed points are in a euclidean stratum (with            
            //correct scale)
            PinholeCamera estimatedCamera1 = mEstimatedCamera1.getCamera();
            PinholeCamera estimatedCamera2 = mEstimatedCamera2.getCamera();

            estimatedCamera1.decompose();
            estimatedCamera2.decompose();

            List<Point3D> reconstructedPoints3D = new ArrayList<Point3D>();
            for (int i = 0; i < numPoints; i++) {
                reconstructedPoints3D.add(
                        mReconstructedPoints.get(i).getPoint());
            }

            //check that all points are in front of both cameras
            for (int i = 0; i < numPoints; i++) {
                Point3D p = reconstructedPoints3D.get(i);
                assertTrue(estimatedCamera1.isPointInFrontOfCamera(p));
                assertTrue(estimatedCamera2.isPointInFrontOfCamera(p));
            }
            
            Point3D estimatedCenter1 = estimatedCamera1.getCameraCenter();
            Point3D estimatedCenter2 = estimatedCamera2.getCameraCenter();

            PinholeCameraIntrinsicParameters estimatedIntrinsic1
                    = estimatedCamera1.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters estimatedIntrinsic2
                    = estimatedCamera2.getIntrinsicParameters();

            Rotation3D estimatedRotation1
                    = estimatedCamera1.getCameraRotation();
            Rotation3D estimatedRotation2
                    = estimatedCamera2.getCameraRotation();

            double estimatedBaseline = estimatedCenter1.distanceTo(
                    estimatedCenter2);

            //check cameras are correct
            double maxBaseline = Math.max(estimatedBaseline, baseline);
            double minBaseline = Math.min(estimatedBaseline, baseline);
            double absoluteScaleError = RELATIVE_ERROR * maxBaseline;
            if (Math.abs(estimatedBaseline - baseline) > absoluteScaleError) {
                continue;
            }
            assertEquals(estimatedBaseline, baseline, absoluteScaleError);
            
            assertTrue(center1.equals(estimatedCenter1, ABSOLUTE_ERROR));
            if (!center2.equals(estimatedCenter2, absoluteScaleError)) {
                continue;
            }
            assertTrue(center2.equals(estimatedCenter2, absoluteScaleError));
            
            assertEquals(estimatedIntrinsic1.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertEquals(estimatedIntrinsic2.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertTrue(estimatedRotation1.asInhomogeneousMatrix().equals(
                    rotation1.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
            assertTrue(estimatedRotation2.asInhomogeneousMatrix().equals(
                    rotation2.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

            //check that points are correct (after scale correction)
            double scale = baseline / estimatedBaseline;
            MetricTransformation3D scaleTransformation
                    = new MetricTransformation3D(scale);

            boolean validPoints = true;
            for (int i = 0; i < numPoints; i++) {
                Point3D rescaledPoint = Point3D.create();
                scaleTransformation.transform(reconstructedPoints3D.get(i),
                        rescaledPoint);
                if (!points3D.get(i).equals(rescaledPoint, 
                        LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D.get(i).equals(rescaledPoint,
                        LARGE_ABSOLUTE_ERROR));
            }

            if (!validPoints) {
                continue;
            }
            
            double scaleRelativeError = 1.0 - minBaseline / maxBaseline; 
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
    public void testGeneralPointsEssentialWithConstantAccelerationAndRotationWithNoise()
            throws InvalidPairOfCamerasException, AlgebraException,
            CameraException, RotationException, FailedReconstructionException,
            CancelledReconstructionException, NotReadyException,
            NotAvailableException {
        
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer offsetRandomizer = new UniformRandomizer(
                    new Random());                        
            GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);
            
            ConstantVelocityModelSlamTwoViewsSparseReconstructorConfiguration configuration =
                    new ConstantVelocityModelSlamTwoViewsSparseReconstructorConfiguration();
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

            ConstantVelocityModelSlamCalibrator calibrator = createFinishedCalibrator(
                    accelerationOffsetX, accelerationOffsetY,
                    accelerationOffsetZ, angularOffsetX, angularOffsetY,
                    angularOffsetZ, noiseRandomizer);
            ConstantVelocityModelSlamCalibrationData calibrationData
                    = calibrator.getCalibrationData();
            configuration.setCalibrationData(calibrationData);

            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double focalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH_ESSENTIAL,
                    MAX_FOCAL_LENGTH_ESSENTIAL);
            double aspectRatio = configuration.getInitialCamerasAspectRatio();
            double skewness = 0.0;
            double principalPoint = 0.0;

            PinholeCameraIntrinsicParameters intrinsic
                    = new PinholeCameraIntrinsicParameters(focalLength,
                            focalLength, principalPoint, principalPoint, skewness);
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

            final int numPoints = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);

            InhomogeneousPoint3D point3D;
            List<InhomogeneousPoint3D> points3D
                    = new ArrayList<InhomogeneousPoint3D>();
            Point2D projectedPoint1, projectedPoint2;
            final List<Point2D> projectedPoints1 = new ArrayList<Point2D>();
            final List<Point2D> projectedPoints2 = new ArrayList<Point2D>();
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
                } while (!front1 || !front2);
                points3D.add(point3D);

                //check that 3D point is in front of both cameras
                assertTrue(front1);
                assertTrue(front2);

                //project 3D point into both cameras
                projectedPoint1 = new InhomogeneousPoint2D();
                camera1.project(point3D, projectedPoint1);
                projectedPoints1.add(projectedPoint1);

                projectedPoint2 = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2);
                projectedPoints2.add(projectedPoint2);
            }
            
            double accelerationNoiseStandardDeviation = 
                    ACCELERATION_NOISE_STANDARD_DEVIATION;
            double angularSpeedNoiseStandardDeviation = 
                    ANGULAR_SPEED_NOISE_STANDARD_DEVIATION;
            
            final GaussianRandomizer accelerationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0, 
                    accelerationNoiseStandardDeviation);
            final GaussianRandomizer angularSpeedRandomizer =
                    new GaussianRandomizer(new Random(), 0.0, 
                    angularSpeedNoiseStandardDeviation);            

            ConstantVelocityModelSlamTwoViewsSparseReconstructorListener listener =
                    new ConstantVelocityModelSlamTwoViewsSparseReconstructorListener() {
                @Override
                public void onSlamDataAvailable(ConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor,
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
                public void onSlamCameraEstimated(ConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor,
                                                  PinholeCamera camera) {
                    mSlamCameraEstimated++;
                    mSlamCamera = camera;
                }

                @Override
                public boolean hasMoreViewsAvailable(
                        ConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor) {
                    return mViewCount < 2;
                }

                @Override
                public void onRequestSamplesForCurrentView(
                        ConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor, 
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

                        //assume the following accelerator and gyroscope samples
                        //are obtained during a period of 1 second between 1st
                        //and 2nd view (50 samples * 0.02 s/sample = 1 second)
                        long timestamp = 0;
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
                            
                            reconstructor.updateAccelerometerSample(timestamp,
                                    accelerationWithNoise);
                            reconstructor.updateGyroscopeSample(timestamp,
                                    angularSpeedWithNoise);
                            timestamp += DELTA_NANOS;
                        }

                    } else {
                        //second view
                        for (int i = 0; i < numPoints; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints2.get(i));
                            sample.setViewId(viewId);
                            samples.add(sample);
                        }
                    }
                }

                @Override
                public void onSamplesAccepted(
                        ConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor, 
                        int viewId, List<Sample2D> samples) {
                    mViewCount++;
                }

                @Override
                public void onSamplesRejected(
                        ConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor, 
                        int viewId, List<Sample2D> samples) { }

                @Override
                public void onRequestMatches(
                        ConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor, 
                        List<Sample2D> samples1, List<Sample2D> samples2, 
                        int viewId1, int viewId2, List<MatchedSamples> matches) {
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
                        ConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor, 
                        EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                }

                @Override
                public void onCamerasEstimated(
                        ConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor, 
                        int viewId1, int viewId2, EstimatedCamera camera1, 
                        EstimatedCamera camera2) {
                    mEstimatedCamera1 = camera1;
                    mEstimatedCamera2 = camera2;
                }

                @Override
                public void onReconstructedPointsEstimated(
                        ConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor, 
                        List<MatchedSamples> matches, 
                        List<ReconstructedPoint3D> points) {
                    mReconstructedPoints = points;
                }

                @Override
                public void onStart(
                        ConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor) {
                    mStarted = true;
                }

                @Override
                public void onFinish(
                        ConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor) {
                    mFinished = true;
                }

                @Override
                public void onCancel(
                        ConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor) {
                    mCancelled = true;
                }

                @Override
                public void onFail(
                        ConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor) {
                    mFailed = true;
                }
            };
            
            ConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor =
                    new ConstantVelocityModelSlamTwoViewsSparseReconstructor(
                            configuration, listener);
            
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

            //check that estimated fundamental matrix is correct
            fundamentalMatrix.normalize();
            mEstimatedFundamentalMatrix.getFundamentalMatrix().normalize();

            //matrices are equal up to scale
            if (!fundamentalMatrix.getInternalMatrix().equals(
                    mEstimatedFundamentalMatrix.getFundamentalMatrix().
                    getInternalMatrix(), ABSOLUTE_ERROR)
                    && !fundamentalMatrix.getInternalMatrix().
                    multiplyByScalarAndReturnNew(-1).equals(
                    mEstimatedFundamentalMatrix.getFundamentalMatrix().
                    getInternalMatrix(), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundamentalMatrix.getInternalMatrix().equals(
                    mEstimatedFundamentalMatrix.getFundamentalMatrix().
                    getInternalMatrix(), ABSOLUTE_ERROR)
                    || fundamentalMatrix.getInternalMatrix().
                    multiplyByScalarAndReturnNew(-1).equals(
                    mEstimatedFundamentalMatrix.getFundamentalMatrix().
                    getInternalMatrix(), ABSOLUTE_ERROR));
            
            //check that reconstructed points are in a euclidean stratum (with            
            //correct scale)
            PinholeCamera estimatedCamera1 = mEstimatedCamera1.getCamera();
            PinholeCamera estimatedCamera2 = mEstimatedCamera2.getCamera();

            estimatedCamera1.decompose();
            estimatedCamera2.decompose();

            List<Point3D> reconstructedPoints3D = new ArrayList<Point3D>();
            for (int i = 0; i < numPoints; i++) {
                reconstructedPoints3D.add(
                        mReconstructedPoints.get(i).getPoint());
            }

            //check that all points are in front of both cameras
            for (int i = 0; i < numPoints; i++) {
                Point3D p = reconstructedPoints3D.get(i);
                assertTrue(estimatedCamera1.isPointInFrontOfCamera(p));
                assertTrue(estimatedCamera2.isPointInFrontOfCamera(p));
            }
            
            Point3D estimatedCenter1 = estimatedCamera1.getCameraCenter();
            Point3D estimatedCenter2 = estimatedCamera2.getCameraCenter();

            PinholeCameraIntrinsicParameters estimatedIntrinsic1
                    = estimatedCamera1.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters estimatedIntrinsic2
                    = estimatedCamera2.getIntrinsicParameters();

            Rotation3D estimatedRotation1
                    = estimatedCamera1.getCameraRotation();
            Rotation3D estimatedRotation2
                    = estimatedCamera2.getCameraRotation();

            double estimatedBaseline = estimatedCenter1.distanceTo(
                    estimatedCenter2);

            //check cameras are correct
            double maxBaseline = Math.max(estimatedBaseline, baseline);
            double minBaseline = Math.min(estimatedBaseline, baseline);
            double absoluteScaleError = RELATIVE_ERROR * maxBaseline;
            if (Math.abs(estimatedBaseline - baseline) > absoluteScaleError) {
                continue;
            }
            assertEquals(estimatedBaseline, baseline, absoluteScaleError);
            
            assertTrue(center1.equals(estimatedCenter1, ABSOLUTE_ERROR));
            if (!center2.equals(estimatedCenter2, absoluteScaleError)) {
                continue;
            }
            assertTrue(center2.equals(estimatedCenter2, absoluteScaleError));
            
            assertEquals(estimatedIntrinsic1.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertEquals(estimatedIntrinsic2.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertTrue(estimatedRotation1.asInhomogeneousMatrix().equals(
                    rotation1.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
            assertTrue(estimatedRotation2.asInhomogeneousMatrix().equals(
                    rotation2.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

            //check that points are correct (after scale correction)
            double scale = baseline / estimatedBaseline;
            MetricTransformation3D scaleTransformation
                    = new MetricTransformation3D(scale);

            boolean validPoints = true;
            for (int i = 0; i < numPoints; i++) {
                Point3D rescaledPoint = Point3D.create();
                scaleTransformation.transform(reconstructedPoints3D.get(i),
                        rescaledPoint);
                if (!points3D.get(i).equals(rescaledPoint, 
                        LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D.get(i).equals(rescaledPoint,
                        LARGE_ABSOLUTE_ERROR));
            }

            if (!validPoints) {
                continue;
            }
            
            double scaleRelativeError = 1.0 - minBaseline / maxBaseline; 
            LOGGER.log(Level.INFO, 
                    "Baseline relative error with noise: {0,number,0.000%}",
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
        mEstimatedFundamentalMatrix = null;
        mEstimatedCamera1 = mEstimatedCamera2 = null;
        mReconstructedPoints = null;
        mStarted = mFinished = mCancelled = mFailed = false;
        mSlamDataAvailable = 0;
        mSlamCameraEstimated = 0;
        mSlamCamera = null;
        mSlamCovariance = null;
    }
    
    private ConstantVelocityModelSlamCalibrator createFinishedCalibrator(
            float accelerationOffsetX, float accelerationOffsetY, 
            float accelerationOffsetZ, float angularOffsetX, 
            float angularOffsetY, float angularOffsetZ,
            GaussianRandomizer noiseRandomizer) {
        ConstantVelocityModelSlamCalibrator calibrator = 
                ConstantVelocityModelSlamEstimator.createCalibrator();
        calibrator.setConvergenceThreshold(ABSOLUTE_ERROR);
        calibrator.setMaxNumSamples(MAX_CALIBRATION_SAMPLES);
        
        long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
        
        float accelerationNoiseX, accelerationNoiseY, accelerationNoiseZ;
        float angularNoiseX, angularNoiseY, angularNoiseZ;
        
        double accelerationX, accelerationY, accelerationZ;
        double angularX, angularY, angularZ;

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
            
            if(calibrator.isFinished()) break;
            
            timestamp += DELTA_NANOS;
        }
        
        return calibrator;
    }        
    
}
