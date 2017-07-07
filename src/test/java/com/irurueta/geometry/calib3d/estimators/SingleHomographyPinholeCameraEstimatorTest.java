/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.calib3d.estimators.SingleHomographyPinholeCameraEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 23, 2017.
 */
package com.irurueta.geometry.calib3d.estimators;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.ArrayUtils;
import com.irurueta.algebra.DecomposerException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.NotAvailableException;
import com.irurueta.algebra.NotReadyException;
import com.irurueta.algebra.SingularValueDecomposer;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.CameraException;
import com.irurueta.geometry.GeometryException;
import com.irurueta.geometry.HomogeneousPoint2D;
import com.irurueta.geometry.HomogeneousPoint3D;
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.MatrixRotation3D;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.Plane;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.ProjectiveTransformation2D;
import com.irurueta.geometry.Rotation3D;
import com.irurueta.geometry.Transformation2D;
import com.irurueta.geometry.epipolar.FundamentalMatrix;
import com.irurueta.geometry.epipolar.InvalidPairOfCamerasException;
import com.irurueta.geometry.epipolar.refiners.HomogeneousRightEpipoleRefiner;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.ProjectiveTransformation2DRobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.UniformRandomizer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.BitSet;
import java.util.List;
import java.util.Random;
import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class SingleHomographyPinholeCameraEstimatorTest implements 
        SingleHomographyPinholeCameraEstimatorListener {
    
    public static final double MIN_RANDOM_VALUE = -1500.0;
    public static final double MAX_RANDOM_VALUE = 1500.0;
    
    public static final double MIN_FOCAL_LENGTH = 750.0;
    public static final double MAX_FOCAL_LENGTH = 1500.0;
    
    public static final double MIN_ANGLE_DEGREES = -30.0;
    public static final double MAX_ANGLE_DEGREES = -15.0;
    
    public static final double MIN_CAMERA_SEPARATION = 500.0;
    public static final double MAX_CAMERA_SEPARATION = 1000.0;
    
    public static final int MIN_NUM_POINTS = 25;
    public static final int MAX_NUM_POINTS = 50;
    
    public static final int TIMES = 50;
    public static final int MAX_TRIES = 5000;
    
    public static final double ABSOLUTE_ERROR = 1e-6;
    public static final double LARGE_ABSOLUTE_ERROR = 1e-3;
    
    private int estimateStart;
    private int estimateEnd;
    
    public SingleHomographyPinholeCameraEstimatorTest() { }
    
    @BeforeClass
    public static void setUpClass() { }
    
    @AfterClass
    public static void tearDownClass() { }
    
    @Before
    public void setUp() { }
    
    @After
    public void tearDown() { }
    
    @Test
    public void testConstructor() {
        //test empty constructor
        SingleHomographyPinholeCameraEstimator estimator = 
                new SingleHomographyPinholeCameraEstimator();
        
        //check default value
        assertEquals(estimator.getFocalDistanceAspectRatio(), 
                SingleHomographyPinholeCameraEstimator.DEFAULT_ASPECT_RATIO, 
                0.0);
        assertNull(estimator.getHomography());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        
        
        //test constructor with listener
        estimator = new SingleHomographyPinholeCameraEstimator(this);
        
        //check default value
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                SingleHomographyPinholeCameraEstimator.DEFAULT_ASPECT_RATIO, 
                0.0);
        assertNull(estimator.getHomography());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), this);
        
        //test constructor with homography
        ProjectiveTransformation2D homography = 
                new ProjectiveTransformation2D();
        estimator = new SingleHomographyPinholeCameraEstimator(homography);
        
        //check default value
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                SingleHomographyPinholeCameraEstimator.DEFAULT_ASPECT_RATIO, 
                0.0);
        assertSame(estimator.getHomography(), homography);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        
        //test constructor with homography and listener
        estimator = new SingleHomographyPinholeCameraEstimator(homography, 
                this);
        
        //check default value
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                SingleHomographyPinholeCameraEstimator.DEFAULT_ASPECT_RATIO, 
                0.0);
        assertSame(estimator.getHomography(), homography);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), this);
    }
    
    @Test
    public void testGetSetFocalDistanceAspectRatio() throws LockedException {
        SingleHomographyPinholeCameraEstimator estimator =
                new SingleHomographyPinholeCameraEstimator();
        
        //check default value
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                SingleHomographyPinholeCameraEstimator.DEFAULT_ASPECT_RATIO,
                0.0);
        
        //set new value
        estimator.setFocalDistanceAspectRatio(
                -SingleHomographyPinholeCameraEstimator.DEFAULT_ASPECT_RATIO);
        
        //check correctness
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                -SingleHomographyPinholeCameraEstimator.DEFAULT_ASPECT_RATIO, 
                0.0);
    }
    
    @Test
    public void testGetSetHomographyAndIsReady() throws LockedException {
        SingleHomographyPinholeCameraEstimator estimator = 
                new SingleHomographyPinholeCameraEstimator();
        
        //check default value
        assertNull(estimator.getHomography());
        assertFalse(estimator.isReady());
        
        //set new value
        ProjectiveTransformation2D homography = 
                new ProjectiveTransformation2D();
        estimator.setHomography(homography);
        
        //check correctness
        assertSame(estimator.getHomography(), homography);
        assertTrue(estimator.isReady());
    }
    
    @Test
    public void testGetSetListener() throws LockedException {
        SingleHomographyPinholeCameraEstimator estimator =
                new SingleHomographyPinholeCameraEstimator();
        
        //check default value
        assertNull(estimator.getListener());
        
        //set new value
        estimator.setListener(this);
        
        //check correctness
        assertSame(estimator.getListener(), this);
    }
    
    @Test
    public void testEstimate() throws InvalidPairOfCamerasException, 
            WrongSizeException, NotReadyException, 
            com.irurueta.algebra.LockedException, DecomposerException, NotAvailableException, CameraException, LockedException, com.irurueta.geometry.estimators.NotReadyException, RobustEstimatorException, SingleHomographyPinholeCameraEstimatorException, com.irurueta.geometry.NotAvailableException, AlgebraException, GeometryException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, 
                    MAX_FOCAL_LENGTH);
            double aspectRatio = 1.0;
            double skewness = 0.0;
            double principalPoint = 0.0;
            
            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength, 
                    focalLength, principalPoint, principalPoint, skewness);
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
            
            double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);
            
            Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);
            
            MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1,
                    betaEuler1, gammaEuler1);
            MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2,
                    betaEuler2, gammaEuler2);
            
            PinholeCamera camera1 = new PinholeCamera(intrinsic, rotation1, 
                    center1);
            PinholeCamera camera2 = new PinholeCamera(intrinsic, rotation2,
                    center2);
            
            FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(camera1, 
                    camera2);
            
            //create 3D points laying in front of both cameras and in a plane
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
            
            double[] principalAxis1 = camera1.getPrincipalAxisArray();
            double[] principalAxis2 = camera2.getPrincipalAxisArray();
            double[] avgPrincipalAxis = ArrayUtils.multiplyByScalarAndReturnNew(
                    ArrayUtils.sumAndReturnNew(principalAxis1, principalAxis2), 
                    0.5);
            
            Plane plane = new Plane(centralCommonPoint, avgPrincipalAxis);
            plane.normalize();
            
            double planeA = plane.getA();
            double planeB = plane.getB();
            double planeC = plane.getC();
            double planeD = plane.getD();
            
            final int numPoints = randomizer.nextInt(MIN_NUM_POINTS, 
                    MAX_NUM_POINTS);
            
            HomogeneousPoint3D point3D;
            Point2D projectedPoint1, projectedPoint2;
            final List<Point2D> projectedPoints1 = new ArrayList<Point2D>();
            final List<Point2D> projectedPoints2 = new ArrayList<Point2D>();
            boolean front1, front2;
            for (int i = 0; i < numPoints; i++) {
                //generate points and ensure they lie in front of both cameras
                int numTry = 0;
                do {
                    //get a random point belonging to the plane
                    //a*x + b*y + c*z + d*w = 0
                    //y = -(a*x + c*z + d*w)/b or x = -(b*y + c*z + d*w)/a
                    double homX, homY;
                    double homW = 1.0;
                    double homZ = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE);
                    if (Math.abs(planeB) > ABSOLUTE_ERROR) {
                        homX = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                                MAX_RANDOM_VALUE);
                        homY = -(planeA * homX + planeC * homZ + planeD * homW) / 
                                planeB;
                    } else {
                        homY = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                                MAX_RANDOM_VALUE);
                        homX = -(planeB * homY + planeC * homZ + planeD * homW) / 
                                planeA;
                    }
                    
                    point3D = new HomogeneousPoint3D(homX, homY, homZ, homW);
                    
                    assertTrue(plane.isLocus(point3D));
                    
                    front1 = camera1.isPointInFrontOfCamera(point3D);
                    front2 = camera2.isPointInFrontOfCamera(point3D);
                    if (numTry > MAX_TRIES) {
                        fail("max tries reached");
                    }
                    numTry++;
                } while(!front1 || !front2);
                
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
            
            //estimate homography
            ProjectiveTransformation2DRobustEstimator homographyEstimator =
                    ProjectiveTransformation2DRobustEstimator.createFromPoints(
                    projectedPoints1, projectedPoints2, 
                    RobustEstimatorMethod.LMedS);            
            
            Transformation2D homography;
            try {
                homography = homographyEstimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }
                        
            SingleHomographyPinholeCameraEstimator estimator = 
                    new SingleHomographyPinholeCameraEstimator(homography, 
                    this);
            
            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            
            PinholeCamera camera;
            try {
                camera = estimator.estimate();
            } catch(SingleHomographyPinholeCameraEstimatorException e) {
                continue;
            }
            
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            
            camera.decompose();
            
            PinholeCameraIntrinsicParameters estimatedIntrinsic = 
                    camera.getIntrinsicParameters();
            Rotation3D estimatedRotation = camera.getCameraRotation();
            Point3D estimatedCenter = camera.getCameraCenter();
                        
            
            PinholeCamera estimatedCamera1 = new PinholeCamera(
                    estimatedIntrinsic, new MatrixRotation3D(), 
                    new InhomogeneousPoint3D());
            PinholeCamera estimatedCamera2 = new PinholeCamera(
                    estimatedIntrinsic, estimatedRotation, estimatedCenter);
            
            FundamentalMatrix estimatedFundamentalMatrix = 
                    new FundamentalMatrix(estimatedCamera1, 
                    estimatedCamera2);
            
            fundamentalMatrix.normalize();
            estimatedFundamentalMatrix.normalize();
            
            BitSet inliers = new BitSet();
            inliers.set(0, numPoints);
            double[] residuals = new double[numPoints];
            Arrays.fill(residuals, 1.0);            
                        
            estimatedFundamentalMatrix.computeEpipoles();
            Point2D estimatedEpipoleRight = 
                    estimatedFundamentalMatrix.getRightEpipole();
            HomogeneousRightEpipoleRefiner refiner = new HomogeneousRightEpipoleRefiner(
                    estimatedEpipoleRight, true, inliers, residuals, numPoints,
                    projectedPoints1, projectedPoints2, 1.0, homography);
            
            HomogeneousPoint2D refinedEpipole = new HomogeneousPoint2D();
            refiner.refine(refinedEpipole);
            
            FundamentalMatrix refinedFundamentalMatrix = 
                    new FundamentalMatrix();
            HomogeneousRightEpipoleRefiner.computeFundamentalMatrix(homography, 
                    refinedEpipole, refinedFundamentalMatrix);
            
            refinedFundamentalMatrix.normalize();            
            
            
            if (!fundamentalMatrix.getInternalMatrix().equals(
                    refinedFundamentalMatrix.getInternalMatrix(), 
                    LARGE_ABSOLUTE_ERROR) && !fundamentalMatrix.getInternalMatrix().
                    multiplyByScalarAndReturnNew(-1.0).equals(
                    refinedFundamentalMatrix.getInternalMatrix(), 
                    LARGE_ABSOLUTE_ERROR)) {
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
        estimateStart = estimateEnd = 0;
    }

    @Override
    public void onEstimateStart(
            SingleHomographyPinholeCameraEstimator estimator) {
        estimateStart++;
        checkLocked(estimator);
    }

    @Override
    public void onEstimateEnd(
            SingleHomographyPinholeCameraEstimator estimator) {
        estimateEnd++;
        checkLocked(estimator);
    }
    
    private void checkLocked(SingleHomographyPinholeCameraEstimator estimator) {
        try {
            estimator.setFocalDistanceAspectRatio(1.0);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setHomography(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setListener(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.estimate();
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { 
        } catch (Exception e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.estimate(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { 
        } catch (Exception e) {
            fail("LockedException expected but not thrown");
        }
        assertTrue(estimator.isLocked());
    }
}
