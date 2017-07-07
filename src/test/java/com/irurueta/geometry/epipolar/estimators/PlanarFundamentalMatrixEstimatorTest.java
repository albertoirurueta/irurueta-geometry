/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.epipolar.estimators.PlanarFundamentalMatrixEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date May 20, 2017.
 */
package com.irurueta.geometry.epipolar.estimators;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.ArrayUtils;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.SingularValueDecomposer;
import com.irurueta.geometry.CameraException;
import com.irurueta.geometry.HomogeneousPoint2D;
import com.irurueta.geometry.HomogeneousPoint3D;
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.MatrixRotation3D;
import com.irurueta.geometry.NotAvailableException;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.Plane;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.ProjectiveTransformation2D;
import com.irurueta.geometry.epipolar.FundamentalMatrix;
import com.irurueta.geometry.epipolar.InvalidPairOfCamerasException;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.geometry.estimators.ProjectiveTransformation2DRobustEstimator;
import com.irurueta.geometry.estimators.RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.UniformRandomizer;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class PlanarFundamentalMatrixEstimatorTest implements 
        PlanarFundamentalMatrixEstimatorListener {
    
    public static final double MIN_FOCAL_LENGTH = 750.0;
    public static final double MAX_FOCAL_LENGTH = 1500.0;
    
    public static final double MIN_ANGLE_DEGREES = -30.0;
    public static final double MAX_ANGLE_DEGREES = -15.0;

    public static final double MIN_CAMERA_SEPARATION = 500.0;
    public static final double MAX_CAMERA_SEPARATION = 1000.0;   
    
    public static final int MIN_NUM_POINTS = 25;
    public static final int MAX_NUM_POINTS = 50;
    
    public static final double MIN_RANDOM_VALUE = -100.0;
    public static final double MAX_RANDOM_VALUE = 100.0;  
    
    public static final double MIN_RANDOM_VALUE_PLANAR = -1500.0;
    public static final double MAX_RANDOM_VALUE_PLANAR = 1500.0;    
    
    public static final int TIMES = 500;
    public static final int MAX_TRIES = 5000;
    
    public static final double HOMOGRAPHY_ESTIMATOR_THRESHOLD = 1e-8;
    
    public static final double ABSOLUTE_ERROR = 1e-6;
    public static final double LARGE_ABSOLUTE_ERROR = 1e-5;
    public static final double VERY_LARGE_ABSOLUTE_ERROR = 1e-3;
    
    private int mEstimateStart;
    private int mEstimateEnd;
    
    public PlanarFundamentalMatrixEstimatorTest() { }
    
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
        //empty constructor
        PlanarFundamentalMatrixEstimator estimator = 
                new PlanarFundamentalMatrixEstimator();
        
        //check default values
        assertNull(estimator.getHomography());
        assertNull(estimator.getLeftIntrinsics());
        assertNull(estimator.getRightIntrinsics());
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        
        
        //constructor with input data
        ProjectiveTransformation2D homography = 
                new ProjectiveTransformation2D();
        PinholeCameraIntrinsicParameters leftIntrinsics = 
                new PinholeCameraIntrinsicParameters();
        PinholeCameraIntrinsicParameters rightIntrinsics =
                new PinholeCameraIntrinsicParameters();
        estimator = new PlanarFundamentalMatrixEstimator(homography, 
                leftIntrinsics, rightIntrinsics);
        
        //check correctness
        assertSame(estimator.getHomography(), homography);
        assertSame(estimator.getLeftIntrinsics(), leftIntrinsics);
        assertSame(estimator.getRightIntrinsics(), rightIntrinsics);
        assertNull(estimator.getListener());
        
        
        //constructor with input data and listener
        estimator = new PlanarFundamentalMatrixEstimator(homography, 
                leftIntrinsics, rightIntrinsics, this);
        
        //check correctness
        assertSame(estimator.getHomography(), homography);
        assertSame(estimator.getLeftIntrinsics(), leftIntrinsics);
        assertSame(estimator.getRightIntrinsics(), rightIntrinsics);
        assertSame(estimator.getListener(), this);        
    }
    
    @Test
    public void testGetSetHomography() throws LockedException {
        PlanarFundamentalMatrixEstimator estimator = 
                new PlanarFundamentalMatrixEstimator();

        //check default value
        assertNull(estimator.getHomography());
        
        //set new value
        ProjectiveTransformation2D homography = 
                new ProjectiveTransformation2D();
        estimator.setHomography(homography);
        
        //check correctness
        assertSame(estimator.getHomography(), homography);
    }
    
    @Test
    public void testGetSetLeftIntrinsics() throws LockedException {
        PlanarFundamentalMatrixEstimator estimator = 
                new PlanarFundamentalMatrixEstimator();
        
        //check default value
        assertNull(estimator.getLeftIntrinsics());
        
        //set new value
        PinholeCameraIntrinsicParameters leftIntrinsics =
                new PinholeCameraIntrinsicParameters();
        estimator.setLeftIntrinsics(leftIntrinsics);
        
        //check correctness
        assertSame(estimator.getLeftIntrinsics(), leftIntrinsics);
    }
    
    @Test
    public void testGetSetRightIntrinsics() throws LockedException {
        PlanarFundamentalMatrixEstimator estimator =
                new PlanarFundamentalMatrixEstimator();
        
        //check default value
        assertNull(estimator.getRightIntrinsics());
        
        //set new value
        PinholeCameraIntrinsicParameters rightIntrinsics =
                new PinholeCameraIntrinsicParameters();
        estimator.setRightIntrinsics(rightIntrinsics);
        
        //check correctness
        assertSame(estimator.getRightIntrinsics(), rightIntrinsics);
    }
    
    @Test
    public void testGetSetListener() {
        PlanarFundamentalMatrixEstimator estimator = 
                new PlanarFundamentalMatrixEstimator();
        
        //check default value
        assertNull(estimator.getListener());
        
        //set new value
        estimator.setListener(this);
        
        //check correctness
        assertSame(estimator.getListener(), this);
    }
 
    @Test
    public void testEstimate() throws InvalidPairOfCamerasException, 
            AlgebraException, CameraException, LockedException, 
            NotReadyException, FundamentalMatrixEstimatorException, 
            NotAvailableException  {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double focalLength1 = randomizer.nextDouble(MIN_FOCAL_LENGTH, 
                    MAX_FOCAL_LENGTH);
            double focalLength2 = randomizer.nextDouble(MIN_FOCAL_LENGTH, 
                    MAX_FOCAL_LENGTH);
            double skewness = 0.0;
            double principalPointX = 0.0;
            double principalPointY = 0.0;
            
            PinholeCameraIntrinsicParameters intrinsic1 =
                    new PinholeCameraIntrinsicParameters(focalLength1, 
                    focalLength1, principalPointX, principalPointY, skewness);
            PinholeCameraIntrinsicParameters intrinsic2 =
                    new PinholeCameraIntrinsicParameters(focalLength2, 
                    focalLength2, principalPointX, principalPointY, skewness);
            
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
                    MIN_CAMERA_SEPARATION, 
                    MAX_CAMERA_SEPARATION);
        
            Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);
        
            MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1, 
                    betaEuler1, gammaEuler1);
            MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2, 
                    betaEuler2, gammaEuler2);
        
            PinholeCamera camera1 = new PinholeCamera(intrinsic1, rotation1, 
                    center1);
            PinholeCamera camera2 = new PinholeCamera(intrinsic2, rotation2,
                    center2);
        
            FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(
                    camera1, camera2);
            
            //create 3D points laying in front of both cameras and laying in
            //a plane
        
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
                        homX = randomizer.nextDouble(MIN_RANDOM_VALUE_PLANAR, 
                                MAX_RANDOM_VALUE_PLANAR);
                        homY = -(planeA * homX + planeC * homZ + planeD * homW) / 
                                planeB;
                    } else {
                        homY = randomizer.nextDouble(MIN_RANDOM_VALUE_PLANAR, 
                                MAX_RANDOM_VALUE_PLANAR);
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
                projectedPoint1 = new HomogeneousPoint2D();
                camera1.project(point3D, projectedPoint1);
                projectedPoint1.normalize();
                
                projectedPoints1.add(projectedPoint1);
            
                projectedPoint2 = new HomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2);
                projectedPoint2.normalize();
                
                projectedPoints2.add(projectedPoint2);
            }
            
            //estimate homography
            ProjectiveTransformation2DRobustEstimator homographyEstimator =
                    ProjectiveTransformation2DRobustEstimator.createFromPoints(
                    projectedPoints1, projectedPoints2, 
                    RobustEstimatorMethod.RANSAC);
            ((RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator)
                    homographyEstimator).setThreshold(
                            HOMOGRAPHY_ESTIMATOR_THRESHOLD);
            ((RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator)
                    homographyEstimator).setComputeAndKeepInliersEnabled(true);
            ((RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator)
                    homographyEstimator).setComputeAndKeepResidualsEnabled(true);            
            homographyEstimator.setResultRefined(false);
            ProjectiveTransformation2D homography;
            try {
                homography = homographyEstimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }
            
            homography.normalize();
            
            //check errors on estimated homography
            boolean failed = false;
            for (int i = 0; i < numPoints; i++) {
                projectedPoint1 = new InhomogeneousPoint2D(
                        projectedPoints1.get(i));
                projectedPoint2 = new InhomogeneousPoint2D(
                        projectedPoints2.get(i));
                
                InhomogeneousPoint2D projectedPoint2b = 
                        new InhomogeneousPoint2D();
                homography.transform(projectedPoint1, projectedPoint2b);
                
                if (!projectedPoint2.equals(projectedPoint2b, 
                        LARGE_ABSOLUTE_ERROR)) {
                    failed = true;
                    break;
                }
                assertTrue(projectedPoint2.equals(projectedPoint2b, 
                        LARGE_ABSOLUTE_ERROR));
            }
            
            if (failed) {
                continue;
            }
            
            PlanarFundamentalMatrixEstimator estimator = 
                    new PlanarFundamentalMatrixEstimator(homography, intrinsic1, 
                   intrinsic2, this);
            
            reset();
            assertEquals(mEstimateStart, 0);
            assertEquals(mEstimateEnd, 0);
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            
            List<FundamentalMatrix> result1 = estimator.estimate();
            List<FundamentalMatrix> result2 = 
                    new ArrayList<FundamentalMatrix>();
            estimator.estimate(result2);
            
            
            //check correctness
            assertEquals(mEstimateStart, 2);
            assertEquals(mEstimateEnd, 2);
            
            assertEquals(result1.size(), result2.size());
            assertTrue(result1.size() == 2 || result1.size() == 4);
            
            boolean atLeastOneValidFundamentalMatrix = false;
            for (int i = 0; i < result1.size(); i++) {
                
                assertTrue(areEqualUpToScale(result1.get(i), result2.get(i), 
                        ABSOLUTE_ERROR));
                
                if(areEqualUpToScale(fundamentalMatrix, result1.get(i), 
                        ABSOLUTE_ERROR)) {
                    atLeastOneValidFundamentalMatrix = true;
                }
            }
            
            if (!atLeastOneValidFundamentalMatrix) {
                continue;
            }
            
            numValid++;       
            
            if (numValid > 0) {
                break;
            }
        }   
        
        assertTrue(numValid > 0);                
    }       

    private static boolean areEqualUpToScale(
            FundamentalMatrix fundamentalMatrix1,
            FundamentalMatrix fundamentalMatrix2, double threshold) 
            throws NotAvailableException, NotReadyException {
        
        //normalize to increase accuracy
        fundamentalMatrix1.normalize();
        fundamentalMatrix2.normalize();
        
        Matrix f1 = fundamentalMatrix1.getInternalMatrix();
        Matrix f2a = fundamentalMatrix2.getInternalMatrix();
        Matrix f2b = f2a.multiplyByScalarAndReturnNew(-1.0);
        
        return f1.equals(f2a, threshold) || f1.equals(f2b, threshold);
    }
    
    private void reset() {
        mEstimateStart = mEstimateEnd = 0;
    }

    @Override
    public void onEstimateStart(PlanarFundamentalMatrixEstimator estimator) {
        mEstimateStart++;
        checkLocked(estimator);
    }

    @Override
    public void onEstimateEnd(PlanarFundamentalMatrixEstimator estimator, 
            List<FundamentalMatrix> fundamentalMatrices) {
        mEstimateEnd++;
        checkLocked(estimator);
    }
    
    private void checkLocked(PlanarFundamentalMatrixEstimator estimator) {
        assertTrue(estimator.isLocked());
        try {
            estimator.setHomography(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setLeftIntrinsics(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setRightIntrinsics(null);
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
    }
}
