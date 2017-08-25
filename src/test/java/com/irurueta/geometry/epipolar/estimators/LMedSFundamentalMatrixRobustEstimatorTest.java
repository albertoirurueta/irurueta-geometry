/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.epipolar.estimators.LMedSFundamentalMatrixRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 23, 2015
 */
package com.irurueta.geometry.epipolar.estimators;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.HomogeneousPoint2D;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Line2D;
import com.irurueta.geometry.MatrixRotation3D;
import com.irurueta.geometry.NotAvailableException;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.Plane;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.Rotation3D;
import com.irurueta.geometry.epipolar.FundamentalMatrix;
import com.irurueta.geometry.epipolar.InvalidFundamentalMatrixException;
import com.irurueta.geometry.epipolar.InvalidPairOfCamerasException;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
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

public class LMedSFundamentalMatrixRobustEstimatorTest implements 
        FundamentalMatrixRobustEstimatorListener {
    
    public static final int MIN_REQUIRED_POINTS_AFFINE = 4;
    public static final int MIN_REQUIRED_POINTS_7 = 7;
    public static final int MIN_REQUIRED_POINTS_8 = 8;
    
    public static final int MIN_POINTS = 100;
    public static final int MAX_POINTS = 500;    
    
    public static final double ABSOLUTE_ERROR = 5e-6;
    
    public static final double MIN_RANDOM_VALUE = -100.0;
    public static final double MAX_RANDOM_VALUE = -50.0;
    
    public static final double MIN_FOCAL_LENGTH = 110.0;
    public static final double MAX_FOCAL_LENGTH = 130.0;
    
    public static final double MIN_SKEWNESS = -0.001;
    public static final double MAX_SKEWNESS = 0.001;
    
    public static final double MIN_PRINCIPAL_POINT = 90.0;
    public static final double MAX_PRINCIPAL_POINT = 100.0;
    
    public static final double MIN_ANGLE_DEGREES = 10.0;
    public static final double MAX_ANGLE_DEGREES = 15.0;
    
    public static final double MIN_CAMERA_SEPARATION = 130.0;
    public static final double MAX_CAMERA_SEPARATION = 150.0;
    
    public static final int TIMES = 100;
    
    public static final int PERCENTAGE_OUTLIERS = 20;
    
    public static final double STD_ERROR = 10.0;
    
    public static final double STOP_THRESHOLD = 1e-9;

    private int estimateStart;
    private int estimateEnd;
    private int estimateNextIteration;
    private int estimateProgressChange;    
    
    public LMedSFundamentalMatrixRobustEstimatorTest() { }
    
    @BeforeClass
    public static void setUpClass() { }
    
    @AfterClass
    public static void tearDownClass() { }
    
    @Before
    public void setUp() { }
    
    @After
    public void tearDown() { }

    public void testConstrutor() {
        //test constructor without arguments
        LMedSFundamentalMatrixRobustEstimator estimator =
                new LMedSFundamentalMatrixRobustEstimator();
        
        //check correctness
        assertEquals(estimator.getStopThreshold(), 
                LMedSFundamentalMatrixRobustEstimator.DEFAULT_STOP_THRESHOLD, 
                0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(estimator.getNonRobustFundamentalMatrixEstimatorMethod(),
                FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM);        
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(), 
                FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                FundamentalMatrixRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                FundamentalMatrixRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getCovariance());
        assertEquals(estimator.getMinRequiredPoints(), MIN_REQUIRED_POINTS_7);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //test constructor with listener
        estimator = new LMedSFundamentalMatrixRobustEstimator(this);
        
        //check correctness
        assertEquals(estimator.getStopThreshold(),
                LMedSFundamentalMatrixRobustEstimator.DEFAULT_STOP_THRESHOLD, 
                0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(estimator.getNonRobustFundamentalMatrixEstimatorMethod(),
                FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM);                
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                FundamentalMatrixRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                FundamentalMatrixRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getCovariance());        
        assertEquals(estimator.getMinRequiredPoints(), MIN_REQUIRED_POINTS_7);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //test constructor with left and right points
        List<Point2D> leftPoints = new ArrayList<Point2D>();
        List<Point2D> rightPoints = new ArrayList<Point2D>();
        for (int i = 0; i < MIN_REQUIRED_POINTS_7; i++) {
            leftPoints.add(Point2D.create());
            rightPoints.add(Point2D.create());
        }
        
        estimator = new LMedSFundamentalMatrixRobustEstimator(leftPoints, 
                rightPoints);
        
        //check correctness
        assertEquals(estimator.getStopThreshold(),
                LMedSFundamentalMatrixRobustEstimator.DEFAULT_STOP_THRESHOLD, 
                0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(estimator.getNonRobustFundamentalMatrixEstimatorMethod(),
                FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM);        
        assertSame(estimator.getLeftPoints(), leftPoints);
        assertSame(estimator.getRightPoints(), rightPoints);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                FundamentalMatrixRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                FundamentalMatrixRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getCovariance());        
        assertEquals(estimator.getMinRequiredPoints(), MIN_REQUIRED_POINTS_7);        
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //Force IllegalArgumentException
        List<Point2D> emptyPoints = new ArrayList<Point2D>();
        estimator = null;
        try {
            estimator = new LMedSFundamentalMatrixRobustEstimator(emptyPoints, 
                    rightPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            estimator = new LMedSFundamentalMatrixRobustEstimator(leftPoints, 
                    emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            estimator = new LMedSFundamentalMatrixRobustEstimator(emptyPoints,
                    emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        assertNull(estimator);
        
        //test constructor with left and right points and listener
        estimator = new LMedSFundamentalMatrixRobustEstimator(leftPoints, 
                rightPoints, this);
        
        //check correctness
        assertEquals(estimator.getStopThreshold(),
                LMedSFundamentalMatrixRobustEstimator.DEFAULT_STOP_THRESHOLD, 
                0.0);        
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(estimator.getNonRobustFundamentalMatrixEstimatorMethod(),
                FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM);        
        assertSame(estimator.getLeftPoints(), leftPoints);
        assertSame(estimator.getRightPoints(), rightPoints);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                FundamentalMatrixRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                FundamentalMatrixRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getCovariance());                
        assertEquals(estimator.getMinRequiredPoints(), MIN_REQUIRED_POINTS_7);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new LMedSFundamentalMatrixRobustEstimator(emptyPoints, 
                    rightPoints, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            estimator = new LMedSFundamentalMatrixRobustEstimator(leftPoints, 
                    emptyPoints, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            estimator = new LMedSFundamentalMatrixRobustEstimator(emptyPoints,
                    emptyPoints, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        assertNull(estimator);        
    }

    @Test
    public void testGetSetNonRobustFundamentalMatrixEstimatorMethod() 
            throws LockedException {
        LMedSFundamentalMatrixRobustEstimator estimator =
                new LMedSFundamentalMatrixRobustEstimator();
        
        //check default value
        assertEquals(estimator.getNonRobustFundamentalMatrixEstimatorMethod(),
                FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM);
        assertEquals(estimator.getMinRequiredPoints(), MIN_REQUIRED_POINTS_7);
        
        //set new value
        estimator.setNonRobustFundamentalMatrixEstimatorMethod(
                FundamentalMatrixEstimatorMethod.EIGHT_POINTS_ALGORITHM);
        
        //check correctness
        assertEquals(estimator.getNonRobustFundamentalMatrixEstimatorMethod(),
                FundamentalMatrixEstimatorMethod.EIGHT_POINTS_ALGORITHM);        
        assertEquals(estimator.getMinRequiredPoints(), MIN_REQUIRED_POINTS_8);
    }
    
    @Test
    public void testGetSetStopThreshold() throws LockedException {
        LMedSFundamentalMatrixRobustEstimator estimator = 
                new LMedSFundamentalMatrixRobustEstimator();
        
        //check default value
        assertEquals(estimator.getStopThreshold(), 
                LMedSFundamentalMatrixRobustEstimator.DEFAULT_STOP_THRESHOLD, 
                0.0);
        
        //set new value
        estimator.setStopThreshold(0.5);
        
        //check correctness
        assertEquals(estimator.getStopThreshold(), 0.5, 0.0);
        
        //Force IllegalArgumentException
        try {
            estimator.setStopThreshold(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
    }
    
    @Test
    public void testGetSetPointsAndIsReady() throws LockedException {
        LMedSFundamentalMatrixRobustEstimator estimator =
                new LMedSFundamentalMatrixRobustEstimator();
        
        //check default values
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertFalse(estimator.isReady());
        
        //sets new values
        List<Point2D> leftPoints = new ArrayList<Point2D>();
        List<Point2D> rightPoints = new ArrayList<Point2D>();
        for (int i = 0; i < MIN_REQUIRED_POINTS_7; i++) {
            leftPoints.add(Point2D.create());
            rightPoints.add(Point2D.create());
        }
        
        estimator.setPoints(leftPoints, rightPoints);
        
        //check correctness
        assertSame(estimator.getLeftPoints(), leftPoints);
        assertSame(estimator.getRightPoints(), rightPoints);
        assertTrue(estimator.isReady());
        
        //Force IllegalArgumentException
        List<Point2D> emptyPoints = new ArrayList<Point2D>();
        try {
            estimator.setPoints(emptyPoints, rightPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            estimator.setPoints(leftPoints, emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            estimator.setPoints(emptyPoints, emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
    }
    
    @Test
    public void testGetSetListenerAndIsListenerAvailable() 
            throws LockedException {
        LMedSFundamentalMatrixRobustEstimator estimator =
                new LMedSFundamentalMatrixRobustEstimator();
        
        //check default value
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        
        //set new value
        estimator.setListener(this);
        
        //check correctness
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
    }
    
    @Test
    public void testGetSetProgressDelta() throws LockedException {
        LMedSFundamentalMatrixRobustEstimator estimator =
                new LMedSFundamentalMatrixRobustEstimator();
        
        //check default value
        assertEquals(estimator.getProgressDelta(),
                FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        
        //set new value
        estimator.setProgressDelta(0.5f);
        
        //check correctness
        assertEquals(estimator.getProgressDelta(), 0.5f, 0.0);
        
        //Force IllegalArgumentException
        try {
            estimator.setProgressDelta(-1.0f);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            estimator.setProgressDelta(2.0f);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
    }
    
    @Test
    public void testGetSetConfidence() throws LockedException {
        LMedSFundamentalMatrixRobustEstimator estimator =
                new LMedSFundamentalMatrixRobustEstimator();
        
        //check default value
        assertEquals(estimator.getConfidence(), 
                FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        
        //set new value
        estimator.setConfidence(0.5);
        
        //check correctness
        assertEquals(estimator.getConfidence(), 0.5, 0.0);
        
        //Force IllegalArgumentException
        try {
            estimator.setConfidence(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            estimator.setConfidence(2.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
    }
    
    @Test
    public void testGetSetMaxIterations() throws LockedException {
        LMedSFundamentalMatrixRobustEstimator estimator =
                new LMedSFundamentalMatrixRobustEstimator();
        
        //check default value
        assertEquals(estimator.getMaxIterations(),
                FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS);
        
        //set new value
        estimator.setMaxIterations(10);
        
        //check correctness
        assertEquals(estimator.getMaxIterations(), 10);
        
        //Force IllegalArgumentException
        try {
            estimator.setMaxIterations(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
    }
    
    @Test
    public void testIsSetResultRefined() throws LockedException {
        LMedSFundamentalMatrixRobustEstimator estimator =
                new LMedSFundamentalMatrixRobustEstimator();
        
        //check default value
        assertEquals(estimator.isResultRefined(),
                FundamentalMatrixRobustEstimator.DEFAULT_REFINE_RESULT);
        
        //set new value
        estimator.setResultRefined(
                !FundamentalMatrixRobustEstimator.DEFAULT_REFINE_RESULT);
        
        //check correctness
        assertEquals(estimator.isResultRefined(),
                !FundamentalMatrixRobustEstimator.DEFAULT_REFINE_RESULT);
    }
    
    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        LMedSFundamentalMatrixRobustEstimator estimator =
                new LMedSFundamentalMatrixRobustEstimator();
        
        //check default value
        assertEquals(estimator.isCovarianceKept(),
                FundamentalMatrixRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        
        //set new value
        estimator.setCovarianceKept(
                !FundamentalMatrixRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        
        //check correctness
        assertEquals(estimator.isCovarianceKept(),
                !FundamentalMatrixRobustEstimator.DEFAULT_KEEP_COVARIANCE);
    }
    
    @Test
    public void testGetSetQualityScores() throws LockedException {
        LMedSFundamentalMatrixRobustEstimator estimator =
                new LMedSFundamentalMatrixRobustEstimator();
        
        //check default value
        assertNull(estimator.getQualityScores());
        
        //set new value
        double[] qualityScores = new double[0];
        estimator.setQualityScores(qualityScores);
        
        //check correctness
        assertNull(estimator.getQualityScores());
    }
    
    @Test
    public void testEstimateSevenPointsWithRefinement() throws LockedException, 
            NotReadyException, RobustEstimatorException, 
            InvalidFundamentalMatrixException, NotAvailableException {
        double leftEpipoleError, rightEpipoleError;
        double avgLeftEpipoleError = 0.0, avgRightEpipoleError = 0.0;
        int numValid = 0;
        for (int j = 0; j < TIMES; j++) {
            //randomly create two pinhole cameras
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double alphaEuler1 = 0.0;
            double betaEuler1 = 0.0;
            double gammaEuler1 = 0.0;
            double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            
            double horizontalFocalLength1 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength1 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double horizontalFocalLength2 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength2 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            
            double skewness1 = randomizer.nextDouble(MIN_SKEWNESS, 
                    MAX_SKEWNESS);
            double skewness2 = randomizer.nextDouble(MIN_SKEWNESS, 
                    MAX_SKEWNESS);  
            
            double horizontalPrincipalPoint1 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint1 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double horizontalPrincipalPoint2 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint2 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

            int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            
            Point3D center1 = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);

            Rotation3D rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1,
                    gammaEuler1);
            Rotation3D rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2,
                    gammaEuler2);

            PinholeCameraIntrinsicParameters intrinsic1 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength1, 
                    verticalFocalLength1, horizontalPrincipalPoint1, 
                    verticalPrincipalPoint1, skewness1);
            PinholeCameraIntrinsicParameters intrinsic2 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength2, 
                    verticalFocalLength2, horizontalPrincipalPoint2, 
                    verticalPrincipalPoint2, skewness2);

            PinholeCamera camera1 = new PinholeCamera(intrinsic1, rotation1, 
                    center1);
            PinholeCamera camera2 = new PinholeCamera(intrinsic2, rotation2,
                    center2);
            
            //generate a random list of 3D points
            List<Point3D> points3D = new ArrayList<Point3D>();
            for (int i = 0; i < nPoints; i++) {
                points3D.add(new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE), randomizer.nextDouble(
                        MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE)));
            }

            //project 3D ponits with both cameras
            List<Point2D> leftPoints = camera1.project(points3D);
            List<Point2D> rightPoints = camera2.project(points3D);
            
            //add outliers
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            
            List<Point2D> leftPointsWithError = new ArrayList<Point2D>();
            for (Point2D leftPoint : leftPoints) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    //outlier
                    double errorX = errorRandomizer.nextDouble();
                    double errorY = errorRandomizer.nextDouble();
                    leftPointsWithError.add(new HomogeneousPoint2D(
                            leftPoint.getInhomX() + errorX, 
                            leftPoint.getInhomY() + errorY, 1.0));
                } else {
                    //inlier
                    leftPointsWithError.add(leftPoint);
                }
            }
            
            List<Point2D> rightPointsWithError = new ArrayList<Point2D>();
            for (Point2D rightPoint : rightPoints) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    //outlier
                    double errorX = errorRandomizer.nextDouble();
                    double errorY = errorRandomizer.nextDouble();
                    rightPointsWithError.add(new HomogeneousPoint2D(
                            rightPoint.getInhomX() + errorX,
                            rightPoint.getInhomY() + errorY, 1.0));
                } else {
                    //inlier
                    rightPointsWithError.add(rightPoint);
                }
            }
            
            //estimate fundamental matrix
            LMedSFundamentalMatrixRobustEstimator estimator =
                    new LMedSFundamentalMatrixRobustEstimator(
                    leftPointsWithError, rightPointsWithError, this);
            estimator.setStopThreshold(STOP_THRESHOLD);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);
            
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getCovariance());
            
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            
            FundamentalMatrix fundMatrix = estimator.estimate();
            assertEquals(estimator.getMinRequiredPoints(),
                    MIN_REQUIRED_POINTS_7);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            assertNotNull(estimator.getCovariance());
            
            //check correctness
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();            
            
            //compute epipoles
            Point2D epipole1a = camera1.project(center2);
            Point2D epipole2a = camera2.project(center1);

            fundMatrix.computeEpipoles();

            Point2D epipole1b = fundMatrix.getLeftEpipole();
            Point2D epipole2b = fundMatrix.getRightEpipole();

            //check correctness of epipoles
            leftEpipoleError = epipole1a.distanceTo(epipole1b);
            rightEpipoleError = epipole2a.distanceTo(epipole2b);
            if (leftEpipoleError > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(leftEpipoleError, 0.0, ABSOLUTE_ERROR);
            if (rightEpipoleError > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(rightEpipoleError, 0.0, ABSOLUTE_ERROR);

            avgLeftEpipoleError += leftEpipoleError;
            avgRightEpipoleError += rightEpipoleError;
            
            //check that all points lie within their corresponding epipolar 
            //lines
            for (int i = 0; i < nPoints; i++) {
                Point2D leftPoint = leftPoints.get(i);
                Point2D rightPoint = rightPoints.get(i);
                Point3D point3D = points3D.get(i);

                //obtain epipolar line on left view using 2D point on right view
                Line2D line1 = fundMatrix.getLeftEpipolarLine(rightPoint);
                //obtain epipolar line on right view using 2D point on left view
                Line2D line2 = fundMatrix.getRightEpipolarLine(leftPoint);

                //check that 2D point on left view belongs to left epipolar line
                assertTrue(line1.isLocus(leftPoint, ABSOLUTE_ERROR));
                //check that 2D point on right view belongs to right epipolar 
                //line
                assertTrue(line2.isLocus(rightPoint, ABSOLUTE_ERROR));

                //obtain epipolar planes
                Plane epipolarPlane1 = camera1.backProject(line1);
                Plane epipolarPlane2 = camera2.backProject(line2);

                //check that both planes are the same
                assertTrue(epipolarPlane1.equals(epipolarPlane2, 
                        ABSOLUTE_ERROR));

                //check that poin3D and camera centers belong to epipolar plane
                assertTrue(epipolarPlane1.isLocus(point3D, 
                        ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1.isLocus(center1, 
                        ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1.isLocus(center2, 
                        ABSOLUTE_ERROR));

                assertTrue(epipolarPlane2.isLocus(point3D, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane2.isLocus(center1, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane2.isLocus(center2, ABSOLUTE_ERROR));
            }            
            
            numValid++;
        }
        
        assertTrue(numValid > 0);
        
        avgLeftEpipoleError /= (double)TIMES;
        avgRightEpipoleError /= (double)TIMES;
        
        assertEquals(avgLeftEpipoleError, 0.0, ABSOLUTE_ERROR);
        assertEquals(avgRightEpipoleError, 0.0, ABSOLUTE_ERROR);
        
        //Force NotReadyException
        LMedSFundamentalMatrixRobustEstimator estimator = 
                new LMedSFundamentalMatrixRobustEstimator();        
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException e) { }
    }

    @Test
    public void testEstimateEightPointsWithRefinement() throws LockedException, 
            NotReadyException, RobustEstimatorException, 
            InvalidFundamentalMatrixException, NotAvailableException {
        double leftEpipoleError, rightEpipoleError;
        double avgLeftEpipoleError = 0.0, avgRightEpipoleError = 0.0;
        int numValid = 0;
        for (int j = 0; j < TIMES; j++) {
            //randomly create two pinhole cameras
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double alphaEuler1 = 0.0;
            double betaEuler1 = 0.0;
            double gammaEuler1 = 0.0;
            double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            
            double horizontalFocalLength1 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength1 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double horizontalFocalLength2 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength2 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            
            double skewness1 = randomizer.nextDouble(MIN_SKEWNESS, 
                    MAX_SKEWNESS);
            double skewness2 = randomizer.nextDouble(MIN_SKEWNESS, 
                    MAX_SKEWNESS);  
            
            double horizontalPrincipalPoint1 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint1 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double horizontalPrincipalPoint2 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint2 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

            int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            
            Point3D center1 = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);

            Rotation3D rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1,
                    gammaEuler1);
            Rotation3D rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2,
                    gammaEuler2);

            PinholeCameraIntrinsicParameters intrinsic1 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength1, 
                    verticalFocalLength1, horizontalPrincipalPoint1, 
                    verticalPrincipalPoint1, skewness1);
            PinholeCameraIntrinsicParameters intrinsic2 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength2, 
                    verticalFocalLength2, horizontalPrincipalPoint2, 
                    verticalPrincipalPoint2, skewness2);

            PinholeCamera camera1 = new PinholeCamera(intrinsic1, rotation1, 
                    center1);
            PinholeCamera camera2 = new PinholeCamera(intrinsic2, rotation2,
                    center2);
            
            //generate a random list of 3D points
            List<Point3D> points3D = new ArrayList<Point3D>();
            for (int i = 0; i < nPoints; i++) {
                points3D.add(new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE), randomizer.nextDouble(
                        MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE)));
            }

            //project 3D ponits with both cameras
            List<Point2D> leftPoints = camera1.project(points3D);
            List<Point2D> rightPoints = camera2.project(points3D);
            
            //add outliers
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            
            List<Point2D> leftPointsWithError = new ArrayList<Point2D>();
            for (Point2D leftPoint : leftPoints) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    //outlier
                    double errorX = errorRandomizer.nextDouble();
                    double errorY = errorRandomizer.nextDouble();
                    leftPointsWithError.add(new HomogeneousPoint2D(
                            leftPoint.getInhomX() + errorX, 
                            leftPoint.getInhomY() + errorY, 1.0));
                } else {
                    //inlier
                    leftPointsWithError.add(leftPoint);
                }
            }
            
            List<Point2D> rightPointsWithError = new ArrayList<Point2D>();
            for (Point2D rightPoint : rightPoints) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    //outlier
                    double errorX = errorRandomizer.nextDouble();
                    double errorY = errorRandomizer.nextDouble();
                    rightPointsWithError.add(new HomogeneousPoint2D(
                            rightPoint.getInhomX() + errorX,
                            rightPoint.getInhomY() + errorY, 1.0));
                } else {
                    //inlier
                    rightPointsWithError.add(rightPoint);
                }
            }
            
            //estimate fundamental matrix
            LMedSFundamentalMatrixRobustEstimator estimator =
                    new LMedSFundamentalMatrixRobustEstimator(
                    FundamentalMatrixEstimatorMethod.EIGHT_POINTS_ALGORITHM,
                    leftPointsWithError, rightPointsWithError, this);
            estimator.setStopThreshold(STOP_THRESHOLD);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);
            
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getCovariance());
            
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            
            FundamentalMatrix fundMatrix = estimator.estimate();
            assertEquals(estimator.getMinRequiredPoints(),
                    MIN_REQUIRED_POINTS_8);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            assertNotNull(estimator.getCovariance());
            
            //check correctness
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();
            
            //compute epipoles
            Point2D epipole1a = camera1.project(center2);
            Point2D epipole2a = camera2.project(center1);

            fundMatrix.computeEpipoles();

            Point2D epipole1b = fundMatrix.getLeftEpipole();
            Point2D epipole2b = fundMatrix.getRightEpipole();

            //check correctness of epipoles
            leftEpipoleError = epipole1a.distanceTo(epipole1b);
            rightEpipoleError = epipole2a.distanceTo(epipole2b);
            if (leftEpipoleError > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(leftEpipoleError, 0.0, ABSOLUTE_ERROR);
            if (rightEpipoleError > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(rightEpipoleError, 0.0, ABSOLUTE_ERROR);

            avgLeftEpipoleError += leftEpipoleError;
            avgRightEpipoleError += rightEpipoleError;
            
            //check that all points lie within their corresponding epipolar 
            //lines
            for (int i = 0; i < nPoints; i++) {
                Point2D leftPoint = leftPoints.get(i);
                Point2D rightPoint = rightPoints.get(i);
                Point3D point3D = points3D.get(i);

                //obtain epipolar line on left view using 2D point on right view
                Line2D line1 = fundMatrix.getLeftEpipolarLine(rightPoint);
                //obtain epipolar line on right view using 2D point on left view
                Line2D line2 = fundMatrix.getRightEpipolarLine(leftPoint);

                //check that 2D point on left view belongs to left epipolar line
                assertTrue(line1.isLocus(leftPoint, ABSOLUTE_ERROR));
                //check that 2D point on right view belongs to right epipolar 
                //line
                assertTrue(line2.isLocus(rightPoint, ABSOLUTE_ERROR));

                //obtain epipolar planes
                Plane epipolarPlane1 = camera1.backProject(line1);
                Plane epipolarPlane2 = camera2.backProject(line2);

                //check that both planes are the same
                assertTrue(epipolarPlane1.equals(epipolarPlane2, 
                        ABSOLUTE_ERROR));

                //check that poin3D and camera centers belong to epipolar plane
                assertTrue(epipolarPlane1.isLocus(point3D, 
                        ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1.isLocus(center1, 
                        ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1.isLocus(center2, 
                        ABSOLUTE_ERROR));

                assertTrue(epipolarPlane2.isLocus(point3D, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane2.isLocus(center1, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane2.isLocus(center2, ABSOLUTE_ERROR));
            }     
            
            numValid++;            
        }
        
        assertTrue(numValid > 0);
        
        avgLeftEpipoleError /= (double)TIMES;
        avgRightEpipoleError /= (double)TIMES;
        
        assertEquals(avgLeftEpipoleError, 0.0, ABSOLUTE_ERROR);
        assertEquals(avgRightEpipoleError, 0.0, ABSOLUTE_ERROR);
        
        //Force NotReadyException
        LMedSFundamentalMatrixRobustEstimator estimator = 
                new LMedSFundamentalMatrixRobustEstimator();        
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch(NotReadyException e) { }
    }

    @Test
    public void testEstimateAffineWithRefinement() throws LockedException, 
            NotReadyException, RobustEstimatorException, 
            InvalidFundamentalMatrixException, NotAvailableException, 
            WrongSizeException, InvalidPairOfCamerasException {

        int numValid = 0;
        for (int j = 0; j < TIMES; j++) {
            //randomly create two pinhole cameras
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double alphaEuler1 = 0.0;
            double betaEuler1 = 0.0;
            double gammaEuler1 = 0.0;
            double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            
            double horizontalFocalLength1 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength1 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double horizontalFocalLength2 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength2 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            
            double skewness1 = randomizer.nextDouble(MIN_SKEWNESS, 
                    MAX_SKEWNESS);
            double skewness2 = randomizer.nextDouble(MIN_SKEWNESS, 
                    MAX_SKEWNESS);  
            
            double horizontalPrincipalPoint1 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint1 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double horizontalPrincipalPoint2 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint2 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

            int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            
            Point3D center1 = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);

            Rotation3D rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1,
                    gammaEuler1);
            Rotation3D rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2,
                    gammaEuler2);

            PinholeCameraIntrinsicParameters intrinsic1 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength1, 
                    verticalFocalLength1, horizontalPrincipalPoint1, 
                    verticalPrincipalPoint1, skewness1);
            PinholeCameraIntrinsicParameters intrinsic2 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength2, 
                    verticalFocalLength2, horizontalPrincipalPoint2, 
                    verticalPrincipalPoint2, skewness2);

            PinholeCamera camera1 = new PinholeCamera(intrinsic1, rotation1, 
                    center1);
            PinholeCamera camera2 = new PinholeCamera(intrinsic2, rotation2,
                    center2);
            
            //convert cameras into affine cameras
            Matrix cameraMatrix1 = camera1.getInternalMatrix();
            cameraMatrix1.setElementAt(2, 0, 0.0);
            cameraMatrix1.setElementAt(2, 1, 0.0);
            cameraMatrix1.setElementAt(2, 2, 0.0);
            cameraMatrix1.setElementAt(2, 3, 1.0);
            camera1.setInternalMatrix(cameraMatrix1);
            
            Matrix cameraMatrix2 = camera2.getInternalMatrix();
            cameraMatrix2.setElementAt(2, 0, 0.0);
            cameraMatrix2.setElementAt(2, 1, 0.0);
            cameraMatrix2.setElementAt(2, 2, 0.0);
            cameraMatrix2.setElementAt(2, 3, 1.0);
            camera2.setInternalMatrix(cameraMatrix2);
            
            
            //generate a random list of 3D points
            List<Point3D> points3D = new ArrayList<Point3D>();
            for (int i = 0; i < nPoints; i++) {
                points3D.add(new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE), randomizer.nextDouble(
                        MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE)));
            }

            //project 3D ponits with both cameras
            List<Point2D> leftPoints = camera1.project(points3D);
            List<Point2D> rightPoints = camera2.project(points3D);
            
            //add outliers
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            
            List<Point2D> leftPointsWithError = new ArrayList<Point2D>();
            for (Point2D leftPoint : leftPoints) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    //outlier
                    double errorX = errorRandomizer.nextDouble();
                    double errorY = errorRandomizer.nextDouble();
                    leftPointsWithError.add(new HomogeneousPoint2D(
                            leftPoint.getInhomX() + errorX, 
                            leftPoint.getInhomY() + errorY, 1.0));
                } else {
                    //inlier
                    leftPointsWithError.add(leftPoint);
                }
            }
            
            List<Point2D> rightPointsWithError = new ArrayList<Point2D>();
            for (Point2D rightPoint : rightPoints) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    //outlier
                    double errorX = errorRandomizer.nextDouble();
                    double errorY = errorRandomizer.nextDouble();
                    rightPointsWithError.add(new HomogeneousPoint2D(
                            rightPoint.getInhomX() + errorX,
                            rightPoint.getInhomY() + errorY, 1.0));
                } else {
                    //inlier
                    rightPointsWithError.add(rightPoint);
                }
            }
            
            //estimate fundamental matrix
            LMedSFundamentalMatrixRobustEstimator estimator =
                    new LMedSFundamentalMatrixRobustEstimator(
                    FundamentalMatrixEstimatorMethod.AFFINE_ALGORITHM,
                    leftPointsWithError, rightPointsWithError, this);
            estimator.setStopThreshold(STOP_THRESHOLD);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);
            
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getCovariance());
            
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            
            FundamentalMatrix fundMatrix = estimator.estimate();
            assertEquals(estimator.getMinRequiredPoints(),
                    MIN_REQUIRED_POINTS_AFFINE);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            
            //check correctness
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();            
            
            //compute epipoles
            FundamentalMatrix fundMatrix2 = new FundamentalMatrix(camera1, 
                    camera2);
            fundMatrix2.computeEpipoles();
            
            Point2D epipole1a = fundMatrix2.getLeftEpipole();
            Point2D epipole2a = fundMatrix2.getRightEpipole();

            fundMatrix.computeEpipoles();

            Point2D epipole1b = fundMatrix.getLeftEpipole();
            Point2D epipole2b = fundMatrix.getRightEpipole();

            //check correctness of epipoles
            if (!epipole1a.equals(epipole1b, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipole1a.equals(epipole1b, ABSOLUTE_ERROR));
            if (!epipole2a.equals(epipole2b, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipole2a.equals(epipole2b, ABSOLUTE_ERROR));
            
                        
            fundMatrix.normalize();
            fundMatrix2.normalize();
            
            //check that both matrices are equal up to scale (i.e. sign)
            if (!fundMatrix.getInternalMatrix().equals(
                    fundMatrix2.getInternalMatrix(), ABSOLUTE_ERROR) &&
                    !fundMatrix.getInternalMatrix().equals(
                    fundMatrix2.getInternalMatrix().
                    multiplyByScalarAndReturnNew(-1.0), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundMatrix.getInternalMatrix().equals(
                    fundMatrix2.getInternalMatrix(), ABSOLUTE_ERROR) ||
                    fundMatrix.getInternalMatrix().equals(
                    fundMatrix2.getInternalMatrix().
                    multiplyByScalarAndReturnNew(-1.0), ABSOLUTE_ERROR));
            
            //check that all points lie within their corresponding epipolar 
            //lines
            for (int i = 0; i < nPoints; i++) {
                Point2D leftPoint = leftPoints.get(i);
                Point2D rightPoint = rightPoints.get(i);
                Point3D point3D = points3D.get(i);

                //obtain epipolar line on left view using 2D point on right view
                Line2D line1 = fundMatrix.getLeftEpipolarLine(rightPoint);
                //obtain epipolar line on right view using 2D point on left view
                Line2D line2 = fundMatrix.getRightEpipolarLine(leftPoint);

                //check that 2D point on left view belongs to left epipolar line
                assertTrue(line1.isLocus(leftPoint, ABSOLUTE_ERROR));
                //check that 2D point on right view belongs to right epipolar 
                //line
                assertTrue(line2.isLocus(rightPoint, ABSOLUTE_ERROR));

                //obtain epipolar planes
                Plane epipolarPlane1 = camera1.backProject(line1);
                Plane epipolarPlane2 = camera2.backProject(line2);

                //check that both planes are the same
                assertTrue(epipolarPlane1.equals(epipolarPlane2, 
                        ABSOLUTE_ERROR));

                //check that poin3D and camera centers belong to epipolar plane
                if (!epipolarPlane1.isLocus(point3D, ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane1.isLocus(point3D, 
                        ABSOLUTE_ERROR));
                if (!epipolarPlane1.isLocus(center1, ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane1.isLocus(center1, 
                        ABSOLUTE_ERROR));
                if (!epipolarPlane1.isLocus(center2, ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane1.isLocus(center2, 
                        ABSOLUTE_ERROR));

                if (!epipolarPlane2.isLocus(point3D, ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane2.isLocus(point3D, ABSOLUTE_ERROR));
                if (!epipolarPlane2.isLocus(center1, ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane2.isLocus(center1, ABSOLUTE_ERROR));
                if (!epipolarPlane2.isLocus(center2, ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane2.isLocus(center2, ABSOLUTE_ERROR));
            }            
            
            numValid++;
        }
        
        assertTrue(numValid > 0);
                
        //Force NotReadyException
        LMedSFundamentalMatrixRobustEstimator estimator = 
                new LMedSFundamentalMatrixRobustEstimator();        
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException e) { }
    }
    
    @Test
    public void testEstimateSevenPointsWithoutRefinement() 
            throws LockedException, NotReadyException, RobustEstimatorException, 
            InvalidFundamentalMatrixException, NotAvailableException {
        double leftEpipoleError, rightEpipoleError;
        double avgLeftEpipoleError = 0.0, avgRightEpipoleError = 0.0;
        for (int j = 0; j < TIMES; j++) {
            //randomly create two pinhole cameras
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double alphaEuler1 = 0.0;
            double betaEuler1 = 0.0;
            double gammaEuler1 = 0.0;
            double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            
            double horizontalFocalLength1 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength1 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double horizontalFocalLength2 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength2 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            
            double skewness1 = randomizer.nextDouble(MIN_SKEWNESS, 
                    MAX_SKEWNESS);
            double skewness2 = randomizer.nextDouble(MIN_SKEWNESS, 
                    MAX_SKEWNESS);  
            
            double horizontalPrincipalPoint1 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint1 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double horizontalPrincipalPoint2 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint2 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

            int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            
            Point3D center1 = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);

            Rotation3D rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1,
                    gammaEuler1);
            Rotation3D rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2,
                    gammaEuler2);

            PinholeCameraIntrinsicParameters intrinsic1 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength1, 
                    verticalFocalLength1, horizontalPrincipalPoint1, 
                    verticalPrincipalPoint1, skewness1);
            PinholeCameraIntrinsicParameters intrinsic2 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength2, 
                    verticalFocalLength2, horizontalPrincipalPoint2, 
                    verticalPrincipalPoint2, skewness2);

            PinholeCamera camera1 = new PinholeCamera(intrinsic1, rotation1, 
                    center1);
            PinholeCamera camera2 = new PinholeCamera(intrinsic2, rotation2,
                    center2);
            
            //generate a random list of 3D points
            List<Point3D> points3D = new ArrayList<Point3D>();
            for (int i = 0; i < nPoints; i++) {
                points3D.add(new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE), randomizer.nextDouble(
                        MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE)));
            }

            //project 3D ponits with both cameras
            List<Point2D> leftPoints = camera1.project(points3D);
            List<Point2D> rightPoints = camera2.project(points3D);
            
            //add outliers
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            
            List<Point2D> leftPointsWithError = new ArrayList<Point2D>();
            for (Point2D leftPoint : leftPoints) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    //outlier
                    double errorX = errorRandomizer.nextDouble();
                    double errorY = errorRandomizer.nextDouble();
                    leftPointsWithError.add(new HomogeneousPoint2D(
                            leftPoint.getInhomX() + errorX, 
                            leftPoint.getInhomY() + errorY, 1.0));
                } else {
                    //inlier
                    leftPointsWithError.add(leftPoint);
                }
            }
            
            List<Point2D> rightPointsWithError = new ArrayList<Point2D>();
            for (Point2D rightPoint : rightPoints) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    //outlier
                    double errorX = errorRandomizer.nextDouble();
                    double errorY = errorRandomizer.nextDouble();
                    rightPointsWithError.add(new HomogeneousPoint2D(
                            rightPoint.getInhomX() + errorX,
                            rightPoint.getInhomY() + errorY, 1.0));
                } else {
                    //inlier
                    rightPointsWithError.add(rightPoint);
                }
            }
            
            //estimate fundamental matrix
            LMedSFundamentalMatrixRobustEstimator estimator =
                    new LMedSFundamentalMatrixRobustEstimator(
                    leftPointsWithError, rightPointsWithError, this);
            estimator.setStopThreshold(STOP_THRESHOLD);
            estimator.setResultRefined(false);
            estimator.setCovarianceKept(false);
            
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getCovariance());
            
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            
            FundamentalMatrix fundMatrix = estimator.estimate();
            assertEquals(estimator.getMinRequiredPoints(),
                    MIN_REQUIRED_POINTS_7);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            assertNull(estimator.getCovariance());
            
            //check correctness
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();            
            
            //compute epipoles
            Point2D epipole1a = camera1.project(center2);
            Point2D epipole2a = camera2.project(center1);

            fundMatrix.computeEpipoles();

            Point2D epipole1b = fundMatrix.getLeftEpipole();
            Point2D epipole2b = fundMatrix.getRightEpipole();

            //check correctness of epipoles
            leftEpipoleError = epipole1a.distanceTo(epipole1b);
            rightEpipoleError = epipole2a.distanceTo(epipole2b);
            assertEquals(leftEpipoleError, 0.0, ABSOLUTE_ERROR);
            assertEquals(rightEpipoleError, 0.0, ABSOLUTE_ERROR);

            avgLeftEpipoleError += leftEpipoleError;
            avgRightEpipoleError += rightEpipoleError;
            
            //check that all points lie within their corresponding epipolar 
            //lines
            for (int i = 0; i < nPoints; i++) {
                Point2D leftPoint = leftPoints.get(i);
                Point2D rightPoint = rightPoints.get(i);
                Point3D point3D = points3D.get(i);

                //obtain epipolar line on left view using 2D point on right view
                Line2D line1 = fundMatrix.getLeftEpipolarLine(rightPoint);
                //obtain epipolar line on right view using 2D point on left view
                Line2D line2 = fundMatrix.getRightEpipolarLine(leftPoint);

                //check that 2D point on left view belongs to left epipolar line
                assertTrue(line1.isLocus(leftPoint, ABSOLUTE_ERROR));
                //check that 2D point on right view belongs to right epipolar 
                //line
                assertTrue(line2.isLocus(rightPoint, ABSOLUTE_ERROR));

                //obtain epipolar planes
                Plane epipolarPlane1 = camera1.backProject(line1);
                Plane epipolarPlane2 = camera2.backProject(line2);

                //check that both planes are the same
                assertTrue(epipolarPlane1.equals(epipolarPlane2, 
                        ABSOLUTE_ERROR));

                //check that poin3D and camera centers belong to epipolar plane
                assertTrue(epipolarPlane1.isLocus(point3D, 
                        ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1.isLocus(center1, 
                        ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1.isLocus(center2, 
                        ABSOLUTE_ERROR));

                assertTrue(epipolarPlane2.isLocus(point3D, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane2.isLocus(center1, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane2.isLocus(center2, ABSOLUTE_ERROR));
            }            
        }
        
        avgLeftEpipoleError /= (double)TIMES;
        avgRightEpipoleError /= (double)TIMES;
        
        assertEquals(avgLeftEpipoleError, 0.0, ABSOLUTE_ERROR);
        assertEquals(avgRightEpipoleError, 0.0, ABSOLUTE_ERROR);
        
        //Force NotReadyException
        LMedSFundamentalMatrixRobustEstimator estimator = 
                new LMedSFundamentalMatrixRobustEstimator();        
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException e) { }
    }

    @Test
    public void testEstimateEightPointsWithoutRefinement() 
            throws LockedException, NotReadyException, RobustEstimatorException, 
            InvalidFundamentalMatrixException, NotAvailableException {
        double leftEpipoleError, rightEpipoleError;
        double avgLeftEpipoleError = 0.0, avgRightEpipoleError = 0.0;
        int numValid = 0;
        for (int j = 0; j < TIMES; j++) {
            //randomly create two pinhole cameras
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double alphaEuler1 = 0.0;
            double betaEuler1 = 0.0;
            double gammaEuler1 = 0.0;
            double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            
            double horizontalFocalLength1 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength1 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double horizontalFocalLength2 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength2 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            
            double skewness1 = randomizer.nextDouble(MIN_SKEWNESS, 
                    MAX_SKEWNESS);
            double skewness2 = randomizer.nextDouble(MIN_SKEWNESS, 
                    MAX_SKEWNESS);  
            
            double horizontalPrincipalPoint1 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint1 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double horizontalPrincipalPoint2 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint2 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

            int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            
            Point3D center1 = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);

            Rotation3D rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1,
                    gammaEuler1);
            Rotation3D rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2,
                    gammaEuler2);

            PinholeCameraIntrinsicParameters intrinsic1 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength1, 
                    verticalFocalLength1, horizontalPrincipalPoint1, 
                    verticalPrincipalPoint1, skewness1);
            PinholeCameraIntrinsicParameters intrinsic2 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength2, 
                    verticalFocalLength2, horizontalPrincipalPoint2, 
                    verticalPrincipalPoint2, skewness2);

            PinholeCamera camera1 = new PinholeCamera(intrinsic1, rotation1, 
                    center1);
            PinholeCamera camera2 = new PinholeCamera(intrinsic2, rotation2,
                    center2);
            
            //generate a random list of 3D points
            List<Point3D> points3D = new ArrayList<Point3D>();
            for (int i = 0; i < nPoints; i++) {
                points3D.add(new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE), randomizer.nextDouble(
                        MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE)));
            }

            //project 3D ponits with both cameras
            List<Point2D> leftPoints = camera1.project(points3D);
            List<Point2D> rightPoints = camera2.project(points3D);
            
            //add outliers
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            
            List<Point2D> leftPointsWithError = new ArrayList<Point2D>();
            for (Point2D leftPoint : leftPoints) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    //outlier
                    double errorX = errorRandomizer.nextDouble();
                    double errorY = errorRandomizer.nextDouble();
                    leftPointsWithError.add(new HomogeneousPoint2D(
                            leftPoint.getInhomX() + errorX, 
                            leftPoint.getInhomY() + errorY, 1.0));
                } else {
                    //inlier
                    leftPointsWithError.add(leftPoint);
                }
            }
            
            List<Point2D> rightPointsWithError = new ArrayList<Point2D>();
            for (Point2D rightPoint : rightPoints) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    //outlier
                    double errorX = errorRandomizer.nextDouble();
                    double errorY = errorRandomizer.nextDouble();
                    rightPointsWithError.add(new HomogeneousPoint2D(
                            rightPoint.getInhomX() + errorX,
                            rightPoint.getInhomY() + errorY, 1.0));
                } else {
                    //inlier
                    rightPointsWithError.add(rightPoint);
                }
            }
            
            //estimate fundamental matrix
            LMedSFundamentalMatrixRobustEstimator estimator =
                    new LMedSFundamentalMatrixRobustEstimator(
                    FundamentalMatrixEstimatorMethod.EIGHT_POINTS_ALGORITHM,
                    leftPointsWithError, rightPointsWithError, this);
            estimator.setStopThreshold(STOP_THRESHOLD);
            estimator.setResultRefined(false);
            estimator.setCovarianceKept(false);
            
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getCovariance());
            
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            
            FundamentalMatrix fundMatrix = estimator.estimate();
            assertEquals(estimator.getMinRequiredPoints(),
                    MIN_REQUIRED_POINTS_8);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            assertNull(estimator.getCovariance());
            
            //check correctness
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();
            
            //compute epipoles
            Point2D epipole1a = camera1.project(center2);
            Point2D epipole2a = camera2.project(center1);

            fundMatrix.computeEpipoles();

            Point2D epipole1b = fundMatrix.getLeftEpipole();
            Point2D epipole2b = fundMatrix.getRightEpipole();

            //check correctness of epipoles
            leftEpipoleError = epipole1a.distanceTo(epipole1b);
            rightEpipoleError = epipole2a.distanceTo(epipole2b);
            if (Math.abs(leftEpipoleError) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(leftEpipoleError, 0.0, ABSOLUTE_ERROR);
            if (Math.abs(rightEpipoleError) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(rightEpipoleError, 0.0, ABSOLUTE_ERROR);

            avgLeftEpipoleError += leftEpipoleError;
            avgRightEpipoleError += rightEpipoleError;
            
            //check that all points lie within their corresponding epipolar 
            //lines
            for (int i = 0; i < nPoints; i++) {
                Point2D leftPoint = leftPoints.get(i);
                Point2D rightPoint = rightPoints.get(i);
                Point3D point3D = points3D.get(i);

                //obtain epipolar line on left view using 2D point on right view
                Line2D line1 = fundMatrix.getLeftEpipolarLine(rightPoint);
                //obtain epipolar line on right view using 2D point on left view
                Line2D line2 = fundMatrix.getRightEpipolarLine(leftPoint);

                //check that 2D point on left view belongs to left epipolar line
                assertTrue(line1.isLocus(leftPoint, ABSOLUTE_ERROR));
                //check that 2D point on right view belongs to right epipolar 
                //line
                assertTrue(line2.isLocus(rightPoint, ABSOLUTE_ERROR));

                //obtain epipolar planes
                Plane epipolarPlane1 = camera1.backProject(line1);
                Plane epipolarPlane2 = camera2.backProject(line2);

                //check that both planes are the same
                assertTrue(epipolarPlane1.equals(epipolarPlane2, 
                        ABSOLUTE_ERROR));

                //check that poin3D and camera centers belong to epipolar plane
                assertTrue(epipolarPlane1.isLocus(point3D, 
                        ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1.isLocus(center1, 
                        ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1.isLocus(center2, 
                        ABSOLUTE_ERROR));

                assertTrue(epipolarPlane2.isLocus(point3D, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane2.isLocus(center1, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane2.isLocus(center2, ABSOLUTE_ERROR));
            }

            numValid++;
        }

        assertTrue(numValid > 0);
        
        avgLeftEpipoleError /= (double)numValid;
        avgRightEpipoleError /= (double)numValid;
        
        assertEquals(avgLeftEpipoleError, 0.0, ABSOLUTE_ERROR);
        assertEquals(avgRightEpipoleError, 0.0, ABSOLUTE_ERROR);
        
        //Force NotReadyException
        LMedSFundamentalMatrixRobustEstimator estimator = 
                new LMedSFundamentalMatrixRobustEstimator();        
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch(NotReadyException e) { }
    }
    
    @Test
    public void testEstimateAffineWithoutRefinement() throws LockedException, 
            NotReadyException, RobustEstimatorException, 
            InvalidFundamentalMatrixException, NotAvailableException, 
            WrongSizeException, InvalidPairOfCamerasException {

        int numValid = 0;
        for (int j = 0; j < TIMES; j++) {
            //randomly create two pinhole cameras
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double alphaEuler1 = 0.0;
            double betaEuler1 = 0.0;
            double gammaEuler1 = 0.0;
            double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            
            double horizontalFocalLength1 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength1 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double horizontalFocalLength2 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength2 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            
            double skewness1 = randomizer.nextDouble(MIN_SKEWNESS, 
                    MAX_SKEWNESS);
            double skewness2 = randomizer.nextDouble(MIN_SKEWNESS, 
                    MAX_SKEWNESS);  
            
            double horizontalPrincipalPoint1 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint1 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double horizontalPrincipalPoint2 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint2 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

            int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            
            Point3D center1 = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);

            Rotation3D rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1,
                    gammaEuler1);
            Rotation3D rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2,
                    gammaEuler2);

            PinholeCameraIntrinsicParameters intrinsic1 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength1, 
                    verticalFocalLength1, horizontalPrincipalPoint1, 
                    verticalPrincipalPoint1, skewness1);
            PinholeCameraIntrinsicParameters intrinsic2 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength2, 
                    verticalFocalLength2, horizontalPrincipalPoint2, 
                    verticalPrincipalPoint2, skewness2);

            PinholeCamera camera1 = new PinholeCamera(intrinsic1, rotation1, 
                    center1);
            PinholeCamera camera2 = new PinholeCamera(intrinsic2, rotation2,
                    center2);
            
            //convert cameras into affine cameras
            Matrix cameraMatrix1 = camera1.getInternalMatrix();
            cameraMatrix1.setElementAt(2, 0, 0.0);
            cameraMatrix1.setElementAt(2, 1, 0.0);
            cameraMatrix1.setElementAt(2, 2, 0.0);
            cameraMatrix1.setElementAt(2, 3, 1.0);
            camera1.setInternalMatrix(cameraMatrix1);
            
            Matrix cameraMatrix2 = camera2.getInternalMatrix();
            cameraMatrix2.setElementAt(2, 0, 0.0);
            cameraMatrix2.setElementAt(2, 1, 0.0);
            cameraMatrix2.setElementAt(2, 2, 0.0);
            cameraMatrix2.setElementAt(2, 3, 1.0);
            camera2.setInternalMatrix(cameraMatrix2);
            
            
            //generate a random list of 3D points
            List<Point3D> points3D = new ArrayList<Point3D>();
            for (int i = 0; i < nPoints; i++) {
                points3D.add(new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE), randomizer.nextDouble(
                        MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE)));
            }

            //project 3D ponits with both cameras
            List<Point2D> leftPoints = camera1.project(points3D);
            List<Point2D> rightPoints = camera2.project(points3D);
            
            //add outliers
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            
            List<Point2D> leftPointsWithError = new ArrayList<Point2D>();
            for (Point2D leftPoint : leftPoints) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    //outlier
                    double errorX = errorRandomizer.nextDouble();
                    double errorY = errorRandomizer.nextDouble();
                    leftPointsWithError.add(new HomogeneousPoint2D(
                            leftPoint.getInhomX() + errorX, 
                            leftPoint.getInhomY() + errorY, 1.0));
                } else {
                    //inlier
                    leftPointsWithError.add(leftPoint);
                }
            }
            
            List<Point2D> rightPointsWithError = new ArrayList<Point2D>();
            for (Point2D rightPoint : rightPoints) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    //outlier
                    double errorX = errorRandomizer.nextDouble();
                    double errorY = errorRandomizer.nextDouble();
                    rightPointsWithError.add(new HomogeneousPoint2D(
                            rightPoint.getInhomX() + errorX,
                            rightPoint.getInhomY() + errorY, 1.0));
                } else {
                    //inlier
                    rightPointsWithError.add(rightPoint);
                }
            }
            
            //estimate fundamental matrix
            LMedSFundamentalMatrixRobustEstimator estimator =
                    new LMedSFundamentalMatrixRobustEstimator(
                    FundamentalMatrixEstimatorMethod.AFFINE_ALGORITHM,
                    leftPointsWithError, rightPointsWithError, this);
            estimator.setStopThreshold(STOP_THRESHOLD);
            estimator.setResultRefined(false);
            estimator.setCovarianceKept(false);
            
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getCovariance());
            
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            
            FundamentalMatrix fundMatrix = estimator.estimate();
            assertEquals(estimator.getMinRequiredPoints(),
                    MIN_REQUIRED_POINTS_AFFINE);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            assertNull(estimator.getCovariance());
            
            //check correctness
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();            
            
            //compute epipoles
            FundamentalMatrix fundMatrix2 = new FundamentalMatrix(camera1, 
                    camera2);
            fundMatrix2.computeEpipoles();
            
            Point2D epipole1a = fundMatrix2.getLeftEpipole();
            Point2D epipole2a = fundMatrix2.getRightEpipole();

            fundMatrix.computeEpipoles();

            Point2D epipole1b = fundMatrix.getLeftEpipole();
            Point2D epipole2b = fundMatrix.getRightEpipole();

            //check correctness of epipoles
            if (!epipole1a.equals(epipole1b, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipole1a.equals(epipole1b, ABSOLUTE_ERROR));
            if (!epipole2a.equals(epipole2b, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipole2a.equals(epipole2b, ABSOLUTE_ERROR));
            
                        
            fundMatrix.normalize();
            fundMatrix2.normalize();
            
            //check that both matrices are equal up to scale (i.e. sign)
            if (!fundMatrix.getInternalMatrix().equals(
                    fundMatrix2.getInternalMatrix(), ABSOLUTE_ERROR) &&
                    !fundMatrix.getInternalMatrix().equals(
                    fundMatrix2.getInternalMatrix().
                    multiplyByScalarAndReturnNew(-1.0), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundMatrix.getInternalMatrix().equals(
                    fundMatrix2.getInternalMatrix(), ABSOLUTE_ERROR) ||
                    fundMatrix.getInternalMatrix().equals(
                    fundMatrix2.getInternalMatrix().
                    multiplyByScalarAndReturnNew(-1.0), ABSOLUTE_ERROR));
            
            //check that all points lie within their corresponding epipolar 
            //lines
            for (int i = 0; i < nPoints; i++) {
                Point2D leftPoint = leftPoints.get(i);
                Point2D rightPoint = rightPoints.get(i);
                Point3D point3D = points3D.get(i);

                //obtain epipolar line on left view using 2D point on right view
                Line2D line1 = fundMatrix.getLeftEpipolarLine(rightPoint);
                //obtain epipolar line on right view using 2D point on left view
                Line2D line2 = fundMatrix.getRightEpipolarLine(leftPoint);

                //check that 2D point on left view belongs to left epipolar line
                assertTrue(line1.isLocus(leftPoint, ABSOLUTE_ERROR));
                //check that 2D point on right view belongs to right epipolar 
                //line
                assertTrue(line2.isLocus(rightPoint, ABSOLUTE_ERROR));

                //obtain epipolar planes
                Plane epipolarPlane1 = camera1.backProject(line1);
                Plane epipolarPlane2 = camera2.backProject(line2);

                //check that both planes are the same
                assertTrue(epipolarPlane1.equals(epipolarPlane2, 
                        ABSOLUTE_ERROR));

                //check that poin3D and camera centers belong to epipolar plane
                if (!epipolarPlane1.isLocus(point3D, ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane1.isLocus(point3D, 
                        ABSOLUTE_ERROR));
                if (!epipolarPlane1.isLocus(center1, ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane1.isLocus(center1, 
                        ABSOLUTE_ERROR));
                if (!epipolarPlane1.isLocus(center2, ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane1.isLocus(center2, 
                        ABSOLUTE_ERROR));

                if (!epipolarPlane2.isLocus(point3D, ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane2.isLocus(point3D, ABSOLUTE_ERROR));
                if (!epipolarPlane2.isLocus(center1, ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane2.isLocus(center1, ABSOLUTE_ERROR));
                if (!epipolarPlane2.isLocus(center2, ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane2.isLocus(center2, ABSOLUTE_ERROR));
            }            
            
            numValid++;
        }
        
        assertTrue(numValid > 0);
                
        //Force NotReadyException
        LMedSFundamentalMatrixRobustEstimator estimator = 
                new LMedSFundamentalMatrixRobustEstimator();        
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException e) { }
    }
    
    private void reset(){
        estimateStart = estimateEnd = estimateNextIteration =
                estimateProgressChange = 0;
    }
    
    @Override
    public void onEstimateStart(FundamentalMatrixRobustEstimator estimator) {
        estimateStart++;
        testLocked((LMedSFundamentalMatrixRobustEstimator)estimator);
    }

    @Override
    public void onEstimateEnd(FundamentalMatrixRobustEstimator estimator) {
        estimateEnd++;
        testLocked((LMedSFundamentalMatrixRobustEstimator)estimator);
    }

    @Override
    public void onEstimateNextIteration(
            FundamentalMatrixRobustEstimator estimator, int iteration) {
        estimateNextIteration++;
        testLocked((LMedSFundamentalMatrixRobustEstimator)estimator);
    }

    @Override
    public void onEstimateProgressChange(
            FundamentalMatrixRobustEstimator estimator, float progress) {
        estimateProgressChange++;
        testLocked((LMedSFundamentalMatrixRobustEstimator)estimator);
    }
    
    private void testLocked(LMedSFundamentalMatrixRobustEstimator estimator) {
        try {
            estimator.setConfidence(0.5);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setListener(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setMaxIterations(10);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setResultRefined(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setCovarianceKept(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setPoints(null, null);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setProgressDelta(0.5f);
            fail("LockedException expected but not thrown");
        } catch (LockedException e){ }
        try {
            estimator.setStopThreshold(0.5);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setNonRobustFundamentalMatrixEstimatorMethod(
                    FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.estimate();
            fail("LockedException expected but not thrown");
        } catch (LockedException e) {
        } catch (NotReadyException e) {
            fail("LockedException expected but not thrown");
        } catch (RobustEstimatorException e) {
            fail("LockedException expected but not thrown");
        }
        assertTrue(estimator.isLocked());
    }    
}
