/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.calib3d.estimators.KruppaDualImageOfAbsoluteConicEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date December 26, 2016.
 */
package com.irurueta.geometry.calib3d.estimators;

import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.InvalidPinholeCameraIntrinsicParametersException;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.Quaternion;
import com.irurueta.geometry.calib3d.DualImageOfAbsoluteConic;
import com.irurueta.geometry.epipolar.FundamentalMatrix;
import com.irurueta.geometry.epipolar.InvalidPairOfCamerasException;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.statistics.UniformRandomizer;
import java.util.Random;
import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class KruppaDualImageOfAbsoluteConicEstimatorTest implements 
        KruppaDualImageOfAbsoluteConicEstimatorListener {
    
    public static final double MIN_FOCAL_LENGTH = 1.0;
    public static final double MAX_FOCAL_LENGTH = 100.0;
    
    public static final double MIN_RANDOM_VALUE = -10.0;
    public static final double MAX_RANDOM_VALUE = 10.0;
    
    public static final double MIN_ANGLE_DEGREES = 0.0;
    public static final double MAX_ANGLE_DEGREES = 90.0;
    
    public static final double MIN_ASPECT_RATIO = 0.5;
    public static final double MAX_ASPECT_RATIO = 2.0;
    
    public static final double ABSOLUTE_ERROR = 1e-6;    
    public static final double LARGE_ABSOLUTE_ERROR = 5.0;
    
    public static final int TIMES = 1000;
    
    private int estimateStart;
    private int estimateEnd;    
    
    public KruppaDualImageOfAbsoluteConicEstimatorTest() { }
    
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
        KruppaDualImageOfAbsoluteConicEstimator estimator = 
                new KruppaDualImageOfAbsoluteConicEstimator();
        
        //check default values
        assertEquals(estimator.getPrincipalPointX(), 0.0, 0.0);
        assertEquals(estimator.getPrincipalPointY(), 0.0, 0.0);
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertFalse(estimator.isLocked());
        assertNull(estimator.getListener());
        assertNull(estimator.getFundamentalMatrix());
        assertFalse(estimator.isReady());
        
        
        //test constructor with listener
        estimator = new KruppaDualImageOfAbsoluteConicEstimator(this);
        
        //check default values
        assertEquals(estimator.getPrincipalPointX(), 0.0, 0.0);
        assertEquals(estimator.getPrincipalPointY(), 0.0, 0.0);
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertFalse(estimator.isLocked());
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getFundamentalMatrix());
        assertFalse(estimator.isReady());
        
        
        //test constructor with fundamental matrix
        FundamentalMatrix f = new FundamentalMatrix();
        estimator = new KruppaDualImageOfAbsoluteConicEstimator(f);
        
        //check default values
        assertEquals(estimator.getPrincipalPointX(), 0.0, 0.0);
        assertEquals(estimator.getPrincipalPointY(), 0.0, 0.0);
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertFalse(estimator.isLocked());
        assertNull(estimator.getListener());
        assertSame(estimator.getFundamentalMatrix(), f);
        assertTrue(estimator.isReady());
        
        
        //test constructor with fundamental matrix and listener
        estimator = new KruppaDualImageOfAbsoluteConicEstimator(f, this);
        
        //check default values
        assertEquals(estimator.getPrincipalPointX(), 0.0, 0.0);
        assertEquals(estimator.getPrincipalPointY(), 0.0, 0.0);
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertFalse(estimator.isLocked());
        assertSame(estimator.getListener(), this);
        assertSame(estimator.getFundamentalMatrix(), f);
        assertTrue(estimator.isReady());        
    }
    
    @Test
    public void testGetSetPrincipalPointX() throws LockedException {
        KruppaDualImageOfAbsoluteConicEstimator estimator = 
                new KruppaDualImageOfAbsoluteConicEstimator();
        
        //check default value
        assertEquals(estimator.getPrincipalPointX(), 0.0, 0.0);
        
        //set new value
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double principalPointX = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        estimator.setPrincipalPointX(principalPointX);
        
        //check correctness
        assertEquals(estimator.getPrincipalPointX(), principalPointX, 0.0);
    }
    
    @Test
    public void testGetSetPrincipalPointY() throws LockedException {
        KruppaDualImageOfAbsoluteConicEstimator estimator = 
                new KruppaDualImageOfAbsoluteConicEstimator();
        
        //check default value
        assertEquals(estimator.getPrincipalPointY(), 0.0, 0.0);
        
        //set new value
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double principalPointY = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        estimator.setPrincipalPointY(principalPointY);
        
        //check correctness
        assertEquals(estimator.getPrincipalPointY(), principalPointY, 0.0);        
    }
    
    @Test
    public void testIsSetFocalDistanceAspectRatioKnown() 
            throws LockedException {
        KruppaDualImageOfAbsoluteConicEstimator estimator = 
                new KruppaDualImageOfAbsoluteConicEstimator();
        
        //check default value
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        
        //st new value
        estimator.setFocalDistanceAspectRatioKnown(false);
        
        //check correctness
        assertFalse(estimator.isFocalDistanceAspectRatioKnown());
    }
    
    @Test
    public void testGetSetFocalDistanceAspectRatio() throws LockedException {
        KruppaDualImageOfAbsoluteConicEstimator estimator =
                new KruppaDualImageOfAbsoluteConicEstimator();
        
        //check default value
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        
        //set new value
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double aspectRatio = randomizer.nextDouble(MIN_ASPECT_RATIO, 
                MAX_ASPECT_RATIO);
        
        estimator.setFocalDistanceAspectRatio(aspectRatio);
        
        //check correctness
        assertEquals(estimator.getFocalDistanceAspectRatio(), aspectRatio, 0.0);        
    }
    
    @Test
    public void testGetSetListener() throws LockedException {
        KruppaDualImageOfAbsoluteConicEstimator estimator =
                new KruppaDualImageOfAbsoluteConicEstimator();
        
        //check default value
        assertNull(estimator.getListener());
        
        //set new value
        estimator.setListener(this);
        
        //check correctness
        assertSame(estimator.getListener(), this);
    }
    
    @Test
    public void testGetSetFundamentalMatrixAndIsReady() throws LockedException {
        KruppaDualImageOfAbsoluteConicEstimator estimator =
                new KruppaDualImageOfAbsoluteConicEstimator();
        
        //check default value
        assertNull(estimator.getFundamentalMatrix());
        assertFalse(estimator.isReady());
        
        //set new vlaue
        FundamentalMatrix f = new FundamentalMatrix();
        estimator.setFundamentalMatrix(f);
        
        //check correctness
        assertSame(estimator.getFundamentalMatrix(), f);
        assertTrue(estimator.isReady());
    }
    
    @Test
    public void testEstimateWithKnownAspectRatio() 
            throws InvalidPairOfCamerasException, 
            KruppaDualImageOfAbsoluteConicEstimatorException, LockedException, 
            NotReadyException, 
            InvalidPinholeCameraIntrinsicParametersException {
        int numValid = 0;
        for(int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
            //create ground truth intrinsic parameters
            double horizontalFocalLength, verticalFocalLength, skewness,
                    horizontalPrincipalPoint, verticalPrincipalPoint;
            double aspectRatio = 1.0;
            horizontalFocalLength = 
                    randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            verticalFocalLength = aspectRatio * horizontalFocalLength;
            skewness = 0.0;
            horizontalPrincipalPoint = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            verticalPrincipalPoint = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);

            PinholeCameraIntrinsicParameters intrinsic = 
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength, 
                    verticalFocalLength, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, skewness);
            
            double rollLeft, pitchLeft, yawLeft, xLeft, yLeft, zLeft;
            double rollRight, pitchRight, yawRight, xRight, yRight, zRight;
            Quaternion qLeft, qRight;
            InhomogeneousPoint3D leftCameraCenter, rightCameraCenter;
            PinholeCamera leftCamera, rightCamera;
            
            rollLeft = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 
                    2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            pitchLeft = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            yawLeft = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 
                    2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            qLeft = new Quaternion(rollLeft, pitchLeft, yawLeft);
            
            rollRight = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 
                    2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            pitchRight = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            yawRight = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 
                    2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            qRight = new Quaternion(rollRight, pitchRight, yawRight);
            
            xLeft = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            yLeft = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            zLeft = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            leftCameraCenter = new InhomogeneousPoint3D(xLeft, yLeft, zLeft);

            xRight = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            yRight = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            zRight = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            rightCameraCenter = new InhomogeneousPoint3D(xRight, yRight, 
                    zRight);
            
            leftCamera = new PinholeCamera(intrinsic, qLeft, leftCameraCenter);
            rightCamera = new PinholeCamera(intrinsic, qRight, 
                    rightCameraCenter);
            
            FundamentalMatrix fundamentalMatrix;
            try {
                fundamentalMatrix = new FundamentalMatrix(leftCamera, 
                        rightCamera);
            } catch (InvalidPairOfCamerasException e) {
                continue;
            }
        
            KruppaDualImageOfAbsoluteConicEstimator estimator = 
                    new KruppaDualImageOfAbsoluteConicEstimator(
                            fundamentalMatrix, this);
            estimator.setPrincipalPointX(horizontalPrincipalPoint);
            estimator.setPrincipalPointY(verticalPrincipalPoint);
            estimator.setFocalDistanceAspectRatioKnown(true);
            estimator.setFocalDistanceAspectRatio(aspectRatio);
            
            //comprobamos
            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertTrue(estimator.isReady());
        
            DualImageOfAbsoluteConic diac;
            DualImageOfAbsoluteConic diac2 = new DualImageOfAbsoluteConic(
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0);            
            try {
                diac = estimator.estimate();
                estimator.estimate(diac2);
            } catch (KruppaDualImageOfAbsoluteConicEstimatorException e) {
                continue;
            }
        
            //check correctness
            assertEquals(estimateStart, 2);
            assertEquals(estimateEnd, 2);
            diac.normalize();
            diac2.normalize();
        
            assertTrue(diac.asMatrix().equals(diac2.asMatrix(), 
                    ABSOLUTE_ERROR));
        
            PinholeCameraIntrinsicParameters intrinsic2 = 
                    diac.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters intrinsic3 =
                    diac2.getIntrinsicParameters();
        
            
            if(Math.abs(intrinsic2.getHorizontalFocalLength() - 
                    horizontalFocalLength) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(intrinsic2.getHorizontalFocalLength(), 
                    horizontalFocalLength, ABSOLUTE_ERROR);
            assertEquals(intrinsic2.getVerticalFocalLength(), 
                    verticalFocalLength, ABSOLUTE_ERROR);
            assertEquals(intrinsic2.getHorizontalPrincipalPoint(), 
                    horizontalPrincipalPoint, ABSOLUTE_ERROR);
            assertEquals(intrinsic2.getVerticalPrincipalPoint(), 
                    verticalPrincipalPoint, ABSOLUTE_ERROR);
            assertEquals(intrinsic2.getSkewness(), 0.0, ABSOLUTE_ERROR);

            assertEquals(intrinsic3.getHorizontalFocalLength(), 
                    horizontalFocalLength, ABSOLUTE_ERROR);
            assertEquals(intrinsic3.getVerticalFocalLength(), 
                    verticalFocalLength, ABSOLUTE_ERROR);
            assertEquals(intrinsic3.getHorizontalPrincipalPoint(), 
                    horizontalPrincipalPoint, ABSOLUTE_ERROR);
            assertEquals(intrinsic3.getVerticalPrincipalPoint(), 
                    verticalPrincipalPoint, ABSOLUTE_ERROR);
            assertEquals(intrinsic3.getSkewness(), 0.0, ABSOLUTE_ERROR);
            
            numValid++;
        }
        
        assertTrue(numValid > 0);
    }

    @Test
    public void testEstimateWithUnknownAspectRatio() 
            throws KruppaDualImageOfAbsoluteConicEstimatorException, 
            LockedException, NotReadyException, 
            InvalidPinholeCameraIntrinsicParametersException {
        int numValid = 0;
        for(int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
            //create ground truth intrinsic parameters
            double horizontalFocalLength, verticalFocalLength, skewness,
                    horizontalPrincipalPoint, verticalPrincipalPoint;
            double aspectRatio = randomizer.nextDouble(MIN_ASPECT_RATIO, 
                    MAX_ASPECT_RATIO);
            horizontalFocalLength = 
                    randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            verticalFocalLength = aspectRatio * horizontalFocalLength;
            skewness = 0.0;
            horizontalPrincipalPoint = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            verticalPrincipalPoint = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);

            PinholeCameraIntrinsicParameters intrinsic = 
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength, 
                    verticalFocalLength, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, skewness);
            
            double rollLeft, pitchLeft, yawLeft, xLeft, yLeft, zLeft;
            double rollRight, pitchRight, yawRight, xRight, yRight, zRight;
            Quaternion qLeft, qRight;
            InhomogeneousPoint3D leftCameraCenter, rightCameraCenter;
            PinholeCamera leftCamera, rightCamera;
            
            rollLeft = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 
                    2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            pitchLeft = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            yawLeft = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 
                    2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            qLeft = new Quaternion(rollLeft, pitchLeft, yawLeft);
            
            rollRight = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 
                    2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            pitchRight = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            yawRight = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 
                    2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            qRight = new Quaternion(rollRight, pitchRight, yawRight);
            
            xLeft = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            yLeft = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            zLeft = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            leftCameraCenter = new InhomogeneousPoint3D(xLeft, yLeft, zLeft);

            xRight = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            yRight = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            zRight = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            rightCameraCenter = new InhomogeneousPoint3D(xRight, yRight, 
                    zRight);
            
            leftCamera = new PinholeCamera(intrinsic, qLeft, leftCameraCenter);
            rightCamera = new PinholeCamera(intrinsic, qRight, 
                    rightCameraCenter);
            
            FundamentalMatrix fundamentalMatrix;
            try {
                fundamentalMatrix = new FundamentalMatrix(leftCamera, 
                        rightCamera);
            } catch (InvalidPairOfCamerasException e) {
                continue;
            }
        
            KruppaDualImageOfAbsoluteConicEstimator estimator = 
                    new KruppaDualImageOfAbsoluteConicEstimator(
                            fundamentalMatrix, this);
            estimator.setPrincipalPointX(horizontalPrincipalPoint);
            estimator.setPrincipalPointY(verticalPrincipalPoint);
            estimator.setFocalDistanceAspectRatioKnown(false);
            
            //comprobamos
            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertTrue(estimator.isReady());
        
            DualImageOfAbsoluteConic diac;
            DualImageOfAbsoluteConic diac2 = new DualImageOfAbsoluteConic(
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0);            
            try {
                diac = estimator.estimate();
                estimator.estimate(diac2);
            } catch (KruppaDualImageOfAbsoluteConicEstimatorException e) {
                continue;
            }
        
            //check correctness
            assertEquals(estimateStart, 2);
            assertEquals(estimateEnd, 2);
            diac.normalize();
            diac2.normalize();
        
            assertTrue(diac.asMatrix().equals(diac2.asMatrix(), 
                    ABSOLUTE_ERROR));
        
            PinholeCameraIntrinsicParameters intrinsic2 = 
                    diac.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters intrinsic3 =
                    diac2.getIntrinsicParameters();
        
            
            if(Math.abs(intrinsic2.getHorizontalFocalLength() - 
                    horizontalFocalLength) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(intrinsic2.getHorizontalFocalLength(), 
                    horizontalFocalLength, LARGE_ABSOLUTE_ERROR);
            if(Math.abs(intrinsic2.getVerticalFocalLength() - 
                    verticalFocalLength) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(intrinsic2.getVerticalFocalLength(), 
                    verticalFocalLength, LARGE_ABSOLUTE_ERROR);
            if(Math.abs(intrinsic2.getHorizontalPrincipalPoint() - 
                    horizontalPrincipalPoint) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(intrinsic2.getHorizontalPrincipalPoint(), 
                    horizontalPrincipalPoint, ABSOLUTE_ERROR);
            assertEquals(intrinsic2.getVerticalPrincipalPoint(), 
                    verticalPrincipalPoint, ABSOLUTE_ERROR);
            assertEquals(intrinsic2.getSkewness(), 0.0, ABSOLUTE_ERROR);

            assertEquals(intrinsic3.getHorizontalFocalLength(), 
                    horizontalFocalLength, LARGE_ABSOLUTE_ERROR);
            assertEquals(intrinsic3.getVerticalFocalLength(), 
                    verticalFocalLength, LARGE_ABSOLUTE_ERROR);
            assertEquals(intrinsic3.getHorizontalPrincipalPoint(), 
                    horizontalPrincipalPoint, ABSOLUTE_ERROR);
            assertEquals(intrinsic3.getVerticalPrincipalPoint(), 
                    verticalPrincipalPoint, ABSOLUTE_ERROR);
            assertEquals(intrinsic3.getSkewness(), 0.0, ABSOLUTE_ERROR);
            
            numValid++;
        }
        
        assertTrue(numValid > 0);
    }
    
    private void reset() {
        estimateStart = estimateEnd = 0;
    }

    @Override
    public void onEstimateStart(
            KruppaDualImageOfAbsoluteConicEstimator estimator) {
        estimateStart++;
        checkLocked(estimator);
    }

    @Override
    public void onEstimateEnd(
            KruppaDualImageOfAbsoluteConicEstimator estimator) {
        estimateEnd++;
        checkLocked(estimator);
    }
    
    private void checkLocked(
            KruppaDualImageOfAbsoluteConicEstimator estimator) {
        assertTrue(estimator.isLocked());
        
        try {
            estimator.setPrincipalPointX(0.0);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setPrincipalPointY(0.0);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setFocalDistanceAspectRatioKnown(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setFocalDistanceAspectRatio(1.0);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setListener(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setFundamentalMatrix(null);
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
