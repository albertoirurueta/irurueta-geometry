/**
 * @file
 * This fiel contains unit tests for
 * com.irurueta.geometry.calib3d.estimators.PROMedSDualAbsoluteQuadricRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date December 12, 2016.
 */
package com.irurueta.geometry.calib3d.estimators;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.geometry.CameraException;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.NotAvailableException;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.ProjectiveTransformation3D;
import com.irurueta.geometry.Quaternion;
import com.irurueta.geometry.Rotation3D;
import com.irurueta.geometry.calib3d.DualAbsoluteQuadric;
import com.irurueta.geometry.calib3d.DualImageOfAbsoluteConic;
import com.irurueta.geometry.calib3d.InvalidTransformationException;
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

public class PROMedSDualAbsoluteQuadricRobustEstimatorTest implements 
        DualAbsoluteQuadricRobustEstimatorListener {
    
    public static final double MIN_FOCAL_LENGTH = 1.0;
    public static final double MAX_FOCAL_LENGTH = 100.0;
    
    public static final double MIN_RANDOM_VALUE = -10.0;
    public static final double MAX_RANDOM_VALUE = 10.0;
    
    public static final double MIN_ANGLE_DEGREES = 0.0;
    public static final double MAX_ANGLE_DEGREES = 90.0;

    public static final double ABSOLUTE_ERROR = 1e-6;  
    public static final double LARGE_ABSOLUTE_ERROR = 1e-3;
    
    public static final int TIMES = 50; //1000;
    
    public static final int NUM_CAMS = 100;
    
    public static final int PERCENTAGE_OUTLIERS = 10;
    
    public static final double STD_ERROR = 1.0;
    
    public PROMedSDualAbsoluteQuadricRobustEstimatorTest() { }
    
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
        PROMedSDualAbsoluteQuadricRobustEstimator estimator =
                new PROMedSDualAbsoluteQuadricRobustEstimator();
        
        //check correctness
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(estimator.getDeterminantThreshold(),
                DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD, 
                0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getCameras());
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 2);
        assertTrue(estimator.areValidConstraints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertEquals(estimator.getStopThreshold(), 
                PROMedSDualAbsoluteQuadricRobustEstimator.
                        DEFAULT_STOP_THRESHOLD, 0.0);
        
        
        //test with listener
        estimator = new PROMedSDualAbsoluteQuadricRobustEstimator(this);
        
        //check correctness
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(estimator.getDeterminantThreshold(),
                DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD, 
                0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(), 
                DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getCameras());
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 2);
        assertTrue(estimator.areValidConstraints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertEquals(estimator.getStopThreshold(), 
                PROMedSDualAbsoluteQuadricRobustEstimator.DEFAULT_STOP_THRESHOLD, 
                0.0);
        
        
        //test with cameras
        List<PinholeCamera> cameras = new ArrayList<PinholeCamera>();
        cameras.add(new PinholeCamera());
        cameras.add(new PinholeCamera());        
        
        estimator = new PROMedSDualAbsoluteQuadricRobustEstimator(cameras);
        
        //check correctness
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(estimator.getDeterminantThreshold(),
                DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD, 
                0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getCameras(), cameras);
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 2);
        assertTrue(estimator.areValidConstraints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertEquals(estimator.getStopThreshold(), 
                PROMedSDualAbsoluteQuadricRobustEstimator.
                        DEFAULT_STOP_THRESHOLD, 0.0);

        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSDualAbsoluteQuadricRobustEstimator(
                    new ArrayList<PinholeCamera>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        assertNull(estimator);
        
        
        //test with cameras and listener
        estimator = new PROMedSDualAbsoluteQuadricRobustEstimator(cameras, 
                this);
        
        //check correctness
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(estimator.getDeterminantThreshold(),
                DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD, 
                0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getCameras(), cameras);
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 2);
        assertTrue(estimator.areValidConstraints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertEquals(estimator.getStopThreshold(), 
                PROMedSDualAbsoluteQuadricRobustEstimator.
                        DEFAULT_STOP_THRESHOLD, 0.0);

        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSDualAbsoluteQuadricRobustEstimator(
                    new ArrayList<PinholeCamera>(), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        assertNull(estimator);        
        
        
        //test with quality scores
        double[] qualityScores = new double[2];
        estimator = new PROMedSDualAbsoluteQuadricRobustEstimator(
                qualityScores);
        
        //check correctness
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(estimator.getDeterminantThreshold(),
                DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD, 
                0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getCameras());
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 2);
        assertTrue(estimator.areValidConstraints());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertEquals(estimator.getStopThreshold(), 
                PROMedSDualAbsoluteQuadricRobustEstimator.
                        DEFAULT_STOP_THRESHOLD, 0.0);
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSDualAbsoluteQuadricRobustEstimator(
                    new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        assertNull(estimator);
        
        
        //test with quality scores and listener
        estimator = new PROMedSDualAbsoluteQuadricRobustEstimator(qualityScores, 
                this);
        
        //check correctness
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(estimator.getDeterminantThreshold(),
                DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD, 
                0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(), 
                DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getCameras());
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 2);
        assertTrue(estimator.areValidConstraints());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertEquals(estimator.getStopThreshold(), 
                PROMedSDualAbsoluteQuadricRobustEstimator.DEFAULT_STOP_THRESHOLD, 
                0.0);
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSDualAbsoluteQuadricRobustEstimator(
                    new double[1], this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        assertNull(estimator);
        
        
        //test with quality scores and cameras
        estimator = new PROMedSDualAbsoluteQuadricRobustEstimator(cameras, 
                qualityScores);
        
        //check correctness
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(estimator.getDeterminantThreshold(),
                DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD, 
                0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getCameras(), cameras);
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 2);
        assertTrue(estimator.areValidConstraints());
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertEquals(estimator.getStopThreshold(), 
                PROMedSDualAbsoluteQuadricRobustEstimator.
                        DEFAULT_STOP_THRESHOLD, 0.0);

        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSDualAbsoluteQuadricRobustEstimator(
                    new ArrayList<PinholeCamera>(), qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            estimator = new PROMedSDualAbsoluteQuadricRobustEstimator(
                    cameras, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }        
        assertNull(estimator);
        
        
        //test with quality scores, cameras and listener
        estimator = new PROMedSDualAbsoluteQuadricRobustEstimator(cameras, 
                qualityScores, this);
        
        //check correctness
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(estimator.getDeterminantThreshold(),
                DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD, 
                0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getCameras(), cameras);
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 2);
        assertTrue(estimator.areValidConstraints());
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertEquals(estimator.getStopThreshold(), 
                PROMedSDualAbsoluteQuadricRobustEstimator.
                        DEFAULT_STOP_THRESHOLD, 0.0);

        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSDualAbsoluteQuadricRobustEstimator(
                    new ArrayList<PinholeCamera>(), qualityScores, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            estimator = new PROMedSDualAbsoluteQuadricRobustEstimator(
                    cameras, new double[1], this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }        
        assertNull(estimator);        
    }
    
    @Test
    public void testGetSetStopThreshold() throws LockedException {
        PROMedSDualAbsoluteQuadricRobustEstimator estimator =
                new PROMedSDualAbsoluteQuadricRobustEstimator();
        
        //check default value
        assertEquals(estimator.getStopThreshold(),
                PROMedSDualAbsoluteQuadricRobustEstimator.
                        DEFAULT_STOP_THRESHOLD, 0.0);
        
        //set new value
        estimator.setStopThreshold(0.5);
        
        //check correctness
        assertEquals(estimator.getStopThreshold(), 0.5, 0.0);
    }
    
    @Test
    public void testGetSetQualityScores() throws LockedException {
        PROMedSDualAbsoluteQuadricRobustEstimator estimator =
                new PROMedSDualAbsoluteQuadricRobustEstimator();
        
        //check default value
        assertNull(estimator.getQualityScores());
        
        //set new value
        double[] qualityScores = new double[2];
        estimator.setQualityScores(qualityScores);
        
        //check correctness
        assertSame(estimator.getQualityScores(), qualityScores);
    }
    
    @Test
    public void testIsSetZeroSkewness() throws LockedException {
        PROMedSDualAbsoluteQuadricRobustEstimator estimator =
                new PROMedSDualAbsoluteQuadricRobustEstimator();
        
        //check default value
        assertTrue(estimator.isZeroSkewness());
        
        //set new value
        estimator.setZeroSkewness(false);
        
        //check correctness
        assertFalse(estimator.isZeroSkewness());
    }
    
    @Test
    public void testIsSetPrincipalPointAtOrigin() throws LockedException {
        PROMedSDualAbsoluteQuadricRobustEstimator estimator =
                new PROMedSDualAbsoluteQuadricRobustEstimator();
        
        //check default value
        assertTrue(estimator.isPrincipalPointAtOrigin());
        
        //set new value
        estimator.setPrincipalPointAtOrigin(false);
        
        //check correctness
        assertFalse(estimator.isPrincipalPointAtOrigin());
    }
    
    @Test
    public void testIsFocalDistanceAspectRatioKnown() throws LockedException {
        PROMedSDualAbsoluteQuadricRobustEstimator estimator =
                new PROMedSDualAbsoluteQuadricRobustEstimator();
        
        //check default value
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        
        //set new value
        estimator.setFocalDistanceAspectRatioKnown(false);
        
        //check correctness
        assertFalse(estimator.isFocalDistanceAspectRatioKnown());
    }
    
    @Test
    public void testGetSetFocalDistanceAspectRatio() throws LockedException {
        PROMedSDualAbsoluteQuadricRobustEstimator estimator =
                new PROMedSDualAbsoluteQuadricRobustEstimator();
        
        //check default value
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        
        //set new value
        estimator.setFocalDistanceAspectRatio(0.5);
        
        //check correctness
        assertEquals(estimator.getFocalDistanceAspectRatio(), 0.5, 0.0);
    }
    
    @Test
    public void testIsSetSingularityEnforced() throws LockedException {
        PROMedSDualAbsoluteQuadricRobustEstimator estimator =
                new PROMedSDualAbsoluteQuadricRobustEstimator();
        
        //check default value
        assertTrue(estimator.isSingularityEnforced());
        
        //set new value
        estimator.setSingularityEnforced(false);
        
        //check correctness
        assertFalse(estimator.isSingularityEnforced());
    }
    
    @Test
    public void testIsSetEnforcedsingularityValidated() throws LockedException {
        PROMedSDualAbsoluteQuadricRobustEstimator estimator =
                new PROMedSDualAbsoluteQuadricRobustEstimator();
        
        //check default value
        assertTrue(estimator.isEnforcedSingularityValidated());
        
        //set new value
        estimator.setEnforcedSingularityValidated(false);
        
        //check correctness
        assertFalse(estimator.isEnforcedSingularityValidated());
    }
    
    @Test
    public void testGetSetDeterminantThreshold() throws LockedException {
        PROMedSDualAbsoluteQuadricRobustEstimator estimator =
                new PROMedSDualAbsoluteQuadricRobustEstimator();
        
        //check default value
        assertEquals(estimator.getDeterminantThreshold(),
                DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD, 
                0.0);
        
        //set new value
        estimator.setDeterminantThreshold(1e-3);
        
        //check correctness
        assertEquals(estimator.getDeterminantThreshold(), 1e-3, 0.0);
    }
    
    @Test
    public void testGetSetListenerAndIsListenerAvailable()
            throws LockedException {
        PROMedSDualAbsoluteQuadricRobustEstimator estimator =
                new PROMedSDualAbsoluteQuadricRobustEstimator();
        
        //check default value
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        
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
        PROMedSDualAbsoluteQuadricRobustEstimator estimator =
                new PROMedSDualAbsoluteQuadricRobustEstimator();
        
        //check default value
        assertEquals(estimator.getProgressDelta(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        
        //set new value
        estimator.setProgressDelta(0.1f);
        
        //check correctness
        assertEquals(estimator.getProgressDelta(), 0.1f, 0.0);
        
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
        PROMedSDualAbsoluteQuadricRobustEstimator estimator =
                new PROMedSDualAbsoluteQuadricRobustEstimator();
        
        //check default value
        assertEquals(estimator.getConfidence(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        
        //set new value
        estimator.setConfidence(0.8);
        
        //check correctness
        assertEquals(estimator.getConfidence(), 0.8, 0.0);
        
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
        PROMedSDualAbsoluteQuadricRobustEstimator estimator =
                new PROMedSDualAbsoluteQuadricRobustEstimator();
        
        //check default value
        assertEquals(estimator.getMaxIterations(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        
        //set new value
        estimator.setMaxIterations(100);
        
        //check correctness
        assertEquals(estimator.getMaxIterations(), 100);
        
        //Force IllegalArgumentException
        try {
            estimator.setMaxIterations(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
    }
    
    @Test
    public void testGetSetCameras() throws LockedException {
        PROMedSDualAbsoluteQuadricRobustEstimator estimator =
                new PROMedSDualAbsoluteQuadricRobustEstimator();
        
        //initial value
        assertNull(estimator.getCameras());
        
        //set new value
        List<PinholeCamera> cameras = new ArrayList<PinholeCamera>();
        cameras.add(new PinholeCamera());
        cameras.add(new PinholeCamera());
        estimator.setCameras(cameras);
        
        //check correctness
        assertSame(estimator.getCameras(), cameras);
        
        //Force IllegalArgumentException
        try {
            estimator.setCameras(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            estimator.setCameras(new ArrayList<PinholeCamera>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
    }
    
    @Test
    public void testGetMinNumberOfRequiredCameras() throws LockedException {
        PROMedSDualAbsoluteQuadricRobustEstimator estimator =
                new PROMedSDualAbsoluteQuadricRobustEstimator();
        
        //check default value
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 2);
        
        //disable principal point at origin
        estimator.setPrincipalPointAtOrigin(false);
        
        //check correctness
        assertEquals(estimator.getMinNumberOfRequiredCameras(), -1);
        
        //disable zero skewness
        estimator.setPrincipalPointAtOrigin(true);
        estimator.setZeroSkewness(false);
        
        //check correctness
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 4);
        
        
        //disable focal distance aspect ratio known
        estimator.setZeroSkewness(true);
        estimator.setFocalDistanceAspectRatioKnown(false);
        
        //check correctness
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 3);
        
        
        //disable zero skewness and singularity enforcement
        estimator.setZeroSkewness(false);
        estimator.setSingularityEnforced(false);
        
        //check correctness
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 5);
        
        
        //disable focal distance aspect ratio known and singularity enforcement
        estimator.setZeroSkewness(true);
        
        //check correctness
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 3);        
    }
    
    @Test
    public void testAreValidConstraints() throws LockedException {
        PROMedSDualAbsoluteQuadricRobustEstimator estimator =
                new PROMedSDualAbsoluteQuadricRobustEstimator();
        
        //check default value
        assertTrue(estimator.areValidConstraints());
        
        //disable principal point at origin
        estimator.setPrincipalPointAtOrigin(false);
        
        //check correctness
        assertFalse(estimator.areValidConstraints());
        
        
        //disable zero skewness
        estimator.setPrincipalPointAtOrigin(true);
        estimator.setZeroSkewness(false);
        
        //check correctness
        assertTrue(estimator.areValidConstraints());
        
        
        //disable focal distance aspect ratio known
        estimator.setZeroSkewness(true);
        estimator.setFocalDistanceAspectRatioKnown(false);
        
        //check correctness
        assertFalse(estimator.areValidConstraints());
        
        //disable singularity enforcement
        estimator.setSingularityEnforced(false);
        
        //check correctness
        assertTrue(estimator.areValidConstraints());        
    }
    
    @Test
    public void testIsReady() throws LockedException {
        PROMedSDualAbsoluteQuadricRobustEstimator estimator =
                new PROMedSDualAbsoluteQuadricRobustEstimator();
        
        //check default value
        assertNull(estimator.getCameras());
        assertTrue(estimator.areValidConstraints());        
        assertFalse(estimator.isReady());
        
        List<PinholeCamera> cameras = new ArrayList<PinholeCamera>();
        cameras.add(new PinholeCamera());
        cameras.add(new PinholeCamera());
        
        estimator.setCameras(cameras);
        estimator.setQualityScores(new double[cameras.size()]);
        
        //check correctness
        assertTrue(estimator.isReady());

        
        //disable principal point at origin
        estimator.setPrincipalPointAtOrigin(false);
        
        //check correctness
        assertFalse(estimator.areValidConstraints());
        assertFalse(estimator.isReady());

        //enable principal point at origin
        estimator.setPrincipalPointAtOrigin(true);
        
        //check correctness
        assertTrue(estimator.isReady());
        
        //clear cameras
        cameras.clear();

        //check correctness
        assertFalse(estimator.isReady());        
    }
    
    @Test
    public void testEstimate() throws AlgebraException, 
            InvalidTransformationException, LockedException, NotReadyException, 
            RobustEstimatorException, CameraException, NotAvailableException {
        
        int numSucceeded = 0;
        for (int t = 0; t < 10*TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
            //create ground truth intrinsic parameters
            double horizontalFocalLength, verticalFocalLength, skewness,
                    horizontalPrincipalPoint, verticalPrincipalPoint;
            double aspectRatio = 1.0;
            horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, 
                    MAX_FOCAL_LENGTH);
            verticalFocalLength = aspectRatio * horizontalFocalLength;
            skewness = horizontalPrincipalPoint = verticalPrincipalPoint = 0.0;
        
            PinholeCameraIntrinsicParameters metricIntrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                    verticalFocalLength, horizontalPrincipalPoint,
                    verticalPrincipalPoint, skewness);
        
            DualImageOfAbsoluteConic metricDiac = new DualImageOfAbsoluteConic(
                    metricIntrinsic);
            metricDiac.normalize();
            Matrix metricDiacMatrix = metricDiac.asMatrix();
        
            //generate random projective transformation to transform ground 
            //truth cameras
            Matrix T = Matrix.createWithUniformRandomValues(
                    ProjectiveTransformation3D.HOM_COORDS, 
                    ProjectiveTransformation3D.HOM_COORDS, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
        
            //ensure last element is not zero
            T.setElementAt(ProjectiveTransformation3D.HOM_COORDS - 1, 
                    ProjectiveTransformation3D.HOM_COORDS - 1, 1.0);        

            ProjectiveTransformation3D transformation = 
                    new ProjectiveTransformation3D(T);
        
            transformation.normalize();
        
            DualAbsoluteQuadric projectiveDaq = new DualAbsoluteQuadric(
                    transformation);
            projectiveDaq.normalize();
                
            Matrix projectiveDaqMatrix = projectiveDaq.asMatrix();
        
            double roll, pitch, yaw, x, y, z;
            Quaternion q;
            InhomogeneousPoint3D cameraCenter;
            PinholeCamera metricCamera, projectiveCamera;
            List<PinholeCamera> metricCameras = new ArrayList<PinholeCamera>();
            List<PinholeCamera> projectiveCameras = 
                    new ArrayList<PinholeCamera>();
        
            PROMedSDualAbsoluteQuadricRobustEstimator estimator =
                    new PROMedSDualAbsoluteQuadricRobustEstimator();
            estimator.setListener(this);
            estimator.setZeroSkewness(true);
            estimator.setPrincipalPointAtOrigin(true);
            estimator.setFocalDistanceAspectRatioKnown(true);
            estimator.setFocalDistanceAspectRatio(1.0);
            estimator.setSingularityEnforced(false);
        
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            boolean[] inliers = new boolean[NUM_CAMS];
            double[] qualityScores = new double[NUM_CAMS];
            for (int i = 0; i < NUM_CAMS; i++) {
                roll = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 
                        2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
                pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                        MAX_ANGLE_DEGREES) * Math.PI / 180.0;
                yaw = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 
                        2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;

                q = new Quaternion(roll, pitch, yaw);
            
                x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                z = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                cameraCenter = new InhomogeneousPoint3D(x, y, z);
            
                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    //outlier (add error to metric intrinsics)
                    inliers[i] = false;
                    double errorHorizontalFocalLength = 
                            errorRandomizer.nextDouble();
                    double errorAspectRatio = errorRandomizer.nextDouble();
                    double errorSkewness = errorRandomizer.nextDouble();
                    double errorHorizontalPrincipalPoint = 
                            errorRandomizer.nextDouble();
                    double errorVerticalPrincipalPoint =
                            errorRandomizer.nextDouble();
                    
                    double avgAbsError = (Math.abs(errorHorizontalFocalLength) + 
                            Math.abs(errorAspectRatio) + 
                            Math.abs(errorSkewness) +
                            Math.abs(errorHorizontalPrincipalPoint) +
                            Math.abs(errorVerticalPrincipalPoint)) / 5.0;
                    qualityScores[i] = 1.0 / (1.0 + avgAbsError);
                
                    double outlierHorizontalFocalLength = 
                            horizontalFocalLength + errorHorizontalFocalLength;
                    double outlierAspectRatio = aspectRatio + errorAspectRatio;
                    double outlierVerticalFocalLength = 
                            outlierAspectRatio * outlierHorizontalFocalLength;
                    double outlierSkewness = skewness + errorSkewness;
                    double outlierHorizontalPrincipalPoint = 
                            horizontalPrincipalPoint + 
                            errorHorizontalPrincipalPoint;
                    double outlierVerticalPrincipalPoint =
                            verticalPrincipalPoint + errorVerticalPrincipalPoint;
                
                    PinholeCameraIntrinsicParameters outlierMetricIntrinsic = 
                            new PinholeCameraIntrinsicParameters(
                            outlierHorizontalFocalLength,
                            outlierVerticalFocalLength, 
                            outlierHorizontalPrincipalPoint,
                            outlierVerticalPrincipalPoint, outlierSkewness);
                      
                    metricCamera = new PinholeCamera(outlierMetricIntrinsic, q, 
                            cameraCenter);
                } else {
                    //inlier
                    inliers[i] = true;
                    qualityScores[i] = 1.0;
                    metricCamera = new PinholeCamera(metricIntrinsic, q, 
                        cameraCenter);
                }
                metricCamera.normalize();   
                metricCameras.add(metricCamera);
            
                //transform camera
                projectiveCamera = transformation.transformAndReturnNew(
                        metricCamera);
            
                projectiveCameras.add(projectiveCamera);            
            }
        
            estimator.setCameras(projectiveCameras);
            estimator.setQualityScores(qualityScores);
        
            try {
                DualAbsoluteQuadric estimatedDaq = estimator.estimate();
                estimatedDaq.normalize();
                Matrix estimatedDaqMatrix = estimatedDaq.asMatrix();

                //check that DAQ has rank 3 (zero determinant)
                if(Math.abs(Utils.det(estimatedDaqMatrix)) > ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(Utils.det(estimatedDaqMatrix), 0.0, 
                        ABSOLUTE_ERROR);
                
                if(!projectiveDaqMatrix.equals(estimatedDaqMatrix, 
                        ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(projectiveDaqMatrix.equals(estimatedDaqMatrix, 
                        ABSOLUTE_ERROR));
                
                ProjectiveTransformation3D estimatedTransformation =
                        estimatedDaq.getMetricToProjectiveTransformation();
                estimatedTransformation.normalize();
                ProjectiveTransformation3D invEstimatedTransformation =
                        (ProjectiveTransformation3D)estimatedTransformation.
                                inverseAndReturnNew();
                
                //projected estimated DAQ using projective cameras to obtain 
                //DIAC and check that DIAC in projective stratum is equal to 
                //DIAC in metric stratum (for inlier cameras only)
                Point3D previousEstimatedMetricCenter = null;
                Point3D previousMetricCenter = null;
                Point3D estimatedMetricCenter, metricCenter;
                Rotation3D previousEstimatedMetricRotation = null;
                Rotation3D previousMetricRotation = null;
                Rotation3D estimatedMetricRotation, metricRotation;
                double distanceEstimatedCenter, distanceCenter;
                double previousScale = 1.0, scale = 1.0;                                
                PinholeCamera estimatedMetricCamera;
                boolean anyFailed = false;
                int count = 0;                
                for (int i = 0; i < NUM_CAMS; i++) {
                    if(!inliers[i]) continue;
                    
                    projectiveCamera = projectiveCameras.get(i);
                    
                    DualImageOfAbsoluteConic projectedProjectiveDiac =
                            new DualImageOfAbsoluteConic(projectiveCamera,
                            estimatedDaq);
                    projectedProjectiveDiac.normalize();
                    
                    Matrix projectedProjectiveDiacMatrix =
                            projectedProjectiveDiac.asMatrix();
                    
                    if(!metricDiacMatrix.equals(projectedProjectiveDiacMatrix, 
                            ABSOLUTE_ERROR)) {
                        anyFailed = true;
                        continue;
                    }
                    assertTrue(metricDiacMatrix.equals(
                            projectedProjectiveDiacMatrix, ABSOLUTE_ERROR));
                    
                    estimatedMetricCamera = invEstimatedTransformation.
                            transformAndReturnNew(projectiveCamera);
                    
                    estimatedMetricCamera.decompose();
                    PinholeCameraIntrinsicParameters estimatedIntrinsic =
                            estimatedMetricCamera.getIntrinsicParameters();
                    
                    assertEquals(horizontalFocalLength,
                            estimatedIntrinsic.getHorizontalFocalLength(),
                            5*LARGE_ABSOLUTE_ERROR);
                    assertEquals(verticalFocalLength,
                            estimatedIntrinsic.getVerticalFocalLength(),
                            5*LARGE_ABSOLUTE_ERROR);
                    assertEquals(skewness, estimatedIntrinsic.getSkewness(),
                            5*LARGE_ABSOLUTE_ERROR);
                    assertEquals(horizontalPrincipalPoint,
                            estimatedIntrinsic.getHorizontalPrincipalPoint(),
                            5*LARGE_ABSOLUTE_ERROR);
                    assertEquals(verticalPrincipalPoint,
                            estimatedIntrinsic.getVerticalPrincipalPoint(),
                            5*LARGE_ABSOLUTE_ERROR);
                    
                    if(anyFailed) continue;
                    
                    //check that when DAQ is successfully estimated, estimated
                    //metric cameras are in the metric stratum up to an 
                    //arbitrary scale
                    
                    metricCamera = metricCameras.get(i);
                    metricCamera.decompose();
                    
                    estimatedMetricCenter = estimatedMetricCamera.
                            getCameraCenter();
                    metricCenter = metricCamera.getCameraCenter();
                    estimatedMetricRotation = estimatedMetricCamera.
                            getCameraRotation();
                    metricRotation = metricCamera.getCameraRotation();
                                        
                    if(count > 0) {                                            
                        distanceEstimatedCenter = previousEstimatedMetricCenter.
                                distanceTo(estimatedMetricCenter);
                        distanceCenter = previousMetricCenter.distanceTo(
                                metricCenter);
                        scale = distanceEstimatedCenter / distanceCenter;
                        
                        Rotation3D diffEstimatedRotation =
                                estimatedMetricRotation.combineAndReturnNew(
                                previousEstimatedMetricRotation.
                                inverseRotationAndReturnNew());
                        Rotation3D diffRotation = metricRotation.
                                combineAndReturnNew(previousMetricRotation.
                                inverseRotationAndReturnNew());
                        
                        Matrix rot1 = diffEstimatedRotation.
                                asInhomogeneousMatrix();
                        Matrix rot2 = diffRotation.asInhomogeneousMatrix();
                        assertTrue(rot1.equals(rot2, LARGE_ABSOLUTE_ERROR));
                    }
                    
                    if(count > 1) {
                        assertEquals(scale, previousScale, 5*LARGE_ABSOLUTE_ERROR);                    
                    }
                                        
                    previousEstimatedMetricCenter = estimatedMetricCenter;
                    previousMetricCenter = metricCenter;
                    previousScale = scale;         
                    
                    previousEstimatedMetricRotation = estimatedMetricRotation;
                    previousMetricRotation = metricRotation;                    
                    
                    count++;                    
                }
                
                numSucceeded++;
                
                if (numSucceeded > 0) {
                    break;
                }
            }  catch (RobustEstimatorException ex) { }
        }
        
        //sometimes if cameras are in degenerate configurations, DAQ estimation
        //can fail, for that reason we check that algorithm at least works one
        //if we retry multiple times.
        assertTrue(numSucceeded > 0);
    }
    
    @Override
    public void onEstimateStart(DualAbsoluteQuadricRobustEstimator estimator) {
        assertTrue(estimator.isLocked());
        checkLocked(estimator);
    }

    @Override
    public void onEstimateEnd(DualAbsoluteQuadricRobustEstimator estimator) {
        assertTrue(estimator.isLocked());
        checkLocked(estimator);
    }

    @Override
    public void onEstimateNextIteration(
            DualAbsoluteQuadricRobustEstimator estimator, int iteration) {
        checkLocked(estimator);
    }

    @Override
    public void onEstimateProgressChange(
            DualAbsoluteQuadricRobustEstimator estimator, float progress) {
        checkLocked(estimator);
    }
    
    private void checkLocked(DualAbsoluteQuadricRobustEstimator estimator) {
        PROMedSDualAbsoluteQuadricRobustEstimator promedsEstimator =
                (PROMedSDualAbsoluteQuadricRobustEstimator)estimator;
        
        try {
            promedsEstimator.setStopThreshold(1.0);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            promedsEstimator.setQualityScores(new double[2]);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            promedsEstimator.setZeroSkewness(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            promedsEstimator.setPrincipalPointAtOrigin(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            promedsEstimator.setFocalDistanceAspectRatioKnown(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            promedsEstimator.setFocalDistanceAspectRatio(2.0);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            promedsEstimator.setSingularityEnforced(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            promedsEstimator.setEnforcedSingularityValidated(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            promedsEstimator.setDeterminantThreshold(1e-3);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            promedsEstimator.setListener(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            promedsEstimator.setProgressDelta(0.1f);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            promedsEstimator.setConfidence(0.8);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            promedsEstimator.setMaxIterations(100);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            promedsEstimator.setCameras(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            promedsEstimator.estimate();
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { 
        } catch (Exception e) {
            fail("LockedException expected but not thrown");
        }
    }    
}
