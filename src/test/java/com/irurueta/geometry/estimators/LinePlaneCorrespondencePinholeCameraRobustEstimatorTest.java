/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.estimators.LinePlaneCorrespondencePinholeCameraRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date March 12, 2015
 */
package com.irurueta.geometry.estimators;

import com.irurueta.geometry.Line2D;
import com.irurueta.geometry.Plane;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import java.util.ArrayList;
import java.util.List;
import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class LinePlaneCorrespondencePinholeCameraRobustEstimatorTest {
    
    public LinePlaneCorrespondencePinholeCameraRobustEstimatorTest() { }
    
    @BeforeClass
    public static void setUpClass() { }
    
    @AfterClass
    public static void tearDownClass() { }
    
    @Before
    public void setUp() { }
    
    @After
    public void tearDown() { }

    @Test
    public void testCreate() {
        LinePlaneCorrespondencePinholeCameraRobustEstimator estimator;
        
        //create with robust estimator method
        estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getLines());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getCovariance());
        
        
        estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getLines());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);        
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getCovariance());

        
        estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getLines());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);        
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getCovariance());

        
        estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getLines());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);        
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getCovariance());

        
        estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getLines());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);        
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getCovariance());
        
        //create with robust estimator method and lines and planes
        List<Plane> planes = new ArrayList<Plane>();
        List<Line2D> lines = new ArrayList<Line2D>();
        for (int i = 0; i < LinePlaneCorrespondencePinholeCameraRobustEstimator.MIN_NUMBER_OF_LINE_PLANE_CORRESPONDENCES; i++) {
            planes.add(new Plane());
            lines.add(new Line2D());
        }
        
        
        estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                planes, lines, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getLines(), lines);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getCovariance());

        
        estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                planes, lines, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getLines(), lines);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getCovariance());

        
        estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                planes, lines, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getLines(), lines);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getCovariance());

        
        estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                planes, lines, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getLines(), lines);
        assertSame(estimator.getPlanes(), planes);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getCovariance());

        
        estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                planes, lines, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getLines(), lines);
        assertSame(estimator.getPlanes(), planes);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getCovariance());
        
        //Force IllegalArgumentException
        List<Plane> emptyPlanes = new ArrayList<Plane>();
        List<Line2D> emptyLines = new ArrayList<Line2D>();
        estimator = null;
        try {
            //empty points
            estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.
                    create(emptyPlanes, emptyLines, 
                    RobustEstimatorMethod.RANSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            //different sizes
            estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.
                    create(planes, emptyLines, RobustEstimatorMethod.RANSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        assertNull(estimator);
        
        //create with listener and robust estimator method
        PinholeCameraRobustEstimatorListener listener =
                new PinholeCameraRobustEstimatorListener(){

            @Override
            public void onEstimateStart(PinholeCameraRobustEstimator estimator) {}

            @Override
            public void onEstimateEnd(PinholeCameraRobustEstimator estimator) {}

            @Override
            public void onEstimateNextIteration(PinholeCameraRobustEstimator estimator, int iteration) {}

            @Override
            public void onEstimateProgressChange(PinholeCameraRobustEstimator estimator, float progress) {}
        };
        
        estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                listener, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getLines());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getCovariance());

        
        estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                listener, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getLines());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getCovariance());

        
        estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                listener, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getLines());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getCovariance());

        
        estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                listener, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getLines());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getCovariance());

        
        estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                listener, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getLines());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getCovariance());

        
        //test with listener, points and robust estimator method
        estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                listener, planes, lines, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getLines(), lines);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getCovariance());

        
        estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                listener, planes, lines, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getLines(), lines);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getCovariance());

        
        estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                listener, planes, lines, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getLines(), lines);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getCovariance());

        
        estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                listener, planes, lines, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getLines(), lines);
        assertSame(estimator.getPlanes(), planes);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getCovariance());

        
        estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                listener, planes, lines, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getLines(), lines);
        assertSame(estimator.getPlanes(), planes);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getCovariance());

        
        //test with quality scores and robust estimator method
        double[] qualityScores = new double[
                LinePlaneCorrespondencePinholeCameraRobustEstimator.
                MIN_NUMBER_OF_LINE_PLANE_CORRESPONDENCES];
        double[] shortScores = new double[1];
        
        estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                qualityScores, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getLines());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getCovariance());

        
        estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                qualityScores, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getLines());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getCovariance());

        
        estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                qualityScores, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getLines());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getCovariance());

        
        estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                qualityScores, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getLines());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getCovariance());

        
        estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                qualityScores, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getLines());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getCovariance());
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.
                    create(shortScores, RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        assertNull(estimator);
        
        //test with points, quality scores and robust estimator method
        estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                planes, lines, qualityScores, 
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getLines(), lines);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getCovariance());

        
        estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                planes, lines, qualityScores, 
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getLines(), lines);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getCovariance());

        
        estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                planes, lines, qualityScores, 
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getLines(), lines);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getCovariance());

        
        estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                planes, lines, qualityScores, 
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getLines(), lines);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getCovariance());

        
        estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                planes, lines, qualityScores, 
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getLines(), lines);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getCovariance());
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            //empty points
            estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.
                    create(emptyPlanes, emptyLines, qualityScores, 
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            //different sizes
            estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.
                    create(planes, emptyLines, qualityScores, 
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            //short scores
            estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.
                    create(planes, lines, shortScores, 
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        assertNull(estimator);
        
        //test with listener, quality scores and method
        estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                listener, qualityScores, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getLines());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getCovariance());

        
        estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                listener, qualityScores, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getLines());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getCovariance());

        
        estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                listener, qualityScores, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getLines());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getCovariance());

        
        estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                listener, qualityScores, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getLines());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getCovariance());

        
        estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                listener, qualityScores, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getLines());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getCovariance());
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.
                    create(listener, shortScores, RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        assertNull(estimator);
        
        //test with listener, points, quality scores and method
        estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                listener, planes, lines, qualityScores, 
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getLines(), lines);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getCovariance());

        
        estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                listener, planes, lines, qualityScores, 
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getLines(), lines);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getCovariance());

        
        estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                listener, planes, lines, qualityScores, 
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getLines(), lines);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getCovariance());

        
        estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                listener, planes, lines, qualityScores, 
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getLines(), lines);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getCovariance());

        
        estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                listener, planes, lines, qualityScores, 
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getLines(), lines);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getCovariance());
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            //empty points
            estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.
                    create(listener, emptyPlanes, emptyLines, qualityScores, 
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            //different sizes
            estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.
                    create(listener, planes, emptyLines, qualityScores, 
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            //short scores
            estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.
                    create(listener, planes, lines, shortScores, 
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        assertNull(estimator);
        
        //test with no arguments
        estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.
                create();
        assertTrue(estimator instanceof
                PROMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getLines());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);        
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getCovariance());

        
        //test with points
        estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                planes, lines);
        assertTrue(estimator instanceof
                PROMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getLines(), lines);
        assertSame(estimator.getPlanes(), planes);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);        
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getCovariance());
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            //empty points
            estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.
                    create(emptyPlanes, emptyLines);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            //different sizes
            estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.
                    create(planes, emptyLines);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        assertNull(estimator);     
        
        
        //test with listener
        estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                listener);
        assertTrue(estimator instanceof
                PROMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getLines());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);        
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getCovariance());

        
        //test with listener and points
        estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                listener, planes, lines);
        assertTrue(estimator instanceof
                PROMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getLines(), lines);
        assertSame(estimator.getPlanes(), planes);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);        
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getCovariance());
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            //empty points
            estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.
                    create(listener, emptyPlanes, emptyLines);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            //different sizes
            estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.
                    create(listener, planes, emptyLines);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        assertNull(estimator);     
        
        
        //test with quality scores
        estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                qualityScores);
        assertTrue(estimator instanceof
                PROMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getLines());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);        
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getCovariance());
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.
                    create(shortScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        assertNull(estimator);
        
        
        //test with points and quality scores
        estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                planes, lines, qualityScores);
        assertTrue(estimator instanceof
                PROMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getLines(), lines);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);        
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getCovariance());
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            //empty points
            estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.
                    create(emptyPlanes, emptyLines, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            //different sizes
            estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.
                    create(planes, emptyLines, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            //short scores
            estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.
                    create(planes, lines, shortScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        assertNull(estimator);     
        
        
        //test with listener and scores
        estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                listener, qualityScores);
        assertTrue(estimator instanceof
                PROMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getLines());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);        
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getCovariance());
        
        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.
                    create(listener, shortScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        assertNull(estimator);
        
        
        //test with listener, points and quality scores
        estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                listener, planes, lines, qualityScores);
        assertTrue(estimator instanceof
                PROMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getLines(), lines);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);        
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getCovariance());
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            //empty points
            estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.
                    create(emptyPlanes, emptyLines, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            //different sizes
            estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.
                    create(planes, emptyLines, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            //short scores
            estimator = LinePlaneCorrespondencePinholeCameraRobustEstimator.
                    create(planes, lines, shortScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        assertNull(estimator);        
    }
}