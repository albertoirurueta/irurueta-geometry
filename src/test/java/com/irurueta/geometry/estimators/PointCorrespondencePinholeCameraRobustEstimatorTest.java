/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.estimators.PointCorrespondencePinholeCameraRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date March 10, 2015
 */
package com.irurueta.geometry.estimators;

import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.Point3D;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import java.util.ArrayList;
import java.util.List;
import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class PointCorrespondencePinholeCameraRobustEstimatorTest {
    
    public PointCorrespondencePinholeCameraRobustEstimatorTest() {}
    
    @BeforeClass
    public static void setUpClass() {}
    
    @AfterClass
    public static void tearDownClass() {}
    
    @Before
    public void setUp() {}
    
    @After
    public void tearDown() {}

    @Test
    public void testCreate(){
        PointCorrespondencePinholeCameraRobustEstimator estimator;
        
        //create with robust estimator method
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
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

        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
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
     
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
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
        
        //create with robust estimator method and points
        List<Point3D> points3D = new ArrayList<Point3D>();
        List<Point2D> points2D = new ArrayList<Point2D>();
        for(int i = 0; i < PointCorrespondencePinholeCameraRobustEstimator.MIN_NUMBER_OF_POINT_CORRESPONDENCES; i++){
            points3D.add(Point3D.create());
            points2D.add(Point2D.create());
        }
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, points2D, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
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

        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, points2D, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, points2D, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                MSACDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, points2D, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, points2D, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
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
        
        //Force IllegalArgumentException
        List<Point3D> empty3D = new ArrayList<Point3D>();
        List<Point2D> empty2D = new ArrayList<Point2D>();
        estimator = null;
        try{
            //empty points
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    empty3D, empty2D, RobustEstimatorMethod.RANSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //different sizes
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    points3D, empty2D, RobustEstimatorMethod.RANSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}        
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
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

        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                MSACDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
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
        
        //test with listener, points and robust estimator method
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
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

        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                MSACDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
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
        
        //test with quality scores and robust estimator method
        double[] qualityScores = new double[
                PointCorrespondencePinholeCameraRobustEstimator.
                MIN_NUMBER_OF_POINT_CORRESPONDENCES];
        double[] shortScores = new double[1];
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                qualityScores, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
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

        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                qualityScores, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                qualityScores, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                MSACDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                qualityScores, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                qualityScores, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
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
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    shortScores, RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with points, quality scores and robust estimator method
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, points2D, qualityScores, 
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
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

        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, points2D, qualityScores, 
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, points2D, qualityScores, 
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                MSACDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, points2D, qualityScores, 
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
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

        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, points2D, qualityScores, 
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
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
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            //empty points
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    empty3D, empty2D, qualityScores, 
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //different sizes
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    points3D, empty2D, qualityScores, 
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //short scores
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    points3D, points2D, shortScores, 
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with listener, quality scores and method
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, qualityScores, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
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

        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, qualityScores, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, qualityScores, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                MSACDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, qualityScores, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, qualityScores, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
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
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    listener, shortScores, RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with listener, points, quality scores and method
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, qualityScores, 
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
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

        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, qualityScores, 
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, qualityScores, 
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                MSACDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, qualityScores, 
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, qualityScores, 
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
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
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            //empty points
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    listener, empty3D, empty2D, qualityScores, 
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //different sizes
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    listener, points3D, empty2D, qualityScores, 
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //short scores
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    listener, points3D, points2D, shortScores, 
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with no arguments
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create();
        assertTrue(estimator instanceof
                PROMedSDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
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

        //test with points
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, points2D);
        assertTrue(estimator instanceof
                PROMedSDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
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
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            //empty points
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    empty3D, empty2D);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //different sizes
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    points3D, empty2D);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);     
        
        //test with listener
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener);
        assertTrue(estimator instanceof
                PROMedSDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
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
        
        //test with listener and points
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D);
        assertTrue(estimator instanceof
                PROMedSDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
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
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            //empty points
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    listener, empty3D, empty2D);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //different sizes
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    listener, points3D, empty2D);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);     
        
        //test with quality scores
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                qualityScores);
        assertTrue(estimator instanceof
                PROMedSDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
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
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    shortScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with points and quality scores
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, points2D, qualityScores);
        assertTrue(estimator instanceof
                PROMedSDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
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
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            //empty points
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    empty3D, empty2D, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //different sizes
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    points3D, empty2D, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //short scores
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    points3D, points2D, shortScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);     
        
        //test with listener and scores
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, qualityScores);
        assertTrue(estimator instanceof
                PROMedSDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
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
        
        //force IllegalArgumentException
        estimator = null;
        try{
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    listener, shortScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with listener, points and quality scores
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, qualityScores);
        assertTrue(estimator instanceof
                PROMedSDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
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
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            //empty points
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    empty3D, empty2D, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //different sizes
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    points3D, empty2D, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //short scores
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    points3D, points2D, shortScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);     
        
    }
    
    @Test
    public void testCreateWithIntrinsic(){
        PinholeCameraIntrinsicParameters intrinsic = 
                new PinholeCameraIntrinsicParameters();
        
        PointCorrespondencePinholeCameraRobustEstimator estimator;
        
        //create with robust estimator method
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                intrinsic, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                intrinsic, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                intrinsic, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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

        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                intrinsic, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
     
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                intrinsic, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        //create with robust estimator method and points
        List<Point3D> points3D = new ArrayList<Point3D>();
        List<Point2D> points2D = new ArrayList<Point2D>();
        for(int i = 0; i < PointCorrespondencePinholeCameraRobustEstimator.MIN_NUMBER_OF_POINT_CORRESPONDENCES; i++){
            points3D.add(Point3D.create());
            points2D.add(Point2D.create());
        }
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                intrinsic, points3D, points2D, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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

        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                intrinsic, points3D, points2D, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                intrinsic, points3D, points2D, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                MSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                intrinsic, points3D, points2D, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                intrinsic, points3D, points2D, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        //Force IllegalArgumentException
        List<Point3D> empty3D = new ArrayList<Point3D>();
        List<Point2D> empty2D = new ArrayList<Point2D>();
        estimator = null;
        try{
            //empty points
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    intrinsic, empty3D, empty2D, RobustEstimatorMethod.RANSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //different sizes
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    intrinsic, points3D, empty2D, RobustEstimatorMethod.RANSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}        
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, intrinsic, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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

        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, intrinsic, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, intrinsic, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                MSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, intrinsic, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, intrinsic, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        //test with listener, points and robust estimator method
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, intrinsic, points3D, points2D, 
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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

        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, intrinsic, points3D, points2D, 
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, intrinsic, points3D, points2D, 
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                MSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, intrinsic, points3D, points2D, 
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, intrinsic, points3D, points2D, 
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        //test with quality scores and robust estimator method
        double[] qualityScores = new double[
                PointCorrespondencePinholeCameraRobustEstimator.
                MIN_NUMBER_OF_POINT_CORRESPONDENCES];
        double[] shortScores = new double[1];
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                intrinsic, qualityScores, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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

        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                intrinsic, qualityScores, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                intrinsic, qualityScores, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                MSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                intrinsic, qualityScores, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                intrinsic, qualityScores, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    intrinsic, shortScores, RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with points, quality scores and robust estimator method
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                intrinsic, points3D, points2D, qualityScores, 
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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

        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                intrinsic, points3D, points2D, qualityScores, 
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                intrinsic, points3D, points2D, qualityScores, 
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                MSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                intrinsic, points3D, points2D, qualityScores, 
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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

        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                intrinsic, points3D, points2D, qualityScores, 
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            //empty points
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    intrinsic, empty3D, empty2D, qualityScores, 
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //different sizes
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    intrinsic, points3D, empty2D, qualityScores, 
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //short scores
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    intrinsic, points3D, points2D, shortScores, 
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with listener, quality scores and method
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, intrinsic, qualityScores, 
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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

        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, intrinsic, qualityScores, 
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, intrinsic, qualityScores, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                MSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, intrinsic, qualityScores, 
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, intrinsic, qualityScores, 
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    listener, intrinsic, shortScores, 
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with listener, points, quality scores and method
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, intrinsic, points3D, points2D, qualityScores, 
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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

        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, intrinsic, points3D, points2D, qualityScores, 
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, intrinsic, points3D, points2D, qualityScores, 
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                MSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, intrinsic, points3D, points2D, qualityScores, 
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, intrinsic, points3D, points2D, qualityScores, 
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            //empty points
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    listener, intrinsic, empty3D, empty2D, qualityScores, 
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //different sizes
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    listener, intrinsic, points3D, empty2D, qualityScores, 
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //short scores
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    listener, intrinsic, points3D, points2D, shortScores, 
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with no arguments
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                intrinsic);
        assertTrue(estimator instanceof
                PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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

        //test with points
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                intrinsic, points3D, points2D);
        assertTrue(estimator instanceof
                PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            //empty points
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    intrinsic, empty3D, empty2D);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //different sizes
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    intrinsic, points3D, empty2D);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);     
        
        //test with listener
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, intrinsic);
        assertTrue(estimator instanceof
                PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        //test with listener and points
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, intrinsic, points3D, points2D);
        assertTrue(estimator instanceof
                PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            //empty points
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    listener, intrinsic, empty3D, empty2D);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //different sizes
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    listener, intrinsic, points3D, empty2D);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);     
        
        //test with quality scores
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                intrinsic, qualityScores);
        assertTrue(estimator instanceof
                PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    intrinsic, shortScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with points and quality scores
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                intrinsic, points3D, points2D, qualityScores);
        assertTrue(estimator instanceof
                PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            //empty points
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    intrinsic, empty3D, empty2D, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //different sizes
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    intrinsic, points3D, empty2D, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //short scores
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    intrinsic, points3D, points2D, shortScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);     
        
        //test with listener and scores
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, intrinsic, qualityScores);
        assertTrue(estimator instanceof
                PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        //force IllegalArgumentException
        estimator = null;
        try{
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    listener, intrinsic, shortScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with listener, points and quality scores
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, intrinsic, points3D, points2D, qualityScores);
        assertTrue(estimator instanceof
                PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            //empty points
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    intrinsic, empty3D, empty2D, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //different sizes
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    intrinsic, points3D, empty2D, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //short scores
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    intrinsic, points3D, points2D, shortScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);             
    }    
    
    @Test
    public void testCreateWithSkewnessAndPrincipalPoint(){
        double skewness = 0.0;
        double horizontalPrincipalPoint = 0.0;
        double verticalPrincipalPoint = 0.0;
        
        PointCorrespondencePinholeCameraRobustEstimator estimator;
        
        //create with robust estimator method
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                skewness, horizontalPrincipalPoint, verticalPrincipalPoint, 
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                skewness, horizontalPrincipalPoint, verticalPrincipalPoint, 
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                skewness, horizontalPrincipalPoint, verticalPrincipalPoint, 
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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

        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                skewness, horizontalPrincipalPoint, verticalPrincipalPoint, 
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
     
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                skewness, horizontalPrincipalPoint, verticalPrincipalPoint, 
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        //create with robust estimator method and points
        List<Point3D> points3D = new ArrayList<Point3D>();
        List<Point2D> points2D = new ArrayList<Point2D>();
        for(int i = 0; i < PointCorrespondencePinholeCameraRobustEstimator.MIN_NUMBER_OF_POINT_CORRESPONDENCES; i++){
            points3D.add(Point3D.create());
            points2D.add(Point2D.create());
        }
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                skewness, horizontalPrincipalPoint, verticalPrincipalPoint, 
                points3D, points2D, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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

        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                skewness, horizontalPrincipalPoint, verticalPrincipalPoint, 
                points3D, points2D, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                skewness, horizontalPrincipalPoint, verticalPrincipalPoint, 
                points3D, points2D, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                MSACUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                skewness, horizontalPrincipalPoint, verticalPrincipalPoint, 
                points3D, points2D, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                skewness, horizontalPrincipalPoint, verticalPrincipalPoint, 
                points3D, points2D, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        //Force IllegalArgumentException
        List<Point3D> empty3D = new ArrayList<Point3D>();
        List<Point2D> empty2D = new ArrayList<Point2D>();
        estimator = null;
        try{
            //empty points
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    skewness, horizontalPrincipalPoint, verticalPrincipalPoint, 
                    empty3D, empty2D, RobustEstimatorMethod.RANSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //different sizes
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    skewness, horizontalPrincipalPoint, verticalPrincipalPoint, 
                    points3D, empty2D, RobustEstimatorMethod.RANSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}        
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, skewness, horizontalPrincipalPoint, 
                verticalPrincipalPoint, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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

        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, skewness, horizontalPrincipalPoint, 
                verticalPrincipalPoint, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, skewness, horizontalPrincipalPoint, 
                verticalPrincipalPoint, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                MSACUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, skewness, horizontalPrincipalPoint, 
                verticalPrincipalPoint, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, skewness, horizontalPrincipalPoint, 
                verticalPrincipalPoint, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        //test with listener, points and robust estimator method
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, skewness, horizontalPrincipalPoint, 
                verticalPrincipalPoint, points3D, points2D, 
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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

        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, skewness, horizontalPrincipalPoint, 
                verticalPrincipalPoint, points3D, points2D, 
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, skewness, horizontalPrincipalPoint, 
                verticalPrincipalPoint, points3D, points2D, 
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                MSACUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, skewness, horizontalPrincipalPoint, 
                verticalPrincipalPoint, points3D, points2D, 
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, skewness, horizontalPrincipalPoint, 
                verticalPrincipalPoint, points3D, points2D, 
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        //test with quality scores and robust estimator method
        double[] qualityScores = new double[
                PointCorrespondencePinholeCameraRobustEstimator.
                MIN_NUMBER_OF_POINT_CORRESPONDENCES];
        double[] shortScores = new double[1];
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                skewness, horizontalPrincipalPoint, verticalPrincipalPoint, 
                qualityScores, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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

        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                skewness, horizontalPrincipalPoint, verticalPrincipalPoint, 
                qualityScores, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                skewness, horizontalPrincipalPoint, verticalPrincipalPoint, 
                qualityScores, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                MSACUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                skewness, horizontalPrincipalPoint, verticalPrincipalPoint, 
                qualityScores, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                skewness, horizontalPrincipalPoint, verticalPrincipalPoint, 
                qualityScores, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    skewness, horizontalPrincipalPoint, verticalPrincipalPoint, 
                    shortScores, RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with points, quality scores and robust estimator method
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                skewness, horizontalPrincipalPoint, verticalPrincipalPoint, 
                points3D, points2D, qualityScores, 
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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

        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                skewness, horizontalPrincipalPoint, verticalPrincipalPoint, 
                points3D, points2D, qualityScores, 
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                skewness, horizontalPrincipalPoint, verticalPrincipalPoint, 
                points3D, points2D, qualityScores, 
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                MSACUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                skewness, horizontalPrincipalPoint, verticalPrincipalPoint, 
                points3D, points2D, qualityScores, 
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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

        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                skewness, horizontalPrincipalPoint, verticalPrincipalPoint, 
                points3D, points2D, qualityScores, 
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            //empty points
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    skewness, horizontalPrincipalPoint, verticalPrincipalPoint, 
                    empty3D, empty2D, qualityScores, 
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //different sizes
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    skewness, horizontalPrincipalPoint, verticalPrincipalPoint, 
                    points3D, empty2D, qualityScores, 
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //short scores
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    skewness, horizontalPrincipalPoint, verticalPrincipalPoint, 
                    points3D, points2D, shortScores, 
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with listener, quality scores and method
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, skewness, horizontalPrincipalPoint, 
                verticalPrincipalPoint, qualityScores, 
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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

        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, skewness, horizontalPrincipalPoint, 
                verticalPrincipalPoint, qualityScores, 
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, skewness, horizontalPrincipalPoint, 
                verticalPrincipalPoint, qualityScores, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                MSACUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, skewness, horizontalPrincipalPoint, 
                verticalPrincipalPoint, qualityScores, 
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, skewness, horizontalPrincipalPoint, 
                verticalPrincipalPoint, qualityScores, 
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    listener, skewness, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, shortScores, 
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with listener, points, quality scores and method
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, skewness, horizontalPrincipalPoint, 
                verticalPrincipalPoint, points3D, points2D, qualityScores, 
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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

        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, skewness, horizontalPrincipalPoint, 
                verticalPrincipalPoint, points3D, points2D, qualityScores, 
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, skewness, horizontalPrincipalPoint, 
                verticalPrincipalPoint, points3D, points2D, qualityScores, 
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                MSACUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, skewness, horizontalPrincipalPoint, 
                verticalPrincipalPoint, points3D, points2D, qualityScores, 
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, skewness, horizontalPrincipalPoint, 
                verticalPrincipalPoint, points3D, points2D, qualityScores, 
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            //empty points
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    listener, skewness, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, empty3D, empty2D, qualityScores, 
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //different sizes
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    listener, skewness, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, points3D, empty2D, qualityScores, 
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //short scores
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    listener, skewness, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, points3D, points2D, shortScores, 
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with no arguments
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                skewness, horizontalPrincipalPoint, verticalPrincipalPoint);
        assertTrue(estimator instanceof
                PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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

        //test with points
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                skewness, horizontalPrincipalPoint, verticalPrincipalPoint, 
                points3D, points2D);
        assertTrue(estimator instanceof
                PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            //empty points
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    skewness, horizontalPrincipalPoint, verticalPrincipalPoint, 
                    empty3D, empty2D);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //different sizes
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    skewness, horizontalPrincipalPoint, verticalPrincipalPoint,
                    points3D, empty2D);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);     
        
        //test with listener
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, skewness, horizontalPrincipalPoint, 
                verticalPrincipalPoint);
        assertTrue(estimator instanceof
                PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        //test with listener and points
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, skewness, horizontalPrincipalPoint, 
                verticalPrincipalPoint, points3D, points2D);
        assertTrue(estimator instanceof
                PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            //empty points
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    listener, skewness, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, empty3D, empty2D);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //different sizes
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    listener, skewness, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, points3D, empty2D);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);     
        
        //test with quality scores
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                skewness, horizontalPrincipalPoint, verticalPrincipalPoint, 
                qualityScores);
        assertTrue(estimator instanceof
                PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    skewness, horizontalPrincipalPoint, verticalPrincipalPoint, 
                    shortScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with points and quality scores
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                skewness, horizontalPrincipalPoint, verticalPrincipalPoint, 
                points3D, points2D, qualityScores);
        assertTrue(estimator instanceof
                PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            //empty points
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    skewness, horizontalPrincipalPoint, verticalPrincipalPoint, 
                    empty3D, empty2D, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //different sizes
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    skewness, horizontalPrincipalPoint, verticalPrincipalPoint, 
                    points3D, empty2D, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //short scores
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    skewness, horizontalPrincipalPoint, verticalPrincipalPoint, 
                    points3D, points2D, shortScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);     
        
        //test with listener and scores
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, skewness, horizontalPrincipalPoint, 
                verticalPrincipalPoint, qualityScores);
        assertTrue(estimator instanceof
                PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        //force IllegalArgumentException
        estimator = null;
        try{
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    listener, skewness, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, shortScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with listener, points and quality scores
        estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                listener, skewness, horizontalPrincipalPoint, 
                verticalPrincipalPoint, points3D, points2D, qualityScores);
        assertTrue(estimator instanceof
                PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
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
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            //empty points
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    skewness, horizontalPrincipalPoint, verticalPrincipalPoint, 
                    empty3D, empty2D, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //different sizes
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    skewness, horizontalPrincipalPoint, verticalPrincipalPoint, 
                    points3D, empty2D, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //short scores
            estimator = PointCorrespondencePinholeCameraRobustEstimator.create(
                    skewness, horizontalPrincipalPoint, verticalPrincipalPoint, 
                    points3D, points2D, shortScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);             
    }    
    
}
