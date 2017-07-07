/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.estimators.UPnPPointCorrespondencePinholeCameraRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date March 8, 2017.
 */
package com.irurueta.geometry.estimators;

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

public class UPnPPointCorrespondencePinholeCameraRobustEstimatorTest {
    
    public UPnPPointCorrespondencePinholeCameraRobustEstimatorTest() { }
    
    @BeforeClass
    public static void setUpClass() { }
    
    @AfterClass
    public static void tearDownClass() { }
    
    @Before
    public void setUp() { }
    
    @After
    public void tearDown() { }

    @Test
    public void testCreate(){
        UPnPPointCorrespondencePinholeCameraRobustEstimator estimator;
        
        //create with robust estimator method
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
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
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(estimator.getPlanarThreshold(), 
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.getSkewness(), 0.0, 0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(), 0.0, 0.0);
        
        
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
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
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(estimator.getPlanarThreshold(), 
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.getSkewness(), 0.0, 0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(), 0.0, 0.0);

        
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
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
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(estimator.getPlanarThreshold(), 
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.getSkewness(), 0.0, 0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(), 0.0, 0.0);


        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
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
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(estimator.getPlanarThreshold(), 
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.getSkewness(), 0.0, 0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(), 0.0, 0.0);

     
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
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
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(estimator.getPlanarThreshold(), 
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.getSkewness(), 0.0, 0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(), 0.0, 0.0);

        
        //create with robust estimator method and points
        List<Point3D> points3D = new ArrayList<Point3D>();
        List<Point2D> points2D = new ArrayList<Point2D>();
        for(int i = 0; i < PointCorrespondencePinholeCameraRobustEstimator.MIN_NUMBER_OF_POINT_CORRESPONDENCES; i++){
            points3D.add(Point3D.create());
            points2D.add(Point2D.create());
        }
        
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
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
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(estimator.getPlanarThreshold(), 
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.getSkewness(), 0.0, 0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(), 0.0, 0.0);


        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
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
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(estimator.getPlanarThreshold(), 
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.getSkewness(), 0.0, 0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(), 0.0, 0.0);

        
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
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
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(estimator.getPlanarThreshold(), 
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.getSkewness(), 0.0, 0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(), 0.0, 0.0);

        
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
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
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(estimator.getPlanarThreshold(), 
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.getSkewness(), 0.0, 0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(), 0.0, 0.0);

        
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
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
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(estimator.getPlanarThreshold(), 
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.getSkewness(), 0.0, 0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(), 0.0, 0.0);
        
        //Force IllegalArgumentException
        List<Point3D> empty3D = new ArrayList<Point3D>();
        List<Point2D> empty2D = new ArrayList<Point2D>();
        estimator = null;
        try{
            //empty points
            estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(empty3D, empty2D, RobustEstimatorMethod.RANSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //different sizes
            estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(points3D, empty2D, RobustEstimatorMethod.RANSAC);
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
        
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, RobustEstimatorMethod.RANSAC);
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
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(estimator.getPlanarThreshold(), 
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.getSkewness(), 0.0, 0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(), 0.0, 0.0);


        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, RobustEstimatorMethod.LMedS);
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
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(estimator.getPlanarThreshold(), 
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.getSkewness(), 0.0, 0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(), 0.0, 0.0);

        
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, RobustEstimatorMethod.MSAC);
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
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(estimator.getPlanarThreshold(), 
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.getSkewness(), 0.0, 0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(), 0.0, 0.0);

        
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, RobustEstimatorMethod.PROSAC);
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
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(estimator.getPlanarThreshold(), 
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.getSkewness(), 0.0, 0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(), 0.0, 0.0);

        
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, RobustEstimatorMethod.PROMedS);
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
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(estimator.getPlanarThreshold(), 
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.getSkewness(), 0.0, 0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(), 0.0, 0.0);

        
        //test with listener, points and robust estimator method
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, RobustEstimatorMethod.RANSAC);
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
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(estimator.getPlanarThreshold(), 
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.getSkewness(), 0.0, 0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(), 0.0, 0.0);


        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, RobustEstimatorMethod.LMedS);
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
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(estimator.getPlanarThreshold(), 
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.getSkewness(), 0.0, 0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(), 0.0, 0.0);

        
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, RobustEstimatorMethod.MSAC);
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
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(estimator.getPlanarThreshold(), 
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.getSkewness(), 0.0, 0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(), 0.0, 0.0);

        
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, RobustEstimatorMethod.PROSAC);
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
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(estimator.getPlanarThreshold(), 
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.getSkewness(), 0.0, 0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(), 0.0, 0.0);

        
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, RobustEstimatorMethod.PROMedS);
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
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(estimator.getPlanarThreshold(), 
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.getSkewness(), 0.0, 0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(), 0.0, 0.0);
        
        //test with quality scores and robust estimator method
        double[] qualityScores = new double[
                PointCorrespondencePinholeCameraRobustEstimator.
                MIN_NUMBER_OF_POINT_CORRESPONDENCES];
        double[] shortScores = new double[1];
        
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
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
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(estimator.getPlanarThreshold(), 
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.getSkewness(), 0.0, 0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(), 0.0, 0.0);


        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
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
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(estimator.getPlanarThreshold(), 
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.getSkewness(), 0.0, 0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(), 0.0, 0.0);

        
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
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
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(estimator.getPlanarThreshold(), 
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.getSkewness(), 0.0, 0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(), 0.0, 0.0);

        
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
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
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(estimator.getPlanarThreshold(), 
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.getSkewness(), 0.0, 0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(), 0.0, 0.0);

        
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
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
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(estimator.getPlanarThreshold(), 
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.getSkewness(), 0.0, 0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(), 0.0, 0.0);
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(shortScores, RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with points, quality scores and robust estimator method
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
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
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(estimator.getPlanarThreshold(), 
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.getSkewness(), 0.0, 0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(), 0.0, 0.0);


        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
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
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(estimator.getPlanarThreshold(), 
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.getSkewness(), 0.0, 0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(), 0.0, 0.0);

        
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
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
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(estimator.getPlanarThreshold(), 
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.getSkewness(), 0.0, 0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(), 0.0, 0.0);

        
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
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
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(estimator.getPlanarThreshold(), 
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.getSkewness(), 0.0, 0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(), 0.0, 0.0);


        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
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
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(estimator.getPlanarThreshold(), 
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.getSkewness(), 0.0, 0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(), 0.0, 0.0);

        
        //Force IllegalArgumentException
        estimator = null;
        try{
            //empty points
            estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(empty3D, empty2D, qualityScores, 
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //different sizes
            estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(points3D, empty2D, qualityScores, 
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //short scores
            estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(points3D, points2D, shortScores, 
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with listener, quality scores and method
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, qualityScores, RobustEstimatorMethod.RANSAC);
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
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(estimator.getPlanarThreshold(), 
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.getSkewness(), 0.0, 0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(), 0.0, 0.0);


        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, qualityScores, RobustEstimatorMethod.LMedS);
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
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(estimator.getPlanarThreshold(), 
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.getSkewness(), 0.0, 0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(), 0.0, 0.0);

        
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, qualityScores, RobustEstimatorMethod.MSAC);
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
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(estimator.getPlanarThreshold(), 
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.getSkewness(), 0.0, 0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(), 0.0, 0.0);

        
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, qualityScores, RobustEstimatorMethod.PROSAC);
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
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(estimator.getPlanarThreshold(), 
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.getSkewness(), 0.0, 0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(), 0.0, 0.0);

        
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, qualityScores, RobustEstimatorMethod.PROMedS);
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
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(estimator.getPlanarThreshold(), 
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.getSkewness(), 0.0, 0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(), 0.0, 0.0);

        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(listener, shortScores, RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with listener, points, quality scores and method
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, qualityScores, 
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
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(estimator.getPlanarThreshold(), 
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.getSkewness(), 0.0, 0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(), 0.0, 0.0);


        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, qualityScores, 
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
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(estimator.getPlanarThreshold(), 
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.getSkewness(), 0.0, 0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(), 0.0, 0.0);

        
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, qualityScores, 
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
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(estimator.getPlanarThreshold(), 
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.getSkewness(), 0.0, 0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(), 0.0, 0.0);

        
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, qualityScores, 
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
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(estimator.getPlanarThreshold(), 
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.getSkewness(), 0.0, 0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(), 0.0, 0.0);

        
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, qualityScores, 
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
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(estimator.getPlanarThreshold(), 
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.getSkewness(), 0.0, 0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(), 0.0, 0.0);
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            //empty points
            estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(listener, empty3D, empty2D, qualityScores, 
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //different sizes
            estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(listener, points3D, empty2D, qualityScores, 
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //short scores
            estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(listener, points3D, points2D, shortScores, 
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);

        
        //test with no arguments
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.
                create();
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
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(estimator.getPlanarThreshold(), 
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.getSkewness(), 0.0, 0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(), 0.0, 0.0);


        //test with points
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
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
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(estimator.getPlanarThreshold(), 
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.getSkewness(), 0.0, 0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(), 0.0, 0.0);
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            //empty points
            estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(empty3D, empty2D);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //different sizes
            estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(points3D, empty2D);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);     
        
        //test with listener
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener);
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
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(estimator.getPlanarThreshold(), 
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.getSkewness(), 0.0, 0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(), 0.0, 0.0);

        
        //test with listener and points
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D);
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
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(estimator.getPlanarThreshold(), 
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.getSkewness(), 0.0, 0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(), 0.0, 0.0);
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            //empty points
            estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(listener, empty3D, empty2D);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //different sizes
            estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(listener, points3D, empty2D);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);     
        
        //test with quality scores
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
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
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(estimator.getPlanarThreshold(), 
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.getSkewness(), 0.0, 0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(), 0.0, 0.0);
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(shortScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with points and quality scores
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
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
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(estimator.getPlanarThreshold(), 
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.getSkewness(), 0.0, 0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(), 0.0, 0.0);
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            //empty points
            estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(empty3D, empty2D, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //different sizes
            estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(points3D, empty2D, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //short scores
            estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(points3D, points2D, shortScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);     
        
        //test with listener and scores
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, qualityScores);
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
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(estimator.getPlanarThreshold(), 
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.getSkewness(), 0.0, 0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(), 0.0, 0.0);
        
        //force IllegalArgumentException
        estimator = null;
        try{
            estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(listener, shortScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with listener, points and quality scores
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, qualityScores);
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
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(estimator.getPlanarThreshold(), 
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.getSkewness(), 0.0, 0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(), 0.0, 0.0);
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            //empty points
            estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(empty3D, empty2D, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //different sizes
            estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(points3D, empty2D, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //short scores
            estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(points3D, points2D, shortScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);   
    }
}
