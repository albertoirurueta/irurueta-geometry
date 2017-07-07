/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.estimators.PinholeCameraRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date March 13, 2015
 */
package com.irurueta.geometry.estimators;

import com.irurueta.geometry.Line2D;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.Plane;
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

public class PinholeCameraRobustEstimatorTest {
    
    public PinholeCameraRobustEstimatorTest() {}
    
    @BeforeClass
    public static void setUpClass() {}
    
    @AfterClass
    public static void tearDownClass() {}
    
    @Before
    public void setUp() {}
    
    @After
    public void tearDown() {}

    @Test
    public void testCreateFromPoints(){
        PinholeCameraRobustEstimator estimator;
        
        //create with points
        List<Point2D> points2D = new ArrayList<Point2D>();
        List<Point3D> points3D = new ArrayList<Point3D>();
        for(int i = 0; i < PointCorrespondencePinholeCameraRobustEstimator.MIN_NUMBER_OF_POINT_CORRESPONDENCES; i++){
            points2D.add(Point2D.create());
            points3D.add(Point3D.create());
        }
        
        estimator = PinholeCameraRobustEstimator.createFromPoints(points3D, 
                points2D, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        estimator = PinholeCameraRobustEstimator.createFromPoints(points3D, 
                points2D, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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

        estimator = PinholeCameraRobustEstimator.createFromPoints(points3D, 
                points2D, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        estimator = PinholeCameraRobustEstimator.createFromPoints(points3D, 
                points2D, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        estimator = PinholeCameraRobustEstimator.createFromPoints(points3D, 
                points2D, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        //create with listener and points
        PinholeCameraRobustEstimatorListener listener = new PinholeCameraRobustEstimatorListener() {

            @Override
            public void onEstimateStart(PinholeCameraRobustEstimator estimator) {}

            @Override
            public void onEstimateEnd(PinholeCameraRobustEstimator estimator) {}

            @Override
            public void onEstimateNextIteration(PinholeCameraRobustEstimator estimator, int iteration) {}

            @Override
            public void onEstimateProgressChange(PinholeCameraRobustEstimator estimator, float progress) {}
        };
        
        estimator = PinholeCameraRobustEstimator.createFromPoints(listener, 
                points3D, points2D, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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

        estimator = PinholeCameraRobustEstimator.createFromPoints(listener, 
                points3D, points2D, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        estimator = PinholeCameraRobustEstimator.createFromPoints(listener, 
                points3D, points2D, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        estimator = PinholeCameraRobustEstimator.createFromPoints(listener, 
                points3D, points2D, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        estimator = PinholeCameraRobustEstimator.createFromPoints(listener, 
                points3D, points2D, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        
        //create with points and quality scores
        double[] pointQualityScores = new double[
                PointCorrespondencePinholeCameraRobustEstimator.MIN_NUMBER_OF_POINT_CORRESPONDENCES];
        estimator = PinholeCameraRobustEstimator.createFromPoints(points3D, 
                points2D, pointQualityScores, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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

        estimator = PinholeCameraRobustEstimator.createFromPoints(points3D, 
                points2D, pointQualityScores, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        estimator = PinholeCameraRobustEstimator.createFromPoints(points3D, 
                points2D, pointQualityScores, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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

        estimator = PinholeCameraRobustEstimator.createFromPoints(points3D, 
                points2D, pointQualityScores, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        estimator = PinholeCameraRobustEstimator.createFromPoints(points3D, 
                points2D, pointQualityScores, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        //create with points, listener and quality scores
        estimator = PinholeCameraRobustEstimator.createFromPoints(listener, 
                points3D, points2D, pointQualityScores, 
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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

        estimator = PinholeCameraRobustEstimator.createFromPoints(listener, 
                points3D, points2D, pointQualityScores, 
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        estimator = PinholeCameraRobustEstimator.createFromPoints(listener, 
                points3D, points2D, pointQualityScores, 
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        estimator = PinholeCameraRobustEstimator.createFromPoints(listener, 
                points3D, points2D, pointQualityScores, 
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        estimator = PinholeCameraRobustEstimator.createFromPoints(listener, 
                points3D, points2D, pointQualityScores, 
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        //create with points
        estimator = PinholeCameraRobustEstimator.createFromPoints(points3D, 
                points2D);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        //create with listener and points
        estimator = PinholeCameraRobustEstimator.createFromPoints(listener, 
                points3D, points2D);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        //create with points and quality scores
        estimator = PinholeCameraRobustEstimator.createFromPoints(points3D, 
                points2D, pointQualityScores);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        //create with listener, points and qualitys cores
        estimator = PinholeCameraRobustEstimator.createFromPoints(listener, 
                points3D, points2D, pointQualityScores);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
    }

    @Test
    public void testCreateFromPointsWithIntrinsic(){
        PinholeCameraIntrinsicParameters intrinsic = 
                new PinholeCameraIntrinsicParameters();
        
        PinholeCameraRobustEstimator estimator;
        
        //create with points
        List<Point2D> points2D = new ArrayList<Point2D>();
        List<Point3D> points3D = new ArrayList<Point3D>();
        for(int i = 0; i < PointCorrespondencePinholeCameraRobustEstimator.MIN_NUMBER_OF_POINT_CORRESPONDENCES; i++){
            points2D.add(Point2D.create());
            points3D.add(Point3D.create());
        }
        
        estimator = PinholeCameraRobustEstimator.createFromPoints(intrinsic,
                points3D, points2D, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        estimator = PinholeCameraRobustEstimator.createFromPoints(intrinsic, 
                points3D, points2D, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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

        estimator = PinholeCameraRobustEstimator.createFromPoints(intrinsic, 
                points3D, points2D, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        estimator = PinholeCameraRobustEstimator.createFromPoints(intrinsic, 
                points3D, points2D, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        estimator = PinholeCameraRobustEstimator.createFromPoints(intrinsic, 
                points3D, points2D, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        //create with listener and points
        PinholeCameraRobustEstimatorListener listener = new PinholeCameraRobustEstimatorListener() {

            @Override
            public void onEstimateStart(PinholeCameraRobustEstimator estimator) {}

            @Override
            public void onEstimateEnd(PinholeCameraRobustEstimator estimator) {}

            @Override
            public void onEstimateNextIteration(PinholeCameraRobustEstimator estimator, int iteration) {}

            @Override
            public void onEstimateProgressChange(PinholeCameraRobustEstimator estimator, float progress) {}
        };
        
        estimator = PinholeCameraRobustEstimator.createFromPoints(listener, 
                intrinsic, points3D, points2D, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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

        estimator = PinholeCameraRobustEstimator.createFromPoints(listener, 
                intrinsic, points3D, points2D, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        estimator = PinholeCameraRobustEstimator.createFromPoints(listener, 
                intrinsic, points3D, points2D, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        estimator = PinholeCameraRobustEstimator.createFromPoints(listener, 
                intrinsic, points3D, points2D, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        estimator = PinholeCameraRobustEstimator.createFromPoints(listener, 
                intrinsic, points3D, points2D, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        
        //create with points and quality scores
        double[] pointQualityScores = new double[
                PointCorrespondencePinholeCameraRobustEstimator.MIN_NUMBER_OF_POINT_CORRESPONDENCES];
        estimator = PinholeCameraRobustEstimator.createFromPoints(intrinsic,
                points3D, points2D, pointQualityScores, 
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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

        estimator = PinholeCameraRobustEstimator.createFromPoints(intrinsic,
                points3D, points2D, pointQualityScores, 
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        estimator = PinholeCameraRobustEstimator.createFromPoints(intrinsic,
                points3D, points2D, pointQualityScores, 
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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

        estimator = PinholeCameraRobustEstimator.createFromPoints(intrinsic,
                points3D, points2D, pointQualityScores, 
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        estimator = PinholeCameraRobustEstimator.createFromPoints(intrinsic,
                points3D, points2D, pointQualityScores, 
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        //create with points, listener and quality scores
        estimator = PinholeCameraRobustEstimator.createFromPoints(listener, 
                intrinsic, points3D, points2D, pointQualityScores, 
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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

        estimator = PinholeCameraRobustEstimator.createFromPoints(listener, 
                intrinsic, points3D, points2D, pointQualityScores, 
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        estimator = PinholeCameraRobustEstimator.createFromPoints(listener, 
                intrinsic, points3D, points2D, pointQualityScores, 
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        estimator = PinholeCameraRobustEstimator.createFromPoints(listener, 
                intrinsic, points3D, points2D, pointQualityScores, 
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        estimator = PinholeCameraRobustEstimator.createFromPoints(listener, 
                intrinsic, points3D, points2D, pointQualityScores, 
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        //create with points
        estimator = PinholeCameraRobustEstimator.createFromPoints(intrinsic, 
                points3D, points2D);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        //create with listener and points
        estimator = PinholeCameraRobustEstimator.createFromPoints(listener, 
                intrinsic, points3D, points2D);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        //create with points and quality scores
        estimator = PinholeCameraRobustEstimator.createFromPoints(intrinsic, 
                points3D, points2D, pointQualityScores);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        //create with listener, points and qualitys cores
        estimator = PinholeCameraRobustEstimator.createFromPoints(listener, 
                intrinsic, points3D, points2D, pointQualityScores);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
    }

    @Test
    public void testCreateFromPointsWithSkewnessAndPrincipalPoint(){
        double skewness = 0.0;
        double horizontalPrincipalPoint = 0.0;
        double verticalPrincipalPoint = 0.0;
        
        PinholeCameraRobustEstimator estimator;
        
        //create with points
        List<Point2D> points2D = new ArrayList<Point2D>();
        List<Point3D> points3D = new ArrayList<Point3D>();
        for(int i = 0; i < PointCorrespondencePinholeCameraRobustEstimator.MIN_NUMBER_OF_POINT_CORRESPONDENCES; i++){
            points2D.add(Point2D.create());
            points3D.add(Point3D.create());
        }
        
        estimator = PinholeCameraRobustEstimator.createFromPoints(skewness, 
                horizontalPrincipalPoint, verticalPrincipalPoint,
                points3D, points2D, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        estimator = PinholeCameraRobustEstimator.createFromPoints(skewness, 
                horizontalPrincipalPoint, verticalPrincipalPoint, 
                points3D, points2D, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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

        estimator = PinholeCameraRobustEstimator.createFromPoints(skewness, 
                horizontalPrincipalPoint, verticalPrincipalPoint, 
                points3D, points2D, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        estimator = PinholeCameraRobustEstimator.createFromPoints(skewness,
                horizontalPrincipalPoint, verticalPrincipalPoint, 
                points3D, points2D, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        estimator = PinholeCameraRobustEstimator.createFromPoints(skewness,
                horizontalPrincipalPoint, verticalPrincipalPoint, 
                points3D, points2D, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        //create with listener and points
        PinholeCameraRobustEstimatorListener listener = new PinholeCameraRobustEstimatorListener() {

            @Override
            public void onEstimateStart(PinholeCameraRobustEstimator estimator) {}

            @Override
            public void onEstimateEnd(PinholeCameraRobustEstimator estimator) {}

            @Override
            public void onEstimateNextIteration(PinholeCameraRobustEstimator estimator, int iteration) {}

            @Override
            public void onEstimateProgressChange(PinholeCameraRobustEstimator estimator, float progress) {}
        };
        
        estimator = PinholeCameraRobustEstimator.createFromPoints(listener, 
                skewness, horizontalPrincipalPoint, verticalPrincipalPoint, 
                points3D, points2D, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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

        estimator = PinholeCameraRobustEstimator.createFromPoints(listener, 
                skewness, horizontalPrincipalPoint, verticalPrincipalPoint, 
                points3D, points2D, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        estimator = PinholeCameraRobustEstimator.createFromPoints(listener, 
                skewness, horizontalPrincipalPoint, verticalPrincipalPoint, 
                points3D, points2D, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        estimator = PinholeCameraRobustEstimator.createFromPoints(listener, 
                skewness, horizontalPrincipalPoint, verticalPrincipalPoint, 
                points3D, points2D, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        estimator = PinholeCameraRobustEstimator.createFromPoints(listener, 
                skewness, horizontalPrincipalPoint, verticalPrincipalPoint, 
                points3D, points2D, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        
        //create with points and quality scores
        double[] pointQualityScores = new double[
                PointCorrespondencePinholeCameraRobustEstimator.MIN_NUMBER_OF_POINT_CORRESPONDENCES];
        estimator = PinholeCameraRobustEstimator.createFromPoints(skewness,
                horizontalPrincipalPoint, verticalPrincipalPoint, points3D, 
                points2D, pointQualityScores, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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

        estimator = PinholeCameraRobustEstimator.createFromPoints(skewness, 
                horizontalPrincipalPoint, verticalPrincipalPoint, points3D, 
                points2D, pointQualityScores, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        estimator = PinholeCameraRobustEstimator.createFromPoints(skewness,
                horizontalPrincipalPoint, verticalPrincipalPoint, points3D, 
                points2D, pointQualityScores, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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

        estimator = PinholeCameraRobustEstimator.createFromPoints(skewness,
                horizontalPrincipalPoint, verticalPrincipalPoint, points3D, 
                points2D, pointQualityScores, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        estimator = PinholeCameraRobustEstimator.createFromPoints(skewness,
                horizontalPrincipalPoint, verticalPrincipalPoint, points3D, 
                points2D, pointQualityScores, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        //create with points, listener and quality scores
        estimator = PinholeCameraRobustEstimator.createFromPoints(listener, 
                skewness, horizontalPrincipalPoint, verticalPrincipalPoint, 
                points3D, points2D, pointQualityScores, 
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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

        estimator = PinholeCameraRobustEstimator.createFromPoints(listener, 
                skewness, horizontalPrincipalPoint, verticalPrincipalPoint, 
                points3D, points2D, pointQualityScores, 
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        estimator = PinholeCameraRobustEstimator.createFromPoints(listener, 
                skewness, horizontalPrincipalPoint, verticalPrincipalPoint, 
                points3D, points2D, pointQualityScores, 
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        estimator = PinholeCameraRobustEstimator.createFromPoints(listener, 
                skewness, horizontalPrincipalPoint, verticalPrincipalPoint, 
                points3D, points2D, pointQualityScores, 
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        estimator = PinholeCameraRobustEstimator.createFromPoints(listener, 
                skewness, horizontalPrincipalPoint, verticalPrincipalPoint, 
                points3D, points2D, pointQualityScores, 
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        //create with points
        estimator = PinholeCameraRobustEstimator.createFromPoints(skewness,
                horizontalPrincipalPoint, verticalPrincipalPoint, points3D, 
                points2D);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        //create with listener and points
        estimator = PinholeCameraRobustEstimator.createFromPoints(listener, 
                skewness, horizontalPrincipalPoint, verticalPrincipalPoint, 
                points3D, points2D);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        //create with points and quality scores
        estimator = PinholeCameraRobustEstimator.createFromPoints(skewness,
                horizontalPrincipalPoint, verticalPrincipalPoint, points3D, 
                points2D, pointQualityScores);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
        
        //create with listener, points and qualitys cores
        estimator = PinholeCameraRobustEstimator.createFromPoints(listener, 
                skewness, horizontalPrincipalPoint, verticalPrincipalPoint, 
                points3D, points2D, pointQualityScores);
        assertTrue(estimator instanceof 
                PointCorrespondencePinholeCameraRobustEstimator);
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
    }
    
    @Test
    public void testCreateFromPlanesAndLines(){
        PinholeCameraRobustEstimator estimator;
        
        //create with planes and lines
        List<Line2D> lines = new ArrayList<Line2D>();
        List<Plane> planes = new ArrayList<Plane>();
        for(int i = 0; i < PointCorrespondencePinholeCameraRobustEstimator.MIN_NUMBER_OF_POINT_CORRESPONDENCES; i++){
            lines.add(new Line2D());
            planes.add(new Plane());
        }
        
        estimator = PinholeCameraRobustEstimator.createFromPlanesAndLines(
                planes, lines, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                LinePlaneCorrespondencePinholeCameraRobustEstimator);
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
        
        estimator = PinholeCameraRobustEstimator.createFromPlanesAndLines(
                planes, lines, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LinePlaneCorrespondencePinholeCameraRobustEstimator);
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

        estimator = PinholeCameraRobustEstimator.createFromPlanesAndLines(
                planes, lines, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                LinePlaneCorrespondencePinholeCameraRobustEstimator);
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
        
        estimator = PinholeCameraRobustEstimator.createFromPlanesAndLines(
                planes, lines, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                LinePlaneCorrespondencePinholeCameraRobustEstimator);
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
        
        estimator = PinholeCameraRobustEstimator.createFromPlanesAndLines(
                planes, lines, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                LinePlaneCorrespondencePinholeCameraRobustEstimator);
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
        
        //create with listener and points
        PinholeCameraRobustEstimatorListener listener = new PinholeCameraRobustEstimatorListener() {

            @Override
            public void onEstimateStart(PinholeCameraRobustEstimator estimator) {}

            @Override
            public void onEstimateEnd(PinholeCameraRobustEstimator estimator) {}

            @Override
            public void onEstimateNextIteration(PinholeCameraRobustEstimator estimator, int iteration) {}

            @Override
            public void onEstimateProgressChange(PinholeCameraRobustEstimator estimator, float progress) {}
        };
        
        estimator = PinholeCameraRobustEstimator.createFromPlanesAndLines(
                listener, planes, lines, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                LinePlaneCorrespondencePinholeCameraRobustEstimator);
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

        estimator = PinholeCameraRobustEstimator.createFromPlanesAndLines(
                listener, planes, lines, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LinePlaneCorrespondencePinholeCameraRobustEstimator);
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
        
        estimator = PinholeCameraRobustEstimator.createFromPlanesAndLines(
                listener, planes, lines, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                LinePlaneCorrespondencePinholeCameraRobustEstimator);
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
        
        estimator = PinholeCameraRobustEstimator.createFromPlanesAndLines(
                listener, planes, lines, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                LinePlaneCorrespondencePinholeCameraRobustEstimator);
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
        
        estimator = PinholeCameraRobustEstimator.createFromPlanesAndLines(
                listener, planes, lines, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                LinePlaneCorrespondencePinholeCameraRobustEstimator);
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
        
        
        //create with points and quality scores
        double[] pointQualityScores = new double[
                PointCorrespondencePinholeCameraRobustEstimator.MIN_NUMBER_OF_POINT_CORRESPONDENCES];
        estimator = PinholeCameraRobustEstimator.createFromPlanesAndLines(
                planes, lines, pointQualityScores, 
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                LinePlaneCorrespondencePinholeCameraRobustEstimator);
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

        estimator = PinholeCameraRobustEstimator.createFromPlanesAndLines(
                planes, lines, pointQualityScores, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LinePlaneCorrespondencePinholeCameraRobustEstimator);
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
        
        estimator = PinholeCameraRobustEstimator.createFromPlanesAndLines(
                planes, lines, pointQualityScores, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                LinePlaneCorrespondencePinholeCameraRobustEstimator);
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

        estimator = PinholeCameraRobustEstimator.createFromPlanesAndLines(
                planes, lines, pointQualityScores, 
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                LinePlaneCorrespondencePinholeCameraRobustEstimator);
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
        
        estimator = PinholeCameraRobustEstimator.createFromPlanesAndLines(
                planes, lines, pointQualityScores, 
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                LinePlaneCorrespondencePinholeCameraRobustEstimator);
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
        
        //create with points, listener and quality scores
        estimator = PinholeCameraRobustEstimator.createFromPlanesAndLines(
                listener, planes, lines, pointQualityScores, 
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                LinePlaneCorrespondencePinholeCameraRobustEstimator);
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

        estimator = PinholeCameraRobustEstimator.createFromPlanesAndLines(
                listener, planes, lines, pointQualityScores, 
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LinePlaneCorrespondencePinholeCameraRobustEstimator);
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
        
        estimator = PinholeCameraRobustEstimator.createFromPlanesAndLines(
                listener, planes, lines, pointQualityScores, 
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                LinePlaneCorrespondencePinholeCameraRobustEstimator);
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
        
        estimator = PinholeCameraRobustEstimator.createFromPlanesAndLines(
                listener, planes, lines, pointQualityScores, 
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                LinePlaneCorrespondencePinholeCameraRobustEstimator);
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
        
        estimator = PinholeCameraRobustEstimator.createFromPlanesAndLines(
                listener, planes, lines, pointQualityScores, 
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                LinePlaneCorrespondencePinholeCameraRobustEstimator);
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
        
        //create with points
        estimator = PinholeCameraRobustEstimator.createFromPlanesAndLines(
                planes, lines);
        assertTrue(estimator instanceof 
                LinePlaneCorrespondencePinholeCameraRobustEstimator);
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
        
        //create with listener and points
        estimator = PinholeCameraRobustEstimator.createFromPlanesAndLines(
                listener, planes, lines);
        assertTrue(estimator instanceof 
                LinePlaneCorrespondencePinholeCameraRobustEstimator);
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
        
        //create with points and quality scores
        estimator = PinholeCameraRobustEstimator.createFromPlanesAndLines(
                planes, lines, pointQualityScores);
        assertTrue(estimator instanceof 
                LinePlaneCorrespondencePinholeCameraRobustEstimator);
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
        
        //create with listener, points and qualitys cores
        estimator = PinholeCameraRobustEstimator.createFromPlanesAndLines(
                listener, planes, lines, pointQualityScores);
        assertTrue(estimator instanceof 
                LinePlaneCorrespondencePinholeCameraRobustEstimator);
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
    }
}
