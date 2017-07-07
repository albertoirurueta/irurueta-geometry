/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.estimators.CircleRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date February 28, 2015
 */
package com.irurueta.geometry.estimators;

import com.irurueta.geometry.Point2D;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import java.util.ArrayList;
import java.util.List;
import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class CircleRobustEstimatorTest {
    
    public CircleRobustEstimatorTest() {}
    
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
        CircleRobustEstimator estimator;
        
        //test with robust method
        estimator = CircleRobustEstimator.create(RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof RANSACCircleRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                CircleRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                CircleRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                CircleRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        estimator = CircleRobustEstimator.create(RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof LMedSCircleRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                CircleRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                CircleRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                CircleRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());  
        
        estimator = CircleRobustEstimator.create(RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof MSACCircleRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                CircleRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                CircleRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                CircleRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());  

        estimator = CircleRobustEstimator.create(RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof PROSACCircleRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                CircleRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                CircleRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                CircleRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());  
        
        estimator = CircleRobustEstimator.create(RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof PROMedSCircleRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                CircleRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                CircleRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                CircleRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());  
        
        //test with points and method
        List<Point2D> points = new ArrayList<Point2D>();
        for(int i = 0; i < CircleRobustEstimator.MINIMUM_SIZE; i++){
            points.add(Point2D.create());
        }
        List<Point2D> emptyPoints = new ArrayList<Point2D>();
        
        estimator = CircleRobustEstimator.create(points, 
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof RANSACCircleRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                CircleRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                CircleRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                CircleRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPoints(), points);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());

        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = CircleRobustEstimator.create(emptyPoints, 
                    RobustEstimatorMethod.RANSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        estimator = CircleRobustEstimator.create(points, 
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof LMedSCircleRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                CircleRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                CircleRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                CircleRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPoints(), points);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());

        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = CircleRobustEstimator.create(emptyPoints, 
                    RobustEstimatorMethod.LMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);    
        
        estimator = CircleRobustEstimator.create(points, 
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof MSACCircleRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                CircleRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                CircleRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                CircleRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPoints(), points);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());

        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = CircleRobustEstimator.create(emptyPoints, 
                    RobustEstimatorMethod.MSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);    
        
        estimator = CircleRobustEstimator.create(points, 
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof PROSACCircleRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                CircleRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                CircleRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                CircleRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPoints(), points);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());

        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = CircleRobustEstimator.create(emptyPoints, 
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator); 
        
        estimator = CircleRobustEstimator.create(points, 
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof PROMedSCircleRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                CircleRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                CircleRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                CircleRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPoints(), points);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());

        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = CircleRobustEstimator.create(emptyPoints, 
                    RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);   
        
        //test with listener
        CircleRobustEstimatorListener listener = new CircleRobustEstimatorListener() {

            @Override
            public void onEstimateStart(CircleRobustEstimator estimator) {}

            @Override
            public void onEstimateEnd(CircleRobustEstimator estimator) {}

            @Override
            public void onEstimateNextIteration(CircleRobustEstimator estimator, 
                    int iteration) {}

            @Override
            public void onEstimateProgressChange(CircleRobustEstimator estimator, 
                    float progress) {}
        };
        
        estimator = CircleRobustEstimator.create(listener, 
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof RANSACCircleRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                CircleRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                CircleRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                CircleRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());        
        
        estimator = CircleRobustEstimator.create(listener, 
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof LMedSCircleRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                CircleRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                CircleRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                CircleRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());  
        
        estimator = CircleRobustEstimator.create(listener, 
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof MSACCircleRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                CircleRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                CircleRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                CircleRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());   
        
        estimator = CircleRobustEstimator.create(listener, 
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof PROSACCircleRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                CircleRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                CircleRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                CircleRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());         
        
        estimator = CircleRobustEstimator.create(listener, 
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof PROMedSCircleRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                CircleRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                CircleRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                CircleRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());         
        
        //test with listener, points and method
        estimator = CircleRobustEstimator.create(listener, points,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof RANSACCircleRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                CircleRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                CircleRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                CircleRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPoints(), points);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());        
        
        estimator = CircleRobustEstimator.create(listener, points,
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof LMedSCircleRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                CircleRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                CircleRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                CircleRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPoints(), points);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());        

        estimator = CircleRobustEstimator.create(listener, points,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof MSACCircleRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                CircleRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                CircleRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                CircleRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPoints(), points);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());        
        
        estimator = CircleRobustEstimator.create(listener, points,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof PROSACCircleRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                CircleRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                CircleRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                CircleRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPoints(), points);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());        
        
        estimator = CircleRobustEstimator.create(listener, points,
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof PROMedSCircleRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                CircleRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                CircleRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                CircleRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPoints(), points);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = CircleRobustEstimator.create(listener, emptyPoints,
                    RobustEstimatorMethod.RANSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with quality scores
        double[] qualityScores = new double[CircleRobustEstimator.MINIMUM_SIZE];
        double[] emptyScores = new double[0];
        
        estimator = CircleRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof RANSACCircleRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                CircleRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                CircleRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                CircleRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        estimator = CircleRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof LMedSCircleRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                CircleRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                CircleRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                CircleRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        estimator = CircleRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.MSAC);        
        assertTrue(estimator instanceof MSACCircleRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                CircleRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                CircleRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                CircleRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());

        estimator = CircleRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.PROSAC);        
        assertTrue(estimator instanceof PROSACCircleRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                CircleRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                CircleRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                CircleRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);

        estimator = CircleRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.PROMedS);        
        assertTrue(estimator instanceof PROMedSCircleRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                CircleRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                CircleRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                CircleRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = CircleRobustEstimator.create(emptyScores,
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //Test with points and quality scores
        estimator = CircleRobustEstimator.create(points, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof RANSACCircleRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                CircleRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                CircleRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                CircleRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPoints(), points);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        estimator = CircleRobustEstimator.create(points, qualityScores,
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof LMedSCircleRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                CircleRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                CircleRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                CircleRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPoints(), points);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());

        estimator = CircleRobustEstimator.create(points, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof MSACCircleRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                CircleRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                CircleRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                CircleRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPoints(), points);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        estimator = CircleRobustEstimator.create(points, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof PROSACCircleRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                CircleRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                CircleRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                CircleRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPoints(), points);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);

        estimator = CircleRobustEstimator.create(points, qualityScores,
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof PROMedSCircleRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                CircleRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                CircleRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                CircleRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPoints(), points);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = CircleRobustEstimator.create(emptyPoints, qualityScores,
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = CircleRobustEstimator.create(points, emptyScores,
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with listener and quality scores
        estimator = CircleRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof RANSACCircleRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                CircleRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                CircleRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                CircleRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        estimator = CircleRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof LMedSCircleRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                CircleRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                CircleRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                CircleRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());

        estimator = CircleRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof MSACCircleRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                CircleRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                CircleRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                CircleRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        estimator = CircleRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof PROSACCircleRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                CircleRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                CircleRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                CircleRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);

        estimator = CircleRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof PROMedSCircleRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                CircleRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                CircleRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                CircleRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = CircleRobustEstimator.create(listener, emptyScores,
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with listener, points and qualityScores
        estimator = CircleRobustEstimator.create(listener, points, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof RANSACCircleRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                CircleRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                CircleRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                CircleRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPoints(), points);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        estimator = CircleRobustEstimator.create(listener, points, qualityScores,
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof LMedSCircleRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                CircleRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                CircleRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                CircleRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPoints(), points);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());

        estimator = CircleRobustEstimator.create(listener, points, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof MSACCircleRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                CircleRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                CircleRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                CircleRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPoints(), points);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        estimator = CircleRobustEstimator.create(listener, points, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof PROSACCircleRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                CircleRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                CircleRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                CircleRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPoints(), points);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        
        estimator = CircleRobustEstimator.create(listener, points, qualityScores,
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof PROMedSCircleRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                CircleRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                CircleRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                CircleRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPoints(), points);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        
        estimator = null;
        try{
            estimator = CircleRobustEstimator.create(listener, emptyPoints,
                    qualityScores, RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = CircleRobustEstimator.create(listener, points, 
                    emptyScores, RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test without arguments
        estimator = CircleRobustEstimator.create();
        assertTrue(estimator instanceof PROMedSCircleRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                CircleRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                CircleRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                CircleRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //test with points
        estimator = CircleRobustEstimator.create(points);
        assertTrue(estimator instanceof PROMedSCircleRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                CircleRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                CircleRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                CircleRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPoints(), points);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = CircleRobustEstimator.create(emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with listener
        estimator = CircleRobustEstimator.create(listener);
        assertTrue(estimator instanceof PROMedSCircleRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                CircleRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                CircleRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                CircleRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //test with listener and points
        estimator = CircleRobustEstimator.create(listener, points);
        assertTrue(estimator instanceof PROMedSCircleRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                CircleRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                CircleRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                CircleRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPoints(), points);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = CircleRobustEstimator.create(listener, emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with quality scores
        estimator = CircleRobustEstimator.create(qualityScores);
        assertTrue(estimator instanceof PROMedSCircleRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                CircleRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                CircleRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                CircleRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = CircleRobustEstimator.create(emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with points and quality scores
        estimator = CircleRobustEstimator.create(points, qualityScores);
        assertTrue(estimator instanceof PROMedSCircleRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                CircleRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                CircleRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                CircleRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPoints(), points);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = CircleRobustEstimator.create(emptyPoints, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = CircleRobustEstimator.create(points, emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}        
        assertNull(estimator);
        
        //test with listener and quality scores
        estimator = CircleRobustEstimator.create(listener, qualityScores);
        assertTrue(estimator instanceof PROMedSCircleRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                CircleRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                CircleRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                CircleRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = CircleRobustEstimator.create(listener, emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with listener, points and quality scores
        estimator = CircleRobustEstimator.create(listener, points, 
                qualityScores);
        assertTrue(estimator instanceof PROMedSCircleRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                CircleRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                CircleRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                CircleRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPoints(), points);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);  
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = CircleRobustEstimator.create(listener, emptyPoints, 
                    qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = CircleRobustEstimator.create(listener, points, 
                    emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}        
        assertNull(estimator);        
    }
}
