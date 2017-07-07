/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.estimators.Point2DRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date March 1, 2015
 */
package com.irurueta.geometry.estimators;

import com.irurueta.geometry.Line2D;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import java.util.ArrayList;
import java.util.List;
import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class Point2DRobustEstimatorTest {
    
    public Point2DRobustEstimatorTest() {}
    
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
        Point2DRobustEstimator estimator;
        
        //test with robust method
        estimator = Point2DRobustEstimator.create(RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof RANSACPoint2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        estimator = Point2DRobustEstimator.create(RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof LMedSPoint2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());  
        
        estimator = Point2DRobustEstimator.create(RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof MSACPoint2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());  

        estimator = Point2DRobustEstimator.create(RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof PROSACPoint2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());  
        
        estimator = Point2DRobustEstimator.create(RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof PROMedSPoint2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());  
        
        //test with lines and method
        List<Line2D> lines = new ArrayList<Line2D>();
        for(int i = 0; i < Point2DRobustEstimator.MINIMUM_SIZE; i++){
            lines.add(new Line2D());
        }
        List<Line2D> emptyLines = new ArrayList<Line2D>();
        
        estimator = Point2DRobustEstimator.create(lines, 
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof RANSACPoint2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getLines(), lines);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());

        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = Point2DRobustEstimator.create(emptyLines, 
                    RobustEstimatorMethod.RANSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        estimator = Point2DRobustEstimator.create(lines, 
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof LMedSPoint2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getLines(), lines);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());

        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = Point2DRobustEstimator.create(emptyLines, 
                    RobustEstimatorMethod.LMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);    
        
        estimator = Point2DRobustEstimator.create(lines, 
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof MSACPoint2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getLines(), lines);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());

        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = Point2DRobustEstimator.create(emptyLines, 
                    RobustEstimatorMethod.MSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);    
        
        estimator = Point2DRobustEstimator.create(lines, 
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof PROSACPoint2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getLines(), lines);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());

        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = Point2DRobustEstimator.create(emptyLines, 
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator); 
        
        estimator = Point2DRobustEstimator.create(lines, 
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof PROMedSPoint2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getLines(), lines);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());

        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = Point2DRobustEstimator.create(emptyLines, 
                    RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);   
        
        //test with listener
        Point2DRobustEstimatorListener listener = new Point2DRobustEstimatorListener() {

            @Override
            public void onEstimateStart(Point2DRobustEstimator estimator) {}

            @Override
            public void onEstimateEnd(Point2DRobustEstimator estimator) {}

            @Override
            public void onEstimateNextIteration(Point2DRobustEstimator estimator, 
                    int iteration) {}

            @Override
            public void onEstimateProgressChange(Point2DRobustEstimator estimator, 
                    float progress) {}
        };
        
        estimator = Point2DRobustEstimator.create(listener, 
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof RANSACPoint2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());        
        
        estimator = Point2DRobustEstimator.create(listener, 
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof LMedSPoint2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());  
        
        estimator = Point2DRobustEstimator.create(listener, 
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof MSACPoint2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());   
        
        estimator = Point2DRobustEstimator.create(listener, 
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof PROSACPoint2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());         
        
        estimator = Point2DRobustEstimator.create(listener, 
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof PROMedSPoint2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());         
        
        //test with listener, lines and method
        estimator = Point2DRobustEstimator.create(listener, lines,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof RANSACPoint2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getLines(), lines);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());        
        
        estimator = Point2DRobustEstimator.create(listener, lines,
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof LMedSPoint2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getLines(), lines);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());        

        estimator = Point2DRobustEstimator.create(listener, lines,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof MSACPoint2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getLines(), lines);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());        
        
        estimator = Point2DRobustEstimator.create(listener, lines,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof PROSACPoint2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getLines(), lines);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());        
        
        estimator = Point2DRobustEstimator.create(listener, lines,
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof PROMedSPoint2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getLines(), lines);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = Point2DRobustEstimator.create(listener, emptyLines,
                    RobustEstimatorMethod.RANSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with quality scores
        double[] qualityScores = new double[Point2DRobustEstimator.MINIMUM_SIZE];
        double[] emptyScores = new double[0];
        
        estimator = Point2DRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof RANSACPoint2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        estimator = Point2DRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof LMedSPoint2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        estimator = Point2DRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.MSAC);        
        assertTrue(estimator instanceof MSACPoint2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());

        estimator = Point2DRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.PROSAC);        
        assertTrue(estimator instanceof PROSACPoint2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getLines());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);

        estimator = Point2DRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.PROMedS);        
        assertTrue(estimator instanceof PROMedSPoint2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getLines());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = Point2DRobustEstimator.create(emptyScores,
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //Test with lines and quality scores
        estimator = Point2DRobustEstimator.create(lines, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof RANSACPoint2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getLines(), lines);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        estimator = Point2DRobustEstimator.create(lines, qualityScores,
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof LMedSPoint2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getLines(), lines);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());

        estimator = Point2DRobustEstimator.create(lines, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof MSACPoint2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getLines(), lines);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        estimator = Point2DRobustEstimator.create(lines, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof PROSACPoint2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getLines(), lines);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);

        estimator = Point2DRobustEstimator.create(lines, qualityScores,
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof PROMedSPoint2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getLines(), lines);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = Point2DRobustEstimator.create(emptyLines, qualityScores,
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = Point2DRobustEstimator.create(lines, emptyScores,
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with listener and quality scores
        estimator = Point2DRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof RANSACPoint2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        estimator = Point2DRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof LMedSPoint2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());

        estimator = Point2DRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof MSACPoint2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        estimator = Point2DRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof PROSACPoint2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getLines());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);

        estimator = Point2DRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof PROMedSPoint2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getLines());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = Point2DRobustEstimator.create(listener, emptyScores,
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with listener, lines and qualityScores
        estimator = Point2DRobustEstimator.create(listener, lines, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof RANSACPoint2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getLines(), lines);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        estimator = Point2DRobustEstimator.create(listener, lines, qualityScores,
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof LMedSPoint2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getLines(), lines);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());

        estimator = Point2DRobustEstimator.create(listener, lines, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof MSACPoint2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getLines(), lines);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        estimator = Point2DRobustEstimator.create(listener, lines, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof PROSACPoint2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getLines(), lines);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        
        estimator = Point2DRobustEstimator.create(listener, lines, qualityScores,
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof PROMedSPoint2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getLines(), lines);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        
        estimator = null;
        try{
            estimator = Point2DRobustEstimator.create(listener, emptyLines,
                    qualityScores, RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = Point2DRobustEstimator.create(listener, lines, 
                    emptyScores, RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test without arguments
        estimator = Point2DRobustEstimator.create();
        assertTrue(estimator instanceof PROMedSPoint2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //test with lines
        estimator = Point2DRobustEstimator.create(lines);
        assertTrue(estimator instanceof PROMedSPoint2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getLines(), lines);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = Point2DRobustEstimator.create(emptyLines);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with listener
        estimator = Point2DRobustEstimator.create(listener);
        assertTrue(estimator instanceof PROMedSPoint2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //test with listener and lines
        estimator = Point2DRobustEstimator.create(listener, lines);
        assertTrue(estimator instanceof PROMedSPoint2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getLines(), lines);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = Point2DRobustEstimator.create(listener, emptyLines);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with quality scores
        estimator = Point2DRobustEstimator.create(qualityScores);
        assertTrue(estimator instanceof PROMedSPoint2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getLines());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = Point2DRobustEstimator.create(emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with lines and quality scores
        estimator = Point2DRobustEstimator.create(lines, qualityScores);
        assertTrue(estimator instanceof PROMedSPoint2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getLines(), lines);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = Point2DRobustEstimator.create(emptyLines, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = Point2DRobustEstimator.create(lines, emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}        
        assertNull(estimator);
        
        //test with listener and quality scores
        estimator = Point2DRobustEstimator.create(listener, qualityScores);
        assertTrue(estimator instanceof PROMedSPoint2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getLines());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = Point2DRobustEstimator.create(listener, emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with listener, lines and quality scores
        estimator = Point2DRobustEstimator.create(listener, lines, 
                qualityScores);
        assertTrue(estimator instanceof PROMedSPoint2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getLines(), lines);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);  
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = Point2DRobustEstimator.create(listener, emptyLines, 
                    qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = Point2DRobustEstimator.create(listener, lines, 
                    emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}        
        assertNull(estimator);        
    }    
}
