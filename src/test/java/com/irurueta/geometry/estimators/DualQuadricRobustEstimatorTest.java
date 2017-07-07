/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.estimators.DualQuadricRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date February 24, 2015
 */
package com.irurueta.geometry.estimators;

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

public class DualQuadricRobustEstimatorTest {
    
    public DualQuadricRobustEstimatorTest() {}
    
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
        DualQuadricRobustEstimator estimator;
        
        //test with robust method
        estimator = DualQuadricRobustEstimator.create(RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof RANSACDualQuadricRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        estimator = DualQuadricRobustEstimator.create(
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof LMedSDualQuadricRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());  
        
        estimator = DualQuadricRobustEstimator.create(
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof MSACDualQuadricRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());  

        estimator = DualQuadricRobustEstimator.create(
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof PROSACDualQuadricRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());  
        
        estimator = DualQuadricRobustEstimator.create(
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof PROMedSDualQuadricRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());  
        
        //test with lines and method
        List<Plane> planes = new ArrayList<Plane>();
        for(int i = 0; i < DualQuadricRobustEstimator.MINIMUM_SIZE; i++){
            planes.add(new Plane());
        }
        List<Plane> emptyPlanes = new ArrayList<Plane>();
        
        estimator = DualQuadricRobustEstimator.create(planes, 
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof RANSACDualQuadricRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());

        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = DualQuadricRobustEstimator.create(emptyPlanes, 
                    RobustEstimatorMethod.RANSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        estimator = DualQuadricRobustEstimator.create(planes, 
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof LMedSDualQuadricRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());

        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = DualQuadricRobustEstimator.create(emptyPlanes, 
                    RobustEstimatorMethod.LMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);    
        
        estimator = DualQuadricRobustEstimator.create(planes, 
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof MSACDualQuadricRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());

        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = DualQuadricRobustEstimator.create(emptyPlanes, 
                    RobustEstimatorMethod.MSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);    
        
        estimator = DualQuadricRobustEstimator.create(planes, 
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof PROSACDualQuadricRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());

        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = DualQuadricRobustEstimator.create(emptyPlanes, 
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator); 
        
        estimator = DualQuadricRobustEstimator.create(planes, 
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof PROMedSDualQuadricRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualConicRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualConicRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                DualConicRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());

        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = DualQuadricRobustEstimator.create(emptyPlanes, 
                    RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);   
        
        //test with listener
        DualQuadricRobustEstimatorListener listener = 
                new DualQuadricRobustEstimatorListener() {

            @Override
            public void onEstimateStart(DualQuadricRobustEstimator estimator) {}

            @Override
            public void onEstimateEnd(DualQuadricRobustEstimator estimator) {}

            @Override
            public void onEstimateNextIteration(
                    DualQuadricRobustEstimator estimator, int iteration) {}

            @Override
            public void onEstimateProgressChange(
                    DualQuadricRobustEstimator estimator, float progress) {}
        };
        
        estimator = DualQuadricRobustEstimator.create(listener, 
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof RANSACDualQuadricRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());        
        
        estimator = DualQuadricRobustEstimator.create(listener, 
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof LMedSDualQuadricRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());  
        
        estimator = DualQuadricRobustEstimator.create(listener, 
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof MSACDualQuadricRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());   
        
        estimator = DualQuadricRobustEstimator.create(listener, 
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof PROSACDualQuadricRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());         
        
        estimator = DualQuadricRobustEstimator.create(listener, 
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof PROMedSDualQuadricRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());         
        
        //test with listener, lines and method
        estimator = DualQuadricRobustEstimator.create(listener, planes,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof RANSACDualQuadricRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());        
        
        estimator = DualQuadricRobustEstimator.create(listener, planes,
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof LMedSDualQuadricRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());        

        estimator = DualQuadricRobustEstimator.create(listener, planes,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof MSACDualQuadricRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());        
        
        estimator = DualQuadricRobustEstimator.create(listener, planes,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof PROSACDualQuadricRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());        
        
        estimator = DualQuadricRobustEstimator.create(listener, planes,
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof PROMedSDualQuadricRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = DualQuadricRobustEstimator.create(listener, emptyPlanes,
                    RobustEstimatorMethod.RANSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with quality scores
        double[] qualityScores = new double[
                DualQuadricRobustEstimator.MINIMUM_SIZE];
        double[] emptyScores = new double[0];
        
        estimator = DualQuadricRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof RANSACDualQuadricRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        estimator = DualQuadricRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof LMedSDualQuadricRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        estimator = DualQuadricRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.MSAC);        
        assertTrue(estimator instanceof MSACDualQuadricRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());

        estimator = DualQuadricRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.PROSAC);        
        assertTrue(estimator instanceof PROSACDualQuadricRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);

        estimator = DualQuadricRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.PROMedS);        
        assertTrue(estimator instanceof PROMedSDualQuadricRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = DualQuadricRobustEstimator.create(emptyScores,
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //Test with lines and quality scores
        estimator = DualQuadricRobustEstimator.create(planes, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof RANSACDualQuadricRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        estimator = DualQuadricRobustEstimator.create(planes, qualityScores,
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof LMedSDualQuadricRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());

        estimator = DualQuadricRobustEstimator.create(planes, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof MSACDualQuadricRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        estimator = DualQuadricRobustEstimator.create(planes, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof PROSACDualQuadricRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);

        estimator = DualQuadricRobustEstimator.create(planes, qualityScores,
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof PROMedSDualQuadricRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = DualQuadricRobustEstimator.create(emptyPlanes, 
                    qualityScores, RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = DualQuadricRobustEstimator.create(planes, emptyScores,
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with listener and quality scores
        estimator = DualQuadricRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof RANSACDualQuadricRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        estimator = DualQuadricRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof LMedSDualQuadricRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());

        estimator = DualQuadricRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof MSACDualQuadricRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        estimator = DualQuadricRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof PROSACDualQuadricRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);

        estimator = DualQuadricRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof PROMedSDualQuadricRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = DualQuadricRobustEstimator.create(listener, emptyScores,
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with listener, lines and qualityScores
        estimator = DualQuadricRobustEstimator.create(listener, planes, 
                qualityScores, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof RANSACDualQuadricRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        estimator = DualQuadricRobustEstimator.create(listener, planes, 
                qualityScores, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof LMedSDualQuadricRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());

        estimator = DualQuadricRobustEstimator.create(listener, planes, 
                qualityScores, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof MSACDualQuadricRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        estimator = DualQuadricRobustEstimator.create(listener, planes, 
                qualityScores, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof PROSACDualQuadricRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        
        estimator = DualQuadricRobustEstimator.create(listener, planes, 
                qualityScores, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof PROMedSDualQuadricRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        
        estimator = null;
        try{
            estimator = DualQuadricRobustEstimator.create(listener, emptyPlanes,
                    qualityScores, RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = DualQuadricRobustEstimator.create(listener, planes, 
                    emptyScores, RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test without arguments
        estimator = DualQuadricRobustEstimator.create();
        assertTrue(estimator instanceof PROMedSDualQuadricRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualConicRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualConicRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                DualConicRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //test with lines
        estimator = DualQuadricRobustEstimator.create(planes);
        assertTrue(estimator instanceof PROMedSDualQuadricRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = DualQuadricRobustEstimator.create(emptyPlanes);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with listener
        estimator = DualQuadricRobustEstimator.create(listener);
        assertTrue(estimator instanceof PROMedSDualQuadricRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //test with listener and lines
        estimator = DualQuadricRobustEstimator.create(listener, planes);
        assertTrue(estimator instanceof PROMedSDualQuadricRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = DualQuadricRobustEstimator.create(listener, 
                    emptyPlanes);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with quality scores
        estimator = DualQuadricRobustEstimator.create(qualityScores);
        assertTrue(estimator instanceof PROMedSDualQuadricRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = DualQuadricRobustEstimator.create(emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with lines and quality scores
        estimator = DualQuadricRobustEstimator.create(planes, qualityScores);
        assertTrue(estimator instanceof PROMedSDualQuadricRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = DualQuadricRobustEstimator.create(emptyPlanes, 
                    qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = DualQuadricRobustEstimator.create(planes, emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}        
        assertNull(estimator);
        
        //test with listener and quality scores
        estimator = DualQuadricRobustEstimator.create(listener, qualityScores);
        assertTrue(estimator instanceof PROMedSDualQuadricRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = DualQuadricRobustEstimator.create(listener, 
                    emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with listener, lines and quality scores
        estimator = DualQuadricRobustEstimator.create(listener, planes, 
                qualityScores);
        assertTrue(estimator instanceof PROMedSDualQuadricRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);  
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = DualQuadricRobustEstimator.create(listener, emptyPlanes, 
                    qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = DualQuadricRobustEstimator.create(listener, planes, 
                    emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}        
        assertNull(estimator);        
    }    
}
