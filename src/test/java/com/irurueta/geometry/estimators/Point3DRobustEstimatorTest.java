/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.estimators.Point3DRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date March3, 2015
 */
package com.irurueta.geometry.estimators;

import com.irurueta.geometry.CoordinatesType;
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

public class Point3DRobustEstimatorTest {
    
    public Point3DRobustEstimatorTest() {}
    
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
        Point3DRobustEstimator estimator;
        
        //test with robust method
        estimator = Point3DRobustEstimator.create(RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof RANSACPoint3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isResultRefined());
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        
        estimator = Point3DRobustEstimator.create(RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof LMedSPoint3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());  
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isResultRefined());
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertFalse(estimator.isCovarianceKept());        
        assertNull(estimator.getCovariance());
        
        estimator = Point3DRobustEstimator.create(RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof MSACPoint3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());  
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isResultRefined());
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertFalse(estimator.isCovarianceKept());       
        assertNull(estimator.getCovariance());

        estimator = Point3DRobustEstimator.create(RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof PROSACPoint3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());  
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isResultRefined());
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertFalse(estimator.isCovarianceKept());        
        assertNull(estimator.getCovariance());
        
        estimator = Point3DRobustEstimator.create(RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof PROMedSPoint3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());  
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isResultRefined());
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertFalse(estimator.isCovarianceKept());      
        assertNull(estimator.getCovariance());
        
        //test with planes and method
        List<Plane> planes = new ArrayList<Plane>();
        for(int i = 0; i < Point3DRobustEstimator.MINIMUM_SIZE; i++){
            planes.add(new Plane());
        }
        List<Plane> emptyPlanes = new ArrayList<Plane>();
        
        estimator = Point3DRobustEstimator.create(planes, 
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof RANSACPoint3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isResultRefined());
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertFalse(estimator.isCovarianceKept());     
        assertNull(estimator.getCovariance());

        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = Point3DRobustEstimator.create(emptyPlanes, 
                    RobustEstimatorMethod.RANSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        estimator = Point3DRobustEstimator.create(planes, 
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof LMedSPoint3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isResultRefined());
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertFalse(estimator.isCovarianceKept());    
        assertNull(estimator.getCovariance());

        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = Point3DRobustEstimator.create(emptyPlanes, 
                    RobustEstimatorMethod.LMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);    
        
        estimator = Point3DRobustEstimator.create(planes, 
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof MSACPoint3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isResultRefined());
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertFalse(estimator.isCovarianceKept());  
        assertNull(estimator.getCovariance());

        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = Point3DRobustEstimator.create(emptyPlanes, 
                    RobustEstimatorMethod.MSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);    
        
        estimator = Point3DRobustEstimator.create(planes, 
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof PROSACPoint3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isResultRefined());
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertFalse(estimator.isCovarianceKept());        
        assertNull(estimator.getCovariance());

        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = Point3DRobustEstimator.create(emptyPlanes, 
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator); 
        
        estimator = Point3DRobustEstimator.create(planes, 
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof PROMedSPoint3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isResultRefined());
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertFalse(estimator.isCovarianceKept());      
        assertNull(estimator.getCovariance());

        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = Point3DRobustEstimator.create(emptyPlanes, 
                    RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);   
        
        //test with listener
        Point3DRobustEstimatorListener listener = new Point3DRobustEstimatorListener() {

            @Override
            public void onEstimateStart(Point3DRobustEstimator estimator) {}

            @Override
            public void onEstimateEnd(Point3DRobustEstimator estimator) {}

            @Override
            public void onEstimateNextIteration(Point3DRobustEstimator estimator, 
                    int iteration) {}

            @Override
            public void onEstimateProgressChange(Point3DRobustEstimator estimator, 
                    float progress) {}
        };
        
        estimator = Point3DRobustEstimator.create(listener, 
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof RANSACPoint3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());     
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isResultRefined());
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertFalse(estimator.isCovarianceKept());        
        assertNull(estimator.getCovariance());
        
        estimator = Point3DRobustEstimator.create(listener, 
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof LMedSPoint3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());  
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isResultRefined());
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertFalse(estimator.isCovarianceKept());       
        assertNull(estimator.getCovariance());
        
        estimator = Point3DRobustEstimator.create(listener, 
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof MSACPoint3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());  
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isResultRefined());
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertFalse(estimator.isCovarianceKept());        
        assertNull(estimator.getCovariance());
        
        estimator = Point3DRobustEstimator.create(listener, 
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof PROSACPoint3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());   
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isResultRefined());
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertFalse(estimator.isCovarianceKept());        
        assertNull(estimator.getCovariance());
        
        estimator = Point3DRobustEstimator.create(listener, 
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof PROMedSPoint3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());  
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isResultRefined());
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertFalse(estimator.isCovarianceKept());      
        assertNull(estimator.getCovariance());
        
        //test with listener, planes and method
        estimator = Point3DRobustEstimator.create(listener, planes,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof RANSACPoint3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());    
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isResultRefined());
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertFalse(estimator.isCovarianceKept());  
        assertNull(estimator.getCovariance());
        
        estimator = Point3DRobustEstimator.create(listener, planes,
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof LMedSPoint3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());       
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isResultRefined());
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertFalse(estimator.isCovarianceKept());     
        assertNull(estimator.getCovariance());

        estimator = Point3DRobustEstimator.create(listener, planes,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof MSACPoint3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());  
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isResultRefined());
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertFalse(estimator.isCovarianceKept());        
        assertNull(estimator.getCovariance());
        
        estimator = Point3DRobustEstimator.create(listener, planes,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof PROSACPoint3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());   
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isResultRefined());
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertFalse(estimator.isCovarianceKept());        
        assertNull(estimator.getCovariance());
        
        estimator = Point3DRobustEstimator.create(listener, planes,
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof PROMedSPoint3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());    
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isResultRefined());
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertFalse(estimator.isCovarianceKept());        
        assertNull(estimator.getCovariance());
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = Point3DRobustEstimator.create(listener, emptyPlanes,
                    RobustEstimatorMethod.RANSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with quality scores
        double[] qualityScores = new double[Point3DRobustEstimator.MINIMUM_SIZE];
        double[] emptyScores = new double[0];
        
        estimator = Point3DRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof RANSACPoint3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isResultRefined());
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertFalse(estimator.isCovarianceKept());       
        assertNull(estimator.getCovariance());
        
        estimator = Point3DRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof LMedSPoint3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isResultRefined());
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertFalse(estimator.isCovarianceKept());       
        assertNull(estimator.getCovariance());
        
        estimator = Point3DRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.MSAC);        
        assertTrue(estimator instanceof MSACPoint3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isResultRefined());
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertFalse(estimator.isCovarianceKept());      
        assertNull(estimator.getCovariance());

        estimator = Point3DRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.PROSAC);        
        assertTrue(estimator instanceof PROSACPoint3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isResultRefined());
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertFalse(estimator.isCovarianceKept());    
        assertNull(estimator.getCovariance());

        estimator = Point3DRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.PROMedS);        
        assertTrue(estimator instanceof PROMedSPoint3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isResultRefined());
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertFalse(estimator.isCovarianceKept());   
        assertNull(estimator.getCovariance());
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = Point3DRobustEstimator.create(emptyScores,
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //Test with planes and quality scores
        estimator = Point3DRobustEstimator.create(planes, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof RANSACPoint3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isResultRefined());
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertFalse(estimator.isCovarianceKept());   
        assertNull(estimator.getCovariance());
        
        estimator = Point3DRobustEstimator.create(planes, qualityScores,
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof LMedSPoint3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isResultRefined());
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertFalse(estimator.isCovarianceKept());  
        assertNull(estimator.getCovariance());

        estimator = Point3DRobustEstimator.create(planes, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof MSACPoint3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isResultRefined());
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertFalse(estimator.isCovarianceKept());     
        assertNull(estimator.getCovariance());
        
        estimator = Point3DRobustEstimator.create(planes, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof PROSACPoint3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isResultRefined());
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertFalse(estimator.isCovarianceKept());     
        assertNull(estimator.getCovariance());

        estimator = Point3DRobustEstimator.create(planes, qualityScores,
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof PROMedSPoint3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isResultRefined());
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertFalse(estimator.isCovarianceKept());       
        assertNull(estimator.getCovariance());
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = Point3DRobustEstimator.create(emptyPlanes, qualityScores,
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = Point3DRobustEstimator.create(planes, emptyScores,
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with listener and quality scores
        estimator = Point3DRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof RANSACPoint3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isResultRefined());
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertFalse(estimator.isCovarianceKept());    
        assertNull(estimator.getCovariance());
        
        estimator = Point3DRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof LMedSPoint3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isResultRefined());
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertFalse(estimator.isCovarianceKept());      
        assertNull(estimator.getCovariance());

        estimator = Point3DRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof MSACPoint3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isResultRefined());
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertFalse(estimator.isCovarianceKept());      
        assertNull(estimator.getCovariance());
        
        estimator = Point3DRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof PROSACPoint3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isResultRefined());
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertFalse(estimator.isCovarianceKept());  
        assertNull(estimator.getCovariance());

        estimator = Point3DRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof PROMedSPoint3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isResultRefined());
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertFalse(estimator.isCovarianceKept());   
        assertNull(estimator.getCovariance());
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = Point3DRobustEstimator.create(listener, emptyScores,
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with listener, planes and qualityScores
        estimator = Point3DRobustEstimator.create(listener, planes, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof RANSACPoint3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isResultRefined());
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertFalse(estimator.isCovarianceKept());    
        assertNull(estimator.getCovariance());
        
        estimator = Point3DRobustEstimator.create(listener, planes, qualityScores,
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof LMedSPoint3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isResultRefined());
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertFalse(estimator.isCovarianceKept());     
        assertNull(estimator.getCovariance());

        estimator = Point3DRobustEstimator.create(listener, planes, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof MSACPoint3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isResultRefined());
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertFalse(estimator.isCovarianceKept());     
        assertNull(estimator.getCovariance());
        
        estimator = Point3DRobustEstimator.create(listener, planes, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof PROSACPoint3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isResultRefined());
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertFalse(estimator.isCovarianceKept());     
        assertNull(estimator.getCovariance());
        
        estimator = Point3DRobustEstimator.create(listener, planes, qualityScores,
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof PROMedSPoint3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isResultRefined());
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertFalse(estimator.isCovarianceKept());     
        assertNull(estimator.getCovariance());
        
        estimator = null;
        try{
            estimator = Point3DRobustEstimator.create(listener, emptyPlanes,
                    qualityScores, RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = Point3DRobustEstimator.create(listener, planes, 
                    emptyScores, RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test without arguments
        estimator = Point3DRobustEstimator.create();
        assertTrue(estimator instanceof PROMedSPoint3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isResultRefined());
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertFalse(estimator.isCovarianceKept());   
        assertNull(estimator.getCovariance());
        
        //test with planes
        estimator = Point3DRobustEstimator.create(planes);
        assertTrue(estimator instanceof PROMedSPoint3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isResultRefined());
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertFalse(estimator.isCovarianceKept());     
        assertNull(estimator.getCovariance());
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = Point3DRobustEstimator.create(emptyPlanes);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with listener
        estimator = Point3DRobustEstimator.create(listener);
        assertTrue(estimator instanceof PROMedSPoint3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isResultRefined());
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertFalse(estimator.isCovarianceKept());    
        assertNull(estimator.getCovariance());
        
        //test with listener and planes
        estimator = Point3DRobustEstimator.create(listener, planes);
        assertTrue(estimator instanceof PROMedSPoint3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isResultRefined());
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertFalse(estimator.isCovarianceKept());     
        assertNull(estimator.getCovariance());
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = Point3DRobustEstimator.create(listener, emptyPlanes);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with quality scores
        estimator = Point3DRobustEstimator.create(qualityScores);
        assertTrue(estimator instanceof PROMedSPoint3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isResultRefined());
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertFalse(estimator.isCovarianceKept());    
        assertNull(estimator.getCovariance());
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = Point3DRobustEstimator.create(emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with planes and quality scores
        estimator = Point3DRobustEstimator.create(planes, qualityScores);
        assertTrue(estimator instanceof PROMedSPoint3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isResultRefined());
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertFalse(estimator.isCovarianceKept());        
        assertNull(estimator.getCovariance());
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = Point3DRobustEstimator.create(emptyPlanes, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = Point3DRobustEstimator.create(planes, emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}        
        assertNull(estimator);
        
        //test with listener and quality scores
        estimator = Point3DRobustEstimator.create(listener, qualityScores);
        assertTrue(estimator instanceof PROMedSPoint3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isResultRefined());
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertFalse(estimator.isCovarianceKept());    
        assertNull(estimator.getCovariance());
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = Point3DRobustEstimator.create(listener, emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with listener, planes and quality scores
        estimator = Point3DRobustEstimator.create(listener, planes, 
                qualityScores);
        assertTrue(estimator instanceof PROMedSPoint3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);  
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isResultRefined());
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertFalse(estimator.isCovarianceKept());    
        assertNull(estimator.getCovariance());
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = Point3DRobustEstimator.create(listener, emptyPlanes, 
                    qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = Point3DRobustEstimator.create(listener, planes, 
                    emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}        
        assertNull(estimator);        
    }    
}
