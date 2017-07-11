/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.estimators.EuclideanTransformation3DRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date February 27, 2017.
 */
package com.irurueta.geometry.estimators;

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

public class EuclideanTransformation3DRobustEstimatorTest {
    
    public EuclideanTransformation3DRobustEstimatorTest() { }
    
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
        EuclideanTransformation3DRobustEstimator estimator;
                
        //create with points and method
        List<Point3D> inputPoints = new ArrayList<Point3D>();
        List<Point3D> outputPoints = new ArrayList<Point3D>();
        for (int i = 0; i < EuclideanTransformation3DRobustEstimator.MINIMUM_SIZE; i++) {
            inputPoints.add(Point3D.create());
            outputPoints.add(Point3D.create());
        }
        
        estimator = EuclideanTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACEuclideanTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = EuclideanTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSEuclideanTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        

        estimator = EuclideanTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                MSACEuclideanTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = EuclideanTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACEuclideanTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = EuclideanTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSEuclideanTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        List<Point3D> emptyPoints = new ArrayList<Point3D>();
        
        estimator = null;
        try{
            estimator = EuclideanTransformation3DRobustEstimator.create(
                    emptyPoints, outputPoints, RobustEstimatorMethod.LMedS);
            fail("IllegalArugmentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = EuclideanTransformation3DRobustEstimator.create(
                    inputPoints, emptyPoints, RobustEstimatorMethod.LMedS);
            fail("IllegalArugmentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
                                
        //test with listener and points
        EuclideanTransformation3DRobustEstimatorListener listener = 
                new EuclideanTransformation3DRobustEstimatorListener() {

            @Override
            public void onEstimateStart(
                    EuclideanTransformation3DRobustEstimator estimator) { }

            @Override
            public void onEstimateEnd(
                    EuclideanTransformation3DRobustEstimator estimator) { }

            @Override
            public void onEstimateNextIteration(
                    EuclideanTransformation3DRobustEstimator estimator, 
                    int iteration) { }

            @Override
            public void onEstimateProgressChange(
                    EuclideanTransformation3DRobustEstimator estimator, 
                    float progress) { }
        };
        
        estimator = EuclideanTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, 
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACEuclideanTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = EuclideanTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, 
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSEuclideanTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = EuclideanTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, 
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACEuclideanTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = EuclideanTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, 
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACEuclideanTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = EuclideanTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, 
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSEuclideanTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        

        
        //test with points, quality scores and method
        double[] qualityScores = new double[
                EuclideanTransformation3DRobustEstimator.MINIMUM_SIZE];
        double[] wrongQualityScores = new double[1];
        
        estimator = EuclideanTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores, 
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACEuclideanTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = EuclideanTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores, 
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSEuclideanTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = EuclideanTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores, 
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACEuclideanTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = EuclideanTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores, 
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACEuclideanTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = EuclideanTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores, 
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSEuclideanTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = EuclideanTransformation3DRobustEstimator.create(
                    emptyPoints, outputPoints, qualityScores, 
                    RobustEstimatorMethod.PROMedS);            
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = EuclideanTransformation3DRobustEstimator.create(
                    inputPoints, emptyPoints, qualityScores, 
                    RobustEstimatorMethod.PROMedS);            
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = EuclideanTransformation3DRobustEstimator.create(
                    inputPoints, outputPoints, wrongQualityScores, 
                    RobustEstimatorMethod.PROMedS);            
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with listener, points, quality scores and method
        estimator = EuclideanTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACEuclideanTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = EuclideanTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSEuclideanTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = EuclideanTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACEuclideanTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = EuclideanTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACEuclideanTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = EuclideanTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSEuclideanTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = EuclideanTransformation3DRobustEstimator.create(
                    listener, emptyPoints, outputPoints, qualityScores,
                    RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = EuclideanTransformation3DRobustEstimator.create(
                    listener, inputPoints, emptyPoints, qualityScores,
                    RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = EuclideanTransformation3DRobustEstimator.create(
                    listener, inputPoints, outputPoints, wrongQualityScores,
                    RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        
        //test with points
        estimator = EuclideanTransformation3DRobustEstimator.create(
                inputPoints, outputPoints);
        assertTrue(estimator instanceof
                PROMedSEuclideanTransformation3DRobustEstimator);        
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = EuclideanTransformation3DRobustEstimator.create(
                    emptyPoints, outputPoints);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = EuclideanTransformation3DRobustEstimator.create(
                    inputPoints, emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}        
        assertNull(estimator);
        
        //test with listener and points
        estimator = EuclideanTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints);
        assertTrue(estimator instanceof
                PROMedSEuclideanTransformation3DRobustEstimator);        
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = EuclideanTransformation3DRobustEstimator.create(
                    listener, emptyPoints, outputPoints);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = EuclideanTransformation3DRobustEstimator.create(
                    listener, inputPoints, emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        
        //test with points and quality scores
        estimator = EuclideanTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores);
        assertTrue(estimator instanceof
                PROMedSEuclideanTransformation3DRobustEstimator);        
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        
        //test with listener, points and quality scores
        estimator = EuclideanTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores);
        assertTrue(estimator instanceof
                PROMedSEuclideanTransformation3DRobustEstimator);        
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        
        //create with points and method and weak points
        inputPoints = new ArrayList<Point3D>();
        outputPoints = new ArrayList<Point3D>();
        for (int i = 0; i < EuclideanTransformation3DRobustEstimator.WEAK_MINIMUM_SIZE; i++) {
            inputPoints.add(Point3D.create());
            outputPoints.add(Point3D.create());
        }
        
        estimator = EuclideanTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, true, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACEuclideanTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = EuclideanTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, true, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSEuclideanTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        

        estimator = EuclideanTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, true, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                MSACEuclideanTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = EuclideanTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, true, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACEuclideanTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = EuclideanTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, true, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSEuclideanTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = EuclideanTransformation3DRobustEstimator.create(
                    emptyPoints, outputPoints, true, 
                    RobustEstimatorMethod.LMedS);
            fail("IllegalArugmentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = EuclideanTransformation3DRobustEstimator.create(
                    inputPoints, emptyPoints, true, 
                    RobustEstimatorMethod.LMedS);
            fail("IllegalArugmentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
                                
        //test with listener and points        
        estimator = EuclideanTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, true,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACEuclideanTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = EuclideanTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, true,
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSEuclideanTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = EuclideanTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, true,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACEuclideanTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = EuclideanTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, true,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACEuclideanTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = EuclideanTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, true,
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSEuclideanTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        

        
        //test with points, quality scores and method
        qualityScores = new double[
                EuclideanTransformation3DRobustEstimator.WEAK_MINIMUM_SIZE];
        
        estimator = EuclideanTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores, true,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACEuclideanTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = EuclideanTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores, true,
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSEuclideanTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = EuclideanTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores, true,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACEuclideanTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = EuclideanTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores, true,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACEuclideanTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = EuclideanTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores, true,
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSEuclideanTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = EuclideanTransformation3DRobustEstimator.create(
                    emptyPoints, outputPoints, qualityScores, true,
                    RobustEstimatorMethod.PROMedS);            
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = EuclideanTransformation3DRobustEstimator.create(
                    inputPoints, emptyPoints, qualityScores, true,
                    RobustEstimatorMethod.PROMedS);            
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = EuclideanTransformation3DRobustEstimator.create(
                    inputPoints, outputPoints, wrongQualityScores, true,
                    RobustEstimatorMethod.PROMedS);            
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        
        //test with listener, points, quality scores and method
        estimator = EuclideanTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores, true,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACEuclideanTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = EuclideanTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores, true,
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSEuclideanTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = EuclideanTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores, true,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACEuclideanTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = EuclideanTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores, true,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACEuclideanTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = EuclideanTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores, true,
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSEuclideanTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = EuclideanTransformation3DRobustEstimator.create(
                    listener, emptyPoints, outputPoints, qualityScores, true,
                    RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = EuclideanTransformation3DRobustEstimator.create(
                    listener, inputPoints, emptyPoints, qualityScores, true,
                    RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = EuclideanTransformation3DRobustEstimator.create(
                    listener, inputPoints, outputPoints, wrongQualityScores, 
                    true, RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        
        //test with points
        estimator = EuclideanTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, true);
        assertTrue(estimator instanceof
                PROMedSEuclideanTransformation3DRobustEstimator);        
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = EuclideanTransformation3DRobustEstimator.create(
                    emptyPoints, outputPoints, true);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = EuclideanTransformation3DRobustEstimator.create(
                    inputPoints, emptyPoints, true);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}        
        assertNull(estimator);
        
        
        //test with listener and points
        estimator = EuclideanTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, true);
        assertTrue(estimator instanceof
                PROMedSEuclideanTransformation3DRobustEstimator);        
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = EuclideanTransformation3DRobustEstimator.create(
                    listener, emptyPoints, outputPoints, true);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = EuclideanTransformation3DRobustEstimator.create(
                    listener, inputPoints, emptyPoints, true);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        
        //test with points and quality scores
        estimator = EuclideanTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores, true);
        assertTrue(estimator instanceof
                PROMedSEuclideanTransformation3DRobustEstimator);        
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        
        //test with listener, points and quality scores
        estimator = EuclideanTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores, true);
        assertTrue(estimator instanceof
                PROMedSEuclideanTransformation3DRobustEstimator);        
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());        
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());                
    }    
}