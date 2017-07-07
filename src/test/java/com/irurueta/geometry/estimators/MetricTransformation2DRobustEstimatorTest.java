/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.estimators.MetricTransformation2DRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date February 26, 2017.
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

public class MetricTransformation2DRobustEstimatorTest {
    
    public MetricTransformation2DRobustEstimatorTest() { }
    
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
        MetricTransformation2DRobustEstimator estimator;
                
        //create with points and method
        List<Point2D> inputPoints = new ArrayList<Point2D>();
        List<Point2D> outputPoints = new ArrayList<Point2D>();
        for (int i = 0; i < MetricTransformation2DRobustEstimator.MINIMUM_SIZE; i++) {
            inputPoints.add(Point2D.create());
            outputPoints.add(Point2D.create());
        }
        
        estimator = MetricTransformation2DRobustEstimator.create(
                inputPoints, outputPoints, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACMetricTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation2DRobustEstimator.create(
                inputPoints, outputPoints, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSMetricTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        

        estimator = MetricTransformation2DRobustEstimator.create(
                inputPoints, outputPoints, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                MSACMetricTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation2DRobustEstimator.create(
                inputPoints, outputPoints, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACMetricTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation2DRobustEstimator.create(
                inputPoints, outputPoints, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSMetricTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        List<Point2D> emptyPoints = new ArrayList<Point2D>();
        
        estimator = null;
        try{
            estimator = MetricTransformation2DRobustEstimator.create(
                    emptyPoints, outputPoints, RobustEstimatorMethod.LMedS);
            fail("IllegalArugmentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = MetricTransformation2DRobustEstimator.create(
                    inputPoints, emptyPoints, RobustEstimatorMethod.LMedS);
            fail("IllegalArugmentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
                                
        //test with listener and points
        MetricTransformation2DRobustEstimatorListener listener = 
                new MetricTransformation2DRobustEstimatorListener() {

            @Override
            public void onEstimateStart(
                    MetricTransformation2DRobustEstimator estimator) { }

            @Override
            public void onEstimateEnd(
                    MetricTransformation2DRobustEstimator estimator) { }

            @Override
            public void onEstimateNextIteration(
                    MetricTransformation2DRobustEstimator estimator, 
                    int iteration) { }

            @Override
            public void onEstimateProgressChange(
                    MetricTransformation2DRobustEstimator estimator, 
                    float progress) { }
        };
        
        estimator = MetricTransformation2DRobustEstimator.create(
                listener, inputPoints, outputPoints, 
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACMetricTransformation2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation2DRobustEstimator.create(
                listener, inputPoints, outputPoints, 
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSMetricTransformation2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation2DRobustEstimator.create(
                listener, inputPoints, outputPoints, 
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACMetricTransformation2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation2DRobustEstimator.create(
                listener, inputPoints, outputPoints, 
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACMetricTransformation2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation2DRobustEstimator.create(
                listener, inputPoints, outputPoints, 
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
                        
        //test with points, quality scores and method
        double[] qualityScores = new double[
                MetricTransformation2DRobustEstimator.MINIMUM_SIZE];
        double[] wrongQualityScores = new double[1];
        
        estimator = MetricTransformation2DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores, 
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACMetricTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation2DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores, 
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSMetricTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation2DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores, 
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACMetricTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation2DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores, 
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACMetricTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation2DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores, 
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = MetricTransformation2DRobustEstimator.create(
                    emptyPoints, outputPoints, qualityScores, 
                    RobustEstimatorMethod.PROMedS);            
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = MetricTransformation2DRobustEstimator.create(
                    inputPoints, emptyPoints, qualityScores, 
                    RobustEstimatorMethod.PROMedS);            
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = MetricTransformation2DRobustEstimator.create(
                    inputPoints, outputPoints, wrongQualityScores, 
                    RobustEstimatorMethod.PROMedS);            
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with listener, points, quality scores and method
        estimator = MetricTransformation2DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACMetricTransformation2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation2DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSMetricTransformation2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation2DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACMetricTransformation2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation2DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACMetricTransformation2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation2DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = MetricTransformation2DRobustEstimator.create(
                    listener, emptyPoints, outputPoints, qualityScores,
                    RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = MetricTransformation2DRobustEstimator.create(
                    listener, inputPoints, emptyPoints, qualityScores,
                    RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = MetricTransformation2DRobustEstimator.create(
                    listener, inputPoints, outputPoints, wrongQualityScores,
                    RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with points
        estimator = MetricTransformation2DRobustEstimator.create(
                inputPoints, outputPoints);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation2DRobustEstimator);        
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = MetricTransformation2DRobustEstimator.create(
                    emptyPoints, outputPoints);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = MetricTransformation2DRobustEstimator.create(
                    inputPoints, emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}        
        assertNull(estimator);
        
        //test with listener and points
        estimator = MetricTransformation2DRobustEstimator.create(
                listener, inputPoints, outputPoints);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation2DRobustEstimator);        
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = MetricTransformation2DRobustEstimator.create(
                    listener, emptyPoints, outputPoints);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = MetricTransformation2DRobustEstimator.create(
                    listener, inputPoints, emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with points and quality scores
        estimator = MetricTransformation2DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation2DRobustEstimator);        
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //test with listener, points and quality scores
        estimator = MetricTransformation2DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation2DRobustEstimator);        
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        
        //create with points and method and weak points
        inputPoints = new ArrayList<Point2D>();
        outputPoints = new ArrayList<Point2D>();
        for (int i = 0; i < MetricTransformation2DRobustEstimator.WEAK_MINIMUM_SIZE; i++) {
            inputPoints.add(Point2D.create());
            outputPoints.add(Point2D.create());
        }
        
        estimator = MetricTransformation2DRobustEstimator.create(
                inputPoints, outputPoints, true, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACMetricTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation2DRobustEstimator.create(
                inputPoints, outputPoints, true, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSMetricTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        

        estimator = MetricTransformation2DRobustEstimator.create(
                inputPoints, outputPoints, true, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                MSACMetricTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation2DRobustEstimator.create(
                inputPoints, outputPoints, true, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACMetricTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation2DRobustEstimator.create(
                inputPoints, outputPoints, true, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSMetricTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = MetricTransformation2DRobustEstimator.create(
                    emptyPoints, outputPoints, true, 
                    RobustEstimatorMethod.LMedS);
            fail("IllegalArugmentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = MetricTransformation2DRobustEstimator.create(
                    inputPoints, emptyPoints, true, 
                    RobustEstimatorMethod.LMedS);
            fail("IllegalArugmentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
                                
        //test with listener and points        
        estimator = MetricTransformation2DRobustEstimator.create(
                listener, inputPoints, outputPoints, true,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACMetricTransformation2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation2DRobustEstimator.create(
                listener, inputPoints, outputPoints, true, 
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSMetricTransformation2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation2DRobustEstimator.create(
                listener, inputPoints, outputPoints, true,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACMetricTransformation2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation2DRobustEstimator.create(
                listener, inputPoints, outputPoints, true,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACMetricTransformation2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation2DRobustEstimator.create(
                listener, inputPoints, outputPoints, true,
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
                        
        //test with points, quality scores and method
        qualityScores = new double[
                MetricTransformation2DRobustEstimator.WEAK_MINIMUM_SIZE];
        
        estimator = MetricTransformation2DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores, true,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACMetricTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation2DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores, true,
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSMetricTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation2DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores, true,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACMetricTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation2DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores, true,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACMetricTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation2DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores, true,
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = MetricTransformation2DRobustEstimator.create(
                    emptyPoints, outputPoints, qualityScores, true,
                    RobustEstimatorMethod.PROMedS);            
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = MetricTransformation2DRobustEstimator.create(
                    inputPoints, emptyPoints, qualityScores, true,
                    RobustEstimatorMethod.PROMedS);            
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = MetricTransformation2DRobustEstimator.create(
                    inputPoints, outputPoints, wrongQualityScores, true,
                    RobustEstimatorMethod.PROMedS);            
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with listener, points, quality scores and method
        estimator = MetricTransformation2DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores, true,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACMetricTransformation2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation2DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores, true,
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSMetricTransformation2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation2DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores, true,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACMetricTransformation2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation2DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores, true,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACMetricTransformation2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation2DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores, true,
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = MetricTransformation2DRobustEstimator.create(
                    listener, emptyPoints, outputPoints, qualityScores, true,
                    RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = MetricTransformation2DRobustEstimator.create(
                    listener, inputPoints, emptyPoints, qualityScores, true,
                    RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = MetricTransformation2DRobustEstimator.create(
                    listener, inputPoints, outputPoints, wrongQualityScores, 
                    true, RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with points
        estimator = MetricTransformation2DRobustEstimator.create(
                inputPoints, outputPoints, true);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation2DRobustEstimator);        
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = MetricTransformation2DRobustEstimator.create(
                    emptyPoints, outputPoints, true);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = MetricTransformation2DRobustEstimator.create(
                    inputPoints, emptyPoints, true);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}        
        assertNull(estimator);
        
        //test with listener and points
        estimator = MetricTransformation2DRobustEstimator.create(
                listener, inputPoints, outputPoints, true);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation2DRobustEstimator);        
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = MetricTransformation2DRobustEstimator.create(
                    listener, emptyPoints, outputPoints, true);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = MetricTransformation2DRobustEstimator.create(
                    listener, inputPoints, emptyPoints, true);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with points and quality scores
        estimator = MetricTransformation2DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores, true);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation2DRobustEstimator);        
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //test with listener, points and quality scores
        estimator = MetricTransformation2DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores, true);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation2DRobustEstimator);        
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());        
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());                
    }        
}
