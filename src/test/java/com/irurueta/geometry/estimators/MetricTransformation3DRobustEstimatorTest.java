/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.estimators.MetricTransformation2DRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date February 28, 2017.
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

public class MetricTransformation3DRobustEstimatorTest {
    
    public MetricTransformation3DRobustEstimatorTest() { }
    
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
        MetricTransformation3DRobustEstimator estimator;
                
        //create with points and method
        List<Point3D> inputPoints = new ArrayList<Point3D>();
        List<Point3D> outputPoints = new ArrayList<Point3D>();
        for (int i = 0; i < MetricTransformation3DRobustEstimator.MINIMUM_SIZE; i++) {
            inputPoints.add(Point3D.create());
            outputPoints.add(Point3D.create());
        }
        
        estimator = MetricTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACMetricTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSMetricTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        

        estimator = MetricTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                MSACMetricTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACMetricTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSMetricTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        List<Point3D> emptyPoints = new ArrayList<Point3D>();
        
        estimator = null;
        try{
            estimator = MetricTransformation3DRobustEstimator.create(
                    emptyPoints, outputPoints, RobustEstimatorMethod.LMedS);
            fail("IllegalArugmentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = MetricTransformation3DRobustEstimator.create(
                    inputPoints, emptyPoints, RobustEstimatorMethod.LMedS);
            fail("IllegalArugmentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
                                
        //test with listener and points
        MetricTransformation3DRobustEstimatorListener listener = 
                new MetricTransformation3DRobustEstimatorListener() {

            @Override
            public void onEstimateStart(
                    MetricTransformation3DRobustEstimator estimator) { }

            @Override
            public void onEstimateEnd(
                    MetricTransformation3DRobustEstimator estimator) { }

            @Override
            public void onEstimateNextIteration(
                    MetricTransformation3DRobustEstimator estimator, 
                    int iteration) { }

            @Override
            public void onEstimateProgressChange(
                    MetricTransformation3DRobustEstimator estimator, 
                    float progress) { }
        };
        
        estimator = MetricTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, 
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACMetricTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, 
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSMetricTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, 
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACMetricTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, 
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACMetricTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, 
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
                        
        //test with points, quality scores and method
        double[] qualityScores = new double[
                MetricTransformation3DRobustEstimator.MINIMUM_SIZE];
        double[] wrongQualityScores = new double[1];
        
        estimator = MetricTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores, 
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACMetricTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores, 
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSMetricTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores, 
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACMetricTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores, 
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACMetricTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores, 
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = MetricTransformation3DRobustEstimator.create(
                    emptyPoints, outputPoints, qualityScores, 
                    RobustEstimatorMethod.PROMedS);            
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = MetricTransformation3DRobustEstimator.create(
                    inputPoints, emptyPoints, qualityScores, 
                    RobustEstimatorMethod.PROMedS);            
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = MetricTransformation3DRobustEstimator.create(
                    inputPoints, outputPoints, wrongQualityScores, 
                    RobustEstimatorMethod.PROMedS);            
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with listener, points, quality scores and method
        estimator = MetricTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACMetricTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSMetricTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACMetricTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACMetricTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = MetricTransformation3DRobustEstimator.create(
                    listener, emptyPoints, outputPoints, qualityScores,
                    RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = MetricTransformation3DRobustEstimator.create(
                    listener, inputPoints, emptyPoints, qualityScores,
                    RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = MetricTransformation3DRobustEstimator.create(
                    listener, inputPoints, outputPoints, wrongQualityScores,
                    RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with points
        estimator = MetricTransformation3DRobustEstimator.create(
                inputPoints, outputPoints);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation3DRobustEstimator);        
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = MetricTransformation3DRobustEstimator.create(
                    emptyPoints, outputPoints);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = MetricTransformation3DRobustEstimator.create(
                    inputPoints, emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}        
        assertNull(estimator);
        
        //test with listener and points
        estimator = MetricTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation3DRobustEstimator);        
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = MetricTransformation3DRobustEstimator.create(
                    listener, emptyPoints, outputPoints);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = MetricTransformation3DRobustEstimator.create(
                    listener, inputPoints, emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with points and quality scores
        estimator = MetricTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation3DRobustEstimator);        
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //test with listener, points and quality scores
        estimator = MetricTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation3DRobustEstimator);        
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        
        //create with points and method and weak points
        inputPoints = new ArrayList<Point3D>();
        outputPoints = new ArrayList<Point3D>();
        for (int i = 0; i < MetricTransformation3DRobustEstimator.WEAK_MINIMUM_SIZE; i++) {
            inputPoints.add(Point3D.create());
            outputPoints.add(Point3D.create());
        }
        
        estimator = MetricTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, true, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACMetricTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, true, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSMetricTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        

        estimator = MetricTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, true, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                MSACMetricTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, true, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACMetricTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, true, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSMetricTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = MetricTransformation3DRobustEstimator.create(
                    emptyPoints, outputPoints, true, 
                    RobustEstimatorMethod.LMedS);
            fail("IllegalArugmentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = MetricTransformation3DRobustEstimator.create(
                    inputPoints, emptyPoints, true, 
                    RobustEstimatorMethod.LMedS);
            fail("IllegalArugmentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
                                
        //test with listener and points
        estimator = MetricTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, true,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACMetricTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, true,
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSMetricTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, true,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACMetricTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, true,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACMetricTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, true,
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
                        
        //test with points, quality scores and method
        qualityScores = new double[
                MetricTransformation3DRobustEstimator.WEAK_MINIMUM_SIZE];
        
        estimator = MetricTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores, true,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACMetricTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores, true,
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSMetricTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores, true,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACMetricTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores, true,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACMetricTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores, true,
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = MetricTransformation3DRobustEstimator.create(
                    emptyPoints, outputPoints, qualityScores, true,
                    RobustEstimatorMethod.PROMedS);            
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = MetricTransformation3DRobustEstimator.create(
                    inputPoints, emptyPoints, qualityScores, true,
                    RobustEstimatorMethod.PROMedS);            
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = MetricTransformation3DRobustEstimator.create(
                    inputPoints, outputPoints, wrongQualityScores, true, 
                    RobustEstimatorMethod.PROMedS);            
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with listener, points, quality scores and method
        estimator = MetricTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores, true,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACMetricTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores, true,
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSMetricTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores, true,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACMetricTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores, true,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACMetricTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = MetricTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores, true,
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = MetricTransformation3DRobustEstimator.create(
                    listener, emptyPoints, outputPoints, qualityScores, true,
                    RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = MetricTransformation3DRobustEstimator.create(
                    listener, inputPoints, emptyPoints, qualityScores, true,
                    RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = MetricTransformation3DRobustEstimator.create(
                    listener, inputPoints, outputPoints, wrongQualityScores, 
                    true, RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with points
        estimator = MetricTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, true);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation3DRobustEstimator);        
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = MetricTransformation3DRobustEstimator.create(
                    emptyPoints, outputPoints, true);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = MetricTransformation3DRobustEstimator.create(
                    inputPoints, emptyPoints, true);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}        
        assertNull(estimator);
        
        //test with listener and points
        estimator = MetricTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, true);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation3DRobustEstimator);        
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = MetricTransformation3DRobustEstimator.create(
                    listener, emptyPoints, outputPoints, true);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = MetricTransformation3DRobustEstimator.create(
                    listener, inputPoints, emptyPoints, true);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with points and quality scores
        estimator = MetricTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores, true);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation3DRobustEstimator);        
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //test with listener, points and quality scores
        estimator = MetricTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores, true);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation3DRobustEstimator);        
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
    }            
}