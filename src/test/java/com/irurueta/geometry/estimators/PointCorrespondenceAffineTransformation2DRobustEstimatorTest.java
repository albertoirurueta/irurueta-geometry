/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.estimators.PointCorrespondenceAffineTransformation2DRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date February 11, 2015
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

public class PointCorrespondenceAffineTransformation2DRobustEstimatorTest {
    
    public PointCorrespondenceAffineTransformation2DRobustEstimatorTest() {}
    
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
        PointCorrespondenceAffineTransformation2DRobustEstimator estimator;
        
        //create with robust estimator method
        estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());  
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //create with points and method
        List<Point2D> inputPoints = new ArrayList<Point2D>();
        List<Point2D> outputPoints = new ArrayList<Point2D>();
        for(int i = 0; i < PointCorrespondenceAffineTransformation2DRobustEstimator.MINIMUM_SIZE; i++){
            inputPoints.add(Point2D.create());
            outputPoints.add(Point2D.create());
        }
        
        estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(inputPoints, outputPoints, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(inputPoints, outputPoints, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        

        estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(inputPoints, outputPoints, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                MSACPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(inputPoints, outputPoints, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(inputPoints, outputPoints, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        List<Point2D> emptyPoints = new ArrayList<Point2D>();
        
        estimator = null;
        try{
            estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                    create(emptyPoints, outputPoints, 
                    RobustEstimatorMethod.LMedS);
            fail("IllegalArugmentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                    create(inputPoints, emptyPoints, 
                    RobustEstimatorMethod.LMedS);
            fail("IllegalArugmentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //create with listener and method
        AffineTransformation2DRobustEstimatorListener listener = new AffineTransformation2DRobustEstimatorListener() {

            @Override
            public void onEstimateStart(AffineTransformation2DRobustEstimator estimator) {}

            @Override
            public void onEstimateEnd(AffineTransformation2DRobustEstimator estimator) {}

            @Override
            public void onEstimateNextIteration(AffineTransformation2DRobustEstimator estimator, int iteration) {}

            @Override
            public void onEstimateProgressChange(AffineTransformation2DRobustEstimator estimator, float progress) {}
        };
        
        
        estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        

        estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                MSACPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //test with listener and points
        estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, inputPoints, outputPoints, 
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, inputPoints, outputPoints, 
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, inputPoints, outputPoints, 
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, inputPoints, outputPoints, 
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, inputPoints, outputPoints, 
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //test with quality scores and method
        double[] qualityScores = new double[
                PointCorrespondenceAffineTransformation2DRobustEstimator.MINIMUM_SIZE];
        double[] wrongQualityScores = new double[1];
        
        estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(qualityScores, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(qualityScores, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        

        estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(qualityScores, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(qualityScores, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(qualityScores, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //test with points, quality scores and method
        estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(inputPoints, outputPoints, qualityScores, 
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(inputPoints, outputPoints, qualityScores, 
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(inputPoints, outputPoints, qualityScores, 
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(inputPoints, outputPoints, qualityScores, 
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(inputPoints, outputPoints, qualityScores, 
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(emptyPoints, outputPoints, qualityScores, 
                RobustEstimatorMethod.PROMedS);            
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(inputPoints, emptyPoints, qualityScores, 
                RobustEstimatorMethod.PROMedS);            
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(inputPoints, outputPoints, wrongQualityScores, 
                RobustEstimatorMethod.PROMedS);            
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with listener, quality scores and method
        estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, qualityScores, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, qualityScores, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, qualityScores, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, qualityScores, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, qualityScores, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //test with listener, points, quality scores and method
        estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, emptyPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, inputPoints, emptyPoints, qualityScores,
                RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, inputPoints, outputPoints, wrongQualityScores,
                RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test no arguments
        estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create();
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //test with points
        estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(inputPoints, outputPoints);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator);        
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(emptyPoints, outputPoints);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(inputPoints, emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}        
        assertNull(estimator);
        
        //test with listener
        estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator);        
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //test with listener and points
        estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, inputPoints, outputPoints);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator);        
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, emptyPoints, outputPoints);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, inputPoints, emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with quality scores
        estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(qualityScores);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator);        
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //test with points and quality scores
        estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(inputPoints, outputPoints, qualityScores);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator);        
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //test with listener and quality scores
        estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, qualityScores);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator);        
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //test with listener, points and quality scores
        estimator = PointCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, inputPoints, outputPoints, qualityScores);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator);        
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
    }
}