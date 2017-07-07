/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.estimators.LineCorrespondenceAffineTransformation2DRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date February 13, 2015
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

public class LineCorrespondenceAffineTransformation2DRobustEstimatorTest {
    
    public LineCorrespondenceAffineTransformation2DRobustEstimatorTest() {}
    
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
        LineCorrespondenceAffineTransformation2DRobustEstimator estimator;
        
        //create with robust estimator method
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());  
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //create with lines and method
        List<Line2D> inputLines = new ArrayList<Line2D>();
        List<Line2D> outputLines = new ArrayList<Line2D>();
        for(int i = 0; i < LineCorrespondenceAffineTransformation2DRobustEstimator.MINIMUM_SIZE; i++){
            inputLines.add(new Line2D());
            outputLines.add(new Line2D());
        }
        
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(inputLines, outputLines, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(estimator.getInputLines(), inputLines);
        assertSame(estimator.getOutputLines(), outputLines);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(inputLines, outputLines, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(estimator.getInputLines(), inputLines);
        assertSame(estimator.getOutputLines(), outputLines);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        

        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(inputLines, outputLines, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                MSACLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(estimator.getInputLines(), inputLines);
        assertSame(estimator.getOutputLines(), outputLines);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(inputLines, outputLines, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(estimator.getInputLines(), inputLines);
        assertSame(estimator.getOutputLines(), outputLines);
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(inputLines, outputLines, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(estimator.getInputLines(), inputLines);
        assertSame(estimator.getOutputLines(), outputLines);
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        List<Line2D> emptyLines = new ArrayList<Line2D>();
        
        estimator = null;
        try{
            estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                    create(emptyLines, outputLines, 
                    RobustEstimatorMethod.LMedS);
            fail("IllegalArugmentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                    create(inputLines, emptyLines, 
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
        
        
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        

        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                MSACLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //test with listener and points
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, inputLines, outputLines, 
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(estimator.getInputLines(), inputLines);
        assertSame(estimator.getOutputLines(), outputLines);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, inputLines, outputLines, 
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(estimator.getInputLines(), inputLines);
        assertSame(estimator.getOutputLines(), outputLines);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, inputLines, outputLines, 
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(estimator.getInputLines(), inputLines);
        assertSame(estimator.getOutputLines(), outputLines);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, inputLines, outputLines, 
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(estimator.getInputLines(), inputLines);
        assertSame(estimator.getOutputLines(), outputLines);
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, inputLines, outputLines, 
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(estimator.getInputLines(), inputLines);
        assertSame(estimator.getOutputLines(), outputLines);
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
                LineCorrespondenceAffineTransformation2DRobustEstimator.MINIMUM_SIZE];
        double[] wrongQualityScores = new double[1];
        
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(qualityScores, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(qualityScores, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        

        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(qualityScores, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(qualityScores, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(qualityScores, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //test with points, quality scores and method
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(inputLines, outputLines, qualityScores, 
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(estimator.getInputLines(), inputLines);
        assertSame(estimator.getOutputLines(), outputLines);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(inputLines, outputLines, qualityScores, 
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(estimator.getInputLines(), inputLines);
        assertSame(estimator.getOutputLines(), outputLines);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(inputLines, outputLines, qualityScores, 
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(estimator.getInputLines(), inputLines);
        assertSame(estimator.getOutputLines(), outputLines);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(inputLines, outputLines, qualityScores, 
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(estimator.getInputLines(), inputLines);
        assertSame(estimator.getOutputLines(), outputLines);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(inputLines, outputLines, qualityScores, 
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(estimator.getInputLines(), inputLines);
        assertSame(estimator.getOutputLines(), outputLines);
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
            estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(emptyLines, outputLines, qualityScores, 
                RobustEstimatorMethod.PROMedS);            
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(inputLines, emptyLines, qualityScores, 
                RobustEstimatorMethod.PROMedS);            
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(inputLines, outputLines, wrongQualityScores, 
                RobustEstimatorMethod.PROMedS);            
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with listener, quality scores and method
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, qualityScores, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, qualityScores, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, qualityScores, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, qualityScores, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, qualityScores, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //test with listener, points, quality scores and method
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, inputLines, outputLines, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(estimator.getInputLines(), inputLines);
        assertSame(estimator.getOutputLines(), outputLines);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, inputLines, outputLines, qualityScores,
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(estimator.getInputLines(), inputLines);
        assertSame(estimator.getOutputLines(), outputLines);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, inputLines, outputLines, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(estimator.getInputLines(), inputLines);
        assertSame(estimator.getOutputLines(), outputLines);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, inputLines, outputLines, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(estimator.getInputLines(), inputLines);
        assertSame(estimator.getOutputLines(), outputLines);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, inputLines, outputLines, qualityScores,
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(estimator.getInputLines(), inputLines);
        assertSame(estimator.getOutputLines(), outputLines);
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
            estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, emptyLines, outputLines, qualityScores,
                RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, inputLines, emptyLines, qualityScores,
                RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, inputLines, outputLines, wrongQualityScores,
                RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test no arguments
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create();
        assertTrue(estimator instanceof
                PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //test with points
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(inputLines, outputLines);
        assertTrue(estimator instanceof
                PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator);        
        assertSame(estimator.getInputLines(), inputLines);
        assertSame(estimator.getOutputLines(), outputLines);
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
            estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(emptyLines, outputLines);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(inputLines, emptyLines);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}        
        assertNull(estimator);
        
        //test with listener
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener);
        assertTrue(estimator instanceof
                PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator);        
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //test with listener and points
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, inputLines, outputLines);
        assertTrue(estimator instanceof
                PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator);        
        assertSame(estimator.getInputLines(), inputLines);
        assertSame(estimator.getOutputLines(), outputLines);
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
            estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, emptyLines, outputLines);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, inputLines, emptyLines);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with quality scores
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(qualityScores);
        assertTrue(estimator instanceof
                PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator);        
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //test with points and quality scores
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(inputLines, outputLines, qualityScores);
        assertTrue(estimator instanceof
                PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator);        
        assertSame(estimator.getInputLines(), inputLines);
        assertSame(estimator.getOutputLines(), outputLines);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //test with listener and quality scores
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, qualityScores);
        assertTrue(estimator instanceof
                PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator);        
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //test with listener, points and quality scores
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.
                create(listener, inputLines, outputLines, qualityScores);
        assertTrue(estimator instanceof
                PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator);        
        assertSame(estimator.getInputLines(), inputLines);
        assertSame(estimator.getOutputLines(), outputLines);
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
