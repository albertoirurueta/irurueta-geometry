/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.estimators.PlaneCorrespondenceAffineTransformation3DRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date February 15, 2015
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

public class PlaneCorrespondenceAffineTransformation3DRobustEstimatorTest {
    
    public PlaneCorrespondenceAffineTransformation3DRobustEstimatorTest() {}
    
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
        PlaneCorrespondenceAffineTransformation3DRobustEstimator estimator;
        
        //create with robust estimator method
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());  
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //create with planes and method
        List<Plane> inputPlanes = new ArrayList<Plane>();
        List<Plane> outputPlanes = new ArrayList<Plane>();
        for(int i = 0; i < PlaneCorrespondenceAffineTransformation3DRobustEstimator.MINIMUM_SIZE; i++){
            inputPlanes.add(new Plane());
            outputPlanes.add(new Plane());
        }
        
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(inputPlanes, outputPlanes, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(inputPlanes, outputPlanes, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        

        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(inputPlanes, outputPlanes, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                MSACPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(inputPlanes, outputPlanes, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(inputPlanes, outputPlanes, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        List<Plane> emptyPlanes = new ArrayList<Plane>();
        
        estimator = null;
        try{
            estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                    create(emptyPlanes, outputPlanes, 
                    RobustEstimatorMethod.LMedS);
            fail("IllegalArugmentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                    create(inputPlanes, emptyPlanes, 
                    RobustEstimatorMethod.LMedS);
            fail("IllegalArugmentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //create with listener and method
        AffineTransformation3DRobustEstimatorListener listener = 
                new AffineTransformation3DRobustEstimatorListener() {

            @Override
            public void onEstimateStart(AffineTransformation3DRobustEstimator estimator) {}

            @Override
            public void onEstimateEnd(AffineTransformation3DRobustEstimator estimator) {}

            @Override
            public void onEstimateNextIteration(AffineTransformation3DRobustEstimator estimator, int iteration) {}

            @Override
            public void onEstimateProgressChange(AffineTransformation3DRobustEstimator estimator, float progress) {}
        };
        
        
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(listener, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        

        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(listener, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(listener, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                MSACPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(listener, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(listener, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //test with listener and points
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(listener, inputPlanes, outputPlanes, 
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(listener, inputPlanes, outputPlanes, 
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(listener, inputPlanes, outputPlanes, 
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(listener, inputPlanes, outputPlanes, 
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(listener, inputPlanes, outputPlanes, 
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
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
                PlaneCorrespondenceAffineTransformation3DRobustEstimator.MINIMUM_SIZE];
        double[] wrongQualityScores = new double[1];
        
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(qualityScores, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(qualityScores, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        

        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(qualityScores, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(qualityScores, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(qualityScores, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //test with points, quality scores and method
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(inputPlanes, outputPlanes, qualityScores, 
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(inputPlanes, outputPlanes, qualityScores, 
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(inputPlanes, outputPlanes, qualityScores, 
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(inputPlanes, outputPlanes, qualityScores, 
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(inputPlanes, outputPlanes, qualityScores, 
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
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
            estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(emptyPlanes, outputPlanes, qualityScores, 
                RobustEstimatorMethod.PROMedS);            
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(inputPlanes, emptyPlanes, qualityScores, 
                RobustEstimatorMethod.PROMedS);            
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(inputPlanes, outputPlanes, wrongQualityScores, 
                RobustEstimatorMethod.PROMedS);            
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with listener, quality scores and method
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(listener, qualityScores, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(listener, qualityScores, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(listener, qualityScores, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(listener, qualityScores, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(listener, qualityScores, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //test with listener, points, quality scores and method
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(listener, inputPlanes, outputPlanes, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(listener, inputPlanes, outputPlanes, qualityScores,
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(listener, inputPlanes, outputPlanes, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(listener, inputPlanes, outputPlanes, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(listener, inputPlanes, outputPlanes, qualityScores,
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
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
            estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(listener, emptyPlanes, outputPlanes, qualityScores,
                RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(listener, inputPlanes, emptyPlanes, qualityScores,
                RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(listener, inputPlanes, outputPlanes, wrongQualityScores,
                RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test no arguments
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create();
        assertTrue(estimator instanceof
                PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //test with points
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(inputPlanes, outputPlanes);
        assertTrue(estimator instanceof
                PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator);        
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
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
            estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(emptyPlanes, outputPlanes);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(inputPlanes, emptyPlanes);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}        
        assertNull(estimator);
        
        //test with listener
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(listener);
        assertTrue(estimator instanceof
                PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator);        
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //test with listener and points
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(listener, inputPlanes, outputPlanes);
        assertTrue(estimator instanceof
                PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator);        
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
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
            estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(listener, emptyPlanes, outputPlanes);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(listener, inputPlanes, emptyPlanes);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with quality scores
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(qualityScores);
        assertTrue(estimator instanceof
                PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator);        
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //test with planes and quality scores
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(inputPlanes, outputPlanes, qualityScores);
        assertTrue(estimator instanceof
                PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator);        
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //test with listener and quality scores
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(listener, qualityScores);
        assertTrue(estimator instanceof
                PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator);        
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //test with listener, points and quality scores
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                create(listener, inputPlanes, outputPlanes, qualityScores);
        assertTrue(estimator instanceof
                PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator);        
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
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
