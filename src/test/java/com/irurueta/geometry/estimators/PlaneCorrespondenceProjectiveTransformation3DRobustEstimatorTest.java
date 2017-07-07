/**
 * @file
 * This file contains unit tests for
 * com.irureuta.geometry.estimators.PlaneCorrespondenceProjectiveTransformation3DRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date March 7, 2015
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

public class PlaneCorrespondenceProjectiveTransformation3DRobustEstimatorTest {
    
    public PlaneCorrespondenceProjectiveTransformation3DRobustEstimatorTest() {}
    
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
        PlaneCorrespondenceProjectiveTransformation3DRobustEstimator estimator;
        
        //create with robust estimator method
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());  
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //create with lines and method
        List<Plane> inputPlanes = new ArrayList<Plane>();
        List<Plane> outputPlanes = new ArrayList<Plane>();
        for(int i = 0; i < PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.MINIMUM_SIZE; i++){
            inputPlanes.add(new Plane());
            outputPlanes.add(new Plane());
        }
        
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(inputPlanes, outputPlanes, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(inputPlanes, outputPlanes, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        

        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(inputPlanes, outputPlanes, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(inputPlanes, outputPlanes, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(inputPlanes, outputPlanes, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        List<Plane> emptyLines = new ArrayList<Plane>();
        
        estimator = null;
        try{
            estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                    create(emptyLines, outputPlanes, 
                    RobustEstimatorMethod.LMedS);
            fail("IllegalArugmentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                    create(inputPlanes, emptyLines, 
                    RobustEstimatorMethod.LMedS);
            fail("IllegalArugmentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //create with listener and method
        ProjectiveTransformation3DRobustEstimatorListener listener = new ProjectiveTransformation3DRobustEstimatorListener() {

            @Override
            public void onEstimateStart(ProjectiveTransformation3DRobustEstimator estimator) {}

            @Override
            public void onEstimateEnd(ProjectiveTransformation3DRobustEstimator estimator) {}

            @Override
            public void onEstimateNextIteration(ProjectiveTransformation3DRobustEstimator estimator, int iteration) {}

            @Override
            public void onEstimateProgressChange(ProjectiveTransformation3DRobustEstimator estimator, float progress) {}
        };
        
        
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(listener, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        

        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(listener, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(listener, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(listener, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(listener, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //test with listener and points
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(listener, inputPlanes, outputPlanes, 
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(listener, inputPlanes, outputPlanes, 
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(listener, inputPlanes, outputPlanes, 
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(listener, inputPlanes, outputPlanes, 
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(listener, inputPlanes, outputPlanes, 
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //test with quality scores and method
        double[] qualityScores = new double[
                PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.MINIMUM_SIZE];
        double[] wrongQualityScores = new double[1];
        
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(qualityScores, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(qualityScores, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        

        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(qualityScores, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(qualityScores, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(qualityScores, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //test with points, quality scores and method
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(inputPlanes, outputPlanes, qualityScores, 
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(inputPlanes, outputPlanes, qualityScores, 
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(inputPlanes, outputPlanes, qualityScores, 
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(inputPlanes, outputPlanes, qualityScores, 
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(inputPlanes, outputPlanes, qualityScores, 
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(emptyLines, outputPlanes, qualityScores, 
                RobustEstimatorMethod.PROMedS);            
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(inputPlanes, emptyLines, qualityScores, 
                RobustEstimatorMethod.PROMedS);            
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(inputPlanes, outputPlanes, wrongQualityScores, 
                RobustEstimatorMethod.PROMedS);            
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with listener, quality scores and method
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(listener, qualityScores, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(listener, qualityScores, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(listener, qualityScores, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(listener, qualityScores, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(listener, qualityScores, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //test with listener, points, quality scores and method
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(listener, inputPlanes, outputPlanes, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(listener, inputPlanes, outputPlanes, qualityScores,
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(listener, inputPlanes, outputPlanes, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(listener, inputPlanes, outputPlanes, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(listener, inputPlanes, outputPlanes, qualityScores,
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(listener, emptyLines, outputPlanes, qualityScores,
                RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(listener, inputPlanes, emptyLines, qualityScores,
                RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(listener, inputPlanes, outputPlanes, wrongQualityScores,
                RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test no arguments
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create();
        assertTrue(estimator instanceof
                PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //test with points
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(inputPlanes, outputPlanes);
        assertTrue(estimator instanceof
                PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);        
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(emptyLines, outputPlanes);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(inputPlanes, emptyLines);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}        
        assertNull(estimator);
        
        //test with listener
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(listener);
        assertTrue(estimator instanceof
                PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);        
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //test with listener and points
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(listener, inputPlanes, outputPlanes);
        assertTrue(estimator instanceof
                PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);        
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(listener, emptyLines, outputPlanes);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(listener, inputPlanes, emptyLines);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with quality scores
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(qualityScores);
        assertTrue(estimator instanceof
                PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);        
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //test with points and quality scores
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(inputPlanes, outputPlanes, qualityScores);
        assertTrue(estimator instanceof
                PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);        
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //test with listener and quality scores
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(listener, qualityScores);
        assertTrue(estimator instanceof
                PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);        
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //test with listener, points and quality scores
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                create(listener, inputPlanes, outputPlanes, qualityScores);
        assertTrue(estimator instanceof
                PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);        
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
    }
}
