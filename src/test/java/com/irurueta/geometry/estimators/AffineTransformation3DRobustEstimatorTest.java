/**
 * @file
 * This file contains unit tests for com.irurueta.geometry.estimators.AffineTransformation3DRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date February 15, 2015
 */
package com.irurueta.geometry.estimators;

import com.irurueta.geometry.Plane;
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

public class AffineTransformation3DRobustEstimatorTest {
    
    public AffineTransformation3DRobustEstimatorTest() {}
    
    @BeforeClass
    public static void setUpClass() {}
    
    @AfterClass
    public static void tearDownClass() {}
    
    @Before
    public void setUp() {}
    
    @After
    public void tearDown() {}

    @Test
    public void testCreateFromPoints(){
        AffineTransformation3DRobustEstimator estimator;
                
        //create with points and method
        List<Point3D> inputPoints = new ArrayList<Point3D>();
        List<Point3D> outputPoints = new ArrayList<Point3D>();
        for(int i = 0; i < AffineTransformation3DRobustEstimator.MINIMUM_SIZE; i++){
            inputPoints.add(Point3D.create());
            outputPoints.add(Point3D.create());
        }
        
        estimator = AffineTransformation3DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = AffineTransformation3DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        

        estimator = AffineTransformation3DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                MSACPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = AffineTransformation3DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = AffineTransformation3DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        List<Point3D> emptyPoints = new ArrayList<Point3D>();
        
        estimator = null;
        try{
            estimator = AffineTransformation3DRobustEstimator.createFromPoints(
                    emptyPoints, outputPoints, RobustEstimatorMethod.LMedS);
            fail("IllegalArugmentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = AffineTransformation3DRobustEstimator.createFromPoints(
                    inputPoints, emptyPoints, RobustEstimatorMethod.LMedS);
            fail("IllegalArugmentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
                                
        //test with listener and points
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
        
        estimator = AffineTransformation3DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints, 
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = AffineTransformation3DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints, 
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = AffineTransformation3DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints, 
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = AffineTransformation3DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints, 
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = AffineTransformation3DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints, 
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
                        
        //test with points, quality scores and method
        double[] qualityScores = new double[
                AffineTransformation3DRobustEstimator.MINIMUM_SIZE];
        double[] wrongQualityScores = new double[1];
        
        estimator = AffineTransformation3DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, qualityScores, 
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = AffineTransformation3DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, qualityScores, 
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = AffineTransformation3DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, qualityScores, 
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = AffineTransformation3DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, qualityScores, 
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = AffineTransformation3DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, qualityScores, 
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = AffineTransformation3DRobustEstimator.createFromPoints(
                    emptyPoints, outputPoints, qualityScores, 
                    RobustEstimatorMethod.PROMedS);            
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = AffineTransformation3DRobustEstimator.createFromPoints(
                    inputPoints, emptyPoints, qualityScores, 
                    RobustEstimatorMethod.PROMedS);            
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = AffineTransformation3DRobustEstimator.createFromPoints(
                    inputPoints, outputPoints, wrongQualityScores, 
                    RobustEstimatorMethod.PROMedS);            
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with listener, points, quality scores and method
        estimator = AffineTransformation3DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = AffineTransformation3DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = AffineTransformation3DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = AffineTransformation3DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = AffineTransformation3DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = AffineTransformation3DRobustEstimator.createFromPoints(
                    listener, emptyPoints, outputPoints, qualityScores,
                    RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = AffineTransformation3DRobustEstimator.createFromPoints(
                    listener, inputPoints, emptyPoints, qualityScores,
                    RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = AffineTransformation3DRobustEstimator.createFromPoints(
                    listener, inputPoints, outputPoints, wrongQualityScores,
                    RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with points
        estimator = AffineTransformation3DRobustEstimator.createFromPoints(
                inputPoints, outputPoints);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceAffineTransformation3DRobustEstimator);        
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = AffineTransformation3DRobustEstimator.createFromPoints(
                    emptyPoints, outputPoints);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = AffineTransformation3DRobustEstimator.createFromPoints(
                    inputPoints, emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}        
        assertNull(estimator);
        
        //test with listener and points
        estimator = AffineTransformation3DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceAffineTransformation3DRobustEstimator);        
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = AffineTransformation3DRobustEstimator.createFromPoints(
                    listener, emptyPoints, outputPoints);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = AffineTransformation3DRobustEstimator.createFromPoints(
                    listener, inputPoints, emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with points and quality scores
        estimator = AffineTransformation3DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, qualityScores);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceAffineTransformation3DRobustEstimator);        
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //test with listener, points and quality scores
        estimator = AffineTransformation3DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints, qualityScores);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceAffineTransformation3DRobustEstimator);        
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());                
    }
    
    @Test
    public void testFromPlanes(){
        AffineTransformation3DRobustEstimator estimator;
        
        //create with planes and method
        List<Plane> inputPlanes = new ArrayList<Plane>();
        List<Plane> outputPlanes = new ArrayList<Plane>();
        for(int i = 0; i < AffineTransformation3DRobustEstimator.MINIMUM_SIZE; i++){
            inputPlanes.add(new Plane());
            outputPlanes.add(new Plane());
        }
        
        estimator = AffineTransformation3DRobustEstimator.createFromPlanes(
                inputPlanes, outputPlanes, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = AffineTransformation3DRobustEstimator.createFromPlanes(
                inputPlanes, outputPlanes, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        

        estimator = AffineTransformation3DRobustEstimator.createFromPlanes(
                inputPlanes, outputPlanes, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                MSACPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = AffineTransformation3DRobustEstimator.createFromPlanes(
                inputPlanes, outputPlanes, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = AffineTransformation3DRobustEstimator.createFromPlanes(
                inputPlanes, outputPlanes, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        List<Plane> emptyPlanes = new ArrayList<Plane>();
        
        estimator = null;
        try{
            estimator = AffineTransformation3DRobustEstimator.createFromPlanes(
                    emptyPlanes, outputPlanes, RobustEstimatorMethod.LMedS);
            fail("IllegalArugmentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = AffineTransformation3DRobustEstimator.createFromPlanes(
                    inputPlanes, emptyPlanes, RobustEstimatorMethod.LMedS);
            fail("IllegalArugmentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
                        
        //test with listener and points
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
        
        estimator = AffineTransformation3DRobustEstimator.createFromPlanes(
                listener, inputPlanes, outputPlanes, 
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = AffineTransformation3DRobustEstimator.createFromPlanes(
                listener, inputPlanes, outputPlanes, 
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = AffineTransformation3DRobustEstimator.createFromPlanes(
                listener, inputPlanes, outputPlanes, 
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = AffineTransformation3DRobustEstimator.createFromPlanes(
                listener, inputPlanes, outputPlanes, 
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = AffineTransformation3DRobustEstimator.createFromPlanes(
                listener, inputPlanes, outputPlanes, 
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //test with points, quality scores and method
        double[] qualityScores = new double[
                PointCorrespondenceAffineTransformation3DRobustEstimator.MINIMUM_SIZE];
        double[] wrongQualityScores = new double[1];
        
        estimator = AffineTransformation3DRobustEstimator.createFromPlanes(
                inputPlanes, outputPlanes, qualityScores, 
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = AffineTransformation3DRobustEstimator.createFromPlanes(
                inputPlanes, outputPlanes, qualityScores, 
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = AffineTransformation3DRobustEstimator.createFromPlanes(
                inputPlanes, outputPlanes, qualityScores, 
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = AffineTransformation3DRobustEstimator.createFromPlanes(
                inputPlanes, outputPlanes, qualityScores, 
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = AffineTransformation3DRobustEstimator.createFromPlanes(
                inputPlanes, outputPlanes, qualityScores, 
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = AffineTransformation3DRobustEstimator.createFromPlanes(
                    emptyPlanes, outputPlanes, qualityScores, 
                    RobustEstimatorMethod.PROMedS);            
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = AffineTransformation3DRobustEstimator.createFromPlanes(
                    inputPlanes, emptyPlanes, qualityScores, 
                    RobustEstimatorMethod.PROMedS);            
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = AffineTransformation3DRobustEstimator.createFromPlanes(
                    inputPlanes, outputPlanes, wrongQualityScores, 
                    RobustEstimatorMethod.PROMedS);            
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with listener, planes, quality scores and method
        estimator = AffineTransformation3DRobustEstimator.createFromPlanes(
                listener, inputPlanes, outputPlanes, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = AffineTransformation3DRobustEstimator.createFromPlanes(
                listener, inputPlanes, outputPlanes, qualityScores,
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = AffineTransformation3DRobustEstimator.createFromPlanes(
                listener, inputPlanes, outputPlanes, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = AffineTransformation3DRobustEstimator.createFromPlanes(
                listener, inputPlanes, outputPlanes, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = AffineTransformation3DRobustEstimator.createFromPlanes(
                listener, inputPlanes, outputPlanes, qualityScores,
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = AffineTransformation3DRobustEstimator.createFromPlanes(
                    listener, emptyPlanes, outputPlanes, qualityScores,
                    RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = AffineTransformation3DRobustEstimator.createFromPlanes(
                    listener, inputPlanes, emptyPlanes, qualityScores,
                    RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = AffineTransformation3DRobustEstimator.createFromPlanes(
                    listener, inputPlanes, outputPlanes, wrongQualityScores,
                    RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with points
        estimator = AffineTransformation3DRobustEstimator.createFromPlanes(
                inputPlanes, outputPlanes);
        assertTrue(estimator instanceof
                PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator);        
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = AffineTransformation3DRobustEstimator.createFromPlanes(
                    emptyPlanes, outputPlanes);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = AffineTransformation3DRobustEstimator.createFromPlanes(
                    inputPlanes, emptyPlanes);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}        
        assertNull(estimator);
        
        //test with listener and points
        estimator = AffineTransformation3DRobustEstimator.
                createFromPlanes(listener, inputPlanes, outputPlanes);
        assertTrue(estimator instanceof
                PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator);        
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = AffineTransformation3DRobustEstimator.createFromPlanes(
                    listener, emptyPlanes, outputPlanes);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = AffineTransformation3DRobustEstimator.createFromPlanes(
                    listener, inputPlanes, emptyPlanes);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with points and quality scores
        estimator = AffineTransformation3DRobustEstimator.createFromPlanes(
                inputPlanes, outputPlanes, qualityScores);
        assertTrue(estimator instanceof
                PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator);        
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //test with listener, points and quality scores
        estimator = AffineTransformation3DRobustEstimator.createFromPlanes(
                listener, inputPlanes, outputPlanes, qualityScores);
        assertTrue(estimator instanceof
                PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator);        
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());        
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());                
    }    
}
