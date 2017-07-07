/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.estimators.ProjectiveTransformation3DRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date May 4, 2017.
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

public class ProjectiveTransformation3DRobustEstimatorTest {
    
    public ProjectiveTransformation3DRobustEstimatorTest() { }
    
    @BeforeClass
    public static void setUpClass() { }
    
    @AfterClass
    public static void tearDownClass() { }
    
    @Before
    public void setUp() { }
    
    @After
    public void tearDown() { }

    @Test
    public void testCreateFromPoints(){
        ProjectiveTransformation3DRobustEstimator estimator;
                
        //create with points and method
        List<Point3D> inputPoints = new ArrayList<Point3D>();
        List<Point3D> outputPoints = new ArrayList<Point3D>();
        for(int i = 0; i < ProjectiveTransformation3DRobustEstimator.MINIMUM_SIZE; i++){
            inputPoints.add(Point3D.create());
            outputPoints.add(Point3D.create());
        }
        
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSPointCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                MSACPointCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACPointCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSPointCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        List<Point3D> emptyPoints = new ArrayList<Point3D>();
        
        estimator = null;
        try{
            estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                    emptyPoints, outputPoints, RobustEstimatorMethod.LMedS);
            fail("IllegalArugmentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                    inputPoints, emptyPoints, RobustEstimatorMethod.LMedS);
            fail("IllegalArugmentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
                                
        //test with listener and points
        ProjectiveTransformation3DRobustEstimatorListener listener = 
                new ProjectiveTransformation3DRobustEstimatorListener() {

            @Override
            public void onEstimateStart(ProjectiveTransformation3DRobustEstimator estimator) {}

            @Override
            public void onEstimateEnd(ProjectiveTransformation3DRobustEstimator estimator) {}

            @Override
            public void onEstimateNextIteration(ProjectiveTransformation3DRobustEstimator estimator, int iteration) {}

            @Override
            public void onEstimateProgressChange(ProjectiveTransformation3DRobustEstimator estimator, float progress) {}
        };
        
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints, 
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints, 
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSPointCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints, 
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPointCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints, 
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPointCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints, 
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
                        
        //test with points, quality scores and method
        double[] qualityScores = new double[
                ProjectiveTransformation3DRobustEstimator.MINIMUM_SIZE];
        double[] wrongQualityScores = new double[1];
        
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, qualityScores, 
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, qualityScores, 
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSPointCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, qualityScores, 
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPointCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, qualityScores, 
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPointCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, qualityScores, 
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                    emptyPoints, outputPoints, qualityScores, 
                    RobustEstimatorMethod.PROMedS);            
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                    inputPoints, emptyPoints, qualityScores, 
                    RobustEstimatorMethod.PROMedS);            
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                    inputPoints, outputPoints, wrongQualityScores, 
                    RobustEstimatorMethod.PROMedS);            
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with listener, points, quality scores and method
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSPointCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPointCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPointCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                    listener, emptyPoints, outputPoints, qualityScores,
                    RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                    listener, inputPoints, emptyPoints, qualityScores,
                    RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                    listener, inputPoints, outputPoints, wrongQualityScores,
                    RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with points
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                inputPoints, outputPoints);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceProjectiveTransformation3DRobustEstimator);        
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                    emptyPoints, outputPoints);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                    inputPoints, emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}        
        assertNull(estimator);
        
        //test with listener and points
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceProjectiveTransformation3DRobustEstimator);        
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                    listener, emptyPoints, outputPoints);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                    listener, inputPoints, emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with points and quality scores
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, qualityScores);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceProjectiveTransformation3DRobustEstimator);        
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //test with listener, points and quality scores
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints, qualityScores);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceProjectiveTransformation3DRobustEstimator);        
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());                
    }
    
    @Test
    public void testFromPlanes(){
        ProjectiveTransformation3DRobustEstimator estimator;
        
        //create with planes and method
        List<Plane> inputPlanes = new ArrayList<Plane>();
        List<Plane> outputPlanes = new ArrayList<Plane>();
        for(int i = 0; i < ProjectiveTransformation3DRobustEstimator.MINIMUM_SIZE; i++){
            inputPlanes.add(new Plane());
            outputPlanes.add(new Plane());
        }
        
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                inputPlanes, outputPlanes, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                inputPlanes, outputPlanes, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                inputPlanes, outputPlanes, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                inputPlanes, outputPlanes, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                inputPlanes, outputPlanes, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        List<Plane> emptyPlanes = new ArrayList<Plane>();
        
        estimator = null;
        try{
            estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                    emptyPlanes, outputPlanes, RobustEstimatorMethod.LMedS);
            fail("IllegalArugmentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                    inputPlanes, emptyPlanes, RobustEstimatorMethod.LMedS);
            fail("IllegalArugmentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
                        
        //test with listener and points
        ProjectiveTransformation3DRobustEstimatorListener listener = 
                new ProjectiveTransformation3DRobustEstimatorListener() {

            @Override
            public void onEstimateStart(ProjectiveTransformation3DRobustEstimator estimator) {}

            @Override
            public void onEstimateEnd(ProjectiveTransformation3DRobustEstimator estimator) {}

            @Override
            public void onEstimateNextIteration(ProjectiveTransformation3DRobustEstimator estimator, int iteration) {}

            @Override
            public void onEstimateProgressChange(ProjectiveTransformation3DRobustEstimator estimator, float progress) {}
        };
        
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                listener, inputPlanes, outputPlanes, 
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                listener, inputPlanes, outputPlanes, 
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                listener, inputPlanes, outputPlanes, 
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                listener, inputPlanes, outputPlanes, 
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                listener, inputPlanes, outputPlanes, 
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //test with points, quality scores and method
        double[] qualityScores = new double[
                PointCorrespondenceProjectiveTransformation3DRobustEstimator.MINIMUM_SIZE];
        double[] wrongQualityScores = new double[1];
        
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                inputPlanes, outputPlanes, qualityScores, 
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                inputPlanes, outputPlanes, qualityScores, 
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                inputPlanes, outputPlanes, qualityScores, 
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                inputPlanes, outputPlanes, qualityScores, 
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                inputPlanes, outputPlanes, qualityScores, 
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                    emptyPlanes, outputPlanes, qualityScores, 
                    RobustEstimatorMethod.PROMedS);            
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                    inputPlanes, emptyPlanes, qualityScores, 
                    RobustEstimatorMethod.PROMedS);            
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                    inputPlanes, outputPlanes, wrongQualityScores, 
                    RobustEstimatorMethod.PROMedS);            
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with listener, planes, quality scores and method
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                listener, inputPlanes, outputPlanes, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                listener, inputPlanes, outputPlanes, qualityScores,
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                listener, inputPlanes, outputPlanes, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                listener, inputPlanes, outputPlanes, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                listener, inputPlanes, outputPlanes, qualityScores,
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                    listener, emptyPlanes, outputPlanes, qualityScores,
                    RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                    listener, inputPlanes, emptyPlanes, qualityScores,
                    RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                    listener, inputPlanes, outputPlanes, wrongQualityScores,
                    RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with points
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                inputPlanes, outputPlanes);
        assertTrue(estimator instanceof
                PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);        
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                    emptyPlanes, outputPlanes);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                    inputPlanes, emptyPlanes);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}        
        assertNull(estimator);
        
        //test with listener and points
        estimator = ProjectiveTransformation3DRobustEstimator.
                createFromPlanes(listener, inputPlanes, outputPlanes);
        assertTrue(estimator instanceof
                PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);        
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                    listener, emptyPlanes, outputPlanes);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                    listener, inputPlanes, emptyPlanes);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with points and quality scores
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                inputPlanes, outputPlanes, qualityScores);
        assertTrue(estimator instanceof
                PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);        
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //test with listener, points and quality scores
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                listener, inputPlanes, outputPlanes, qualityScores);
        assertTrue(estimator instanceof
                PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);        
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());        
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());                
    }        
}
