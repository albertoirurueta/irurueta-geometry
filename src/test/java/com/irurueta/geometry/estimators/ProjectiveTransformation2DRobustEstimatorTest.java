/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.estimators.ProjectiveTransformation2DRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date March 4, 2015
 */
package com.irurueta.geometry.estimators;

import com.irurueta.geometry.Line2D;
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

public class ProjectiveTransformation2DRobustEstimatorTest {
    
    public ProjectiveTransformation2DRobustEstimatorTest() {}
    
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
        ProjectiveTransformation2DRobustEstimator estimator;
                
        //create with points and method
        List<Point2D> inputPoints = new ArrayList<Point2D>();
        List<Point2D> outputPoints = new ArrayList<Point2D>();
        for(int i = 0; i < ProjectiveTransformation2DRobustEstimator.MINIMUM_SIZE; i++){
            inputPoints.add(Point2D.create());
            outputPoints.add(Point2D.create());
        }
        
        estimator = ProjectiveTransformation2DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation2DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        

        estimator = ProjectiveTransformation2DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                MSACPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation2DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation2DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        List<Point2D> emptyPoints = new ArrayList<Point2D>();
        
        estimator = null;
        try{
            estimator = ProjectiveTransformation2DRobustEstimator.createFromPoints(
                    emptyPoints, outputPoints, RobustEstimatorMethod.LMedS);
            fail("IllegalArugmentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = ProjectiveTransformation2DRobustEstimator.createFromPoints(
                    inputPoints, emptyPoints, RobustEstimatorMethod.LMedS);
            fail("IllegalArugmentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
                                
        //test with listener and points
        ProjectiveTransformation2DRobustEstimatorListener listener = new ProjectiveTransformation2DRobustEstimatorListener() {

            @Override
            public void onEstimateStart(ProjectiveTransformation2DRobustEstimator estimator) {}

            @Override
            public void onEstimateEnd(ProjectiveTransformation2DRobustEstimator estimator) {}

            @Override
            public void onEstimateNextIteration(ProjectiveTransformation2DRobustEstimator estimator, int iteration) {}

            @Override
            public void onEstimateProgressChange(ProjectiveTransformation2DRobustEstimator estimator, float progress) {}
        };
        
        estimator = ProjectiveTransformation2DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints, 
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation2DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints, 
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation2DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints, 
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation2DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints, 
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation2DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints, 
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
                        
        //test with points, quality scores and method
        double[] qualityScores = new double[
                ProjectiveTransformation2DRobustEstimator.MINIMUM_SIZE];
        double[] wrongQualityScores = new double[1];
        
        estimator = ProjectiveTransformation2DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, qualityScores, 
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation2DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, qualityScores, 
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation2DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, qualityScores, 
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation2DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, qualityScores, 
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation2DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, qualityScores, 
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator);
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
            estimator = ProjectiveTransformation2DRobustEstimator.createFromPoints(
                    emptyPoints, outputPoints, qualityScores, 
                    RobustEstimatorMethod.PROMedS);            
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = ProjectiveTransformation2DRobustEstimator.createFromPoints(
                    inputPoints, emptyPoints, qualityScores, 
                    RobustEstimatorMethod.PROMedS);            
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = ProjectiveTransformation2DRobustEstimator.createFromPoints(
                    inputPoints, outputPoints, wrongQualityScores, 
                    RobustEstimatorMethod.PROMedS);            
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with listener, points, quality scores and method
        estimator = ProjectiveTransformation2DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation2DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation2DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation2DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation2DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator);
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
            estimator = ProjectiveTransformation2DRobustEstimator.createFromPoints(
                    listener, emptyPoints, outputPoints, qualityScores,
                    RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = ProjectiveTransformation2DRobustEstimator.createFromPoints(
                    listener, inputPoints, emptyPoints, qualityScores,
                    RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = ProjectiveTransformation2DRobustEstimator.createFromPoints(
                    listener, inputPoints, outputPoints, wrongQualityScores,
                    RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with points
        estimator = ProjectiveTransformation2DRobustEstimator.createFromPoints(
                inputPoints, outputPoints);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator);        
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
            estimator = ProjectiveTransformation2DRobustEstimator.createFromPoints(
                    emptyPoints, outputPoints);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = ProjectiveTransformation2DRobustEstimator.createFromPoints(
                    inputPoints, emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}        
        assertNull(estimator);
        
        //test with listener and points
        estimator = ProjectiveTransformation2DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator);        
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
            estimator = ProjectiveTransformation2DRobustEstimator.createFromPoints(
                    listener, emptyPoints, outputPoints);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = ProjectiveTransformation2DRobustEstimator.createFromPoints(
                    listener, inputPoints, emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with points and quality scores
        estimator = ProjectiveTransformation2DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, qualityScores);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator);        
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //test with listener, points and quality scores
        estimator = ProjectiveTransformation2DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints, qualityScores);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator);        
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
    }
    
    @Test
    public void testFromLines(){
        ProjectiveTransformation2DRobustEstimator estimator;
        
        //create with lines and method
        List<Line2D> inputLines = new ArrayList<Line2D>();
        List<Line2D> outputLines = new ArrayList<Line2D>();
        for(int i = 0; i < ProjectiveTransformation2DRobustEstimator.MINIMUM_SIZE; i++){
            inputLines.add(new Line2D());
            outputLines.add(new Line2D());
        }
        
        estimator = ProjectiveTransformation2DRobustEstimator.createFromLines(
                inputLines, outputLines, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation2DRobustEstimator.createFromLines(
                inputLines, outputLines, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        

        estimator = ProjectiveTransformation2DRobustEstimator.createFromLines(
                inputLines, outputLines, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                MSACLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation2DRobustEstimator.createFromLines(
                inputLines, outputLines, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation2DRobustEstimator.createFromLines(
                inputLines, outputLines, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        List<Line2D> emptyLines = new ArrayList<Line2D>();
        
        estimator = null;
        try{
            estimator = ProjectiveTransformation2DRobustEstimator.createFromLines(
                    emptyLines, outputLines, RobustEstimatorMethod.LMedS);
            fail("IllegalArugmentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = ProjectiveTransformation2DRobustEstimator.createFromLines(
                    inputLines, emptyLines, RobustEstimatorMethod.LMedS);
            fail("IllegalArugmentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
                        
        //test with listener and points
        ProjectiveTransformation2DRobustEstimatorListener listener = new ProjectiveTransformation2DRobustEstimatorListener() {

            @Override
            public void onEstimateStart(ProjectiveTransformation2DRobustEstimator estimator) {}

            @Override
            public void onEstimateEnd(ProjectiveTransformation2DRobustEstimator estimator) {}

            @Override
            public void onEstimateNextIteration(ProjectiveTransformation2DRobustEstimator estimator, int iteration) {}

            @Override
            public void onEstimateProgressChange(ProjectiveTransformation2DRobustEstimator estimator, float progress) {}
        };
        
        estimator = ProjectiveTransformation2DRobustEstimator.createFromLines(
                listener, inputLines, outputLines, 
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation2DRobustEstimator.createFromLines(
                listener, inputLines, outputLines, 
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation2DRobustEstimator.createFromLines(
                listener, inputLines, outputLines, 
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation2DRobustEstimator.createFromLines(
                listener, inputLines, outputLines, 
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation2DRobustEstimator.createFromLines(
                listener, inputLines, outputLines, 
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //test with points, quality scores and method
        double[] qualityScores = new double[
                PointCorrespondenceProjectiveTransformation2DRobustEstimator.MINIMUM_SIZE];
        double[] wrongQualityScores = new double[1];
        
        estimator = ProjectiveTransformation2DRobustEstimator.createFromLines(
                inputLines, outputLines, qualityScores, 
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation2DRobustEstimator.createFromLines(
                inputLines, outputLines, qualityScores, 
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation2DRobustEstimator.createFromLines(
                inputLines, outputLines, qualityScores, 
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation2DRobustEstimator.createFromLines(
                inputLines, outputLines, qualityScores, 
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation2DRobustEstimator.createFromLines(
                inputLines, outputLines, qualityScores, 
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator);
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
            estimator = ProjectiveTransformation2DRobustEstimator.createFromLines(
                    emptyLines, outputLines, qualityScores, 
                    RobustEstimatorMethod.PROMedS);            
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = ProjectiveTransformation2DRobustEstimator.createFromLines(
                    inputLines, emptyLines, qualityScores, 
                    RobustEstimatorMethod.PROMedS);            
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = ProjectiveTransformation2DRobustEstimator.createFromLines(
                    inputLines, outputLines, wrongQualityScores, 
                    RobustEstimatorMethod.PROMedS);            
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with listener, points, quality scores and method
        estimator = ProjectiveTransformation2DRobustEstimator.createFromLines(
                listener, inputLines, outputLines, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation2DRobustEstimator.createFromLines(
                listener, inputLines, outputLines, qualityScores,
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation2DRobustEstimator.createFromLines(
                listener, inputLines, outputLines, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation2DRobustEstimator.createFromLines(
                listener, inputLines, outputLines, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        estimator = ProjectiveTransformation2DRobustEstimator.createFromLines(
                listener, inputLines, outputLines, qualityScores,
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator);
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
            estimator = ProjectiveTransformation2DRobustEstimator.createFromLines(
                    listener, emptyLines, outputLines, qualityScores,
                    RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = ProjectiveTransformation2DRobustEstimator.createFromLines(
                    listener, inputLines, emptyLines, qualityScores,
                    RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = ProjectiveTransformation2DRobustEstimator.createFromLines(
                    listener, inputLines, outputLines, wrongQualityScores,
                    RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with points
        estimator = ProjectiveTransformation2DRobustEstimator.createFromLines(
                inputLines, outputLines);
        assertTrue(estimator instanceof
                PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator);        
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
            estimator = ProjectiveTransformation2DRobustEstimator.createFromLines(
                    emptyLines, outputLines);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = ProjectiveTransformation2DRobustEstimator.createFromLines(
                    inputLines, emptyLines);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}        
        assertNull(estimator);
        
        //test with listener and points
        estimator = ProjectiveTransformation2DRobustEstimator.
                createFromLines(listener, inputLines, outputLines);
        assertTrue(estimator instanceof
                PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator);        
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
            estimator = ProjectiveTransformation2DRobustEstimator.createFromLines(
                    listener, emptyLines, outputLines);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = ProjectiveTransformation2DRobustEstimator.createFromLines(
                    listener, inputLines, emptyLines);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test with points and quality scores
        estimator = ProjectiveTransformation2DRobustEstimator.createFromLines(
                inputLines, outputLines, qualityScores);
        assertTrue(estimator instanceof
                PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator);        
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //test with listener, points and quality scores
        estimator = ProjectiveTransformation2DRobustEstimator.createFromLines(
                listener, inputLines, outputLines, qualityScores);
        assertTrue(estimator instanceof
                PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator);        
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());        
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
    }
}
