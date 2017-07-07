/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.epipolar.estimators.FundamentalMatrixEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 14, 2015
 */
package com.irurueta.geometry.epipolar.estimators;

import com.irurueta.geometry.Point2D;
import java.util.ArrayList;
import java.util.List;
import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class FundamentalMatrixEstimatorTest {
    
    public FundamentalMatrixEstimatorTest() { }
    
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
        //test create with method
        
        //7 points
        FundamentalMatrixEstimator estimator = FundamentalMatrixEstimator.
                create(FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM);
        
        //check correctness
        assertTrue(estimator instanceof SevenPointsFundamentalMatrixEstimator);
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMethod(), 
                FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM);
        
        //8 points
        estimator = FundamentalMatrixEstimator.create(
                FundamentalMatrixEstimatorMethod.EIGHT_POINTS_ALGORITHM);
        
        //check correctness
        assertTrue(estimator instanceof EightPointsFundamentalMatrixEstimator);
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMethod(), 
                FundamentalMatrixEstimatorMethod.EIGHT_POINTS_ALGORITHM);        
        
        //affine
        estimator = FundamentalMatrixEstimator.create(
                FundamentalMatrixEstimatorMethod.AFFINE_ALGORITHM);
        
        //check correctness
        assertTrue(estimator instanceof AffineFundamentalMatrixEstimator);
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMethod(), 
                FundamentalMatrixEstimatorMethod.AFFINE_ALGORITHM);        
        
        //test create with points and method
        List<Point2D> leftPoints = new ArrayList<Point2D>();
        List<Point2D> rightPoints = new ArrayList<Point2D>();
        for (int i = 0; i < 8; i++) {
            leftPoints.add(Point2D.create());
            rightPoints.add(Point2D.create());
        }
        
        //7 points
        estimator = FundamentalMatrixEstimator.create(leftPoints, rightPoints,
                FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM);
        
        //check correctness
        assertTrue(estimator instanceof SevenPointsFundamentalMatrixEstimator);
        assertSame(estimator.getLeftPoints(), leftPoints);
        assertSame(estimator.getRightPoints(), rightPoints);
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertEquals(estimator.getMethod(),
                FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM);
        
        //8 points
        estimator = FundamentalMatrixEstimator.create(leftPoints, rightPoints, 
                FundamentalMatrixEstimatorMethod.EIGHT_POINTS_ALGORITHM);
        
        //check correctness
        assertTrue(estimator instanceof EightPointsFundamentalMatrixEstimator);
        assertSame(estimator.getLeftPoints(), leftPoints);
        assertSame(estimator.getRightPoints(), rightPoints);
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertEquals(estimator.getMethod(),
                FundamentalMatrixEstimatorMethod.EIGHT_POINTS_ALGORITHM);        
        
        //affine
        estimator = FundamentalMatrixEstimator.create(leftPoints, rightPoints, 
                FundamentalMatrixEstimatorMethod.AFFINE_ALGORITHM);
        
        //check correctness
        assertTrue(estimator instanceof AffineFundamentalMatrixEstimator);
        assertSame(estimator.getLeftPoints(), leftPoints);
        assertSame(estimator.getRightPoints(), rightPoints);
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertEquals(estimator.getMethod(),
                FundamentalMatrixEstimatorMethod.AFFINE_ALGORITHM);        
        
        
        //test create with default method
        estimator = FundamentalMatrixEstimator.create();
        
        assertTrue(estimator instanceof SevenPointsFundamentalMatrixEstimator);
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMethod(), 
                FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM);
        
        //test create with points and default method
        estimator = FundamentalMatrixEstimator.create(leftPoints, rightPoints);
        
        //check correctness
        assertTrue(estimator instanceof SevenPointsFundamentalMatrixEstimator);
        assertSame(estimator.getLeftPoints(), leftPoints);
        assertSame(estimator.getRightPoints(), rightPoints);
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertEquals(estimator.getMethod(),
                FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM);        
    }
}
