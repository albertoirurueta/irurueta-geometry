/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.estimators.RadialDistortionEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date March 22, 2015
 */
package com.irurueta.geometry.calib3d.estimators;

import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.estimators.LockedException;
import java.util.ArrayList;
import java.util.List;
import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class RadialDistortionEstimatorTest {
    
    public RadialDistortionEstimatorTest() {}
    
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
        RadialDistortionEstimator estimator = 
                RadialDistortionEstimator.create();
        
        //check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getType(), 
                RadialDistortionEstimatorType.LMSE_RADIAL_DISTORTION_ESTIMATOR);
        
        //test with type
        estimator = RadialDistortionEstimator.create(
                RadialDistortionEstimatorType.LMSE_RADIAL_DISTORTION_ESTIMATOR);
        
        //check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());        
        assertFalse(estimator.isReady());
        assertEquals(estimator.getType(), 
                RadialDistortionEstimatorType.LMSE_RADIAL_DISTORTION_ESTIMATOR);        
        
        estimator = RadialDistortionEstimator.create(
                RadialDistortionEstimatorType.WEIGHTED_RADIAL_DISTORTION_ESTIMATOR);
        
        //check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());        
        assertFalse(estimator.isReady());
        assertEquals(estimator.getType(), 
                RadialDistortionEstimatorType.WEIGHTED_RADIAL_DISTORTION_ESTIMATOR);
    }
    
    @Test
    public void testGetSetdistortedUndistortedPoints() throws LockedException{
        RadialDistortionEstimator estimator = 
                RadialDistortionEstimator.create();
        
        //check default values
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        
        //set new value
        List<Point2D> distortedPoints = new ArrayList<Point2D>();
        List<Point2D> undistortedPoints = new ArrayList<Point2D>();
        for(int i = 0; i < estimator.getMinNumberOfMatchedPoints(); i++){
            distortedPoints.add(Point2D.create());
            undistortedPoints.add(Point2D.create());
        }
        
        estimator.setPoints(distortedPoints, undistortedPoints);
        
        //check correctness
        assertSame(estimator.getDistortedPoints(), distortedPoints);
        assertSame(estimator.getUndistortedPoints(), undistortedPoints);
        
        //Force IllegalArgumentException
        List<Point2D> emptyPoints = new ArrayList<Point2D>();
        try{
            estimator.setPoints(emptyPoints, undistortedPoints);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator.setPoints(emptyPoints, emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator.setPoints((List<Point2D>)null, undistortedPoints);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator.setPoints(distortedPoints, (List<Point2D>)null);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}        
    }
    
    @Test
    public void testGetSetDistortionCenter() throws LockedException{
        RadialDistortionEstimator estimator = 
                RadialDistortionEstimator.create();

        //check default value
        assertNull(estimator.getDistortionCenter());
        
        //set new value
        Point2D center = Point2D.create();
        estimator.setDistortionCenter(center);
        
        //check correctness
        assertSame(estimator.getDistortionCenter(), center);
    }
    
    @Test
    public void testAreValidLists(){
        RadialDistortionEstimator estimator = 
                RadialDistortionEstimator.create();
        
        List<Point2D> distortedPoints = new ArrayList<Point2D>();
        List<Point2D> undistortedPoints = new ArrayList<Point2D>();
        for(int i = 0; i < estimator.getMinNumberOfMatchedPoints(); i++){
            distortedPoints.add(Point2D.create());
            undistortedPoints.add(Point2D.create());
        }
        List<Point2D> emptyPoints = new ArrayList<Point2D>();

        assertTrue(estimator.areValidPoints(distortedPoints, 
                undistortedPoints));
        assertFalse(estimator.areValidPoints(emptyPoints, undistortedPoints));
        assertFalse(estimator.areValidPoints(distortedPoints, emptyPoints));
        assertFalse(estimator.areValidPoints(null, undistortedPoints));
        assertFalse(estimator.areValidPoints(distortedPoints, null));
    }    
}
