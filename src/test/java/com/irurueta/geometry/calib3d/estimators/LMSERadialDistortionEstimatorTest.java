/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.estimators.LMSERadialDistortionEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date March 22, 2015
 */
package com.irurueta.geometry.calib3d.estimators;

import com.irurueta.geometry.HomogeneousPoint2D;
import com.irurueta.geometry.calib3d.DistortionException;
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.NotSupportedException;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.calib3d.RadialDistortion;
import com.irurueta.geometry.calib3d.RadialDistortionException;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.statistics.UniformRandomizer;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class LMSERadialDistortionEstimatorTest implements 
        RadialDistortionEstimatorListener{
    
    public static final double MIN_POINT_VALUE = -1.0;
    public static final double MAX_POINT_VALUE = 1.0;    
    
    public static final double MIN_PARAM_VALUE = -1e-4;
    public static final double MAX_PARAM_VALUE = 1e-4;    
    
    public static final double ABSOLUTE_ERROR = 1e-8;
    
    public static final int NUM_POINTS = 10;
    
    private int estimateStart;
    private int estimateEnd;
    private int estimationProgressChange;
    
    public LMSERadialDistortionEstimatorTest() {}
    
    @BeforeClass
    public static void setUpClass() {}
    
    @AfterClass
    public static void tearDownClass() {}
    
    @Before
    public void setUp() {}
    
    @After
    public void tearDown() {}

    @Test
    public void testConstructor(){
        //test constructor without parameters
        LMSERadialDistortionEstimator estimator =
                new LMSERadialDistortionEstimator();
        
        //check correctness
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertEquals(estimator.getHorizontalFocalLength(), 
                RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getVerticalFocalLength(),
                RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getSkew(), 
                RadialDistortionEstimator.DEFAULT_SKEW, 0.0);
        assertEquals(estimator.getIntrinsic().getHorizontalPrincipalPoint(),
                0.0, 0.0);
        assertEquals(estimator.getIntrinsic().getVerticalPrincipalPoint(),
                0.0, 0.0);
        assertEquals(estimator.getIntrinsic().getHorizontalFocalLength(),
                RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getIntrinsic().getVerticalFocalLength(),
                RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getIntrinsic().getSkewness(),
                RadialDistortionEstimator.DEFAULT_SKEW, 0.0);
        assertEquals(estimator.getNumKParams(), 
                RadialDistortionEstimator.DEFAULT_NUM_K_PARAMS);
        assertEquals(estimator.getMinNumberOfMatchedPoints(),
                estimator.getMinNumberOfMatchedPoints());
        assertEquals(estimator.isLMSESolutionAllowed(), 
                LMSERadialDistortionEstimator.DEFAULT_ALLOW_LMSE_SOLUTION);
        assertFalse(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getType(), 
                RadialDistortionEstimatorType.LMSE_RADIAL_DISTORTION_ESTIMATOR);
        
        //test constructor with listener
        estimator = new LMSERadialDistortionEstimator(this);
        
        //check correctness
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertEquals(estimator.getHorizontalFocalLength(), 
                RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getVerticalFocalLength(),
                RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getSkew(), 
                RadialDistortionEstimator.DEFAULT_SKEW, 0.0);
        assertEquals(estimator.getIntrinsic().getHorizontalPrincipalPoint(),
                0.0, 0.0);
        assertEquals(estimator.getIntrinsic().getVerticalPrincipalPoint(),
                0.0, 0.0);
        assertEquals(estimator.getIntrinsic().getHorizontalFocalLength(),
                RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getIntrinsic().getVerticalFocalLength(),
                RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getIntrinsic().getSkewness(),
                RadialDistortionEstimator.DEFAULT_SKEW, 0.0);        
        assertEquals(estimator.getNumKParams(), 
                RadialDistortionEstimator.DEFAULT_NUM_K_PARAMS);
        assertEquals(estimator.getMinNumberOfMatchedPoints(),
                estimator.getMinNumberOfMatchedPoints());        
        assertEquals(estimator.isLMSESolutionAllowed(), 
                LMSERadialDistortionEstimator.DEFAULT_ALLOW_LMSE_SOLUTION);
        assertFalse(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getType(), 
                RadialDistortionEstimatorType.LMSE_RADIAL_DISTORTION_ESTIMATOR);
        
        //test constructor with distorted/undistorted points
        List<Point2D> distortedPoints = new ArrayList<Point2D>();
        List<Point2D> undistortedPoints = new ArrayList<Point2D>();
        for(int i = 0; i < estimator.getMinNumberOfMatchedPoints(); i++){
            distortedPoints.add(Point2D.create());
            undistortedPoints.add(Point2D.create());
        }
        
        estimator = new LMSERadialDistortionEstimator(distortedPoints, 
                undistortedPoints);
        
        //check correctness
        assertSame(estimator.getDistortedPoints(), distortedPoints);
        assertSame(estimator.getUndistortedPoints(), undistortedPoints);
        assertNull(estimator.getDistortionCenter());
        assertEquals(estimator.getHorizontalFocalLength(), 
                RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getVerticalFocalLength(),
                RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getSkew(), 
                RadialDistortionEstimator.DEFAULT_SKEW, 0.0);
        assertEquals(estimator.getIntrinsic().getHorizontalPrincipalPoint(),
                0.0, 0.0);
        assertEquals(estimator.getIntrinsic().getVerticalPrincipalPoint(),
                0.0, 0.0);
        assertEquals(estimator.getIntrinsic().getHorizontalFocalLength(),
                RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getIntrinsic().getVerticalFocalLength(),
                RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getIntrinsic().getSkewness(),
                RadialDistortionEstimator.DEFAULT_SKEW, 0.0);        
        assertEquals(estimator.getNumKParams(), 
                RadialDistortionEstimator.DEFAULT_NUM_K_PARAMS);
        assertEquals(estimator.getMinNumberOfMatchedPoints(),
                estimator.getMinNumberOfMatchedPoints());        
        assertEquals(estimator.isLMSESolutionAllowed(), 
                LMSERadialDistortionEstimator.DEFAULT_ALLOW_LMSE_SOLUTION);
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getType(), 
                RadialDistortionEstimatorType.LMSE_RADIAL_DISTORTION_ESTIMATOR);

        //Force IllegalArgumentExcpetion
        List<Point2D> emptyPoints = new ArrayList<Point2D>();
        estimator = null;
        try{
            estimator = new LMSERadialDistortionEstimator(emptyPoints,
                    undistortedPoints);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = new LMSERadialDistortionEstimator(emptyPoints,
                    emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = new LMSERadialDistortionEstimator((List<Point2D>)null,
                    undistortedPoints);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = new LMSERadialDistortionEstimator(distortedPoints,
                    (List<Point2D>)null);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test constructor with distortion center
        Point2D center = Point2D.create();
        estimator = new LMSERadialDistortionEstimator(center);
        
        //check correctness
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertSame(estimator.getDistortionCenter(), center);
        assertEquals(estimator.getHorizontalFocalLength(), 
                RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getVerticalFocalLength(),
                RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getSkew(), 
                RadialDistortionEstimator.DEFAULT_SKEW, 0.0);
        assertEquals(estimator.getIntrinsic().getHorizontalPrincipalPoint(),
                0.0, 0.0);
        assertEquals(estimator.getIntrinsic().getVerticalPrincipalPoint(),
                0.0, 0.0);
        assertEquals(estimator.getIntrinsic().getHorizontalFocalLength(),
                RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getIntrinsic().getVerticalFocalLength(),
                RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getIntrinsic().getSkewness(),
                RadialDistortionEstimator.DEFAULT_SKEW, 0.0);        
        assertEquals(estimator.getNumKParams(), 
                RadialDistortionEstimator.DEFAULT_NUM_K_PARAMS);
        assertEquals(estimator.getMinNumberOfMatchedPoints(),
                estimator.getMinNumberOfMatchedPoints());        
        assertEquals(estimator.isLMSESolutionAllowed(), 
                LMSERadialDistortionEstimator.DEFAULT_ALLOW_LMSE_SOLUTION);
        assertFalse(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getType(), 
                RadialDistortionEstimatorType.LMSE_RADIAL_DISTORTION_ESTIMATOR);        
        
        //test constructor with distortion center and listener
        estimator = new LMSERadialDistortionEstimator(center, this);
        
        //check correctness
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertSame(estimator.getDistortionCenter(), center);
        assertEquals(estimator.getHorizontalFocalLength(), 
                RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getVerticalFocalLength(),
                RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getSkew(), 
                RadialDistortionEstimator.DEFAULT_SKEW, 0.0);
        assertEquals(estimator.getIntrinsic().getHorizontalPrincipalPoint(),
                0.0, 0.0);
        assertEquals(estimator.getIntrinsic().getVerticalPrincipalPoint(),
                0.0, 0.0);
        assertEquals(estimator.getIntrinsic().getHorizontalFocalLength(),
                RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getIntrinsic().getVerticalFocalLength(),
                RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getIntrinsic().getSkewness(),
                RadialDistortionEstimator.DEFAULT_SKEW, 0.0);        
        assertEquals(estimator.getNumKParams(), 
                RadialDistortionEstimator.DEFAULT_NUM_K_PARAMS);
        assertEquals(estimator.getMinNumberOfMatchedPoints(),
                estimator.getMinNumberOfMatchedPoints());        
        assertEquals(estimator.isLMSESolutionAllowed(), 
                LMSERadialDistortionEstimator.DEFAULT_ALLOW_LMSE_SOLUTION);
        assertFalse(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getType(), 
                RadialDistortionEstimatorType.LMSE_RADIAL_DISTORTION_ESTIMATOR);        
        
        //test constructor with distorted/undistorted points and distortion 
        //center
        estimator = new LMSERadialDistortionEstimator(distortedPoints, 
                undistortedPoints, center);
        assertSame(estimator.getDistortedPoints(), distortedPoints);
        assertSame(estimator.getUndistortedPoints(), undistortedPoints);
        assertSame(estimator.getDistortionCenter(), center);
        assertEquals(estimator.getHorizontalFocalLength(), 
                RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getVerticalFocalLength(),
                RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getSkew(), 
                RadialDistortionEstimator.DEFAULT_SKEW, 0.0);
        assertEquals(estimator.getIntrinsic().getHorizontalPrincipalPoint(),
                0.0, 0.0);
        assertEquals(estimator.getIntrinsic().getVerticalPrincipalPoint(),
                0.0, 0.0);
        assertEquals(estimator.getIntrinsic().getHorizontalFocalLength(),
                RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getIntrinsic().getVerticalFocalLength(),
                RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getIntrinsic().getSkewness(),
                RadialDistortionEstimator.DEFAULT_SKEW, 0.0);        
        assertEquals(estimator.getNumKParams(), 
                RadialDistortionEstimator.DEFAULT_NUM_K_PARAMS);
        assertEquals(estimator.getMinNumberOfMatchedPoints(),
                estimator.getMinNumberOfMatchedPoints());        
        assertEquals(estimator.isLMSESolutionAllowed(), 
                LMSERadialDistortionEstimator.DEFAULT_ALLOW_LMSE_SOLUTION);
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getType(), 
                RadialDistortionEstimatorType.LMSE_RADIAL_DISTORTION_ESTIMATOR);        
        
        //Force IllegalArgumentExcpetion
        estimator = null;
        try{
            estimator = new LMSERadialDistortionEstimator(emptyPoints,
                    undistortedPoints, center);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = new LMSERadialDistortionEstimator(emptyPoints,
                    emptyPoints, center);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = new LMSERadialDistortionEstimator((List<Point2D>)null,
                    undistortedPoints, center);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = new LMSERadialDistortionEstimator(distortedPoints,
                    (List<Point2D>)null, center);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);

        //test constructor with distorted/undistorted points and distortion 
        //center
        estimator = new LMSERadialDistortionEstimator(distortedPoints, 
                undistortedPoints, center, this);
        assertSame(estimator.getDistortedPoints(), distortedPoints);
        assertSame(estimator.getUndistortedPoints(), undistortedPoints);
        assertSame(estimator.getDistortionCenter(), center);
        assertEquals(estimator.getHorizontalFocalLength(), 
                RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getVerticalFocalLength(),
                RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getSkew(), 
                RadialDistortionEstimator.DEFAULT_SKEW, 0.0);
        assertEquals(estimator.getIntrinsic().getHorizontalPrincipalPoint(),
                0.0, 0.0);
        assertEquals(estimator.getIntrinsic().getVerticalPrincipalPoint(),
                0.0, 0.0);
        assertEquals(estimator.getIntrinsic().getHorizontalFocalLength(),
                RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getIntrinsic().getVerticalFocalLength(),
                RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getIntrinsic().getSkewness(),
                RadialDistortionEstimator.DEFAULT_SKEW, 0.0);        
        assertEquals(estimator.getNumKParams(), 
                RadialDistortionEstimator.DEFAULT_NUM_K_PARAMS);
        assertEquals(estimator.getMinNumberOfMatchedPoints(),
                estimator.getMinNumberOfMatchedPoints());        
        assertEquals(estimator.isLMSESolutionAllowed(), 
                LMSERadialDistortionEstimator.DEFAULT_ALLOW_LMSE_SOLUTION);
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getType(), 
                RadialDistortionEstimatorType.LMSE_RADIAL_DISTORTION_ESTIMATOR);        
        
        //Force IllegalArgumentExcpetion
        estimator = null;
        try{
            estimator = new LMSERadialDistortionEstimator(emptyPoints,
                    undistortedPoints, center, this);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = new LMSERadialDistortionEstimator(emptyPoints,
                    emptyPoints, center, this);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = new LMSERadialDistortionEstimator((List<Point2D>)null,
                    undistortedPoints, center, this);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = new LMSERadialDistortionEstimator(distortedPoints,
                    (List<Point2D>)null, center, this);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);        
    }
    
    @Test
    public void testGetSetDistortedUndistortedPoints() throws LockedException{
        LMSERadialDistortionEstimator estimator = 
                new LMSERadialDistortionEstimator();
        
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
        LMSERadialDistortionEstimator estimator = 
                new LMSERadialDistortionEstimator();
        
        //check default value
        assertNull(estimator.getDistortionCenter());
        assertEquals(estimator.getIntrinsic().getHorizontalPrincipalPoint(),
                0.0, 0.0);
        assertEquals(estimator.getIntrinsic().getVerticalPrincipalPoint(),
                0.0, 0.0);
        
        //set new value
        Point2D center = Point2D.create();
        center.setInhomogeneousCoordinates(1.0, 2.0);
        estimator.setDistortionCenter(center);
        
        //check correctness
        assertSame(estimator.getDistortionCenter(), center);        
        assertEquals(estimator.getIntrinsic().getHorizontalPrincipalPoint(),
                center.getInhomX(), 0.0);
        assertEquals(estimator.getIntrinsic().getVerticalPrincipalPoint(),
                center.getInhomY(), 0.0);
    }
    
    @Test
    public void testGetSetHorizontalFocalLength() throws LockedException, 
            RadialDistortionException{
        LMSERadialDistortionEstimator estimator =
                new LMSERadialDistortionEstimator();
        
        //check default value
        assertEquals(estimator.getHorizontalFocalLength(),
                RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getIntrinsic().getHorizontalFocalLength(),
                RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        
        //set new value
        estimator.setHorizontalFocalLength(2.0);
        
        //check correctness
        assertEquals(estimator.getHorizontalFocalLength(), 2.0, 0.0);
        assertEquals(estimator.getIntrinsic().getHorizontalFocalLength(),
                2.0, 0.0);
        
        //Force RadialDistortionException
        try{
            estimator.setHorizontalFocalLength(0.0);
            fail("RadialDistortionException expected but not thrown");
        }catch(RadialDistortionException e){}
    }
    
    @Test
    public void testGetSetVerticalFocalLength() throws LockedException, 
            RadialDistortionException{
        LMSERadialDistortionEstimator estimator =
                new LMSERadialDistortionEstimator();
        
        //check default value
        assertEquals(estimator.getVerticalFocalLength(),
                RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getIntrinsic().getVerticalFocalLength(),
                RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        
        //set new value
        estimator.setVerticalFocalLength(2.0);
        
        //check correctness
        assertEquals(estimator.getVerticalFocalLength(), 2.0, 0.0);
        assertEquals(estimator.getIntrinsic().getVerticalFocalLength(),
                2.0, 0.0);
        
        //Force RadialDistortionException
        try{
            estimator.setVerticalFocalLength(0.0);
            fail("RadialDistortionException expected but not thrown");
        }catch(RadialDistortionException e){}
    }
    
    @Test
    public void testGetSetSkew() throws LockedException{
        LMSERadialDistortionEstimator estimator = 
                new LMSERadialDistortionEstimator();
        
        //check default value
        assertEquals(estimator.getSkew(),
                RadialDistortionEstimator.DEFAULT_SKEW, 0.0);
        assertEquals(estimator.getIntrinsic().getSkewness(),
                RadialDistortionEstimator.DEFAULT_SKEW, 0.0);
        
        //set new value
        estimator.setSkew(1.0);
        
        //check correctness
        assertEquals(estimator.getSkew(), 1.0, 0.0);
        assertEquals(estimator.getIntrinsic().getSkewness(), 1.0, 0.0);
    }
    
    @Test
    public void testSetIntrinsic() throws LockedException, 
            RadialDistortionException{
        LMSERadialDistortionEstimator estimator =
                new LMSERadialDistortionEstimator();
        
        //check default values
        assertNull(estimator.getDistortionCenter());
        assertEquals(estimator.getHorizontalFocalLength(),
                RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getVerticalFocalLength(),
                RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getSkew(),
                RadialDistortionEstimator.DEFAULT_SKEW, 0.0);
        
        //set new values
        Point2D center = new InhomogeneousPoint2D(1.0, 2.0);
        
        estimator.setIntrinsic(center, 3.0, 4.0, 5.0);
        
        //check correctness
        assertEquals(estimator.getDistortionCenter().getInhomX(), 1.0, 0.0);
        assertEquals(estimator.getDistortionCenter().getInhomY(), 2.0, 0.0);
        assertEquals(estimator.getHorizontalFocalLength(), 3.0, 0.0);
        assertEquals(estimator.getVerticalFocalLength(), 4.0, 0.0);
        assertEquals(estimator.getSkew(), 5.0, 0.0);
        
        //Force RadialDistortionException
        try{
            estimator.setIntrinsic(center, 0.0, 0.0, 1.0);
            fail("RadialDistortionException expected but not thrown");
        }catch(RadialDistortionException e){}
        
        //set new values
        PinholeCameraIntrinsicParameters intrinsic = 
                new PinholeCameraIntrinsicParameters(1.0, 2.0, 3.0, 4.0, 5.0);
        estimator.setIntrinsic(intrinsic);
        
        //check correctness
        assertEquals(estimator.getDistortionCenter().getInhomX(), 3.0, 0.0);
        assertEquals(estimator.getDistortionCenter().getInhomY(), 4.0, 0.0);
        assertEquals(estimator.getHorizontalFocalLength(), 1.0, 0.0);
        assertEquals(estimator.getVerticalFocalLength(), 2.0, 0.0);
        assertEquals(estimator.getSkew(), 5.0, 0.0);
        
        //Force RadialDistortionException
        intrinsic = new PinholeCameraIntrinsicParameters(0.0, 0.0, 3.0, 4.0, 
                5.0);
        try{
            estimator.setIntrinsic(intrinsic);
            fail("RadialDistortionException expected but not thrown");
        }catch(RadialDistortionException e){}
    }
    
    @Test
    public void testGetSetNumKParamsAndGetMinNumberOfMatchedPoints() 
            throws LockedException{
        LMSERadialDistortionEstimator estimator =
                new LMSERadialDistortionEstimator();
        
        //check default values
        assertEquals(estimator.getNumKParams(),
                RadialDistortionEstimator.DEFAULT_NUM_K_PARAMS);
        assertEquals(estimator.getMinNumberOfMatchedPoints(),
                estimator.getNumKParams());
        
        //set new value
        estimator.setNumKParams(3);
        
        //check correctness
        assertEquals(estimator.getNumKParams(), 3);
        assertEquals(estimator.getMinNumberOfMatchedPoints(),
                estimator.getNumKParams());
        
        //Force IllegalArgumentException
        try{
            estimator.setNumKParams(0);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
    }
    
    @Test
    public void testIsSetLMSESolutionAllowed() throws LockedException{
        LMSERadialDistortionEstimator estimator = 
                new LMSERadialDistortionEstimator();
        
        //check default value
        assertEquals(estimator.isLMSESolutionAllowed(),
                LMSERadialDistortionEstimator.DEFAULT_ALLOW_LMSE_SOLUTION);
        
        //set new value
        estimator.setLMSESolutionAllowed(
                !LMSERadialDistortionEstimator.DEFAULT_ALLOW_LMSE_SOLUTION);
        
        //check default value
        assertEquals(estimator.isLMSESolutionAllowed(),
                !LMSERadialDistortionEstimator.DEFAULT_ALLOW_LMSE_SOLUTION);
    }
    
    @Test
    public void testAreValidPoints(){
        LMSERadialDistortionEstimator estimator = 
                new LMSERadialDistortionEstimator();
        
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
    
    @Test
    public void testEstimate() throws NotSupportedException, LockedException, 
            NotReadyException, RadialDistortionEstimatorException, 
            DistortionException{
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        double k1 = randomizer.nextDouble(MIN_PARAM_VALUE, MAX_PARAM_VALUE);
        double k2 = randomizer.nextDouble(MIN_PARAM_VALUE, MAX_PARAM_VALUE);
        
        Point2D center = new InhomogeneousPoint2D(//0.0, 0.0);
                randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE), 
                randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE));
        
        RadialDistortion distortion = new RadialDistortion(k1, k2, center);
        
        List<Point2D> distortedPoints = new ArrayList<Point2D>();
        List<Point2D> undistortedPoints = new ArrayList<Point2D>();
        Point2D undistortedPoint;
        for(int i = 0; i < NUM_POINTS; i++){
            undistortedPoint = new HomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE), 
                    randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE), 
                    1.0);
            
            undistortedPoints.add(undistortedPoint);
            distortedPoints.add(distortion.distort(undistortedPoint));
        }
        
        //test estimation when LMSE is not allowed
        LMSERadialDistortionEstimator estimator = 
                new LMSERadialDistortionEstimator(distortedPoints, 
                undistortedPoints, center, this);
        
        assertFalse(estimator.isLMSESolutionAllowed());
        assertEquals(distortion.getCenter().getInhomX(),
                estimator.getDistortionCenter().getInhomX(), ABSOLUTE_ERROR);
        assertEquals(distortion.getCenter().getInhomY(),
                estimator.getDistortionCenter().getInhomY(), ABSOLUTE_ERROR);
        assertEquals(distortion.getHorizontalFocalLength(),
                estimator.getHorizontalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(distortion.getVerticalFocalLength(),
                estimator.getVerticalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(distortion.getSkew(),
                estimator.getSkew(), ABSOLUTE_ERROR);
        
        assertTrue(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertEquals(estimateStart, 0);
        assertEquals(estimateEnd, 0);
        assertEquals(estimationProgressChange, 0);
        
        RadialDistortion distortion2 = estimator.estimate();
        
        //check correctness
        assertTrue(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertEquals(estimateStart, 1);
        assertEquals(estimateEnd, 1);
        assertTrue(estimationProgressChange >= 0);
        reset();
        
        assertEquals(distortion2.getK1(), k1, ABSOLUTE_ERROR);
        assertEquals(distortion2.getK2(), k2, ABSOLUTE_ERROR);
        assertEquals(distortion2.getCenter(), center);
        
        //test estimation when LMSE is allowed
        estimator = new LMSERadialDistortionEstimator(distortedPoints, 
                undistortedPoints, center, this);
        estimator.setLMSESolutionAllowed(true);
        
        assertTrue(estimator.isLMSESolutionAllowed());
        
        assertTrue(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertEquals(estimateStart, 0);
        assertEquals(estimateEnd, 0);
        assertEquals(estimationProgressChange, 0);
        
        distortion2 = estimator.estimate();
        
        //check correctness
        assertTrue(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertEquals(estimateStart, 1);
        assertEquals(estimateEnd, 1);
        assertTrue(estimationProgressChange >= 0);
        reset();
        
        assertEquals(distortion2.getK1(), k1, ABSOLUTE_ERROR);
        assertEquals(distortion2.getK2(), k2, ABSOLUTE_ERROR);
        assertEquals(distortion2.getCenter(), center);        
    }
    
    private void reset(){
        estimateStart = estimateEnd = estimationProgressChange = 0;
    }
    
    @Override
    public void onEstimateStart(RadialDistortionEstimator estimator) {
        estimateStart++;
        testLocked((LMSERadialDistortionEstimator)estimator);
    }

    @Override
    public void onEstimateEnd(RadialDistortionEstimator estimator) {
        estimateEnd++;
        testLocked((LMSERadialDistortionEstimator)estimator);
    }

    @Override
    public void onEstimationProgressChange(RadialDistortionEstimator estimator, 
            float progress) {
        estimationProgressChange++;
        testLocked((LMSERadialDistortionEstimator)estimator);
    }
    
    private void testLocked(LMSERadialDistortionEstimator estimator){
        try{
            estimator.setPoints(null, null);
            fail("LockedException expected but not thrown");
        }catch(LockedException e){}
        try{
            estimator.setDistortionCenter(null);
            fail("LockedException expected but not thrown");
        }catch(LockedException e){}
        try{
            estimator.setLMSESolutionAllowed(true);
            fail("LockedException expected but not thrown");
        }catch(LockedException e){}
    }
}
