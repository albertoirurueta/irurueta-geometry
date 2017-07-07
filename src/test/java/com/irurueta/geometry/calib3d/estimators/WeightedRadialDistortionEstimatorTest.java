/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.estimators.WeightedRadialDistortionEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date March 22, 2015
 */
package com.irurueta.geometry.calib3d.estimators;

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

public class WeightedRadialDistortionEstimatorTest implements 
        RadialDistortionEstimatorListener{
    
    public static final double MIN_POINT_VALUE = -2.0;
    public static final double MAX_POINT_VALUE = 2.0;    
    
    public static final double MIN_PARAM_VALUE = -1e-3;
    public static final double MAX_PARAM_VALUE = 1e-3;    
    
    public static final double MIN_WEIGHT_VALUE = 0.5;
    public static final double MAX_WEIGHT_VALUE = 1.0;    
    
    public static final double ABSOLUTE_ERROR = 1e-8;
    
    public static final int NUM_POINTS = 10;
    
    private int estimateStart;
    private int estimateEnd;
    private int estimationProgressChange;
    
    public WeightedRadialDistortionEstimatorTest() {}
    
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
        WeightedRadialDistortionEstimator estimator =
                new WeightedRadialDistortionEstimator();
        
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
        assertNull(estimator.getWeights());
        assertFalse(estimator.arePointsAvailable());
        assertFalse(estimator.areWeightsAvailable());
        assertEquals(estimator.getMaxPoints(), 
                WeightedRadialDistortionEstimator.DEFAULT_MAX_POINTS);
        assertEquals(estimator.isSortWeightsEnabled(),
                WeightedRadialDistortionEstimator.DEFAULT_SORT_WEIGHTS);
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getType(), 
                RadialDistortionEstimatorType.
                WEIGHTED_RADIAL_DISTORTION_ESTIMATOR);
        
        //test constructor with listener
        estimator = new WeightedRadialDistortionEstimator(this);
        
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
        assertNull(estimator.getWeights());
        assertFalse(estimator.arePointsAvailable());
        assertFalse(estimator.areWeightsAvailable());
        assertEquals(estimator.getMaxPoints(), 
                WeightedRadialDistortionEstimator.DEFAULT_MAX_POINTS);
        assertEquals(estimator.isSortWeightsEnabled(),
                WeightedRadialDistortionEstimator.DEFAULT_SORT_WEIGHTS);        
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getType(), 
                RadialDistortionEstimatorType.
                WEIGHTED_RADIAL_DISTORTION_ESTIMATOR);
        
        //test constructor with distorted/undistorted points
        List<Point2D> distortedPoints = new ArrayList<Point2D>();
        List<Point2D> undistortedPoints = new ArrayList<Point2D>();
        double[] weights = new double[estimator.getMinNumberOfMatchedPoints()];
        for(int i = 0; i < estimator.getMinNumberOfMatchedPoints(); i++){
            distortedPoints.add(Point2D.create());
            undistortedPoints.add(Point2D.create());
        }
        
        estimator = new WeightedRadialDistortionEstimator(distortedPoints, 
                undistortedPoints, weights);
        
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
        assertSame(estimator.getWeights(), weights);
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.areWeightsAvailable());
        assertEquals(estimator.getMaxPoints(), 
                WeightedRadialDistortionEstimator.DEFAULT_MAX_POINTS);
        assertEquals(estimator.isSortWeightsEnabled(),
                WeightedRadialDistortionEstimator.DEFAULT_SORT_WEIGHTS);        
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getType(), 
                RadialDistortionEstimatorType.
                WEIGHTED_RADIAL_DISTORTION_ESTIMATOR);

        //Force IllegalArgumentExcpetion
        List<Point2D> emptyPoints = new ArrayList<Point2D>();
        double[] shortWeights = new double[1];
        estimator = null;
        try{
            estimator = new WeightedRadialDistortionEstimator(emptyPoints,
                    undistortedPoints, weights);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = new WeightedRadialDistortionEstimator(emptyPoints,
                    emptyPoints, weights);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = new WeightedRadialDistortionEstimator(
                    (List<Point2D>)null, undistortedPoints, weights);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = new WeightedRadialDistortionEstimator(distortedPoints,
                    (List<Point2D>)null, weights);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = new WeightedRadialDistortionEstimator(distortedPoints,
                    undistortedPoints, shortWeights);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test constructor with distortion center
        Point2D center = Point2D.create();
        estimator = new WeightedRadialDistortionEstimator(center);
        
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
        assertNull(estimator.getWeights());
        assertFalse(estimator.arePointsAvailable());
        assertFalse(estimator.areWeightsAvailable());
        assertEquals(estimator.getMaxPoints(), 
                WeightedRadialDistortionEstimator.DEFAULT_MAX_POINTS);
        assertEquals(estimator.isSortWeightsEnabled(),
                WeightedRadialDistortionEstimator.DEFAULT_SORT_WEIGHTS);        
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getType(), 
                RadialDistortionEstimatorType.
                WEIGHTED_RADIAL_DISTORTION_ESTIMATOR);
        
        //test constructor with distortion center and listener
        estimator = new WeightedRadialDistortionEstimator(center, this);
        
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
        assertNull(estimator.getWeights());
        assertFalse(estimator.arePointsAvailable());
        assertFalse(estimator.areWeightsAvailable());
        assertEquals(estimator.getMaxPoints(), 
                WeightedRadialDistortionEstimator.DEFAULT_MAX_POINTS);
        assertEquals(estimator.isSortWeightsEnabled(),
                WeightedRadialDistortionEstimator.DEFAULT_SORT_WEIGHTS);        
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getType(), 
                RadialDistortionEstimatorType.
                WEIGHTED_RADIAL_DISTORTION_ESTIMATOR);
        
        //test constructor with distorted/undistorted points and distortion 
        //center
        estimator = new WeightedRadialDistortionEstimator(distortedPoints, 
                undistortedPoints, weights, center);
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
        assertSame(estimator.getWeights(), weights);
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.areWeightsAvailable());
        assertEquals(estimator.getMaxPoints(), 
                WeightedRadialDistortionEstimator.DEFAULT_MAX_POINTS);
        assertEquals(estimator.isSortWeightsEnabled(),
                WeightedRadialDistortionEstimator.DEFAULT_SORT_WEIGHTS);                
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getType(), 
                RadialDistortionEstimatorType.
                WEIGHTED_RADIAL_DISTORTION_ESTIMATOR);
        
        //Force IllegalArgumentExcpetion
        estimator = null;
        try{
            estimator = new WeightedRadialDistortionEstimator(emptyPoints,
                    undistortedPoints, weights, center);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = new WeightedRadialDistortionEstimator(emptyPoints,
                    emptyPoints, weights, center);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = new WeightedRadialDistortionEstimator(
                    (List<Point2D>)null, undistortedPoints, weights, center);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = new WeightedRadialDistortionEstimator(distortedPoints,
                    (List<Point2D>)null, weights, center);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = new WeightedRadialDistortionEstimator(distortedPoints, 
                    undistortedPoints, shortWeights, center);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);

        //test constructor with distorted/undistorted points and distortion 
        //center
        estimator = new WeightedRadialDistortionEstimator(distortedPoints, 
                undistortedPoints, weights, center, this);
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
        assertSame(estimator.getWeights(), weights);
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.areWeightsAvailable());
        assertEquals(estimator.getMaxPoints(), 
                WeightedRadialDistortionEstimator.DEFAULT_MAX_POINTS);
        assertEquals(estimator.isSortWeightsEnabled(),
                WeightedRadialDistortionEstimator.DEFAULT_SORT_WEIGHTS);                        
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getType(), 
                RadialDistortionEstimatorType.
                WEIGHTED_RADIAL_DISTORTION_ESTIMATOR);
        
        //Force IllegalArgumentExcpetion
        estimator = null;
        try{
            estimator = new WeightedRadialDistortionEstimator(emptyPoints,
                    undistortedPoints, weights, center, this);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = new WeightedRadialDistortionEstimator(emptyPoints,
                    emptyPoints, weights, center, this);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = new WeightedRadialDistortionEstimator(
                    (List<Point2D>)null, undistortedPoints, weights, center, 
                    this);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = new WeightedRadialDistortionEstimator(distortedPoints,
                    (List<Point2D>)null, weights, center, this);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = new WeightedRadialDistortionEstimator(distortedPoints,
                    undistortedPoints, shortWeights, center, this);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);        
    }
    
    @Test
    public void testGetSetListsAndWeights() throws LockedException{
        WeightedRadialDistortionEstimator estimator =
                new WeightedRadialDistortionEstimator();
        
        //check default values
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertNull(estimator.getWeights());
        
        //set new value
        List<Point2D> distortedPoints = new ArrayList<Point2D>();
        List<Point2D> undistortedPoints = new ArrayList<Point2D>();
        double[] weights = new double[estimator.getMinNumberOfMatchedPoints()];
        for(int i = 0; i < estimator.getMinNumberOfMatchedPoints(); i++){
            distortedPoints.add(Point2D.create());
            undistortedPoints.add(Point2D.create());
        }
        
        estimator.setPointsAndWeights(distortedPoints, undistortedPoints, 
                weights);
        
        //check correctness
        assertSame(estimator.getDistortedPoints(), distortedPoints);
        assertSame(estimator.getUndistortedPoints(), undistortedPoints);
        assertSame(estimator.getWeights(), weights);
        
        //Force IllegalArgumentExcpetion
        List<Point2D> emptyPoints = new ArrayList<Point2D>();
        double[] shortWeights = new double[1];
        try{
            estimator.setPointsAndWeights(emptyPoints, undistortedPoints, 
                    weights);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator.setPointsAndWeights(emptyPoints, emptyPoints, weights);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator.setPointsAndWeights((List<Point2D>)null, 
                    undistortedPoints, weights);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator.setPointsAndWeights(distortedPoints, (List<Point2D>)null, 
                    weights);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator.setPointsAndWeights(distortedPoints, undistortedPoints, 
                    shortWeights);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
    }
    
    @Test
    public void testAreValidPointsAndWeights(){
        WeightedRadialDistortionEstimator estimator =
                new WeightedRadialDistortionEstimator();
        
        List<Point2D> distortedPoints = new ArrayList<Point2D>();
        List<Point2D> undistortedPoints = new ArrayList<Point2D>();
        double[] weights = new double[estimator.getMinNumberOfMatchedPoints()];
        for(int i = 0; i < estimator.getMinNumberOfMatchedPoints(); i++){
            distortedPoints.add(Point2D.create());
            undistortedPoints.add(Point2D.create());
        }
        List<Point2D> emptyPoints = new ArrayList<Point2D>();

        assertTrue(estimator.areValidPointsAndWeights(distortedPoints, 
                undistortedPoints, weights));
        assertFalse(estimator.areValidPointsAndWeights(emptyPoints, 
                undistortedPoints, weights));
        assertFalse(estimator.areValidPointsAndWeights(distortedPoints, 
                emptyPoints, weights));
        assertFalse(estimator.areValidPointsAndWeights((List<Point2D>)null, 
                undistortedPoints, weights));
        assertFalse(estimator.areValidPointsAndWeights(distortedPoints, 
                (List<Point2D>)null, weights));
    }
    
    @Test
    public void testGetSetMaxPoints() throws LockedException{
        WeightedRadialDistortionEstimator estimator = 
                new WeightedRadialDistortionEstimator();
        
        //check default value
        assertEquals(estimator.getMaxPoints(), 
                WeightedRadialDistortionEstimator.DEFAULT_MAX_POINTS);
        
        //set new value
        estimator.setMaxPoints(10);
        
        //check correctness
        assertEquals(estimator.getMaxPoints(), 10);
        
        //Force IllegalArgumentException
        try{
            estimator.setMaxPoints(1);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
    }
    
    @Test
    public void testIsSetSortWeightsEnabled() throws LockedException{
        WeightedRadialDistortionEstimator estimator =
                new WeightedRadialDistortionEstimator();
        
        //check default value
        assertEquals(estimator.isSortWeightsEnabled(),
                WeightedRadialDistortionEstimator.DEFAULT_SORT_WEIGHTS);
        
        //set new value
        estimator.setSortWeightsEnabled(
                !WeightedRadialDistortionEstimator.DEFAULT_SORT_WEIGHTS);
        
        //check correctness
        assertEquals(estimator.isSortWeightsEnabled(),
                !WeightedRadialDistortionEstimator.DEFAULT_SORT_WEIGHTS);
    }
    
    @Test
    public void testGetSetDistortionCenter() throws LockedException{
        WeightedRadialDistortionEstimator estimator =
                new WeightedRadialDistortionEstimator();

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
        WeightedRadialDistortionEstimator estimator =
                new WeightedRadialDistortionEstimator();
        
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
        WeightedRadialDistortionEstimator estimator =
                new WeightedRadialDistortionEstimator();
        
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
        WeightedRadialDistortionEstimator estimator = 
                new WeightedRadialDistortionEstimator();
        
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
        WeightedRadialDistortionEstimator estimator =
                new WeightedRadialDistortionEstimator();
        
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
        WeightedRadialDistortionEstimator estimator =
                new WeightedRadialDistortionEstimator();
        
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
    public void testEstimate() throws NotSupportedException, LockedException, 
            NotReadyException, RadialDistortionEstimatorException, 
            DistortionException{
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        double k1 = randomizer.nextDouble(MIN_PARAM_VALUE, MAX_PARAM_VALUE);
        double k2 = randomizer.nextDouble(MIN_PARAM_VALUE, MAX_PARAM_VALUE);
        
        Point2D center = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE), 
                randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE));
        
        RadialDistortion distortion = new RadialDistortion(k1, k2, center);
        
        List<Point2D> distortedPoints = new ArrayList<Point2D>();
        List<Point2D> undistortedPoints = new ArrayList<Point2D>();
        double[] weights = new double[NUM_POINTS];
        Point2D undistortedPoint;
        for(int i = 0; i < NUM_POINTS; i++){
            undistortedPoint = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE), 
                    randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE));
            
            undistortedPoints.add(undistortedPoint);
            distortedPoints.add(distortion.distort(undistortedPoint));
            weights[i] = randomizer.nextDouble(MIN_WEIGHT_VALUE, 
                    MAX_WEIGHT_VALUE);
        }
        
        WeightedRadialDistortionEstimator estimator = 
                new WeightedRadialDistortionEstimator(distortedPoints, 
                undistortedPoints, weights, center, this);
        
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
    }
    
    private void reset(){
        estimateStart = estimateEnd = estimationProgressChange = 0;
    }
    
    @Override
    public void onEstimateStart(RadialDistortionEstimator estimator) {
        estimateStart++;
        testLocked((WeightedRadialDistortionEstimator)estimator);
    }

    @Override
    public void onEstimateEnd(RadialDistortionEstimator estimator) {
        estimateEnd++;
        testLocked((WeightedRadialDistortionEstimator)estimator);
    }

    @Override
    public void onEstimationProgressChange(RadialDistortionEstimator estimator, 
            float progress) {
        estimationProgressChange++;
        testLocked((WeightedRadialDistortionEstimator)estimator);
    }
    
    private void testLocked(WeightedRadialDistortionEstimator estimator){
        try{
            estimator.setPoints(null, null);
            fail("LockedException expected but not thrown");
        }catch(LockedException e){}
        try{
            estimator.setDistortionCenter(null);
            fail("LockedException expected but not thrown");
        }catch(LockedException e){}
        try{
            estimator.setPointsAndWeights(null, null, null);
            fail("LockedException expected but not thrown");
        }catch(LockedException e){}
        try{
            estimator.setMaxPoints(NUM_POINTS);
            fail("LockedException expected but not thrown");
        }catch(LockedException e){}
        try{
            estimator.setSortWeightsEnabled(true);
            fail("LockedException expected but not thrown");
        }catch(LockedException e){}
    }    
}
