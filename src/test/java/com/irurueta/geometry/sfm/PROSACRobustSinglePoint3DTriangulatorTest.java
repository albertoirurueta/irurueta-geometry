/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.sfm.PROSACRobustSinglepoint3DTriangulator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date May 2, 2015
 */
package com.irurueta.geometry.sfm;

import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.MatrixRotation3D;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
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

public class PROSACRobustSinglePoint3DTriangulatorTest implements 
        RobustSinglePoint3DTriangulatorListener{
    
    public static final int MIN_VIEWS = 100;
    public static final int MAX_VIEWS = 500;
    
    public static final double MIN_RANDOM_VALUE = 100.0;
    public static final double MAX_RANDOM_VALUE = 500.0;
    
    public static final double ABSOLUTE_ERROR = 5e-5;
    
    public static final double MIN_FOCAL_LENGTH = 1.0;
    public static final double MAX_FOCAL_LENGTH = 10.0;
    
    public static final double MIN_SKEWNESS = -0.001;
    public static final double MAX_SKEWNESS = 0.001;
    
    public static final double MIN_PRINCIPAL_POINT = 0.0;
    public static final double MAX_PRINCIPAL_POINT = 100.0;
    
    public static final double MIN_ANGLE_DEGREES = -10.0;
    public static final double MAX_ANGLE_DEGREES = 10.0;
    
    public static final double MIN_CAMERA_SEPARATION = 5.0;
    public static final double MAX_CAMERA_SEPARATION = 10.0;
    
    public static final double PERCENTAGE_OUTLIERS = 20;
    
    public static final double STD_ERROR = 10.0;
    
    public static final double THRESHOLD = 1e-8;
    
    public static final int TIMES = 100;
    
    private int triangulateStart;
    private int triangulateEnd;
    private int triangulateNextIteration;
    private int triangulateProgressChange;
    
    public PROSACRobustSinglePoint3DTriangulatorTest() {}
    
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
        PROSACRobustSinglePoint3DTriangulator triangulator;
        
        //test constructor without arguments
        triangulator = new PROSACRobustSinglePoint3DTriangulator();
        
        //check correctness
        assertEquals(triangulator.getThreshold(), 
                PROSACRobustSinglePoint3DTriangulator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(triangulator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertNull(triangulator.getListener());
        assertFalse(triangulator.isListenerAvailable());
        assertEquals(triangulator.isUseHomogeneousSolution(),
                PROSACRobustSinglePoint3DTriangulator.
                DEFAULT_USE_HOMOGENEOUS_SOLUTION);
        assertFalse(triangulator.isLocked());
        assertEquals(triangulator.getProgressDelta(), 
                PROSACRobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertEquals(triangulator.getConfidence(), 
                PROSACRobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(triangulator.getMaxIterations(), 
                PROSACRobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS);
        assertNull(triangulator.getPoints2D());
        assertNull(triangulator.getCameras());
        assertNull(triangulator.getQualityScores());
        assertFalse(triangulator.isReady());
        
        //test constructor with listener
        triangulator = new PROSACRobustSinglePoint3DTriangulator(this);
        
        //check correctness
        assertEquals(triangulator.getThreshold(), 
                PROSACRobustSinglePoint3DTriangulator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(triangulator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertSame(triangulator.getListener(), this);
        assertTrue(triangulator.isListenerAvailable());
        assertEquals(triangulator.isUseHomogeneousSolution(),
                PROSACRobustSinglePoint3DTriangulator.
                DEFAULT_USE_HOMOGENEOUS_SOLUTION);
        assertFalse(triangulator.isLocked());
        assertEquals(triangulator.getProgressDelta(), 
                PROSACRobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertEquals(triangulator.getConfidence(), 
                PROSACRobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(triangulator.getMaxIterations(), 
                PROSACRobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS);
        assertNull(triangulator.getPoints2D());
        assertNull(triangulator.getCameras());
        assertNull(triangulator.getQualityScores());
        assertFalse(triangulator.isReady());     
        
        //test constructor with points and cameras
        List<Point2D> points = new ArrayList<Point2D>();
        points.add(Point2D.create());
        points.add(Point2D.create());
        
        List<PinholeCamera> cameras = new ArrayList<PinholeCamera>();
        cameras.add(new PinholeCamera());
        cameras.add(new PinholeCamera());
        
        triangulator = new PROSACRobustSinglePoint3DTriangulator(points, 
                cameras);
        
        //check correctness
        assertEquals(triangulator.getThreshold(), 
                PROSACRobustSinglePoint3DTriangulator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(triangulator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertNull(triangulator.getListener());
        assertFalse(triangulator.isListenerAvailable());
        assertEquals(triangulator.isUseHomogeneousSolution(),
                PROSACRobustSinglePoint3DTriangulator.
                DEFAULT_USE_HOMOGENEOUS_SOLUTION);
        assertFalse(triangulator.isLocked());
        assertEquals(triangulator.getProgressDelta(), 
                PROSACRobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertEquals(triangulator.getConfidence(), 
                PROSACRobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(triangulator.getMaxIterations(), 
                PROSACRobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS);
        assertSame(triangulator.getPoints2D(), points);
        assertSame(triangulator.getCameras(), cameras);
        assertNull(triangulator.getQualityScores());
        assertFalse(triangulator.isReady());        
        
        //Force IllegalArgumentException
        List<Point2D> emptyPoints = new ArrayList<Point2D>();
        List<PinholeCamera> emptyCameras = new ArrayList<PinholeCamera>();
        
        triangulator = null;
        try{
            triangulator = new PROSACRobustSinglePoint3DTriangulator(
                    emptyPoints, cameras);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            triangulator = new PROSACRobustSinglePoint3DTriangulator(
                    points, emptyCameras);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            triangulator = new PROSACRobustSinglePoint3DTriangulator(
                    emptyPoints, emptyCameras);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(triangulator);
        
        //test constructor with points, cameras and listener
        triangulator = new PROSACRobustSinglePoint3DTriangulator(points, 
                cameras, this);
        
        //check correctness
        assertEquals(triangulator.getThreshold(), 
                PROSACRobustSinglePoint3DTriangulator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(triangulator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertSame(triangulator.getListener(), this);
        assertTrue(triangulator.isListenerAvailable());
        assertEquals(triangulator.isUseHomogeneousSolution(),
                PROSACRobustSinglePoint3DTriangulator.
                DEFAULT_USE_HOMOGENEOUS_SOLUTION);
        assertFalse(triangulator.isLocked());
        assertEquals(triangulator.getProgressDelta(), 
                PROSACRobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertEquals(triangulator.getConfidence(), 
                PROSACRobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(triangulator.getMaxIterations(), 
                PROSACRobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS);
        assertSame(triangulator.getPoints2D(), points);
        assertSame(triangulator.getCameras(), cameras);
        assertNull(triangulator.getQualityScores());
        assertFalse(triangulator.isReady());        
        
        //Force IllegalArgumentException
        triangulator = null;
        try{
            triangulator = new PROSACRobustSinglePoint3DTriangulator(
                    emptyPoints, cameras, this);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            triangulator = new PROSACRobustSinglePoint3DTriangulator(
                    points, emptyCameras, this);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            triangulator = new PROSACRobustSinglePoint3DTriangulator(
                    emptyPoints, emptyCameras, this);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(triangulator);  
        
        //test constructor with quality scores
        double[] qualityScores = new double[2];
        triangulator = new PROSACRobustSinglePoint3DTriangulator(qualityScores);
        
        //check correctness
        assertEquals(triangulator.getThreshold(), 
                PROSACRobustSinglePoint3DTriangulator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(triangulator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertNull(triangulator.getListener());
        assertFalse(triangulator.isListenerAvailable());
        assertEquals(triangulator.isUseHomogeneousSolution(),
                PROSACRobustSinglePoint3DTriangulator.
                DEFAULT_USE_HOMOGENEOUS_SOLUTION);
        assertFalse(triangulator.isLocked());
        assertEquals(triangulator.getProgressDelta(), 
                PROSACRobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertEquals(triangulator.getConfidence(), 
                PROSACRobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(triangulator.getMaxIterations(), 
                PROSACRobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS);
        assertNull(triangulator.getPoints2D());
        assertNull(triangulator.getCameras());
        assertSame(triangulator.getQualityScores(), qualityScores);
        assertFalse(triangulator.isReady());
        
        //Force IllegalArgumentException
        double[] shortScores = new double[1];
        triangulator = null;
        try{
            triangulator = new PROSACRobustSinglePoint3DTriangulator(
                    shortScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(triangulator);
        
        //test constructor with quality scores and listener
        triangulator = new PROSACRobustSinglePoint3DTriangulator(qualityScores, 
                this);
        
        //check correctness
        assertEquals(triangulator.getThreshold(), 
                PROSACRobustSinglePoint3DTriangulator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(triangulator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertSame(triangulator.getListener(), this);
        assertTrue(triangulator.isListenerAvailable());
        assertEquals(triangulator.isUseHomogeneousSolution(),
                PROSACRobustSinglePoint3DTriangulator.
                DEFAULT_USE_HOMOGENEOUS_SOLUTION);
        assertFalse(triangulator.isLocked());
        assertEquals(triangulator.getProgressDelta(), 
                PROSACRobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertEquals(triangulator.getConfidence(), 
                PROSACRobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(triangulator.getMaxIterations(), 
                PROSACRobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS);
        assertNull(triangulator.getPoints2D());
        assertNull(triangulator.getCameras());
        assertSame(triangulator.getQualityScores(), qualityScores);
        assertFalse(triangulator.isReady());     
        
        //Force IllegalArgumentException
        triangulator = null;
        try{
            triangulator = new PROSACRobustSinglePoint3DTriangulator(
                    shortScores, this);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(triangulator);
        
        //test constructor with points, cameras and quality scores
        triangulator = new PROSACRobustSinglePoint3DTriangulator(points, 
                cameras, qualityScores);
        
        //check correctness
        assertEquals(triangulator.getThreshold(), 
                PROSACRobustSinglePoint3DTriangulator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(triangulator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertNull(triangulator.getListener());
        assertFalse(triangulator.isListenerAvailable());
        assertEquals(triangulator.isUseHomogeneousSolution(),
                PROSACRobustSinglePoint3DTriangulator.
                DEFAULT_USE_HOMOGENEOUS_SOLUTION);
        assertFalse(triangulator.isLocked());
        assertEquals(triangulator.getProgressDelta(), 
                PROSACRobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertEquals(triangulator.getConfidence(), 
                PROSACRobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(triangulator.getMaxIterations(), 
                PROSACRobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS);
        assertSame(triangulator.getPoints2D(), points);
        assertSame(triangulator.getCameras(), cameras);
        assertSame(triangulator.getQualityScores(), qualityScores);
        assertTrue(triangulator.isReady());        
        
        //Force IllegalArgumentException
        triangulator = null;
        try{
            triangulator = new PROSACRobustSinglePoint3DTriangulator(
                    emptyPoints, cameras, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            triangulator = new PROSACRobustSinglePoint3DTriangulator(
                    points, emptyCameras, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            triangulator = new PROSACRobustSinglePoint3DTriangulator(
                    emptyPoints, emptyCameras, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            triangulator = new PROSACRobustSinglePoint3DTriangulator(
                    emptyPoints, emptyCameras, shortScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            triangulator = new PROSACRobustSinglePoint3DTriangulator(
                    points, cameras, shortScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}        
        assertNull(triangulator);
        
        //test constructor with points, cameras, quality scores and listener
        triangulator = new PROSACRobustSinglePoint3DTriangulator(points, 
                cameras, qualityScores, this);
        
        //check correctness
        assertEquals(triangulator.getThreshold(), 
                PROSACRobustSinglePoint3DTriangulator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(triangulator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertSame(triangulator.getListener(), this);
        assertTrue(triangulator.isListenerAvailable());
        assertEquals(triangulator.isUseHomogeneousSolution(),
                PROSACRobustSinglePoint3DTriangulator.
                DEFAULT_USE_HOMOGENEOUS_SOLUTION);
        assertFalse(triangulator.isLocked());
        assertEquals(triangulator.getProgressDelta(), 
                PROSACRobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertEquals(triangulator.getConfidence(), 
                PROSACRobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(triangulator.getMaxIterations(), 
                PROSACRobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS);
        assertSame(triangulator.getPoints2D(), points);
        assertSame(triangulator.getCameras(), cameras);
        assertSame(triangulator.getQualityScores(), qualityScores);
        assertTrue(triangulator.isReady());        
        
        //Force IllegalArgumentException
        triangulator = null;
        try{
            triangulator = new PROSACRobustSinglePoint3DTriangulator(
                    emptyPoints, cameras, qualityScores, this);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            triangulator = new PROSACRobustSinglePoint3DTriangulator(
                    points, emptyCameras, qualityScores, this);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            triangulator = new PROSACRobustSinglePoint3DTriangulator(
                    emptyPoints, emptyCameras, qualityScores, this);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            triangulator = new PROSACRobustSinglePoint3DTriangulator(
                    emptyPoints, emptyCameras, shortScores, this);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            triangulator = new PROSACRobustSinglePoint3DTriangulator(
                    points, cameras, shortScores, this);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}        
        assertNull(triangulator);          
    }
    
    @Test
    public void testGetSetThreshold() throws LockedException{
        PROSACRobustSinglePoint3DTriangulator triangulator = 
                new PROSACRobustSinglePoint3DTriangulator();
        
        //check default value
        assertEquals(triangulator.getThreshold(),
                PROSACRobustSinglePoint3DTriangulator.DEFAULT_THRESHOLD, 0.0);
        
        //set new value
        triangulator.setThreshold(0.5);
        
        //check correctness
        assertEquals(triangulator.getThreshold(), 0.5, 0.0);
        
        //Force IllegalArgumentException
        try{
            triangulator.setThreshold(0.0);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
    }
    
    @Test
    public void testGetSetListener() throws LockedException{
        PROSACRobustSinglePoint3DTriangulator triangulator =
                new PROSACRobustSinglePoint3DTriangulator();
        
        //check default value
        assertNull(triangulator.getListener());
        assertFalse(triangulator.isListenerAvailable());
        
        //set new value
        triangulator.setListener(this);
        
        //check correctness
        assertSame(triangulator.getListener(), this);
        assertTrue(triangulator.isListenerAvailable());
    }
    
    @Test
    public void testIsSetUseHomogeneousSolution() throws LockedException{
        PROSACRobustSinglePoint3DTriangulator triangulator =
                new PROSACRobustSinglePoint3DTriangulator();
        
        //check default value
        assertEquals(triangulator.isUseHomogeneousSolution(),
                PROSACRobustSinglePoint3DTriangulator.
                DEFAULT_USE_HOMOGENEOUS_SOLUTION);
        
        //set new value
        triangulator.setUseHomogeneousSolution(
                !PROSACRobustSinglePoint3DTriangulator.
                DEFAULT_USE_HOMOGENEOUS_SOLUTION);
        
        //check correctness
        assertEquals(triangulator.isUseHomogeneousSolution(),
                !PROSACRobustSinglePoint3DTriangulator.
                DEFAULT_USE_HOMOGENEOUS_SOLUTION);
    }
    
    @Test
    public void testGetSetProgressDelta() throws LockedException{
        PROSACRobustSinglePoint3DTriangulator triangulator =
                new PROSACRobustSinglePoint3DTriangulator();
        
        //check default value
        assertEquals(triangulator.getProgressDelta(),
                PROSACRobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        
        //set new value
        triangulator.setProgressDelta(0.5f);
        
        //check correctness
        assertEquals(triangulator.getProgressDelta(), 0.5, 0.0);
        
        //Force IllegalArgumentException
        try{
            triangulator.setProgressDelta(-1.0f);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            triangulator.setProgressDelta(2.0f);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
    }
    
    @Test
    public void testGetSetConfidence() throws LockedException{
        PROSACRobustSinglePoint3DTriangulator triangulator =
                new PROSACRobustSinglePoint3DTriangulator();

        //check default value
        assertEquals(triangulator.getConfidence(),
                PROSACRobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE, 0.0);
        
        //set new value
        triangulator.setConfidence(0.5);
        
        //check correctness
        assertEquals(triangulator.getConfidence(), 0.5, 0.0);
        
        //Force IllegalArgumentException
        try{
            triangulator.setConfidence(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            triangulator.setConfidence(2.0);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
    }
    
    @Test
    public void testGetSetMaxIterations() throws LockedException{
        PROSACRobustSinglePoint3DTriangulator triangulator =
                new PROSACRobustSinglePoint3DTriangulator();
        
        //check default value
        assertEquals(triangulator.getMaxIterations(),
                PROSACRobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS);
        
        //set new value
        triangulator.setMaxIterations(1);
        
        //check correctness
        assertEquals(triangulator.getMaxIterations(), 1);
        
        //Force IllegalArgumentException
        try{
            triangulator.setMaxIterations(0);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
    }
    
    @Test
    public void testGetSetPointsAndCamerasAndIsReady() throws LockedException{
        PROSACRobustSinglePoint3DTriangulator triangulator =
                new PROSACRobustSinglePoint3DTriangulator();
        
        //check default values
        assertNull(triangulator.getPoints2D());
        assertNull(triangulator.getCameras());
        assertFalse(triangulator.isReady());
        
        //set new values
        List<Point2D> points = new ArrayList<Point2D>();
        points.add(Point2D.create());
        points.add(Point2D.create());
        
        List<PinholeCamera> cameras = new ArrayList<PinholeCamera>();
        cameras.add(new PinholeCamera());
        cameras.add(new PinholeCamera());
        
        triangulator.setPointsAndCameras(points, cameras);
        
        //check correctness
        assertSame(triangulator.getPoints2D(), points);
        assertSame(triangulator.getCameras(), cameras);
        assertFalse(triangulator.isReady());
        
        //if we set quality scores, instance becomes ready
        double[] qualityScores = new double[2];
        triangulator.setQualityScores(qualityScores);
        
        assertTrue(triangulator.isReady());
        
        //Force IllegalArgumentException
        List<Point2D> emptyPoints = new ArrayList<Point2D>();
        List<PinholeCamera> emptyCameras = new ArrayList<PinholeCamera>();
        
        try{
            triangulator.setPointsAndCameras(emptyPoints, cameras);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            triangulator.setPointsAndCameras(points, emptyCameras);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            triangulator.setPointsAndCameras(emptyPoints, emptyCameras);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}        
    }
    
    @Test
    public void testGetSetQualityScores() throws LockedException{
        PROSACRobustSinglePoint3DTriangulator triangulator =
                new PROSACRobustSinglePoint3DTriangulator();
        
        //check default value
        assertNull(triangulator.getQualityScores());
        
        //set new value
        double[] qualityScores = new double[2];
        triangulator.setQualityScores(qualityScores);
        
        //check correctness
        assertSame(triangulator.getQualityScores(), qualityScores);
        
        //Force IllegalArgumentException
        double[] emptyScores = new double[0];
        try{
            triangulator.setQualityScores(emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
    }
    
    @Test
    public void testTriangulate() throws LockedException, NotReadyException, 
            RobustEstimatorException{
        for(int t = 0; t < TIMES; t++){
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            //obtain number of views
            int numViews = randomizer.nextInt(MIN_VIEWS, MAX_VIEWS);

            //create a random 3D point
            Point3D point3D = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            List<Point2D> points2D = new ArrayList<Point2D>();
            List<PinholeCamera> cameras = new ArrayList<PinholeCamera>();
            double[] qualityScores = new double[numViews];
            Point3D previousCameraCenter = new InhomogeneousPoint3D();
            for(int i = 0; i < numViews; i++){
                //create a random camera
                double horizontalFocalLength = randomizer.nextDouble(
                        MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
                double verticalFocalLength = randomizer.nextDouble(
                        MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
                double skewness = randomizer.nextDouble(MIN_SKEWNESS, 
                        MAX_SKEWNESS);
                double horizontalPrincipalPoint = randomizer.nextDouble(
                        MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
                double verticalPrincipalPoint = randomizer.nextDouble(
                        MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

                double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                        MAX_ANGLE_DEGREES) * Math.PI / 180.0;
                double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                        MAX_ANGLE_DEGREES) * Math.PI / 180.0;
                double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                        MAX_ANGLE_DEGREES) * Math.PI / 180.0;

                double cameraSeparationX = randomizer.nextDouble(
                        MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);
                double cameraSeparationY = randomizer.nextDouble(
                        MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);
                double cameraSeparationZ = randomizer.nextDouble(
                        MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

                PinholeCameraIntrinsicParameters intrinsic = 
                        new PinholeCameraIntrinsicParameters(
                        horizontalFocalLength, verticalFocalLength, 
                        horizontalPrincipalPoint, verticalPrincipalPoint, 
                        skewness);            

                MatrixRotation3D rotation = new MatrixRotation3D(alphaEuler, 
                        betaEuler, gammaEuler);

                Point3D cameraCenter = new InhomogeneousPoint3D(
                        previousCameraCenter.getInhomX() + cameraSeparationX,
                        previousCameraCenter.getInhomY() + cameraSeparationY,
                        previousCameraCenter.getInhomZ() + cameraSeparationZ);

                PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                        cameraCenter);

                //project 3D point using camera
                Point2D point2D = camera.project(point3D);
                
                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS){
                    double errorX = errorRandomizer.nextDouble();
                    double errorY = errorRandomizer.nextDouble();
                    double error = Math.sqrt(errorX * errorX + errorY * errorY);
                    double inhomX = point2D.getInhomX();
                    double inhomY = point2D.getInhomY();
                    
                    point2D.setInhomogeneousCoordinates(inhomX + errorX, 
                            inhomY + errorY);
                    
                    qualityScores[i] = 1.0 / (1.0 + error);
                }else{
                    //inlier
                    qualityScores[i] = 1.0;
                }

                cameras.add(camera);
                points2D.add(point2D);
            }

            //create triangulator
            PROSACRobustSinglePoint3DTriangulator triangulator = 
                    new PROSACRobustSinglePoint3DTriangulator(points2D, cameras, 
                    qualityScores, this);
            triangulator.setThreshold(THRESHOLD);
            
            //check default values
            assertTrue(triangulator.isReady());
            assertFalse(triangulator.isLocked());
            assertEquals(triangulateStart, 0);
            assertEquals(triangulateEnd, 0);
            assertEquals(triangulateNextIteration, 0);
            assertEquals(triangulateProgressChange, 0);
            
            Point3D triangulated = triangulator.triangulate();
            
            //check correctness
            assertTrue(triangulator.isReady());
            assertFalse(triangulator.isLocked());
            assertEquals(triangulateStart, 1);
            assertEquals(triangulateEnd, 1);
            assertTrue(triangulateNextIteration > 0);
            assertTrue(triangulateProgressChange >= 0);
            reset();
            
            assertEquals(point3D.distanceTo(triangulated), 0.0, ABSOLUTE_ERROR);
        }
    }
    
    private void reset(){
        triangulateStart = triangulateEnd = triangulateNextIteration =
                triangulateProgressChange = 0;
    }
    
    @Override
    public void onTriangulateStart(
            RobustSinglePoint3DTriangulator triangulator) {
        triangulateStart++;
        testLocked((PROSACRobustSinglePoint3DTriangulator)triangulator);
    }

    @Override
    public void onTriangulateEnd(RobustSinglePoint3DTriangulator triangulator) {
        triangulateEnd++;
        testLocked((PROSACRobustSinglePoint3DTriangulator)triangulator);
    }

    @Override
    public void onTriangulateNextIteration(
            RobustSinglePoint3DTriangulator triangulator, int iteration) {
        triangulateNextIteration++;
        testLocked((PROSACRobustSinglePoint3DTriangulator)triangulator);
    }

    @Override
    public void onTriangulateProgressChange(
            RobustSinglePoint3DTriangulator triangulator, float progress) {
        triangulateProgressChange++;
        testLocked((PROSACRobustSinglePoint3DTriangulator)triangulator);
    }
    
    private void testLocked(PROSACRobustSinglePoint3DTriangulator triangulator){
        try{
            triangulator.setThreshold(0.5);
            fail("LockedException expected but not thrown");
        }catch(LockedException e){}
        try{
            triangulator.setListener(this);
            fail("LockedException expected but not thrown");
        }catch(LockedException e){}
        try{
            triangulator.setProgressDelta(0.5f);
            fail("LockedException expected but not thrown");
        }catch(LockedException e){}
        try{
            triangulator.setConfidence(0.5);
            fail("LockedException expected but not thrown");
        }catch(LockedException e){}
        try{
            triangulator.setMaxIterations(1);
            fail("LockedException expected but not thrown");
        }catch(LockedException e){}
        try{
            triangulator.setPointsAndCameras(null, null);
            fail("LockedException expected but not thrown");
        }catch(LockedException e){}
        try{
            triangulator.triangulate();
            fail("LockedException expected but not thrown");
        }catch(LockedException e){
        }catch(Exception e){
            fail("LockedException expected but not thrown");
        }
        assertTrue(triangulator.isLocked());        
    }
}
