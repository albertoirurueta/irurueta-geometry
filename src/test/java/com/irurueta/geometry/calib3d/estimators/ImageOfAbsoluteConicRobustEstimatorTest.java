/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.calib3d.estimators.ImageOfAbsoluteConicRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 5, 2015
 */
package com.irurueta.geometry.calib3d.estimators;

import com.irurueta.geometry.ProjectiveTransformation2D;
import com.irurueta.geometry.Transformation2D;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import java.util.ArrayList;
import java.util.List;
import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class ImageOfAbsoluteConicRobustEstimatorTest {
    
    public ImageOfAbsoluteConicRobustEstimatorTest() {}
    
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
        //test with method
        
        //RANSAC
        ImageOfAbsoluteConicRobustEstimator estimator =
                ImageOfAbsoluteConicRobustEstimator.create(
                RobustEstimatorMethod.RANSAC);
        
        //check correctness
        assertEquals(estimator.isZeroSkewness(),
                ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);
        assertEquals(estimator.isPrincipalPointAtOrigin(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);
        assertEquals(estimator.isFocalDistanceAspectRatioKnown(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, 0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertEquals(estimator.getConfidence(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getHomographies());
        assertEquals(estimator.getMinNumberOfRequiredHomographies(),
                1);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACImageOfAbsoluteConicRobustEstimator);
        
        //LMedS
        estimator = ImageOfAbsoluteConicRobustEstimator.create(
                RobustEstimatorMethod.LMedS);
        
        //check correctness
        assertEquals(estimator.isZeroSkewness(),
                ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);
        assertEquals(estimator.isPrincipalPointAtOrigin(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);
        assertEquals(estimator.isFocalDistanceAspectRatioKnown(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, 0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertEquals(estimator.getConfidence(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getHomographies());
        assertEquals(estimator.getMinNumberOfRequiredHomographies(),
                1);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSImageOfAbsoluteConicRobustEstimator);
        
        //MSAC
        estimator = ImageOfAbsoluteConicRobustEstimator.create(
                RobustEstimatorMethod.MSAC);
        
        //check correctness
        assertEquals(estimator.isZeroSkewness(),
                ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);
        assertEquals(estimator.isPrincipalPointAtOrigin(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);
        assertEquals(estimator.isFocalDistanceAspectRatioKnown(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, 0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertEquals(estimator.getConfidence(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getHomographies());
        assertEquals(estimator.getMinNumberOfRequiredHomographies(),
                1);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                MSACImageOfAbsoluteConicRobustEstimator);
        
        //PROSAC
        estimator = ImageOfAbsoluteConicRobustEstimator.create(
                RobustEstimatorMethod.PROSAC);
        
        //check correctness
        assertEquals(estimator.isZeroSkewness(),
                ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);
        assertEquals(estimator.isPrincipalPointAtOrigin(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);
        assertEquals(estimator.isFocalDistanceAspectRatioKnown(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, 0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertEquals(estimator.getConfidence(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getHomographies());
        assertEquals(estimator.getMinNumberOfRequiredHomographies(),
                1);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACImageOfAbsoluteConicRobustEstimator);
        
        //PROMedS
        estimator = ImageOfAbsoluteConicRobustEstimator.create(
                RobustEstimatorMethod.PROMedS);
        
        //check correctness
        assertEquals(estimator.isZeroSkewness(),
                ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);
        assertEquals(estimator.isPrincipalPointAtOrigin(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);
        assertEquals(estimator.isFocalDistanceAspectRatioKnown(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, 0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertEquals(estimator.getConfidence(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getHomographies());
        assertEquals(estimator.getMinNumberOfRequiredHomographies(),
                1);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSImageOfAbsoluteConicRobustEstimator);
        
        //test with homographies, quality scores and method
        List<Transformation2D> homographies = new ArrayList<Transformation2D>();
        homographies.add(new ProjectiveTransformation2D());
        homographies.add(new ProjectiveTransformation2D());
        homographies.add(new ProjectiveTransformation2D());
        double[] qualityScores = new double[3];
        
        //RANSAC
        estimator = ImageOfAbsoluteConicRobustEstimator.create(homographies, 
                qualityScores, RobustEstimatorMethod.RANSAC);
        
        //check correctness
        assertEquals(estimator.isZeroSkewness(), 
                ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);
        assertEquals(estimator.isPrincipalPointAtOrigin(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);
        assertEquals(estimator.isFocalDistanceAspectRatioKnown(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, 0.0);                
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertEquals(estimator.getConfidence(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getHomographies(), homographies);
        assertEquals(estimator.getMinNumberOfRequiredHomographies(),
                1);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACImageOfAbsoluteConicRobustEstimator);
        
        //LMedS
        estimator = ImageOfAbsoluteConicRobustEstimator.create(homographies, 
                qualityScores, RobustEstimatorMethod.LMedS);
        
        //check correctness
        assertEquals(estimator.isZeroSkewness(), 
                ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);
        assertEquals(estimator.isPrincipalPointAtOrigin(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);
        assertEquals(estimator.isFocalDistanceAspectRatioKnown(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, 0.0);                
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertEquals(estimator.getConfidence(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getHomographies(), homographies);
        assertEquals(estimator.getMinNumberOfRequiredHomographies(),
                1);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSImageOfAbsoluteConicRobustEstimator);

        //MSAC
        estimator = ImageOfAbsoluteConicRobustEstimator.create(homographies, 
                qualityScores, RobustEstimatorMethod.MSAC);
        
        //check correctness
        assertEquals(estimator.isZeroSkewness(), 
                ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);
        assertEquals(estimator.isPrincipalPointAtOrigin(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);
        assertEquals(estimator.isFocalDistanceAspectRatioKnown(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, 0.0);        
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertEquals(estimator.getConfidence(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getHomographies(), homographies);
        assertEquals(estimator.getMinNumberOfRequiredHomographies(),
                1);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                MSACImageOfAbsoluteConicRobustEstimator);

        //PROSAC
        estimator = ImageOfAbsoluteConicRobustEstimator.create(homographies, 
                qualityScores, RobustEstimatorMethod.PROSAC);
        
        //check correctness
        assertEquals(estimator.isZeroSkewness(), 
                ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);
        assertEquals(estimator.isPrincipalPointAtOrigin(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);
        assertEquals(estimator.isFocalDistanceAspectRatioKnown(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, 0.0);                
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertEquals(estimator.getConfidence(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getHomographies(), homographies);
        assertEquals(estimator.getMinNumberOfRequiredHomographies(),
                1);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACImageOfAbsoluteConicRobustEstimator);

        //PROMedS
        estimator = ImageOfAbsoluteConicRobustEstimator.create(homographies, 
                qualityScores, RobustEstimatorMethod.PROMedS);
        
        //check correctness
        assertEquals(estimator.isZeroSkewness(), 
                ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);
        assertEquals(estimator.isPrincipalPointAtOrigin(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);
        assertEquals(estimator.isFocalDistanceAspectRatioKnown(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, 0.0);                
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertEquals(estimator.getConfidence(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getHomographies(), homographies);
        assertEquals(estimator.getMinNumberOfRequiredHomographies(),
                1);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSImageOfAbsoluteConicRobustEstimator);
        
        
        //test with homographies and method
        
        //RANSAC
        estimator = ImageOfAbsoluteConicRobustEstimator.create(homographies, 
                RobustEstimatorMethod.RANSAC);
        
        //check correctness
        assertEquals(estimator.isZeroSkewness(), 
                ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);
        assertEquals(estimator.isPrincipalPointAtOrigin(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);
        assertEquals(estimator.isFocalDistanceAspectRatioKnown(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, 0.0);        
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertEquals(estimator.getConfidence(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getHomographies(), homographies);
        assertEquals(estimator.getMinNumberOfRequiredHomographies(),
                1);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof 
                RANSACImageOfAbsoluteConicRobustEstimator);

        //LMedS
        estimator = ImageOfAbsoluteConicRobustEstimator.create(homographies, 
                RobustEstimatorMethod.LMedS);
        
        //check correctness
        assertEquals(estimator.isZeroSkewness(), 
                ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);
        assertEquals(estimator.isPrincipalPointAtOrigin(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);
        assertEquals(estimator.isFocalDistanceAspectRatioKnown(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, 0.0);                
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertEquals(estimator.getConfidence(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getHomographies(), homographies);
        assertEquals(estimator.getMinNumberOfRequiredHomographies(),
                1);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof 
                LMedSImageOfAbsoluteConicRobustEstimator);

        //MSAC
        estimator = ImageOfAbsoluteConicRobustEstimator.create(homographies, 
                RobustEstimatorMethod.MSAC);
        
        //check correctness
        assertEquals(estimator.isZeroSkewness(), 
                ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);
        assertEquals(estimator.isPrincipalPointAtOrigin(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);
        assertEquals(estimator.isFocalDistanceAspectRatioKnown(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, 0.0);                
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertEquals(estimator.getConfidence(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getHomographies(), homographies);
        assertEquals(estimator.getMinNumberOfRequiredHomographies(),
                1);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof 
                MSACImageOfAbsoluteConicRobustEstimator);
        
        //PROSAC
        estimator = ImageOfAbsoluteConicRobustEstimator.create(homographies, 
                RobustEstimatorMethod.PROSAC);
        
        //check correctness
        assertEquals(estimator.isZeroSkewness(), 
                ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);
        assertEquals(estimator.isPrincipalPointAtOrigin(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);
        assertEquals(estimator.isFocalDistanceAspectRatioKnown(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, 0.0);                
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertEquals(estimator.getConfidence(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getHomographies(), homographies);
        assertEquals(estimator.getMinNumberOfRequiredHomographies(),
                1);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACImageOfAbsoluteConicRobustEstimator);

        //PROMedS
        estimator = ImageOfAbsoluteConicRobustEstimator.create(homographies, 
                RobustEstimatorMethod.PROMedS);
        
        //check correctness
        assertEquals(estimator.isZeroSkewness(), 
                ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);
        assertEquals(estimator.isPrincipalPointAtOrigin(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);
        assertEquals(estimator.isFocalDistanceAspectRatioKnown(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, 0.0);                
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertEquals(estimator.getConfidence(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getHomographies(), homographies);
        assertEquals(estimator.getMinNumberOfRequiredHomographies(),
                1);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSImageOfAbsoluteConicRobustEstimator);
        
        //test without aruments
        estimator = ImageOfAbsoluteConicRobustEstimator.create();
        
        //check correctness
        assertEquals(estimator.isZeroSkewness(), 
                ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);
        assertEquals(estimator.isPrincipalPointAtOrigin(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);
        assertEquals(estimator.isFocalDistanceAspectRatioKnown(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, 0.0);                
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertEquals(estimator.getConfidence(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getHomographies());
        assertEquals(estimator.getMinNumberOfRequiredHomographies(),
                1);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), 
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_ROBUST_METHOD);
        assertTrue(estimator instanceof 
                PROSACImageOfAbsoluteConicRobustEstimator);
        
        //test with homographies and quality scores
        estimator = ImageOfAbsoluteConicRobustEstimator.create(homographies, 
                qualityScores);
        
        //check correctness
        assertEquals(estimator.isZeroSkewness(), 
                ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);
        assertEquals(estimator.isPrincipalPointAtOrigin(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);
        assertEquals(estimator.isFocalDistanceAspectRatioKnown(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, 0.0);                
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertEquals(estimator.getConfidence(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getHomographies(), homographies);
        assertEquals(estimator.getMinNumberOfRequiredHomographies(),
                1);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getMethod(), 
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_ROBUST_METHOD);
        assertTrue(estimator instanceof 
                PROSACImageOfAbsoluteConicRobustEstimator);

        //test with homographies
        estimator = ImageOfAbsoluteConicRobustEstimator.create(homographies);
        
        //check correctness
        assertEquals(estimator.isZeroSkewness(), 
                ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);
        assertEquals(estimator.isPrincipalPointAtOrigin(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);
        assertEquals(estimator.isFocalDistanceAspectRatioKnown(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, 0.0);                
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertEquals(estimator.getConfidence(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getHomographies(), homographies);
        assertEquals(estimator.getMinNumberOfRequiredHomographies(),
                1);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), 
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_ROBUST_METHOD);
        assertTrue(estimator instanceof 
                PROSACImageOfAbsoluteConicRobustEstimator);        
    }
}
