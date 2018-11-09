/*
 * Copyright (C) 2017 Alberto Irurueta Carro (alberto@irurueta.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *         http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package com.irurueta.geometry.estimators;

import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.*;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class RANSACEPnPPointCorrespondencePinholeCameraRobustEstimatorTest 
        implements PinholeCameraRobustEstimatorListener {
    
    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-5;
    
    private static final double MIN_RANDOM_VALUE = 50.0;
    private static final double MAX_RANDOM_VALUE = 100.0;
    
    private static final double MIN_FOCAL_LENGTH = 110.0;
    private static final double MAX_FOCAL_LENGTH = 130.0;
    
    private static final double MIN_SKEWNESS = -0.001;
    private static final double MAX_SKEWNESS = 0.001;
    
    private static final double MIN_PRINCIPAL_POINT = 90.0;
    private static final double MAX_PRINCIPAL_POINT = 100.0;
    
    private static final double MIN_ANGLE_DEGREES = 10.0;
    private static final double MAX_ANGLE_DEGREES = 15.0;
    
    private static final int MIN_POINTS = 500;
    private static final int MAX_POINTS = 1000;
    
    private static final int PERCENTAGE_OUTLIER = 20;
    
    private static final int TIMES = 10;
    
    private static final double THRESHOLD = 1e-6;
    
    private static final double OUTLIER_STD_ERROR = 100.0;
    
    private int estimateStart;
    private int estimateEnd;
    private int estimateNextIteration;
    private int estimateProgressChange;
    
    public RANSACEPnPPointCorrespondencePinholeCameraRobustEstimatorTest() { }
    
    @BeforeClass
    public static void setUpClass() { }
    
    @AfterClass
    public static void tearDownClass() { }
    
    @Before
    public void setUp() { }
    
    @After
    public void tearDown() { }

    @Test
    public void testConstructor() {
        //test constructor without arguments
        RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator();
        
        assertEquals(estimator.getThreshold(),
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_PROGRESS_DELTA, 0.0);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertEquals(estimator.isFastRefinementUsed(),
                PinholeCameraRobustEstimator.DEFAULT_USE_FAST_REFINEMENT);
        assertEquals(estimator.isSuggestSkewnessValueEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED);
        assertEquals(estimator.getSuggestedSkewnessValue(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGESTED_SKEWNESS_VALUE, 
                0.0);
        assertEquals(estimator.isSuggestHorizontalFocalLengthEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);
        assertEquals(estimator.getSuggestedHorizontalFocalLengthValue(), 0.0, 
                0.0);
        assertEquals(estimator.isSuggestVerticalFocalLengthEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED);
        assertEquals(estimator.getSuggestedVerticalFocalLengthValue(), 0.0, 
                0.0);
        assertEquals(estimator.isSuggestAspectRatioEnabled(), 
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED);
        assertEquals(estimator.getSuggestedAspectRatioValue(), 
                PinholeCameraRobustEstimator.DEFAULT_SUGGESTED_ASPECT_RATIO_VALUE, 
                0.0);
        assertEquals(estimator.isSuggestPrincipalPointEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED);
        assertNull(estimator.getSuggestedPrincipalPointValue());
        assertEquals(estimator.isSuggestRotationEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED);
        assertNull(estimator.getSuggestedRotationValue());
        assertEquals(estimator.isSuggestCenterEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_CENTER_ENABLED);
        assertNull(estimator.getSuggestedCenterValue());
        assertNull(estimator.getCovariance());       
        assertEquals(estimator.isPlanarConfigurationAllowed(),
                EPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_CONFIGURATION_ALLOWED);
        assertEquals(estimator.isNullspaceDimension2Allowed(),
                EPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_NULLSPACE_DIMENSION2_ALLOWED);
        assertEquals(estimator.isNullspaceDimension3Allowed(),
                EPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_NULLSPACE_DIMENSION3_ALLOWED);
        assertEquals(estimator.getPlanarThreshold(),
                EPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.isComputeAndKeepInliersEnabled(),
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(estimator.isComputeAndKeepResidualsEnabled(),
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);           
        assertNull(estimator.getIntrinsic());
        
        
        //test constructor with listener
        estimator = 
                new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        this);
        
        assertEquals(estimator.getThreshold(),
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_PROGRESS_DELTA, 0.0);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertEquals(estimator.isFastRefinementUsed(),
                PinholeCameraRobustEstimator.DEFAULT_USE_FAST_REFINEMENT);
        assertEquals(estimator.isSuggestSkewnessValueEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED);
        assertEquals(estimator.getSuggestedSkewnessValue(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGESTED_SKEWNESS_VALUE, 
                0.0);
        assertEquals(estimator.isSuggestHorizontalFocalLengthEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);
        assertEquals(estimator.getSuggestedHorizontalFocalLengthValue(), 0.0, 
                0.0);
        assertEquals(estimator.isSuggestVerticalFocalLengthEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED);
        assertEquals(estimator.getSuggestedVerticalFocalLengthValue(), 0.0, 
                0.0);
        assertEquals(estimator.isSuggestAspectRatioEnabled(), 
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED);
        assertEquals(estimator.getSuggestedAspectRatioValue(), 
                PinholeCameraRobustEstimator.DEFAULT_SUGGESTED_ASPECT_RATIO_VALUE, 
                0.0);
        assertEquals(estimator.isSuggestPrincipalPointEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED);
        assertNull(estimator.getSuggestedPrincipalPointValue());
        assertEquals(estimator.isSuggestRotationEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED);
        assertNull(estimator.getSuggestedRotationValue());
        assertEquals(estimator.isSuggestCenterEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_CENTER_ENABLED);
        assertNull(estimator.getSuggestedCenterValue());
        assertNull(estimator.getCovariance());       
        assertEquals(estimator.isPlanarConfigurationAllowed(),
                EPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_CONFIGURATION_ALLOWED);
        assertEquals(estimator.isNullspaceDimension2Allowed(),
                EPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_NULLSPACE_DIMENSION2_ALLOWED);
        assertEquals(estimator.isNullspaceDimension3Allowed(),
                EPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_NULLSPACE_DIMENSION3_ALLOWED);
        assertEquals(estimator.getPlanarThreshold(),
                EPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.isComputeAndKeepInliersEnabled(),
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(estimator.isComputeAndKeepResidualsEnabled(),
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);        
        assertNull(estimator.getIntrinsic());  
        
        
        //test constructor with points
        List<Point3D> points3D = new ArrayList<>();
        List<Point2D> points2D = new ArrayList<>();
        for (int i = 0; i < PointCorrespondencePinholeCameraRobustEstimator.MIN_NUMBER_OF_POINT_CORRESPONDENCES; i++) {
            points3D.add(Point3D.create());
            points2D.add(Point2D.create());
        }
        
        estimator = 
                new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                points3D, points2D);
        
        assertEquals(estimator.getThreshold(),
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_PROGRESS_DELTA, 0.0);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertEquals(estimator.isFastRefinementUsed(),
                PinholeCameraRobustEstimator.DEFAULT_USE_FAST_REFINEMENT);
        assertEquals(estimator.isSuggestSkewnessValueEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED);
        assertEquals(estimator.getSuggestedSkewnessValue(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGESTED_SKEWNESS_VALUE, 
                0.0);
        assertEquals(estimator.isSuggestHorizontalFocalLengthEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);
        assertEquals(estimator.getSuggestedHorizontalFocalLengthValue(), 0.0, 
                0.0);
        assertEquals(estimator.isSuggestVerticalFocalLengthEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED);
        assertEquals(estimator.getSuggestedVerticalFocalLengthValue(), 0.0, 
                0.0);
        assertEquals(estimator.isSuggestAspectRatioEnabled(), 
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED);
        assertEquals(estimator.getSuggestedAspectRatioValue(), 
                PinholeCameraRobustEstimator.DEFAULT_SUGGESTED_ASPECT_RATIO_VALUE, 
                0.0);
        assertEquals(estimator.isSuggestPrincipalPointEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED);
        assertNull(estimator.getSuggestedPrincipalPointValue());
        assertEquals(estimator.isSuggestRotationEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED);
        assertNull(estimator.getSuggestedRotationValue());
        assertEquals(estimator.isSuggestCenterEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_CENTER_ENABLED);
        assertNull(estimator.getSuggestedCenterValue());
        assertNull(estimator.getCovariance());       
        assertEquals(estimator.isPlanarConfigurationAllowed(),
                EPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_CONFIGURATION_ALLOWED);
        assertEquals(estimator.isNullspaceDimension2Allowed(),
                EPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_NULLSPACE_DIMENSION2_ALLOWED);
        assertEquals(estimator.isNullspaceDimension3Allowed(),
                EPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_NULLSPACE_DIMENSION3_ALLOWED);
        assertEquals(estimator.getPlanarThreshold(),
                EPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.isComputeAndKeepInliersEnabled(),
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(estimator.isComputeAndKeepResidualsEnabled(),
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);        
        assertNull(estimator.getIntrinsic());        
        
        //Force IllegalArgumentException
        List<Point3D> points3DEmpty = new ArrayList<>();
        List<Point2D> points2DEmpty = new ArrayList<>();
        estimator = null;
        try {        
            //not enough points
            estimator = 
                    new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                    points3DEmpty, points2DEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            //different sizes
            estimator = 
                    new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                    points3D, points2DEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);
        
        
        //test constructor with listener and points
        estimator = 
                new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                this, points3D, points2D);
        
        assertEquals(estimator.getThreshold(),
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_PROGRESS_DELTA, 0.0);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertEquals(estimator.isFastRefinementUsed(),
                PinholeCameraRobustEstimator.DEFAULT_USE_FAST_REFINEMENT);
        assertEquals(estimator.isSuggestSkewnessValueEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED);
        assertEquals(estimator.getSuggestedSkewnessValue(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGESTED_SKEWNESS_VALUE, 
                0.0);
        assertEquals(estimator.isSuggestHorizontalFocalLengthEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);
        assertEquals(estimator.getSuggestedHorizontalFocalLengthValue(), 0.0, 
                0.0);
        assertEquals(estimator.isSuggestVerticalFocalLengthEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED);
        assertEquals(estimator.getSuggestedVerticalFocalLengthValue(), 0.0, 
                0.0);
        assertEquals(estimator.isSuggestAspectRatioEnabled(), 
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED);
        assertEquals(estimator.getSuggestedAspectRatioValue(), 
                PinholeCameraRobustEstimator.DEFAULT_SUGGESTED_ASPECT_RATIO_VALUE, 
                0.0);
        assertEquals(estimator.isSuggestPrincipalPointEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED);
        assertNull(estimator.getSuggestedPrincipalPointValue());
        assertEquals(estimator.isSuggestRotationEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED);
        assertNull(estimator.getSuggestedRotationValue());
        assertEquals(estimator.isSuggestCenterEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_CENTER_ENABLED);
        assertNull(estimator.getSuggestedCenterValue());
        assertNull(estimator.getCovariance());       
        assertEquals(estimator.isPlanarConfigurationAllowed(),
                EPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_CONFIGURATION_ALLOWED);
        assertEquals(estimator.isNullspaceDimension2Allowed(),
                EPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_NULLSPACE_DIMENSION2_ALLOWED);
        assertEquals(estimator.isNullspaceDimension3Allowed(),
                EPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_NULLSPACE_DIMENSION3_ALLOWED);
        assertEquals(estimator.getPlanarThreshold(),
                EPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.isComputeAndKeepInliersEnabled(),
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(estimator.isComputeAndKeepResidualsEnabled(),
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);        
        assertNull(estimator.getIntrinsic());        
        
        //Force IllegalArgumentException
        estimator = null;
        try {        
            //not enough points
            estimator = 
                    new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                    this, points3DEmpty, points2DEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            //different sizes
            estimator = 
                    new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                    this, points3D, points2DEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);
        
        
        //test constructor with intrinsic and listener
        PinholeCameraIntrinsicParameters intrinsic = 
                new PinholeCameraIntrinsicParameters();
        
        estimator = 
                new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                intrinsic);
        
        assertEquals(estimator.getThreshold(),
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_PROGRESS_DELTA, 0.0);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertEquals(estimator.isFastRefinementUsed(),
                PinholeCameraRobustEstimator.DEFAULT_USE_FAST_REFINEMENT);
        assertEquals(estimator.isSuggestSkewnessValueEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED);
        assertEquals(estimator.getSuggestedSkewnessValue(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGESTED_SKEWNESS_VALUE, 
                0.0);
        assertEquals(estimator.isSuggestHorizontalFocalLengthEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);
        assertEquals(estimator.getSuggestedHorizontalFocalLengthValue(), 0.0, 
                0.0);
        assertEquals(estimator.isSuggestVerticalFocalLengthEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED);
        assertEquals(estimator.getSuggestedVerticalFocalLengthValue(), 0.0, 
                0.0);
        assertEquals(estimator.isSuggestAspectRatioEnabled(), 
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED);
        assertEquals(estimator.getSuggestedAspectRatioValue(), 
                PinholeCameraRobustEstimator.DEFAULT_SUGGESTED_ASPECT_RATIO_VALUE, 
                0.0);
        assertEquals(estimator.isSuggestPrincipalPointEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED);
        assertNull(estimator.getSuggestedPrincipalPointValue());
        assertEquals(estimator.isSuggestRotationEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED);
        assertNull(estimator.getSuggestedRotationValue());
        assertEquals(estimator.isSuggestCenterEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_CENTER_ENABLED);
        assertNull(estimator.getSuggestedCenterValue());
        assertNull(estimator.getCovariance());       
        assertEquals(estimator.isPlanarConfigurationAllowed(),
                EPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_CONFIGURATION_ALLOWED);
        assertEquals(estimator.isNullspaceDimension2Allowed(),
                EPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_NULLSPACE_DIMENSION2_ALLOWED);
        assertEquals(estimator.isNullspaceDimension3Allowed(),
                EPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_NULLSPACE_DIMENSION3_ALLOWED);
        assertEquals(estimator.getPlanarThreshold(),
                EPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.isComputeAndKeepInliersEnabled(),
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(estimator.isComputeAndKeepResidualsEnabled(),
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);        
        assertSame(estimator.getIntrinsic(), intrinsic);
        
        
        //test constructor with listener and intrinsic
        estimator = 
                new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                        this, intrinsic);
        
        assertEquals(estimator.getThreshold(),
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_PROGRESS_DELTA, 0.0);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertEquals(estimator.isFastRefinementUsed(),
                PinholeCameraRobustEstimator.DEFAULT_USE_FAST_REFINEMENT);
        assertEquals(estimator.isSuggestSkewnessValueEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED);
        assertEquals(estimator.getSuggestedSkewnessValue(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGESTED_SKEWNESS_VALUE, 
                0.0);
        assertEquals(estimator.isSuggestHorizontalFocalLengthEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);
        assertEquals(estimator.getSuggestedHorizontalFocalLengthValue(), 0.0, 
                0.0);
        assertEquals(estimator.isSuggestVerticalFocalLengthEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED);
        assertEquals(estimator.getSuggestedVerticalFocalLengthValue(), 0.0, 
                0.0);
        assertEquals(estimator.isSuggestAspectRatioEnabled(), 
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED);
        assertEquals(estimator.getSuggestedAspectRatioValue(), 
                PinholeCameraRobustEstimator.DEFAULT_SUGGESTED_ASPECT_RATIO_VALUE, 
                0.0);
        assertEquals(estimator.isSuggestPrincipalPointEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED);
        assertNull(estimator.getSuggestedPrincipalPointValue());
        assertEquals(estimator.isSuggestRotationEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED);
        assertNull(estimator.getSuggestedRotationValue());
        assertEquals(estimator.isSuggestCenterEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_CENTER_ENABLED);
        assertNull(estimator.getSuggestedCenterValue());
        assertNull(estimator.getCovariance());       
        assertEquals(estimator.isPlanarConfigurationAllowed(),
                EPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_CONFIGURATION_ALLOWED);
        assertEquals(estimator.isNullspaceDimension2Allowed(),
                EPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_NULLSPACE_DIMENSION2_ALLOWED);
        assertEquals(estimator.isNullspaceDimension3Allowed(),
                EPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_NULLSPACE_DIMENSION3_ALLOWED);
        assertEquals(estimator.getPlanarThreshold(),
                EPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.isComputeAndKeepInliersEnabled(),
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(estimator.isComputeAndKeepResidualsEnabled(),
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);        
        assertSame(estimator.getIntrinsic(), intrinsic);
        
        
        //test constructor with points and intrinsic        
        estimator = 
                new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                intrinsic, points3D, points2D);
        
        assertEquals(estimator.getThreshold(),
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_PROGRESS_DELTA, 0.0);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertEquals(estimator.isFastRefinementUsed(),
                PinholeCameraRobustEstimator.DEFAULT_USE_FAST_REFINEMENT);
        assertEquals(estimator.isSuggestSkewnessValueEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED);
        assertEquals(estimator.getSuggestedSkewnessValue(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGESTED_SKEWNESS_VALUE, 
                0.0);
        assertEquals(estimator.isSuggestHorizontalFocalLengthEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);
        assertEquals(estimator.getSuggestedHorizontalFocalLengthValue(), 0.0, 
                0.0);
        assertEquals(estimator.isSuggestVerticalFocalLengthEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED);
        assertEquals(estimator.getSuggestedVerticalFocalLengthValue(), 0.0, 
                0.0);
        assertEquals(estimator.isSuggestAspectRatioEnabled(), 
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED);
        assertEquals(estimator.getSuggestedAspectRatioValue(), 
                PinholeCameraRobustEstimator.DEFAULT_SUGGESTED_ASPECT_RATIO_VALUE, 
                0.0);
        assertEquals(estimator.isSuggestPrincipalPointEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED);
        assertNull(estimator.getSuggestedPrincipalPointValue());
        assertEquals(estimator.isSuggestRotationEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED);
        assertNull(estimator.getSuggestedRotationValue());
        assertEquals(estimator.isSuggestCenterEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_CENTER_ENABLED);
        assertNull(estimator.getSuggestedCenterValue());
        assertNull(estimator.getCovariance());       
        assertEquals(estimator.isPlanarConfigurationAllowed(),
                EPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_CONFIGURATION_ALLOWED);
        assertEquals(estimator.isNullspaceDimension2Allowed(),
                EPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_NULLSPACE_DIMENSION2_ALLOWED);
        assertEquals(estimator.isNullspaceDimension3Allowed(),
                EPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_NULLSPACE_DIMENSION3_ALLOWED);
        assertEquals(estimator.getPlanarThreshold(),
                EPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.isComputeAndKeepInliersEnabled(),
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(estimator.isComputeAndKeepResidualsEnabled(),
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);        
        assertSame(estimator.getIntrinsic(), intrinsic);
        
        //Force IllegalArgumentException
        estimator = null;
        try {        
            //not enough points
            estimator = 
                    new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                    intrinsic, points3DEmpty, points2DEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            //different sizes
            estimator = 
                    new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                    intrinsic, points3D, points2DEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);
        
        
        //test constructor with listener, points and intrinsic
        estimator = new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                this, intrinsic, points3D, points2D);
        
        assertEquals(estimator.getThreshold(),
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PointCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_PROGRESS_DELTA, 0.0);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(), 
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertEquals(estimator.isFastRefinementUsed(),
                PinholeCameraRobustEstimator.DEFAULT_USE_FAST_REFINEMENT);
        assertEquals(estimator.isSuggestSkewnessValueEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED);
        assertEquals(estimator.getSuggestedSkewnessValue(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGESTED_SKEWNESS_VALUE, 
                0.0);
        assertEquals(estimator.isSuggestHorizontalFocalLengthEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);
        assertEquals(estimator.getSuggestedHorizontalFocalLengthValue(), 0.0, 
                0.0);
        assertEquals(estimator.isSuggestVerticalFocalLengthEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED);
        assertEquals(estimator.getSuggestedVerticalFocalLengthValue(), 0.0, 
                0.0);
        assertEquals(estimator.isSuggestAspectRatioEnabled(), 
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED);
        assertEquals(estimator.getSuggestedAspectRatioValue(), 
                PinholeCameraRobustEstimator.DEFAULT_SUGGESTED_ASPECT_RATIO_VALUE, 
                0.0);
        assertEquals(estimator.isSuggestPrincipalPointEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED);
        assertNull(estimator.getSuggestedPrincipalPointValue());
        assertEquals(estimator.isSuggestRotationEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED);
        assertNull(estimator.getSuggestedRotationValue());
        assertEquals(estimator.isSuggestCenterEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_CENTER_ENABLED);
        assertNull(estimator.getSuggestedCenterValue());
        assertNull(estimator.getCovariance());       
        assertEquals(estimator.isPlanarConfigurationAllowed(),
                EPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_CONFIGURATION_ALLOWED);
        assertEquals(estimator.isNullspaceDimension2Allowed(),
                EPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_NULLSPACE_DIMENSION2_ALLOWED);
        assertEquals(estimator.isNullspaceDimension3Allowed(),
                EPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_NULLSPACE_DIMENSION3_ALLOWED);
        assertEquals(estimator.getPlanarThreshold(),
                EPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.isComputeAndKeepInliersEnabled(),
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(estimator.isComputeAndKeepResidualsEnabled(),
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);        
        assertSame(estimator.getIntrinsic(), intrinsic);
        
        //Force IllegalArgumentException
        estimator = null;
        try {        
            //not enough points
            estimator = 
                    new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                    this, intrinsic, points3DEmpty, points2DEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            //different sizes
            estimator = 
                    new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                    this, intrinsic, points3D, points2DEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);        
    }
    
    @Test
    public void testGetSetThreshold() throws LockedException {
        RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator();
        
        //check default value
        assertEquals(estimator.getThreshold(),
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_THRESHOLD, 0.0);
        
        //set new value
        estimator.setThreshold(0.5);
        
        //check correctness
        assertEquals(estimator.getThreshold(), 0.5, 0.0);
        
        //Force IllegalArgumentException
        try {
            estimator.setThreshold(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetConfidence() throws LockedException {
        RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator();

        //check default value
        assertEquals(estimator.getConfidence(),
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        
        //set new value
        estimator.setConfidence(0.5);
        
        //check correctness
        assertEquals(estimator.getConfidence(), 0.5, 0.0);
        
        //Force IllegalArgumentException
        try {
            estimator.setConfidence(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        
        try {
            estimator.setConfidence(2.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetMaxIterations() throws LockedException {
        RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator();
        
        //check default value
        assertEquals(estimator.getMaxIterations(),
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        
        //set new value
        estimator.setMaxIterations(10);
        
        //check correctness
        assertEquals(estimator.getMaxIterations(), 10);
        
        //Force IllegalArgumentException
        try {
            estimator.setMaxIterations(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testIsSetResultRefined() throws LockedException {
        RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator();
        
        //check default value
        assertEquals(estimator.isResultRefined(),
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        
        //set new value
        estimator.setResultRefined(!PinholeCameraRobustEstimator.
                DEFAULT_REFINE_RESULT);
        
        //check correctness
        assertEquals(estimator.isResultRefined(),
                !PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
    }
    
    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator();
        
        //check default value
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        
        //set new value
        estimator.setCovarianceKept(!PinholeCameraRobustEstimator.
                DEFAULT_KEEP_COVARIANCE);
        
        //check correctness
        assertEquals(estimator.isCovarianceKept(),
                !PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
    }
    
    @Test
    public void testIsSetFastRefinementUsed() throws LockedException {
        RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator();
        
        //check default value
        assertEquals(estimator.isFastRefinementUsed(),
                PinholeCameraRobustEstimator.DEFAULT_USE_FAST_REFINEMENT);
        
        //set new value
        estimator.setFastRefinementUsed(!PinholeCameraRobustEstimator.
                DEFAULT_USE_FAST_REFINEMENT);
        
        //check correctness
        assertEquals(estimator.isFastRefinementUsed(),
                !PinholeCameraRobustEstimator.DEFAULT_USE_FAST_REFINEMENT);
    }

    @Test
    public void testGetSetPoints() throws LockedException {
        RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator();
        
        //check default values
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        
        //set new value
        List<Point3D> points3D = new ArrayList<>();
        List<Point2D> points2D = new ArrayList<>();
        for (int i = 0; i < PointCorrespondencePinholeCameraRobustEstimator.MIN_NUMBER_OF_POINT_CORRESPONDENCES; i++) {
            points3D.add(Point3D.create());
            points2D.add(Point2D.create());
        }
        
        estimator.setPoints(points3D, points2D);
        
        //check correctness
        assertSame(estimator.getPoints3D(), points3D);
        assertSame(estimator.getPoints2D(), points2D);
        assertFalse(estimator.isReady());
        
        //Force IllegalArgumentException
        List<Point3D> points3DEmpty = new ArrayList<>();
        List<Point2D> points2DEmpty = new ArrayList<>();
        try {
            //not enough points
            estimator.setPoints(points3DEmpty, points2DEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            //different sizes
            estimator.setPoints(points3D, points2DEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetListenerAndIsListeneravailable() 
            throws LockedException {
        RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator();
        
        //check default value
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        
        //set new value
        estimator.setListener(this);
        
        //check correctness
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
    }

    @Test
    public void testIsSetSugestSkewnessValueEnabled() throws LockedException {
        RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator();
        
        //check default value
        assertEquals(estimator.isSuggestSkewnessValueEnabled(),
                PinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED);
        
        //set new value
        estimator.setSuggestSkewnessValueEnabled(
                !PinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED);
        
        //check correctness
        assertEquals(estimator.isSuggestSkewnessValueEnabled(),
                !PinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED);
    }
    
    @Test
    public void testGetSetSuggestedSkewnessValue() throws LockedException {
        RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator();
        
        //check default value
        assertEquals(estimator.getSuggestedSkewnessValue(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGESTED_SKEWNESS_VALUE, 
                0.0);
        
        //set new value
        estimator.setSuggestedSkewnessValue(-1.0);
        
        //check correctness
        assertEquals(estimator.getSuggestedSkewnessValue(), -1.0, 0.0);
    }
    
    @Test
    public void testIsSetSuggestedHorizontalFocalLengthEnabled() 
            throws LockedException  {
        RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator();
        
        //check default value
        assertEquals(estimator.isSuggestHorizontalFocalLengthEnabled(),
                PinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);
        
        //set new value
        estimator.setSuggestHorizontalFocalLengthEnabled(
                !PinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);
        
        //check correctness
        assertEquals(estimator.isSuggestHorizontalFocalLengthEnabled(),
                !PinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);
    }
    
    @Test
    public void testGetSetSuggestedHorizontalFocalLengthValue() 
            throws LockedException {
        RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator();
        
        //check default value
        assertEquals(estimator.getSuggestedHorizontalFocalLengthValue(), 0.0, 
                0.0);
        
        //set new value
        estimator.setSuggestedHorizontalFocalLengthValue(100.0);
        
        //check correctness
        assertEquals(estimator.getSuggestedHorizontalFocalLengthValue(), 100.0, 
                0.0);
    }
    
    @Test
    public void testIsSetSuggestVerticalFocalLengthEnabled() 
            throws LockedException {
        RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator();
        
        //check default value
        assertEquals(estimator.isSuggestVerticalFocalLengthEnabled(),
                PinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED);
        
        //set new value
        estimator.setSuggestVerticalFocalLengthEnabled(
                !PinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED);
        
        //check correctness
        assertEquals(estimator.isSuggestVerticalFocalLengthEnabled(),
                !PinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED);
    }
    
    @Test
    public void testGetSetSuggestedVerticalFocalLengthValue() 
            throws LockedException {
        RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator();
        
        //check default value
        assertEquals(estimator.getSuggestedVerticalFocalLengthValue(), 0.0, 
                0.0);
        
        estimator.setSuggestedVerticalFocalLengthValue(100.0);
        
        //check correctness
        assertEquals(estimator.getSuggestedVerticalFocalLengthValue(), 100.0,
                0.0);
    }
    
    @Test
    public void testIsSetSuggestAspectRatioEnabled() throws LockedException {
        RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator();
        
        //check default value
        assertEquals(estimator.isSuggestAspectRatioEnabled(),
                PinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED);
        
        //set new value
        estimator.setSuggestAspectRatioEnabled(!PinholeCameraRobustEstimator.
                DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED);
        
        //check correctness
        assertEquals(estimator.isSuggestAspectRatioEnabled(),
                !PinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED);
    }
    
    @Test
    public void testGetSetSuggestedAspectRatioValue() throws LockedException {
        RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator();
        
        //check default value
        assertEquals(estimator.getSuggestedAspectRatioValue(),
                PinholeCameraRobustEstimator.
                        DEFAULT_SUGGESTED_ASPECT_RATIO_VALUE, 0.0);
        
        //set new value
        estimator.setSuggestedAspectRatioValue(-1.0);
        
        //check correctness
        assertEquals(estimator.getSuggestedAspectRatioValue(), -1.0, 0.0);
    }
    
    @Test
    public void testIsSetSuggestPrincipalPointEnabled() throws LockedException {
        RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator();
        
        //check default value
        assertEquals(estimator.isSuggestPrincipalPointEnabled(),
                PinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED);
        
        //set new value
        estimator.setSuggestPrincipalPointEnabled(!PinholeCameraRobustEstimator.
                DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED);
        
        //check correctness
        assertEquals(estimator.isSuggestPrincipalPointEnabled(),
                !PinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED);
    }
    
    @Test
    public void testGetSetSuggestedPrincipalPointValue() 
            throws LockedException {
        RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator();
        
        //check default value
        assertNull(estimator.getSuggestedPrincipalPointValue());
        
        //set new value
        InhomogeneousPoint2D principalPoint = new InhomogeneousPoint2D();
        estimator.setSuggestedPrincipalPointValue(principalPoint);
        
        //check correctness
        assertSame(estimator.getSuggestedPrincipalPointValue(), principalPoint);
    }
    
    @Test
    public void testIsSetSuggestRotationEnabled() throws LockedException {
        RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator();
        
        //check default value
        assertEquals(estimator.isSuggestRotationEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED);
        
        //set new value
        estimator.setSuggestRotationEnabled(!PinholeCameraRobustEstimator.
                DEFAULT_SUGGEST_ROTATION_ENABLED);
        
        //check correctness
        assertEquals(estimator.isSuggestRotationEnabled(),
                !PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED);
    }
    
    @Test
    public void testGetSetSuggestedRotationValue() throws LockedException {
        RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator();
        
        //check default value
        assertNull(estimator.getSuggestedRotationValue());
        
        //set new value
        Quaternion q = new Quaternion();
        estimator.setSuggestedRotationValue(q);
        
        //check correctness
        assertSame(estimator.getSuggestedRotationValue(), q);
    }
    
    @Test
    public void testIsSetSuggestCenterEnabled() throws LockedException {
        RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator();
        
        //check default value
        assertEquals(estimator.isSuggestCenterEnabled(),
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_CENTER_ENABLED);
        
        //set new value
        estimator.setSuggestCenterEnabled(
                !RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_CENTER_ENABLED);
        
        //check correctness
        assertEquals(estimator.isSuggestCenterEnabled(),
                !RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_CENTER_ENABLED);
    }
    
    @Test
    public void testGetSetSuggestedCenterValue() throws LockedException {
        RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator();
        
        //check default value
        assertNull(estimator.getSuggestedCenterValue());
        
        //set new value
        InhomogeneousPoint3D center = new InhomogeneousPoint3D();
        estimator.setSuggestedCenterValue(center);
        
        //check correctness
        assertSame(estimator.getSuggestedCenterValue(), center);
    }
    
    @Test
    public void testGetSetProgressDelta() throws LockedException {
        RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator();
        
        //check default value
        assertEquals(estimator.getProgressDelta(),
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        
        //set new value
        estimator.setProgressDelta(0.5f);
        
        //check correctness
        assertEquals(estimator.getProgressDelta(), 0.5f, 0.0);
        
        //Force IllegalArgumentException
        try {
            estimator.setProgressDelta(-1.0f);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator.setProgressDelta(2.0f);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testIsNormalizeSubsetPointCorrespondences() 
            throws LockedException {
        RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator();
        
        //check default value
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        
        //set new value
        estimator.setNormalizeSubsetPointCorrespondences(true);
        
        //check correctness
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());        
    }
    
    @Test
    public void testGetSetIntrinsic() throws LockedException {
        RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator();
        
        //check default value
        assertNull(estimator.getIntrinsic());
        assertFalse(estimator.isReady());
        
        //set new value
        PinholeCameraIntrinsicParameters intrinsic = 
                new PinholeCameraIntrinsicParameters();
        estimator.setIntrinsic(intrinsic);
        
        //check correctness
        assertSame(estimator.getIntrinsic(), intrinsic);
        assertFalse(estimator.isReady());
    }
    
    @Test
    public void testIsReady() throws LockedException {
        RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator();
        
        //check default values
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertNull(estimator.getIntrinsic());
        
        //set new value
        List<Point3D> points3D = new ArrayList<>();
        List<Point2D> points2D = new ArrayList<>();
        for (int i = 0; i < PointCorrespondencePinholeCameraRobustEstimator.MIN_NUMBER_OF_POINT_CORRESPONDENCES; i++) {
            points3D.add(Point3D.create());
            points2D.add(Point2D.create());
        }
        
        estimator.setPoints(points3D, points2D);
        
        PinholeCameraIntrinsicParameters intrinsic = 
                new PinholeCameraIntrinsicParameters();
        estimator.setIntrinsic(intrinsic);
        
        //check correctness
        assertSame(estimator.getPoints3D(), points3D);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getIntrinsic(), intrinsic);
        assertTrue(estimator.isReady());        
    }
    
    @Test
    public void testIsSetPlanarConfigurationAllowed() throws LockedException {
        RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator();
        
        //check default values
        assertEquals(estimator.isPlanarConfigurationAllowed(),
                EPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_CONFIGURATION_ALLOWED);
        
        //set new value
        estimator.setPlanarConfigurationAllowed(
                !EPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_CONFIGURATION_ALLOWED);
        
        //check correctness
        assertEquals(estimator.isPlanarConfigurationAllowed(),
                !EPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_CONFIGURATION_ALLOWED);
    }
    
    @Test
    public void testIsSetNullspaceDimension2Allowed() throws LockedException {
        RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator();
        
        //check default values
        assertEquals(estimator.isNullspaceDimension2Allowed(),
                EPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_NULLSPACE_DIMENSION2_ALLOWED);
        
        //set new value
        estimator.setNullspaceDimension2Allowed(
                !EPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_NULLSPACE_DIMENSION2_ALLOWED);
        
        //check correctness
        assertEquals(estimator.isNullspaceDimension2Allowed(),
                !EPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_NULLSPACE_DIMENSION2_ALLOWED);
    }
    
    @Test
    public void testIsSetNullspaceDimension3Allowed() throws LockedException {
        RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator();
        
        //check default values
        assertEquals(estimator.isNullspaceDimension3Allowed(),
                EPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_NULLSPACE_DIMENSION3_ALLOWED);
        
        //set new value
        estimator.setNullspaceDimension3Allowed(
                !EPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_NULLSPACE_DIMENSION3_ALLOWED);

        //check correctness
        assertEquals(estimator.isNullspaceDimension3Allowed(),
                !EPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_NULLSPACE_DIMENSION3_ALLOWED);        
    }
    
    @Test
    public void testGetSetPlanarThreshold() throws LockedException {
        RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator();
        
        //check default value
        assertEquals(estimator.getPlanarThreshold(),
                EPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        
        //set new value
        estimator.setPlanarThreshold(1e9);
        
        //check correctness
        assertEquals(estimator.getPlanarThreshold(), 1e9, 0.0);
    }

    @Test
    public void testEstimateGeneralNoSuggestion() 
            throws LockedException, NotReadyException, CameraException,
            NotAvailableException {
        int numValidCameras = 0;
        int numValidProjections = 0;
        for (int t = 0; t < 2*TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            
            //intrinsic parameters
            double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                    verticalFocalLength, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, skewness);
            
            //create rotation parameters
            double roll = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double yaw = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            Quaternion rotation = new Quaternion(roll, pitch, yaw);
            rotation.normalize();
            
            //create camera center
            double[] cameraCenterArray = new double[3];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);
            
            //normalize the camera to improve accuracy
            camera.normalize();  
            
            int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);            
            List<Point3D> points3D = new ArrayList<>(nPoints);
            Point3D point3D;
            for (int i = 0; i < nPoints; i++) {
                point3D = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                points3D.add(point3D);
            }
            
            List<Point2D> points2D = camera.project(points3D);  
            
            //create outliers
            Point2D point2DWithError;
            List<Point2D> points2DWithError = new ArrayList<>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, OUTLIER_STD_ERROR); 
            for (Point2D point2D : points2D) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //point is oulier
                    double errorX = errorRandomizer.nextDouble();
                    double errorY = errorRandomizer.nextDouble();
                    double errorW = errorRandomizer.nextDouble();
                    point2DWithError = new HomogeneousPoint2D(
                            point2D.getHomX() + errorX, 
                            point2D.getHomY() + errorY, 
                            point2D.getHomW() + errorW);
                } else {
                    //inlier point (without error)
                    point2DWithError = point2D;
                }
                
                points2DWithError.add(point2DWithError);
            }
            
            RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                    new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                    this, intrinsic, points3D, points2DWithError);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);            
            estimator.setResultRefined(false);
            estimator.setCovarianceKept(false);
            
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getCovariance());

            reset();
            
            PinholeCamera camera2;
            try {
                camera2 = estimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }
            
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            assertNull(estimator.getCovariance());
            reset();

            //check correctness of estimation
                        
            //decompose estimated camera and check its parameters
            camera2.decompose();
            
            //Comparing camera intrinsic parameters
            PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    camera2.getIntrinsicParameters();

            assertEquals(horizontalFocalLength, 
                    estimatedIntrinsic.getHorizontalFocalLength(), 
                    ABSOLUTE_ERROR);
            assertEquals(verticalFocalLength,
                    estimatedIntrinsic.getVerticalFocalLength(), 
                    ABSOLUTE_ERROR);
            assertEquals(horizontalPrincipalPoint,
                    estimatedIntrinsic.getHorizontalPrincipalPoint(), 
                    ABSOLUTE_ERROR);
            assertEquals(verticalPrincipalPoint,
                    estimatedIntrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(skewness, estimatedIntrinsic.getSkewness(), 
                    ABSOLUTE_ERROR);
            
            //comparing estimated camera center
            Point3D estimatedCameraCenter = camera2.getCameraCenter();
            if(!cameraCenter.equals(estimatedCameraCenter, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(cameraCenter.equals(estimatedCameraCenter, 
                    ABSOLUTE_ERROR));            
            
            //comparing estimated rotation
            Quaternion estimatedRotation = camera2.getCameraRotation().
                    toQuaternion();
            estimatedRotation.normalize();
            
            Matrix rotMatrix = rotation.asInhomogeneousMatrix();
            Matrix estimatedRotMatrix = estimatedRotation.
                    asInhomogeneousMatrix();
            
            assertEquals(rotation.getA(), estimatedRotation.getA(), 
                    ABSOLUTE_ERROR);
            assertEquals(rotation.getB(), estimatedRotation.getB(), 
                    ABSOLUTE_ERROR);
            assertEquals(rotation.getC(), estimatedRotation.getC(), 
                    ABSOLUTE_ERROR);
            assertEquals(rotation.getD(), estimatedRotation.getD(), 
                    ABSOLUTE_ERROR);  
            
            assertTrue(rotMatrix.equals(estimatedRotMatrix, ABSOLUTE_ERROR));
            
            numValidCameras++;

            //project original 3D points using estimated camera and check
            //distance to 2D points without error
            Point2D originalPoint2D, estimatedPoint2D;
            boolean isValid = true;
            for (int i = 0; i < nPoints; i++) {
                point3D = points3D.get(i);
                originalPoint2D = points2D.get(i);
                estimatedPoint2D = camera2.project(point3D);

                if(originalPoint2D.distanceTo(estimatedPoint2D) > ABSOLUTE_ERROR) {
                    isValid = false;
                    break;
                }
                assertEquals(originalPoint2D.distanceTo(estimatedPoint2D), 
                        0.0, ABSOLUTE_ERROR);
            }                   
            
            if(isValid) {
                numValidProjections++;  
            }         
            
            if (numValidProjections > 0 && numValidCameras > 0) {
                break;
            }
        }
        
        assertTrue(numValidCameras > 0);
        assertTrue(numValidProjections > 0);
    }
    
    @Test
    public void testEstimateGeneralNoSuggestionWithRefinement() 
            throws LockedException, NotReadyException, CameraException,
            NotAvailableException {
        int numValidCameras = 0;
        int numValidProjections = 0;
        int numCovariances = 0;
        for (int t = 0; t < 5*TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            
            //intrinsic parameters
            double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                    verticalFocalLength, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, skewness);
            
            //create rotation parameters
            double roll = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double yaw = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            Quaternion rotation = new Quaternion(roll, pitch, yaw);
            rotation.normalize();
            
            //create camera center
            double[] cameraCenterArray = new double[3];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);
            
            //normalize the camera to improve accuracy
            camera.normalize();  
            
            int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);            
            List<Point3D> points3D = new ArrayList<>(nPoints);
            Point3D point3D;
            for (int i = 0; i < nPoints; i++) {
                point3D = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                points3D.add(point3D);
            }
            
            List<Point2D> points2D = camera.project(points3D);  
            
            //create outliers
            Point2D point2DWithError;
            List<Point2D> points2DWithError = new ArrayList<>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, OUTLIER_STD_ERROR); 
            for (Point2D point2D : points2D) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //point is oulier
                    double errorX = errorRandomizer.nextDouble();
                    double errorY = errorRandomizer.nextDouble();
                    double errorW = errorRandomizer.nextDouble();
                    point2DWithError = new HomogeneousPoint2D(
                            point2D.getHomX() + errorX, 
                            point2D.getHomY() + errorY, 
                            point2D.getHomW() + errorW);
                } else {
                    //inlier point (without error)
                    point2DWithError = point2D;
                }
                
                points2DWithError.add(point2DWithError);
            }
            
            RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                    new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                    this, intrinsic, points3D, points2DWithError);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);            
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);
            
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getCovariance());

            reset();
            
            PinholeCamera camera2;
            try {
                camera2 = estimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }
            
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            if (estimator.getCovariance() != null) {
                numCovariances++;
            }
            reset();

            //check correctness of estimation
                        
            //decompose estimated camera and check its parameters
            camera2.decompose();
            
            //Comparing camera intrinsic parameters
            PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    camera2.getIntrinsicParameters();

            if(Math.abs(horizontalFocalLength - 
                    estimatedIntrinsic.getHorizontalFocalLength()) > ABSOLUTE_ERROR) {
                continue;
            }            
            assertEquals(horizontalFocalLength, 
                    estimatedIntrinsic.getHorizontalFocalLength(), 
                    ABSOLUTE_ERROR);
            
            if (Math.abs(verticalFocalLength - estimatedIntrinsic.getVerticalFocalLength()) > ABSOLUTE_ERROR) {
                continue;
            }            
            assertEquals(verticalFocalLength,
                    estimatedIntrinsic.getVerticalFocalLength(), 
                    ABSOLUTE_ERROR);
            
            if (Math.abs(horizontalPrincipalPoint - estimatedIntrinsic.getHorizontalPrincipalPoint()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(horizontalPrincipalPoint,
                    estimatedIntrinsic.getHorizontalPrincipalPoint(), 
                    ABSOLUTE_ERROR);
            
            if (Math.abs(verticalPrincipalPoint - estimatedIntrinsic.getVerticalPrincipalPoint()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(verticalPrincipalPoint,
                    estimatedIntrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            
            if (Math.abs(skewness - estimatedIntrinsic.getSkewness()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(skewness, estimatedIntrinsic.getSkewness(), 
                    ABSOLUTE_ERROR);
            
            //comparing estimated camera center
            Point3D estimatedCameraCenter = camera2.getCameraCenter();
            if(!cameraCenter.equals(estimatedCameraCenter, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(cameraCenter.equals(estimatedCameraCenter, 
                    ABSOLUTE_ERROR));            
            
            //comparing estimated rotation
            Quaternion estimatedRotation = camera2.getCameraRotation().
                    toQuaternion();
            estimatedRotation.normalize();
            
            Matrix rotMatrix = rotation.asInhomogeneousMatrix();
            Matrix estimatedRotMatrix = estimatedRotation.
                    asInhomogeneousMatrix();
            
            assertEquals(rotation.getA(), estimatedRotation.getA(), 
                    ABSOLUTE_ERROR);
            assertEquals(rotation.getB(), estimatedRotation.getB(), 
                    ABSOLUTE_ERROR);
            assertEquals(rotation.getC(), estimatedRotation.getC(), 
                    ABSOLUTE_ERROR);
            assertEquals(rotation.getD(), estimatedRotation.getD(), 
                    ABSOLUTE_ERROR);  
            
            assertTrue(rotMatrix.equals(estimatedRotMatrix, ABSOLUTE_ERROR));
            
            numValidCameras++;

            //project original 3D points using estimated camera and check
            //distance to 2D points without error
            Point2D originalPoint2D, estimatedPoint2D;
            boolean isValid = true;
            for (int i = 0; i < nPoints; i++) {
                point3D = points3D.get(i);
                originalPoint2D = points2D.get(i);
                estimatedPoint2D = camera2.project(point3D);

                if(originalPoint2D.distanceTo(estimatedPoint2D) > LARGE_ABSOLUTE_ERROR) {
                    isValid = false;
                    break;
                }
                assertEquals(originalPoint2D.distanceTo(estimatedPoint2D), 
                        0.0, LARGE_ABSOLUTE_ERROR);
            }                   
            
            if(isValid) {
                numValidProjections++;  
            }
            
            if (numValidCameras > 0 && numValidProjections > 0 && 
                    numCovariances > 0) {
                break;
            }
        }
        
        assertTrue(numValidCameras > 0);
        assertTrue(numValidProjections > 0);
        assertTrue(numCovariances > 0);
    }
    
    @Test
    public void testEstimateGeneralNoSuggestionWithFastRefinement() 
            throws LockedException, NotReadyException, CameraException,
            NotAvailableException {
        int numValidCameras = 0;
        int numValidProjections = 0;
        int numCovariances = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            
            //intrinsic parameters
            double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                    verticalFocalLength, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, skewness);
            
            //create rotation parameters
            double roll = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double yaw = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            Quaternion rotation = new Quaternion(roll, pitch, yaw);
            rotation.normalize();
            
            //create camera center
            double[] cameraCenterArray = new double[3];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);
            
            //normalize the camera to improve accuracy
            camera.normalize();  
            
            int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);            
            List<Point3D> points3D = new ArrayList<>(nPoints);
            Point3D point3D;
            for (int i = 0; i < nPoints; i++) {
                point3D = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                points3D.add(point3D);
            }
            
            List<Point2D> points2D = camera.project(points3D);  
            
            //create outliers
            Point2D point2DWithError;
            List<Point2D> points2DWithError = new ArrayList<>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, OUTLIER_STD_ERROR); 
            for (Point2D point2D : points2D) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //point is oulier
                    double errorX = errorRandomizer.nextDouble();
                    double errorY = errorRandomizer.nextDouble();
                    double errorW = errorRandomizer.nextDouble();
                    point2DWithError = new HomogeneousPoint2D(
                            point2D.getHomX() + errorX, 
                            point2D.getHomY() + errorY, 
                            point2D.getHomW() + errorW);
                } else {
                    //inlier point (without error)
                    point2DWithError = point2D;
                }
                
                points2DWithError.add(point2DWithError);
            }
            
            RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                    new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                    this, intrinsic, points3D, points2DWithError);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);            
            estimator.setResultRefined(true);
            estimator.setFastRefinementUsed(true);
            estimator.setCovarianceKept(true);
            
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getCovariance());

            reset();
            
            PinholeCamera camera2;
            try {
                camera2 = estimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }
            
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            if (estimator.getCovariance() != null) {
                numCovariances++;
            }
            reset();

            //check correctness of estimation
                        
            //decompose estimated camera and check its parameters
            camera2.decompose();
            
            //Comparing camera intrinsic parameters
            PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    camera2.getIntrinsicParameters();

            assertEquals(horizontalFocalLength, 
                    estimatedIntrinsic.getHorizontalFocalLength(), 
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(verticalFocalLength,
                    estimatedIntrinsic.getVerticalFocalLength(), 
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(horizontalPrincipalPoint,
                    estimatedIntrinsic.getHorizontalPrincipalPoint(), 
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(verticalPrincipalPoint,
                    estimatedIntrinsic.getVerticalPrincipalPoint(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(skewness, estimatedIntrinsic.getSkewness(), 
                    LARGE_ABSOLUTE_ERROR);
            
            //comparing estimated camera center
            Point3D estimatedCameraCenter = camera2.getCameraCenter();
            if(!cameraCenter.equals(estimatedCameraCenter, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(cameraCenter.equals(estimatedCameraCenter, 
                    ABSOLUTE_ERROR));            
            
            //comparing estimated rotation
            Quaternion estimatedRotation = camera2.getCameraRotation().
                    toQuaternion();
            estimatedRotation.normalize();
            
            Matrix rotMatrix = rotation.asInhomogeneousMatrix();
            Matrix estimatedRotMatrix = estimatedRotation.
                    asInhomogeneousMatrix();
            
            assertEquals(rotation.getA(), estimatedRotation.getA(), 
                    ABSOLUTE_ERROR);
            assertEquals(rotation.getB(), estimatedRotation.getB(), 
                    ABSOLUTE_ERROR);
            assertEquals(rotation.getC(), estimatedRotation.getC(), 
                    ABSOLUTE_ERROR);
            assertEquals(rotation.getD(), estimatedRotation.getD(), 
                    ABSOLUTE_ERROR);  
            
            assertTrue(rotMatrix.equals(estimatedRotMatrix, ABSOLUTE_ERROR));
            
            numValidCameras++;

            //project original 3D points using estimated camera and check
            //distance to 2D points without error
            Point2D originalPoint2D, estimatedPoint2D;
            boolean isValid = true;
            for (int i = 0; i < nPoints; i++) {
                point3D = points3D.get(i);
                originalPoint2D = points2D.get(i);
                estimatedPoint2D = camera2.project(point3D);

                if(originalPoint2D.distanceTo(estimatedPoint2D) > LARGE_ABSOLUTE_ERROR) {
                    isValid = false;
                    break;
                }
                assertEquals(originalPoint2D.distanceTo(estimatedPoint2D), 
                        0.0, LARGE_ABSOLUTE_ERROR);
            }                   
            
            if(isValid) {
                numValidProjections++;  
            }
            
            if (numValidCameras > 0 && numValidProjections > 0 && 
                    numCovariances > 0) {
                break;
            }
        }
        
        assertTrue(numValidCameras > 0);
        assertTrue(numValidProjections > 0);
        assertTrue(numCovariances > 0);
    }
    
    @Test
    public void testEstimateGeneralSuggestedRotationEnabled() 
            throws LockedException, NotReadyException, CameraException,
            NotAvailableException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            
            //intrinsic parameters
            double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                    verticalFocalLength, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, skewness);
            
            //create rotation parameters
            double roll = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double yaw = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            Quaternion rotation = new Quaternion(roll, pitch, yaw);
            rotation.normalize();
            
            //create camera center
            double[] cameraCenterArray = new double[3];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);
            
            //normalize the camera to improve accuracy
            camera.normalize();  
            
            int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);            
            List<Point3D> points3D = new ArrayList<>(nPoints);
            Point3D point3D;
            for (int i = 0; i < nPoints; i++) {
                point3D = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                points3D.add(point3D);
            }
            
            List<Point2D> points2D = camera.project(points3D);  
            
            //create outliers
            Point2D point2DWithError;
            List<Point2D> points2DWithError = new ArrayList<>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, OUTLIER_STD_ERROR); 
            for (Point2D point2D : points2D) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //point is oulier
                    double errorX = errorRandomizer.nextDouble();
                    double errorY = errorRandomizer.nextDouble();
                    double errorW = errorRandomizer.nextDouble();
                    point2DWithError = new HomogeneousPoint2D(
                            point2D.getHomX() + errorX, 
                            point2D.getHomY() + errorY, 
                            point2D.getHomW() + errorW);
                } else {
                    //inlier point (without error)
                    point2DWithError = point2D;
                }
                
                points2DWithError.add(point2DWithError);
            }
            
            RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                    new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                    this, intrinsic, points3D, points2DWithError);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);            
            estimator.setSuggestRotationEnabled(true);
            estimator.setSuggestedRotationValue(rotation);            
            
            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getCovariance());
            
            
            PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }

            assertFalse(estimator.isLocked());
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();
            
            //decompone estimated camera and check its parameters
            estimatedCamera.decompose();
            
            //comparing rotation
            Quaternion estimatedRotation = estimatedCamera.getCameraRotation().
                    toQuaternion();
            estimatedRotation.normalize();
            
            //estimate without suggestion
            estimator.setSuggestRotationEnabled(false);
            
            PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }         
            
            estimatedCameraNoSuggestion.decompose();

            Quaternion estimatedRotationNoSuggestion = 
                    estimatedCameraNoSuggestion.getCameraRotation().
                            toQuaternion();
            
            //check that rotation has become closer to suggested value
            double diffEstimatedNoSuggestion = 
                    Math.pow(rotation.getA() - 
                            estimatedRotationNoSuggestion.getA(), 2.0) +
                    Math.pow(rotation.getB() - 
                            estimatedRotationNoSuggestion.getB(), 2.0) +
                    Math.pow(rotation.getC() - 
                            estimatedRotationNoSuggestion.getC(), 2.0) +
                    Math.pow(rotation.getD() - 
                            estimatedRotationNoSuggestion.getD(), 2.0);
            double diffEstimated =
                    Math.pow(rotation.getA() - estimatedRotation.getA(), 2.0) +
                    Math.pow(rotation.getB() - estimatedRotation.getB(), 2.0) +
                    Math.pow(rotation.getC() - estimatedRotation.getC(), 2.0) +
                    Math.pow(rotation.getD() - estimatedRotation.getD(), 2.0);
            
            if (diffEstimatedNoSuggestion >= diffEstimated) {
                numValid++;
            }            
            
            if (numValid > 0) {
                break;
            }
        }
        
        assertTrue(numValid > 0);
    }
    
    @Test
    public void testEstimateGeneralSuggestedRotationWithRefinement() 
            throws LockedException, NotReadyException, CameraException,
            NotAvailableException {
        int numCovariances = 0;
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            
            //intrinsic parameters
            double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                    verticalFocalLength, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, skewness);
            
            //create rotation parameters
            double roll = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double yaw = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            Quaternion rotation = new Quaternion(roll, pitch, yaw);
            rotation.normalize();
            
            //create camera center
            double[] cameraCenterArray = new double[3];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);
            
            //normalize the camera to improve accuracy
            camera.normalize();  
            
            int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);            
            List<Point3D> points3D = new ArrayList<>(nPoints);
            Point3D point3D;
            for (int i = 0; i < nPoints; i++) {
                point3D = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                points3D.add(point3D);
            }
            
            List<Point2D> points2D = camera.project(points3D);  
            
            //create outliers
            Point2D point2DWithError;
            List<Point2D> points2DWithError = new ArrayList<>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, OUTLIER_STD_ERROR); 
            for (Point2D point2D : points2D) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //point is oulier
                    double errorX = errorRandomizer.nextDouble();
                    double errorY = errorRandomizer.nextDouble();
                    double errorW = errorRandomizer.nextDouble();
                    point2DWithError = new HomogeneousPoint2D(
                            point2D.getHomX() + errorX, 
                            point2D.getHomY() + errorY, 
                            point2D.getHomW() + errorW);
                } else {
                    //inlier point (without error)
                    point2DWithError = point2D;
                }
                
                points2DWithError.add(point2DWithError);
            }
            
            RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                    new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                    this, intrinsic, points3D, points2DWithError);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);                        
            estimator.setSuggestRotationEnabled(true);
            estimator.setSuggestedRotationValue(rotation);   
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);            
            
            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getCovariance());
                        
            PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }

            assertFalse(estimator.isLocked());
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            if (estimator.getCovariance() != null) {
                numCovariances++;
            }            
            reset();
            
            //decompone estimated camera and check its parameters
            estimatedCamera.decompose();
            
            //comparing rotation
            Quaternion estimatedRotation = estimatedCamera.getCameraRotation().
                    toQuaternion();
            estimatedRotation.normalize();
            
            //estimate without suggestion
            estimator.setSuggestRotationEnabled(false);
            
            PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }         
            
            estimatedCameraNoSuggestion.decompose();

            Quaternion estimatedRotationNoSuggestion = 
                    estimatedCameraNoSuggestion.getCameraRotation().
                            toQuaternion();
            
            //check that rotation has become closer to suggested value
            double diffEstimatedNoSuggestion = 
                    Math.pow(rotation.getA() - 
                            estimatedRotationNoSuggestion.getA(), 2.0) +
                    Math.pow(rotation.getB() - 
                            estimatedRotationNoSuggestion.getB(), 2.0) +
                    Math.pow(rotation.getC() - 
                            estimatedRotationNoSuggestion.getC(), 2.0) +
                    Math.pow(rotation.getD() - 
                            estimatedRotationNoSuggestion.getD(), 2.0);
            double diffEstimated =
                    Math.pow(rotation.getA() - estimatedRotation.getA(), 2.0) +
                    Math.pow(rotation.getB() - estimatedRotation.getB(), 2.0) +
                    Math.pow(rotation.getC() - estimatedRotation.getC(), 2.0) +
                    Math.pow(rotation.getD() - estimatedRotation.getD(), 2.0);
            
            if (diffEstimatedNoSuggestion >= diffEstimated) {
                numValid++;
            }            
            
            if (numCovariances > 0 && numValid > 0) {
                break;
            }
        }
        
        assertTrue(numCovariances > 0);
        assertTrue(numValid > 0);
    }
    
    @Test
    public void testEstimateGeneralSuggestedRotationWithFastRefinement() 
            throws LockedException, NotReadyException, CameraException,
            NotAvailableException {
        int numCovariances = 0;
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            
            //intrinsic parameters
            double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                    verticalFocalLength, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, skewness);
            
            //create rotation parameters
            double roll = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double yaw = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            Quaternion rotation = new Quaternion(roll, pitch, yaw);
            rotation.normalize();
            
            //create camera center
            double[] cameraCenterArray = new double[3];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);
            
            //normalize the camera to improve accuracy
            camera.normalize();  
            
            int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);            
            List<Point3D> points3D = new ArrayList<>(nPoints);
            Point3D point3D;
            for (int i = 0; i < nPoints; i++) {
                point3D = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                points3D.add(point3D);
            }
            
            List<Point2D> points2D = camera.project(points3D);  
            
            //create outliers
            Point2D point2DWithError;
            List<Point2D> points2DWithError = new ArrayList<>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, OUTLIER_STD_ERROR); 
            for (Point2D point2D : points2D) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //point is oulier
                    double errorX = errorRandomizer.nextDouble();
                    double errorY = errorRandomizer.nextDouble();
                    double errorW = errorRandomizer.nextDouble();
                    point2DWithError = new HomogeneousPoint2D(
                            point2D.getHomX() + errorX, 
                            point2D.getHomY() + errorY, 
                            point2D.getHomW() + errorW);
                } else {
                    //inlier point (without error)
                    point2DWithError = point2D;
                }
                
                points2DWithError.add(point2DWithError);
            }
            
            RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                    new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                    this, intrinsic, points3D, points2DWithError);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);            
            estimator.setSuggestRotationEnabled(true);
            estimator.setSuggestedRotationValue(rotation);   
            estimator.setResultRefined(true);
            estimator.setFastRefinementUsed(true);
            estimator.setCovarianceKept(true);            
            
            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getCovariance());
                        
            PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }

            assertFalse(estimator.isLocked());
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            if (estimator.getCovariance() != null) {
                numCovariances++;
            }            
            reset();
            
            //decompone estimated camera and check its parameters
            estimatedCamera.decompose();
            
            //comparing rotation
            Quaternion estimatedRotation = estimatedCamera.getCameraRotation().
                    toQuaternion();
            estimatedRotation.normalize();
            
            //estimate without suggestion
            estimator.setSuggestRotationEnabled(false);
            
            PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }         
            
            estimatedCameraNoSuggestion.decompose();

            Quaternion estimatedRotationNoSuggestion = 
                    estimatedCameraNoSuggestion.getCameraRotation().
                            toQuaternion();
            
            //check that rotation has become closer to suggested value
            double diffEstimatedNoSuggestion = 
                    Math.pow(rotation.getA() - 
                            estimatedRotationNoSuggestion.getA(), 2.0) +
                    Math.pow(rotation.getB() - 
                            estimatedRotationNoSuggestion.getB(), 2.0) +
                    Math.pow(rotation.getC() - 
                            estimatedRotationNoSuggestion.getC(), 2.0) +
                    Math.pow(rotation.getD() - 
                            estimatedRotationNoSuggestion.getD(), 2.0);
            double diffEstimated =
                    Math.pow(rotation.getA() - estimatedRotation.getA(), 2.0) +
                    Math.pow(rotation.getB() - estimatedRotation.getB(), 2.0) +
                    Math.pow(rotation.getC() - estimatedRotation.getC(), 2.0) +
                    Math.pow(rotation.getD() - estimatedRotation.getD(), 2.0);
            
            if (diffEstimatedNoSuggestion >= diffEstimated) {
                numValid++;
            }            
            
            if (numCovariances > 0 && numValid > 0) {
                break;
            }
        }
        
        assertTrue(numCovariances > 0);
        assertTrue(numValid > 0);
    }
    
    @Test
    public void testEstimateGeneralSuggestedCenterEnabled() 
            throws LockedException, NotReadyException, CameraException,
            NotAvailableException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            
            //intrinsic parameters
            double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                    verticalFocalLength, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, skewness);
            
            //create rotation parameters
            double roll = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double yaw = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            Quaternion rotation = new Quaternion(roll, pitch, yaw);
            rotation.normalize();
            
            //create camera center
            double[] cameraCenterArray = new double[3];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);
            
            //normalize the camera to improve accuracy
            camera.normalize();  
            
            int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);            
            List<Point3D> points3D = new ArrayList<>(nPoints);
            Point3D point3D;
            for (int i = 0; i < nPoints; i++) {
                point3D = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                points3D.add(point3D);
            }
            
            List<Point2D> points2D = camera.project(points3D);  
            
            //create outliers
            Point2D point2DWithError;
            List<Point2D> points2DWithError = new ArrayList<>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, OUTLIER_STD_ERROR); 
            for (Point2D point2D : points2D) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //point is oulier
                    double errorX = errorRandomizer.nextDouble();
                    double errorY = errorRandomizer.nextDouble();
                    double errorW = errorRandomizer.nextDouble();
                    point2DWithError = new HomogeneousPoint2D(
                            point2D.getHomX() + errorX, 
                            point2D.getHomY() + errorY, 
                            point2D.getHomW() + errorW);
                } else {
                    //inlier point (without error)
                    point2DWithError = point2D;
                }
                
                points2DWithError.add(point2DWithError);
            }
            
            RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                    new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                    this, intrinsic, points3D, points2DWithError);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);            
            estimator.setSuggestCenterEnabled(true);
            estimator.setSuggestedCenterValue(cameraCenter);
            
            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getCovariance());
            
            
            PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }

            assertFalse(estimator.isLocked());
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();
            
            //decompone estimated camera and check its parameters
            estimatedCamera.decompose();
            
            //comparing center
            Point3D estimatedCenter = estimatedCamera.getCameraCenter();
            
            //estimate without suggestion
            estimator.setSuggestCenterEnabled(false);
            
            PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }         
            
            estimatedCameraNoSuggestion.decompose();

            Point3D estimatedCenterNoSuggestion = 
                    estimatedCameraNoSuggestion.getCameraCenter();
            
            //check that camera center has become closer to suggested value
            if (cameraCenter.distanceTo(estimatedCenterNoSuggestion) >=
                    cameraCenter.distanceTo(estimatedCenter)) {
                numValid++;
            }
            
            if (numValid > 0) {
                break;
            }
        }
        
        assertTrue(numValid > 0);
    }
    
    @Test
    public void testEstimateGeneralSuggestedCenterWithRefinement() 
            throws LockedException, NotReadyException, CameraException,
            NotAvailableException {
        int numCovariances = 0;
        int numValid = 0;
        for (int t = 0; t < 5*TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            
            //intrinsic parameters
            double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                    verticalFocalLength, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, skewness);
            
            //create rotation parameters
            double roll = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double yaw = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            Quaternion rotation = new Quaternion(roll, pitch, yaw);
            rotation.normalize();
            
            //create camera center
            double[] cameraCenterArray = new double[3];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);
            
            //normalize the camera to improve accuracy
            camera.normalize();  
            
            int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);            
            List<Point3D> points3D = new ArrayList<>(nPoints);
            Point3D point3D;
            for (int i = 0; i < nPoints; i++) {
                point3D = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                points3D.add(point3D);
            }
            
            List<Point2D> points2D = camera.project(points3D);  
            
            //create outliers
            Point2D point2DWithError;
            List<Point2D> points2DWithError = new ArrayList<>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, OUTLIER_STD_ERROR); 
            for (Point2D point2D : points2D) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //point is oulier
                    double errorX = errorRandomizer.nextDouble();
                    double errorY = errorRandomizer.nextDouble();
                    double errorW = errorRandomizer.nextDouble();
                    point2DWithError = new HomogeneousPoint2D(
                            point2D.getHomX() + errorX, 
                            point2D.getHomY() + errorY, 
                            point2D.getHomW() + errorW);
                } else {
                    //inlier point (without error)
                    point2DWithError = point2D;
                }
                
                points2DWithError.add(point2DWithError);
            }
            
            RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                    new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                    this, intrinsic, points3D, points2DWithError);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);            
            estimator.setSuggestCenterEnabled(true);
            estimator.setSuggestedCenterValue(cameraCenter);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);            
            
            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getCovariance());
            
            
            PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }

            assertFalse(estimator.isLocked());
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            if (estimator.getCovariance() != null) {
                numCovariances++;
            }            
            reset();
            
            //decompone estimated camera and check its parameters
            estimatedCamera.decompose();
            
            //comparing center
            Point3D estimatedCenter = estimatedCamera.getCameraCenter();
            
            //estimate without suggestion
            estimator.setSuggestCenterEnabled(false);
            
            PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }         
            
            estimatedCameraNoSuggestion.decompose();

            Point3D estimatedCenterNoSuggestion = 
                    estimatedCameraNoSuggestion.getCameraCenter();
            
            //check that camera center has become closer to suggested value
            if (cameraCenter.distanceTo(estimatedCenterNoSuggestion) >=
                    cameraCenter.distanceTo(estimatedCenter)) {
                numValid++;
            }
            
            if (numCovariances > 0 && numValid > 0) {
                break;
            }
        }
        
        assertTrue(numCovariances > 0);
        assertTrue(numValid > 0);
    }
    
    @Test
    public void testEstimateGeneralSuggestedCenterWithFastRefinement() 
            throws LockedException, NotReadyException, CameraException,
            NotAvailableException {
        int numCovariances = 0;
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            
            //intrinsic parameters
            double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                    verticalFocalLength, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, skewness);
            
            //create rotation parameters
            double roll = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double yaw = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            Quaternion rotation = new Quaternion(roll, pitch, yaw);
            rotation.normalize();
            
            //create camera center
            double[] cameraCenterArray = new double[3];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);
            
            //normalize the camera to improve accuracy
            camera.normalize();  
            
            int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);            
            List<Point3D> points3D = new ArrayList<>(nPoints);
            Point3D point3D;
            for (int i = 0; i < nPoints; i++) {
                point3D = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                points3D.add(point3D);
            }
            
            List<Point2D> points2D = camera.project(points3D);  
            
            //create outliers
            Point2D point2DWithError;
            List<Point2D> points2DWithError = new ArrayList<>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, OUTLIER_STD_ERROR); 
            for (Point2D point2D : points2D) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //point is oulier
                    double errorX = errorRandomizer.nextDouble();
                    double errorY = errorRandomizer.nextDouble();
                    double errorW = errorRandomizer.nextDouble();
                    point2DWithError = new HomogeneousPoint2D(
                            point2D.getHomX() + errorX, 
                            point2D.getHomY() + errorY, 
                            point2D.getHomW() + errorW);
                } else {
                    //inlier point (without error)
                    point2DWithError = point2D;
                }
                
                points2DWithError.add(point2DWithError);
            }
            
            RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                    new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                    this, intrinsic, points3D, points2DWithError);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);            
            estimator.setSuggestCenterEnabled(true);
            estimator.setSuggestedCenterValue(cameraCenter);
            estimator.setResultRefined(true);
            estimator.setFastRefinementUsed(true);
            estimator.setCovarianceKept(true);            
            
            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getCovariance());
            
            
            PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }

            assertFalse(estimator.isLocked());
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            if (estimator.getCovariance() != null) {
                numCovariances++;
            }            
            reset();
            
            //decompone estimated camera and check its parameters
            estimatedCamera.decompose();
            
            //comparing center
            Point3D estimatedCenter = estimatedCamera.getCameraCenter();
            
            //estimate without suggestion
            estimator.setSuggestCenterEnabled(false);
            
            PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }         
            
            estimatedCameraNoSuggestion.decompose();

            Point3D estimatedCenterNoSuggestion = 
                    estimatedCameraNoSuggestion.getCameraCenter();
            
            //check that camera center has become closer to suggested value
            if (cameraCenter.distanceTo(estimatedCenterNoSuggestion) >=
                    cameraCenter.distanceTo(estimatedCenter)) {
                numValid++;
            }
            
            if (numCovariances > 0 && numValid > 0) {
                break;
            }
        }
        
        assertTrue(numCovariances > 0);
        assertTrue(numValid > 0);
    }
    
    @Test
    public void testEstimatePlanarNoSuggestion() 
            throws LockedException, NotReadyException, CameraException,
            NotAvailableException {
        int numValidCameras = 0;
        int numValidProjections = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            
            //intrinsic parameters
            double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                    verticalFocalLength, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, skewness);
            
            //create rotation parameters
            double roll = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double yaw = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            Quaternion rotation = new Quaternion(roll, pitch, yaw);
            rotation.normalize();
            
            //create camera center
            double[] cameraCenterArray = new double[3];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);
            
            //normalize the camera to improve accuracy
            camera.normalize();  
            
            //generate plane parallel to the camera's principal plane and
            //located at certain distance from it.
            double[] principalAxis = camera.getPrincipalAxisArray();
            double depth = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            Point3D centroid = new InhomogeneousPoint3D(
                    cameraCenter.getInhomX() + depth * principalAxis[0],
                    cameraCenter.getInhomY() + depth * principalAxis[1],
                    cameraCenter.getInhomZ() + depth * principalAxis[2]);
            assertTrue(camera.isPointInFrontOfCamera(centroid));
            
            Plane plane = new Plane(centroid, principalAxis);            
            
            int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);            
            
            double a = plane.getA();
            double b = plane.getB();
            double c = plane.getC();
            double d = plane.getD();
            
            List<Point3D> points3D = new ArrayList<>(nPoints);
            Point3D point3D;
            for (int i = 0; i < nPoints; i++) {
                //get a random point belonging to the plane
                //a*x + b*y + c*z + d*w = 0
                //y = -(a*x + c*z + d*w)/b or x = -(b*y + c*z + d*w)/a
                double homX, homY;
                double homW = 1.0;
                double homZ = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE);                
                if(Math.abs(b) > ABSOLUTE_ERROR){
                    homX = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE);
                    homY = -(a * homX + c * homZ + d * homW) / b;
                }else{
                    homY = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE);
                    homX = -(b * homY + c * homZ + d * homW) / a;
                }
                
                point3D = new HomogeneousPoint3D(homX, homY, homZ, homW);
                
                assertTrue(plane.isLocus(point3D));
                assertTrue(camera.isPointInFrontOfCamera(point3D));
                
                points3D.add(point3D);
            }
            
            List<Point2D> points2D = camera.project(points3D);  
            
            //create outliers
            Point2D point2DWithError;
            List<Point2D> points2DWithError = new ArrayList<>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, OUTLIER_STD_ERROR); 
            for (Point2D point2D : points2D) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //point is oulier
                    double errorX = errorRandomizer.nextDouble();
                    double errorY = errorRandomizer.nextDouble();
                    double errorW = errorRandomizer.nextDouble();
                    point2DWithError = new HomogeneousPoint2D(
                            point2D.getHomX() + errorX, 
                            point2D.getHomY() + errorY, 
                            point2D.getHomW() + errorW);
                } else {
                    //inlier point (without error)
                    point2DWithError = point2D;
                }
                
                points2DWithError.add(point2DWithError);
            }
            
            RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                    new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                    this, intrinsic, points3D, points2DWithError);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);            
            estimator.setResultRefined(false);
            estimator.setCovarianceKept(false);
            
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getCovariance());

            reset();
            
            PinholeCamera camera2;
            try {
                camera2 = estimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }
            
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            assertNull(estimator.getCovariance());
            reset();

            //check correctness of estimation
                        
            //decompose estimated camera and check its parameters
            camera2.decompose();
            
            //Comparing camera intrinsic parameters
            PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    camera2.getIntrinsicParameters();

            assertEquals(horizontalFocalLength, 
                    estimatedIntrinsic.getHorizontalFocalLength(), 
                    ABSOLUTE_ERROR);
            assertEquals(verticalFocalLength,
                    estimatedIntrinsic.getVerticalFocalLength(), 
                    ABSOLUTE_ERROR);
            assertEquals(horizontalPrincipalPoint,
                    estimatedIntrinsic.getHorizontalPrincipalPoint(), 
                    ABSOLUTE_ERROR);
            assertEquals(verticalPrincipalPoint,
                    estimatedIntrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(skewness, estimatedIntrinsic.getSkewness(), 
                    ABSOLUTE_ERROR);
            
            //comparing estimated camera center
            Point3D estimatedCameraCenter = camera2.getCameraCenter();
            if(!cameraCenter.equals(estimatedCameraCenter, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(cameraCenter.equals(estimatedCameraCenter, 
                    ABSOLUTE_ERROR));            
            
            //comparing estimated rotation
            Quaternion estimatedRotation = camera2.getCameraRotation().
                    toQuaternion();
            estimatedRotation.normalize();
            
            Matrix rotMatrix = rotation.asInhomogeneousMatrix();
            Matrix estimatedRotMatrix = estimatedRotation.
                    asInhomogeneousMatrix();
            
            assertEquals(rotation.getA(), estimatedRotation.getA(), 
                    ABSOLUTE_ERROR);
            assertEquals(rotation.getB(), estimatedRotation.getB(), 
                    ABSOLUTE_ERROR);
            assertEquals(rotation.getC(), estimatedRotation.getC(), 
                    ABSOLUTE_ERROR);
            assertEquals(rotation.getD(), estimatedRotation.getD(), 
                    ABSOLUTE_ERROR);  
            
            assertTrue(rotMatrix.equals(estimatedRotMatrix, ABSOLUTE_ERROR));
            
            numValidCameras++;

            //project original 3D points using estimated camera and check
            //distance to 2D points without error
            Point2D originalPoint2D, estimatedPoint2D;
            boolean isValid = true;
            for (int i = 0; i < nPoints; i++) {
                point3D = points3D.get(i);
                originalPoint2D = points2D.get(i);
                estimatedPoint2D = camera2.project(point3D);

                if(originalPoint2D.distanceTo(estimatedPoint2D) > ABSOLUTE_ERROR) {
                    isValid = false;
                    break;
                }
                assertEquals(originalPoint2D.distanceTo(estimatedPoint2D), 
                        0.0, ABSOLUTE_ERROR);
            }                   
            
            if(isValid) {
                numValidProjections++;  
            }
            
            if (numValidCameras > 0 && numValidProjections > 0) {
                break;
            }
        }
        
        assertTrue(numValidCameras > 0);
        assertTrue(numValidProjections > 0);
    }
    
    @Test
    public void testEstimatePlanarNoSuggestionWithRefinement() 
            throws LockedException, NotReadyException, CameraException,
            NotAvailableException {
        int numValidCameras = 0;
        int numValidProjections = 0;
        int numCovariances = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            
            //intrinsic parameters
            double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                    verticalFocalLength, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, skewness);
            
            //create rotation parameters
            double roll = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double yaw = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            Quaternion rotation = new Quaternion(roll, pitch, yaw);
            rotation.normalize();
            
            //create camera center
            double[] cameraCenterArray = new double[3];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);
            
            //normalize the camera to improve accuracy
            camera.normalize();  
            
            //generate plane parallel to the camera's principal plane and
            //located at certain distance from it.
            double[] principalAxis = camera.getPrincipalAxisArray();
            double depth = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            Point3D centroid = new InhomogeneousPoint3D(
                    cameraCenter.getInhomX() + depth * principalAxis[0],
                    cameraCenter.getInhomY() + depth * principalAxis[1],
                    cameraCenter.getInhomZ() + depth * principalAxis[2]);
            assertTrue(camera.isPointInFrontOfCamera(centroid));
            
            Plane plane = new Plane(centroid, principalAxis);            
            
            int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);            
            
            double a = plane.getA();
            double b = plane.getB();
            double c = plane.getC();
            double d = plane.getD();
            
            List<Point3D> points3D = new ArrayList<>(nPoints);
            Point3D point3D;
            for (int i = 0; i < nPoints; i++) {
                //get a random point belonging to the plane
                //a*x + b*y + c*z + d*w = 0
                //y = -(a*x + c*z + d*w)/b or x = -(b*y + c*z + d*w)/a
                double homX, homY;
                double homW = 1.0;
                double homZ = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE);                
                if(Math.abs(b) > ABSOLUTE_ERROR){
                    homX = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE);
                    homY = -(a * homX + c * homZ + d * homW) / b;
                }else{
                    homY = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE);
                    homX = -(b * homY + c * homZ + d * homW) / a;
                }
                
                point3D = new HomogeneousPoint3D(homX, homY, homZ, homW);
                
                assertTrue(plane.isLocus(point3D));
                assertTrue(camera.isPointInFrontOfCamera(point3D));

                points3D.add(point3D);
            }
            
            List<Point2D> points2D = camera.project(points3D);  
            
            //create outliers
            Point2D point2DWithError;
            List<Point2D> points2DWithError = new ArrayList<>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, OUTLIER_STD_ERROR); 
            for (Point2D point2D : points2D) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //point is oulier
                    double errorX = errorRandomizer.nextDouble();
                    double errorY = errorRandomizer.nextDouble();
                    double errorW = errorRandomizer.nextDouble();
                    point2DWithError = new HomogeneousPoint2D(
                            point2D.getHomX() + errorX, 
                            point2D.getHomY() + errorY, 
                            point2D.getHomW() + errorW);
                } else {
                    //inlier point (without error)
                    point2DWithError = point2D;
                }
                
                points2DWithError.add(point2DWithError);
            }
            
            RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                    new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                    this, intrinsic, points3D, points2DWithError);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);            
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);
            
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getCovariance());

            reset();
            
            PinholeCamera camera2;
            try {
                camera2 = estimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }
            
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            if (estimator.getCovariance() != null) {
                numCovariances++;
            }
            reset();

            //check correctness of estimation
                        
            //decompose estimated camera and check its parameters
            camera2.decompose();
            
            //Comparing camera intrinsic parameters
            PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    camera2.getIntrinsicParameters();

            assertEquals(horizontalFocalLength, 
                    estimatedIntrinsic.getHorizontalFocalLength(), 
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(verticalFocalLength,
                    estimatedIntrinsic.getVerticalFocalLength(), 
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(horizontalPrincipalPoint,
                    estimatedIntrinsic.getHorizontalPrincipalPoint(), 
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(verticalPrincipalPoint,
                    estimatedIntrinsic.getVerticalPrincipalPoint(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(skewness, estimatedIntrinsic.getSkewness(), 
                    LARGE_ABSOLUTE_ERROR);
            
            //comparing estimated camera center
            Point3D estimatedCameraCenter = camera2.getCameraCenter();
            if(!cameraCenter.equals(estimatedCameraCenter, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(cameraCenter.equals(estimatedCameraCenter, 
                    ABSOLUTE_ERROR));            
            
            //comparing estimated rotation
            Quaternion estimatedRotation = camera2.getCameraRotation().
                    toQuaternion();
            estimatedRotation.normalize();
            
            Matrix rotMatrix = rotation.asInhomogeneousMatrix();
            Matrix estimatedRotMatrix = estimatedRotation.
                    asInhomogeneousMatrix();
            
            assertEquals(rotation.getA(), estimatedRotation.getA(), 
                    ABSOLUTE_ERROR);
            assertEquals(rotation.getB(), estimatedRotation.getB(), 
                    ABSOLUTE_ERROR);
            assertEquals(rotation.getC(), estimatedRotation.getC(), 
                    ABSOLUTE_ERROR);
            assertEquals(rotation.getD(), estimatedRotation.getD(), 
                    ABSOLUTE_ERROR);  
            
            assertTrue(rotMatrix.equals(estimatedRotMatrix, ABSOLUTE_ERROR));
            
            numValidCameras++;

            //project original 3D points using estimated camera and check
            //distance to 2D points without error
            Point2D originalPoint2D, estimatedPoint2D;
            boolean isValid = true;
            for (int i = 0; i < nPoints; i++) {
                point3D = points3D.get(i);
                originalPoint2D = points2D.get(i);
                estimatedPoint2D = camera2.project(point3D);

                if(originalPoint2D.distanceTo(estimatedPoint2D) > LARGE_ABSOLUTE_ERROR) {
                    isValid = false;
                    break;
                }
                assertEquals(originalPoint2D.distanceTo(estimatedPoint2D), 
                        0.0, LARGE_ABSOLUTE_ERROR);
            }                   
            
            if(isValid) {
                numValidProjections++;  
            }
            
            if (numValidCameras > 0 && numValidProjections > 0 && 
                    numCovariances > 0) {
                break;
            }
        }
        
        assertTrue(numValidCameras > 0);
        assertTrue(numValidProjections > 0);
        assertTrue(numCovariances > 0);
    }
    
    @Test
    public void testEstimatePlanarNoSuggestionWithFastRefinement() 
            throws LockedException, NotReadyException, CameraException,
            NotAvailableException {
        int numValidCameras = 0;
        int numValidProjections = 0;
        int numCovariances = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            
            //intrinsic parameters
            double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                    verticalFocalLength, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, skewness);
            
            //create rotation parameters
            double roll = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double yaw = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            Quaternion rotation = new Quaternion(roll, pitch, yaw);
            rotation.normalize();
            
            //create camera center
            double[] cameraCenterArray = new double[3];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);
            
            //normalize the camera to improve accuracy
            camera.normalize();  
            
            //generate plane parallel to the camera's principal plane and
            //located at certain distance from it.
            double[] principalAxis = camera.getPrincipalAxisArray();
            double depth = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            Point3D centroid = new InhomogeneousPoint3D(
                    cameraCenter.getInhomX() + depth * principalAxis[0],
                    cameraCenter.getInhomY() + depth * principalAxis[1],
                    cameraCenter.getInhomZ() + depth * principalAxis[2]);
            assertTrue(camera.isPointInFrontOfCamera(centroid));
            
            Plane plane = new Plane(centroid, principalAxis);            
            
            int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);            
            
            double a = plane.getA();
            double b = plane.getB();
            double c = plane.getC();
            double d = plane.getD();
            
            List<Point3D> points3D = new ArrayList<>(nPoints);
            Point3D point3D;
            for (int i = 0; i < nPoints; i++) {
                //get a random point belonging to the plane
                //a*x + b*y + c*z + d*w = 0
                //y = -(a*x + c*z + d*w)/b or x = -(b*y + c*z + d*w)/a
                double homX, homY;
                double homW = 1.0;
                double homZ = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE);                
                if(Math.abs(b) > ABSOLUTE_ERROR){
                    homX = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE);
                    homY = -(a * homX + c * homZ + d * homW) / b;
                }else{
                    homY = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE);
                    homX = -(b * homY + c * homZ + d * homW) / a;
                }
                
                point3D = new HomogeneousPoint3D(homX, homY, homZ, homW);
                
                assertTrue(plane.isLocus(point3D));
                assertTrue(camera.isPointInFrontOfCamera(point3D));

                points3D.add(point3D);
            }
            
            List<Point2D> points2D = camera.project(points3D);  
            
            //create outliers
            Point2D point2DWithError;
            List<Point2D> points2DWithError = new ArrayList<>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, OUTLIER_STD_ERROR); 
            for (Point2D point2D : points2D) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //point is oulier
                    double errorX = errorRandomizer.nextDouble();
                    double errorY = errorRandomizer.nextDouble();
                    double errorW = errorRandomizer.nextDouble();
                    point2DWithError = new HomogeneousPoint2D(
                            point2D.getHomX() + errorX, 
                            point2D.getHomY() + errorY, 
                            point2D.getHomW() + errorW);
                } else {
                    //inlier point (without error)
                    point2DWithError = point2D;
                }
                
                points2DWithError.add(point2DWithError);
            }
            
            RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                    new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                    this, intrinsic, points3D, points2DWithError);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);            
            estimator.setResultRefined(true);
            estimator.setFastRefinementUsed(true);
            estimator.setCovarianceKept(true);
            
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getCovariance());

            reset();
            
            PinholeCamera camera2;
            try {
                camera2 = estimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }
            
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            if (estimator.getCovariance() != null) {
                numCovariances++;
            }
            reset();

            //check correctness of estimation
                        
            //decompose estimated camera and check its parameters
            camera2.decompose();
            
            //Comparing camera intrinsic parameters
            PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    camera2.getIntrinsicParameters();

            assertEquals(horizontalFocalLength, 
                    estimatedIntrinsic.getHorizontalFocalLength(), 
                    10*LARGE_ABSOLUTE_ERROR);
            assertEquals(verticalFocalLength,
                    estimatedIntrinsic.getVerticalFocalLength(), 
                    10*LARGE_ABSOLUTE_ERROR);
            assertEquals(horizontalPrincipalPoint,
                    estimatedIntrinsic.getHorizontalPrincipalPoint(), 
                    10*LARGE_ABSOLUTE_ERROR);
            assertEquals(verticalPrincipalPoint,
                    estimatedIntrinsic.getVerticalPrincipalPoint(),
                    10*LARGE_ABSOLUTE_ERROR);
            assertEquals(skewness, estimatedIntrinsic.getSkewness(), 
                    10*LARGE_ABSOLUTE_ERROR);
            
            //comparing estimated camera center
            Point3D estimatedCameraCenter = camera2.getCameraCenter();
            if(!cameraCenter.equals(estimatedCameraCenter, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(cameraCenter.equals(estimatedCameraCenter, 
                    ABSOLUTE_ERROR));            
            
            //comparing estimated rotation
            Quaternion estimatedRotation = camera2.getCameraRotation().
                    toQuaternion();
            estimatedRotation.normalize();
            
            Matrix rotMatrix = rotation.asInhomogeneousMatrix();
            Matrix estimatedRotMatrix = estimatedRotation.
                    asInhomogeneousMatrix();
            
            assertEquals(rotation.getA(), estimatedRotation.getA(), 
                    ABSOLUTE_ERROR);
            assertEquals(rotation.getB(), estimatedRotation.getB(), 
                    ABSOLUTE_ERROR);
            assertEquals(rotation.getC(), estimatedRotation.getC(), 
                    ABSOLUTE_ERROR);
            assertEquals(rotation.getD(), estimatedRotation.getD(), 
                    ABSOLUTE_ERROR);  
            
            assertTrue(rotMatrix.equals(estimatedRotMatrix, ABSOLUTE_ERROR));
            
            numValidCameras++;

            //project original 3D points using estimated camera and check
            //distance to 2D points without error
            Point2D originalPoint2D, estimatedPoint2D;
            boolean isValid = true;
            for (int i = 0; i < nPoints; i++) {
                point3D = points3D.get(i);
                originalPoint2D = points2D.get(i);
                estimatedPoint2D = camera2.project(point3D);

                if(originalPoint2D.distanceTo(estimatedPoint2D) > LARGE_ABSOLUTE_ERROR) {
                    isValid = false;
                    break;
                }
                assertEquals(originalPoint2D.distanceTo(estimatedPoint2D), 
                        0.0, LARGE_ABSOLUTE_ERROR);
            }                   
            
            if(isValid) {
                numValidProjections++;  
            }
            
            if (numValidCameras > 0 && numValidProjections > 0 &&
                    numCovariances > 0) {
                break;
            }
        }
        
        assertTrue(numValidCameras > 0);
        assertTrue(numValidProjections > 0);
        assertTrue(numCovariances > 0);
    }
    
    @Test
    public void testEstimatePlanarSuggestedRotationEnabled() 
            throws LockedException, NotReadyException, CameraException,
            NotAvailableException {
        int numValid = 0;
        for (int t = 0; t < 5*TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            
            //intrinsic parameters
            double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                    verticalFocalLength, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, skewness);
            
            //create rotation parameters
            double roll = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double yaw = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            Quaternion rotation = new Quaternion(roll, pitch, yaw);
            rotation.normalize();
            
            //create camera center
            double[] cameraCenterArray = new double[3];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);
            
            //normalize the camera to improve accuracy
            camera.normalize();  
            
            //generate plane parallel to the camera's principal plane and
            //located at certain distance from it.
            double[] principalAxis = camera.getPrincipalAxisArray();
            double depth = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            Point3D centroid = new InhomogeneousPoint3D(
                    cameraCenter.getInhomX() + depth * principalAxis[0],
                    cameraCenter.getInhomY() + depth * principalAxis[1],
                    cameraCenter.getInhomZ() + depth * principalAxis[2]);
            assertTrue(camera.isPointInFrontOfCamera(centroid));
            
            Plane plane = new Plane(centroid, principalAxis);            
            
            int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);            
            
            double a = plane.getA();
            double b = plane.getB();
            double c = plane.getC();
            double d = plane.getD();
            
            List<Point3D> points3D = new ArrayList<>(nPoints);
            Point3D point3D;
            for (int i = 0; i < nPoints; i++) {
                //get a random point belonging to the plane
                //a*x + b*y + c*z + d*w = 0
                //y = -(a*x + c*z + d*w)/b or x = -(b*y + c*z + d*w)/a
                double homX, homY;
                double homW = 1.0;
                double homZ = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE);                
                if(Math.abs(b) > ABSOLUTE_ERROR){
                    homX = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE);
                    homY = -(a * homX + c * homZ + d * homW) / b;
                }else{
                    homY = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE);
                    homX = -(b * homY + c * homZ + d * homW) / a;
                }
                
                point3D = new HomogeneousPoint3D(homX, homY, homZ, homW);
                
                assertTrue(plane.isLocus(point3D));
                assertTrue(camera.isPointInFrontOfCamera(point3D));
                
                points3D.add(point3D);
            }
            
            List<Point2D> points2D = camera.project(points3D);  
            
            //create outliers
            Point2D point2DWithError;
            List<Point2D> points2DWithError = new ArrayList<>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, OUTLIER_STD_ERROR); 
            for (Point2D point2D : points2D) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //point is oulier
                    double errorX = errorRandomizer.nextDouble();
                    double errorY = errorRandomizer.nextDouble();
                    double errorW = errorRandomizer.nextDouble();
                    point2DWithError = new HomogeneousPoint2D(
                            point2D.getHomX() + errorX, 
                            point2D.getHomY() + errorY, 
                            point2D.getHomW() + errorW);
                } else {
                    //inlier point (without error)
                    point2DWithError = point2D;
                }
                
                points2DWithError.add(point2DWithError);
            }
            
            RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                    new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                    this, intrinsic, points3D, points2DWithError);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);            
            estimator.setSuggestRotationEnabled(true);
            estimator.setSuggestedRotationValue(rotation);            
            
            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getCovariance());
            
            
            PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }
            
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            //decompone estimated camera and check its parameters
            estimatedCamera.decompose();
            
            //comparing rotation
            Quaternion estimatedRotation = estimatedCamera.getCameraRotation().
                    toQuaternion();
            estimatedRotation.normalize();
            
            //estimate without suggestion
            estimator.setSuggestRotationEnabled(false);
            
            PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }         
            
            estimatedCameraNoSuggestion.decompose();

            Quaternion estimatedRotationNoSuggestion = 
                    estimatedCameraNoSuggestion.getCameraRotation().
                            toQuaternion();
            
            //check that rotation has become closer to suggested value
            double diffEstimatedNoSuggestion = 
                    Math.pow(rotation.getA() - 
                            estimatedRotationNoSuggestion.getA(), 2.0) +
                    Math.pow(rotation.getB() - 
                            estimatedRotationNoSuggestion.getB(), 2.0) +
                    Math.pow(rotation.getC() - 
                            estimatedRotationNoSuggestion.getC(), 2.0) +
                    Math.pow(rotation.getD() - 
                            estimatedRotationNoSuggestion.getD(), 2.0);
            double diffEstimated =
                    Math.pow(rotation.getA() - estimatedRotation.getA(), 2.0) +
                    Math.pow(rotation.getB() - estimatedRotation.getB(), 2.0) +
                    Math.pow(rotation.getC() - estimatedRotation.getC(), 2.0) +
                    Math.pow(rotation.getD() - estimatedRotation.getD(), 2.0);
            
            if (diffEstimatedNoSuggestion >= diffEstimated) {
                numValid++;
            }        
            
            if (numValid > 0) {
                break;
            }
        }
        
        assertTrue(numValid > 0);
    }
    
    @Test
    public void testEstimatePlanarSuggestedRotationWithRefinement() 
            throws LockedException, NotReadyException, CameraException,
            NotAvailableException {
        int numCovariances = 0;
        int numValid = 0;
        for (int t = 0; t < 5*TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            
            //intrinsic parameters
            double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                    verticalFocalLength, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, skewness);
            
            //create rotation parameters
            double roll = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double yaw = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            Quaternion rotation = new Quaternion(roll, pitch, yaw);
            rotation.normalize();
            
            //create camera center
            double[] cameraCenterArray = new double[3];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);
            
            //normalize the camera to improve accuracy
            camera.normalize();  
            
            //generate plane parallel to the camera's principal plane and
            //located at certain distance from it.
            double[] principalAxis = camera.getPrincipalAxisArray();
            double depth = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            Point3D centroid = new InhomogeneousPoint3D(
                    cameraCenter.getInhomX() + depth * principalAxis[0],
                    cameraCenter.getInhomY() + depth * principalAxis[1],
                    cameraCenter.getInhomZ() + depth * principalAxis[2]);
            assertTrue(camera.isPointInFrontOfCamera(centroid));
            
            Plane plane = new Plane(centroid, principalAxis);            
            
            int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);            
            
            double a = plane.getA();
            double b = plane.getB();
            double c = plane.getC();
            double d = plane.getD();
            
            List<Point3D> points3D = new ArrayList<>(nPoints);
            Point3D point3D;
            for (int i = 0; i < nPoints; i++) {
                //get a random point belonging to the plane
                //a*x + b*y + c*z + d*w = 0
                //y = -(a*x + c*z + d*w)/b or x = -(b*y + c*z + d*w)/a
                double homX, homY;
                double homW = 1.0;
                double homZ = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE);                
                if(Math.abs(b) > ABSOLUTE_ERROR){
                    homX = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE);
                    homY = -(a * homX + c * homZ + d * homW) / b;
                }else{
                    homY = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE);
                    homX = -(b * homY + c * homZ + d * homW) / a;
                }
                
                point3D = new HomogeneousPoint3D(homX, homY, homZ, homW);
                
                assertTrue(plane.isLocus(point3D));
                assertTrue(camera.isPointInFrontOfCamera(point3D));
                
                points3D.add(point3D);
            }
            
            List<Point2D> points2D = camera.project(points3D);  
            
            //create outliers
            Point2D point2DWithError;
            List<Point2D> points2DWithError = new ArrayList<>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, OUTLIER_STD_ERROR); 
            for (Point2D point2D : points2D) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //point is oulier
                    double errorX = errorRandomizer.nextDouble();
                    double errorY = errorRandomizer.nextDouble();
                    double errorW = errorRandomizer.nextDouble();
                    point2DWithError = new HomogeneousPoint2D(
                            point2D.getHomX() + errorX, 
                            point2D.getHomY() + errorY, 
                            point2D.getHomW() + errorW);
                } else {
                    //inlier point (without error)
                    point2DWithError = point2D;
                }
                
                points2DWithError.add(point2DWithError);
            }
            
            RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                    new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                    this, intrinsic, points3D, points2DWithError);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);            
            estimator.setSuggestRotationEnabled(true);
            estimator.setSuggestedRotationValue(rotation);      
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);            

            
            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getCovariance());
            
            
            PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }
            
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            if (estimator.getCovariance() != null) {
                numCovariances++;
            }                        
            reset();

            //decompone estimated camera and check its parameters
            estimatedCamera.decompose();
            
            //comparing rotation
            Quaternion estimatedRotation = estimatedCamera.getCameraRotation().
                    toQuaternion();
            estimatedRotation.normalize();
            
            //estimate without suggestion
            estimator.setSuggestRotationEnabled(false);
            
            PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }         
            
            estimatedCameraNoSuggestion.decompose();

            Quaternion estimatedRotationNoSuggestion = 
                    estimatedCameraNoSuggestion.getCameraRotation().
                            toQuaternion();
            
            //check that rotation has become closer to suggested value
            double diffEstimatedNoSuggestion = 
                    Math.pow(rotation.getA() - 
                            estimatedRotationNoSuggestion.getA(), 2.0) +
                    Math.pow(rotation.getB() - 
                            estimatedRotationNoSuggestion.getB(), 2.0) +
                    Math.pow(rotation.getC() - 
                            estimatedRotationNoSuggestion.getC(), 2.0) +
                    Math.pow(rotation.getD() - 
                            estimatedRotationNoSuggestion.getD(), 2.0);
            double diffEstimated =
                    Math.pow(rotation.getA() - estimatedRotation.getA(), 2.0) +
                    Math.pow(rotation.getB() - estimatedRotation.getB(), 2.0) +
                    Math.pow(rotation.getC() - estimatedRotation.getC(), 2.0) +
                    Math.pow(rotation.getD() - estimatedRotation.getD(), 2.0);
            
            if (diffEstimatedNoSuggestion >= diffEstimated) {
                numValid++;
            }     
            
            if (numCovariances > 0 && numValid > 0) {
                break;
            }
        }
        
        assertTrue(numCovariances > 0);
        assertTrue(numValid > 0);
    }
    
    @Test
    public void testEstimatePlanarSuggestedRotationWithFastRefinement() 
            throws LockedException, NotReadyException, CameraException,
            NotAvailableException {
        int numCovariances = 0;
        int numValid = 0;
        for (int t = 0; t < 5*TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            
            //intrinsic parameters
            double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                    verticalFocalLength, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, skewness);
            
            //create rotation parameters
            double roll = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double yaw = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            Quaternion rotation = new Quaternion(roll, pitch, yaw);
            rotation.normalize();
            
            //create camera center
            double[] cameraCenterArray = new double[3];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);
            
            //normalize the camera to improve accuracy
            camera.normalize();  
            
            //generate plane parallel to the camera's principal plane and
            //located at certain distance from it.
            double[] principalAxis = camera.getPrincipalAxisArray();
            double depth = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            Point3D centroid = new InhomogeneousPoint3D(
                    cameraCenter.getInhomX() + depth * principalAxis[0],
                    cameraCenter.getInhomY() + depth * principalAxis[1],
                    cameraCenter.getInhomZ() + depth * principalAxis[2]);
            assertTrue(camera.isPointInFrontOfCamera(centroid));
            
            Plane plane = new Plane(centroid, principalAxis);            
            
            int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);            
            
            double a = plane.getA();
            double b = plane.getB();
            double c = plane.getC();
            double d = plane.getD();
            
            List<Point3D> points3D = new ArrayList<>(nPoints);
            Point3D point3D;
            for (int i = 0; i < nPoints; i++) {
                //get a random point belonging to the plane
                //a*x + b*y + c*z + d*w = 0
                //y = -(a*x + c*z + d*w)/b or x = -(b*y + c*z + d*w)/a
                double homX, homY;
                double homW = 1.0;
                double homZ = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE);                
                if(Math.abs(b) > ABSOLUTE_ERROR){
                    homX = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE);
                    homY = -(a * homX + c * homZ + d * homW) / b;
                }else{
                    homY = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE);
                    homX = -(b * homY + c * homZ + d * homW) / a;
                }
                
                point3D = new HomogeneousPoint3D(homX, homY, homZ, homW);
                
                assertTrue(plane.isLocus(point3D));
                assertTrue(camera.isPointInFrontOfCamera(point3D));
                
                points3D.add(point3D);
            }
            
            List<Point2D> points2D = camera.project(points3D);  
            
            //create outliers
            Point2D point2DWithError;
            List<Point2D> points2DWithError = new ArrayList<>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, OUTLIER_STD_ERROR); 
            for (Point2D point2D : points2D) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //point is oulier
                    double errorX = errorRandomizer.nextDouble();
                    double errorY = errorRandomizer.nextDouble();
                    double errorW = errorRandomizer.nextDouble();
                    point2DWithError = new HomogeneousPoint2D(
                            point2D.getHomX() + errorX, 
                            point2D.getHomY() + errorY, 
                            point2D.getHomW() + errorW);
                } else {
                    //inlier point (without error)
                    point2DWithError = point2D;
                }
                
                points2DWithError.add(point2DWithError);
            }
            
            RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                    new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                    this, intrinsic, points3D, points2DWithError);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);            
            estimator.setSuggestRotationEnabled(true);
            estimator.setSuggestedRotationValue(rotation);      
            estimator.setResultRefined(true);
            estimator.setFastRefinementUsed(true);
            estimator.setCovarianceKept(true);            

            
            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getCovariance());
            
            
            PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }
            
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            if (estimator.getCovariance() != null) {
                numCovariances++;
            }                        
            reset();

            //decompone estimated camera and check its parameters
            estimatedCamera.decompose();
            
            //comparing rotation
            Quaternion estimatedRotation = estimatedCamera.getCameraRotation().
                    toQuaternion();
            estimatedRotation.normalize();
            
            //estimate without suggestion
            estimator.setSuggestRotationEnabled(false);
            
            PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }         
            
            estimatedCameraNoSuggestion.decompose();

            Quaternion estimatedRotationNoSuggestion = 
                    estimatedCameraNoSuggestion.getCameraRotation().
                            toQuaternion();
            
            //check that rotation has become closer to suggested value
            double diffEstimatedNoSuggestion = 
                    Math.pow(rotation.getA() - 
                            estimatedRotationNoSuggestion.getA(), 2.0) +
                    Math.pow(rotation.getB() - 
                            estimatedRotationNoSuggestion.getB(), 2.0) +
                    Math.pow(rotation.getC() - 
                            estimatedRotationNoSuggestion.getC(), 2.0) +
                    Math.pow(rotation.getD() - 
                            estimatedRotationNoSuggestion.getD(), 2.0);
            double diffEstimated =
                    Math.pow(rotation.getA() - estimatedRotation.getA(), 2.0) +
                    Math.pow(rotation.getB() - estimatedRotation.getB(), 2.0) +
                    Math.pow(rotation.getC() - estimatedRotation.getC(), 2.0) +
                    Math.pow(rotation.getD() - estimatedRotation.getD(), 2.0);
            
            if (diffEstimatedNoSuggestion >= diffEstimated) {
                numValid++;
            }            
            
            if (numCovariances > 0 && numValid > 0) {
                break;
            }
        }
        
        assertTrue(numCovariances > 0);
        assertTrue(numValid > 0);
    }
    
    @Test
    public void testEstimatePlanarSuggestedCenterEnabled() 
            throws LockedException, NotReadyException, CameraException,
            NotAvailableException {
        int numValid = 0;
        for (int t = 0; t < 5*TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            
            //intrinsic parameters
            double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                    verticalFocalLength, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, skewness);
            
            //create rotation parameters
            double roll = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double yaw = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            Quaternion rotation = new Quaternion(roll, pitch, yaw);
            rotation.normalize();
            
            //create camera center
            double[] cameraCenterArray = new double[3];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);
            
            //normalize the camera to improve accuracy
            camera.normalize();  
            
            //generate plane parallel to the camera's principal plane and
            //located at certain distance from it.
            double[] principalAxis = camera.getPrincipalAxisArray();
            double depth = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            Point3D centroid = new InhomogeneousPoint3D(
                    cameraCenter.getInhomX() + depth * principalAxis[0],
                    cameraCenter.getInhomY() + depth * principalAxis[1],
                    cameraCenter.getInhomZ() + depth * principalAxis[2]);
            assertTrue(camera.isPointInFrontOfCamera(centroid));
            
            Plane plane = new Plane(centroid, principalAxis);            
            
            int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);            
            
            double a = plane.getA();
            double b = plane.getB();
            double c = plane.getC();
            double d = plane.getD();
            
            List<Point3D> points3D = new ArrayList<>(nPoints);
            Point3D point3D;
            for (int i = 0; i < nPoints; i++) {
                //get a random point belonging to the plane
                //a*x + b*y + c*z + d*w = 0
                //y = -(a*x + c*z + d*w)/b or x = -(b*y + c*z + d*w)/a
                double homX, homY;
                double homW = 1.0;
                double homZ = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE);                
                if(Math.abs(b) > ABSOLUTE_ERROR){
                    homX = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE);
                    homY = -(a * homX + c * homZ + d * homW) / b;
                }else{
                    homY = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE);
                    homX = -(b * homY + c * homZ + d * homW) / a;
                }
                
                point3D = new HomogeneousPoint3D(homX, homY, homZ, homW);
                
                assertTrue(plane.isLocus(point3D));
                assertTrue(camera.isPointInFrontOfCamera(point3D));
                
                points3D.add(point3D);
            }
            
            List<Point2D> points2D = camera.project(points3D);  
            
            //create outliers
            Point2D point2DWithError;
            List<Point2D> points2DWithError = new ArrayList<>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, OUTLIER_STD_ERROR); 
            for (Point2D point2D : points2D) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //point is oulier
                    double errorX = errorRandomizer.nextDouble();
                    double errorY = errorRandomizer.nextDouble();
                    double errorW = errorRandomizer.nextDouble();
                    point2DWithError = new HomogeneousPoint2D(
                            point2D.getHomX() + errorX, 
                            point2D.getHomY() + errorY, 
                            point2D.getHomW() + errorW);
                } else {
                    //inlier point (without error)
                    point2DWithError = point2D;
                }
                
                points2DWithError.add(point2DWithError);
            }
            
            RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                    new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                    this, intrinsic, points3D, points2DWithError);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);            
            estimator.setSuggestCenterEnabled(true);
            estimator.setSuggestedCenterValue(cameraCenter);
            
            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getCovariance());
            
            
            PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }
            
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            //decompone estimated camera and check its parameters
            estimatedCamera.decompose();
            
            //comparing center
            Point3D estimatedCenter = estimatedCamera.getCameraCenter();
            
            //estimate without suggestion
            estimator.setSuggestCenterEnabled(false);
            
            PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }         
            
            estimatedCameraNoSuggestion.decompose();

            Point3D estimatedCenterNoSuggestion = 
                    estimatedCameraNoSuggestion.getCameraCenter();
            
            //check that camera center has become closer to suggested value
            if (cameraCenter.distanceTo(estimatedCenterNoSuggestion) >=
                    cameraCenter.distanceTo(estimatedCenter)) {
                numValid++;
            }
            
            if (numValid > 0) {
                break;
            }
        }
        
        assertTrue(numValid > 0);
    }
    
    @Test
    public void testEstimatePlanarSuggestedCenterWithRefinement() 
            throws LockedException, NotReadyException, CameraException,
            NotAvailableException {
        int numCovariances = 0;
        int numValid = 0;
        for (int t = 0; t < 5*TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            
            //intrinsic parameters
            double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                    verticalFocalLength, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, skewness);
            
            //create rotation parameters
            double roll = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double yaw = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            Quaternion rotation = new Quaternion(roll, pitch, yaw);
            rotation.normalize();
            
            //create camera center
            double[] cameraCenterArray = new double[3];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);
            
            //normalize the camera to improve accuracy
            camera.normalize();  
            
            //generate plane parallel to the camera's principal plane and
            //located at certain distance from it.
            double[] principalAxis = camera.getPrincipalAxisArray();
            double depth = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            Point3D centroid = new InhomogeneousPoint3D(
                    cameraCenter.getInhomX() + depth * principalAxis[0],
                    cameraCenter.getInhomY() + depth * principalAxis[1],
                    cameraCenter.getInhomZ() + depth * principalAxis[2]);
            assertTrue(camera.isPointInFrontOfCamera(centroid));
            
            Plane plane = new Plane(centroid, principalAxis);            
            
            int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);            
            
            double a = plane.getA();
            double b = plane.getB();
            double c = plane.getC();
            double d = plane.getD();
            
            List<Point3D> points3D = new ArrayList<>(nPoints);
            Point3D point3D;
            for (int i = 0; i < nPoints; i++) {
                //get a random point belonging to the plane
                //a*x + b*y + c*z + d*w = 0
                //y = -(a*x + c*z + d*w)/b or x = -(b*y + c*z + d*w)/a
                double homX, homY;
                double homW = 1.0;
                double homZ = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE);                
                if(Math.abs(b) > ABSOLUTE_ERROR){
                    homX = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE);
                    homY = -(a * homX + c * homZ + d * homW) / b;
                }else{
                    homY = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE);
                    homX = -(b * homY + c * homZ + d * homW) / a;
                }
                
                point3D = new HomogeneousPoint3D(homX, homY, homZ, homW);
                
                assertTrue(plane.isLocus(point3D));
                assertTrue(camera.isPointInFrontOfCamera(point3D));
                
                points3D.add(point3D);
            }
            
            List<Point2D> points2D = camera.project(points3D);  
            
            //create outliers
            Point2D point2DWithError;
            List<Point2D> points2DWithError = new ArrayList<>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, OUTLIER_STD_ERROR); 
            for (Point2D point2D : points2D) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //point is oulier
                    double errorX = errorRandomizer.nextDouble();
                    double errorY = errorRandomizer.nextDouble();
                    double errorW = errorRandomizer.nextDouble();
                    point2DWithError = new HomogeneousPoint2D(
                            point2D.getHomX() + errorX, 
                            point2D.getHomY() + errorY, 
                            point2D.getHomW() + errorW);
                } else {
                    //inlier point (without error)
                    point2DWithError = point2D;
                }
                
                points2DWithError.add(point2DWithError);
            }
            
            RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                    new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                    this, intrinsic, points3D, points2DWithError);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);            
            estimator.setSuggestCenterEnabled(true);
            estimator.setSuggestedCenterValue(cameraCenter);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);
            
            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getCovariance());
            
            
            PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }
            
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            if (estimator.getCovariance() != null) {
                numCovariances++;
            }            
            reset();

            //decompone estimated camera and check its parameters
            estimatedCamera.decompose();
            
            //comparing center
            Point3D estimatedCenter = estimatedCamera.getCameraCenter();
            
            //estimate without suggestion
            estimator.setSuggestCenterEnabled(false);
            
            PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }         
            
            estimatedCameraNoSuggestion.decompose();

            Point3D estimatedCenterNoSuggestion = 
                    estimatedCameraNoSuggestion.getCameraCenter();
            
            //check that camera center has become closer to suggested value
            if (cameraCenter.distanceTo(estimatedCenterNoSuggestion) >=
                    cameraCenter.distanceTo(estimatedCenter)) {
                numValid++;
            }
            
            if (numCovariances > 0 && numValid > 0) {
                break;
            }
        }
        
        assertTrue(numCovariances > 0);
        assertTrue(numValid > 0);
    }
    
    @Test
    public void testEstimatePlanarSuggestedCenterWithFastRefinement() 
            throws LockedException, NotReadyException, CameraException,
            NotAvailableException {
        int numCovariances = 0;
        int numValid = 0;
        for (int t = 0; t < 5*TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            
            //intrinsic parameters
            double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                    verticalFocalLength, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, skewness);
            
            //create rotation parameters
            double roll = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double yaw = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            Quaternion rotation = new Quaternion(roll, pitch, yaw);
            rotation.normalize();
            
            //create camera center
            double[] cameraCenterArray = new double[3];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);
            
            //normalize the camera to improve accuracy
            camera.normalize();  
            
            //generate plane parallel to the camera's principal plane and
            //located at certain distance from it.
            double[] principalAxis = camera.getPrincipalAxisArray();
            double depth = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            Point3D centroid = new InhomogeneousPoint3D(
                    cameraCenter.getInhomX() + depth * principalAxis[0],
                    cameraCenter.getInhomY() + depth * principalAxis[1],
                    cameraCenter.getInhomZ() + depth * principalAxis[2]);
            assertTrue(camera.isPointInFrontOfCamera(centroid));
            
            Plane plane = new Plane(centroid, principalAxis);            
            
            int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);            
            
            double a = plane.getA();
            double b = plane.getB();
            double c = plane.getC();
            double d = plane.getD();
            
            List<Point3D> points3D = new ArrayList<>(nPoints);
            Point3D point3D;
            for (int i = 0; i < nPoints; i++) {
                //get a random point belonging to the plane
                //a*x + b*y + c*z + d*w = 0
                //y = -(a*x + c*z + d*w)/b or x = -(b*y + c*z + d*w)/a
                double homX, homY;
                double homW = 1.0;
                double homZ = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE);                
                if(Math.abs(b) > ABSOLUTE_ERROR){
                    homX = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE);
                    homY = -(a * homX + c * homZ + d * homW) / b;
                }else{
                    homY = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE);
                    homX = -(b * homY + c * homZ + d * homW) / a;
                }
                
                point3D = new HomogeneousPoint3D(homX, homY, homZ, homW);
                
                assertTrue(plane.isLocus(point3D));
                assertTrue(camera.isPointInFrontOfCamera(point3D));
                
                points3D.add(point3D);
            }
            
            List<Point2D> points2D = camera.project(points3D);  
            
            //create outliers
            Point2D point2DWithError;
            List<Point2D> points2DWithError = new ArrayList<>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, OUTLIER_STD_ERROR); 
            for (Point2D point2D : points2D) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //point is oulier
                    double errorX = errorRandomizer.nextDouble();
                    double errorY = errorRandomizer.nextDouble();
                    double errorW = errorRandomizer.nextDouble();
                    point2DWithError = new HomogeneousPoint2D(
                            point2D.getHomX() + errorX, 
                            point2D.getHomY() + errorY, 
                            point2D.getHomW() + errorW);
                } else {
                    //inlier point (without error)
                    point2DWithError = point2D;
                }
                
                points2DWithError.add(point2DWithError);
            }
            
            RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                    new RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
                    this, intrinsic, points3D, points2DWithError);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);            
            estimator.setSuggestCenterEnabled(true);
            estimator.setSuggestedCenterValue(cameraCenter);
            estimator.setResultRefined(true);
            estimator.setFastRefinementUsed(true);
            estimator.setCovarianceKept(true);
            
            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getCovariance());
            
            
            PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }
            
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            if (estimator.getCovariance() != null) {
                numCovariances++;
            }            
            reset();

            //decompone estimated camera and check its parameters
            estimatedCamera.decompose();
            
            //comparing center
            Point3D estimatedCenter = estimatedCamera.getCameraCenter();
            
            //estimate without suggestion
            estimator.setSuggestCenterEnabled(false);
            
            PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }         
            
            estimatedCameraNoSuggestion.decompose();

            Point3D estimatedCenterNoSuggestion = 
                    estimatedCameraNoSuggestion.getCameraCenter();
            
            //check that camera center has become closer to suggested value
            if (cameraCenter.distanceTo(estimatedCenterNoSuggestion) >=
                    cameraCenter.distanceTo(estimatedCenter)) {
                numValid++;
            }
            
            if (numCovariances > 0 && numValid > 0) {
                break;
            }
        }
        
        assertTrue(numCovariances > 0);
        assertTrue(numValid > 0);
    }
        
    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration = 
                estimateProgressChange = 0;
    }    

    @Override
    public void onEstimateStart(PinholeCameraRobustEstimator estimator) {
        estimateStart++;
        checkLocked(
                (RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator)estimator);        
    }

    @Override
    public void onEstimateEnd(PinholeCameraRobustEstimator estimator) {
        estimateEnd++;
        checkLocked(
                (RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator)estimator);        
    }

    @Override
    public void onEstimateNextIteration(PinholeCameraRobustEstimator estimator, 
            int iteration) {
        estimateNextIteration++;
        checkLocked(
                (RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator)estimator);        
    }

    @Override
    public void onEstimateProgressChange(PinholeCameraRobustEstimator estimator,
            float progress) {
        estimateProgressChange++;
        checkLocked(
                (RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator)estimator);
    }
    
    private void checkLocked(
            RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator estimator) {
        List<Point3D> points3D = new ArrayList<>();
        List<Point2D> points2D = new ArrayList<>();
        try {
            estimator.setPoints(points3D, points2D);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setListener(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setProgressDelta(0.01f);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setThreshold(0.5);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setConfidence(0.5);            
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setMaxIterations(10);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setNormalizeSubsetPointCorrespondences(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.estimate();
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) {
        } catch (Exception ignore) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setSuggestSkewnessValueEnabled(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setSuggestedSkewnessValue(0.0);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setSuggestHorizontalFocalLengthEnabled(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setSuggestedHorizontalFocalLengthValue(0.0);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setSuggestVerticalFocalLengthEnabled(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setSuggestedVerticalFocalLengthValue(0.0);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setSuggestAspectRatioEnabled(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {        
            estimator.setSuggestedAspectRatioValue(0.0);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setSuggestPrincipalPointEnabled(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setSuggestedPrincipalPointValue(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setSuggestRotationEnabled(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setSuggestedRotationValue(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setSuggestCenterEnabled(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setSuggestedCenterValue(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setFastRefinementUsed(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setPlanarConfigurationAllowed(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setNullspaceDimension2Allowed(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setNullspaceDimension3Allowed(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setPlanarThreshold(1e9);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setIntrinsic(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        assertTrue(estimator.isLocked());        
    }        
}
