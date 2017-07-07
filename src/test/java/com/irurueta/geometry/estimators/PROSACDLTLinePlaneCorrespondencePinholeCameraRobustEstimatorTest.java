/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.estimators.PROSACLinePlaneCorrespondencePinholeCameraRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date March 12, 2015
 */
package com.irurueta.geometry.estimators;

import com.irurueta.geometry.CameraException;
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Line2D;
import com.irurueta.geometry.MatrixRotation3D;
import com.irurueta.geometry.NotAvailableException;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.Plane;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.Quaternion;
import com.irurueta.geometry.Rotation3D;
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

public class PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimatorTest 
        implements PinholeCameraRobustEstimatorListener {
    
    public static final double ABSOLUTE_ERROR = 1e-5;
    public static final double LARGE_ABSOLUTE_ERROR = 1e-2;
    public static final double VERY_LARGE_ABSOLUTE_ERROR = 1.0;
    
    public static final double MIN_RANDOM_VALUE = 0.0;
    public static final double MAX_RANDOM_VALUE = 1.0;
    
    public static final double MIN_FOCAL_LENGTH = 110.0;
    public static final double MAX_FOCAL_LENGTH = 130.0;
    
    public static final double MIN_SKEWNESS = -0.001;
    public static final double MAX_SKEWNESS = 0.001;
    
    public static final double MIN_PRINCIPAL_POINT = 90.0;
    public static final double MAX_PRINCIPAL_POINT = 100.0;
    
    public static final double MIN_ANGLE_DEGREES = 10.0;
    public static final double MAX_ANGLE_DEGREES = 15.0;

    public static final int INHOM_3D_COORDS = 3;
    public static final int LINE_PARAMS = 3;
    public static final int PLANE_PARAMS = 4;

    public static final int MIN_SAMPLES = 500;
    public static final int MAX_SAMPLES = 1000;
    
    public static final double THRESHOLD = 1e-5;
    
    public static final double STD_ERROR = 100.0;
    
    public static final double MIN_SCORE_ERROR = -0.3;
    public static final double MAX_SCORE_ERROR = 0.3;    
    
    public static final double MIN_CONFIDENCE = 0.95;
    public static final double MAX_CONFIDENCE = 0.99;
    
    public static final int MIN_MAX_ITERATIONS = 500;
    public static final int MAX_MAX_ITERATIONS = 5000;
    
    public static final int PERCENTAGE_OUTLIER = 20;
    
    public static final int TIMES = 10;
    
    private int estimateStart;
    private int estimateEnd;
    private int estimateNextIteration;
    private int estimateProgressChange;    
    
    public PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimatorTest() { }
    
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
        PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();
        
        assertEquals(estimator.getThreshold(),
                PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertNull(estimator.getLines());
        assertNull(estimator.getPlanes());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                LinePlaneCorrespondencePinholeCameraRobustEstimator.
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
        assertEquals(estimator.isComputeAndKeepInliersEnabled(),
                PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(estimator.isComputeAndKeepResidualsEnabled(),
                PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
        
        //test constructor with lines and planes
        List<Line2D> lines = new ArrayList<Line2D>();
        List<Plane> planes = new ArrayList<Plane>();
        for (int i = 0; i < LinePlaneCorrespondencePinholeCameraRobustEstimator.MIN_NUMBER_OF_LINE_PLANE_CORRESPONDENCES; i++) {
            lines.add(new Line2D());
            planes.add(new Plane());
        }
        
        estimator = new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                planes, lines);
        
        assertEquals(estimator.getThreshold(),
                PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertSame(estimator.getPlanes(), planes);
        assertSame(estimator.getLines(), lines);
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                LinePlaneCorrespondencePinholeCameraRobustEstimator.
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
        assertEquals(estimator.isComputeAndKeepInliersEnabled(),
                PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(estimator.isComputeAndKeepResidualsEnabled(),
                PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
        
        //force IllegalArgumentException
        List<Plane> planesEmpty = new ArrayList<Plane>();
        List<Line2D> linesEmpty = new ArrayList<Line2D>();
        estimator = null;
        try {
            //not enough points
            estimator = new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                    planesEmpty, linesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            //different sizes
            estimator = new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                    planes, linesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        assertNull(estimator);
        
        //test constructor with listener
        estimator = new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                this);
        
        assertEquals(estimator.getThreshold(),
                PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertNull(estimator.getPlanes());
        assertNull(estimator.getLines());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                LinePlaneCorrespondencePinholeCameraRobustEstimator.
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
        assertEquals(estimator.isComputeAndKeepInliersEnabled(),
                PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(estimator.isComputeAndKeepResidualsEnabled(),
                PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);

        
        //test constructor with listener and samples
        estimator = new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                this, planes, lines);
        
        assertEquals(estimator.getThreshold(),
                PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertSame(estimator.getPlanes(), planes);
        assertSame(estimator.getLines(), lines);
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                LinePlaneCorrespondencePinholeCameraRobustEstimator.
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
        assertEquals(estimator.isComputeAndKeepInliersEnabled(),
                PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(estimator.isComputeAndKeepResidualsEnabled(),
                PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
        
        //force IllegalArgumentException
        estimator = null;
        try {
            //not enough points
            estimator = new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                    this, planesEmpty, linesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            //different sizes
            estimator = new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                    this, planes, linesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        assertNull(estimator);
        
        //test constructor with quality scores
        double[] qualityScores = new double[
                LinePlaneCorrespondencePinholeCameraRobustEstimator.
                MIN_NUMBER_OF_LINE_PLANE_CORRESPONDENCES];
        double[] shortQualityScores = new double[1];
        
        estimator = new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                qualityScores);
        
        assertEquals(estimator.getThreshold(),
                PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertNull(estimator.getLines());
        assertNull(estimator.getPlanes());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                LinePlaneCorrespondencePinholeCameraRobustEstimator.
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
        assertEquals(estimator.isComputeAndKeepInliersEnabled(),
                PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(estimator.isComputeAndKeepResidualsEnabled(),
                PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                    shortQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        assertNull(estimator);
        
        //test constructor with lines and planes and quality scores
        estimator = new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                planes, lines, qualityScores);
        
        assertEquals(estimator.getThreshold(),
                PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertSame(estimator.getPlanes(), planes);
        assertSame(estimator.getLines(), lines);
        assertSame(estimator.getQualityScores(), qualityScores);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                LinePlaneCorrespondencePinholeCameraRobustEstimator.
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
        assertEquals(estimator.isComputeAndKeepInliersEnabled(),
                PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(estimator.isComputeAndKeepResidualsEnabled(),
                PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
        
        //force IllegalArgumentException
        estimator = null;
        try {
            //not enough points
            estimator = new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                    planesEmpty, linesEmpty, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            //different sizes
            estimator = new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                    planes, linesEmpty, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            //not enough scores
            estimator = new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                    planes, lines, shortQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        assertNull(estimator);
        
        //test constructor with listener and quality scores
        estimator = new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                this, qualityScores);
        
        assertEquals(estimator.getThreshold(),
                PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertNull(estimator.getPlanes());
        assertNull(estimator.getLines());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                LinePlaneCorrespondencePinholeCameraRobustEstimator.
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
        assertEquals(estimator.isComputeAndKeepInliersEnabled(),
                PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(estimator.isComputeAndKeepResidualsEnabled(),
                PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                    this, shortQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        assertNull(estimator);

        //test constructor with listener, samples and quality scores
        estimator = new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                this, planes, lines, qualityScores);
        
        assertEquals(estimator.getThreshold(),
                PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertSame(estimator.getPlanes(), planes);
        assertSame(estimator.getLines(), lines);
        assertSame(estimator.getQualityScores(), qualityScores);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                LinePlaneCorrespondencePinholeCameraRobustEstimator.
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
        assertEquals(estimator.isComputeAndKeepInliersEnabled(),
                PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(estimator.isComputeAndKeepResidualsEnabled(),
                PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
        
        //force IllegalArgumentException
        estimator = null;
        try {
            //not enough points
            estimator = new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                    this, planesEmpty, linesEmpty, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            //different sizes
            estimator = new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                    this, planes, linesEmpty, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            //not enough scores
            estimator = new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                    this, planes, lines, shortQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        assertNull(estimator);        
    }
    
    @Test
    public void testIsSetComputeAndKeepInliersEnabled() throws LockedException {
        PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();

        //check default value
        assertEquals(estimator.isComputeAndKeepInliersEnabled(),
                PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_COMPUTE_AND_KEEP_INLIERS);

        //set new value
        estimator.setComputeAndKeepInliersEnabled(
                !PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        
        //check correctness
        assertEquals(estimator.isComputeAndKeepInliersEnabled(),
                !PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_COMPUTE_AND_KEEP_INLIERS);        
    }
    
    @Test
    public void testIsSetComputeAndKeepResidualsEnabled() 
            throws LockedException {
        PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();

        //check default value
        assertEquals(estimator.isComputeAndKeepResidualsEnabled(),
                PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);

        //set new value
        estimator.setComputeAndKeepResidualsEnabled(
                !PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
        
        //check correctness
        assertEquals(estimator.isComputeAndKeepResidualsEnabled(),
                !PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);        
    }
    
    @Test
    public void testGetSetThreshold() throws LockedException {
        PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();
        
        //check default value
        assertEquals(estimator.getThreshold(),
                PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_THRESHOLD, 0.0);
        
        //set new value
        estimator.setThreshold(0.5);
        
        //check correctness
        assertEquals(estimator.getThreshold(), 0.5, 0.0);
        
        //Force IllegalArgumentException
        try {
            estimator.setThreshold(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
    }
    
    @Test
    public void testGetSetQualityScores() throws LockedException {
        PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();
        
        //check default value
        assertNull(estimator.getQualityScores());
        
        //set new value
        double[] qualityScores = new double[
                LinePlaneCorrespondencePinholeCameraEstimator.
                MIN_NUMBER_OF_LINE_PLANE_CORRESPONDENCES];
        estimator.setQualityScores(qualityScores);
        
        //check correctness
        assertSame(estimator.getQualityScores(), qualityScores);
        
        //Force IllegalArgumentException
        qualityScores = new double[1];
        try {
            estimator.setQualityScores(qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
    }    
    
    @Test
    public void testGetSetConfidence() throws LockedException {
        PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();
        
        //check default value
        assertEquals(estimator.getConfidence(),
                PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_CONFIDENCE, 0.0);
        
        //set new value
        estimator.setConfidence(0.5);
        
        //check correctness
        assertEquals(estimator.getConfidence(), 0.5, 0.0);
        
        //Force IllegalArgumentException
        try {
            estimator.setConfidence(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        
        try {
            estimator.setConfidence(2.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
    }
    
    @Test
    public void testGetSetMaxIterations() throws LockedException {
        PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();
        
        //check default value
        assertEquals(estimator.getMaxIterations(),
                PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                DEFAULT_MAX_ITERATIONS);
        
        //set new value
        estimator.setMaxIterations(10);
        
        //check correctness
        assertEquals(estimator.getMaxIterations(), 10);
        
        //Force IllegalArgumentException
        try {
            estimator.setMaxIterations(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
    }
    
    @Test
    public void testIsSetResultRefined() throws LockedException {
        PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();
        
        //check default value
        assertEquals(estimator.isResultRefined(),
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        
        //set new value
        estimator.setResultRefined(
                !PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        
        //check correctness
        assertEquals(estimator.isResultRefined(),
                !PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
    }
    
    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();
        
        //check default value
        assertEquals(estimator.isCovarianceKept(), 
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        
        //set new value
        estimator.setCovarianceKept(
                !PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        
        //check correctness
        assertEquals(estimator.isCovarianceKept(),
                !PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
    }
    
    @Test
    public void testIsSetFastRefinementUsed() throws LockedException {
        PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();
        
        //check default value
        assertEquals(estimator.isFastRefinementUsed(),
                PinholeCameraRobustEstimator.DEFAULT_USE_FAST_REFINEMENT);
        
        //set new value
        estimator.setFastRefinementUsed(
                !PinholeCameraRobustEstimator.DEFAULT_USE_FAST_REFINEMENT);
        
        //check correctness
        assertEquals(estimator.isFastRefinementUsed(),
                !PinholeCameraRobustEstimator.DEFAULT_USE_FAST_REFINEMENT);        
    }
    
    @Test
    public void testGetSetLinesAndPlanesAndIsReady() throws LockedException {
        PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();
        
        //check default values
        assertNull(estimator.getPlanes());
        assertNull(estimator.getLines());
        assertFalse(estimator.isReady());
        
        //set new value
        List<Plane> planes = new ArrayList<Plane>();
        List<Line2D> lines = new ArrayList<Line2D>();
        for (int i = 0; i < LinePlaneCorrespondencePinholeCameraRobustEstimator.MIN_NUMBER_OF_LINE_PLANE_CORRESPONDENCES; i++) {
            planes.add(new Plane());
            lines.add(new Line2D());
        }

        estimator.setLinesAndPlanes(planes, lines);
        
        //check correctness
        assertSame(estimator.getPlanes(), planes);
        assertSame(estimator.getLines(), lines);
        assertFalse(estimator.isReady());
        
        //if we set quality scores, then estimator becomes ready
        double[] qualityScores = new double[
                LinePlaneCorrespondencePinholeCameraRobustEstimator.
                MIN_NUMBER_OF_LINE_PLANE_CORRESPONDENCES];
        estimator.setQualityScores(qualityScores);
        
        assertTrue(estimator.isReady());

        
        //Force IllegalArgumentException
        List<Plane> planesEmpty = new ArrayList<Plane>();
        List<Line2D> linesEmpty = new ArrayList<Line2D>();
        try {
            //not enough samples
            estimator.setLinesAndPlanes(planesEmpty, linesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            //different sizes
            estimator.setLinesAndPlanes(planes, linesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
    }
    
    @Test
    public void testGetSetListenerAndIsListenerAvailable() 
            throws LockedException {
        PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();
        
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
    public void testIsSetSuggestSkewnessValueEnabled() throws LockedException {
        PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();
        
        //check default value
        assertEquals(estimator.isSuggestSkewnessValueEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED);
        
        //set new value
        estimator.setSuggestSkewnessValueEnabled(
                !PinholeCameraRobustEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED);
        
        //check correctness
        assertEquals(estimator.isSuggestSkewnessValueEnabled(),
                !PinholeCameraRobustEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED);        
    }
    
    @Test
    public void testGetSetSuggestedSkewnessValue() throws LockedException {
        PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();

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
    public void testIsSetSuggestHorizontalFocalLengthEnabled() 
            throws LockedException {
        PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();
        
        //check default value
        assertEquals(estimator.isSuggestHorizontalFocalLengthEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);
        
        //set new value
        estimator.setSuggestHorizontalFocalLengthEnabled(
                !PinholeCameraRobustEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);
        
        //check correctness
        assertEquals(estimator.isSuggestHorizontalFocalLengthEnabled(),
                !PinholeCameraRobustEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);        
    }
    
    @Test
    public void testGetSetSuggestedHorizontalFocalLengthValue() 
            throws LockedException {
        PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();
        
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
        PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();
        
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
        PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();
        
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
        PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();
        
        //check default value
        assertEquals(estimator.isSuggestAspectRatioEnabled(),
                PinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED);
        
        //set new value
        estimator.setSuggestAspectRatioEnabled(
                !PinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED);
        
        //check correctness
        assertEquals(estimator.isSuggestAspectRatioEnabled(),
                !PinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED);        
    }
    
    @Test
    public void testGetSetSuggestedAspectRatioValue() throws LockedException {
        PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();
        
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
        PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();
        
        //check default value
        assertEquals(estimator.isSuggestPrincipalPointEnabled(), 
                PinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED);
        
        //set new value
        estimator.setSuggestPrincipalPointEnabled(
                !PinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED);
        
        //check correctness
        assertEquals(estimator.isSuggestPrincipalPointEnabled(), 
                !PinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED);
    }
    
    @Test
    public void testGetSetSuggestedPrincipalPointValue() 
            throws LockedException {
        PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();
        
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
        PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();
        
        //check default value
        assertEquals(estimator.isSuggestRotationEnabled(),
                PinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_ROTATION_ENABLED);
        
        //set new value
        estimator.setSuggestRotationEnabled(
                !PinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_ROTATION_ENABLED);

        //check correctness
        assertEquals(estimator.isSuggestRotationEnabled(),
                !PinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_ROTATION_ENABLED);        
    }
    
    @Test
    public void testGetSetSuggestedRotationValue() throws LockedException {
        PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();

        //chekc default value
        assertNull(estimator.getSuggestedRotationValue());
        
        //set new value
        Quaternion q = new Quaternion();
        estimator.setSuggestedRotationValue(q);
        
        //check correctness
        assertSame(estimator.getSuggestedRotationValue(), q);        
    }
    
    @Test
    public void testIsSetSuggestCenterEnabled() throws LockedException {
        PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();
        
        //check default value
        assertEquals(estimator.isSuggestCenterEnabled(),
                LMedSDLTPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_CENTER_ENABLED);
        
        //set new value
        estimator.setSuggestCenterEnabled(
                !LMedSDLTPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_CENTER_ENABLED);

        //check correctness
        assertEquals(estimator.isSuggestCenterEnabled(),
                !LMedSDLTPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_CENTER_ENABLED);
    }
    
    @Test
    public void testGetSetSuggestedCenterValue() throws LockedException {
        PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();
        
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
        PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();
        
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
        } catch (IllegalArgumentException e) { }
        try {
            estimator.setProgressDelta(2.0f);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
    }
    
    @Test
    public void testEstimateWithoutRefinement() throws IllegalArgumentException,
            LockedException, NotReadyException, RobustEstimatorException, 
            CameraException, NotAvailableException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
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
            double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler, 
                    gammaEuler);
            
            //create camera center
            double[] cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);

            //normalize the camera to improve accuracy
            camera.normalize();            
            
            int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            List<Line2D> lines = new ArrayList<Line2D>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE));                        
                lines.add(line);
            }
            
            List<Plane> planes = camera.backProjectLines(lines);
            
            //create outliers
            Plane planeWithError;
            List<Plane> planesWithError = new ArrayList<Plane>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR); 
            double[] qualityScores = new double[nSamples];
            int j = 0;            
            for (Plane plane : planes) {
                double scoreError = randomizer.nextDouble(MIN_SCORE_ERROR, 
                        MAX_SCORE_ERROR);
                qualityScores[j] = 1.0 + scoreError;                
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //plane is oulier
                    double errorA = errorRandomizer.nextDouble();
                    double errorB = errorRandomizer.nextDouble();
                    double errorC = errorRandomizer.nextDouble();
                    double errorD = errorRandomizer.nextDouble();
                    planeWithError = new Plane(plane.getA() + errorA,
                            plane.getB() + errorB, plane.getC() + errorC,
                            plane.getD() + errorD);
                    
                    double error = Math.sqrt(errorA * errorA + errorB * errorB +
                            errorC * errorC + errorD * errorD);
                    qualityScores[j] = 1.0 / (1.0 + error) + scoreError;
                } else {
                    //inlier plane (without error)
                    planeWithError = plane;
                }
                
                planesWithError.add(planeWithError);
                j++;
            }
            
            PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                    this, planesWithError, lines, qualityScores);
            
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
            assertNull(estimator.getCovariance());

            PinholeCamera camera2 = estimator.estimate();
            
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
            
            //backproject original 2D lines using estimated camera and check
            //distance to 3D planes without error
            Plane originalPlane, estimatedPlane;
            for (int i = 0; i < nSamples; i++) {
                line = lines.get(i);
                originalPlane = planes.get(i);
                estimatedPlane = camera2.backProject(line);

                assertTrue(originalPlane.equals(estimatedPlane, 
                        ABSOLUTE_ERROR));
            }
            
            //decompose estimated camera and check its parameters
            camera2.decompose();
            
            //compare intrinsic parameters
            PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    camera2.getIntrinsicParameters();
            
            assertEquals(horizontalFocalLength, 
                    estimatedIntrinsic.getHorizontalFocalLength(), 
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(verticalFocalLength,
                    estimatedIntrinsic.getVerticalFocalLength(), 
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(horizontalPrincipalPoint,
                    estimatedIntrinsic.getHorizontalPrincipalPoint(), 
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(verticalPrincipalPoint,
                    estimatedIntrinsic.getVerticalPrincipalPoint(),
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(skewness, estimatedIntrinsic.getSkewness(), 
                    VERY_LARGE_ABSOLUTE_ERROR);
            
            //Comparing estimated rotation
            Rotation3D estimatedRotation = camera2.getCameraRotation();
            
            MatrixRotation3D estimatedRotation2 = 
                    (MatrixRotation3D)estimatedRotation;
            double estimatedAlphaEuler = 
                    estimatedRotation2.getAlphaEulerAngle();
            double estimatedBetaEuler = 
                    estimatedRotation2.getBetaEulerAngle();
            double estimatedGammaEuler = 
                    estimatedRotation2.getGammaEulerAngle();
            boolean validAlphaEuler, validBetaEuler, 
                    validGammaEuler;
            
            if (Math.abs(alphaEuler - estimatedAlphaEuler) <= 
                    LARGE_ABSOLUTE_ERROR) {
                validAlphaEuler = true;
            } else if ((Math.abs(alphaEuler) + Math.abs(estimatedAlphaEuler) - 
                    Math.PI) <= LARGE_ABSOLUTE_ERROR) {
                validAlphaEuler = true;
            } else {
                validAlphaEuler = false;
            }

            if (Math.abs(betaEuler - estimatedBetaEuler) <= 
                    LARGE_ABSOLUTE_ERROR) {
                validBetaEuler = true;
            } else if ((Math.abs(betaEuler) + Math.abs(estimatedBetaEuler) - 
                    Math.PI) <= LARGE_ABSOLUTE_ERROR) {
                validBetaEuler = true;
            } else {
                validBetaEuler = false;
            }

            if (Math.abs(gammaEuler - estimatedGammaEuler) <= 
                    LARGE_ABSOLUTE_ERROR) {
                validGammaEuler = true;
            } else if ((Math.abs(gammaEuler) + Math.abs(estimatedGammaEuler) - 
                    Math.PI) <= LARGE_ABSOLUTE_ERROR) {
                validGammaEuler = true;
            } else {
                validGammaEuler = false;
            }
            
            
            assertTrue(validAlphaEuler);
            assertTrue(validBetaEuler);
            assertTrue(validGammaEuler);
            
        
            //comparing estimated camera center
            Point3D estimatedCameraCenter = camera2.getCameraCenter();
            assertTrue(cameraCenter.equals(estimatedCameraCenter, 
                    LARGE_ABSOLUTE_ERROR));       
            
            numValid++;
            
            if (numValid > 0) {
                break;
            }
        }
        
        assertTrue(numValid > 0);
    }

    @Test
    public void testEstimateWithRefinement() throws IllegalArgumentException,
            LockedException, NotReadyException, RobustEstimatorException, 
            CameraException, NotAvailableException {
        int numCovariances = 0;
        int numValid = 0;        
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
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
            double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler, 
                    gammaEuler);
            
            //create camera center
            double[] cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);

            //normalize the camera to improve accuracy
            camera.normalize();            
            
            int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            List<Line2D> lines = new ArrayList<Line2D>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE));                        
                lines.add(line);
            }
            
            List<Plane> planes = camera.backProjectLines(lines);
            
            //create outliers
            Plane planeWithError;
            List<Plane> planesWithError = new ArrayList<Plane>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR); 
            double[] qualityScores = new double[nSamples];
            int j = 0;            
            for (Plane plane : planes) {
                double scoreError = randomizer.nextDouble(MIN_SCORE_ERROR, 
                        MAX_SCORE_ERROR);
                qualityScores[j] = 1.0 + scoreError;                
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //plane is oulier
                    double errorA = errorRandomizer.nextDouble();
                    double errorB = errorRandomizer.nextDouble();
                    double errorC = errorRandomizer.nextDouble();
                    double errorD = errorRandomizer.nextDouble();
                    planeWithError = new Plane(plane.getA() + errorA,
                            plane.getB() + errorB, plane.getC() + errorC,
                            plane.getD() + errorD);
                    
                    double error = Math.sqrt(errorA * errorA + errorB * errorB +
                            errorC * errorC + errorD * errorD);
                    qualityScores[j] = 1.0 / (1.0 + error) + scoreError;
                } else {
                    //inlier plane (without error)
                    planeWithError = plane;
                }
                
                planesWithError.add(planeWithError);
                j++;
            }
            
            PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                    this, planesWithError, lines, qualityScores);
            
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
            assertNull(estimator.getCovariance());

            PinholeCamera camera2 = estimator.estimate();
            
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
            
            //backproject original 2D lines using estimated camera and check
            //distance to 3D planes without error
            Plane originalPlane, estimatedPlane;
            boolean failed = false;
            for (int i = 0; i < nSamples; i++) {
                line = lines.get(i);
                originalPlane = planes.get(i);
                estimatedPlane = camera2.backProject(line);

                if (!originalPlane.equals(estimatedPlane, ABSOLUTE_ERROR)) {
                    failed = true;
                    break;
                }                
                assertTrue(originalPlane.equals(estimatedPlane, 
                        ABSOLUTE_ERROR));
            }
            
            if (failed) continue;
            
            //decompose estimated camera and check its parameters
            camera2.decompose();
            
            //compare intrinsic parameters
            PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    camera2.getIntrinsicParameters();
            
            if (Math.abs(horizontalFocalLength - 
                    estimatedIntrinsic.getHorizontalFocalLength()) > 
                    VERY_LARGE_ABSOLUTE_ERROR) {
                continue;
            }            
            assertEquals(horizontalFocalLength, 
                    estimatedIntrinsic.getHorizontalFocalLength(), 
                    VERY_LARGE_ABSOLUTE_ERROR);
            
            if (Math.abs(verticalFocalLength -
                    estimatedIntrinsic.getVerticalFocalLength()) >
                    VERY_LARGE_ABSOLUTE_ERROR) {
                continue;
            }            
            assertEquals(verticalFocalLength,
                    estimatedIntrinsic.getVerticalFocalLength(), 
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(horizontalPrincipalPoint,
                    estimatedIntrinsic.getHorizontalPrincipalPoint(), 
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(verticalPrincipalPoint,
                    estimatedIntrinsic.getVerticalPrincipalPoint(),
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(skewness, estimatedIntrinsic.getSkewness(), 
                    VERY_LARGE_ABSOLUTE_ERROR);
            
            //Comparing estimated rotation
            Rotation3D estimatedRotation = camera2.getCameraRotation();
            
            MatrixRotation3D estimatedRotation2 = 
                    (MatrixRotation3D)estimatedRotation;
            double estimatedAlphaEuler = 
                    estimatedRotation2.getAlphaEulerAngle();
            double estimatedBetaEuler = 
                    estimatedRotation2.getBetaEulerAngle();
            double estimatedGammaEuler = 
                    estimatedRotation2.getGammaEulerAngle();
            boolean validAlphaEuler, validBetaEuler, 
                    validGammaEuler;
            
            if (Math.abs(alphaEuler - estimatedAlphaEuler) <= 
                    LARGE_ABSOLUTE_ERROR) {
                validAlphaEuler = true;
            } else if ((Math.abs(alphaEuler) + Math.abs(estimatedAlphaEuler) - 
                    Math.PI) <= LARGE_ABSOLUTE_ERROR) {
                validAlphaEuler = true;
            } else {
                validAlphaEuler = false;
            }

            if (Math.abs(betaEuler - estimatedBetaEuler) <= 
                    LARGE_ABSOLUTE_ERROR) {
                validBetaEuler = true;
            } else if ((Math.abs(betaEuler) + Math.abs(estimatedBetaEuler) - 
                    Math.PI) <= LARGE_ABSOLUTE_ERROR) {
                validBetaEuler = true;
            } else {
                validBetaEuler = false;
            }

            if (Math.abs(gammaEuler - estimatedGammaEuler) <= 
                    LARGE_ABSOLUTE_ERROR) {
                validGammaEuler = true;
            } else if ((Math.abs(gammaEuler) + Math.abs(estimatedGammaEuler) - 
                    Math.PI) <= LARGE_ABSOLUTE_ERROR) {
                validGammaEuler = true;
            } else {
                validGammaEuler = false;
            }
            
            
            assertTrue(validAlphaEuler);
            assertTrue(validBetaEuler);
            assertTrue(validGammaEuler);
            
        
            //comparing estimated camera center
            Point3D estimatedCameraCenter = camera2.getCameraCenter();
            assertTrue(cameraCenter.equals(estimatedCameraCenter, 
                    LARGE_ABSOLUTE_ERROR));       
            
            numValid++;
            
            if (numCovariances > 0 && numValid > 0) {
                break;
            }
        }
        
        assertTrue(numCovariances > 0);
        assertTrue(numValid > 0);        
    }

    @Test
    public void testEstimateWithFastRefinement() 
            throws IllegalArgumentException, LockedException, NotReadyException,
            RobustEstimatorException, CameraException, NotAvailableException {
        int numCovariances = 0;
        int numValid = 0;        
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
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
            double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler, 
                    gammaEuler);
            
            //create camera center
            double[] cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);

            //normalize the camera to improve accuracy
            camera.normalize();            
            
            int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            List<Line2D> lines = new ArrayList<Line2D>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE));                        
                lines.add(line);
            }
            
            List<Plane> planes = camera.backProjectLines(lines);
            
            //create outliers
            Plane planeWithError;
            List<Plane> planesWithError = new ArrayList<Plane>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR); 
            double[] qualityScores = new double[nSamples];
            int j = 0;            
            for (Plane plane : planes) {
                double scoreError = randomizer.nextDouble(MIN_SCORE_ERROR, 
                        MAX_SCORE_ERROR);
                qualityScores[j] = 1.0 + scoreError;                
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //plane is oulier
                    double errorA = errorRandomizer.nextDouble();
                    double errorB = errorRandomizer.nextDouble();
                    double errorC = errorRandomizer.nextDouble();
                    double errorD = errorRandomizer.nextDouble();
                    planeWithError = new Plane(plane.getA() + errorA,
                            plane.getB() + errorB, plane.getC() + errorC,
                            plane.getD() + errorD);
                    
                    double error = Math.sqrt(errorA * errorA + errorB * errorB +
                            errorC * errorC + errorD * errorD);
                    qualityScores[j] = 1.0 / (1.0 + error) + scoreError;
                } else {
                    //inlier plane (without error)
                    planeWithError = plane;
                }
                
                planesWithError.add(planeWithError);
                j++;
            }
            
            PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                    this, planesWithError, lines, qualityScores);
            
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
            assertNull(estimator.getCovariance());

            PinholeCamera camera2 = estimator.estimate();
            
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
            
            //backproject original 2D lines using estimated camera and check
            //distance to 3D planes without error
            Plane originalPlane, estimatedPlane;
            boolean failed = false;
            for (int i = 0; i < nSamples; i++) {
                line = lines.get(i);
                originalPlane = planes.get(i);
                estimatedPlane = camera2.backProject(line);

                if (!originalPlane.equals(estimatedPlane, ABSOLUTE_ERROR)) {
                    failed = true;
                    break;
                }                
                assertTrue(originalPlane.equals(estimatedPlane, 
                        ABSOLUTE_ERROR));
            }
            
            if (failed) continue;
            
            //decompose estimated camera and check its parameters
            camera2.decompose();
            
            //compare intrinsic parameters
            PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    camera2.getIntrinsicParameters();
            
            if (Math.abs(horizontalFocalLength - 
                    estimatedIntrinsic.getHorizontalFocalLength()) > 
                    VERY_LARGE_ABSOLUTE_ERROR) {
                continue;
            }            
            assertEquals(horizontalFocalLength, 
                    estimatedIntrinsic.getHorizontalFocalLength(), 
                    VERY_LARGE_ABSOLUTE_ERROR);
            
            if (Math.abs(verticalFocalLength -
                    estimatedIntrinsic.getVerticalFocalLength()) >
                    VERY_LARGE_ABSOLUTE_ERROR) {
                continue;
            }            
            assertEquals(verticalFocalLength,
                    estimatedIntrinsic.getVerticalFocalLength(), 
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(horizontalPrincipalPoint,
                    estimatedIntrinsic.getHorizontalPrincipalPoint(), 
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(verticalPrincipalPoint,
                    estimatedIntrinsic.getVerticalPrincipalPoint(),
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(skewness, estimatedIntrinsic.getSkewness(), 
                    VERY_LARGE_ABSOLUTE_ERROR);
            
            //Comparing estimated rotation
            Rotation3D estimatedRotation = camera2.getCameraRotation();
            
            MatrixRotation3D estimatedRotation2 = 
                    (MatrixRotation3D)estimatedRotation;
            double estimatedAlphaEuler = 
                    estimatedRotation2.getAlphaEulerAngle();
            double estimatedBetaEuler = 
                    estimatedRotation2.getBetaEulerAngle();
            double estimatedGammaEuler = 
                    estimatedRotation2.getGammaEulerAngle();
            boolean validAlphaEuler, validBetaEuler, 
                    validGammaEuler;
            
            if (Math.abs(alphaEuler - estimatedAlphaEuler) <= 
                    LARGE_ABSOLUTE_ERROR) {
                validAlphaEuler = true;
            } else if ((Math.abs(alphaEuler) + Math.abs(estimatedAlphaEuler) - 
                    Math.PI) <= LARGE_ABSOLUTE_ERROR) {
                validAlphaEuler = true;
            } else {
                validAlphaEuler = false;
            }

            if (Math.abs(betaEuler - estimatedBetaEuler) <= 
                    LARGE_ABSOLUTE_ERROR) {
                validBetaEuler = true;
            } else if ((Math.abs(betaEuler) + Math.abs(estimatedBetaEuler) - 
                    Math.PI) <= LARGE_ABSOLUTE_ERROR) {
                validBetaEuler = true;
            } else {
                validBetaEuler = false;
            }

            if (Math.abs(gammaEuler - estimatedGammaEuler) <= 
                    LARGE_ABSOLUTE_ERROR) {
                validGammaEuler = true;
            } else if ((Math.abs(gammaEuler) + Math.abs(estimatedGammaEuler) - 
                    Math.PI) <= LARGE_ABSOLUTE_ERROR) {
                validGammaEuler = true;
            } else {
                validGammaEuler = false;
            }
            
            
            assertTrue(validAlphaEuler);
            assertTrue(validBetaEuler);
            assertTrue(validGammaEuler);
            
        
            //comparing estimated camera center
            Point3D estimatedCameraCenter = camera2.getCameraCenter();
            assertTrue(cameraCenter.equals(estimatedCameraCenter, 
                    LARGE_ABSOLUTE_ERROR));       
            
            numValid++;
            
            if (numCovariances > 0 && numValid > 0) {
                break;
            }
        }
        
        assertTrue(numCovariances > 0);
        assertTrue(numValid > 0);        
    }

    @Test
    public void testEstimateSuggestedSkewness() throws IllegalArgumentException,
            LockedException, NotReadyException, RobustEstimatorException, 
            CameraException, NotAvailableException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
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
            double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler, 
                    gammaEuler);
            
            //create camera center
            double[] cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);

            //normalize the camera to improve accuracy
            camera.normalize();            
            
            int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            List<Line2D> lines = new ArrayList<Line2D>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE));                        
                lines.add(line);
            }
            
            List<Plane> planes = camera.backProjectLines(lines);
            
            //create outliers
            Plane planeWithError;
            List<Plane> planesWithError = new ArrayList<Plane>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR); 
            double[] qualityScores = new double[nSamples];
            int j = 0;            
            for (Plane plane : planes) {
                double scoreError = randomizer.nextDouble(MIN_SCORE_ERROR, 
                        MAX_SCORE_ERROR);
                qualityScores[j] = 1.0 + scoreError;                
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //plane is oulier
                    double errorA = errorRandomizer.nextDouble();
                    double errorB = errorRandomizer.nextDouble();
                    double errorC = errorRandomizer.nextDouble();
                    double errorD = errorRandomizer.nextDouble();
                    planeWithError = new Plane(plane.getA() + errorA,
                            plane.getB() + errorB, plane.getC() + errorC,
                            plane.getD() + errorD);
                    
                    double error = Math.sqrt(errorA * errorA + errorB * errorB +
                            errorC * errorC + errorD * errorD);
                    qualityScores[j] = 1.0 / (1.0 + error) + scoreError;
                } else {
                    //inlier plane (without error)
                    planeWithError = plane;
                }
                
                planesWithError.add(planeWithError);
                j++;
            }
            
            PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                    this, planesWithError, lines, qualityScores);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);
            estimator.setResultRefined(false);
            estimator.setCovarianceKept(false);            
            estimator.setSuggestSkewnessValueEnabled(true);
            estimator.setSuggestedSkewnessValue(skewness);            

            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
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
            assertNull(estimator.getCovariance());            
            reset();

            //decompone estimated camera and check its parameters
            estimatedCamera.decompose();
            
            //comparing camera intrinsic parameters
            PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    estimatedCamera.getIntrinsicParameters();            
            double estimatedSkewness = estimatedIntrinsic.getSkewness();
            
            //estimate without suggestion
            estimator.setSuggestSkewnessValueEnabled(false);
            
            PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }         
            
            estimatedCameraNoSuggestion.decompose();
            
            PinholeCameraIntrinsicParameters estimatedIntrinsicNoSuggestion =
                    estimatedCameraNoSuggestion.getIntrinsicParameters();
            double estimatedSkewnessNoSuggestion = 
                    estimatedIntrinsicNoSuggestion.getSkewness();
            
            //check that skewness has become closer to suggested value
            if (Math.abs(skewness - estimatedSkewnessNoSuggestion) >= 
                    Math.abs(skewness - estimatedSkewness)) {            
                numValid++;
            }           
            
            if (numValid > 0) {
                break;
            }
        }
        
        assertTrue(numValid > 0);
    }

    @Test
    public void testEstimateSuggestedSkewnessWithRefinement() 
            throws IllegalArgumentException, LockedException, NotReadyException,
            RobustEstimatorException, CameraException, NotAvailableException {
        int numCovariances = 0;
        int numValid = 0;        
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
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
            double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler, 
                    gammaEuler);
            
            //create camera center
            double[] cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);

            //normalize the camera to improve accuracy
            camera.normalize();            
            
            int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            List<Line2D> lines = new ArrayList<Line2D>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE));                        
                lines.add(line);
            }
            
            List<Plane> planes = camera.backProjectLines(lines);
            
            //create outliers
            Plane planeWithError;
            List<Plane> planesWithError = new ArrayList<Plane>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR); 
            double[] qualityScores = new double[nSamples];
            int j = 0;            
            for (Plane plane : planes) {
                double scoreError = randomizer.nextDouble(MIN_SCORE_ERROR, 
                        MAX_SCORE_ERROR);
                qualityScores[j] = 1.0 + scoreError;                
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //plane is oulier
                    double errorA = errorRandomizer.nextDouble();
                    double errorB = errorRandomizer.nextDouble();
                    double errorC = errorRandomizer.nextDouble();
                    double errorD = errorRandomizer.nextDouble();
                    planeWithError = new Plane(plane.getA() + errorA,
                            plane.getB() + errorB, plane.getC() + errorC,
                            plane.getD() + errorD);
                    
                    double error = Math.sqrt(errorA * errorA + errorB * errorB +
                            errorC * errorC + errorD * errorD);
                    qualityScores[j] = 1.0 / (1.0 + error) + scoreError;
                } else {
                    //inlier plane (without error)
                    planeWithError = plane;
                }
                
                planesWithError.add(planeWithError);
                j++;
            }
            
            PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                    this, planesWithError, lines, qualityScores);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);            
            estimator.setSuggestSkewnessValueEnabled(true);
            estimator.setSuggestedSkewnessValue(skewness);                        

            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
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
            
            //comparing camera intrinsic parameters
            PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    estimatedCamera.getIntrinsicParameters();            
            double estimatedSkewness = estimatedIntrinsic.getSkewness();
            
            //estimate without suggestion
            estimator.setSuggestSkewnessValueEnabled(false);
            
            PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }         
            
            estimatedCameraNoSuggestion.decompose();
            
            PinholeCameraIntrinsicParameters estimatedIntrinsicNoSuggestion =
                    estimatedCameraNoSuggestion.getIntrinsicParameters();
            double estimatedSkewnessNoSuggestion = 
                    estimatedIntrinsicNoSuggestion.getSkewness();
            
            //check that skewness has become closer to suggested value
            if (Math.abs(skewness - estimatedSkewnessNoSuggestion) >= 
                    Math.abs(skewness - estimatedSkewness)) {            
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
    public void testEstimateSuggestedSkewnessWithFastRefinement() 
            throws IllegalArgumentException, LockedException, NotReadyException,
            RobustEstimatorException, CameraException, NotAvailableException {
        int numCovariances = 0;
        int numValid = 0;        
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
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
            double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler, 
                    gammaEuler);
            
            //create camera center
            double[] cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);

            //normalize the camera to improve accuracy
            camera.normalize();            
            
            int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            List<Line2D> lines = new ArrayList<Line2D>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE));                        
                lines.add(line);
            }
            
            List<Plane> planes = camera.backProjectLines(lines);
            
            //create outliers
            Plane planeWithError;
            List<Plane> planesWithError = new ArrayList<Plane>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR); 
            double[] qualityScores = new double[nSamples];
            int j = 0;            
            for (Plane plane : planes) {
                double scoreError = randomizer.nextDouble(MIN_SCORE_ERROR, 
                        MAX_SCORE_ERROR);
                qualityScores[j] = 1.0 + scoreError;                
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //plane is oulier
                    double errorA = errorRandomizer.nextDouble();
                    double errorB = errorRandomizer.nextDouble();
                    double errorC = errorRandomizer.nextDouble();
                    double errorD = errorRandomizer.nextDouble();
                    planeWithError = new Plane(plane.getA() + errorA,
                            plane.getB() + errorB, plane.getC() + errorC,
                            plane.getD() + errorD);
                    
                    double error = Math.sqrt(errorA * errorA + errorB * errorB +
                            errorC * errorC + errorD * errorD);
                    qualityScores[j] = 1.0 / (1.0 + error) + scoreError;
                } else {
                    //inlier plane (without error)
                    planeWithError = plane;
                }
                
                planesWithError.add(planeWithError);
                j++;
            }
            
            PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                    this, planesWithError, lines, qualityScores);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);
            estimator.setResultRefined(true);
            estimator.setFastRefinementUsed(true);
            estimator.setCovarianceKept(true);            
            estimator.setSuggestSkewnessValueEnabled(true);
            estimator.setSuggestedSkewnessValue(skewness);            
            
            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
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
            
            //comparing camera intrinsic parameters
            PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    estimatedCamera.getIntrinsicParameters();            
            double estimatedSkewness = estimatedIntrinsic.getSkewness();
            
            //estimate without suggestion
            estimator.setSuggestSkewnessValueEnabled(false);
            
            PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }         
            
            estimatedCameraNoSuggestion.decompose();
            
            PinholeCameraIntrinsicParameters estimatedIntrinsicNoSuggestion =
                    estimatedCameraNoSuggestion.getIntrinsicParameters();
            double estimatedSkewnessNoSuggestion = 
                    estimatedIntrinsicNoSuggestion.getSkewness();
            
            //check that skewness has become closer to suggested value
            if (Math.abs(skewness - estimatedSkewnessNoSuggestion) >= 
                    Math.abs(skewness - estimatedSkewness)) {            
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
    public void testEstimateSuggestedHorizontalFocalLengthEnabled() 
            throws IllegalArgumentException, LockedException, NotReadyException, 
            RobustEstimatorException, CameraException, NotAvailableException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
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
            double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler, 
                    gammaEuler);
            
            //create camera center
            double[] cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);

            //normalize the camera to improve accuracy
            camera.normalize();            
            
            int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            List<Line2D> lines = new ArrayList<Line2D>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE));                        
                lines.add(line);
            }
            
            List<Plane> planes = camera.backProjectLines(lines);
            
            //create outliers
            Plane planeWithError;
            List<Plane> planesWithError = new ArrayList<Plane>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR); 
            double[] qualityScores = new double[nSamples];
            int j = 0;            
            for (Plane plane : planes) {
                double scoreError = randomizer.nextDouble(MIN_SCORE_ERROR, 
                        MAX_SCORE_ERROR);
                qualityScores[j] = 1.0 + scoreError;                
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //plane is oulier
                    double errorA = errorRandomizer.nextDouble();
                    double errorB = errorRandomizer.nextDouble();
                    double errorC = errorRandomizer.nextDouble();
                    double errorD = errorRandomizer.nextDouble();
                    planeWithError = new Plane(plane.getA() + errorA,
                            plane.getB() + errorB, plane.getC() + errorC,
                            plane.getD() + errorD);
                    
                    double error = Math.sqrt(errorA * errorA + errorB * errorB +
                            errorC * errorC + errorD * errorD);
                    qualityScores[j] = 1.0 / (1.0 + error) + scoreError;
                } else {
                    //inlier plane (without error)
                    planeWithError = plane;
                }
                
                planesWithError.add(planeWithError);
                j++;
            }
            
            PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                    this, planesWithError, lines, qualityScores);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);
            estimator.setResultRefined(false);
            estimator.setCovarianceKept(false);            
            estimator.setSuggestHorizontalFocalLengthEnabled(true);
            estimator.setSuggestedHorizontalFocalLengthValue(
                    horizontalFocalLength);

            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
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
            assertNull(estimator.getCovariance());            
            reset();

            //decompone estimated camera and check its parameters
            estimatedCamera.decompose();
            
            //comparing camera intrinsic parameters
            PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    estimatedCamera.getIntrinsicParameters();            
            double estimatedHorizontalFocalLength = 
                    estimatedIntrinsic.getHorizontalFocalLength();
            
            //estimate without suggestion
            estimator.setSuggestHorizontalFocalLengthEnabled(false);
            
            PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }         
            
            estimatedCameraNoSuggestion.decompose();
            
            PinholeCameraIntrinsicParameters estimatedIntrinsicNoSuggestion =
                    estimatedCameraNoSuggestion.getIntrinsicParameters();
            double estimatedHorizontalFocalLengthNoSuggestion =
                    estimatedIntrinsicNoSuggestion.getHorizontalFocalLength();
            
            //check that horizontal focal length has become closer to suggested 
            //value
            if (Math.abs(horizontalFocalLength - estimatedHorizontalFocalLengthNoSuggestion) >=
                    Math.abs(horizontalFocalLength - estimatedHorizontalFocalLength)) {
                numValid++;
            }          
            
            if (numValid > 0) {
                break;
            }
        }
        
        assertTrue(numValid > 0);
    }

    @Test
    public void testEstimateSuggestedHorizontalFocalLengthWithRefinement() 
            throws IllegalArgumentException, LockedException, NotReadyException,
            RobustEstimatorException, CameraException, NotAvailableException {
        int numCovariances = 0;
        int numValid = 0;        
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
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
            double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler, 
                    gammaEuler);
            
            //create camera center
            double[] cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);

            //normalize the camera to improve accuracy
            camera.normalize();            
            
            int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            List<Line2D> lines = new ArrayList<Line2D>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE));                        
                lines.add(line);
            }
            
            List<Plane> planes = camera.backProjectLines(lines);
            
            //create outliers
            Plane planeWithError;
            List<Plane> planesWithError = new ArrayList<Plane>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR); 
            double[] qualityScores = new double[nSamples];
            int j = 0;            
            for (Plane plane : planes) {
                double scoreError = randomizer.nextDouble(MIN_SCORE_ERROR, 
                        MAX_SCORE_ERROR);
                qualityScores[j] = 1.0 + scoreError;                
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //plane is oulier
                    double errorA = errorRandomizer.nextDouble();
                    double errorB = errorRandomizer.nextDouble();
                    double errorC = errorRandomizer.nextDouble();
                    double errorD = errorRandomizer.nextDouble();
                    planeWithError = new Plane(plane.getA() + errorA,
                            plane.getB() + errorB, plane.getC() + errorC,
                            plane.getD() + errorD);
                    
                    double error = Math.sqrt(errorA * errorA + errorB * errorB +
                            errorC * errorC + errorD * errorD);
                    qualityScores[j] = 1.0 / (1.0 + error) + scoreError;
                } else {
                    //inlier plane (without error)
                    planeWithError = plane;
                }
                
                planesWithError.add(planeWithError);
                j++;
            }
            
            PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                    this, planesWithError, lines, qualityScores);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);            
            estimator.setSuggestHorizontalFocalLengthEnabled(true);
            estimator.setSuggestedHorizontalFocalLengthValue(
                    horizontalFocalLength);

            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
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
            
            //comparing camera intrinsic parameters
            PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    estimatedCamera.getIntrinsicParameters();            
            double estimatedHorizontalFocalLength = 
                    estimatedIntrinsic.getHorizontalFocalLength();
            
            //estimate without suggestion
            estimator.setSuggestHorizontalFocalLengthEnabled(false);
            
            PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }         
            
            estimatedCameraNoSuggestion.decompose();
            
            PinholeCameraIntrinsicParameters estimatedIntrinsicNoSuggestion =
                    estimatedCameraNoSuggestion.getIntrinsicParameters();
            double estimatedHorizontalFocalLengthNoSuggestion =
                    estimatedIntrinsicNoSuggestion.getHorizontalFocalLength();
            
            //check that horizontal focal length has become closer to suggested 
            //value
            if (Math.abs(horizontalFocalLength - estimatedHorizontalFocalLengthNoSuggestion) >=
                    Math.abs(horizontalFocalLength - estimatedHorizontalFocalLength)) {
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
    public void testEstimateSuggestedHorizontalFocalLengthWithFastRefinement() 
            throws IllegalArgumentException, LockedException, NotReadyException,
            RobustEstimatorException, CameraException, NotAvailableException {
        int numCovariances = 0;
        int numValid = 0;        
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
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
            double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler, 
                    gammaEuler);
            
            //create camera center
            double[] cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);

            //normalize the camera to improve accuracy
            camera.normalize();            
            
            int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            List<Line2D> lines = new ArrayList<Line2D>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE));                        
                lines.add(line);
            }
            
            List<Plane> planes = camera.backProjectLines(lines);
            
            //create outliers
            Plane planeWithError;
            List<Plane> planesWithError = new ArrayList<Plane>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR); 
            double[] qualityScores = new double[nSamples];
            int j = 0;            
            for (Plane plane : planes) {
                double scoreError = randomizer.nextDouble(MIN_SCORE_ERROR, 
                        MAX_SCORE_ERROR);
                qualityScores[j] = 1.0 + scoreError;                
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //plane is oulier
                    double errorA = errorRandomizer.nextDouble();
                    double errorB = errorRandomizer.nextDouble();
                    double errorC = errorRandomizer.nextDouble();
                    double errorD = errorRandomizer.nextDouble();
                    planeWithError = new Plane(plane.getA() + errorA,
                            plane.getB() + errorB, plane.getC() + errorC,
                            plane.getD() + errorD);
                    
                    double error = Math.sqrt(errorA * errorA + errorB * errorB +
                            errorC * errorC + errorD * errorD);
                    qualityScores[j] = 1.0 / (1.0 + error) + scoreError;
                } else {
                    //inlier plane (without error)
                    planeWithError = plane;
                }
                
                planesWithError.add(planeWithError);
                j++;
            }
            
            PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                    this, planesWithError, lines, qualityScores);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);
            estimator.setResultRefined(true);
            estimator.setFastRefinementUsed(true);
            estimator.setCovarianceKept(true);            
            estimator.setSuggestHorizontalFocalLengthEnabled(true);
            estimator.setSuggestedHorizontalFocalLengthValue(
                    horizontalFocalLength);
            
            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
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
            
            //comparing camera intrinsic parameters
            PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    estimatedCamera.getIntrinsicParameters();            
            double estimatedHorizontalFocalLength = 
                    estimatedIntrinsic.getHorizontalFocalLength();

            
            //estimate without suggestion
            estimator.setSuggestHorizontalFocalLengthEnabled(false);
            
            PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }         
            
            estimatedCameraNoSuggestion.decompose();
            
            PinholeCameraIntrinsicParameters estimatedIntrinsicNoSuggestion =
                    estimatedCameraNoSuggestion.getIntrinsicParameters();
            double estimatedHorizontalFocalLengthNoSuggestion =
                    estimatedIntrinsicNoSuggestion.getHorizontalFocalLength();

            //check that horizontal focal length has become closer to suggested 
            //value
            if (Math.abs(horizontalFocalLength - estimatedHorizontalFocalLengthNoSuggestion) >=
                    Math.abs(horizontalFocalLength - estimatedHorizontalFocalLength)) {
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
    public void testEstimateSuggestedVerticalFocalLengthEnabled() 
            throws IllegalArgumentException, LockedException, NotReadyException, 
            RobustEstimatorException, CameraException, NotAvailableException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
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
            double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler, 
                    gammaEuler);
            
            //create camera center
            double[] cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);

            //normalize the camera to improve accuracy
            camera.normalize();            
            
            int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            List<Line2D> lines = new ArrayList<Line2D>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE));                        
                lines.add(line);
            }
            
            List<Plane> planes = camera.backProjectLines(lines);
            
            //create outliers
            Plane planeWithError;
            List<Plane> planesWithError = new ArrayList<Plane>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR); 
            double[] qualityScores = new double[nSamples];
            int j = 0;            
            for (Plane plane : planes) {
                double scoreError = randomizer.nextDouble(MIN_SCORE_ERROR, 
                        MAX_SCORE_ERROR);
                qualityScores[j] = 1.0 + scoreError;                
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //plane is oulier
                    double errorA = errorRandomizer.nextDouble();
                    double errorB = errorRandomizer.nextDouble();
                    double errorC = errorRandomizer.nextDouble();
                    double errorD = errorRandomizer.nextDouble();
                    planeWithError = new Plane(plane.getA() + errorA,
                            plane.getB() + errorB, plane.getC() + errorC,
                            plane.getD() + errorD);
                    
                    double error = Math.sqrt(errorA * errorA + errorB * errorB +
                            errorC * errorC + errorD * errorD);
                    qualityScores[j] = 1.0 / (1.0 + error) + scoreError;
                } else {
                    //inlier plane (without error)
                    planeWithError = plane;
                }
                
                planesWithError.add(planeWithError);
                j++;
            }
            
            PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                    this, planesWithError, lines, qualityScores);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);
            estimator.setResultRefined(false);
            estimator.setCovarianceKept(false);            
            estimator.setSuggestVerticalFocalLengthEnabled(true);
            estimator.setSuggestedVerticalFocalLengthValue(
                    verticalFocalLength);

            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
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
            assertNull(estimator.getCovariance());            
            reset();

            //decompone estimated camera and check its parameters
            estimatedCamera.decompose();
            
            //comparing camera intrinsic parameters
            PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    estimatedCamera.getIntrinsicParameters();            
            double estimatedVerticalFocalLength = 
                    estimatedIntrinsic.getVerticalFocalLength();
            
            //estimate without suggestion
            estimator.setSuggestVerticalFocalLengthEnabled(false);
            
            PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }         
            
            estimatedCameraNoSuggestion.decompose();
            
            PinholeCameraIntrinsicParameters estimatedIntrinsicNoSuggestion =
                    estimatedCameraNoSuggestion.getIntrinsicParameters();
            double estimatedVerticalFocalLengthNoSuggestion =
                    estimatedIntrinsicNoSuggestion.getVerticalFocalLength();
            
            //check that vertical focal length has become closer to suggested 
            //value
            if (Math.abs(verticalFocalLength - estimatedVerticalFocalLengthNoSuggestion) >=
                    Math.abs(verticalFocalLength - estimatedVerticalFocalLength)) {
                numValid++;
            }            
            
            if (numValid > 0) {
                break;
            }
        }
        
        assertTrue(numValid > 0);
    }

    @Test
    public void testEstimateSuggestedVerticalFocalLengthWithRefinement() 
            throws IllegalArgumentException, LockedException, NotReadyException,
            RobustEstimatorException, CameraException, NotAvailableException {
        int numCovariances = 0;
        int numValid = 0;        
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
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
            double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler, 
                    gammaEuler);
            
            //create camera center
            double[] cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);

            //normalize the camera to improve accuracy
            camera.normalize();            
            
            int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            List<Line2D> lines = new ArrayList<Line2D>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE));                        
                lines.add(line);
            }
            
            List<Plane> planes = camera.backProjectLines(lines);
            
            //create outliers
            Plane planeWithError;
            List<Plane> planesWithError = new ArrayList<Plane>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR); 
            double[] qualityScores = new double[nSamples];
            int j = 0;            
            for (Plane plane : planes) {
                double scoreError = randomizer.nextDouble(MIN_SCORE_ERROR, 
                        MAX_SCORE_ERROR);
                qualityScores[j] = 1.0 + scoreError;                
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //plane is oulier
                    double errorA = errorRandomizer.nextDouble();
                    double errorB = errorRandomizer.nextDouble();
                    double errorC = errorRandomizer.nextDouble();
                    double errorD = errorRandomizer.nextDouble();
                    planeWithError = new Plane(plane.getA() + errorA,
                            plane.getB() + errorB, plane.getC() + errorC,
                            plane.getD() + errorD);
                    
                    double error = Math.sqrt(errorA * errorA + errorB * errorB +
                            errorC * errorC + errorD * errorD);
                    qualityScores[j] = 1.0 / (1.0 + error) + scoreError;
                } else {
                    //inlier plane (without error)
                    planeWithError = plane;
                }
                
                planesWithError.add(planeWithError);
                j++;
            }
            
            PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                    this, planesWithError, lines, qualityScores);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);            
            estimator.setSuggestVerticalFocalLengthEnabled(true);
            estimator.setSuggestedVerticalFocalLengthValue(
                    verticalFocalLength);

            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
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
            
            //comparing camera intrinsic parameters
            PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    estimatedCamera.getIntrinsicParameters();            
            double estimatedVerticalFocalLength = 
                    estimatedIntrinsic.getVerticalFocalLength();
            
            //estimate without suggestion
            estimator.setSuggestVerticalFocalLengthEnabled(false);
            
            PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }         
            
            estimatedCameraNoSuggestion.decompose();
            
            PinholeCameraIntrinsicParameters estimatedIntrinsicNoSuggestion =
                    estimatedCameraNoSuggestion.getIntrinsicParameters();
            double estimatedVerticalFocalLengthNoSuggestion =
                    estimatedIntrinsicNoSuggestion.getVerticalFocalLength();
            
            //check that horizontal focal length has become closer to suggested 
            //value
            if (Math.abs(verticalFocalLength - estimatedVerticalFocalLengthNoSuggestion) >=
                    Math.abs(verticalFocalLength - estimatedVerticalFocalLength)) {
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
    public void testEstimateSuggestedVerticalFocalLengthWithFastRefinement() 
            throws IllegalArgumentException, LockedException, NotReadyException,
            RobustEstimatorException, CameraException, NotAvailableException {
        int numCovariances = 0;
        int numValid = 0;        
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
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
            double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler, 
                    gammaEuler);
            
            //create camera center
            double[] cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);

            //normalize the camera to improve accuracy
            camera.normalize();            
            
            int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            List<Line2D> lines = new ArrayList<Line2D>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE));                        
                lines.add(line);
            }
            
            List<Plane> planes = camera.backProjectLines(lines);
            
            //create outliers
            Plane planeWithError;
            List<Plane> planesWithError = new ArrayList<Plane>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR); 
            double[] qualityScores = new double[nSamples];
            int j = 0;            
            for (Plane plane : planes) {
                double scoreError = randomizer.nextDouble(MIN_SCORE_ERROR, 
                        MAX_SCORE_ERROR);
                qualityScores[j] = 1.0 + scoreError;                
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //plane is oulier
                    double errorA = errorRandomizer.nextDouble();
                    double errorB = errorRandomizer.nextDouble();
                    double errorC = errorRandomizer.nextDouble();
                    double errorD = errorRandomizer.nextDouble();
                    planeWithError = new Plane(plane.getA() + errorA,
                            plane.getB() + errorB, plane.getC() + errorC,
                            plane.getD() + errorD);
                    
                    double error = Math.sqrt(errorA * errorA + errorB * errorB +
                            errorC * errorC + errorD * errorD);
                    qualityScores[j] = 1.0 / (1.0 + error) + scoreError;
                } else {
                    //inlier plane (without error)
                    planeWithError = plane;
                }
                
                planesWithError.add(planeWithError);
                j++;
            }
            
            PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                    this, planesWithError, lines, qualityScores);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);
            estimator.setResultRefined(true);
            estimator.setFastRefinementUsed(true);
            estimator.setCovarianceKept(true);            
            estimator.setSuggestVerticalFocalLengthEnabled(true);
            estimator.setSuggestedVerticalFocalLengthValue(
                    verticalFocalLength);
            
            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
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
            
            //comparing camera intrinsic parameters
            PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    estimatedCamera.getIntrinsicParameters();            
            double estimatedVerticalFocalLength = 
                    estimatedIntrinsic.getVerticalFocalLength();
            
            //estimate without suggestion
            estimator.setSuggestVerticalFocalLengthEnabled(false);
            
            PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }         
            
            estimatedCameraNoSuggestion.decompose();
            
            PinholeCameraIntrinsicParameters estimatedIntrinsicNoSuggestion =
                    estimatedCameraNoSuggestion.getIntrinsicParameters();
            double estimatedVerticalFocalLengthNoSuggestion =
                    estimatedIntrinsicNoSuggestion.getVerticalFocalLength();
            
            //check that vertical focal length has become closer to suggested 
            //value
            if (Math.abs(horizontalFocalLength - estimatedVerticalFocalLengthNoSuggestion) >=
                    Math.abs(horizontalFocalLength - estimatedVerticalFocalLength)) {
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
    public void testEstimateSuggestedAspectRatioEnabled() 
            throws IllegalArgumentException, LockedException, NotReadyException, 
            RobustEstimatorException, CameraException, NotAvailableException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
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
            
            double aspectRatio = intrinsic.getAspectRatio();
            
            //create rotation parameters
            double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler, 
                    gammaEuler);
            
            //create camera center
            double[] cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);

            //normalize the camera to improve accuracy
            camera.normalize();            
            
            int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            List<Line2D> lines = new ArrayList<Line2D>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE));                        
                lines.add(line);
            }
            
            List<Plane> planes = camera.backProjectLines(lines);
            
            //create outliers
            Plane planeWithError;
            List<Plane> planesWithError = new ArrayList<Plane>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR); 
            double[] qualityScores = new double[nSamples];
            int j = 0;            
            for (Plane plane : planes) {
                double scoreError = randomizer.nextDouble(MIN_SCORE_ERROR, 
                        MAX_SCORE_ERROR);
                qualityScores[j] = 1.0 + scoreError;                
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //plane is oulier
                    double errorA = errorRandomizer.nextDouble();
                    double errorB = errorRandomizer.nextDouble();
                    double errorC = errorRandomizer.nextDouble();
                    double errorD = errorRandomizer.nextDouble();
                    planeWithError = new Plane(plane.getA() + errorA,
                            plane.getB() + errorB, plane.getC() + errorC,
                            plane.getD() + errorD);
                    
                    double error = Math.sqrt(errorA * errorA + errorB * errorB +
                            errorC * errorC + errorD * errorD);
                    qualityScores[j] = 1.0 / (1.0 + error) + scoreError;
                } else {
                    //inlier plane (without error)
                    planeWithError = plane;
                }
                
                planesWithError.add(planeWithError);
                j++;
            }
            
            PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                    this, planesWithError, lines, qualityScores);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);
            estimator.setResultRefined(false);
            estimator.setCovarianceKept(false);            
            estimator.setSuggestAspectRatioEnabled(true);
            estimator.setSuggestedAspectRatioValue(aspectRatio);

            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
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
            assertNull(estimator.getCovariance());            
            reset();

            //decompone estimated camera and check its parameters
            estimatedCamera.decompose();
            
            //comparing camera intrinsic parameters
            PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    estimatedCamera.getIntrinsicParameters();            
            double estimatedAspectRatio = estimatedIntrinsic.getAspectRatio();
            
            //estimate without suggestion
            estimator.setSuggestAspectRatioEnabled(false);
            
            PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }         
            
            estimatedCameraNoSuggestion.decompose();
            
            PinholeCameraIntrinsicParameters estimatedIntrinsicNoSuggestion =
                    estimatedCameraNoSuggestion.getIntrinsicParameters();
            double estimatedAspectRatioNoSuggestion =
                    estimatedIntrinsicNoSuggestion.getAspectRatio();
            
            //check that aspect ratio has become closer to suggested
            //value
            if (Math.abs(aspectRatio - estimatedAspectRatioNoSuggestion) >=
                    Math.abs(aspectRatio - estimatedAspectRatio)) {
                numValid++;
            }        
            
            if (numValid > 0) {
                break;
            }
        }
        
        assertTrue(numValid > 0);
    }

    @Test
    public void testEstimateSuggestedAspectRatioWithRefinement() 
            throws IllegalArgumentException, LockedException, NotReadyException,
            RobustEstimatorException, CameraException, NotAvailableException {
        int numCovariances = 0;
        int numValid = 0;        
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
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
            
            double aspectRatio = intrinsic.getAspectRatio();
            
            //create rotation parameters
            double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler, 
                    gammaEuler);
            
            //create camera center
            double[] cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);

            //normalize the camera to improve accuracy
            camera.normalize();            
            
            int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            List<Line2D> lines = new ArrayList<Line2D>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE));                        
                lines.add(line);
            }
            
            List<Plane> planes = camera.backProjectLines(lines);
            
            //create outliers
            Plane planeWithError;
            List<Plane> planesWithError = new ArrayList<Plane>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR); 
            double[] qualityScores = new double[nSamples];
            int j = 0;            
            for (Plane plane : planes) {
                double scoreError = randomizer.nextDouble(MIN_SCORE_ERROR, 
                        MAX_SCORE_ERROR);
                qualityScores[j] = 1.0 + scoreError;                
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //plane is oulier
                    double errorA = errorRandomizer.nextDouble();
                    double errorB = errorRandomizer.nextDouble();
                    double errorC = errorRandomizer.nextDouble();
                    double errorD = errorRandomizer.nextDouble();
                    planeWithError = new Plane(plane.getA() + errorA,
                            plane.getB() + errorB, plane.getC() + errorC,
                            plane.getD() + errorD);
                    
                    double error = Math.sqrt(errorA * errorA + errorB * errorB +
                            errorC * errorC + errorD * errorD);
                    qualityScores[j] = 1.0 / (1.0 + error) + scoreError;
                } else {
                    //inlier plane (without error)
                    planeWithError = plane;
                }
                
                planesWithError.add(planeWithError);
                j++;
            }
            
            PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                    this, planesWithError, lines, qualityScores);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);            
            estimator.setSuggestAspectRatioEnabled(true);
            estimator.setSuggestedAspectRatioValue(aspectRatio);

            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
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
            
            //comparing camera intrinsic parameters
            PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    estimatedCamera.getIntrinsicParameters();            
            double estimatedAspectRatio = estimatedIntrinsic.getAspectRatio();
            
            //estimate without suggestion
            estimator.setSuggestAspectRatioEnabled(false);
            
            PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }         
            
            estimatedCameraNoSuggestion.decompose();
            
            PinholeCameraIntrinsicParameters estimatedIntrinsicNoSuggestion =
                    estimatedCameraNoSuggestion.getIntrinsicParameters();
            double estimatedAspectRatioNoSuggestion =
                    estimatedIntrinsicNoSuggestion.getAspectRatio();
            
            //check that aspect ratio has become closer to suggested
            //value
            if (Math.abs(aspectRatio - estimatedAspectRatioNoSuggestion) >=
                    Math.abs(aspectRatio - estimatedAspectRatio)) {
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
    public void testEstimateSuggestedAspectRatioWithFastRefinement() 
            throws IllegalArgumentException, LockedException, NotReadyException,
            RobustEstimatorException, CameraException, NotAvailableException {
        int numCovariances = 0;
        int numValid = 0;        
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
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
            
            double aspectRatio = intrinsic.getAspectRatio();
            
            //create rotation parameters
            double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler, 
                    gammaEuler);
            
            //create camera center
            double[] cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);

            //normalize the camera to improve accuracy
            camera.normalize();            
            
            int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            List<Line2D> lines = new ArrayList<Line2D>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE));                        
                lines.add(line);
            }
            
            List<Plane> planes = camera.backProjectLines(lines);
            
            //create outliers
            Plane planeWithError;
            List<Plane> planesWithError = new ArrayList<Plane>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR); 
            double[] qualityScores = new double[nSamples];
            int j = 0;            
            for (Plane plane : planes) {
                double scoreError = randomizer.nextDouble(MIN_SCORE_ERROR, 
                        MAX_SCORE_ERROR);
                qualityScores[j] = 1.0 + scoreError;                
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //plane is oulier
                    double errorA = errorRandomizer.nextDouble();
                    double errorB = errorRandomizer.nextDouble();
                    double errorC = errorRandomizer.nextDouble();
                    double errorD = errorRandomizer.nextDouble();
                    planeWithError = new Plane(plane.getA() + errorA,
                            plane.getB() + errorB, plane.getC() + errorC,
                            plane.getD() + errorD);
                    
                    double error = Math.sqrt(errorA * errorA + errorB * errorB +
                            errorC * errorC + errorD * errorD);
                    qualityScores[j] = 1.0 / (1.0 + error) + scoreError;
                } else {
                    //inlier plane (without error)
                    planeWithError = plane;
                }
                
                planesWithError.add(planeWithError);
                j++;
            }
            
            PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                    this, planesWithError, lines, qualityScores);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);
            estimator.setResultRefined(true);
            estimator.setFastRefinementUsed(true);
            estimator.setCovarianceKept(true);            
            estimator.setSuggestAspectRatioEnabled(true);
            estimator.setSuggestedAspectRatioValue(aspectRatio);
            
            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
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
            
            //comparing camera intrinsic parameters
            PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    estimatedCamera.getIntrinsicParameters();            
            double estimatedAspectRatio = estimatedIntrinsic.getAspectRatio();

            //estimate without suggestion
            estimator.setSuggestAspectRatioEnabled(false);
            
            PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }         
            
            estimatedCameraNoSuggestion.decompose();
            
            PinholeCameraIntrinsicParameters estimatedIntrinsicNoSuggestion =
                    estimatedCameraNoSuggestion.getIntrinsicParameters();
            double estimatedAspectRatioNoSuggestion =
                    estimatedIntrinsicNoSuggestion.getAspectRatio();
            
            //check that aspect ratio has become closer to suggested
            //value
            if (Math.abs(aspectRatio - estimatedAspectRatioNoSuggestion) >=
                    Math.abs(aspectRatio - estimatedAspectRatio)) {
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
    public void testEstimateSuggestedPrincipalPointEnabled() 
            throws IllegalArgumentException, LockedException, NotReadyException, 
            RobustEstimatorException, CameraException, NotAvailableException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            InhomogeneousPoint2D principalPoint = new InhomogeneousPoint2D(
                    horizontalPrincipalPoint, verticalPrincipalPoint);

            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                    verticalFocalLength, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, skewness);
            
            
            //create rotation parameters
            double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler, 
                    gammaEuler);
            
            //create camera center
            double[] cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);

            //normalize the camera to improve accuracy
            camera.normalize();            
            
            int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            List<Line2D> lines = new ArrayList<Line2D>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE));                        
                lines.add(line);
            }
            
            List<Plane> planes = camera.backProjectLines(lines);
            
            //create outliers
            Plane planeWithError;
            List<Plane> planesWithError = new ArrayList<Plane>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR); 
            double[] qualityScores = new double[nSamples];
            int j = 0;            
            for (Plane plane : planes) {
                double scoreError = randomizer.nextDouble(MIN_SCORE_ERROR, 
                        MAX_SCORE_ERROR);
                qualityScores[j] = 1.0 + scoreError;                
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //plane is oulier
                    double errorA = errorRandomizer.nextDouble();
                    double errorB = errorRandomizer.nextDouble();
                    double errorC = errorRandomizer.nextDouble();
                    double errorD = errorRandomizer.nextDouble();
                    planeWithError = new Plane(plane.getA() + errorA,
                            plane.getB() + errorB, plane.getC() + errorC,
                            plane.getD() + errorD);
                    
                    double error = Math.sqrt(errorA * errorA + errorB * errorB +
                            errorC * errorC + errorD * errorD);
                    qualityScores[j] = 1.0 / (1.0 + error) + scoreError;
                } else {
                    //inlier plane (without error)
                    planeWithError = plane;
                }
                
                planesWithError.add(planeWithError);
                j++;
            }
            
            PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                    this, planesWithError, lines, qualityScores);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);
            estimator.setResultRefined(false);
            estimator.setCovarianceKept(false);            
            estimator.setSuggestPrincipalPointEnabled(true);
            estimator.setSuggestedPrincipalPointValue(new InhomogeneousPoint2D(
                    horizontalPrincipalPoint, verticalPrincipalPoint));

            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
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
            assertNull(estimator.getCovariance());            
            reset();

            //decompone estimated camera and check its parameters
            estimatedCamera.decompose();
            
            //comparing camera intrinsic parameters
            PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    estimatedCamera.getIntrinsicParameters();            
            InhomogeneousPoint2D estimatedPrincipalPoint = 
                    new InhomogeneousPoint2D(
                            estimatedIntrinsic.getHorizontalPrincipalPoint(), 
                            estimatedIntrinsic.getVerticalPrincipalPoint());            
            
            //estimate without suggestion
            estimator.setSuggestPrincipalPointEnabled(false);
            
            PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }         
            
            estimatedCameraNoSuggestion.decompose();
            
            PinholeCameraIntrinsicParameters estimatedIntrinsicNoSuggestion =
                    estimatedCameraNoSuggestion.getIntrinsicParameters();
            InhomogeneousPoint2D estimatedPrincipalPointNoSuggestion =
                    new InhomogeneousPoint2D(
                            estimatedIntrinsicNoSuggestion.getHorizontalPrincipalPoint(), 
                            estimatedIntrinsicNoSuggestion.getVerticalPrincipalPoint());
            
            //check that principal point has become closer to suggested value
            if (principalPoint.distanceTo(estimatedPrincipalPointNoSuggestion) >=
                    principalPoint.distanceTo(estimatedPrincipalPoint)) {
                numValid++;
            }           
            
            if (numValid > 0) {
                break;
            }
        }
        
        assertTrue(numValid > 0);
    }

    @Test
    public void testEstimateSuggestedPrincipalPointWithRefinement() 
            throws IllegalArgumentException, LockedException, NotReadyException,
            RobustEstimatorException, CameraException, NotAvailableException {
        int numCovariances = 0;
        int numValid = 0;        
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            InhomogeneousPoint2D principalPoint = new InhomogeneousPoint2D(
                    horizontalPrincipalPoint, verticalPrincipalPoint);

            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                    verticalFocalLength, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, skewness);
            
            
            //create rotation parameters
            double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler, 
                    gammaEuler);
            
            //create camera center
            double[] cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);

            //normalize the camera to improve accuracy
            camera.normalize();            
            
            int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            List<Line2D> lines = new ArrayList<Line2D>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE));                        
                lines.add(line);
            }
            
            List<Plane> planes = camera.backProjectLines(lines);
            
            //create outliers
            Plane planeWithError;
            List<Plane> planesWithError = new ArrayList<Plane>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR); 
            double[] qualityScores = new double[nSamples];
            int j = 0;            
            for (Plane plane : planes) {
                double scoreError = randomizer.nextDouble(MIN_SCORE_ERROR, 
                        MAX_SCORE_ERROR);
                qualityScores[j] = 1.0 + scoreError;                
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //plane is oulier
                    double errorA = errorRandomizer.nextDouble();
                    double errorB = errorRandomizer.nextDouble();
                    double errorC = errorRandomizer.nextDouble();
                    double errorD = errorRandomizer.nextDouble();
                    planeWithError = new Plane(plane.getA() + errorA,
                            plane.getB() + errorB, plane.getC() + errorC,
                            plane.getD() + errorD);
                    
                    double error = Math.sqrt(errorA * errorA + errorB * errorB +
                            errorC * errorC + errorD * errorD);
                    qualityScores[j] = 1.0 / (1.0 + error) + scoreError;
                } else {
                    //inlier plane (without error)
                    planeWithError = plane;
                }
                
                planesWithError.add(planeWithError);
                j++;
            }
            
            PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                    this, planesWithError, lines, qualityScores);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);            
            estimator.setSuggestPrincipalPointEnabled(true);
            estimator.setSuggestedPrincipalPointValue(new InhomogeneousPoint2D(
                    horizontalPrincipalPoint, verticalPrincipalPoint));

            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
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
            
            //comparing camera intrinsic parameters
            PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    estimatedCamera.getIntrinsicParameters();            
            InhomogeneousPoint2D estimatedPrincipalPoint = 
                    new InhomogeneousPoint2D(
                            estimatedIntrinsic.getHorizontalPrincipalPoint(), 
                            estimatedIntrinsic.getVerticalPrincipalPoint());
            
            //estimate without suggestion
            estimator.setSuggestPrincipalPointEnabled(false);
            
            PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }         
            
            estimatedCameraNoSuggestion.decompose();
            
            PinholeCameraIntrinsicParameters estimatedIntrinsicNoSuggestion =
                    estimatedCameraNoSuggestion.getIntrinsicParameters();
            InhomogeneousPoint2D estimatedPrincipalPointNoSuggestion =
                    new InhomogeneousPoint2D(
                            estimatedIntrinsicNoSuggestion.getHorizontalPrincipalPoint(), 
                            estimatedIntrinsicNoSuggestion.getVerticalPrincipalPoint());
            
            //check that principal point has become closer to suggested value
            if (principalPoint.distanceTo(estimatedPrincipalPointNoSuggestion) >=
                    principalPoint.distanceTo(estimatedPrincipalPoint)) {
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
    public void testEstimateSuggestedPrincipalPointWithFastRefinement() 
            throws IllegalArgumentException, LockedException, NotReadyException,
            RobustEstimatorException, CameraException, NotAvailableException {
        int numCovariances = 0;
        int numValid = 0;        
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            InhomogeneousPoint2D principalPoint = new InhomogeneousPoint2D(
                    horizontalPrincipalPoint, verticalPrincipalPoint);

            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                    verticalFocalLength, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, skewness);
            
            
            //create rotation parameters
            double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler, 
                    gammaEuler);
            
            //create camera center
            double[] cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);

            //normalize the camera to improve accuracy
            camera.normalize();            
            
            int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            List<Line2D> lines = new ArrayList<Line2D>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE));                        
                lines.add(line);
            }
            
            List<Plane> planes = camera.backProjectLines(lines);
            
            //create outliers
            Plane planeWithError;
            List<Plane> planesWithError = new ArrayList<Plane>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR); 
            double[] qualityScores = new double[nSamples];
            int j = 0;            
            for (Plane plane : planes) {
                double scoreError = randomizer.nextDouble(MIN_SCORE_ERROR, 
                        MAX_SCORE_ERROR);
                qualityScores[j] = 1.0 + scoreError;                
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //plane is oulier
                    double errorA = errorRandomizer.nextDouble();
                    double errorB = errorRandomizer.nextDouble();
                    double errorC = errorRandomizer.nextDouble();
                    double errorD = errorRandomizer.nextDouble();
                    planeWithError = new Plane(plane.getA() + errorA,
                            plane.getB() + errorB, plane.getC() + errorC,
                            plane.getD() + errorD);
                    
                    double error = Math.sqrt(errorA * errorA + errorB * errorB +
                            errorC * errorC + errorD * errorD);
                    qualityScores[j] = 1.0 / (1.0 + error) + scoreError;
                } else {
                    //inlier plane (without error)
                    planeWithError = plane;
                }
                
                planesWithError.add(planeWithError);
                j++;
            }
            
            PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                    this, planesWithError, lines, qualityScores);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);
            estimator.setResultRefined(true);
            estimator.setFastRefinementUsed(true);
            estimator.setCovarianceKept(true);            
            estimator.setSuggestPrincipalPointEnabled(true);
            estimator.setSuggestedPrincipalPointValue(new InhomogeneousPoint2D(
                    horizontalPrincipalPoint, verticalPrincipalPoint));
            
            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
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
            
            //comparing camera intrinsic parameters
            PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    estimatedCamera.getIntrinsicParameters();            
            InhomogeneousPoint2D estimatedPrincipalPoint = 
                    new InhomogeneousPoint2D(
                            estimatedIntrinsic.getHorizontalPrincipalPoint(), 
                            estimatedIntrinsic.getVerticalPrincipalPoint());
            
            //estimate without suggestion
            estimator.setSuggestPrincipalPointEnabled(false);
            
            PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }         
            
            estimatedCameraNoSuggestion.decompose();
            
            PinholeCameraIntrinsicParameters estimatedIntrinsicNoSuggestion =
                    estimatedCameraNoSuggestion.getIntrinsicParameters();
            InhomogeneousPoint2D estimatedPrincipalPointNoSuggestion =
                    new InhomogeneousPoint2D(
                            estimatedIntrinsicNoSuggestion.getHorizontalPrincipalPoint(), 
                            estimatedIntrinsicNoSuggestion.getVerticalPrincipalPoint());
            
            //check that principal point has become closer to suggested value
            if (principalPoint.distanceTo(estimatedPrincipalPointNoSuggestion) >=
                    principalPoint.distanceTo(estimatedPrincipalPoint)) {
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
    public void testEstimateSuggestedRotationEnabled() 
            throws IllegalArgumentException, LockedException, NotReadyException, 
            RobustEstimatorException, CameraException, NotAvailableException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
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
            double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler, 
                    gammaEuler);
            Quaternion q = rotation.toQuaternion();
            q.normalize();            
            
            //create camera center
            double[] cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);

            //normalize the camera to improve accuracy
            camera.normalize();            
            
            int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            List<Line2D> lines = new ArrayList<Line2D>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE));                        
                lines.add(line);
            }
            
            List<Plane> planes = camera.backProjectLines(lines);
            
            //create outliers
            Plane planeWithError;
            List<Plane> planesWithError = new ArrayList<Plane>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR); 
            double[] qualityScores = new double[nSamples];
            int j = 0;            
            for (Plane plane : planes) {
                double scoreError = randomizer.nextDouble(MIN_SCORE_ERROR, 
                        MAX_SCORE_ERROR);
                qualityScores[j] = 1.0 + scoreError;                
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //plane is oulier
                    double errorA = errorRandomizer.nextDouble();
                    double errorB = errorRandomizer.nextDouble();
                    double errorC = errorRandomizer.nextDouble();
                    double errorD = errorRandomizer.nextDouble();
                    planeWithError = new Plane(plane.getA() + errorA,
                            plane.getB() + errorB, plane.getC() + errorC,
                            plane.getD() + errorD);
                    
                    double error = Math.sqrt(errorA * errorA + errorB * errorB +
                            errorC * errorC + errorD * errorD);
                    qualityScores[j] = 1.0 / (1.0 + error) + scoreError;
                } else {
                    //inlier plane (without error)
                    planeWithError = plane;
                }
                
                planesWithError.add(planeWithError);
                j++;
            }
            
            PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                    this, planesWithError, lines, qualityScores);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);
            estimator.setResultRefined(false);
            estimator.setCovarianceKept(false);            
            estimator.setSuggestRotationEnabled(true);
            estimator.setSuggestedRotationValue(q);

            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
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
            assertNull(estimator.getCovariance());            
            reset();

            //decompone estimated camera and check its parameters
            estimatedCamera.decompose();
            
            //comparing rotation
            Quaternion estimatedQ = estimatedCamera.getCameraRotation().
                    toQuaternion();
            estimatedQ.normalize();
            
            //estimate without suggestion
            estimator.setSuggestRotationEnabled(false);
            
            PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }         
            
            estimatedCameraNoSuggestion.decompose();
            
            Quaternion estimatedQNoSuggestion = estimatedCameraNoSuggestion.
                    getCameraRotation().toQuaternion();
            
            //check that rotation has become closer to suggested value
            double diffEstimatedNoSuggestion = 
                    Math.pow(q.getA() - estimatedQNoSuggestion.getA(), 2.0) +
                    Math.pow(q.getB() - estimatedQNoSuggestion.getB(), 2.0) +
                    Math.pow(q.getC() - estimatedQNoSuggestion.getC(), 2.0) +
                    Math.pow(q.getD() - estimatedQNoSuggestion.getD(), 2.0);
            double diffEstimated =
                    Math.pow(q.getA() - estimatedQ.getA(), 2.0) +
                    Math.pow(q.getB() - estimatedQ.getB(), 2.0) +
                    Math.pow(q.getC() - estimatedQ.getC(), 2.0) +
                    Math.pow(q.getD() - estimatedQ.getD(), 2.0);
            
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
    public void testEstimateSuggestedRotationWithRefinement() 
            throws IllegalArgumentException, LockedException, NotReadyException,
            RobustEstimatorException, CameraException, NotAvailableException {
        int numCovariances = 0;
        int numValid = 0;        
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
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
            double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler, 
                    gammaEuler);
            Quaternion q = rotation.toQuaternion();
            q.normalize();            
            
            //create camera center
            double[] cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);

            //normalize the camera to improve accuracy
            camera.normalize();            
            
            int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            List<Line2D> lines = new ArrayList<Line2D>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE));                        
                lines.add(line);
            }
            
            List<Plane> planes = camera.backProjectLines(lines);
            
            //create outliers
            Plane planeWithError;
            List<Plane> planesWithError = new ArrayList<Plane>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR); 
            double[] qualityScores = new double[nSamples];
            int j = 0;            
            for (Plane plane : planes) {
                double scoreError = randomizer.nextDouble(MIN_SCORE_ERROR, 
                        MAX_SCORE_ERROR);
                qualityScores[j] = 1.0 + scoreError;                
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //plane is oulier
                    double errorA = errorRandomizer.nextDouble();
                    double errorB = errorRandomizer.nextDouble();
                    double errorC = errorRandomizer.nextDouble();
                    double errorD = errorRandomizer.nextDouble();
                    planeWithError = new Plane(plane.getA() + errorA,
                            plane.getB() + errorB, plane.getC() + errorC,
                            plane.getD() + errorD);
                    
                    double error = Math.sqrt(errorA * errorA + errorB * errorB +
                            errorC * errorC + errorD * errorD);
                    qualityScores[j] = 1.0 / (1.0 + error) + scoreError;
                } else {
                    //inlier plane (without error)
                    planeWithError = plane;
                }
                
                planesWithError.add(planeWithError);
                j++;
            }
            
            PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                    this, planesWithError, lines, qualityScores);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);            
            estimator.setSuggestRotationEnabled(true);
            estimator.setSuggestedRotationValue(q);

            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
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
            Quaternion estimatedQ = estimatedCamera.getCameraRotation().
                    toQuaternion();
            estimatedQ.normalize();
            
            //estimate without suggestion
            estimator.setSuggestRotationEnabled(false);
            
            PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }         
            
            estimatedCameraNoSuggestion.decompose();
            
            Quaternion estimatedQNoSuggestion = estimatedCameraNoSuggestion.
                    getCameraRotation().toQuaternion();
            
            //check that rotation has become closer to suggested value
            double diffEstimatedNoSuggestion = 
                    Math.pow(q.getA() - estimatedQNoSuggestion.getA(), 2.0) +
                    Math.pow(q.getB() - estimatedQNoSuggestion.getB(), 2.0) +
                    Math.pow(q.getC() - estimatedQNoSuggestion.getC(), 2.0) +
                    Math.pow(q.getD() - estimatedQNoSuggestion.getD(), 2.0);
            double diffEstimated =
                    Math.pow(q.getA() - estimatedQ.getA(), 2.0) +
                    Math.pow(q.getB() - estimatedQ.getB(), 2.0) +
                    Math.pow(q.getC() - estimatedQ.getC(), 2.0) +
                    Math.pow(q.getD() - estimatedQ.getD(), 2.0);
            
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
    public void testEstimateSuggestedRotationWithFastRefinement() 
            throws IllegalArgumentException, LockedException, NotReadyException,
            RobustEstimatorException, CameraException, NotAvailableException {
        int numCovariances = 0;
        int numValid = 0;        
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
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
            double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler, 
                    gammaEuler);
            Quaternion q = rotation.toQuaternion();
            q.normalize();
            
            //create camera center
            double[] cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);

            //normalize the camera to improve accuracy
            camera.normalize();            
            
            int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            List<Line2D> lines = new ArrayList<Line2D>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE));                        
                lines.add(line);
            }
            
            List<Plane> planes = camera.backProjectLines(lines);
            
            //create outliers
            Plane planeWithError;
            List<Plane> planesWithError = new ArrayList<Plane>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR); 
            double[] qualityScores = new double[nSamples];
            int j = 0;            
            for (Plane plane : planes) {
                double scoreError = randomizer.nextDouble(MIN_SCORE_ERROR, 
                        MAX_SCORE_ERROR);
                qualityScores[j] = 1.0 + scoreError;                
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //plane is oulier
                    double errorA = errorRandomizer.nextDouble();
                    double errorB = errorRandomizer.nextDouble();
                    double errorC = errorRandomizer.nextDouble();
                    double errorD = errorRandomizer.nextDouble();
                    planeWithError = new Plane(plane.getA() + errorA,
                            plane.getB() + errorB, plane.getC() + errorC,
                            plane.getD() + errorD);
                    
                    double error = Math.sqrt(errorA * errorA + errorB * errorB +
                            errorC * errorC + errorD * errorD);
                    qualityScores[j] = 1.0 / (1.0 + error) + scoreError;
                } else {
                    //inlier plane (without error)
                    planeWithError = plane;
                }
                
                planesWithError.add(planeWithError);
                j++;
            }
            
            PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                    this, planesWithError, lines, qualityScores);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);
            estimator.setResultRefined(true);
            estimator.setFastRefinementUsed(true);
            estimator.setCovarianceKept(true);            
            estimator.setSuggestRotationEnabled(true);
            estimator.setSuggestedRotationValue(q);
            
            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
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
            Quaternion estimatedQ = estimatedCamera.getCameraRotation().
                    toQuaternion();
            estimatedQ.normalize();
            
            //estimate without suggestion
            estimator.setSuggestRotationEnabled(false);
            
            PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }         
            
            estimatedCameraNoSuggestion.decompose();
            
            Quaternion estimatedQNoSuggestion = estimatedCameraNoSuggestion.
                    getCameraRotation().toQuaternion();
            
            //check that rotation has become closer to suggested value
            double diffEstimatedNoSuggestion = 
                    Math.pow(q.getA() - estimatedQNoSuggestion.getA(), 2.0) +
                    Math.pow(q.getB() - estimatedQNoSuggestion.getB(), 2.0) +
                    Math.pow(q.getC() - estimatedQNoSuggestion.getC(), 2.0) +
                    Math.pow(q.getD() - estimatedQNoSuggestion.getD(), 2.0);
            double diffEstimated =
                    Math.pow(q.getA() - estimatedQ.getA(), 2.0) +
                    Math.pow(q.getB() - estimatedQ.getB(), 2.0) +
                    Math.pow(q.getC() - estimatedQ.getC(), 2.0) +
                    Math.pow(q.getD() - estimatedQ.getD(), 2.0);
            
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
    public void testEstimateSuggestedCenterEnabled() 
            throws IllegalArgumentException, LockedException, NotReadyException, 
            RobustEstimatorException, CameraException, NotAvailableException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
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
            double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler, 
                    gammaEuler);
            
            //create camera center
            double[] cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);

            //normalize the camera to improve accuracy
            camera.normalize();            
            
            int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            List<Line2D> lines = new ArrayList<Line2D>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE));                        
                lines.add(line);
            }
            
            List<Plane> planes = camera.backProjectLines(lines);
            
            //create outliers
            Plane planeWithError;
            List<Plane> planesWithError = new ArrayList<Plane>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR); 
            double[] qualityScores = new double[nSamples];
            int j = 0;            
            for (Plane plane : planes) {
                double scoreError = randomizer.nextDouble(MIN_SCORE_ERROR, 
                        MAX_SCORE_ERROR);
                qualityScores[j] = 1.0 + scoreError;                
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //plane is oulier
                    double errorA = errorRandomizer.nextDouble();
                    double errorB = errorRandomizer.nextDouble();
                    double errorC = errorRandomizer.nextDouble();
                    double errorD = errorRandomizer.nextDouble();
                    planeWithError = new Plane(plane.getA() + errorA,
                            plane.getB() + errorB, plane.getC() + errorC,
                            plane.getD() + errorD);
                    
                    double error = Math.sqrt(errorA * errorA + errorB * errorB +
                            errorC * errorC + errorD * errorD);
                    qualityScores[j] = 1.0 / (1.0 + error) + scoreError;
                } else {
                    //inlier plane (without error)
                    planeWithError = plane;
                }
                
                planesWithError.add(planeWithError);
                j++;
            }
            
            PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                    this, planesWithError, lines, qualityScores);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);
            estimator.setResultRefined(false);
            estimator.setCovarianceKept(false);            
            estimator.setSuggestCenterEnabled(true);
            estimator.setSuggestedCenterValue(cameraCenter);

            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
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
            assertNull(estimator.getCovariance());            
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
    public void testEstimateSuggestedCenterWithRefinement() 
            throws IllegalArgumentException, LockedException, NotReadyException,
            RobustEstimatorException, CameraException, NotAvailableException {
        int numCovariances = 0;
        int numValid = 0;        
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
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
            double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler, 
                    gammaEuler);
            
            //create camera center
            double[] cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);

            //normalize the camera to improve accuracy
            camera.normalize();            
            
            int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            List<Line2D> lines = new ArrayList<Line2D>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE));                        
                lines.add(line);
            }
            
            List<Plane> planes = camera.backProjectLines(lines);
            
            //create outliers
            Plane planeWithError;
            List<Plane> planesWithError = new ArrayList<Plane>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR); 
            double[] qualityScores = new double[nSamples];
            int j = 0;            
            for (Plane plane : planes) {
                double scoreError = randomizer.nextDouble(MIN_SCORE_ERROR, 
                        MAX_SCORE_ERROR);
                qualityScores[j] = 1.0 + scoreError;                
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //plane is oulier
                    double errorA = errorRandomizer.nextDouble();
                    double errorB = errorRandomizer.nextDouble();
                    double errorC = errorRandomizer.nextDouble();
                    double errorD = errorRandomizer.nextDouble();
                    planeWithError = new Plane(plane.getA() + errorA,
                            plane.getB() + errorB, plane.getC() + errorC,
                            plane.getD() + errorD);
                    
                    double error = Math.sqrt(errorA * errorA + errorB * errorB +
                            errorC * errorC + errorD * errorD);
                    qualityScores[j] = 1.0 / (1.0 + error) + scoreError;
                } else {
                    //inlier plane (without error)
                    planeWithError = plane;
                }
                
                planesWithError.add(planeWithError);
                j++;
            }
            
            PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                    this, planesWithError, lines, qualityScores);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);            
            estimator.setSuggestCenterEnabled(true);
            estimator.setSuggestedCenterValue(cameraCenter);

            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
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
    public void testEstimateSuggestedCenterWithFastRefinement() 
            throws IllegalArgumentException, LockedException, NotReadyException,
            RobustEstimatorException, CameraException, NotAvailableException {
        int numCovariances = 0;
        int numValid = 0;        
        for (int t = 0; t < 5*TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
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
            double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler, 
                    gammaEuler);
            
            //create camera center
            double[] cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);

            //normalize the camera to improve accuracy
            camera.normalize();            
            
            int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            List<Line2D> lines = new ArrayList<Line2D>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE));                        
                lines.add(line);
            }
            
            List<Plane> planes = camera.backProjectLines(lines);
            
            //create outliers
            Plane planeWithError;
            List<Plane> planesWithError = new ArrayList<Plane>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR); 
            double[] qualityScores = new double[nSamples];
            int j = 0;            
            for (Plane plane : planes) {
                double scoreError = randomizer.nextDouble(MIN_SCORE_ERROR, 
                        MAX_SCORE_ERROR);
                qualityScores[j] = 1.0 + scoreError;                
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //plane is oulier
                    double errorA = errorRandomizer.nextDouble();
                    double errorB = errorRandomizer.nextDouble();
                    double errorC = errorRandomizer.nextDouble();
                    double errorD = errorRandomizer.nextDouble();
                    planeWithError = new Plane(plane.getA() + errorA,
                            plane.getB() + errorB, plane.getC() + errorC,
                            plane.getD() + errorD);
                    
                    double error = Math.sqrt(errorA * errorA + errorB * errorB +
                            errorC * errorC + errorD * errorD);
                    qualityScores[j] = 1.0 / (1.0 + error) + scoreError;
                } else {
                    //inlier plane (without error)
                    planeWithError = plane;
                }
                
                planesWithError.add(planeWithError);
                j++;
            }
            
            PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                    this, planesWithError, lines, qualityScores);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);
            estimator.setResultRefined(true);
            estimator.setFastRefinementUsed(true);
            estimator.setCovarianceKept(true);            
            estimator.setSuggestCenterEnabled(true);
            estimator.setSuggestedCenterValue(cameraCenter);
            
            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
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
    public void testEstimateZeroSkewnessZeroPrincipalPointAndEqualFocalLength() 
            throws IllegalArgumentException, LockedException, NotReadyException, 
            RobustEstimatorException, CameraException, NotAvailableException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength = horizontalFocalLength;
            
            double skewness = 0.0;
            double horizontalPrincipalPoint = 0.0;
            double verticalPrincipalPoint = 0.0;
            InhomogeneousPoint2D principalPoint = new InhomogeneousPoint2D();

            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                    verticalFocalLength, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, skewness);
            
            double aspectRatio = intrinsic.getAspectRatio();
            
            //create rotation parameters
            double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler, 
                    gammaEuler);
            
            //create camera center
            double[] cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);

            //normalize the camera to improve accuracy
            camera.normalize();            
            
            int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            List<Line2D> lines = new ArrayList<Line2D>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE));                        
                lines.add(line);
            }
            
            List<Plane> planes = camera.backProjectLines(lines);
            
            //create outliers
            Plane planeWithError;
            List<Plane> planesWithError = new ArrayList<Plane>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR); 
            double[] qualityScores = new double[nSamples];
            int j = 0;            
            for (Plane plane : planes) {
                double scoreError = randomizer.nextDouble(MIN_SCORE_ERROR, 
                        MAX_SCORE_ERROR);
                qualityScores[j] = 1.0 + scoreError;                
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //plane is oulier
                    double errorA = errorRandomizer.nextDouble();
                    double errorB = errorRandomizer.nextDouble();
                    double errorC = errorRandomizer.nextDouble();
                    double errorD = errorRandomizer.nextDouble();
                    planeWithError = new Plane(plane.getA() + errorA,
                            plane.getB() + errorB, plane.getC() + errorC,
                            plane.getD() + errorD);
                    
                    double error = Math.sqrt(errorA * errorA + errorB * errorB +
                            errorC * errorC + errorD * errorD);
                    qualityScores[j] = 1.0 / (1.0 + error) + scoreError;
                } else {
                    //inlier plane (without error)
                    planeWithError = plane;
                }
                
                planesWithError.add(planeWithError);
                j++;
            }
            
            PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                    this, planesWithError, lines, qualityScores);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);
            estimator.setResultRefined(false);
            estimator.setCovarianceKept(false);            
            estimator.setSuggestSkewnessValueEnabled(true);
            estimator.setSuggestPrincipalPointEnabled(true);
            estimator.setSuggestAspectRatioEnabled(true);

            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
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
            assertNull(estimator.getCovariance());            
            reset();

            //decompone estimated camera and check its parameters
            estimatedCamera.decompose();
            
            //comparing camera intrinsic parameters
            PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    estimatedCamera.getIntrinsicParameters();    
            double estimatedSkewness = estimatedIntrinsic.getSkewness();
            InhomogeneousPoint2D estimatedPrincipalPoint = 
                    new InhomogeneousPoint2D(
                            estimatedIntrinsic.getHorizontalPrincipalPoint(), 
                            estimatedIntrinsic.getVerticalPrincipalPoint());
            double estimatedAspectRatio = estimatedIntrinsic.getAspectRatio();
            
            //estimate without suggestions
            estimator.setSuggestSkewnessValueEnabled(false);
            estimator.setSuggestPrincipalPointEnabled(false);
            estimator.setSuggestAspectRatioEnabled(false);
            
            PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }         
            
            estimatedCameraNoSuggestion.decompose();
            
            PinholeCameraIntrinsicParameters estimatedIntrinsicNoSuggestion =
                    estimatedCameraNoSuggestion.getIntrinsicParameters();
            double estimatedSkewnessNoSuggestion = 
                    estimatedIntrinsicNoSuggestion.getSkewness();
            InhomogeneousPoint2D estimatedPrincipalPointNoSuggestion =
                    new InhomogeneousPoint2D(
                            estimatedIntrinsicNoSuggestion.getHorizontalPrincipalPoint(),
                            estimatedIntrinsicNoSuggestion.getVerticalPrincipalPoint());
            double estimatedAspectRatioNoSuggestion = 
                    estimatedIntrinsicNoSuggestion.getAspectRatio();
            
            //check that intrinsic values have become closer to suggested ones
            if ((Math.abs(skewness - estimatedSkewnessNoSuggestion) >=
                    Math.abs(skewness - estimatedSkewness)) &&
                    (principalPoint.distanceTo(estimatedPrincipalPointNoSuggestion) >=
                    principalPoint.distanceTo(estimatedPrincipalPoint)) &&
                    (Math.abs(aspectRatio - estimatedAspectRatioNoSuggestion) >=
                    Math.abs(aspectRatio - estimatedAspectRatio))) {
                numValid++;
            }            
            
            if (numValid > 0) {
                break;
            }
        }
        
        assertTrue(numValid > 0);
    }

    @Test
    public void testEstimateZeroSkewnessZeroPrincipalPointAndEqualFocalLengthWithRefinement() 
            throws IllegalArgumentException, LockedException, NotReadyException,
            RobustEstimatorException, CameraException, NotAvailableException {
        int numCovariances = 0;
        int numValid = 0;        
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength = horizontalFocalLength;
            
            double skewness = 0.0;
            double horizontalPrincipalPoint = 0.0;
            double verticalPrincipalPoint = 0.0;
            InhomogeneousPoint2D principalPoint = new InhomogeneousPoint2D();

            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                    verticalFocalLength, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, skewness);
            
            double aspectRatio = intrinsic.getAspectRatio();
            
            //create rotation parameters
            double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler, 
                    gammaEuler);
            
            //create camera center
            double[] cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);

            //normalize the camera to improve accuracy
            camera.normalize();            
            
            int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            List<Line2D> lines = new ArrayList<Line2D>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE));                        
                lines.add(line);
            }
            
            List<Plane> planes = camera.backProjectLines(lines);
            
            //create outliers
            Plane planeWithError;
            List<Plane> planesWithError = new ArrayList<Plane>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR); 
            double[] qualityScores = new double[nSamples];
            int j = 0;            
            for (Plane plane : planes) {
                double scoreError = randomizer.nextDouble(MIN_SCORE_ERROR, 
                        MAX_SCORE_ERROR);
                qualityScores[j] = 1.0 + scoreError;                
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //plane is oulier
                    double errorA = errorRandomizer.nextDouble();
                    double errorB = errorRandomizer.nextDouble();
                    double errorC = errorRandomizer.nextDouble();
                    double errorD = errorRandomizer.nextDouble();
                    planeWithError = new Plane(plane.getA() + errorA,
                            plane.getB() + errorB, plane.getC() + errorC,
                            plane.getD() + errorD);
                    
                    double error = Math.sqrt(errorA * errorA + errorB * errorB +
                            errorC * errorC + errorD * errorD);
                    qualityScores[j] = 1.0 / (1.0 + error) + scoreError;
                } else {
                    //inlier plane (without error)
                    planeWithError = plane;
                }
                
                planesWithError.add(planeWithError);
                j++;
            }
            
            PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                    this, planesWithError, lines, qualityScores);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);            
            estimator.setSuggestSkewnessValueEnabled(true);
            estimator.setSuggestPrincipalPointEnabled(true);
            estimator.setSuggestAspectRatioEnabled(true);

            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
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
            
            //comparing camera intrinsic parameters
            PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    estimatedCamera.getIntrinsicParameters();    
            double estimatedSkewness = estimatedIntrinsic.getSkewness();
            InhomogeneousPoint2D estimatedPrincipalPoint = 
                    new InhomogeneousPoint2D(
                            estimatedIntrinsic.getHorizontalPrincipalPoint(), 
                            estimatedIntrinsic.getVerticalPrincipalPoint());
            double estimatedAspectRatio = estimatedIntrinsic.getAspectRatio();
            
            //estimate without suggestions
            estimator.setSuggestSkewnessValueEnabled(false);
            estimator.setSuggestPrincipalPointEnabled(false);
            estimator.setSuggestAspectRatioEnabled(false);
            
            PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }         
            
            estimatedCameraNoSuggestion.decompose();
            
            PinholeCameraIntrinsicParameters estimatedIntrinsicNoSuggestion =
                    estimatedCameraNoSuggestion.getIntrinsicParameters();
            double estimatedSkewnessNoSuggestion = 
                    estimatedIntrinsicNoSuggestion.getSkewness();
            InhomogeneousPoint2D estimatedPrincipalPointNoSuggestion =
                    new InhomogeneousPoint2D(
                            estimatedIntrinsicNoSuggestion.getHorizontalPrincipalPoint(),
                            estimatedIntrinsicNoSuggestion.getVerticalPrincipalPoint());
            double estimatedAspectRatioNoSuggestion = 
                    estimatedIntrinsicNoSuggestion.getAspectRatio();
            
            //check that intrinsic values have become closer to suggested ones
            if ((Math.abs(skewness - estimatedSkewnessNoSuggestion) >=
                    Math.abs(skewness - estimatedSkewness)) &&
                    (principalPoint.distanceTo(estimatedPrincipalPointNoSuggestion) >=
                    principalPoint.distanceTo(estimatedPrincipalPoint)) &&
                    (Math.abs(aspectRatio - estimatedAspectRatioNoSuggestion) >=
                    Math.abs(aspectRatio - estimatedAspectRatio))) {
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
    public void testEstimateZeroSkewnessZeroPrincipalPointAndEqualFocalLengthWithFastRefinement() 
            throws IllegalArgumentException, LockedException, NotReadyException,
            RobustEstimatorException, CameraException, NotAvailableException {
        int numCovariances = 0;
        int numValid = 0;        
        for (int t = 0; t < 5*TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength = horizontalFocalLength;
            
            double skewness = 0.0;
            double horizontalPrincipalPoint = 0.0;
            double verticalPrincipalPoint = 0.0;
            InhomogeneousPoint2D principalPoint = new InhomogeneousPoint2D();

            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                    verticalFocalLength, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, skewness);
            
            double aspectRatio = intrinsic.getAspectRatio();
            
            //create rotation parameters
            double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler, 
                    gammaEuler);
            
            //create camera center
            double[] cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);

            //normalize the camera to improve accuracy
            camera.normalize();            
            
            int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            List<Line2D> lines = new ArrayList<Line2D>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE));                        
                lines.add(line);
            }
            
            List<Plane> planes = camera.backProjectLines(lines);
            
            //create outliers
            Plane planeWithError;
            List<Plane> planesWithError = new ArrayList<Plane>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR); 
            double[] qualityScores = new double[nSamples];
            int j = 0;            
            for (Plane plane : planes) {
                double scoreError = randomizer.nextDouble(MIN_SCORE_ERROR, 
                        MAX_SCORE_ERROR);
                qualityScores[j] = 1.0 + scoreError;                
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //plane is oulier
                    double errorA = errorRandomizer.nextDouble();
                    double errorB = errorRandomizer.nextDouble();
                    double errorC = errorRandomizer.nextDouble();
                    double errorD = errorRandomizer.nextDouble();
                    planeWithError = new Plane(plane.getA() + errorA,
                            plane.getB() + errorB, plane.getC() + errorC,
                            plane.getD() + errorD);
                    
                    double error = Math.sqrt(errorA * errorA + errorB * errorB +
                            errorC * errorC + errorD * errorD);
                    qualityScores[j] = 1.0 / (1.0 + error) + scoreError;
                } else {
                    //inlier plane (without error)
                    planeWithError = plane;
                }
                
                planesWithError.add(planeWithError);
                j++;
            }
            
            PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                    this, planesWithError, lines, qualityScores);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);
            estimator.setResultRefined(true);
            estimator.setFastRefinementUsed(true);
            estimator.setCovarianceKept(true);            
            estimator.setSuggestSkewnessValueEnabled(true);
            estimator.setSuggestPrincipalPointEnabled(true);
            estimator.setSuggestAspectRatioEnabled(true);
            
            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
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
            
            //comparing camera intrinsic parameters
            PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    estimatedCamera.getIntrinsicParameters();    
            double estimatedSkewness = estimatedIntrinsic.getSkewness();
            InhomogeneousPoint2D estimatedPrincipalPoint = 
                    new InhomogeneousPoint2D(
                            estimatedIntrinsic.getHorizontalPrincipalPoint(), 
                            estimatedIntrinsic.getVerticalPrincipalPoint());
            double estimatedAspectRatio = estimatedIntrinsic.getAspectRatio();
            
            //estimate without suggestion
            estimator.setSuggestSkewnessValueEnabled(false);
            estimator.setSuggestPrincipalPointEnabled(false);
            estimator.setSuggestAspectRatioEnabled(false);
            
            PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }         
            
            estimatedCameraNoSuggestion.decompose();
            
            PinholeCameraIntrinsicParameters estimatedIntrinsicNoSuggestion =
                    estimatedCameraNoSuggestion.getIntrinsicParameters();
            double estimatedSkewnessNoSuggestion = 
                    estimatedIntrinsicNoSuggestion.getSkewness();
            InhomogeneousPoint2D estimatedPrincipalPointNoSuggestion =
                    new InhomogeneousPoint2D(
                            estimatedIntrinsicNoSuggestion.getHorizontalPrincipalPoint(),
                            estimatedIntrinsicNoSuggestion.getVerticalPrincipalPoint());
            double estimatedAspectRatioNoSuggestion = 
                    estimatedIntrinsicNoSuggestion.getAspectRatio();
            
            //check that intrinsic values have become closer to suggested ones
            if ((Math.abs(skewness - estimatedSkewnessNoSuggestion) >=
                    Math.abs(skewness - estimatedSkewness)) &&
                    (principalPoint.distanceTo(estimatedPrincipalPointNoSuggestion) >=
                    principalPoint.distanceTo(estimatedPrincipalPoint)) &&
                    (Math.abs(aspectRatio - estimatedAspectRatioNoSuggestion) >=
                    Math.abs(aspectRatio - estimatedAspectRatio))) {
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
        testLocked((PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator)estimator);
    }

    @Override
    public void onEstimateEnd(PinholeCameraRobustEstimator estimator) {
        estimateEnd++;
        testLocked((PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator)estimator);
    }

    @Override
    public void onEstimateNextIteration(PinholeCameraRobustEstimator estimator, 
            int iteration) {
        estimateNextIteration++;
        testLocked((PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator)estimator);
    }

    @Override
    public void onEstimateProgressChange(PinholeCameraRobustEstimator estimator, 
            float progress) {
        estimateProgressChange++;
        testLocked((PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator)estimator);
    }
    
    private void testLocked(
            PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator) {
        List<Plane> planes = new ArrayList<Plane>();
        List<Line2D> lines = new ArrayList<Line2D>();
        try {
            estimator.setLinesAndPlanes(planes, lines);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setQualityScores(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setListener(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setProgressDelta(0.01f);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setThreshold(0.5);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setConfidence(0.5);            
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setMaxIterations(10);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.estimate();
            fail("LockedException expected but not thrown");
        } catch (LockedException e) {
        } catch (Exception e) {
            fail("LockedException expected but not thrown");
        }
        assertTrue(estimator.isLocked());        
    }    
}
