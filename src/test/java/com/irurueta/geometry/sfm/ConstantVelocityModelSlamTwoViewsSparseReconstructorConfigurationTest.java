/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.sfm.ConstantVelocityModelSlamTwoViewsSparseReconstructorConfiguration
 * 
 * @author Alberto Irurueta (albeto@irurueta.com)
 * @date June 16, 2017.
 */
package com.irurueta.geometry.sfm;

import com.irurueta.geometry.slam.ConstantVelocityModelSlamCalibrationData;
import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class ConstantVelocityModelSlamTwoViewsSparseReconstructorConfigurationTest {
    
    public ConstantVelocityModelSlamTwoViewsSparseReconstructorConfigurationTest() { }
    
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
        ConstantVelocityModelSlamTwoViewsSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamTwoViewsSparseReconstructorConfiguration();
        
        //check default values
        assertEquals(cfg.getNonRobustFundamentalMatrixEstimatorMethod(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_NON_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD);
        assertEquals(cfg.getRobustFundamentalMatrixEstimatorMethod(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD);
        assertEquals(cfg.isFundamentalMatrixRefined(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_REFINE_FUNDAMENTAL_MATRIX);
        assertEquals(cfg.isFundamentalMatrixCovarianceKept(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_KEEP_FUNDAMENTAL_MATRIX_COVARIANCE);
        assertEquals(cfg.getFundamentalMatrixConfidence(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_CONFIDENCE, 0.0);
        assertEquals(cfg.getFundamentalMatrixMaxIterations(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_MAX_ITERATIONS);
        assertEquals(cfg.getFundamentalMatrixThreshold(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_THRESHOLD, 0.0);
        assertEquals(cfg.getFundamentalMatrixComputeAndKeepInliers(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(cfg.getFundamentalMatrixComputeAndKeepResiduals(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(cfg.getInitialCamerasEstimatorMethod(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_INITIAL_CAMERAS_ESTIMATOR_METHOD);
        assertEquals(cfg.getDaqUseHomogeneousPointTriangulator(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_DAQ_USE_HOMOGENEOUS_POINT_TRIANGULATOR);
        assertEquals(cfg.getInitialCamerasAspectRatio(), 
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_INITIAL_CAMERAS_ASPECT_RATIO, 0.0);
        assertEquals(cfg.getPrincipalPointX(), 0.0, 0.0);
        assertEquals(cfg.getPrincipalPointY(), 0.0, 0.0);
        assertEquals(cfg.getInitialCamerasCorrectorType(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_INITIAL_CAMERAS_CORRECTOR_TYPE);
        assertEquals(cfg.getInitialCamerasMarkValidTriangulatedPoints(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_INITIAL_CAMERAS_MARK_VALID_TRIANGULATED_POINTS);
        assertNull(cfg.getInitialIntrinsic1());
        assertNull(cfg.getInitialIntrinsic2());
        assertEquals(cfg.isGeneralSceneAllowed(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_GENERAL_SCENE);
        assertEquals(cfg.isPlanarSceneAllowed(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_PLANAR_SCENE);
        assertEquals(cfg.getRobustPlanarHomographyEstimatorMethod(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_ROBUST_PLANAR_HOMOGRAPHY_ESTIMATOR_METHOD);
        assertEquals(cfg.isPlanarHomographyRefined(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_REFINE_PLANAR_HOMOGRAPHY);
        assertEquals(cfg.isPlanarHomographyCovarianceKept(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE);
        assertEquals(cfg.getPlanarHomographyConfidence(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_CONFIDENCE, 0.0);
        assertEquals(cfg.getPlanarHomographyMaxIterations(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_MAX_ITERATIONS);
        assertEquals(cfg.getPlanarHomographyThreshold(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_THRESHOLD, 0.0);        
        assertEquals(cfg.getPlanarHomographyComputeAndKeepInliers(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(cfg.getPlanarHomographyComputeAndKeepResiduals(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS);        
        assertNull(cfg.getCalibrationData());
    }
    
    @Test
    public void testMake() {
        ConstantVelocityModelSlamTwoViewsSparseReconstructorConfiguration cfg =
                ConstantVelocityModelSlamTwoViewsSparseReconstructorConfiguration.make();
        
        //check default values
        assertEquals(cfg.getNonRobustFundamentalMatrixEstimatorMethod(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_NON_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD);
        assertEquals(cfg.getRobustFundamentalMatrixEstimatorMethod(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD);
        assertEquals(cfg.isFundamentalMatrixRefined(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_REFINE_FUNDAMENTAL_MATRIX);
        assertEquals(cfg.isFundamentalMatrixCovarianceKept(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_KEEP_FUNDAMENTAL_MATRIX_COVARIANCE);
        assertEquals(cfg.getFundamentalMatrixConfidence(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_CONFIDENCE, 0.0);
        assertEquals(cfg.getFundamentalMatrixMaxIterations(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_MAX_ITERATIONS);
        assertEquals(cfg.getFundamentalMatrixThreshold(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_THRESHOLD, 0.0);
        assertEquals(cfg.getFundamentalMatrixComputeAndKeepInliers(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(cfg.getFundamentalMatrixComputeAndKeepResiduals(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(cfg.getInitialCamerasEstimatorMethod(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_INITIAL_CAMERAS_ESTIMATOR_METHOD);
        assertEquals(cfg.getDaqUseHomogeneousPointTriangulator(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_DAQ_USE_HOMOGENEOUS_POINT_TRIANGULATOR);
        assertEquals(cfg.getInitialCamerasAspectRatio(), 
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_INITIAL_CAMERAS_ASPECT_RATIO, 0.0);
        assertEquals(cfg.getPrincipalPointX(), 0.0, 0.0);
        assertEquals(cfg.getPrincipalPointY(), 0.0, 0.0);
        assertEquals(cfg.getInitialCamerasCorrectorType(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_INITIAL_CAMERAS_CORRECTOR_TYPE);
        assertEquals(cfg.getInitialCamerasMarkValidTriangulatedPoints(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_INITIAL_CAMERAS_MARK_VALID_TRIANGULATED_POINTS);
        assertNull(cfg.getInitialIntrinsic1());
        assertNull(cfg.getInitialIntrinsic2());
        assertEquals(cfg.isGeneralSceneAllowed(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_GENERAL_SCENE);
        assertEquals(cfg.isPlanarSceneAllowed(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_PLANAR_SCENE);
        assertEquals(cfg.getRobustPlanarHomographyEstimatorMethod(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_ROBUST_PLANAR_HOMOGRAPHY_ESTIMATOR_METHOD);
        assertEquals(cfg.isPlanarHomographyRefined(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_REFINE_PLANAR_HOMOGRAPHY);
        assertEquals(cfg.isPlanarHomographyCovarianceKept(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE);
        assertEquals(cfg.getPlanarHomographyConfidence(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_CONFIDENCE, 0.0);
        assertEquals(cfg.getPlanarHomographyMaxIterations(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_MAX_ITERATIONS);
        assertEquals(cfg.getPlanarHomographyThreshold(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_THRESHOLD, 0.0);        
        assertEquals(cfg.getPlanarHomographyComputeAndKeepInliers(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(cfg.getPlanarHomographyComputeAndKeepResiduals(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS);        
        assertNull(cfg.getCalibrationData());        
    }
    
    @Test
    public void testGetSetCalibrationData() {
        ConstantVelocityModelSlamTwoViewsSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamTwoViewsSparseReconstructorConfiguration();

        //check default value
        assertNull(cfg.getCalibrationData());
        
        //set new value
        ConstantVelocityModelSlamCalibrationData calibrationData =
                new ConstantVelocityModelSlamCalibrationData();
        cfg.setCalibrationData(calibrationData);
        
        //check correctness
        assertSame(cfg.getCalibrationData(), calibrationData);
    }
}
