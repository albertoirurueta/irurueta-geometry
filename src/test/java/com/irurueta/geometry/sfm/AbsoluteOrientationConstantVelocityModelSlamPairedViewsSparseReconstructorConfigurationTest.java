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

package com.irurueta.geometry.sfm;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.epipolar.CorrectorType;
import com.irurueta.geometry.epipolar.estimators.FundamentalMatrixEstimatorMethod;
import com.irurueta.geometry.slam.AbsoluteOrientationConstantVelocityModelSlamCalibrationData;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import org.junit.*;

import static org.junit.Assert.*;

public class AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfigurationTest {

    private static final int POINT_INHOM_COORDS = 3;
    private static final double CAMERA_POSITION_VARIANCE = 1e-6;

    public AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfigurationTest() { }

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
        AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        //check default values
        assertEquals(cfg.getNonRobustFundamentalMatrixEstimatorMethod(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_NON_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD);
        assertEquals(cfg.getRobustFundamentalMatrixEstimatorMethod(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD);
        assertEquals(cfg.isFundamentalMatrixRefined(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_REFINE_FUNDAMENTAL_MATRIX);
        assertEquals(cfg.isFundamentalMatrixCovarianceKept(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KEEP_FUNDAMENTAL_MATRIX_COVARIANCE);
        assertEquals(cfg.getFundamentalMatrixConfidence(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_CONFIDENCE, 0.0);
        assertEquals(cfg.getFundamentalMatrixMaxIterations(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_MAX_ITERATIONS);
        assertEquals(cfg.getFundamentalMatrixThreshold(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_THRESHOLD, 0.0);
        assertEquals(cfg.getFundamentalMatrixComputeAndKeepInliers(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(cfg.getFundamentalMatrixComputeAndKeepResiduals(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(cfg.getPairedCamerasEstimatorMethod(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_ESTIMATOR_METHOD);
        assertEquals(cfg.getDaqUseHomogeneousPointTriangulator(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_DAQ_USE_HOMOGENEOUS_POINT_TRIANGULATOR);
        assertEquals(cfg.getPairedCamerasAspectRatio(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_ASPECT_RATIO, 0.0);
        assertEquals(cfg.getPrincipalPointX(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_PRINCIPAL_POINT_X, 0.0);
        assertEquals(cfg.getPrincipalPointY(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_PRINCIPAL_POINT_Y, 0.0);
        assertEquals(cfg.getPairedCamerasCorrectorType(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_CORRECTOR_TYPE);
        assertEquals(cfg.getPairedCamerasMarkValidTriangulatedPoints(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_MARK_VALID_TRIANGULATED_POINTS);
        assertEquals(cfg.areIntrinsicParametersKnown(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KNOWN_INTRINSIC_PARAMETERS);
        assertEquals(cfg.isGeneralSceneAllowed(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_GENERAL_SCENE);
        assertEquals(cfg.isPlanarSceneAllowed(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_PLANAR_SCENE);
        assertEquals(cfg.getRobustPlanarHomographyEstimatorMethod(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ROBUST_PLANAR_HOMOGRAPHY_ESTIMATOR_METHOD);
        assertEquals(cfg.isPlanarHomographyRefined(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_REFINE_PLANAR_HOMOGRAPHY);
        assertEquals(cfg.isPlanarHomographyCovarianceKept(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE);
        assertEquals(cfg.getPlanarHomographyConfidence(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_CONFIDENCE, 0.0);
        assertEquals(cfg.getPlanarHomographyMaxIterations(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_MAX_ITERATIONS);
        assertEquals(cfg.getPlanarHomographyThreshold(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_THRESHOLD, 0.0);
        assertEquals(cfg.getPlanarHomographyComputeAndKeepInliers(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(cfg.getPlanarHomographyComputeAndKeepResiduals(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS);
        assertNull(cfg.getCalibrationData());
        assertNotNull(cfg.getCameraPositionCovariance());
        assertEquals(cfg.isNotifyAvailableSlamDataEnabled(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_NOTIFY_SLAM_DATA_AVAILABLE);
        assertEquals(cfg.isNotifyEstimatedSlamCameraEnabled(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_NOTIFY_ESTIMATED_SLAM_CAMERA);

    }

    @Test
    public void testMake() {
        AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.make();

        //check default values
        assertEquals(cfg.getNonRobustFundamentalMatrixEstimatorMethod(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_NON_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD);
        assertEquals(cfg.getRobustFundamentalMatrixEstimatorMethod(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD);
        assertEquals(cfg.isFundamentalMatrixRefined(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_REFINE_FUNDAMENTAL_MATRIX);
        assertEquals(cfg.isFundamentalMatrixCovarianceKept(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KEEP_FUNDAMENTAL_MATRIX_COVARIANCE);
        assertEquals(cfg.getFundamentalMatrixConfidence(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_CONFIDENCE, 0.0);
        assertEquals(cfg.getFundamentalMatrixMaxIterations(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_MAX_ITERATIONS);
        assertEquals(cfg.getFundamentalMatrixThreshold(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_THRESHOLD, 0.0);
        assertEquals(cfg.getFundamentalMatrixComputeAndKeepInliers(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(cfg.getFundamentalMatrixComputeAndKeepResiduals(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(cfg.getPairedCamerasEstimatorMethod(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_ESTIMATOR_METHOD);
        assertEquals(cfg.getDaqUseHomogeneousPointTriangulator(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_DAQ_USE_HOMOGENEOUS_POINT_TRIANGULATOR);
        assertEquals(cfg.getPairedCamerasAspectRatio(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_ASPECT_RATIO, 0.0);
        assertEquals(cfg.getPrincipalPointX(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_PRINCIPAL_POINT_X, 0.0);
        assertEquals(cfg.getPrincipalPointY(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_PRINCIPAL_POINT_Y, 0.0);
        assertEquals(cfg.getPairedCamerasCorrectorType(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_CORRECTOR_TYPE);
        assertEquals(cfg.getPairedCamerasMarkValidTriangulatedPoints(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_MARK_VALID_TRIANGULATED_POINTS);
        assertEquals(cfg.areIntrinsicParametersKnown(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KNOWN_INTRINSIC_PARAMETERS);
        assertEquals(cfg.isGeneralSceneAllowed(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_GENERAL_SCENE);
        assertEquals(cfg.isPlanarSceneAllowed(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_PLANAR_SCENE);
        assertEquals(cfg.getRobustPlanarHomographyEstimatorMethod(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ROBUST_PLANAR_HOMOGRAPHY_ESTIMATOR_METHOD);
        assertEquals(cfg.isPlanarHomographyRefined(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_REFINE_PLANAR_HOMOGRAPHY);
        assertEquals(cfg.isPlanarHomographyCovarianceKept(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE);
        assertEquals(cfg.getPlanarHomographyConfidence(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_CONFIDENCE, 0.0);
        assertEquals(cfg.getPlanarHomographyMaxIterations(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_MAX_ITERATIONS);
        assertEquals(cfg.getPlanarHomographyThreshold(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_THRESHOLD, 0.0);
        assertEquals(cfg.getPlanarHomographyComputeAndKeepInliers(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(cfg.getPlanarHomographyComputeAndKeepResiduals(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS);
        assertNull(cfg.getCalibrationData());
        assertNotNull(cfg.getCameraPositionCovariance());
        assertEquals(cfg.isNotifyAvailableSlamDataEnabled(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_NOTIFY_SLAM_DATA_AVAILABLE);
        assertEquals(cfg.isNotifyEstimatedSlamCameraEnabled(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_NOTIFY_ESTIMATED_SLAM_CAMERA);
    }

    @Test
    public void testGetSetNonRobustFundamentalMatrixEstimatorMethod() {
        AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getNonRobustFundamentalMatrixEstimatorMethod(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_NON_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD);

        //set new value
        assertSame(cfg.setNonRobustFundamentalMatrixEstimatorMethod(
                FundamentalMatrixEstimatorMethod.EIGHT_POINTS_ALGORITHM), cfg);

        //check correctness
        assertEquals(cfg.getNonRobustFundamentalMatrixEstimatorMethod(),
                FundamentalMatrixEstimatorMethod.EIGHT_POINTS_ALGORITHM);
    }

    @Test
    public void testGetSetRobustFundamentalMatrixEstimatorMethod() {
        AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getRobustFundamentalMatrixEstimatorMethod(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD);

        //set new value
        assertSame(cfg.setRobustFundamentalMatrixEstimatorMethod(
                RobustEstimatorMethod.LMedS), cfg);

        //check correctness
        assertEquals(cfg.getRobustFundamentalMatrixEstimatorMethod(),
                RobustEstimatorMethod.LMedS);
    }

    @Test
    public void testIsSetFundamentalMatrixRefined() {
        AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.isFundamentalMatrixRefined(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_REFINE_FUNDAMENTAL_MATRIX);

        //set new value
        assertSame(cfg.setFundamentalMatrixRefined(false), cfg);

        //check correctness
        assertFalse(cfg.isFundamentalMatrixRefined());
    }

    @Test
    public void testIsSetFundamentalMatrixCovarianceKept() {
        AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.isFundamentalMatrixCovarianceKept(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KEEP_FUNDAMENTAL_MATRIX_COVARIANCE);

        //set new value
        assertSame(cfg.setFundamentalMatrixCovarianceKept(true), cfg);

        //check correctness
        assertTrue(cfg.isFundamentalMatrixCovarianceKept());
    }

    @Test
    public void testGetSetFundamentalMatrixConfidence() {
        AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getFundamentalMatrixConfidence(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_CONFIDENCE, 0.0);

        //set new value
        assertSame(cfg.setFundamentalMatrixConfidence(0.7), cfg);

        //check correctness
        assertEquals(cfg.getFundamentalMatrixConfidence(), 0.7, 0.0);
    }

    @Test
    public void testGetSetFundamentalMatrixMaxIterations() {
        AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getFundamentalMatrixMaxIterations(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_MAX_ITERATIONS);

        //set new value
        assertSame(cfg.setFundamentalMatrixMaxIterations(10), cfg);

        //check correctness
        assertEquals(cfg.getFundamentalMatrixMaxIterations(), 10);
    }

    @Test
    public void testGetSetFundamentalMatrixThreshold() {
        AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getFundamentalMatrixThreshold(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_THRESHOLD, 0.0);

        //set new value
        assertSame(cfg.setFundamentalMatrixThreshold(2.0), cfg);

        //check correctness
        assertEquals(cfg.getFundamentalMatrixThreshold(), 2.0, 0.0);
    }

    @Test
    public void testGetSetFundamentalMatrixComputeAndKeepInliers() {
        AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getFundamentalMatrixComputeAndKeepInliers(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_INLIERS);

        //set new value
        assertSame(cfg.setFundamentalMatrixComputeAndKeepInliers(false), cfg);

        //check correctness
        assertFalse(cfg.getFundamentalMatrixComputeAndKeepInliers());
    }

    @Test
    public void testGetSetFundamentalMatrixComputeAndKeepResiduals() {
        AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getFundamentalMatrixComputeAndKeepResiduals(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_RESIDUALS);

        //set new value
        assertSame(cfg.setFundamentalMatrixComputeAndKeepResiduals(false), cfg);

        //check correctness
        assertFalse(cfg.getFundamentalMatrixComputeAndKeepResiduals());
    }

    @Test
    public void testGetSetPairedCamerasEstimatorMethod() {
        AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getPairedCamerasEstimatorMethod(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_ESTIMATOR_METHOD);

        //set new value
        assertSame(cfg.setPairedCamerasEstimatorMethod(
                InitialCamerasEstimatorMethod.DUAL_IMAGE_OF_ABSOLUTE_CONIC),
                cfg);

        //check correctness
        assertEquals(cfg.getPairedCamerasEstimatorMethod(),
                InitialCamerasEstimatorMethod.DUAL_IMAGE_OF_ABSOLUTE_CONIC);
    }

    @Test
    public void testGetSetDaqUseHomogeneousPointTriangulator() {
        AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getDaqUseHomogeneousPointTriangulator(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_DAQ_USE_HOMOGENEOUS_POINT_TRIANGULATOR);

        //set new value
        assertSame(cfg.setDaqUseHomogeneousPointTriangulator(false), cfg);

        //check correctness
        assertFalse(cfg.getDaqUseHomogeneousPointTriangulator());
    }

    @Test
    public void testGetSetPairedCamerasAspectRatio() {
        AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getPairedCamerasAspectRatio(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_ASPECT_RATIO, 0.0);

        //set new value
        assertSame(cfg.setPairedCamerasAspectRatio(0.5), cfg);

        //check correctness
        assertEquals(cfg.getPairedCamerasAspectRatio(), 0.5, 0.0);
    }

    @Test
    public void testGetSetPrincipalPointX() {
        AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getPrincipalPointX(), 0.0, 0.0);

        //set new value
        assertSame(cfg.setPrincipalPointX(10.0), cfg);

        //check correctness
        assertEquals(cfg.getPrincipalPointX(), 10.0, 0.0);
    }

    @Test
    public void testGetSetPrincipalPointY() {
        AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getPrincipalPointY(), 0.0, 0.0);

        //set new value
        assertSame(cfg.setPrincipalPointY(10.0), cfg);

        //check correctness
        assertEquals(cfg.getPrincipalPointY(), 10.0, 0.0);
    }

    @Test
    public void testGetSetPairedCamerasCorrectorType() {
        AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getPairedCamerasCorrectorType(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_CORRECTOR_TYPE);

        //set new value
        assertSame(cfg.setPairedCamerasCorrectorType(
                CorrectorType.GOLD_STANDARD), cfg);

        //check correctness
        assertEquals(cfg.getPairedCamerasCorrectorType(),
                CorrectorType.GOLD_STANDARD);
    }

    @Test
    public void testGetSetPairedCamerasMarkValidTriangulatedPoints() {
        AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getPairedCamerasMarkValidTriangulatedPoints(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_MARK_VALID_TRIANGULATED_POINTS);

        //set new value
        assertSame(cfg.setPairedCamerasMarkValidTriangulatedPoints(false),
                cfg);

        //check correctness
        assertFalse(cfg.getPairedCamerasMarkValidTriangulatedPoints());
    }

    @Test
    public void testAreSetIntrinsicParametersKnown() {
        AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.areIntrinsicParametersKnown(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KNOWN_INTRINSIC_PARAMETERS);

        //set new value
        assertSame(cfg.setIntrinsicParametersKnown(
                !AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KNOWN_INTRINSIC_PARAMETERS), cfg);

        //check correctness
        assertEquals(cfg.areIntrinsicParametersKnown(),
                !AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KNOWN_INTRINSIC_PARAMETERS);
    }

    @Test
    public void testIsSetGeneralSceneAllowed() {
        AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.isGeneralSceneAllowed(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_GENERAL_SCENE);

        //set new value
        assertSame(cfg.setGeneralSceneAllowed(
                !AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_GENERAL_SCENE), cfg);

        //check correctness
        assertEquals(cfg.isGeneralSceneAllowed(),
                !AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_GENERAL_SCENE);
    }

    @Test
    public void testIsSetPlanarSceneAllowed() {
        AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.isPlanarSceneAllowed(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_PLANAR_SCENE);

        //set new value
        assertSame(cfg.setPlanarSceneAllowed(
                !AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_PLANAR_SCENE), cfg);

        //check correctness
        assertEquals(cfg.isPlanarSceneAllowed(),
                !AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_PLANAR_SCENE);
    }

    @Test
    public void testGetSetRobustPlanarHomographyEstimatorMethod() {
        AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getRobustPlanarHomographyEstimatorMethod(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ROBUST_PLANAR_HOMOGRAPHY_ESTIMATOR_METHOD);

        //set new value
        assertSame(cfg.setRobustPlanarHomographyEstimatorMethod(
                RobustEstimatorMethod.RANSAC), cfg);

        //check correctness
        assertEquals(cfg.getRobustPlanarHomographyEstimatorMethod(),
                RobustEstimatorMethod.RANSAC);
    }

    @Test
    public void testIsSetPlanarHomographyRefined() {
        AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.isPlanarHomographyRefined(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_REFINE_PLANAR_HOMOGRAPHY);

        //set new value
        assertSame(cfg.setPlanarHomographyRefined(
                !AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_REFINE_PLANAR_HOMOGRAPHY), cfg);

        //check correctness
        assertEquals(cfg.isPlanarHomographyRefined(),
                !AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_REFINE_PLANAR_HOMOGRAPHY);
    }

    @Test
    public void testIsSetPlanarHomographyCovarianceKept() {
        AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.isPlanarHomographyCovarianceKept(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE);

        //set new value
        assertSame(cfg.setPlanarHomographyCovarianceKept(
                !AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE), cfg);

        //check correctness
        assertEquals(cfg.isPlanarHomographyCovarianceKept(),
                !AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE);
    }

    @Test
    public void testGetSetPlanarHomographyConfidence() {
        AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getPlanarHomographyConfidence(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_CONFIDENCE, 0.0);

        //set new value
        assertSame(cfg.setPlanarHomographyConfidence(0.5), cfg);

        //check correctness
        assertEquals(cfg.getPlanarHomographyConfidence(), 0.5, 0.0);
    }

    @Test
    public void testGetSetPlanarHomographyMaxIterations() {
        AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getPlanarHomographyMaxIterations(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_MAX_ITERATIONS);

        //set new value
        assertSame(cfg.setPlanarHomographyMaxIterations(100), cfg);

        //check correctness
        assertEquals(cfg.getPlanarHomographyMaxIterations(), 100);
    }

    @Test
    public void testGetSetPlanarHomographyThreshold() {
        AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getPlanarHomographyThreshold(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_THRESHOLD, 0.0);

        //set new value
        assertSame(cfg.setPlanarHomographyThreshold(0.5), cfg);

        //check correctness
        assertEquals(cfg.getPlanarHomographyThreshold(), 0.5, 0.0);
    }

    @Test
    public void testGetSetPlanarHomographyComputeAndKeepInliers() {
        AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getPlanarHomographyComputeAndKeepInliers(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS);

        //set new value
        assertSame(cfg.setPlanarHomographyComputeAndKeepInliers(
                !AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS), cfg);

        //check correctness
        assertEquals(cfg.getPlanarHomographyComputeAndKeepInliers(),
                !AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS);
    }

    @Test
    public void testGetSetPlanarHomographyComputeAndKeepResiduals() {
        AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getPlanarHomographyComputeAndKeepResiduals(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS);

        //set new value
        assertSame(cfg.setPlanarHomographyComputeAndKeepResiduals(
                !AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS), cfg);

        //check correctness
        assertEquals(!cfg.getPlanarHomographyComputeAndKeepResiduals(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS);
    }

    @Test
    public void testGetSetCalibrationData() {
        AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        //check default value
        assertNull(cfg.getCalibrationData());

        //set new value
        AbsoluteOrientationConstantVelocityModelSlamCalibrationData calibrationData =
                new AbsoluteOrientationConstantVelocityModelSlamCalibrationData();
        assertSame(cfg.setCalibrationData(calibrationData), cfg);

        //check correctness
        assertSame(cfg.getCalibrationData(), calibrationData);
    }

    @Test
    public void testGetSetCameraPositionCovarianceAndVariance() throws AlgebraException {
        AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        //check default value
        assertNotNull(cfg.getCameraPositionCovariance());

        Matrix cov = Matrix.identity(POINT_INHOM_COORDS, POINT_INHOM_COORDS);
        cov.multiplyByScalar(CAMERA_POSITION_VARIANCE);
        assertEquals(cfg.getCameraPositionCovariance(), cov);

        //set new value
        cov = Matrix.identity(POINT_INHOM_COORDS, POINT_INHOM_COORDS);
        cov.multiplyByScalar(2.0);
        assertSame(cfg.setCameraPositionCovariance(cov), cfg);

        //check
        assertSame(cfg.getCameraPositionCovariance(), cov);

        //set variance
        assertSame(cfg.setCameraPositionVariance(5.0), cfg);

        //check
        cov = Matrix.identity(POINT_INHOM_COORDS, POINT_INHOM_COORDS);
        cov.multiplyByScalar(5.0);
        assertEquals(cfg.getCameraPositionCovariance(), cov);
    }

    @Test
    public void testIsSetNotifyAvailableSlamDataEnabled() {
        AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.isNotifyAvailableSlamDataEnabled(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_NOTIFY_SLAM_DATA_AVAILABLE);

        //set new value
        assertSame(cfg.setNotifyAvailableSlamDataEnabled(
                !AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_NOTIFY_SLAM_DATA_AVAILABLE), cfg);

        //check correctness
        assertEquals(cfg.isNotifyAvailableSlamDataEnabled(),
                !AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_NOTIFY_SLAM_DATA_AVAILABLE);
    }

    @Test
    public void testIsSetNotifyEstimatedSlamCameraEnabled() {
        AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.isNotifyEstimatedSlamCameraEnabled(),
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_NOTIFY_ESTIMATED_SLAM_CAMERA);

        //set new value
        assertSame(cfg.setNotifyEstimatedSlamCameraEnabled(
                !AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_NOTIFY_ESTIMATED_SLAM_CAMERA), cfg);

        //check correctness
        assertEquals(cfg.isNotifyEstimatedSlamCameraEnabled(),
                !AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_NOTIFY_ESTIMATED_SLAM_CAMERA);
    }
}
