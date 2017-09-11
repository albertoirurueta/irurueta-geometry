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

import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.epipolar.CorrectorType;
import com.irurueta.geometry.epipolar.estimators.FundamentalMatrixEstimatorMethod;
import com.irurueta.geometry.slam.AbsoluteOrientationSlamCalibrationData;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import org.junit.*;
import static org.junit.Assert.*;

public class AbsoluteOrientationSlamSparseReconstructorConfigurationTest {

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
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default values
        assertEquals(cfg.getNonRobustFundamentalMatrixEstimatorMethod(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_NON_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD);
        assertEquals(cfg.getRobustFundamentalMatrixEstimatorMethod(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD);
        assertEquals(cfg.isFundamentalMatrixRefined(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_REFINE_FUNDAMENTAL_MATRIX);
        assertEquals(cfg.isFundamentalMatrixCovarianceKept(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_KEEP_FUNDAMENTAL_MATRIX_COVARIANCE);
        assertEquals(cfg.getFundamentalMatrixConfidence(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_FUNDAMENTAL_MATRIX_CONFIDENCE,
                0.0);
        assertEquals(cfg.getFundamentalMatrixMaxIterations(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_FUNDAMENTAL_MATRIX_MAX_ITERATIONS);
        assertEquals(cfg.getFundamentalMatrixThreshold(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_FUNDAMENTAL_MATRIX_THRESHOLD,
                0.0);
        assertEquals(cfg.getFundamentalMatrixComputeAndKeepInliers(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(cfg.getFundamentalMatrixComputeAndKeepResiduals(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(cfg.getInitialCamerasEstimatorMethod(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_INITIAL_CAMERAS_ESTIMATOR_METHOD);
        assertEquals(cfg.getDaqUseHomogeneousPointTriangulator(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_DAQ_USE_HOMOGENEOUS_POINT_TRIANGULATOR);
        assertEquals(cfg.getInitialCamerasAspectRatio(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_INITIAL_CAMERAS_ASPECT_RATIO,
                0.0);
        assertEquals(cfg.getPrincipalPointX(), 0.0, 0.0);
        assertEquals(cfg.getPrincipalPointY(), 0.0, 0.0);
        assertEquals(cfg.getInitialCamerasCorrectorType(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_INITIAL_CAMERAS_CORRECTOR_TYPE);
        assertEquals(cfg.getInitialCamerasMarkValidTriangulatedPoints(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_INITIAL_CAMERAS_MARK_VALID_TRIANGULATED_POINTS);
        assertNull(cfg.getInitialIntrinsic1());
        assertNull(cfg.getInitialIntrinsic2());
        assertEquals(cfg.isGeneralSceneAllowed(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_ALLOW_GENERAL_SCENE);
        assertEquals(cfg.isPlanarSceneAllowed(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_ALLOW_PLANAR_SCENE);
        assertEquals(cfg.getRobustPlanarHomographyEstimatorMethod(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ROBUST_PLANAR_HOMOGRAPHY_ESTIMATOR_METHOD);
        assertEquals(cfg.isPlanarHomographyRefined(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_REFINE_PLANAR_HOMOGRAPHY);
        assertEquals(cfg.isPlanarHomographyCovarianceKept(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE);
        assertEquals(cfg.getPlanarHomographyConfidence(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_PLANAR_HOMOGRAPHY_CONFIDENCE,
                0.0);
        assertEquals(cfg.getPlanarHomographyMaxIterations(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_PLANAR_HOMOGRAPHY_MAX_ITERATIONS);
        assertEquals(cfg.getPlanarHomographyThreshold(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_PLANAR_HOMOGRAPHY_THRESHOLD,
                0.0);
        assertEquals(cfg.getPlanarHomographyComputeAndKeepInliers(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(cfg.getPlanarHomographyComputeAndKeepResiduals(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(cfg.getUseDAQForAdditionalCamerasIntrinsics(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_DAQ_FOR_ADDITIONAL_CAMERAS_INTRINSICS);
        assertEquals(cfg.getUseDIACForAdditionalCamerasIntrinsics(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_DIAC_FOR_ADDITIONAL_CAMERAS_INTRINSICS);
        assertNull(cfg.getAdditionalCamerasIntrinsics());
        assertEquals(cfg.getAdditionalCamerasSkewness(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_ADDITIONAL_CAMERAS_SKEWNESS,
                0.0);
        assertEquals(cfg.getAdditionalCamerasHorizontalPrincipalPoint(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_HORIZONTAL_PRINCIPAL_POINT, 0.0);
        assertEquals(cfg.getAdditionalCamerasVerticalPrincipalPoint(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_VERTICAL_PRINCIPAL_POINT, 0.0);
        assertEquals(cfg.getAdditionalCamerasAspectRatio(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_ADDITIONAL_CAMERAS_ASPECT_RATIO, 0.0);
        assertEquals(cfg.getUseEPnPForAdditionalCamerasEstimation(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_EPNP_FOR_ADDITIONAL_CAMERAS_ESTIMATION);
        assertEquals(cfg.getUseUPnPForAdditionalCamerasEstimation(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_UPNP_FOR_ADDITIONAL_CAMERAS_ESTIMATION);
        assertEquals(cfg.getAdditionalCamerasRobustEstimationMethod(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ROBUST_ESTIMATION_METHOD);
        assertEquals(cfg.getAdditionalCamerasAllowPlanarConfiguration(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ALLOW_PLANAR_CONFIGURATION);
        assertEquals(cfg.getAdditionalCamerasAllowNullspaceDimension2(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ALLOW_NULLSPACE_DIMENSION2);
        assertEquals(cfg.getAdditionalCamerasAllowNullspaceDimension3(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ALLOW_NULLSPACE_DIMENSION3);
        assertEquals(cfg.getAdditionalCamerasPlanarThreshold(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_ADDITIONAL_CAMERAS_PLANAR_THRESHOLD,
                0.0);
        assertEquals(cfg.areAdditionalCamerasRefined(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_REFINE_ADDITIONAL_CAMERAS);
        assertEquals(cfg.isAdditionalCamerasCovarianceKept(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_KEEP_COVARIANCE_ADDITIONAL_CAMERAS);
        assertEquals(cfg.getAdditionalCamerasUseFastRefinement(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_USE_FAST_REFINEMENT);
        assertEquals(cfg.getAdditionalCamerasConfidence(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_ADDITIONAL_CAMERAS_CONFIDENCE,
                0.0);
        assertEquals(cfg.getAdditionalCamerasMaxIterations(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_ADDITIONAL_CAMERAS_MAX_ITERATIONS);
        assertEquals(cfg.isAdditionalCamerasSuggestSkewnessValueEnabled(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_SKEWNESS_VALUE_ENABLED);
        assertEquals(cfg.getAdditionalCamerasSuggestedSkewnessValue(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGESTED_SKEWNESS_VALUE, 0.0);
        assertEquals(cfg.isAdditionalCamerasSuggestHorizontalFocalLengthEnabled(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);
        assertEquals(cfg.getAdditionalCamerasSuggestedHorizontalFocalLengthValue(), 0.0, 0.0);
        assertEquals(cfg.isAdditionalCamerasSuggestVerticalFocalLengthEnabled(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED);
        assertEquals(cfg.getAdditionalCamerasSuggestedVerticalFocalLengthValue(), 0.0, 0.0);
        assertEquals(cfg.isAdditionalCamerasSuggestAspectRatioEnabled(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_ASPECT_RATIO_ENABLED);
        assertEquals(cfg.getAdditionalCamerasSuggestedAspectRatioValue(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGESTED_ASPECT_RATIO_VALUE, 0.0);
        assertEquals(cfg.isAdditionalCamerasSuggestPrincipalPointEnabled(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_PRINCIPAL_POINT_ENABLED);
        assertNull(cfg.getAdditionalCamerasSuggestedPrincipalPointValue());
        assertEquals(cfg.isHomogeneousPointTriangulatorUsed(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_USE_HOMOGENEOUS_POINT_TRIANGULATOR);
        assertEquals(cfg.getRobustPointTriangulatorMethod(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_ROBUST_POINT_TRIANGULATOR_METHOD);
        assertEquals(cfg.getPointTriangulatorConfidence(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_POINT_TRIANGULATOR_CONFIDENCE,
                0.0);
        assertEquals(cfg.getPointTriangulatorMaxIterations(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_POINT_TRIANGULATOR_MAX_ITERATIONS);
        assertEquals(cfg.getPointTriangulatorThreshold(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_POINT_TRIANGULATOR_THRESHOLD,
                0.0);
        assertNull(cfg.getCalibrationData());
    }

    @Test
    public void testMake() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                AbsoluteOrientationSlamSparseReconstructorConfiguration.make();

        //check default values
        assertEquals(cfg.getNonRobustFundamentalMatrixEstimatorMethod(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_NON_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD);
        assertEquals(cfg.getRobustFundamentalMatrixEstimatorMethod(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD);
        assertEquals(cfg.isFundamentalMatrixRefined(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_REFINE_FUNDAMENTAL_MATRIX);
        assertEquals(cfg.isFundamentalMatrixCovarianceKept(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_KEEP_FUNDAMENTAL_MATRIX_COVARIANCE);
        assertEquals(cfg.getFundamentalMatrixConfidence(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_FUNDAMENTAL_MATRIX_CONFIDENCE,
                0.0);
        assertEquals(cfg.getFundamentalMatrixMaxIterations(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_FUNDAMENTAL_MATRIX_MAX_ITERATIONS);
        assertEquals(cfg.getFundamentalMatrixThreshold(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_FUNDAMENTAL_MATRIX_THRESHOLD,
                0.0);
        assertEquals(cfg.getFundamentalMatrixComputeAndKeepInliers(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(cfg.getFundamentalMatrixComputeAndKeepResiduals(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(cfg.getInitialCamerasEstimatorMethod(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_INITIAL_CAMERAS_ESTIMATOR_METHOD);
        assertEquals(cfg.getDaqUseHomogeneousPointTriangulator(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_DAQ_USE_HOMOGENEOUS_POINT_TRIANGULATOR);
        assertEquals(cfg.getInitialCamerasAspectRatio(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_INITIAL_CAMERAS_ASPECT_RATIO,
                0.0);
        assertEquals(cfg.getPrincipalPointX(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_INITIAL_CAMERAS_PRINCIPAL_POINT_X,
                0.0);
        assertEquals(cfg.getPrincipalPointY(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_INITIAL_CAMERAS_PRINCIPAL_POINT_Y,
                0.0);
        assertEquals(cfg.getInitialCamerasCorrectorType(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_INITIAL_CAMERAS_CORRECTOR_TYPE);
        assertEquals(cfg.getInitialCamerasMarkValidTriangulatedPoints(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_INITIAL_CAMERAS_MARK_VALID_TRIANGULATED_POINTS);
        assertNull(cfg.getInitialIntrinsic1());
        assertNull(cfg.getInitialIntrinsic2());
        assertEquals(cfg.isGeneralSceneAllowed(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_ALLOW_GENERAL_SCENE);
        assertEquals(cfg.isPlanarSceneAllowed(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_ALLOW_PLANAR_SCENE);
        assertEquals(cfg.getRobustPlanarHomographyEstimatorMethod(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ROBUST_PLANAR_HOMOGRAPHY_ESTIMATOR_METHOD);
        assertEquals(cfg.isPlanarHomographyRefined(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_REFINE_PLANAR_HOMOGRAPHY);
        assertEquals(cfg.isPlanarHomographyCovarianceKept(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE);
        assertEquals(cfg.getPlanarHomographyConfidence(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_PLANAR_HOMOGRAPHY_CONFIDENCE,
                0.0);
        assertEquals(cfg.getPlanarHomographyMaxIterations(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_PLANAR_HOMOGRAPHY_MAX_ITERATIONS);
        assertEquals(cfg.getPlanarHomographyThreshold(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_PLANAR_HOMOGRAPHY_THRESHOLD,
                0.0);
        assertEquals(cfg.getPlanarHomographyComputeAndKeepInliers(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(cfg.getPlanarHomographyComputeAndKeepResiduals(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(cfg.getUseDAQForAdditionalCamerasIntrinsics(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_DAQ_FOR_ADDITIONAL_CAMERAS_INTRINSICS);
        assertEquals(cfg.getUseDIACForAdditionalCamerasIntrinsics(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_DIAC_FOR_ADDITIONAL_CAMERAS_INTRINSICS);
        assertNull(cfg.getAdditionalCamerasIntrinsics());
        assertEquals(cfg.getAdditionalCamerasSkewness(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_ADDITIONAL_CAMERAS_SKEWNESS,
                0.0);
        assertEquals(cfg.getAdditionalCamerasHorizontalPrincipalPoint(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_HORIZONTAL_PRINCIPAL_POINT, 0.0);
        assertEquals(cfg.getAdditionalCamerasVerticalPrincipalPoint(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_VERTICAL_PRINCIPAL_POINT, 0.0);
        assertEquals(cfg.getAdditionalCamerasAspectRatio(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_ADDITIONAL_CAMERAS_ASPECT_RATIO, 0.0);
        assertEquals(cfg.getUseEPnPForAdditionalCamerasEstimation(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_EPNP_FOR_ADDITIONAL_CAMERAS_ESTIMATION);
        assertEquals(cfg.getUseUPnPForAdditionalCamerasEstimation(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_UPNP_FOR_ADDITIONAL_CAMERAS_ESTIMATION);
        assertEquals(cfg.getAdditionalCamerasRobustEstimationMethod(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ROBUST_ESTIMATION_METHOD);
        assertEquals(cfg.getAdditionalCamerasAllowPlanarConfiguration(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ALLOW_PLANAR_CONFIGURATION);
        assertEquals(cfg.getAdditionalCamerasAllowNullspaceDimension2(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ALLOW_NULLSPACE_DIMENSION2);
        assertEquals(cfg.getAdditionalCamerasAllowNullspaceDimension3(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ALLOW_NULLSPACE_DIMENSION3);
        assertEquals(cfg.getAdditionalCamerasPlanarThreshold(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_ADDITIONAL_CAMERAS_PLANAR_THRESHOLD,
                0.0);
        assertEquals(cfg.areAdditionalCamerasRefined(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_REFINE_ADDITIONAL_CAMERAS);
        assertEquals(cfg.isAdditionalCamerasCovarianceKept(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_KEEP_COVARIANCE_ADDITIONAL_CAMERAS);
        assertEquals(cfg.getAdditionalCamerasUseFastRefinement(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_USE_FAST_REFINEMENT);
        assertEquals(cfg.getAdditionalCamerasConfidence(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_ADDITIONAL_CAMERAS_CONFIDENCE,
                0.0);
        assertEquals(cfg.getAdditionalCamerasMaxIterations(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_ADDITIONAL_CAMERAS_MAX_ITERATIONS);
        assertEquals(cfg.isAdditionalCamerasSuggestSkewnessValueEnabled(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_SKEWNESS_VALUE_ENABLED);
        assertEquals(cfg.getAdditionalCamerasSuggestedSkewnessValue(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGESTED_SKEWNESS_VALUE, 0.0);
        assertEquals(cfg.isAdditionalCamerasSuggestHorizontalFocalLengthEnabled(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);
        assertEquals(cfg.getAdditionalCamerasSuggestedHorizontalFocalLengthValue(), 0.0, 0.0);
        assertEquals(cfg.isAdditionalCamerasSuggestVerticalFocalLengthEnabled(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED);
        assertEquals(cfg.getAdditionalCamerasSuggestedVerticalFocalLengthValue(), 0.0, 0.0);
        assertEquals(cfg.isAdditionalCamerasSuggestAspectRatioEnabled(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_ASPECT_RATIO_ENABLED);
        assertEquals(cfg.getAdditionalCamerasSuggestedAspectRatioValue(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGESTED_ASPECT_RATIO_VALUE, 0.0);
        assertEquals(cfg.isAdditionalCamerasSuggestPrincipalPointEnabled(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_PRINCIPAL_POINT_ENABLED);
        assertNull(cfg.getAdditionalCamerasSuggestedPrincipalPointValue());
        assertEquals(cfg.isHomogeneousPointTriangulatorUsed(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_USE_HOMOGENEOUS_POINT_TRIANGULATOR);
        assertEquals(cfg.getRobustPointTriangulatorMethod(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_ROBUST_POINT_TRIANGULATOR_METHOD);
        assertEquals(cfg.getPointTriangulatorConfidence(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_POINT_TRIANGULATOR_CONFIDENCE,
                0.0);
        assertEquals(cfg.getPointTriangulatorMaxIterations(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_POINT_TRIANGULATOR_MAX_ITERATIONS);
        assertEquals(cfg.getPointTriangulatorThreshold(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_POINT_TRIANGULATOR_THRESHOLD,
                0.0);
        assertNull(cfg.getCalibrationData());
    }

    @Test
    public void testGetSetNonRobustFundamentalMatrixEstimatorMethod() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getNonRobustFundamentalMatrixEstimatorMethod(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
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
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getRobustFundamentalMatrixEstimatorMethod(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
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
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.isFundamentalMatrixRefined(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_REFINE_FUNDAMENTAL_MATRIX);

        //set new value
        assertSame(cfg.setFundamentalMatrixRefined(false), cfg);

        //check correctness
        assertFalse(cfg.isFundamentalMatrixRefined());
    }

    @Test
    public void testIsSetFundamentalMatrixCovarianceKept() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.isFundamentalMatrixCovarianceKept(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_KEEP_FUNDAMENTAL_MATRIX_COVARIANCE);

        //set new value
        assertSame(cfg.setFundamentalMatrixCovarianceKept(true), cfg);

        //check correctness
        assertTrue(cfg.isFundamentalMatrixCovarianceKept());
    }

    @Test
    public void testGetSetFundamentalMatrixConfidence() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getFundamentalMatrixConfidence(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_FUNDAMENTAL_MATRIX_CONFIDENCE,
                0.0);

        //set new value
        assertSame(cfg.setFundamentalMatrixConfidence(0.7), cfg);

        //check correctness
        assertEquals(cfg.getFundamentalMatrixConfidence(), 0.7, 0.0);
    }

    @Test
    public void testGetSetFundamentalMatrixMaxIterations() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getFundamentalMatrixMaxIterations(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_FUNDAMENTAL_MATRIX_MAX_ITERATIONS);

        //set new value
        assertSame(cfg.setFundamentalMatrixMaxIterations(10), cfg);

        //check correctness
        assertEquals(cfg.getFundamentalMatrixMaxIterations(), 10);
    }

    @Test
    public void testGetSetFundamentalMatrixThreshold() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getFundamentalMatrixThreshold(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_FUNDAMENTAL_MATRIX_THRESHOLD,
                0.0);

        //set new value
        assertSame(cfg.setFundamentalMatrixThreshold(2.0), cfg);

        //check correctness
        assertEquals(cfg.getFundamentalMatrixThreshold(), 2.0, 0.0);
    }

    @Test
    public void testGetSetFundamentalMatrixComputeAndKeepInliers() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getFundamentalMatrixComputeAndKeepInliers(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_INLIERS);

        //set new value
        assertSame(cfg.setFundamentalMatrixComputeAndKeepInliers(false), cfg);

        //check correctness
        assertFalse(cfg.getFundamentalMatrixComputeAndKeepInliers());
    }

    @Test
    public void testGetSetFundamentalMatrixComputeAndKeepResiduals() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getFundamentalMatrixComputeAndKeepResiduals(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_RESIDUALS);

        //set new value
        assertSame(cfg.setFundamentalMatrixComputeAndKeepResiduals(false), cfg);

        //check correctness
        assertFalse(cfg.getFundamentalMatrixComputeAndKeepResiduals());
    }

    @Test
    public void testGetSetInitialCamerasEstimatorMethod() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getInitialCamerasEstimatorMethod(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_INITIAL_CAMERAS_ESTIMATOR_METHOD);

        //set new value
        assertSame(cfg.setInitialCamerasEstimatorMethod(
                InitialCamerasEstimatorMethod.DUAL_IMAGE_OF_ABSOLUTE_CONIC),
                cfg);

        //check correctness
        assertEquals(cfg.getInitialCamerasEstimatorMethod(),
                InitialCamerasEstimatorMethod.DUAL_IMAGE_OF_ABSOLUTE_CONIC);
    }

    @Test
    public void testGetSetDaqUseHomogeneousPointTriangulator() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getDaqUseHomogeneousPointTriangulator(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_DAQ_USE_HOMOGENEOUS_POINT_TRIANGULATOR);

        //set new value
        assertSame(cfg.setDaqUseHomogeneousPointTriangulator(false), cfg);

        //check correctness
        assertFalse(cfg.getDaqUseHomogeneousPointTriangulator());
    }

    @Test
    public void testGetSetInitialCamerasAspectRatio() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getInitialCamerasAspectRatio(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_INITIAL_CAMERAS_ASPECT_RATIO,
                0.0);

        //set new value
        assertSame(cfg.setInitialCamerasAspectRatio(0.5), cfg);

        //check correctness
        assertEquals(cfg.getInitialCamerasAspectRatio(), 0.5, 0.0);
    }

    @Test
    public void testGetSetPrincipalPointX() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getPrincipalPointX(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_INITIAL_CAMERAS_PRINCIPAL_POINT_X,
                0.0);

        //set new value
        assertSame(cfg.setPrincipalPointX(10.0), cfg);

        //check correctness
        assertEquals(cfg.getPrincipalPointX(), 10.0, 0.0);
    }

    @Test
    public void testGetSetPrincipalPointY() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getPrincipalPointY(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_INITIAL_CAMERAS_PRINCIPAL_POINT_Y,
                0.0);

        //set new value
        assertSame(cfg.setPrincipalPointY(10.0), cfg);

        //check correctness
        assertEquals(cfg.getPrincipalPointY(), 10.0, 0.0);
    }

    @Test
    public void testGetSetInitialCamerasCorrectorType() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getInitialCamerasCorrectorType(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_INITIAL_CAMERAS_CORRECTOR_TYPE);

        //set new value
        assertSame(cfg.setInitialCamerasCorrectorType(
                CorrectorType.GOLD_STANDARD), cfg);

        //check correctness
        assertEquals(cfg.getInitialCamerasCorrectorType(),
                CorrectorType.GOLD_STANDARD);
    }

    @Test
    public void testGetSetInitialCamerasMarkValidTriangulatedPoints() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getInitialCamerasMarkValidTriangulatedPoints(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_INITIAL_CAMERAS_MARK_VALID_TRIANGULATED_POINTS);

        //set new value
        assertSame(cfg.setInitialCamerasMarkValidTriangulatedPoints(false),
                cfg);

        //check correctness
        assertFalse(cfg.getInitialCamerasMarkValidTriangulatedPoints());
    }

    @Test
    public void testGetSetInitialIntrinsic1() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertNull(cfg.getInitialIntrinsic1());

        //set new value
        PinholeCameraIntrinsicParameters intrinsic =
                new PinholeCameraIntrinsicParameters();
        assertSame(cfg.setInitialIntrinsic1(intrinsic), cfg);

        //check correctness
        assertSame(cfg.getInitialIntrinsic1(), intrinsic);
    }

    @Test
    public void testGetSetInitialIntrinsic2() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertNull(cfg.getInitialIntrinsic2());

        //set new value
        PinholeCameraIntrinsicParameters intrinsic =
                new PinholeCameraIntrinsicParameters();
        assertSame(cfg.setInitialIntrinsic2(intrinsic), cfg);

        //check correctness
        assertSame(cfg.getInitialIntrinsic2(), intrinsic);
    }

    @Test
    public void testIsSetGeneralSceneAllowed() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.isGeneralSceneAllowed(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_ALLOW_GENERAL_SCENE);

        //set new value
        assertSame(cfg.setGeneralSceneAllowed(
                !AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_ALLOW_GENERAL_SCENE), cfg);

        //check correctness
        assertEquals(cfg.isGeneralSceneAllowed(),
                !AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_ALLOW_GENERAL_SCENE);
    }

    @Test
    public void testIsSetPlanarSceneAllowed() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.isPlanarSceneAllowed(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_ALLOW_PLANAR_SCENE);

        //set new value
        assertSame(cfg.setPlanarSceneAllowed(
                !AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_ALLOW_PLANAR_SCENE), cfg);

        //check correctness
        assertEquals(cfg.isPlanarSceneAllowed(),
                !AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_ALLOW_PLANAR_SCENE);
    }

    @Test
    public void testGetSetRobustPlanarHomographyEstimatorMethod() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getRobustPlanarHomographyEstimatorMethod(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
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
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.isPlanarHomographyRefined(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_REFINE_PLANAR_HOMOGRAPHY);

        //set new value
        assertSame(cfg.setPlanarHomographyRefined(
                !AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_REFINE_PLANAR_HOMOGRAPHY), cfg);

        //check correctness
        assertEquals(cfg.isPlanarHomographyRefined(),
                !AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_REFINE_PLANAR_HOMOGRAPHY);
    }

    @Test
    public void testIsSetPlanarHomographyCovarianceKept() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.isPlanarHomographyCovarianceKept(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE);

        //set new value
        assertSame(cfg.setPlanarHomographyCovarianceKept(
                !AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE),
                cfg);

        //check correctness
        assertEquals(cfg.isPlanarHomographyCovarianceKept(),
                !AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE);
    }

    @Test
    public void testGetSetPlanarHomographyConfidence() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getPlanarHomographyConfidence(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_PLANAR_HOMOGRAPHY_CONFIDENCE,
                0.0);

        //set new value
        assertSame(cfg.setPlanarHomographyConfidence(0.5), cfg);

        //check correctness
        assertEquals(cfg.getPlanarHomographyConfidence(), 0.5, 0.0);
    }

    @Test
    public void testGetSetPlanarHomographyMaxIterations() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getPlanarHomographyMaxIterations(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_PLANAR_HOMOGRAPHY_MAX_ITERATIONS);

        //set new value
        assertSame(cfg.setPlanarHomographyMaxIterations(100), cfg);

        //check correctness
        assertEquals(cfg.getPlanarHomographyMaxIterations(), 100);
    }

    @Test
    public void testGetSetPlanarHomographyThreshold() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getPlanarHomographyThreshold(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_PLANAR_HOMOGRAPHY_THRESHOLD,
                0.0);

        //set new value
        assertSame(cfg.setPlanarHomographyThreshold(0.5), cfg);

        //check correctness
        assertEquals(cfg.getPlanarHomographyThreshold(), 0.5, 0.0);
    }

    @Test
    public void testGetSetPlanarHomographyComputeAndKeepInliers() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getPlanarHomographyComputeAndKeepInliers(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS);

        //set new value
        assertSame(cfg.setPlanarHomographyComputeAndKeepInliers(
                !AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS), cfg);

        //check correctness
        assertEquals(cfg.getPlanarHomographyComputeAndKeepInliers(),
                !AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS);
    }

    @Test
    public void testGetSetPlanarHomographyComputeAndKeepResiduals() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getPlanarHomographyComputeAndKeepResiduals(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS);

        //set new value
        assertSame(cfg.setPlanarHomographyComputeAndKeepResiduals(
                !AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS), cfg);

        //check correctness
        assertEquals(!cfg.getPlanarHomographyComputeAndKeepResiduals(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS);
    }

    @Test
    public void testGetSetUseDAQForAdditionalCamerasIntrinsics() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getUseDAQForAdditionalCamerasIntrinsics(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_DAQ_FOR_ADDITIONAL_CAMERAS_INTRINSICS);

        //set new value
        assertSame(cfg.setUseDAQForAdditionalCamerasIntrinics(
                !AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_DAQ_FOR_ADDITIONAL_CAMERAS_INTRINSICS), cfg);

        //check correctness
        assertEquals(!cfg.getUseDAQForAdditionalCamerasIntrinsics(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_DAQ_FOR_ADDITIONAL_CAMERAS_INTRINSICS);
    }

    @Test
    public void testGetSetUseDIACForAdditionalCamerasIntrinsics() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getUseDIACForAdditionalCamerasIntrinsics(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_DIAC_FOR_ADDITIONAL_CAMERAS_INTRINSICS);

        //set new value
        assertSame(cfg.setUseDIACForAdditionalCamerasIntrinsics(
                !AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_DIAC_FOR_ADDITIONAL_CAMERAS_INTRINSICS), cfg);

        //check correctness
        assertEquals(!cfg.getUseDIACForAdditionalCamerasIntrinsics(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_DIAC_FOR_ADDITIONAL_CAMERAS_INTRINSICS);
    }

    @Test
    public void testGetSetAdditionalCamerasIntrinsics() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertNull(cfg.getAdditionalCamerasIntrinsics());

        //set new value
        PinholeCameraIntrinsicParameters intrinsics = new PinholeCameraIntrinsicParameters();
        assertSame(cfg.setAdditionalCamerasIntrinsics(intrinsics), cfg);

        //check correctness
        assertSame(cfg.getAdditionalCamerasIntrinsics(), intrinsics);
    }

    @Test
    public void testGetSetAdditionalCamerasSkewness() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getAdditionalCamerasSkewness(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SKEWNESS, 0.0);

        //set new value
        assertSame(cfg.setAdditionalCamerasSkewness(1e-3), cfg);

        //check correctness
        assertEquals(cfg.getAdditionalCamerasSkewness(), 1e-3, 0.0);
    }

    @Test
    public void testGetSetAdditionalCamerasHorizontalPrincipalPoint() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getAdditionalCamerasHorizontalPrincipalPoint(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_HORIZONTAL_PRINCIPAL_POINT, 0.0);

        //set new value
        assertSame(cfg.setAdditionalCamerasHorizontalPrincipalPoint(320), cfg);

        //check correctness
        assertEquals(cfg.getAdditionalCamerasHorizontalPrincipalPoint(), 320, 0.0);
    }

    @Test
    public void testGetSetAdditionalCamerasVerticalPrincipalPoint() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getAdditionalCamerasVerticalPrincipalPoint(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_VERTICAL_PRINCIPAL_POINT, 0.0);

        //set new value
        assertSame(cfg.setAdditionalCamerasVerticalPrincipalPoint(240), cfg);

        //check correctness
        assertEquals(cfg.getAdditionalCamerasVerticalPrincipalPoint(), 240, 0.0);
    }

    @Test
    public void testGetSetAdditionalCamerasAspectRatio() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getAdditionalCamerasAspectRatio(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_ADDITIONAL_CAMERAS_ASPECT_RATIO, 0.0);

        //set new value
        assertSame(cfg.setAdditionalCamerasAspectRatio(-1.0), cfg);

        //check correctness
        assertEquals(cfg.getAdditionalCamerasAspectRatio(), -1.0, 0.0);
    }

    @Test
    public void testGetSetUseEPnPForAdditionalCamerasEstimation() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getUseEPnPForAdditionalCamerasEstimation(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_EPNP_FOR_ADDITIONAL_CAMERAS_ESTIMATION);

        //set new value
        assertSame(cfg.setUseEPnPForAdditionalCamerasEstimation(
                !AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_EPNP_FOR_ADDITIONAL_CAMERAS_ESTIMATION), cfg);

        //check correctness
        assertEquals(cfg.getUseEPnPForAdditionalCamerasEstimation(),
                !AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_EPNP_FOR_ADDITIONAL_CAMERAS_ESTIMATION);
    }

    @Test
    public void testGetSetUseUPnPForAdditionalCamerasEstimation() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getUseUPnPForAdditionalCamerasEstimation(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_UPNP_FOR_ADDITIONAL_CAMERAS_ESTIMATION);

        //set new value
        assertSame(cfg.setUseUPnPForAdditionalCamerasEstimation(
                !AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_UPNP_FOR_ADDITIONAL_CAMERAS_ESTIMATION), cfg);

        //check correctness
        assertEquals(cfg.getUseUPnPForAdditionalCamerasEstimation(),
                !AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_UPNP_FOR_ADDITIONAL_CAMERAS_ESTIMATION);
    }

    @Test
    public void testGetSetAdditionalCamerasRobustEstimationMethod() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getAdditionalCamerasRobustEstimationMethod(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ROBUST_ESTIMATION_METHOD);

        //set new value
        assertSame(cfg.setAdditionalCamerasRobustEstimationMethod(
                RobustEstimatorMethod.LMedS), cfg);

        //check correctness
        assertEquals(cfg.getAdditionalCamerasRobustEstimationMethod(),
                RobustEstimatorMethod.LMedS);
    }

    @Test
    public void testGetSetAdditionalCamerasAllowPlanarConfiguration() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getAdditionalCamerasAllowPlanarConfiguration(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ALLOW_PLANAR_CONFIGURATION);

        //set new value
        assertSame(cfg.setAdditionalCamerasAllowPlanarConfiguration(
                !AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ALLOW_PLANAR_CONFIGURATION), cfg);

        //check correctness
        assertEquals(cfg.getAdditionalCamerasAllowPlanarConfiguration(),
                !AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ALLOW_PLANAR_CONFIGURATION);
    }

    @Test
    public void testGetSetAdditionalCamerasAllowNullspaceDimension2() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getAdditionalCamerasAllowNullspaceDimension2(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ALLOW_NULLSPACE_DIMENSION2);

        //set new value
        assertSame(cfg.setAdditionalCamerasAllowNullspaceDimension2(
                !AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ALLOW_NULLSPACE_DIMENSION2), cfg);

        //check correctness
        assertEquals(cfg.getAdditionalCamerasAllowNullspaceDimension2(),
                !AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ALLOW_NULLSPACE_DIMENSION2);
    }

    @Test
    public void testGetSetAdditionalCamerasAllowNullspaceDimension3() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getAdditionalCamerasAllowNullspaceDimension3(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ALLOW_NULLSPACE_DIMENSION3);

        //set new value
        assertSame(cfg.setAdditionalCamerasAllowNullspaceDimension3(
                !AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ALLOW_NULLSPACE_DIMENSION3), cfg);

        //check correctness
        assertEquals(cfg.getAdditionalCamerasAllowNullspaceDimension3(),
                !AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ALLOW_NULLSPACE_DIMENSION3);
    }

    @Test
    public void testGetSetAdditionalCamerasPlanarThreshold() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getAdditionalCamerasPlanarThreshold(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_ADDITIONAL_CAMERAS_PLANAR_THRESHOLD,
                0.0);

        //set new value
        assertSame(cfg.setAdditionalCamerasPlanarThreshold(1e-3), cfg);

        //check correctness
        assertEquals(cfg.getAdditionalCamerasPlanarThreshold(), 1e-3, 0.0);
    }

    @Test
    public void testAreSetAdditionalCamerasRefined() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.areAdditionalCamerasRefined(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_REFINE_ADDITIONAL_CAMERAS);

        //set new value
        assertSame(cfg.setAdditionalCamerasRefined(
                !AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_REFINE_ADDITIONAL_CAMERAS), cfg);

        //check correctness
        assertEquals(cfg.areAdditionalCamerasRefined(),
                !AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_REFINE_ADDITIONAL_CAMERAS);
    }

    @Test
    public void testIsSetAdditionalCamerasCovarianceKept() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.isAdditionalCamerasCovarianceKept(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_KEEP_COVARIANCE_ADDITIONAL_CAMERAS);

        //set new value
        assertSame(cfg.setAdditionalCamerasCovarianceKept(
                !AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_KEEP_COVARIANCE_ADDITIONAL_CAMERAS),
                cfg);

        //check correctness
        assertEquals(cfg.isAdditionalCamerasCovarianceKept(),
                !AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_KEEP_COVARIANCE_ADDITIONAL_CAMERAS);
    }

    @Test
    public void testGetSetAdditionalCamerasUseFastRefinement() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getAdditionalCamerasUseFastRefinement(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_USE_FAST_REFINEMENT);

        //set new value
        assertSame(cfg.setAdditionalCamerasUseFastRefinement(
                !AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_USE_FAST_REFINEMENT), cfg);

        //check correctness
        assertEquals(cfg.getAdditionalCamerasUseFastRefinement(),
                !AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_USE_FAST_REFINEMENT);
    }

    @Test
    public void testGetSetAdditionalCamerasConfidence() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getAdditionalCamerasConfidence(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_CONFIDENCE, 0.0);

        //set new value
        assertSame(cfg.setAdditionalCamerasConfidence(0.8), cfg);

        //check correctness
        assertEquals(cfg.getAdditionalCamerasConfidence(), 0.8, 0.0);
    }

    @Test
    public void testGetSetAdditionalCamerasMaxIterations() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getAdditionalCamerasMaxIterations(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_ADDITIONAL_CAMERAS_MAX_ITERATIONS);

        //set new value
        assertSame(cfg.setAdditionalCamerasMaxIterations(100), cfg);

        //check correctness
        assertEquals(cfg.getAdditionalCamerasMaxIterations(), 100);
    }

    @Test
    public void testIsSetAdditionalCamerasSuggestSkewnessValueEnabled() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.isAdditionalCamerasSuggestSkewnessValueEnabled(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_SKEWNESS_VALUE_ENABLED);

        //set new value
        assertSame(cfg.setAdditionalCamerasSuggestSkewnessValueEnabled(
                !AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_SKEWNESS_VALUE_ENABLED), cfg);

        //check correctness
        assertEquals(cfg.isAdditionalCamerasSuggestSkewnessValueEnabled(),
                !AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_SKEWNESS_VALUE_ENABLED);
    }

    @Test
    public void testGetSetAdditionalCamerasSuggestedSkewnessValue() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getAdditionalCamerasSuggestedSkewnessValue(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGESTED_SKEWNESS_VALUE, 0.0);

        //set new value
        assertSame(cfg.setAdditionalCamerasSuggestedSkewnessValue(1e-3), cfg);

        //check correctness
        assertEquals(cfg.getAdditionalCamerasSuggestedSkewnessValue(), 1e-3, 0.0);
    }

    @Test
    public void testIsSetAdditionalCamerasSuggestHorizontalFocalLengthEnabled() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.isAdditionalCamerasSuggestHorizontalFocalLengthEnabled(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);

        //set new value
        assertSame(cfg.setAdditionalCamerasSuggestHorizontalFocalLengthEnabled(
                !AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED),
                cfg);

        //check correctness
        assertEquals(cfg.isAdditionalCamerasSuggestHorizontalFocalLengthEnabled(),
                !AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);
    }

    @Test
    public void testGetSetAdditionalCamerasSuggestedHorizontalFocalLengthValue() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getAdditionalCamerasSuggestedHorizontalFocalLengthValue(), 0.0, 0.0);

        //set new value
        assertSame(cfg.setAdditionalCamerasSuggestedHorizontalFocalLengthValue(320), cfg);

        //check correctness
        assertEquals(cfg.getAdditionalCamerasSuggestedHorizontalFocalLengthValue(), 320, 0.0);
    }

    @Test
    public void testIsSetAdditionalCamerasSuggestVerticalFocalLengthEnabled() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.isAdditionalCamerasSuggestVerticalFocalLengthEnabled(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED);

        //set new value
        assertSame(cfg.setAdditionalCamerasSuggestVerticalFocalLengthEnabled(
                !AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED),
                cfg);

        //check correctness
        assertEquals(cfg.isAdditionalCamerasSuggestVerticalFocalLengthEnabled(),
                !AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED);
    }

    @Test
    public void testGetSetAdditionalCamerasSuggestedVerticalFocalLengthValue() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getAdditionalCamerasSuggestedVerticalFocalLengthValue(), 0.0, 0.0);

        //set new value
        assertSame(cfg.setAdditionalCamerasSuggestedVerticalFocalLengthValue(240), cfg);

        //check correctness
        assertEquals(cfg.getAdditionalCamerasSuggestedVerticalFocalLengthValue(), 240, 0.0);
    }

    @Test
    public void testIsSetAdditionalCamerasSuggestAspectRatioEnabled() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.isAdditionalCamerasSuggestAspectRatioEnabled(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_ASPECT_RATIO_ENABLED);

        //set new value
        assertSame(cfg.setAdditionalCamerasSuggestAspectRatioEnabled(
                !AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_ASPECT_RATIO_ENABLED), cfg);

        //check correctness
        assertEquals(cfg.isAdditionalCamerasSuggestAspectRatioEnabled(),
                !AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_ASPECT_RATIO_ENABLED);
    }

    @Test
    public void testGetSetAdditionalCamerasSuggestedAspectRatioValue() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getAdditionalCamerasSuggestedAspectRatioValue(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGESTED_ASPECT_RATIO_VALUE, 0.0);

        //set new value
        assertSame(cfg.setAdditionalCamerasSuggestedAspectRatioValue(1.1), cfg);

        //check correctness
        assertEquals(cfg.getAdditionalCamerasSuggestedAspectRatioValue(), 1.1, 0.0);
    }

    @Test
    public void testIsSetAdditionalCamerasSuggestPrincipalPointEnabled() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.isAdditionalCamerasSuggestPrincipalPointEnabled(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_PRINCIPAL_POINT_ENABLED);

        //set new value
        assertSame(cfg.setAdditionalCamerasSuggestPrincipalPointEnabled(
                !AbsoluteOrientationSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_PRINCIPAL_POINT_ENABLED), cfg);
    }

    @Test
    public void testGetSetAdditionalCamerasSuggestedPrincipalPointValue() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertNull(cfg.getAdditionalCamerasSuggestedPrincipalPointValue());

        //set new value
        InhomogeneousPoint2D principalPoint = new InhomogeneousPoint2D();
        assertSame(cfg.setAdditionalCamerasSuggestedPrincipalPointValue(principalPoint), cfg);

        //check correctness
        assertSame(cfg.getAdditionalCamerasSuggestedPrincipalPointValue(), principalPoint);
    }

    @Test
    public void testIsSetHomogeneousPointTriangulatorUsed() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.isHomogeneousPointTriangulatorUsed(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_USE_HOMOGENEOUS_POINT_TRIANGULATOR);

        //set new value
        assertSame(cfg.setHomogeneousPointTriangulatorUsed(
                !AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_USE_HOMOGENEOUS_POINT_TRIANGULATOR),
                cfg);

        //check correctness
        assertEquals(cfg.isHomogeneousPointTriangulatorUsed(),
                !AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_USE_HOMOGENEOUS_POINT_TRIANGULATOR);
    }

    @Test
    public void testGetSetRobustPointTriangulatorMethod() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getRobustPointTriangulatorMethod(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_ROBUST_POINT_TRIANGULATOR_METHOD);

        //set new value
        assertSame(cfg.setRobustPointTriangulatorMethod(RobustEstimatorMethod.MSAC), cfg);

        //check correctness
        assertEquals(cfg.getRobustPointTriangulatorMethod(), RobustEstimatorMethod.MSAC);
    }

    @Test
    public void testGetSetPointTriangulatorConfidence() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getPointTriangulatorConfidence(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_POINT_TRIANGULATOR_CONFIDENCE, 0.0);

        //set new value
        assertSame(cfg.setPointTriangulatorConfidence(0.8), cfg);

        //check correctness
        assertEquals(cfg.getPointTriangulatorConfidence(), 0.8, 0.0);
    }

    @Test
    public void testGetSetPointTriangulatorMaxIterations() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getPointTriangulatorMaxIterations(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_POINT_TRIANGULATOR_MAX_ITERATIONS);

        //set new value
        assertSame(cfg.setPointTriangulatorMaxIterations(100), cfg);

        //check correctness
        assertEquals(cfg.getPointTriangulatorMaxIterations(), 100);
    }

    @Test
    public void testGetStPointTriangulatorThreshold() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getPointTriangulatorThreshold(),
                AbsoluteOrientationSlamSparseReconstructorConfiguration.DEFAULT_POINT_TRIANGULATOR_THRESHOLD, 0.0);

        //set new value
        assertSame(cfg.setPointTriangulatorThreshold(1e-3), cfg);

        //check correctness
        assertEquals(cfg.getPointTriangulatorThreshold(), 1e-3, 0.0);
    }

    @Test
    public void testGetSetCalibrationData() {
        AbsoluteOrientationSlamSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationSlamSparseReconstructorConfiguration();

        //check default value
        assertNull(cfg.getCalibrationData());

        //set new value
        AbsoluteOrientationSlamCalibrationData calibrationData = new AbsoluteOrientationSlamCalibrationData();
        assertSame(cfg.setCalibrationData(calibrationData), cfg);

        //check correctness
        assertSame(cfg.getCalibrationData(), calibrationData);
    }
}
