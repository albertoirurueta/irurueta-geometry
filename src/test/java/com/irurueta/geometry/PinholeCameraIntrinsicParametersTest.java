/*
 * Copyright (C) 2012 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.geometry;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.ar.calibration.DualImageOfAbsoluteConic;
import com.irurueta.ar.calibration.ImageOfAbsoluteConic;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.Random;

import static org.junit.Assert.*;

public class PinholeCameraIntrinsicParametersTest {
    
    private static final int INTRINSIC_ROWS = 3;
    private static final int INTRINSIC_COLS = 3;
    
    private static final double ABSOLUTE_ERROR = 1e-6;
    
    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;
    
    private static final double MIN_FOCAL_LENGTH = 1.0;
    private static final double MAX_FOCAL_LENGTH = 100.0;
    
    private static final double MIN_SKEWNESS = -1.0;
    private static final double MAX_SKEWNESS = 1.0;
    
    private static final double MIN_PRINCIPAL_POINT = 0.0;
    private static final double MAX_PRINCIPAL_POINT = 100.0;
    
    private static final double MIN_ASPECT_RATIO = 0.5;
    private static final double MAX_ASPECT_RATIO = 2.0;
    
    private static final int MIN_IMAGE_WIDTH = 640;
    private static final int MAX_IMAGE_WIDTH = 1024;
    
    private static final int MIN_IMAGE_HEIGHT = 480;
    private static final int MAX_IMAGE_HEIGHT = 768;
    
    public PinholeCameraIntrinsicParametersTest() { }
    
    @BeforeClass
    public static void setUpClass() { }
    
    @AfterClass
    public static void tearDownClass() { }
    
    @Before
    public void setUp() { }
    
    @After
    public void tearDown() { }
    
    @Test
    public void testConstructor() throws WrongSizeException, 
            InvalidPinholeCameraIntrinsicParametersException {
        
        PinholeCameraIntrinsicParameters K = 
                new PinholeCameraIntrinsicParameters();
        
        //check that internal matrix is the identity, which corresponds to 
        //canonical intrinsic parameters
        assertTrue(K.getInternalMatrix().equals(Matrix.identity(INTRINSIC_ROWS, 
                INTRINSIC_COLS), ABSOLUTE_ERROR));
        
        
        //Test constructor providing an internal matrix
        
        //Build a random upper triangular matrix
        Matrix triangularMatrix = Matrix.createWithUniformRandomValues(
                INTRINSIC_ROWS, INTRINSIC_COLS, MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        for (int u = 0; u < INTRINSIC_ROWS; u++) {
            for (int v = 0; v < u; v++) {
                triangularMatrix.setElementAt(u, v, 0.0);
            }
        }
        
        //Use transposed upper triangular matrix (which is lower triangular) to
        //build any random non-triangular matrix
        Matrix transTriangularMatrix = triangularMatrix.transposeAndReturnNew();
        
        Matrix KMatrix = transTriangularMatrix.multiplyAndReturnNew(
                triangularMatrix);
        
        //Force InvalidPinholeCameraIntrinsicParametersException when matrix is
        //not upper triangular
        K = null;
        try {
            K = new PinholeCameraIntrinsicParameters(KMatrix);
            fail("InvalidPinholeCameraIntrinsicParametersException expected but not thrown");
        } catch (InvalidPinholeCameraIntrinsicParametersException ignore) { }
        try {
            K = new PinholeCameraIntrinsicParameters(KMatrix, ABSOLUTE_ERROR);
            fail("InvalidPinholeCameraIntrinsicParametersException expected but not thrown");
        } catch (InvalidPinholeCameraIntrinsicParametersException ignore) { }
        //Force IllegalArgumentException
        try {
            K = new PinholeCameraIntrinsicParameters(triangularMatrix, 
                    -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(K);
        
        //Use upper-triangular matrix as KMatrix but set last element to a value
        //different of zero. After calling constructor check that provided 
        //matrix has been normalized and that value is now 1.0
        KMatrix = triangularMatrix;
        triangularMatrix.setElementAt(2, 2, 2.0);
        K = new PinholeCameraIntrinsicParameters(KMatrix);
        
        //check correctness of internal matrix
        assertTrue(K.getInternalMatrix().equals(triangularMatrix, 
                ABSOLUTE_ERROR));
        assertEquals(triangularMatrix.getElementAt(2, 2), 1.0, ABSOLUTE_ERROR);
        
        
        //Test constructor with parameters
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, 
                MAX_FOCAL_LENGTH);
        double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, 
                MAX_FOCAL_LENGTH);
        double aspectRatio = verticalFocalLength / horizontalFocalLength;
        
        double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
        double skewnessAngle = Math.atan2(skewness, verticalFocalLength);
        
        double horizontalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        double verticalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        
        K = new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                verticalFocalLength, horizontalPrincipalPoint, 
                verticalPrincipalPoint, skewness);
        assertEquals(K.getHorizontalFocalLength(), horizontalFocalLength,
                ABSOLUTE_ERROR);
        assertEquals(K.getVerticalFocalLength(), verticalFocalLength,
                ABSOLUTE_ERROR);
        assertEquals(K.getAspectRatio(), aspectRatio, ABSOLUTE_ERROR);
        assertEquals(K.getHorizontalPrincipalPoint(), horizontalPrincipalPoint,
                ABSOLUTE_ERROR);
        assertEquals(K.getVerticalPrincipalPoint(), verticalPrincipalPoint,
                ABSOLUTE_ERROR);
        assertEquals(K.getSkewness(), skewness, ABSOLUTE_ERROR);
        assertEquals(K.getSkewnessAngle(), skewnessAngle, ABSOLUTE_ERROR);
        
        Matrix KMatrix2 = new Matrix(INTRINSIC_ROWS, INTRINSIC_COLS);
        KMatrix2.setElementAt(0, 0, horizontalFocalLength);
        KMatrix2.setElementAt(1, 1, verticalFocalLength);
        KMatrix2.setElementAt(2, 2, 1.0);
        KMatrix2.setElementAt(0, 1, skewness);
        KMatrix2.setElementAt(0, 2, horizontalPrincipalPoint);
        KMatrix2.setElementAt(1, 2, verticalPrincipalPoint);
        
        assertTrue(K.getInternalMatrix().equals(KMatrix2, ABSOLUTE_ERROR));        
        
        
        //Test copy constructor
        K = new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                verticalFocalLength, horizontalPrincipalPoint, 
                verticalPrincipalPoint, skewness);
        PinholeCameraIntrinsicParameters K2 = 
                new PinholeCameraIntrinsicParameters(K);
        assertTrue(K.getInternalMatrix().equals(K2.getInternalMatrix(), 
                ABSOLUTE_ERROR));
    }
    
    @Test
    public void testGetSetInternalMatrix() throws WrongSizeException, 
            InvalidPinholeCameraIntrinsicParametersException {
        
        //Build a random upper triangular matrix
        Matrix triangularMatrix = Matrix.createWithUniformRandomValues(
                INTRINSIC_ROWS, INTRINSIC_COLS, MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        for (int u = 0; u < INTRINSIC_ROWS; u++) {
            for (int v = 0; v < u; v++) {
                triangularMatrix.setElementAt(u, v, 0.0);
            }
        }
        
        //Use transposed upper triangular matrix (which is lower triangular) to
        //build any random non-triangular matrix
        Matrix transTriangularMatrix = triangularMatrix.transposeAndReturnNew();
        
        Matrix KMatrix = transTriangularMatrix.multiplyAndReturnNew(
                triangularMatrix);
        
        PinholeCameraIntrinsicParameters K = 
                new PinholeCameraIntrinsicParameters();
        
        //Force InvalidPinholeCameraIntrinsicParametersException when matrix is
        //not upper triangular
        try {
            K.setInternalMatrix(KMatrix);
            fail("InvalidPinholeCameraIntrinsicParametersException expected but not thrown");
        } catch (InvalidPinholeCameraIntrinsicParametersException ignore) { }
        try {
            K.setInternalMatrix(KMatrix, ABSOLUTE_ERROR);
            fail("InvalidPinholeCameraIntrinsicParametersException expected but not thrown");
        } catch (InvalidPinholeCameraIntrinsicParametersException ignore) { }
        //Force IllegalArgumentException
        try {
            K.setInternalMatrix(triangularMatrix, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        
        //Use upper-triangular matrix as KMatrix
        KMatrix = triangularMatrix;
        //Force InvalidPinholeCameraIntrinsicParametersException because 
        //lower-right element is not 1.0 and normalization is disabled
        triangularMatrix.setElementAt(2, 2, 2.0);
        
        K.setInternalMatrix(KMatrix);
        
        //check that now lower-right element of KMatrix is 1.0 (because it has
        //been normalized)
        assertEquals(triangularMatrix.getElementAt(2, 2), 1.0, ABSOLUTE_ERROR);
        
        assertTrue(KMatrix.equals(K.getInternalMatrix(), ABSOLUTE_ERROR));
        
        //try again with a threshold
        K.setInternalMatrix(KMatrix, ABSOLUTE_ERROR);
        assertTrue(KMatrix.equals(K.getInternalMatrix(), ABSOLUTE_ERROR));
    }
    
    @Test
    public void testGetInverseInternalMatrix() throws AlgebraException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, 
                MAX_FOCAL_LENGTH);
        double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, 
                MAX_FOCAL_LENGTH);
        
        double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
        
        double horizontalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        double verticalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        
        PinholeCameraIntrinsicParameters intrinsic = 
                new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                verticalFocalLength, horizontalPrincipalPoint, 
                verticalPrincipalPoint, skewness);

        Matrix K = intrinsic.getInternalMatrix();
        Matrix invK = com.irurueta.algebra.Utils.inverse(K);
        
        Matrix invK2 = intrinsic.getInverseInternalMatrix();
        Matrix invK3 = new Matrix(INTRINSIC_ROWS, INTRINSIC_COLS);
        intrinsic.getInverseInternalMatrix(invK3);
        
        //check correctness
        assertTrue(invK.equals(invK2, ABSOLUTE_ERROR));
        assertTrue(invK.equals(invK3, ABSOLUTE_ERROR));
    }
    
    @Test
    public void testGetSetParameters() throws WrongSizeException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, 
                MAX_FOCAL_LENGTH);
        double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, 
                MAX_FOCAL_LENGTH);
        double aspectRatio = verticalFocalLength / horizontalFocalLength;
        double horizontalFocalLength2 = randomizer.nextDouble(MIN_FOCAL_LENGTH, 
                MAX_FOCAL_LENGTH);
        double verticalFocalLength2 = randomizer.nextDouble(MIN_FOCAL_LENGTH, 
                MAX_FOCAL_LENGTH);
        double aspectRatio2 = verticalFocalLength2 / horizontalFocalLength2;
        
        double aspectRatio3 = randomizer.nextDouble(MIN_ASPECT_RATIO, 
                MAX_ASPECT_RATIO);
        double aspectRatio4 = randomizer.nextDouble(MIN_ASPECT_RATIO,
                MAX_ASPECT_RATIO);
        
        double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
        double skewnessAngle = Math.atan2(skewness, verticalFocalLength);
        double skewness2 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
        double skewnessAngle2 = Math.atan2(skewness2, verticalFocalLength);
        
        double horizontalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        double verticalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        double horizontalPrincipalPoint2 = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        double verticalPrincipalPoint2 = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        
        PinholeCameraIntrinsicParameters K = 
                new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                verticalFocalLength, horizontalPrincipalPoint, 
                verticalPrincipalPoint, skewness);
        
        //Test getting parameters
        assertEquals(K.getHorizontalFocalLength(), horizontalFocalLength, 
                ABSOLUTE_ERROR);
        assertEquals(K.getVerticalFocalLength(), verticalFocalLength,
                ABSOLUTE_ERROR);
        assertEquals(K.getAspectRatio(), aspectRatio, ABSOLUTE_ERROR);
        assertEquals(K.getHorizontalPrincipalPoint(), horizontalPrincipalPoint,
                ABSOLUTE_ERROR);
        assertEquals(K.getVerticalPrincipalPoint(), verticalPrincipalPoint,
                ABSOLUTE_ERROR);
        assertEquals(K.getSkewness(), skewness, ABSOLUTE_ERROR);
        assertEquals(K.getSkewnessAngle(), skewnessAngle, ABSOLUTE_ERROR);
        
        //Simulate internal matrix 
        Matrix KMatrix2 = new Matrix(INTRINSIC_ROWS, INTRINSIC_COLS);
        KMatrix2.setElementAt(0, 0, horizontalFocalLength);
        KMatrix2.setElementAt(1, 1, verticalFocalLength);
        KMatrix2.setElementAt(2, 2, 1.0);
        KMatrix2.setElementAt(0, 1, skewness);
        KMatrix2.setElementAt(0, 2, horizontalPrincipalPoint);
        KMatrix2.setElementAt(1, 2, verticalPrincipalPoint);
        
        Matrix KMatrix = K.getInternalMatrix();
        
        //And compare both matrices
        assertTrue(KMatrix.equals(KMatrix2, ABSOLUTE_ERROR));
        
        
        //set new parameters
        K.setHorizontalFocalLength(horizontalFocalLength2);
        assertEquals(K.getHorizontalFocalLength(), horizontalFocalLength2, 
                ABSOLUTE_ERROR);
        
        K.setVerticalFocalLength(verticalFocalLength2);
        assertEquals(K.getVerticalFocalLength(), verticalFocalLength2,
                ABSOLUTE_ERROR);
        
        assertEquals(K.getAspectRatio(), aspectRatio2, ABSOLUTE_ERROR);
        
        K.setAspectRatioKeepingHorizontalFocalLength(aspectRatio3);
        assertEquals(K.getAspectRatio(), aspectRatio3, ABSOLUTE_ERROR);
        assertEquals(K.getHorizontalFocalLength(), horizontalFocalLength2, 
                ABSOLUTE_ERROR);
        assertEquals(K.getVerticalFocalLength(), 
                aspectRatio3 * horizontalFocalLength2, ABSOLUTE_ERROR);
        
        K.setAspectRatioKeepingVerticalFocalLength(aspectRatio4);
        assertEquals(K.getAspectRatio(), aspectRatio4, ABSOLUTE_ERROR);
        assertEquals(K.getHorizontalFocalLength(), 
                aspectRatio3 * horizontalFocalLength2 / aspectRatio4, 
                ABSOLUTE_ERROR);
        assertEquals(K.getVerticalFocalLength(),
                aspectRatio3 * horizontalFocalLength2, ABSOLUTE_ERROR);
        
        K.setHorizontalPrincipalPoint(horizontalPrincipalPoint2);
        assertEquals(K.getHorizontalPrincipalPoint(),
                horizontalPrincipalPoint2, ABSOLUTE_ERROR);
        
        K.setVerticalPrincipalPoint(verticalPrincipalPoint2);
        assertEquals(K.getVerticalPrincipalPoint(),
                verticalPrincipalPoint2, ABSOLUTE_ERROR);
        
        K.setSkewness(skewness2);
        assertEquals(K.getSkewness(), skewness2, ABSOLUTE_ERROR);
        K.setVerticalFocalLength(verticalFocalLength2);
        K.setSkewnessAngle(skewnessAngle2);
        assertEquals(K.getSkewnessAngle(), skewnessAngle2, ABSOLUTE_ERROR);
    }
    
    @Test
    public void testCreateCanonicalIntrinsicParameters() 
            throws WrongSizeException {
        PinholeCameraIntrinsicParameters K = PinholeCameraIntrinsicParameters.
                createCanonicalIntrinsicParameters();
        
        assertTrue(K.getInternalMatrix().equals(Matrix.identity(INTRINSIC_ROWS, 
                INTRINSIC_COLS), ABSOLUTE_ERROR));
    }
    
    @Test
    public void testCreateTypicalIntrinsicParameters() 
            throws WrongSizeException {
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int imageWidth = randomizer.nextInt(MIN_IMAGE_WIDTH, MAX_IMAGE_WIDTH);
        int imageHeight = randomizer.nextInt(MIN_IMAGE_HEIGHT, MAX_IMAGE_HEIGHT);
        
        PinholeCameraIntrinsicParameters K = PinholeCameraIntrinsicParameters.
                createTypicalIntrinsicParameters(imageWidth, imageHeight);
        
        Matrix KMatrix = K.getInternalMatrix();
        
        Matrix KMatrix2 = new Matrix(INTRINSIC_ROWS, INTRINSIC_COLS);
        KMatrix2.setElementAt(0, 0, (double)(imageWidth + imageHeight) / 2.0);
        KMatrix2.setElementAt(1, 1, (double)(imageWidth + imageHeight) / 2.0);
        KMatrix2.setElementAt(2, 2, 1.0);
        KMatrix2.setElementAt(0, 2, (double)imageWidth / 2.0);
        KMatrix2.setElementAt(1, 2, (double)imageHeight / 2.0);
        
        assertTrue(KMatrix.equals(KMatrix2, ABSOLUTE_ERROR));
        
        assertEquals(K.getHorizontalFocalLength(), 
                (double)(imageWidth + imageHeight) / 2.0, ABSOLUTE_ERROR);
        assertEquals(K.getVerticalFocalLength(), 
                (double)(imageWidth + imageHeight) / 2.0, ABSOLUTE_ERROR);
        assertEquals(K.getSkewness(), 0.0, ABSOLUTE_ERROR);
        assertEquals(K.getHorizontalPrincipalPoint(), (double)imageWidth / 2.0,
                ABSOLUTE_ERROR);
        assertEquals(K.getVerticalPrincipalPoint(), (double)imageHeight / 2.0,
                ABSOLUTE_ERROR);
    }
    
    @Test
    public void testClone() {
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, 
                MAX_FOCAL_LENGTH);
        double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, 
                MAX_FOCAL_LENGTH);
        
        double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
        
        double horizontalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        double verticalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        
        //Create K
        PinholeCameraIntrinsicParameters K = 
                new PinholeCameraIntrinsicParameters(horizontalFocalLength, 
                verticalFocalLength, horizontalPrincipalPoint, 
                verticalPrincipalPoint, skewness);
        
        //Clone K
        PinholeCameraIntrinsicParameters K2 = K.clone();
        
        //check both intrinsic parameters are equal
        assertTrue(K.getInternalMatrix().equals(K2.getInternalMatrix(), 
                ABSOLUTE_ERROR));
        
        //if we change K parameters, K2 will still remain the same
        K.setHorizontalFocalLength(horizontalFocalLength + 1.0);
        K.setVerticalFocalLength(verticalFocalLength + 1.0);
        K.setSkewness(skewness + 1.0);
        K.setHorizontalPrincipalPoint(horizontalPrincipalPoint + 1.0);
        K.setVerticalPrincipalPoint(verticalPrincipalPoint + 1.0);
        assertEquals(K.getHorizontalFocalLength(), horizontalFocalLength + 1.0, 
                ABSOLUTE_ERROR);
        assertEquals(K2.getHorizontalFocalLength(), horizontalFocalLength,
                ABSOLUTE_ERROR);
        assertEquals(K.getVerticalFocalLength(), verticalFocalLength + 1.0, 
                ABSOLUTE_ERROR);
        assertEquals(K2.getVerticalFocalLength(), verticalFocalLength,
                ABSOLUTE_ERROR);
        assertEquals(K.getSkewness(), skewness + 1.0, ABSOLUTE_ERROR);
        assertEquals(K2.getSkewness(), skewness, ABSOLUTE_ERROR);
        assertEquals(K.getHorizontalPrincipalPoint(), 
                horizontalPrincipalPoint + 1.0, ABSOLUTE_ERROR);
        assertEquals(K2.getHorizontalPrincipalPoint(), horizontalPrincipalPoint,
                ABSOLUTE_ERROR);
        assertEquals(K.getVerticalPrincipalPoint(), 
                verticalPrincipalPoint + 1.0, ABSOLUTE_ERROR);
        assertEquals(K2.getVerticalPrincipalPoint(), verticalPrincipalPoint,
                ABSOLUTE_ERROR);        
    }
    
    @Test
    public void testIsValidMatrix() throws WrongSizeException {
        
        //Build a random upper triangular matrix
        Matrix triangularMatrix = Matrix.createWithUniformRandomValues(
                INTRINSIC_ROWS, INTRINSIC_COLS, MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        for (int u = 0; u < INTRINSIC_ROWS; u++) {
            for (int v = 0; v < u; v++) {
                triangularMatrix.setElementAt(u, v, 0.0);
            }
        }
        
        //use transposed upper triangular matrix (which is lower triangular) to
        //build any random non-triangular matrix
        Matrix transTriangularMatrix = triangularMatrix.transposeAndReturnNew();
        
        Matrix KMatrix = transTriangularMatrix.multiplyAndReturnNew(
                triangularMatrix);
        
        assertFalse(PinholeCameraIntrinsicParameters.isValidMatrix(KMatrix));
        assertFalse(PinholeCameraIntrinsicParameters.isValidMatrix(KMatrix,
                ABSOLUTE_ERROR));
        
        //use upper triangular matrix
        KMatrix = triangularMatrix;
        //make matrix invalid because lower-right element is not 10
        KMatrix.setElementAt(2, 2, 2.0);
        
        assertFalse(PinholeCameraIntrinsicParameters.isValidMatrix(KMatrix));
        assertFalse(PinholeCameraIntrinsicParameters.isValidMatrix(KMatrix,
                ABSOLUTE_ERROR));
        
        //we can normalize KMatrix by setting its lower-right element to 1.0
        KMatrix.multiplyByScalar(0.5);
        
        //now KMatrix should be valid
        assertTrue(PinholeCameraIntrinsicParameters.isValidMatrix(KMatrix));
        assertTrue(PinholeCameraIntrinsicParameters.isValidMatrix(KMatrix,
                ABSOLUTE_ERROR));
    }
    
    @Test
    public void testGetDualImageOfAbsoluteConic() 
            throws InvalidPinholeCameraIntrinsicParametersException {
        //create intrinsic parameters
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, 
                MAX_FOCAL_LENGTH);
        double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, 
                MAX_FOCAL_LENGTH);
        
        double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
        
        double horizontalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        double verticalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        
        PinholeCameraIntrinsicParameters intrinsic = 
                new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                verticalFocalLength, horizontalPrincipalPoint, 
                verticalPrincipalPoint, skewness);

        DualImageOfAbsoluteConic diac = intrinsic.getDualImageOfAbsoluteConic();
        
        PinholeCameraIntrinsicParameters intrinsic2 = 
                diac.getIntrinsicParameters();
        
        assertEquals(intrinsic.getHorizontalFocalLength(), 
                intrinsic2.getHorizontalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getVerticalFocalLength(),
                intrinsic2.getVerticalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getHorizontalPrincipalPoint(),
                intrinsic2.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getVerticalPrincipalPoint(),
                intrinsic2.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getSkewness(), intrinsic2.getSkewness(),
                ABSOLUTE_ERROR);        
    }
    
    @Test
    public void testGetImageOfAbsoluteConic() 
            throws InvalidPinholeCameraIntrinsicParametersException {
        //create intrinsic parameters
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, 
                MAX_FOCAL_LENGTH);
        double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, 
                MAX_FOCAL_LENGTH);
        
        double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
        
        double horizontalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        double verticalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        
        PinholeCameraIntrinsicParameters intrinsic = 
                new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                verticalFocalLength, horizontalPrincipalPoint, 
                verticalPrincipalPoint, skewness);

        ImageOfAbsoluteConic iac = intrinsic.getImageOfAbsoluteConic();
        
        PinholeCameraIntrinsicParameters intrinsic2 = 
                iac.getIntrinsicParameters();
        
        assertEquals(intrinsic.getHorizontalFocalLength(), 
                intrinsic2.getHorizontalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getVerticalFocalLength(),
                intrinsic2.getVerticalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getHorizontalPrincipalPoint(),
                intrinsic2.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getVerticalPrincipalPoint(),
                intrinsic2.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getSkewness(), intrinsic2.getSkewness(),
                ABSOLUTE_ERROR);        
    }    
    
    @Test
    public void testCreate() {
        //For nexus 5
        double focalLength = 3.97; //mm
        double sensorWidth = 4.6032; //mm
        double sensorHeight = 3.5168; //mm
        int imageWidth = 2048; //px
        int imageHeight = 1536; //px
        
        PinholeCameraIntrinsicParameters intrinsic =
                PinholeCameraIntrinsicParameters.create(focalLength, 
                        sensorWidth, sensorHeight, imageWidth, imageHeight);
        
        //check
        assertEquals(intrinsic.getHorizontalFocalLength(), 
                focalLength * imageWidth / sensorWidth, ABSOLUTE_ERROR);
        assertEquals(intrinsic.getVerticalFocalLength(),
                focalLength * imageHeight / sensorHeight, ABSOLUTE_ERROR);
        assertEquals(intrinsic.getSkewness(), 0.0, 0.0);
        assertEquals(intrinsic.getHorizontalPrincipalPoint(), 0.0, 
                ABSOLUTE_ERROR);
        assertEquals(intrinsic.getVerticalPrincipalPoint(), 0.0,
                ABSOLUTE_ERROR);
    }
}
