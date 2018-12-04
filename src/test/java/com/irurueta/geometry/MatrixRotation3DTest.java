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

import com.irurueta.algebra.*;
import com.irurueta.algebra.NotAvailableException;
import com.irurueta.algebra.Utils;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.Arrays;
import java.util.Random;

import static org.junit.Assert.*;

public class MatrixRotation3DTest {
    
    private static final int ROTATION_ROWS = 3;
    private static final int ROTATION_COLS = 3;
    private static final int INHOM_COORDS = 3;
    
    private static final int HOM_ROTATION_ROWS = 4;
    private static final int HOM_ROTATION_COLS = 4;
    
    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;
    
    private static final double MIN_RANDOM_SCALE = 2.0;
    private static final double MAX_RANDOM_SCALE = 3.0;
    
    private static final double MIN_ANGLE_DEGREES = -90.0;
    private static final double MAX_ANGLE_DEGREES = 90.0;
    
    private static final double ABSOLUTE_ERROR = 1e-6;
    
    private static final int TIMES = 100;
    
    public MatrixRotation3DTest() { }
    
    @BeforeClass
    public static void setUpClass() { }
    
    @AfterClass
    public static void tearDownClass() { }
    
    @Before
    public void setUp() { }
    
    @After
    public void tearDown() { }
    
    @Test
    public void testConstructors() throws WrongSizeException, NotReadyException, 
            LockedException, DecomposerException, NotAvailableException,
            InvalidRotationMatrixException, RotationException {
        
        for (int t = 0; t < TIMES; t++) {
        
            //Test empty constructor
            MatrixRotation3D rotation = new MatrixRotation3D();
            assertNotNull(rotation);
        
            Matrix internalMatrix = rotation.getInternalMatrix();
        
            assertEquals(internalMatrix.getRows(), ROTATION_ROWS);
            assertEquals(internalMatrix.getColumns(), ROTATION_COLS);
        
            for (int v = 0; v < ROTATION_COLS; v++) {
                for (int u = 0; u < ROTATION_ROWS; u++) {
                    if (u == v) {
                        assertEquals(internalMatrix.getElementAt(u, v), 1.0, 
                                ABSOLUTE_ERROR);
                    } else {
                        assertEquals(internalMatrix.getElementAt(u, v), 0.0,
                                ABSOLUTE_ERROR);
                    }
                }
            }
        
            //Test constructor with any random orthonormal matrix
            Matrix a = Matrix.createWithUniformRandomValues(ROTATION_ROWS, 
                    ROTATION_COLS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            SingularValueDecomposer decomposer = new SingularValueDecomposer(a);
        
            decomposer.decompose();
        
            Matrix rotationMatrix = decomposer.getV();
            
            rotation = new MatrixRotation3D(rotationMatrix, ABSOLUTE_ERROR);
            assertEquals(rotation.getInternalMatrix(), rotationMatrix);
        
            //Attempt Force InvalidRotationMatrixException
            //(when threshold is too small, the following might fail)
            rotation = null;
            try {
                rotation = new MatrixRotation3D(rotationMatrix, 0.0);
                assertNotNull(rotation);
            } catch (InvalidRotationMatrixException e) {
                //noinspection all
                assertNull(rotation);
            }
        
            //or when matrix is not orthogonal
            Matrix transRotationMatrix = rotationMatrix.transposeAndReturnNew();
            
            double[] diagonal = new double[ROTATION_ROWS];
            Arrays.fill(diagonal, 1.0);
            diagonal[2] = 0.0; //make last singular value equal to zero, so that
                                //matrix is singular
        
            Matrix diagMatrix = Matrix.diagonal(diagonal);
            
            Matrix rotationMatrix2 = transRotationMatrix.multiplyAndReturnNew(
                    diagMatrix.multiplyAndReturnNew(rotationMatrix));
            
            rotation = null;
            try {
                rotation = new MatrixRotation3D(rotationMatrix2, 
                        ABSOLUTE_ERROR);
                fail("InvalidRotationMatrixException expected but not thrown");
            } catch (InvalidRotationMatrixException ignore) { }
            assertNull(rotation);
        
        
        
            //Test copy constructor
            rotation = new MatrixRotation3D(rotationMatrix, ABSOLUTE_ERROR);        
            MatrixRotation3D rotation2 = new MatrixRotation3D(rotation);
            assertEquals(rotation2.getInternalMatrix(), 
                    rotation.getInternalMatrix());
        
        
            //test constructor using euler angles
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double alphaEuler = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 
                    2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 
                    2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            //theta will be used along with axis
            double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;        
        
            rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);
            //alpha ambiguity: can be alpha or alpha - pi
            boolean valid = false;
            if (Math.abs(rotation.getAlphaEulerAngle() - alphaEuler) <
                    ABSOLUTE_ERROR) {
                valid = true;
            } else if (Math.abs(rotation.getAlphaEulerAngle() - (alphaEuler -
                    Math.PI)) < ABSOLUTE_ERROR) {
                valid = true;
            }
            assertTrue(valid);
            //bet ambiguity: can be beta or -beta
            valid = false;
            if (Math.abs(rotation.getBetaEulerAngle() - betaEuler) <
                    ABSOLUTE_ERROR) {
                valid = true;
            } else if (Math.abs(rotation.getBetaEulerAngle() + betaEuler) <
                    ABSOLUTE_ERROR) {
                valid = true;
            }
            assertTrue(valid);
            //gamma ambiguity: gamma can be gamma or gamma - pi
            valid = false;
            if (Math.abs(rotation.getGammaEulerAngle() - gammaEuler) <
                    ABSOLUTE_ERROR) {
                valid = true;
            } else if (Math.abs(rotation.getGammaEulerAngle() -
                    (gammaEuler - Math.PI)) < ABSOLUTE_ERROR) {
                valid = true;
            }
            assertTrue(valid);
        
            //check internal matrix
            rotationMatrix = rotation.getInternalMatrix();
        
            rotationMatrix2 = new Matrix(ROTATION_ROWS, ROTATION_COLS);
            rotationMatrix2.setElementAt(0, 0, Math.cos(alphaEuler) * 
                    Math.cos(gammaEuler) - Math.sin(alphaEuler) * Math.sin(
                    betaEuler) * Math.sin(gammaEuler));
            rotationMatrix2.setElementAt(1, 0, Math.cos(alphaEuler) * Math.sin(
                    gammaEuler) + Math.sin(alphaEuler) * Math.sin(betaEuler) *
                    Math.cos(gammaEuler));
            rotationMatrix2.setElementAt(2, 0, -Math.sin(alphaEuler) * Math.cos(
                    betaEuler));
        
            rotationMatrix2.setElementAt(0, 1, -Math.cos(betaEuler) * Math.sin(
                    gammaEuler));
            rotationMatrix2.setElementAt(1, 1, Math.cos(betaEuler) * Math.cos(
                    gammaEuler));
            rotationMatrix2.setElementAt(2, 1, Math.sin(betaEuler));
        
            rotationMatrix2.setElementAt(0, 2, Math.sin(alphaEuler) * Math.cos(
                    gammaEuler) + Math.cos(alphaEuler) * Math.sin(betaEuler) * 
                    Math.sin(gammaEuler));
            rotationMatrix2.setElementAt(1, 2, Math.sin(alphaEuler) * Math.sin(
                    gammaEuler) - Math.cos(alphaEuler) * Math.sin(betaEuler) *
                    Math.cos(gammaEuler));
            rotationMatrix2.setElementAt(2, 2, Math.cos(alphaEuler) * Math.cos(
                    betaEuler));
        
            //check that rotation matrices are equal (up to a certain error)
            assertTrue(rotationMatrix.equals(rotationMatrix2, ABSOLUTE_ERROR));
        
            
            //test constructor using axis and angle
            
            //Force WrongSizeException
            double[] axis = new double[INHOM_COORDS + 1];
            
            rotation = null;
            try {
                rotation = new MatrixRotation3D(axis, theta);
                fail("IllegalArgumentException expected but not thrown");
            } catch (IllegalArgumentException ignore) { }
            assertNull(rotation);
            
            //Find any 3 orthogonal vectors, 1st will be axis of rotation, and
            //the remaining two will lie on the rotation plane and will be used
            //to test theta angle
            
            //To find 3 orthogonal vectors, we use V matrix of a singular
            //decomposition of any Nx3 matrix
            a = Matrix.createWithUniformRandomValues(1, ROTATION_COLS, 
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            decomposer = new SingularValueDecomposer(a);
            
            decomposer.decompose();
            
            Matrix vMatrix = decomposer.getV();
            
            //axis of rotation
            double [] vAxis = vMatrix.getSubmatrixAsArray(0, 0,
                    2, 0);
            Matrix mAxis = Matrix.newFromArray(vAxis, true);
            
            //inhomogeneous coordinates of point laying on rotation plane
            double [] vPoint = vMatrix.getSubmatrixAsArray(0, 1,
                    2, 1);
            
            rotation = new MatrixRotation3D(vAxis, theta);
            
            //To test correctness of rotation, axis should remain equal
            rotationMatrix = rotation.getInternalMatrix();
            
            Matrix mAxis2 = rotationMatrix.multiplyAndReturnNew(mAxis);
            
            assertEquals(mAxis.getElementAtIndex(0), 
                    mAxis2.getElementAtIndex(0), ABSOLUTE_ERROR);
            assertEquals(mAxis.getElementAtIndex(1), 
                    mAxis2.getElementAtIndex(1), ABSOLUTE_ERROR);
            assertEquals(mAxis.getElementAtIndex(2), 
                    mAxis2.getElementAtIndex(2), ABSOLUTE_ERROR);
            
            axis = rotation.getRotationAxis();
            
            double scaleX = axis[0] / vAxis[0];
            double scaleY = axis[0] / vAxis[0];
            double scaleZ = axis[0] / vAxis[0];
            
            assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
            assertEquals(scaleY, scaleZ, ABSOLUTE_ERROR);
            assertEquals(scaleZ, scaleX, ABSOLUTE_ERROR);
            
            //and point on rotated plance should be rotated exactly theta 
            //radians
            Matrix mPoint = Matrix.newFromArray(vPoint);
            //below: rotated point
            Matrix mPoint2 = rotationMatrix.multiplyAndReturnNew(mPoint);
            
            //because vPoint is already normalized (from SVD decomposition)
            //we only need to normalize rotated point to compute the rotation
            //angle as the arcosine of their dot product
            double norm2 = Utils.normF(mPoint2);
            mPoint2.multiplyByScalar(1.0 / norm2);
            
            double dotProduct = mPoint.transposeAndReturnNew().
                    multiplyAndReturnNew(mPoint2).getElementAtIndex(0);
            
            double theta2 = Math.acos(dotProduct);
            double theta2b = rotation.getRotationAngle();
            
            //check correctness of angles (up to sign for this test)
            assertEquals(Math.abs(theta), Math.abs(theta2), ABSOLUTE_ERROR);
            
            //check correctness of angles (including sign) for method in class
            if (scaleX > 0.0) {
                assertEquals(theta, theta2b, ABSOLUTE_ERROR);
            } else {
                assertEquals(theta, -theta2b, ABSOLUTE_ERROR);
            }
        }
    }
    
    @Test
    public void testGetSetInternalMatrix() throws WrongSizeException, 
            NotReadyException, LockedException, DecomposerException,
            NotAvailableException, InvalidRotationMatrixException {
        
        //Create identity rotation
        MatrixRotation3D rotation = new MatrixRotation3D();
        
        //Obtain internal matrix
        Matrix internalMatrix = rotation.getInternalMatrix();
        
        //Check internal matrix is the identity
        assertEquals(internalMatrix.getRows(), ROTATION_ROWS);
        assertEquals(internalMatrix.getColumns(), ROTATION_COLS);
        assertTrue(internalMatrix.equals(Matrix.identity(ROTATION_ROWS, 
                ROTATION_COLS), ABSOLUTE_ERROR));
        
        //Set any random orthonormal matrix
        Matrix a = Matrix.createWithUniformRandomValues(ROTATION_ROWS, 
                ROTATION_COLS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        SingularValueDecomposer decomposer = new SingularValueDecomposer(a);
        
        decomposer.decompose();
        
        //Use V matrix from SVD to obtain a random orthonormal matrix
        Matrix rotationMatrix = decomposer.getV();
        
        rotation.setInternalMatrix(rotationMatrix, ABSOLUTE_ERROR);
        assertTrue(rotationMatrix.equals(rotation.getInternalMatrix(), 
                ABSOLUTE_ERROR));
        
        //Force InvalidRotationMatrixException
        
        //when threshold is too small
        try {
            rotation.setInternalMatrix(rotationMatrix, 0.0);
            fail("InvalidRotationMatrixException expected but not thrown");
        } catch (InvalidRotationMatrixException ignore) { }
        //but still rotation matrix remains from previous valid assignment
        assertTrue(rotationMatrix.equals(rotation.getInternalMatrix(), 
                ABSOLUTE_ERROR));
        
        //or when matrix is not orthogonal
        Matrix transRotationMatrix = rotationMatrix.transposeAndReturnNew();
        
        double[] diag = new double[ROTATION_ROWS];
        Arrays.fill(diag, 1.0);
        diag[2] = 0.0;
        
        Matrix diagMatrix = Matrix.diagonal(diag);
        
        Matrix rotationMatrix2 = transRotationMatrix.multiplyAndReturnNew(
                diagMatrix.multiplyAndReturnNew(rotationMatrix));
        
        try {
            rotation.setInternalMatrix(rotationMatrix2, ABSOLUTE_ERROR);
            fail("InvalidRotationMatrixException expected but not thrown");
        } catch (InvalidRotationMatrixException ignore) { }
        //but still rotation matrix remains from previous valid assignment
        assertTrue(rotationMatrix.equals(rotation.getInternalMatrix(), 
                ABSOLUTE_ERROR));
    }        
    
    @Test
    public void testGetSetEulerAngles() throws WrongSizeException {
        
        //randomizer to randomly compute euler angles
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double alphaEuler = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double gammaEuler = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        boolean valid = false;
        
        MatrixRotation3D rotation = new MatrixRotation3D();
        
        //set euler angles
        rotation.setEulerAngles(alphaEuler, betaEuler, gammaEuler);
        
        //retrieve euler angles to check correctness
        if (Math.abs(rotation.getAlphaEulerAngle() - alphaEuler) <
                ABSOLUTE_ERROR) {
            valid = true;
        } else if (Math.abs(rotation.getAlphaEulerAngle() - (alphaEuler -
                Math.PI)) < ABSOLUTE_ERROR) {
            valid = true;
        }
        assertTrue(valid);
        //bet ambiguity: can be beta or -beta
        valid = false;
        if (Math.abs(rotation.getBetaEulerAngle() - betaEuler) <
                ABSOLUTE_ERROR) {
            valid = true;
        } else if (Math.abs(rotation.getBetaEulerAngle() + betaEuler) <
                ABSOLUTE_ERROR) {
            valid = true;
        }
        assertTrue(valid);
        //gamma ambiguity: gamma can be gamma or gamma - pi
        valid = false;
        if (Math.abs(rotation.getGammaEulerAngle() - gammaEuler) <
                ABSOLUTE_ERROR) {
            valid = true;
        } else if (Math.abs(rotation.getGammaEulerAngle() -
                (gammaEuler - Math.PI)) < ABSOLUTE_ERROR) {
            valid = true;
        }
        assertTrue(valid);

        //check internal matrix
        Matrix rotationMatrix = rotation.getInternalMatrix();
        
        Matrix rotationMatrix2 = new Matrix(ROTATION_ROWS, ROTATION_COLS);
        rotationMatrix2.setElementAt(0, 0, Math.cos(alphaEuler) * 
                Math.cos(gammaEuler) - Math.sin(alphaEuler) * Math.sin(
                betaEuler) * Math.sin(gammaEuler));
        rotationMatrix2.setElementAt(1, 0, Math.cos(alphaEuler) * Math.sin(
                gammaEuler) + Math.sin(alphaEuler) * Math.sin(betaEuler) *
                Math.cos(gammaEuler));
        rotationMatrix2.setElementAt(2, 0, -Math.sin(alphaEuler) * Math.cos(
                betaEuler));
        
        rotationMatrix2.setElementAt(0, 1, -Math.cos(betaEuler) * Math.sin(
                gammaEuler));
        rotationMatrix2.setElementAt(1, 1, Math.cos(betaEuler) * Math.cos(
                gammaEuler));
        rotationMatrix2.setElementAt(2, 1, Math.sin(betaEuler));
        
        rotationMatrix2.setElementAt(0, 2, Math.sin(alphaEuler) * Math.cos(
                gammaEuler) + Math.cos(alphaEuler) * Math.sin(betaEuler) * 
                Math.sin(gammaEuler));
        rotationMatrix2.setElementAt(1, 2, Math.sin(alphaEuler) * Math.sin(
                gammaEuler) - Math.cos(alphaEuler) * Math.sin(betaEuler) *
                Math.cos(gammaEuler));
        rotationMatrix2.setElementAt(2, 2, Math.cos(alphaEuler) * Math.cos(
                betaEuler));
        
        //check that rotation matrices are equal (up to a certain error)
        assertTrue(rotationMatrix.equals(rotationMatrix2, ABSOLUTE_ERROR));
    }
    
    @Test
    public void testGetSetAxisAndRotation() throws WrongSizeException, 
            NotReadyException, LockedException, DecomposerException,
            NotAvailableException, RotationException {
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        
        MatrixRotation3D rotation = new MatrixRotation3D();
        
        //Force IllegalArgumentException
        double[] axis = new double[INHOM_COORDS + 1];
        
        try {
            rotation.setAxisAndRotation(axis, theta);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        
        //Find any 3 orthogonal vectors, 1st will be axis of rotation, and the
        //remaining two will lie on the rotation plane and will be used to test
        //theta angle
        
        //To find 3 orthogonal vectors, we use V matrix of a singular value
        //decomposition of any Nx3 matrix
        Matrix a = Matrix.createWithUniformRandomValues(1, ROTATION_COLS, 
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        SingularValueDecomposer decomposer = new SingularValueDecomposer(a);
        
        decomposer.decompose();
        
        Matrix v = decomposer.getV();
        
        //axis of rotation
        Matrix axisMatrix = v.getSubmatrix(0, 0,
                2, 0);
        
        //inhomogeneous coordinates of point laying on rotation plane
        Matrix pointMatrix = v.getSubmatrix(0, 1,
                2, 1);
        
        axis = axisMatrix.toArray();
        rotation.setAxisAndRotation(axis, theta);
        
        //To test correctness of rotation, axis should remain equal
        Matrix rotationMatrix = rotation.getInternalMatrix();
        
        Matrix axisMatrix2 = rotationMatrix.multiplyAndReturnNew(axisMatrix);
        
        assertTrue(axisMatrix.equals(axisMatrix2, ABSOLUTE_ERROR));
        
        double[] axis2 = rotation.getRotationAxis();
        
        double scaleX = axis[0] / axis2[0];
        double scaleY = axis[0] / axis2[0];
        double scaleZ = axis[0] / axis2[0];
            
        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleZ, ABSOLUTE_ERROR);
        assertEquals(scaleZ, scaleX, ABSOLUTE_ERROR);
            
        //and point on rotated plance should be rotated exactly theta 
        //radians
        Matrix pointMatrix2 = rotationMatrix.multiplyAndReturnNew(pointMatrix);
            
        //because vPoint is already normalized (from SVD decomposition)
        //we only need to normalize rotated point to compute the rotation
        //angle as the arcosine of their dot product
        double norm2 = Utils.normF(pointMatrix2);
        pointMatrix2.multiplyByScalar(1.0 / norm2);
            
        double dotProduct = pointMatrix.transposeAndReturnNew().
                multiplyAndReturnNew(pointMatrix2).getElementAtIndex(0);
            
        double theta2 = Math.acos(dotProduct);
        double theta2b = rotation.getRotationAngle();
            
        //check correctness of angles (up to sign for this test)
        assertEquals(Math.abs(theta), Math.abs(theta2), ABSOLUTE_ERROR);
            
        //check correctness of angles (including sign) for method in class
        if (scaleX > 0.0) {
            assertEquals(theta, theta2b, ABSOLUTE_ERROR);
        } else {
            assertEquals(theta, -theta2b, ABSOLUTE_ERROR);
        }                
    }
    
    @Test
    public void testIsValidRotationMatrix() 
            throws WrongSizeException, NotReadyException, LockedException, 
            DecomposerException, NotAvailableException {
        
        Matrix a = Matrix.createWithUniformRandomValues(ROTATION_ROWS, 
                ROTATION_COLS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        //Use SVD to obtain an orthonormal matrix from V matrix
        SingularValueDecomposer decomposer = new SingularValueDecomposer(a);
        
        decomposer.decompose();
        
        Matrix rotationMatrix = decomposer.getV();
        
        //rotation matrix shouldn't be valid for tiny thresholds
        assertFalse(MatrixRotation3D.isValidRotationMatrix(rotationMatrix, 
                0.0));
        
        //for reasonable threhsolds, matrix should be valid because it is
        //orthonormal with some imprecisions due to machine precision
        assertTrue(MatrixRotation3D.isValidRotationMatrix(rotationMatrix, 
                ABSOLUTE_ERROR));
        assertTrue(MatrixRotation3D.isValidRotationMatrix(rotationMatrix));
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double scale = randomizer.nextDouble(MIN_RANDOM_SCALE, 
                MAX_RANDOM_SCALE);
        
        //change scale of matrix to make it orthogonal instead of orthonormal
        rotationMatrix.multiplyByScalar(scale);
        
        //now matrix shouldn't be valid even for reasonable thresholds
        assertFalse(MatrixRotation3D.isValidRotationMatrix(rotationMatrix, 
                ABSOLUTE_ERROR));
        assertFalse(MatrixRotation3D.isValidRotationMatrix(rotationMatrix));
    }
    
    @Test
    public void testInverseRotation() throws WrongSizeException, 
            NotReadyException, LockedException, DecomposerException,
            NotAvailableException, InvalidRotationMatrixException {
        
        //Create random rotation
        Matrix a = Matrix.createWithUniformRandomValues(ROTATION_ROWS, 
                ROTATION_COLS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        //Use SVD to obtain an orthonormal matrix from V matrix
        SingularValueDecomposer decomposer = new SingularValueDecomposer(a);
        
        decomposer.decompose();
        
        Matrix rotationMatrix = decomposer.getV();

        MatrixRotation3D rotation = new MatrixRotation3D(rotationMatrix, 
                ABSOLUTE_ERROR);
        
        //compute inverse rotation
        MatrixRotation3D invRotation = rotation.inverseRotationAndReturnNew();
        MatrixRotation3D invRotation2 = new MatrixRotation3D();
        rotation.inverseRotation(invRotation2);
        
        //check correctness
        Matrix invRotMatrix = invRotation.getInternalMatrix();
        Matrix invRotMatrix2 = invRotation2.getInternalMatrix();
        
        Matrix identity = Matrix.identity(ROTATION_ROWS, ROTATION_COLS);
        
        assertTrue(rotationMatrix.multiplyAndReturnNew(invRotMatrix).equals(
                identity, ABSOLUTE_ERROR));
        assertTrue(rotationMatrix.multiplyAndReturnNew(invRotMatrix2).equals(
                identity, ABSOLUTE_ERROR));
        
        //we can also inverse the original rotation
        rotation.inverseRotation();
        assertTrue(invRotMatrix.equals(rotation.getInternalMatrix(), 
                ABSOLUTE_ERROR));
        
    }
    
    @Test
    public void testAsInhomogeneousMatrix() throws WrongSizeException, 
            NotReadyException, LockedException, DecomposerException,
            NotAvailableException, InvalidRotationMatrixException {

        //Create random rotation
        Matrix a = Matrix.createWithUniformRandomValues(ROTATION_ROWS, 
                ROTATION_COLS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        //Use SVD to obtain an orthonormal matrix from V matrix
        SingularValueDecomposer decomposer = new SingularValueDecomposer(a);
        
        decomposer.decompose();
        
        Matrix rotationMatrix = decomposer.getV();

        MatrixRotation3D rotation = new MatrixRotation3D(rotationMatrix, 
                ABSOLUTE_ERROR);
        
        assertEquals(rotationMatrix, rotation.asInhomogeneousMatrix());
        
        Matrix rotationMatrix2 = new Matrix(ROTATION_ROWS, ROTATION_COLS);
        rotation.asInhomogeneousMatrix(rotationMatrix2);
        assertEquals(rotationMatrix, rotationMatrix2);
        
        //Force IllegalArgumentException
        try {
            rotation.asInhomogeneousMatrix(new Matrix(HOM_ROTATION_ROWS,
                    HOM_ROTATION_COLS));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testAsHomogeneousMatrix() throws WrongSizeException, 
            NotReadyException, LockedException, DecomposerException,
            NotAvailableException, InvalidRotationMatrixException {
        
        //Create random rotation
        Matrix a = Matrix.createWithUniformRandomValues(ROTATION_ROWS, 
                ROTATION_COLS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        //Use SVD to obtain an orthonormal matrix from V matrix
        SingularValueDecomposer decomposer = new SingularValueDecomposer(a);
        
        decomposer.decompose();
        
        Matrix rotationMatrix = decomposer.getV();

        Matrix homRotationMatrix = Matrix.identity(HOM_ROTATION_ROWS, 
                HOM_ROTATION_COLS);
        //set top-left 3x3 submatrix
        homRotationMatrix.setSubmatrix(0, 0,
                2, 2, rotationMatrix);
        
        
        MatrixRotation3D rotation = new MatrixRotation3D(homRotationMatrix, 
                ABSOLUTE_ERROR);
        
        assertEquals(rotationMatrix, rotation.asInhomogeneousMatrix());
        assertEquals(homRotationMatrix, rotation.asHomogeneousMatrix());
        
        Matrix rotationMatrix2 = 
                new Matrix(HOM_ROTATION_ROWS, HOM_ROTATION_COLS);
        rotation.asHomogeneousMatrix(rotationMatrix2);
        assertEquals(homRotationMatrix, rotationMatrix2);
        
        //Force IllegalArgumentException
        try {
            rotation.asHomogeneousMatrix(new Matrix(ROTATION_ROWS,
                    ROTATION_COLS));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }

    }
    
    @Test
    public void testFromMatrix() throws WrongSizeException, NotReadyException, 
            LockedException, DecomposerException, NotAvailableException,
            InvalidRotationMatrixException {
        
        //Create random rotation
        Matrix a = Matrix.createWithUniformRandomValues(ROTATION_ROWS, 
                ROTATION_COLS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        //Use SVD to obtain an orthonormal matrix from V matrix
        SingularValueDecomposer decomposer = new SingularValueDecomposer(a);
        
        decomposer.decompose();
        
        Matrix rotationMatrix = decomposer.getV();

        Matrix homRotationMatrix = Matrix.identity(HOM_ROTATION_ROWS, 
                HOM_ROTATION_COLS);
        //set top-left 3x3 submatrix
        homRotationMatrix.setSubmatrix(0, 0,
                2, 2, rotationMatrix);
        
        
        MatrixRotation3D rotation = new MatrixRotation3D();
        //test with inhomogeneous matrix
        rotation.fromMatrix(rotationMatrix);
        //Check correctness
        assertEquals(rotationMatrix, rotation.asInhomogeneousMatrix());
        assertEquals(rotationMatrix, rotation.getInternalMatrix());
        assertEquals(homRotationMatrix, rotation.asHomogeneousMatrix());
        
        rotation.fromMatrix(rotationMatrix, ABSOLUTE_ERROR);
        //Check correctness
        assertEquals(rotationMatrix, rotation.asInhomogeneousMatrix());
        assertEquals(rotationMatrix, rotation.getInternalMatrix());
        assertEquals(homRotationMatrix, rotation.asHomogeneousMatrix());
        
        
        //test with homogeneous matrix
        rotation.fromMatrix(homRotationMatrix);
        //check correctness
        assertEquals(rotationMatrix, rotation.asInhomogeneousMatrix());
        assertEquals(rotationMatrix, rotation.getInternalMatrix());
        assertEquals(homRotationMatrix, rotation.asHomogeneousMatrix());                

        rotation.fromMatrix(homRotationMatrix, ABSOLUTE_ERROR);
        //check correctness
        assertEquals(rotationMatrix, rotation.asInhomogeneousMatrix());
        assertEquals(rotationMatrix, rotation.getInternalMatrix());
        assertEquals(homRotationMatrix, rotation.asHomogeneousMatrix());                
        
        
        //Force InvalidRotationMatrixException (using a tiny threshold)
        try {
            rotation.fromMatrix(rotationMatrix, 0.0);
            fail("InvalidRotationMatrixException expected but not thrown");
        } catch (InvalidRotationMatrixException ignore) { }
        
        //or using a non orthonormal matrix
        homRotationMatrix.setElementAt(3, 3, 0.0); //makes matrix singular
        try {
            rotation.fromMatrix(homRotationMatrix, ABSOLUTE_ERROR);
            fail("InvalidRotationMatrixException expected but not thrown");
        } catch (InvalidRotationMatrixException ignore) { }
    }
    
    @Test
    public void testFromInhomogeneousMatrix() throws WrongSizeException, 
            NotReadyException, LockedException, DecomposerException,
            NotAvailableException, InvalidRotationMatrixException {
        
        //Create random rotation
        Matrix a = Matrix.createWithUniformRandomValues(ROTATION_ROWS, 
                ROTATION_COLS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        //Use SVD to obtain an orthonormal matrix from V matrix
        SingularValueDecomposer decomposer = new SingularValueDecomposer(a);
        
        decomposer.decompose();
        
        Matrix rotationMatrix = decomposer.getV();        
        
        MatrixRotation3D rotation = new MatrixRotation3D();
        //test with inhomogeneous matrix
        rotation.fromInhomogeneousMatrix(rotationMatrix);
        //Check correctness
        assertEquals(rotationMatrix, rotation.asInhomogeneousMatrix());
        assertEquals(rotationMatrix, rotation.getInternalMatrix());
        
        rotation.fromInhomogeneousMatrix(rotationMatrix, ABSOLUTE_ERROR);
        //Check correctness
        assertEquals(rotationMatrix, rotation.asInhomogeneousMatrix());
        assertEquals(rotationMatrix, rotation.getInternalMatrix());                        
        
        //Force InvalidRotationMatrixException (using a tiny threshold)
        try {
            rotation.fromInhomogeneousMatrix(rotationMatrix, 0.0);
            fail("InvalidRotationMatrixException expected but not thrown");
        } catch (InvalidRotationMatrixException ignore) { }
        
        //or using a non orthonormal matrix
        //because matrix is orthonormal, it's enough to scale it to some value
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double scale = randomizer.nextDouble(MIN_RANDOM_SCALE, 
                MAX_RANDOM_SCALE);
        rotationMatrix.multiplyByScalar(scale);
        try {
            rotation.fromInhomogeneousMatrix(rotationMatrix, ABSOLUTE_ERROR);
            fail("InvalidRotationMatrixException expected but not thrown");
        } catch (InvalidRotationMatrixException ignore) { }
    }
    
    @Test
    public void testFromHomogeneousMatrix() throws WrongSizeException, 
            NotReadyException, LockedException, DecomposerException,
            NotAvailableException, InvalidRotationMatrixException {
        
        //Create random rotation
        Matrix a = Matrix.createWithUniformRandomValues(ROTATION_ROWS, 
                ROTATION_COLS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        //Use SVD to obtain an orthonormal matrix from V matrix
        SingularValueDecomposer decomposer = new SingularValueDecomposer(a);
        
        decomposer.decompose();
        
        Matrix rotationMatrix = decomposer.getV();

        Matrix homRotationMatrix = Matrix.identity(HOM_ROTATION_ROWS, 
                HOM_ROTATION_COLS);
        //set top-left 3x3 submatrix
        homRotationMatrix.setSubmatrix(0, 0,
                2, 2, rotationMatrix);
        
        
        MatrixRotation3D rotation = new MatrixRotation3D();
        //test with homogeneous matrix
        rotation.fromHomogeneousMatrix(homRotationMatrix);
        //check correctness
        assertEquals(rotationMatrix, rotation.asInhomogeneousMatrix());
        assertEquals(rotationMatrix, rotation.getInternalMatrix());
        assertEquals(homRotationMatrix, rotation.asHomogeneousMatrix());                

        rotation.fromHomogeneousMatrix(homRotationMatrix, ABSOLUTE_ERROR);
        //check correctness
        assertEquals(rotationMatrix, rotation.asInhomogeneousMatrix());
        assertEquals(rotationMatrix, rotation.getInternalMatrix());
        assertEquals(homRotationMatrix, rotation.asHomogeneousMatrix());                
        
        
        //Force InvalidRotationMatrixException (using a tiny threshold)
        try {
            rotation.fromHomogeneousMatrix(rotationMatrix, 0.0);
            fail("InvalidRotationMatrixException expected but not thrown");
        } catch (InvalidRotationMatrixException ignore) { }
        
        //or using a non orthonormal matrix
        homRotationMatrix.setElementAt(3, 3, 0.0); //makes matrix singular
        try {
            rotation.fromHomogeneousMatrix(homRotationMatrix, ABSOLUTE_ERROR);
            fail("InvalidRotationMatrixException expected but not thrown");
        } catch (InvalidRotationMatrixException ignore) { }
    }    
    
    @Test
    public void testRotate() throws WrongSizeException, ColinearPointsException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double alpha = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double beta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double gamma = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        
        MatrixRotation3D rotation = new MatrixRotation3D(alpha, beta, gamma);        
        
        //create 3 random points
        HomogeneousPoint3D point1 = new HomogeneousPoint3D();
        point1.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        
        HomogeneousPoint3D point2 = new HomogeneousPoint3D();
        point2.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        
        HomogeneousPoint3D point3 = new HomogeneousPoint3D();
        point3.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        
        //ensure that points are not colinear
        
        while (Plane.areColinearPoints(point1, point2, point3)) {
            point2.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            point3.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));        
        }
        
        //create plane passing through all three points
        Plane plane = new Plane(point1, point2, point3);
        assertTrue(plane.isLocus(point1, ABSOLUTE_ERROR));
        assertTrue(plane.isLocus(point2, ABSOLUTE_ERROR));
        assertTrue(plane.isLocus(point3, ABSOLUTE_ERROR));
        
        //now rotate points and plane
        Point3D rotPoint1A = rotation.rotate(point1);
        Point3D rotPoint1B = Point3D.create();
        rotation.rotate(point1, rotPoint1B);        
        
        Point3D rotPoint2A = rotation.rotate(point2);
        Point3D rotPoint2B = Point3D.create();
        rotation.rotate(point2, rotPoint2B);
        
        Point3D rotPoint3A = rotation.rotate(point3);
        Point3D rotPoint3B = Point3D.create();
        rotation.rotate(point3, rotPoint3B);
        
        
        //check that rotated points A and B are equal
        assertTrue(rotPoint1A.equals(rotPoint1B, ABSOLUTE_ERROR));
        assertTrue(rotPoint2A.equals(rotPoint2B, ABSOLUTE_ERROR));
        assertTrue(rotPoint3A.equals(rotPoint3B, ABSOLUTE_ERROR));
        
        //check points have been correctly rotated   
        Matrix point1Mat = Matrix.newFromArray(point1.asArray(), true);
        Matrix point2Mat = Matrix.newFromArray(point2.asArray(), true);
        Matrix point3Mat = Matrix.newFromArray(point3.asArray(), true);
        Matrix R = rotation.asHomogeneousMatrix();
        Matrix rotPoint1Mat = R.multiplyAndReturnNew(point1Mat);
        Matrix rotPoint2Mat = R.multiplyAndReturnNew(point2Mat);
        Matrix rotPoint3Mat = R.multiplyAndReturnNew(point3Mat);
        
        //check correctness
        double scaleX = rotPoint1A.getHomX() / 
                rotPoint1Mat.getElementAtIndex(0);
        double scaleY = rotPoint1A.getHomY() / 
                rotPoint1Mat.getElementAtIndex(1);
        double scaleZ = rotPoint1A.getHomZ() / 
                rotPoint1Mat.getElementAtIndex(2);
        double scaleW = rotPoint1A.getHomW() / 
                rotPoint1Mat.getElementAtIndex(3);
        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleZ, ABSOLUTE_ERROR);
        assertEquals(scaleZ, scaleW, ABSOLUTE_ERROR);
        assertEquals(scaleW, scaleX, ABSOLUTE_ERROR);

        scaleX = rotPoint2A.getHomX() / rotPoint2Mat.getElementAtIndex(0);
        scaleY = rotPoint2A.getHomY() / rotPoint2Mat.getElementAtIndex(1);
        scaleZ = rotPoint2A.getHomZ() / rotPoint2Mat.getElementAtIndex(2);
        scaleW = rotPoint2A.getHomW() / rotPoint2Mat.getElementAtIndex(3);
        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleZ, ABSOLUTE_ERROR);
        assertEquals(scaleZ, scaleW, ABSOLUTE_ERROR);
        assertEquals(scaleW, scaleX, ABSOLUTE_ERROR);

        scaleX = rotPoint3A.getHomX() / rotPoint3Mat.getElementAtIndex(0);
        scaleY = rotPoint3A.getHomY() / rotPoint3Mat.getElementAtIndex(1);
        scaleZ = rotPoint3A.getHomZ() / rotPoint3Mat.getElementAtIndex(2);
        scaleW = rotPoint3A.getHomW() / rotPoint3Mat.getElementAtIndex(3);
        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleZ, ABSOLUTE_ERROR);
        assertEquals(scaleZ, scaleW, ABSOLUTE_ERROR);
        assertEquals(scaleW, scaleX, ABSOLUTE_ERROR);
        
        //ensure that points where correctly rotated using inhomogeneous 
        //coordinates
        Matrix inhomPoint = new Matrix(INHOM_COORDS, 1);
        inhomPoint.setElementAtIndex(0, point1.getInhomX());
        inhomPoint.setElementAtIndex(1, point1.getInhomY());
        inhomPoint.setElementAtIndex(2, point1.getInhomZ());
        Matrix inhomR = rotation.asInhomogeneousMatrix();
        Matrix inhomRotPoint = inhomR.multiplyAndReturnNew(inhomPoint);
        Point3D rotP = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES,
                inhomRotPoint.toArray());
        assertTrue(rotP.equals(rotPoint1A, ABSOLUTE_ERROR));
        
        
        
        Plane rotPlaneA = rotation.rotate(plane);
        Plane rotPlaneB = new Plane();
        rotation.rotate(plane, rotPlaneB);
        
        //check both rotated lines are equal
        double scaleA = rotPlaneA.getA() / rotPlaneB.getA();
        double scaleB = rotPlaneA.getB() / rotPlaneB.getB();
        double scaleC = rotPlaneA.getC() / rotPlaneB.getC();
        double scaleD = rotPlaneA.getD() / rotPlaneB.getD();
        
        assertEquals(scaleA, scaleB, ABSOLUTE_ERROR);
        assertEquals(scaleB, scaleC, ABSOLUTE_ERROR);
        assertEquals(scaleC, scaleD, ABSOLUTE_ERROR);
        assertEquals(scaleD, scaleA, ABSOLUTE_ERROR);
        
        //check plane has been correctly rotated by ensuring that rotated points
        //belong into rotated line
        assertTrue(rotPlaneA.isLocus(rotPoint1A, ABSOLUTE_ERROR));
        assertTrue(rotPlaneA.isLocus(rotPoint1B, ABSOLUTE_ERROR));
        assertTrue(rotPlaneA.isLocus(rotPoint2A, ABSOLUTE_ERROR));
        assertTrue(rotPlaneA.isLocus(rotPoint2B, ABSOLUTE_ERROR));
        assertTrue(rotPlaneA.isLocus(rotPoint3A, ABSOLUTE_ERROR));
        assertTrue(rotPlaneA.isLocus(rotPoint3B, ABSOLUTE_ERROR));        
        
        //and by ensuring that rotated plane follow appropriate equation
        Matrix planeMat = Matrix.newFromArray(plane.asArray(), true);
        Matrix rotPlaneMat = R.multiplyAndReturnNew(planeMat);
        
        scaleA = rotPlaneA.getA() / rotPlaneMat.getElementAtIndex(0);
        scaleB = rotPlaneA.getB() / rotPlaneMat.getElementAtIndex(1);
        scaleC = rotPlaneA.getC() / rotPlaneMat.getElementAtIndex(2);
        scaleD = rotPlaneA.getD() / rotPlaneMat.getElementAtIndex(3);
        
        assertEquals(scaleA, scaleB, ABSOLUTE_ERROR);
        assertEquals(scaleB, scaleC, ABSOLUTE_ERROR);
        assertEquals(scaleC, scaleD, ABSOLUTE_ERROR);        
        assertEquals(scaleD, scaleA, ABSOLUTE_ERROR);                
    }
    
    
    @Test
    public void testCombine() throws WrongSizeException {
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double alpha1 = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double alpha2 = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double beta1 = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double beta2 = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double gamma1 = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double gamma2 = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        
        MatrixRotation3D rot1 = new MatrixRotation3D(alpha1, beta1, gamma1);
        MatrixRotation3D rot2 = new MatrixRotation3D(alpha2, beta2, gamma2);
        
        MatrixRotation3D rot = new MatrixRotation3D();
        MatrixRotation3D.combine(rot1, rot2, rot);
        
        assertTrue(rot.getInternalMatrix().equals(rot1.getInternalMatrix().
                multiplyAndReturnNew(rot2.getInternalMatrix()), 
                ABSOLUTE_ERROR));
        
        MatrixRotation3D rot3 = rot.combineAndReturnNew(rot2);
        assertTrue(rot3.getInternalMatrix().equals(rot.getInternalMatrix().
                multiplyAndReturnNew(rot2.getInternalMatrix()), 
                ABSOLUTE_ERROR));
        
        Matrix rotationMatrix = rot.getInternalMatrix();
        rot.combine(rot1);
        assertTrue(rot.getInternalMatrix().equals(rotationMatrix.
                multiplyAndReturnNew(rot1.getInternalMatrix()), 
                ABSOLUTE_ERROR));
    }
    
    @Test
    public void testType() {
        MatrixRotation3D rotation = new MatrixRotation3D();
        assertEquals(rotation.getType(), Rotation3DType.MATRIX_ROTATION3D);
    }    
    
    @Test
    public void testGetSetRollPitchYawAngles() throws WrongSizeException {
        
        //randomizer to randomly compute euler angles
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double roll = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double yaw = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        
        MatrixRotation3D rotation = new MatrixRotation3D();
        
        //set angles
        rotation.setRollPitchYaw(roll, pitch, yaw);
        
        //retrieve angles to check correctness
        assertTrue(Math.abs(rotation.getRollAngle() - roll) < ABSOLUTE_ERROR ||
                Math.abs(rotation.getRollAngle2() - roll) < ABSOLUTE_ERROR);
        assertTrue(Math.abs(rotation.getPitchAngle() - pitch) < ABSOLUTE_ERROR ||
                Math.abs(rotation.getPitchAngle2() - roll) < ABSOLUTE_ERROR);
        assertTrue(Math.abs(rotation.getYawAngle() - yaw) < ABSOLUTE_ERROR ||
                Math.abs(rotation.getYawAngle2() - yaw) < ABSOLUTE_ERROR);

        //check internal matrix
        Matrix rotationMatrix = rotation.getInternalMatrix();
        
        Matrix rotationMatrix2 = new Matrix(ROTATION_ROWS, ROTATION_COLS);
        rotationMatrix2.setElementAt(0, 0, Math.cos(pitch) * Math.cos(yaw));
        rotationMatrix2.setElementAt(1, 0, Math.cos(pitch) * Math.sin(yaw));
        rotationMatrix2.setElementAt(2, 0, -Math.sin(pitch));
        
        rotationMatrix2.setElementAt(0, 1, -Math.cos(roll) * Math.sin(yaw) + 
                Math.sin(roll) * Math.sin(pitch) * Math.cos(yaw));
        rotationMatrix2.setElementAt(1, 1, Math.cos(roll) * Math.cos(yaw) + 
                Math.sin(roll) * Math.sin(pitch) * Math.sin(yaw));
        rotationMatrix2.setElementAt(2, 1, Math.sin(roll) * Math.cos(pitch));
        
        rotationMatrix2.setElementAt(0, 2, Math.sin(roll) * Math.sin(yaw) + 
                Math.cos(roll) * Math.sin(pitch) * Math.cos(yaw));
        rotationMatrix2.setElementAt(1, 2, -Math.sin(roll) * Math.cos(yaw) +
                Math.cos(roll) * Math.sin(pitch) * Math.sin(yaw));
        rotationMatrix2.setElementAt(2, 2, Math.cos(roll) * Math.cos(pitch));
        
        //check that rotation matrices are equal (up to a certain error)
        assertTrue(rotationMatrix.equals(rotationMatrix2, ABSOLUTE_ERROR));
    }
    
    @Test
    public void testGetSetRollPitchYaw() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double roll = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double yaw = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        
        MatrixRotation3D rotation = new MatrixRotation3D();

        //set values
        rotation.setRollPitchYaw(roll, pitch, yaw);
        
        //check correctness
        assertTrue(Math.abs(roll - rotation.getRollAngle()) <= ABSOLUTE_ERROR ||
                Math.abs(roll - rotation.getRollAngle2()) <= ABSOLUTE_ERROR);
        
        assertTrue(Math.abs(pitch - rotation.getPitchAngle()) <= ABSOLUTE_ERROR ||
                Math.abs(pitch - rotation.getPitchAngle2()) <= ABSOLUTE_ERROR);
        
        assertTrue(Math.abs(yaw - rotation.getYawAngle()) <= ABSOLUTE_ERROR ||
                Math.abs(yaw - rotation.getYawAngle2()) <= ABSOLUTE_ERROR);        
    }
    
    @Test
    public void testHasGimbalLock() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double roll = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double pitch = 0.0;
        double yaw = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        
        MatrixRotation3D rotation = new MatrixRotation3D();

        //set values
        rotation.setRollPitchYaw(roll, pitch, yaw);
        
        //check correctness
        assertFalse(rotation.hasGimbalLock());
        
        //set new pitch
        pitch = Math.PI / 4.0;
        rotation.setRollPitchYaw(roll, pitch, yaw);
        
        //check correctness
        assertFalse(rotation.hasGimbalLock());
        
        //set new pitch
        pitch = Math.PI / 2.0;
        rotation.setRollPitchYaw(roll, pitch, yaw);
        
        //check correctness
        assertTrue(rotation.hasGimbalLock());
        
        //set new pitch
        pitch = -Math.PI / 2.0;
        rotation.setRollPitchYaw(roll, pitch, yaw);
        
        //check correctness
        assertTrue(rotation.hasGimbalLock());
    }
    
    @Test
    public void testFromRotation() throws AlgebraException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        
        //Find any 3 orthogonal vectors, 1st will be axis of rotation, and
        //the remaining two will lie on the rotation plane and will be used
        //to test theta angle
            
        //To find 3 orthogonal vectors, we use V matrix of a singular
        //decomposition of any Nx3 matrix
        Matrix a = Matrix.createWithUniformRandomValues(1, ROTATION_COLS, 
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        SingularValueDecomposer decomposer = new SingularValueDecomposer(a);
            
        decomposer.decompose();
            
        Matrix vMatrix = decomposer.getV();
            
        //axis of rotation
        double [] axis = vMatrix.getSubmatrixAsArray(0, 0,
                2, 0);

        MatrixRotation3D matrixRotation = new MatrixRotation3D();
        AxisRotation3D axisRotation = new AxisRotation3D();
        Quaternion quaternion = new Quaternion();

        matrixRotation.setAxisAndRotation(axis, theta);
        axisRotation.setAxisAndRotation(axis, theta);
        quaternion.setAxisAndRotation(axis, theta);                

        //test from matrix rotation
        MatrixRotation3D rotation1 = new MatrixRotation3D();
        rotation1.fromRotation(matrixRotation);
        
        //check correctness
        assertEquals(matrixRotation, rotation1);
        
        //test from axis rotation
        MatrixRotation3D rotation2 = new MatrixRotation3D();
        rotation2.fromRotation(axisRotation);
        
        //check correctness
        //noinspection all
        assertEquals(axisRotation, rotation2);
        
        //test from quaternion
        MatrixRotation3D rotation3 = new MatrixRotation3D();
        rotation3.fromRotation(quaternion);
        
        //check correctness
        //noinspection all
        assertEquals(quaternion, rotation3);
    }
    
    @Test
    public void testToMatrixRotation() throws AlgebraException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        
        //Find any 3 orthogonal vectors, 1st will be axis of rotation, and
        //the remaining two will lie on the rotation plane and will be used
        //to test theta angle
            
        //To find 3 orthogonal vectors, we use V matrix of a singular
        //decomposition of any Nx3 matrix
        Matrix a = Matrix.createWithUniformRandomValues(1, ROTATION_COLS, 
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        SingularValueDecomposer decomposer = new SingularValueDecomposer(a);
            
        decomposer.decompose();
            
        Matrix vMatrix = decomposer.getV();
            
        //axis of rotation
        double [] axis = vMatrix.getSubmatrixAsArray(0, 0,
                2, 0);
        
        MatrixRotation3D rotation = new MatrixRotation3D(axis, theta);
        
        //to matrix rotation
        MatrixRotation3D matrixRotation1 = new MatrixRotation3D();
        rotation.toMatrixRotation(matrixRotation1);
        MatrixRotation3D matrixRotation2 = rotation.toMatrixRotation();
        
        //check correctness
        assertEquals(rotation, matrixRotation1);
        assertEquals(rotation, matrixRotation2);        
    }
    
    @Test
    public void testToAxisRotation() throws AlgebraException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            //Find any 3 orthogonal vectors, 1st will be axis of rotation, and
            //the remaining two will lie on the rotation plane and will be used
            //to test theta angle

            //To find 3 orthogonal vectors, we use V matrix of a singular
            //decomposition of any Nx3 matrix
            Matrix a = Matrix.createWithUniformRandomValues(1, ROTATION_COLS,
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            SingularValueDecomposer decomposer = new SingularValueDecomposer(a);

            decomposer.decompose();

            Matrix vMatrix = decomposer.getV();

            //axis of rotation
            double[] axis = vMatrix.getSubmatrixAsArray(0, 0,
                    2, 0);

            MatrixRotation3D rotation = new MatrixRotation3D(axis, theta);

            //to axis rotation
            AxisRotation3D axisRotation1 = new AxisRotation3D();
            rotation.toAxisRotation(axisRotation1);
            AxisRotation3D axisRotation2 = rotation.toAxisRotation();

            //check correctness
            if (!rotation.equals(axisRotation1)) {
                continue;
            }
            //noinspection all
            assertEquals(rotation, axisRotation1);
            if (!rotation.equals(axisRotation2)) {
                continue;
            }
            //noinspection all
            assertEquals(rotation, axisRotation2);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }
    
    @Test
    public void testToQuaternion() throws AlgebraException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        
        //Find any 3 orthogonal vectors, 1st will be axis of rotation, and
        //the remaining two will lie on the rotation plane and will be used
        //to test theta angle
            
        //To find 3 orthogonal vectors, we use V matrix of a singular
        //decomposition of any Nx3 matrix
        Matrix a = Matrix.createWithUniformRandomValues(1, ROTATION_COLS, 
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        SingularValueDecomposer decomposer = new SingularValueDecomposer(a);
            
        decomposer.decompose();
            
        Matrix vMatrix = decomposer.getV();
            
        //axis of rotation
        double [] axis = vMatrix.getSubmatrixAsArray(0, 0,
                2, 0);
        
        MatrixRotation3D rotation = new MatrixRotation3D(axis, theta);
                
        //to quaternion
        Quaternion quaternion1 = new Quaternion();
        rotation.toQuaternion(quaternion1);
        Quaternion quaternion2 = rotation.toQuaternion();
        
        //check correctness
        //noinspection all
        assertEquals(rotation, quaternion1);
        //noinspection all
        assertEquals(rotation, quaternion2);
    }        
}
