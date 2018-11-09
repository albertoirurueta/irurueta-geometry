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

import java.util.Random;

import static org.junit.Assert.*;

public class Rotation3DTest {
    private static final int ROTATION_COLS = 3;
    private static final int INHOM_COORDS = 3;
    
    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;
    
    private static final double MIN_ANGLE_DEGREES = -90.0;
    private static final double MAX_ANGLE_DEGREES = 90.0;
    
    private static final double ABSOLUTE_ERROR = 1e-6;
    
    private static final int TIMES = 10;
    
    public Rotation3DTest() { }
    
    @BeforeClass
    public static void setUpClass() { }
    
    @AfterClass
    public static void tearDownClass() { }
    
    @Before
    public void setUp() { }
    
    @After
    public void tearDown() { }
    
    @Test
    public void testCreate() throws WrongSizeException, RotationException {
        Rotation3D rotation = Rotation3D.create();
        assertEquals(rotation.getType(), Rotation3D.DEFAULT_TYPE);        
        Matrix m = rotation.asInhomogeneousMatrix();
        assertTrue(m.equals(Matrix.identity(INHOM_COORDS, INHOM_COORDS), 
                ABSOLUTE_ERROR));
        
        
        rotation = Rotation3D.create(Rotation3DType.AXIS_ROTATION3D);
        assertEquals(rotation.getType(), Rotation3DType.AXIS_ROTATION3D);
        m = rotation.asInhomogeneousMatrix();
        assertTrue(m.equals(Matrix.identity(INHOM_COORDS, INHOM_COORDS), 
                ABSOLUTE_ERROR));
        
        
        rotation = Rotation3D.create(Rotation3DType.MATRIX_ROTATION3D);
        assertEquals(rotation.getType(), Rotation3DType.MATRIX_ROTATION3D);
        m = rotation.asInhomogeneousMatrix();
        assertTrue(m.equals(Matrix.identity(INHOM_COORDS, INHOM_COORDS), 
                ABSOLUTE_ERROR));  
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double axisX = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double axisY = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double axisZ = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        //normalize axis values to get better results
        double norm = Math.sqrt(axisX * axisX + axisY * axisY + axisZ * axisZ);
        axisX /= norm;
        axisY /= norm;
        axisZ /= norm;
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        
        double[] axis = new double[INHOM_COORDS];
        axis[0] = axisX;
        axis[1] = axisY;
        axis[2] = axisZ;
        
        rotation = Rotation3D.create(axis, theta);
        assertEquals(rotation.getType(), Rotation3D.DEFAULT_TYPE);
        assertArrayEquals(axis, rotation.getRotationAxis(), ABSOLUTE_ERROR);
        assertEquals(theta, rotation.getRotationAngle(), ABSOLUTE_ERROR);
        
        rotation = Rotation3D.create(axis, theta, 
                Rotation3DType.AXIS_ROTATION3D);
        assertEquals(rotation.getType(), Rotation3DType.AXIS_ROTATION3D);
        assertArrayEquals(axis, rotation.getRotationAxis(), ABSOLUTE_ERROR);
        assertEquals(theta, rotation.getRotationAngle(), ABSOLUTE_ERROR);

        rotation = Rotation3D.create(axis, theta, 
                Rotation3DType.MATRIX_ROTATION3D);
        assertEquals(rotation.getType(), Rotation3DType.MATRIX_ROTATION3D);
        double[] axis2 = rotation.getRotationAxis();
        //check axis are equal up to scale
        double scaleX = axis[0] / axis2[0];
        double scaleY = axis[1] / axis2[1];
        double scaleZ = axis[2] / axis2[2];
        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleZ, ABSOLUTE_ERROR);
        assertEquals(scaleZ, scaleX, ABSOLUTE_ERROR);
        assertEquals(Math.abs(theta / rotation.getRotationAngle()), 1.0, 
                ABSOLUTE_ERROR);
        
        
        rotation = Rotation3D.create(axisX, axisY, axisZ, theta);
        assertEquals(rotation.getType(), Rotation3D.DEFAULT_TYPE);
        assertEquals(axisX, rotation.getRotationAxis()[0], ABSOLUTE_ERROR);
        assertEquals(axisY, rotation.getRotationAxis()[1], ABSOLUTE_ERROR);
        assertEquals(axisZ, rotation.getRotationAxis()[2], ABSOLUTE_ERROR);
        assertEquals(theta, rotation.getRotationAngle(), ABSOLUTE_ERROR);

        rotation = Rotation3D.create(axisX, axisY, axisZ, theta,
                Rotation3DType.AXIS_ROTATION3D);
        assertEquals(rotation.getType(), Rotation3DType.AXIS_ROTATION3D);
        assertEquals(axisX, rotation.getRotationAxis()[0], ABSOLUTE_ERROR);
        assertEquals(axisY, rotation.getRotationAxis()[1], ABSOLUTE_ERROR);
        assertEquals(axisZ, rotation.getRotationAxis()[2], ABSOLUTE_ERROR);
        assertEquals(theta, rotation.getRotationAngle(), ABSOLUTE_ERROR);

        rotation = Rotation3D.create(axisX, axisY, axisZ, theta,
                Rotation3DType.MATRIX_ROTATION3D);
        assertEquals(rotation.getType(), Rotation3DType.MATRIX_ROTATION3D);
        axis2 = rotation.getRotationAxis();
        scaleX = axisX / axis2[0];
        scaleY = axisY / axis2[1];
        scaleZ = axisZ / axis2[2];
        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleZ, ABSOLUTE_ERROR);
        assertEquals(scaleZ, scaleX, ABSOLUTE_ERROR);    
        assertEquals(Math.abs(theta / rotation.getRotationAngle()), 1.0, 
                ABSOLUTE_ERROR);
    }
    
    @Test
    public void testGetSetAxisAndRotation() throws WrongSizeException, 
            NotReadyException, LockedException, DecomposerException,
            NotAvailableException, RotationException {
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        
        Rotation3D rotation = Rotation3D.create();
        
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
        Matrix axisMatrix = v.getSubmatrix(0, 0, 2, 0);
        
        //inhomogeneous coordinates of point laying on rotation plane
        Matrix pointMatrix = v.getSubmatrix(0, 1, 2, 1);
        
        axis = axisMatrix.toArray();
        rotation.setAxisAndRotation(axis, theta);
        
        //To test correctness of rotation, axis should remain equal
        Matrix rotationMatrix = rotation.asInhomogeneousMatrix();
        
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
    public void testEquals() throws RotationException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double axisX = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            double axisY = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            double axisZ = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            //normalize axis values to get better results
            double norm = Math.sqrt(axisX * axisX + axisY * axisY + axisZ * axisZ);
            axisX /= norm;
            axisY /= norm;
            axisZ /= norm;
            double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double threshold = randomizer.nextDouble(
                    Rotation2D.DEFAULT_COMPARISON_THRESHOLD, 
                    2.0 * Rotation2D.DEFAULT_COMPARISON_THRESHOLD);
            double theta2 = theta + threshold;


            double[] axis = new double[INHOM_COORDS];
            axis[0] = axisX;
            axis[1] = axisY;
            axis[2] = axisZ;

            double[] axis2 = ArrayUtils.multiplyByScalarAndReturnNew(axis, -1.0);

            Rotation3D rotation1 = Rotation3D.create(axis, theta);
            //with same axis and rotation angle
            Rotation3D rotation2 = Rotation3D.create(axis, theta);
            //with reversed axis and rotation angle
            Rotation3D rotation3 = Rotation3D.create(axis2, -theta);

            //with different angle
            Rotation3D rotation4 = Rotation3D.create(axis, theta2);
            //with different axis
            double[] axis3 = new double[INHOM_COORDS];
            axis3[0] = axisX + randomizer.nextDouble();
            axis3[1] = axisY + randomizer.nextDouble();
            axis3[2] = axisZ + randomizer.nextDouble();
            Rotation3D rotation5 = Rotation3D.create(axis3, theta);

            //check equalness
            assertEquals(rotation1, rotation1);
            assertTrue(rotation1.equals(rotation1,
                    Rotation3D.DEFAULT_COMPARISON_THRESHOLD));
            assertEquals(rotation1, rotation2);
            assertTrue(rotation1.equals(rotation2, 
                    Rotation3D.DEFAULT_COMPARISON_THRESHOLD));
            assertEquals(rotation1, rotation3);
            assertTrue(rotation1.equals(rotation3, 
                    Rotation3D.DEFAULT_COMPARISON_THRESHOLD));

            assertNotEquals(rotation1, rotation4);
            assertFalse(rotation1.equals(rotation4, 
                    Rotation3D.DEFAULT_COMPARISON_THRESHOLD));
            assertNotEquals(rotation1, rotation5);
            assertFalse(rotation1.equals(rotation5, 
                    Rotation3D.DEFAULT_COMPARISON_THRESHOLD));                

            //check with larger threshold
            assertTrue(rotation1.equals(rotation4, 2.0 * Math.PI));
            double largeThreshold = 5.0 * Utils.normF(axis3);
            if (!rotation1.equals(rotation5, largeThreshold)) {
                continue;
            }
            assertTrue(rotation1.equals(rotation5, largeThreshold));
            numValid++;
        }
        assertTrue(numValid > 0);
    }
    
    @Test
    public void testFromRotation() throws AlgebraException, RotationException {
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
            double[] axis = vMatrix.getSubmatrixAsArray(0, 0, 2, 0);

            MatrixRotation3D matrixRotation = new MatrixRotation3D();
            AxisRotation3D axisRotation = new AxisRotation3D();
            Quaternion quaternion = new Quaternion();

            matrixRotation.setAxisAndRotation(axis, theta);
            axisRotation.setAxisAndRotation(axis, theta);
            quaternion.setAxisAndRotation(axis, theta);

            //test from matrix rotation
            Rotation3D rotation1 = Rotation3D.create();
            rotation1.fromRotation(matrixRotation);

            //check correctness
            if (!matrixRotation.equals(rotation1, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(matrixRotation.equals(rotation1, ABSOLUTE_ERROR));

            //test from axis rotation
            Rotation3D rotation2 = Rotation3D.create();
            rotation2.fromRotation(axisRotation);

            //check correctness
            if (!axisRotation.equals(rotation2, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(axisRotation.equals(rotation2, ABSOLUTE_ERROR));

            //test from quaternion
            Rotation3D rotation3 = Rotation3D.create();
            rotation3.fromRotation(quaternion);

            //check correctness
            if (!quaternion.equals(rotation3, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(quaternion.equals(rotation3, ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
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
        
        Rotation3D rotation = Rotation3D.create(axis, theta);
        
        //to matrix rotation
        MatrixRotation3D matrixRotation1 = new MatrixRotation3D();
        rotation.toMatrixRotation(matrixRotation1);
        MatrixRotation3D matrixRotation2 = rotation.toMatrixRotation();
        
        //check correctness
        assertEquals(rotation, matrixRotation1);
        assertEquals(rotation, matrixRotation2);
        
        //to axis rotation
        AxisRotation3D axisRotation1 = new AxisRotation3D();
        rotation.toAxisRotation(axisRotation1);
        AxisRotation3D axisRotation2 = rotation.toAxisRotation();
        
        //check correctness
        assertEquals(rotation, axisRotation1);
        assertEquals(rotation, axisRotation2);
        
        //to quaternion
        Quaternion quaternion1 = new Quaternion();
        rotation.toQuaternion(quaternion1);
        Quaternion quaternion2 = rotation.toQuaternion();
        
        //check correctness
        assertEquals(rotation, quaternion1);
        assertEquals(rotation, quaternion2);
    }
}
