/*
 * Copyright (C) 2016 Alberto Irurueta Carro (alberto@irurueta.com)
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
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.Random;

import static org.junit.Assert.*;

public class RotationUtilsTest {
    
    private static final double MIN_ANGLE_DEGREES = 0.0;
    private static final double MAX_ANGLE_DEGREES = 90.0;
    
    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;
    
    private static final double ABSOLUTE_ERROR = 1e-6;
    
    public RotationUtilsTest() { }
    
    @BeforeClass
    public static void setUpClass() { }
    
    @AfterClass
    public static void tearDownClass() { }
    
    @Before
    public void setUp() { }
    
    @After
    public void tearDown() { }
    
    @Test
    public void testW2omega() throws AlgebraException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        double w1 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double w2 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double w3 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        Matrix result = new Matrix(4,4);
        RotationUtils.w2omega(w1, w2, w3, result);
        
        //check correctness
        Matrix resultB = new Matrix(4,4);
        resultB.setSubmatrix(0, 0, 3, 3,
                new double[] {
                    0.0, w1, w2, w3,
                    -w1, 0.0, -w3, w2,
                    -w2, w3, 0.0, -w1,
                    -w3, -w2, w1, 0.0
                }, true);
        
        assertTrue(result.equals(resultB, ABSOLUTE_ERROR));
        
        //Force IllegalArgumentException
        try {
            RotationUtils.w2omega(w1, w2, w3, new Matrix(1,1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        
        //test with array
        double[] w = new double[]{ w1, w2, w3 };
        RotationUtils.w2omega(w, result);
        
        //check correctness
        assertTrue(result.equals(resultB, ABSOLUTE_ERROR));
        
        //Force IllegalArgumentException
        try {
            RotationUtils.w2omega(new double[1], result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            RotationUtils.w2omega(w, new Matrix(1,1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }

        //test with new instance
        result = RotationUtils.w2omega(w1, w2, w3);

        //check correctness
        assertTrue(result.equals(resultB, ABSOLUTE_ERROR));        
        
        //test with array
        result = RotationUtils.w2omega(w);
        
        //check correctness
        assertTrue(result.equals(resultB, ABSOLUTE_ERROR));
        
        //Force IllegalArgumentException
        result = null;
        try {
            result = RotationUtils.w2omega(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(result);
    }
    
    @Test
    public void testQuaternionToPiMatrix() throws AlgebraException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        double roll = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double yaw = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        
        Quaternion q = new Quaternion(roll, pitch, yaw);
        
        double a = q.getA();
        double b = q.getB();
        double c = q.getC();
        double d = q.getD();
        
        Matrix pi1 = new Matrix(Quaternion.N_PARAMS, Quaternion.N_ANGLES);
        RotationUtils.quaternionToPiMatrix(q, pi1);
        Matrix pi2 = RotationUtils.quaternionToPiMatrix(q);
        
        //check correctness
        assertEquals(pi1, pi2);
        
        assertEquals(pi1.getElementAt(0, 0), -b, 0.0);
        assertEquals(pi1.getElementAt(1, 0), a, 0.0);
        assertEquals(pi1.getElementAt(2, 0), d, 0.0);
        assertEquals(pi1.getElementAt(3, 0), -c, 0.0);
        
        assertEquals(pi1.getElementAt(0, 1), -c, 0.0);
        assertEquals(pi1.getElementAt(1, 1), -d, 0.0);
        assertEquals(pi1.getElementAt(2, 1), a, 0.0);
        assertEquals(pi1.getElementAt(3, 1), b, 0.0);
        
        assertEquals(pi1.getElementAt(0, 2), -d, 0.0);
        assertEquals(pi1.getElementAt(1, 2), c, 0.0);
        assertEquals(pi1.getElementAt(2, 2), -b, 0.0);
        assertEquals(pi1.getElementAt(3, 2), a, 0.0);
    }
    
    @Test
    public void testQuaternionToConjugatedPiMatrix() throws AlgebraException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        double roll = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double yaw = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        
        Quaternion q = new Quaternion(roll, pitch, yaw);
        
        double a = q.getA();
        double b = q.getB();
        double c = q.getC();
        double d = q.getD();
        
        Matrix cpi1 = new Matrix(Quaternion.N_PARAMS, Quaternion.N_ANGLES);
        RotationUtils.quaternionToConjugatedPiMatrix(q, cpi1);
        Matrix cpi2 = RotationUtils.quaternionToConjugatedPiMatrix(q);
        
        //check correctness
        assertEquals(cpi1, cpi2);
        
        assertEquals(cpi1.getElementAt(0, 0), b, 0.0);
        assertEquals(cpi1.getElementAt(1, 0), a, 0.0);
        assertEquals(cpi1.getElementAt(2, 0), -d, 0.0);
        assertEquals(cpi1.getElementAt(3, 0), c, 0.0);
        
        assertEquals(cpi1.getElementAt(0, 1), c, 0.0);
        assertEquals(cpi1.getElementAt(1, 1), d, 0.0);
        assertEquals(cpi1.getElementAt(2, 1), a, 0.0);
        assertEquals(cpi1.getElementAt(3, 1), -b, 0.0);
        
        assertEquals(cpi1.getElementAt(0, 2), d, 0.0);
        assertEquals(cpi1.getElementAt(1, 2), -c, 0.0);
        assertEquals(cpi1.getElementAt(2, 2), b, 0.0);
        assertEquals(cpi1.getElementAt(3, 2), a, 0.0);
    }    
    
    @Test
    public void testPiMatrixToConjugatedPiMatrix() throws AlgebraException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        double roll = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double yaw = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        
        Quaternion q = new Quaternion(roll, pitch, yaw);
        
        Matrix pi = RotationUtils.quaternionToPiMatrix(q);
        Matrix cpi = RotationUtils.quaternionToConjugatedPiMatrix(q);
        
        Matrix cpi2 = new Matrix(Quaternion.N_PARAMS, Quaternion.N_ANGLES);
        RotationUtils.piMatrixToConjugatedPiMatrix(pi, cpi2);
        Matrix cpi3 = RotationUtils.piMatrixToConjugatedPiMatrix(pi);
        
        assertEquals(cpi, cpi2);
        assertEquals(cpi, cpi3);
        assertEquals(cpi2, cpi3);
    }
    
    @Test
    public void testRotationMatrixTimesVector() throws AlgebraException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        double roll = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double yaw = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        
        Quaternion q = new Quaternion(roll, pitch, yaw);
        
        double[] point = new double[
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
        double[] result = new double[point.length];
        double[] result2 = new double[point.length];
        Matrix jacobianQ = new Matrix(Quaternion.N_ANGLES, Quaternion.N_PARAMS);
        Matrix jacobianP = new Matrix(Quaternion.N_ANGLES, Quaternion.N_ANGLES);
        RotationUtils.rotationMatrixTimesVector(q, point, result, jacobianQ, 
                jacobianP);
        
        //check correctness
        Point3D rotated = new InhomogeneousPoint3D(q.rotate(Point3D.create(
                CoordinatesType.INHOMOGENEOUS_COORDINATES, point)));
        rotated.asArray(result2);
        
        assertArrayEquals(result2, result, ABSOLUTE_ERROR);
        
        assertTrue(jacobianP.equals(q.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
        
        //check jacobian
        double a = q.getA();
        double b = q.getB();
        double c = q.getC();
        double d = q.getD();
        
        double x = point[0];
        double y = point[1];
        double z = point[2];
        
        double axdycz = 2.0*(a*x - d*y + c*z);
        double bxcydz = 2.0*(b*x + c*y + d*z);
        double cxbyaz = 2.0*(c*x - b*y - a*z);
        double dxaybz = 2.0*(d*x + a*y - b*z);
        
        assertEquals(jacobianQ.getElementAt(0, 0), axdycz, ABSOLUTE_ERROR);
        assertEquals(jacobianQ.getElementAt(1, 0), dxaybz, ABSOLUTE_ERROR);
        assertEquals(jacobianQ.getElementAt(2, 0), -cxbyaz, ABSOLUTE_ERROR);
        
        assertEquals(jacobianQ.getElementAt(0, 1), bxcydz, ABSOLUTE_ERROR);
        assertEquals(jacobianQ.getElementAt(1, 1), cxbyaz, ABSOLUTE_ERROR);
        assertEquals(jacobianQ.getElementAt(2, 1), dxaybz, ABSOLUTE_ERROR);
        
        assertEquals(jacobianQ.getElementAt(0, 2), -cxbyaz, ABSOLUTE_ERROR);
        assertEquals(jacobianQ.getElementAt(1, 2), bxcydz, ABSOLUTE_ERROR);
        assertEquals(jacobianQ.getElementAt(2, 2), -axdycz, ABSOLUTE_ERROR);
        
        assertEquals(jacobianQ.getElementAt(0, 3), -dxaybz, ABSOLUTE_ERROR);
        assertEquals(jacobianQ.getElementAt(1, 3), axdycz, ABSOLUTE_ERROR);
        assertEquals(jacobianQ.getElementAt(2, 3), bxcydz, ABSOLUTE_ERROR);
        
        //Force IllegalArgumentException
        try {
            RotationUtils.rotationMatrixTimesVector(q, point, result, 
                    new Matrix(1,1), jacobianP);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            RotationUtils.rotationMatrixTimesVector(q, point, result, jacobianQ, 
                    new Matrix(1,1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        
        
        //test without result
        result = RotationUtils.rotationMatrixTimesVector(q, point, jacobianQ, 
                jacobianP);
        
        //check correctness
        rotated = new InhomogeneousPoint3D(q.rotate(Point3D.create(
                CoordinatesType.INHOMOGENEOUS_COORDINATES, point)));
        rotated.asArray(result2);
        
        assertArrayEquals(result2, result, ABSOLUTE_ERROR);
        
        assertTrue(jacobianP.equals(q.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
        
        //check jacobian
        a = q.getA();
        b = q.getB();
        c = q.getC();
        d = q.getD();
        
        x = point[0];
        y = point[1];
        z = point[2];
        
        axdycz = 2.0*(a*x - d*y + c*z);
        bxcydz = 2.0*(b*x + c*y + d*z);
        cxbyaz = 2.0*(c*x - b*y - a*z);
        dxaybz = 2.0*(d*x + a*y - b*z);
        
        assertEquals(jacobianQ.getElementAt(0, 0), axdycz, ABSOLUTE_ERROR);
        assertEquals(jacobianQ.getElementAt(1, 0), dxaybz, ABSOLUTE_ERROR);
        assertEquals(jacobianQ.getElementAt(2, 0), -cxbyaz, ABSOLUTE_ERROR);
        
        assertEquals(jacobianQ.getElementAt(0, 1), bxcydz, ABSOLUTE_ERROR);
        assertEquals(jacobianQ.getElementAt(1, 1), cxbyaz, ABSOLUTE_ERROR);
        assertEquals(jacobianQ.getElementAt(2, 1), dxaybz, ABSOLUTE_ERROR);
        
        assertEquals(jacobianQ.getElementAt(0, 2), -cxbyaz, ABSOLUTE_ERROR);
        assertEquals(jacobianQ.getElementAt(1, 2), bxcydz, ABSOLUTE_ERROR);
        assertEquals(jacobianQ.getElementAt(2, 2), -axdycz, ABSOLUTE_ERROR);
        
        assertEquals(jacobianQ.getElementAt(0, 3), -dxaybz, ABSOLUTE_ERROR);
        assertEquals(jacobianQ.getElementAt(1, 3), axdycz, ABSOLUTE_ERROR);
        assertEquals(jacobianQ.getElementAt(2, 3), bxcydz, ABSOLUTE_ERROR);
        
        //Force IllegalArgumentException
        double[] result3 = null;
        try {
            result3 = RotationUtils.rotationMatrixTimesVector(q, point, 
                    new Matrix(1,1), jacobianP);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result3 = RotationUtils.rotationMatrixTimesVector(q, point, 
                    jacobianQ, new Matrix(1,1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(result3);

        
        //test without jacobian
        RotationUtils.rotationMatrixTimesVector(q, point, result);
        
        //check correctness
        rotated = new InhomogeneousPoint3D(q.rotate(Point3D.create(
                CoordinatesType.INHOMOGENEOUS_COORDINATES, point)));
        rotated.asArray(result2);
        
        assertArrayEquals(result2, result, ABSOLUTE_ERROR);
        
        assertTrue(jacobianP.equals(q.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
        
        
        //test without jacobian and result
        result = RotationUtils.rotationMatrixTimesVector(q, point);
        
        //check correctness
        rotated = new InhomogeneousPoint3D(q.rotate(Point3D.create(
                CoordinatesType.INHOMOGENEOUS_COORDINATES, point)));
        rotated.asArray(result2);
        
        assertArrayEquals(result2, result, ABSOLUTE_ERROR);
        
        assertTrue(jacobianP.equals(q.asInhomogeneousMatrix(), ABSOLUTE_ERROR));        
    }
    
    @Test
    public void testTransposedRotationMatrixTimesVector() 
            throws AlgebraException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        double roll = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double yaw = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        
        Quaternion q = new Quaternion(roll, pitch, yaw);
        
        double[] point = new double[
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
        double[] result = new double[point.length];
        double[] result2 = new double[point.length];
        Matrix jacobianQ = new Matrix(Quaternion.N_ANGLES, Quaternion.N_PARAMS);
        Matrix jacobianP = new Matrix(Quaternion.N_ANGLES, Quaternion.N_ANGLES);
        RotationUtils.transposedRotationMatrixTimesVector(q, point, result, 
                jacobianQ, jacobianP);
        
        //check correctness
        Point3D rotated = new InhomogeneousPoint3D(q.inverseAndReturnNew().
                rotate(Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES,
                        point)));
        rotated.asArray(result2);
        
        assertArrayEquals(result2, result, ABSOLUTE_ERROR);
        
        assertTrue(jacobianP.equals(q.asInhomogeneousMatrix().
                transposeAndReturnNew(), ABSOLUTE_ERROR));
        
        //check jacobian
        double a = q.getA();
        double b = q.getB();
        double c = q.getC();
        double d = q.getD();
        
        double x = point[0];
        double y = point[1];
        double z = point[2];
        
        double axdycz = 2.0*(a*x + d*y - c*z);
        double bxcydz = 2.0*(b*x + c*y + d*z);
        double cxbyaz = 2.0*(c*x - b*y + a*z);
        double dxaybz = 2.0*(d*x - a*y - b*z);
        
        assertEquals(jacobianQ.getElementAt(0, 0), axdycz, ABSOLUTE_ERROR);
        assertEquals(jacobianQ.getElementAt(1, 0), -dxaybz, ABSOLUTE_ERROR);
        assertEquals(jacobianQ.getElementAt(2, 0), cxbyaz, ABSOLUTE_ERROR);
        
        assertEquals(jacobianQ.getElementAt(0, 1), bxcydz, ABSOLUTE_ERROR);
        assertEquals(jacobianQ.getElementAt(1, 1), cxbyaz, ABSOLUTE_ERROR);
        assertEquals(jacobianQ.getElementAt(2, 1), dxaybz, ABSOLUTE_ERROR);
        
        assertEquals(jacobianQ.getElementAt(0, 2), -cxbyaz, ABSOLUTE_ERROR);
        assertEquals(jacobianQ.getElementAt(1, 2), bxcydz, ABSOLUTE_ERROR);
        assertEquals(jacobianQ.getElementAt(2, 2), axdycz, ABSOLUTE_ERROR);
        
        assertEquals(jacobianQ.getElementAt(0, 3), -dxaybz, ABSOLUTE_ERROR);
        assertEquals(jacobianQ.getElementAt(1, 3), -axdycz, ABSOLUTE_ERROR);
        assertEquals(jacobianQ.getElementAt(2, 3), bxcydz, ABSOLUTE_ERROR);
        
        //Force IllegalArgumentException
        try {
            RotationUtils.transposedRotationMatrixTimesVector(q, point, result, 
                    new Matrix(1,1), jacobianP);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            RotationUtils.transposedRotationMatrixTimesVector(q, point, result, 
                    jacobianQ, new Matrix(1,1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        
        
        //test without result
        result = RotationUtils.transposedRotationMatrixTimesVector(q, point, 
                jacobianQ, jacobianP);
        
        //check correctness
        rotated = new InhomogeneousPoint3D(q.rotate(Point3D.create(
                CoordinatesType.INHOMOGENEOUS_COORDINATES, point)));
        rotated.asArray(result2);
        
        assertArrayEquals(result2, result, ABSOLUTE_ERROR);
        
        assertTrue(jacobianP.equals(q.asInhomogeneousMatrix().
                transposeAndReturnNew(), ABSOLUTE_ERROR));
        
        //check jacobian
        a = q.getA();
        b = q.getB();
        c = q.getC();
        d = q.getD();
        
        x = point[0];
        y = point[1];
        z = point[2];
        
        axdycz = 2.0*(a*x + d*y - c*z);
        bxcydz = 2.0*(b*x + c*y + d*z);
        cxbyaz = 2.0*(c*x - b*y + a*z);
        dxaybz = 2.0*(d*x - a*y - b*z);
        
        assertEquals(jacobianQ.getElementAt(0, 0), axdycz, ABSOLUTE_ERROR);
        assertEquals(jacobianQ.getElementAt(1, 0), -dxaybz, ABSOLUTE_ERROR);
        assertEquals(jacobianQ.getElementAt(2, 0), cxbyaz, ABSOLUTE_ERROR);
        
        assertEquals(jacobianQ.getElementAt(0, 1), bxcydz, ABSOLUTE_ERROR);
        assertEquals(jacobianQ.getElementAt(1, 1), cxbyaz, ABSOLUTE_ERROR);
        assertEquals(jacobianQ.getElementAt(2, 1), dxaybz, ABSOLUTE_ERROR);
        
        assertEquals(jacobianQ.getElementAt(0, 2), -cxbyaz, ABSOLUTE_ERROR);
        assertEquals(jacobianQ.getElementAt(1, 2), bxcydz, ABSOLUTE_ERROR);
        assertEquals(jacobianQ.getElementAt(2, 2), axdycz, ABSOLUTE_ERROR);
        
        assertEquals(jacobianQ.getElementAt(0, 3), -dxaybz, ABSOLUTE_ERROR);
        assertEquals(jacobianQ.getElementAt(1, 3), -axdycz, ABSOLUTE_ERROR);
        assertEquals(jacobianQ.getElementAt(2, 3), bxcydz, ABSOLUTE_ERROR);
        
        //Force IllegalArgumentException
        double[] result3 = null;
        try {
            result3 = RotationUtils.transposedRotationMatrixTimesVector(q, 
                    point, new Matrix(1,1), jacobianP);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result3 = RotationUtils.transposedRotationMatrixTimesVector(q, 
                    point, jacobianQ, new Matrix(1,1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(result3);

        
        //test without jacobian
        RotationUtils.transposedRotationMatrixTimesVector(q, point, result);
        
        //check correctness
        rotated = new InhomogeneousPoint3D(q.rotate(Point3D.create(
                CoordinatesType.INHOMOGENEOUS_COORDINATES, point)));
        rotated.asArray(result2);
        
        assertArrayEquals(result2, result, ABSOLUTE_ERROR);
        
        assertTrue(jacobianP.equals(q.asInhomogeneousMatrix().
                transposeAndReturnNew(), ABSOLUTE_ERROR));
        
        
        //test without jacobian and result
        result = RotationUtils.transposedRotationMatrixTimesVector(q, point);
        
        //check correctness
        rotated = new InhomogeneousPoint3D(q.rotate(Point3D.create(
                CoordinatesType.INHOMOGENEOUS_COORDINATES, point)));
        rotated.asArray(result2);
        
        assertArrayEquals(result2, result, ABSOLUTE_ERROR);
        
        assertTrue(jacobianP.equals(q.asInhomogeneousMatrix().
                transposeAndReturnNew(), ABSOLUTE_ERROR));                
    }
    
    @Test
    public void testCameraBodyToCameraSensorRotation() throws RotationException {
        MatrixRotation3D rot = new MatrixRotation3D();
        RotationUtils.cameraBodyToCameraSensorRotation(rot);
        
        MatrixRotation3D rot2 = 
                RotationUtils.cameraBodyToCameraSensorRotation();
        
        //check correctness
        assertTrue(rot.equals(rot2, ABSOLUTE_ERROR));
        
        assertEquals(rot.getRollAngle(), -Math.PI / 2.0, ABSOLUTE_ERROR);
        assertEquals(rot.getPitchAngle(), 0.0, ABSOLUTE_ERROR);
        assertEquals(rot.getYawAngle(), -Math.PI / 2.0, ABSOLUTE_ERROR);
        
        assertEquals(rot2.getRollAngle(), -Math.PI / 2.0, ABSOLUTE_ERROR);
        assertEquals(rot2.getPitchAngle(), 0.0, ABSOLUTE_ERROR);
        assertEquals(rot2.getYawAngle(), -Math.PI / 2.0, ABSOLUTE_ERROR);
    }
    
    @Test
    public void testQuaternionToEulerGaussian() throws AlgebraException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        double roll = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double yaw = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        
        Quaternion q = new Quaternion(roll, pitch, yaw);
        
        Matrix covQ = Matrix.identity(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
        double[] euler = new double[Quaternion.N_ANGLES];
        Matrix covE = new Matrix(Quaternion.N_ANGLES, Quaternion.N_ANGLES);
        RotationUtils.quaternionToEulerGaussian(q, covQ, euler, covE);
        
        //check correctness
        double[] angles = q.toEulerAngles();
        assertArrayEquals(euler, angles, ABSOLUTE_ERROR);
        
        Matrix j = new Matrix(Quaternion.N_ANGLES, Quaternion.N_PARAMS);
        q.toEulerAngles(angles, j);
        Matrix covE2 = j.multiplyAndReturnNew(covQ).multiplyAndReturnNew(
                j.transposeAndReturnNew());
        
        assertEquals(covE, covE2);
    }
    
    @Test
    public void testAngularRatesToSkew() throws AlgebraException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        double wx = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double wy = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double wz = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        double[] angularRates = new double[]{wx,wy,wz};
        
        Matrix skew1 = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
        RotationUtils.angularRatesToSkew(angularRates, skew1);        
        Matrix skew2 = RotationUtils.angularRatesToSkew(angularRates);
        
        assertEquals(skew1, skew2);
        
        Matrix skew3 = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
        skew3.setElementAtIndex(0, 0.0);
        skew3.setElementAtIndex(1, wx);
        skew3.setElementAtIndex(2, wy);
        skew3.setElementAtIndex(3, wz);
        
        skew3.setElementAtIndex(4, -wx);
        skew3.setElementAtIndex(5, 0.0);
        skew3.setElementAtIndex(6, -wz);
        skew3.setElementAtIndex(7, wy);
        
        skew3.setElementAtIndex(8, -wy);
        skew3.setElementAtIndex(9, wz);
        skew3.setElementAtIndex(10, 0.0);
        skew3.setElementAtIndex(11, -wx);
        
        skew3.setElementAtIndex(12, -wz);
        skew3.setElementAtIndex(13, -wy);
        skew3.setElementAtIndex(14, wx);
        skew3.setElementAtIndex(15, 0.0);
        
        assertEquals(skew1, skew3);
        assertEquals(skew2, skew3);
        
        //Force IllegalArgumentException
        try {
            RotationUtils.angularRatesToSkew(new double[1], skew1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            RotationUtils.angularRatesToSkew(angularRates, new Matrix(1,1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        
        skew2 = null;
        try {
            skew2 = RotationUtils.angularRatesToSkew(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(skew2);
    }
    
}
