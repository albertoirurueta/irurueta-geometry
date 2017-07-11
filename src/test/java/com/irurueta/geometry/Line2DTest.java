/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.Line2D
 * 
 * @author Alberto Irurueta (alberto@irureta.com)
 * @date July 6, 2012
 */
package com.irurueta.geometry;

import com.irurueta.algebra.*;
import com.irurueta.statistics.UniformRandomizer;
import java.util.Random;
import org.junit.*;
import static org.junit.Assert.*;

public class Line2DTest {
    
    public static final int HOM_COORDS = 3;
    public static final int INHOM_COORDS = 2;
    
    public static final double PRECISION_ERROR = 1e-6;
    public static final double MIN_RANDOM_VALUE = 1.0;
    public static final double MAX_RANDOM_VALUE = 100.0;
    public static final double MIN_RANDOM_DISTANCE = -100.0;
    public static final double MAX_RANDOM_DISTANCE = 100.0;
    public static final double MIN_DEGREES = -90.0;
    public static final double MAX_DEGREES = 90.0;
    
    public static final double ABSOLUTE_ERROR = 1e-8;
    
    public static final int TIMES = 100;
    
    public Line2DTest() {
    }

    @BeforeClass
    public static void setUpClass() throws Exception {
    }

    @AfterClass
    public static void tearDownClass() throws Exception {
    }
    
    @Before
    public void setUp() {
    }
    
    @After
    public void tearDown() {
    }

    @Test
    public void testConstructor() throws WrongSizeException, NotReadyException, 
        LockedException, DecomposerException, 
        com.irurueta.algebra.NotAvailableException{
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        double a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        Line2D line = new Line2D();
        assertEquals(line.getA(), 0.0, 0.0);
        assertEquals(line.getB(), 0.0, 0.0);
        assertEquals(line.getC(), 0.0, 0.0);
        assertFalse(line.isNormalized());
        
        line = new Line2D(a, b, c);
        assertEquals(line.getA(), a, 0.0);
        assertEquals(line.getB(), b, 0.0);
        assertEquals(line.getC(), c, 0.0);
        assertFalse(line.isNormalized());
        
        //instantiate line using array
        double[] array = new double[HOM_COORDS];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        line = new Line2D(array);
        assertEquals(line.getA(), array[0], 0.0);
        assertEquals(line.getB(), array[1], 0.0);
        assertEquals(line.getC(), array[2], 0.0);
        
        //Force IllegalArgumentException
        line = null;
        try{
            line = new Line2D(new double[HOM_COORDS + 1]);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(line);
        
        
        //find 1st point
        double[] array1 = new double[HOM_COORDS];
        randomizer.fill(array1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double lambda = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        
        //find 2nd point coincident with 1st point (up to scale)
        double[] array2 = new double[HOM_COORDS];
        ArrayUtils.multiplyByScalar(array1, lambda, array2);
        
        Point2D point1 = Point2D.create(CoordinatesType.HOMOGENEOUS_COORDINATES, 
                array1);
        Point2D point2 = Point2D.create(CoordinatesType.HOMOGENEOUS_COORDINATES, 
                array2);
        
        //Force CoincidentPointsException
        line = null;
        try{
            line = new Line2D(point1, point2, false);
            fail("CoincidentPointsException expected but not thrown");
        }catch(CoincidentPointsException e){}
        assertNull(line);
        
        //try with 2 non-coincident points
        Matrix m = Matrix.createWithUniformRandomValues(2, HOM_COORDS, 
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        SingularValueDecomposer decomposer = new SingularValueDecomposer(m);
        decomposer.decompose();
        
        //ensure we create a matrix with 2 non linear dependent rows
        while(decomposer.getRank() < 2){
            m = Matrix.createWithUniformRandomValues(2, HOM_COORDS, 
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            decomposer.setInputMatrix(m);
            decomposer.decompose();
        }
        
        //V contains the nullspace of m in the last column, which is the line
        //joining both points in the rows of m
        Matrix V = decomposer.getV();
        
        point1 = new HomogeneousPoint2D(m.getElementAt(0, 0),
                m.getElementAt(0, 1),
                m.getElementAt(0, 2));
        point2 = new HomogeneousPoint2D(m.getElementAt(1, 0),
                m.getElementAt(1, 1),
                m.getElementAt(1, 2));
        
        point1.normalize();
        point2.normalize();
        
        double[] arrayLine = new double[HOM_COORDS];
        arrayLine[0] = V.getElementAt(0, 2);
        arrayLine[1] = V.getElementAt(1, 2);
        arrayLine[2] = V.getElementAt(2, 2);
        
        line = new Line2D(point1, point2);
        
        //check correctness of obtained line
        double scaleA = line.getA() / arrayLine[0];
        double scaleB = line.getB() / arrayLine[1];
        double scaleC = line.getC() / arrayLine[2];
        
        assertEquals(scaleA, scaleB, PRECISION_ERROR);
        assertEquals(scaleB, scaleC, PRECISION_ERROR);
        assertEquals(scaleC, scaleA, PRECISION_ERROR);
        
        //create line using point and director vector
        array = new double[HOM_COORDS];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        Point2D point = Point2D.create(CoordinatesType.HOMOGENEOUS_COORDINATES, 
                array);
        
        double[] directorVector = new double[INHOM_COORDS];
        randomizer.fill(directorVector, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double norm = com.irurueta.algebra.Utils.normF(directorVector);
        directorVector = ArrayUtils.multiplyByScalarAndReturnNew(directorVector, 
                1.0 / norm);
        
        line = new Line2D(point, directorVector);
        line.normalize();
        
        //check correctnes of instantiated line
        assertTrue(line.isLocus(point));

        double[] directorVector2 = line.getDirectorVector();
        
        //check that both director vectors are equal
        double scale1 = directorVector2[0] / directorVector[0];
        double scale2 = directorVector2[1] / directorVector[1];
        assertEquals(scale1, scale2, PRECISION_ERROR);
        
        //Force IllegalArgumentException
        double[] wrongArray = new double[HOM_COORDS];
        line = null;
        try{
            line = new Line2D(point, wrongArray);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(line);
    }
    
    @Test
    public void testGettersAndSetters() throws WrongSizeException{
        for(int t = 0; t < TIMES; t++){
            double[] array = new double[HOM_COORDS];
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            Line2D line = new Line2D();

            //set parameters using array
            line.setParameters(array);
            assertEquals(line.getA(), array[0], 0.0);
            assertEquals(line.getB(), array[1], 0.0);
            assertEquals(line.getC(), array[2], 0.0);

            double a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            double b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            double c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            line.setA(a);
            line.setB(b);
            line.setC(c);

            assertEquals(line.getA(), a, 0.0);
            assertEquals(line.getB(), b, 0.0);
            assertEquals(line.getC(), c, 0.0);

            a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            line.setParameters(a, b, c);
            assertEquals(line.getA(), a, 0.0);
            assertEquals(line.getB(), b, 0.0);
            assertEquals(line.getC(), c, 0.0);

            assertEquals(line.getAngle(), Math.atan(-a/b), PRECISION_ERROR);
            assertEquals(line.getSlope(), -a/b, PRECISION_ERROR);

            //find random angle in radians
            double angle = randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES) * 
                    Math.PI / 180.0;
            line.setAngle(angle);

            assertEquals(line.getAngle(), angle, PRECISION_ERROR);        
            assertEquals(line.getAngle(), Math.atan(-line.getA() / line.getB()), 
                    PRECISION_ERROR);
            assertEquals(line.getSlope(), Math.tan(angle), PRECISION_ERROR);

            double slope = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            line.setSlope(slope);
            assertEquals(line.getSlope(), -line.getA() / line.getB(), 
                    PRECISION_ERROR);
            assertEquals(line.getSlope(), slope, PRECISION_ERROR);

            array = new double[HOM_COORDS];
            randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            line = new Line2D(array);
            c = line.getC();
            b = line.getB();
            assertEquals(line.getYIntercept(), -c / b, PRECISION_ERROR);

            randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            line = new Line2D(array);
            double yIntercept = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            line.setYIntercept(yIntercept);
            assertEquals(line.getYIntercept(), yIntercept, PRECISION_ERROR);
        }
    }
    
    @Test
    public void testSetParametersFromPairOfPoints() throws WrongSizeException, 
        NotReadyException, LockedException, DecomposerException, 
        com.irurueta.algebra.NotAvailableException, CoincidentPointsException{
        
        Matrix m = Matrix.createWithUniformRandomValues(2, HOM_COORDS, 
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        SingularValueDecomposer decomposer = new SingularValueDecomposer(m);
        decomposer.decompose();
        
        while(decomposer.getRank() < 2){
            m = Matrix.createWithUniformRandomValues(2, HOM_COORDS, 
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            decomposer.setInputMatrix(m);
            decomposer.decompose();
        }
        
        Matrix V = decomposer.getV();
        
        Point2D point1 = new HomogeneousPoint2D(m.getElementAt(0, 0),
                m.getElementAt(0, 1),
                m.getElementAt(0, 2));
        Point2D point2 = new HomogeneousPoint2D(m.getElementAt(1, 0),
                m.getElementAt(1, 1),
                m.getElementAt(1, 2));
        
        point1.normalize();
        point2.normalize();
        
        double[] lineArray = new double[HOM_COORDS];
        lineArray[0] = V.getElementAt(0, 2);
        lineArray[1] = V.getElementAt(1, 2);
        lineArray[2] = V.getElementAt(2, 2);
        
        //find colinear point using a third point up to scale of the 1st one
        //using homogeneous coordinates
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double scaleValue = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        
        Point2D coincidentPoint = new HomogeneousPoint2D(
            scaleValue * point1.getHomX(),
            scaleValue * point1.getHomY(),
            scaleValue * point1.getHomW());
        
        Line2D line = new Line2D();
        line.setParametersFromPairOfPoints(point1, point2, false);
        
        //compare line parameters respect to computed line vector. Notice that
        //parameters will be equal up to scale
        double scaleA = line.getA() / lineArray[0];
        double scaleB = line.getB() / lineArray[1];
        double scaleC = line.getC() / lineArray[2];
        
        assertEquals(scaleA, scaleB, PRECISION_ERROR);
        assertEquals(scaleB, scaleC, PRECISION_ERROR);
        assertEquals(scaleC, scaleA, PRECISION_ERROR);
        
        //Force CoincidentPointsException
        try{
            line.setParametersFromPairOfPoints(point1, coincidentPoint, false);
        }catch(CoincidentPointsException e){}
        
        //if we try coincident points without raising an exception then line is
        //instantiated no matter what, although obtained parameters might be 
        //numerically unstable
        line.setParametersFromPairOfPoints(point1, coincidentPoint, true);        
    }
    
    @Test
    public void testSetParamsetersFromPointAndDirectorVector(){
        double[] array = new double[HOM_COORDS];
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        Point2D point = Point2D.create(CoordinatesType.HOMOGENEOUS_COORDINATES, 
                array);
        
        double[] directorVector = new double[INHOM_COORDS];
        randomizer.fill(directorVector, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double norm = com.irurueta.algebra.Utils.normF(directorVector);
        directorVector = ArrayUtils.multiplyByScalarAndReturnNew(directorVector, 
                1.0 / norm);
        
        Line2D line = new Line2D();
        
        
        line.setParametersFromPointAndDirectorVector(point, directorVector);
        line.normalize();
        
        //check correctnes of instantiated line
        assertTrue(line.isLocus(point));

        double[] directorVector2 = line.getDirectorVector();
        
        //check that both director vectors are equal
        double scale1 = directorVector2[0] / directorVector[0];
        double scale2 = directorVector2[1] / directorVector[1];
        assertEquals(scale1, scale2, PRECISION_ERROR);
        
        //Force IllegalArgumentException
        double[] wrongArray = new double[HOM_COORDS];
        try{
            line.setParametersFromPointAndDirectorVector(point, wrongArray);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
    }
    
    @Test
    public void testIsLocus() throws WrongSizeException, NotReadyException, 
        LockedException, DecomposerException, 
        com.irurueta.algebra.NotAvailableException{
        
        //randomly choose 2 points to find their corresponding line
        Matrix m = Matrix.createWithUniformRandomValues(2, HOM_COORDS, 
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        SingularValueDecomposer decomposer = new SingularValueDecomposer(m);
        decomposer.decompose();
        
        while(decomposer.getRank() < 2){
            m = Matrix.createWithUniformRandomValues(2, HOM_COORDS, 
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            decomposer.setInputMatrix(m);
            decomposer.decompose();
        }
        
        Matrix V = decomposer.getV();
        
        Line2D line = new Line2D(V.getElementAt(0, 2),
                V.getElementAt(1, 2),
                V.getElementAt(2, 2));
        
        Point2D point1 = new HomogeneousPoint2D(m.getElementAt(0, 0),
                m.getElementAt(0, 1),
                m.getElementAt(0, 2));
        
        Point2D point2 = new HomogeneousPoint2D(m.getElementAt(1, 0),
                m.getElementAt(1, 1),
                m.getElementAt(1, 2));
        
        point1.normalize();
        point2.normalize();
        
        //point1 and point2 belong to line
        assertTrue(line.isLocus(point1, PRECISION_ERROR));
        assertTrue(line.isLocus(point2, PRECISION_ERROR));
        
        //use line director vector to find another point outside of line focus
        HomogeneousPoint2D point3 = new HomogeneousPoint2D(point1);

        double normDirectorVector = Math.sqrt(line.getA() * line.getA() +
                line.getB() * line.getB());        
        point3.setInhomogeneousCoordinates(point3.getInhomX() + 
                line.getA(), point3.getInhomY() + line.getB());
        
        assertFalse(line.isLocus(point3, PRECISION_ERROR));
        
        //indeed point3 is at normDirectorVector distance from line
        assertEquals(line.signedDistance(point3), 
                normDirectorVector, PRECISION_ERROR);
    }
    
    @Test
    public void testSignedDistance() throws WrongSizeException, 
        NotReadyException, LockedException, DecomposerException, 
        com.irurueta.algebra.NotAvailableException{
        
        //randomly choose 2 points to find their corresponding line
        Matrix m = Matrix.createWithUniformRandomValues(2, HOM_COORDS, 
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        SingularValueDecomposer decomposer = new SingularValueDecomposer(m);
        decomposer.decompose();
        
        while(decomposer.getRank() < 2){
            m = Matrix.createWithUniformRandomValues(2, HOM_COORDS, 
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            decomposer.setInputMatrix(m);
            decomposer.decompose();
        }
        
        Matrix V = decomposer.getV();
        
        Line2D line = new Line2D(V.getElementAt(0, 2),
                V.getElementAt(1, 2),
                V.getElementAt(2, 2));
        
        Point2D point1 = new HomogeneousPoint2D(m.getElementAt(0, 0),
                m.getElementAt(0, 1),
                m.getElementAt(0, 2));
        
        Point2D point2 = new HomogeneousPoint2D(m.getElementAt(1, 0),
                m.getElementAt(1, 1),
                m.getElementAt(1, 2));
        
        point1.normalize();
        point2.normalize();
        
        //point1 and point2 belong to line, hence their distance is zero up to
        //machine precision
        assertEquals(line.signedDistance(point1), 0.0, 
                PRECISION_ERROR);
        assertEquals(line.signedDistance(point2), 0.0, 
                PRECISION_ERROR);        
        
        //use line director vector to find another point at desired signed 
        //distance
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double signedDistance = randomizer.nextDouble(MIN_RANDOM_DISTANCE, 
                MAX_RANDOM_DISTANCE);
        
        HomogeneousPoint2D point3 = new HomogeneousPoint2D(point1);

        double normDirectorVector = Math.sqrt(line.getA() * line.getA() +
                line.getB() * line.getB());        
        point3.setInhomogeneousCoordinates(point3.getInhomX() + 
                signedDistance * line.getA() / normDirectorVector, 
                point3.getInhomY() + signedDistance * line.getB() / 
                normDirectorVector);
        
        assertEquals(line.signedDistance(point3), 
                signedDistance, PRECISION_ERROR);
    }
    
    @Test
    public void testAsArray(){
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        double a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        Line2D line = new Line2D(a, b, c);
        double[] array = line.asArray();
        
        assertEquals(array[0], a, 0.0);
        assertEquals(array[1], b, 0.0);
        assertEquals(array[2], c, 0.0);
        
        a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        line.setParameters(a, b, c);
        line.asArray(array);
        
        assertEquals(array[0], a, 0.0);
        assertEquals(array[1], b, 0.0);
        assertEquals(array[2], c, 0.0);

        
        //Force IllegalArgumentException
        try{
            line.asArray(new double[HOM_COORDS + 1]);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
    }
    
    @Test
    public void testNormalize(){
        
        double[] array = new double[HOM_COORDS];
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        Line2D line = new Line2D(array);
        assertFalse(line.isNormalized());
        
        //normalize line
        line.normalize();
        assertTrue(line.isNormalized());
        
        //return line as array
        double[] array2 = line.asArray();
        
        //compare that both arrays are equal up to scale
        //check correctness of obtained line
        double scaleA = array[0] / array2[0];
        double scaleB = array[1] / array2[1];
        double scaleC = array[2] / array2[2];
        
        assertEquals(scaleA, scaleB, PRECISION_ERROR);
        assertEquals(scaleB, scaleC, PRECISION_ERROR);
        assertEquals(scaleC, scaleA, PRECISION_ERROR);
        
        
        //if we provide zero values, then normalization has no effect
        line = new Line2D(0.0, 0.0, 0.0);
        
        assertFalse(line.isNormalized());
        
        line.normalize();
        
        assertFalse(line.isNormalized());
        
    }
    
    @Test
    public void testDirectorVector(){
        double[] array = new double[HOM_COORDS];
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        Line2D line = new Line2D(array);
        
        double[] n = line.getDirectorVector();
        
        assertEquals(n.length, 2);
        assertEquals(n[0], array[0], 0.0);
        assertEquals(n[1], array[1], 0.0);
        
        //try again
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        line.setParameters(array);
        
        line.directorVector(n);
        
        assertEquals(n.length, 2);
        assertEquals(n[0], array[0], 0.0);
        assertEquals(n[1], array[1], 0.0);        
    }
    
    @Test
    public void testIntersection() throws WrongSizeException, 
        DecomposerException,
        NoIntersectionException{
        
        Point2D point1 = new HomogeneousPoint2D();
        Point2D point2 = new HomogeneousPoint2D();
        Point2D point3 = new HomogeneousPoint2D();
        
        //Create two intersecting lines using 3 points
        Matrix m1 = Matrix.createWithUniformRandomValues(2, HOM_COORDS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        Matrix m2 = Matrix.createWithUniformRandomValues(2, HOM_COORDS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        //set 1st point in common in both matrices
        m2.setSubmatrix(0, 0, 0, HOM_COORDS - 1, 
                m1.getSubmatrix(0, 0, 0, HOM_COORDS - 1));
        
        //ensure that both matrices have rank 2 (points are not coincident)
        while(com.irurueta.algebra.Utils.rank(m1) < 2 || 
                com.irurueta.algebra.Utils.rank(m2) < 2){

            //create random matrices again
            m1 = Matrix.createWithUniformRandomValues(2, HOM_COORDS,
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            m2 = Matrix.createWithUniformRandomValues(2, HOM_COORDS,
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            //set 1st point in common in both matrices
            m2.setSubmatrix(0, 0, 0, HOM_COORDS - 1, 
                    m1.getSubmatrix(0, 0, 0, HOM_COORDS - 1));            
        }
        
        point1.setHomogeneousCoordinates(m1.getElementAt(0, 0), 
                m1.getElementAt(0, 1), m1.getElementAt(0, 2));
        point2.setHomogeneousCoordinates(m1.getElementAt(1, 0), 
                m1.getElementAt(1, 1), m1.getElementAt(1, 2));
        point3.setHomogeneousCoordinates(m2.getElementAt(1, 0), 
                m2.getElementAt(1, 1), m2.getElementAt(1, 2));
        
        //Create two lines between point1-point2, and point1-point3
        Line2D line1 = new Line2D(point1, point2);
        Line2D line2 = new Line2D(point1, point3);
        
        //because both lines have in common point1, then their intersection must
        //be point1
        
        Point2D intersection1 = line1.getIntersection(line2);
        Point2D intersection2 = line2.getIntersection(line1);
        
        Point2D intersection3 = Point2D.create();
        line1.intersection(line2, intersection3);
        
        Point2D intersection4 = Point2D.create();
        line2.intersection(line1, intersection4);
        
        //check correctness
        assertTrue(intersection1.equals(point1, ABSOLUTE_ERROR));
        assertTrue(intersection2.equals(point1, ABSOLUTE_ERROR));
        assertTrue(intersection3.equals(point1, ABSOLUTE_ERROR));
        assertTrue(intersection4.equals(point1, ABSOLUTE_ERROR));
        
        //Force NoIntersectionException (by attempting to intersect a line with
        //itself
        intersection1 = null;
        try{
            intersection1 = line1.getIntersection(line1);
            fail("NoIntersectionException expected but not thrown");
        }catch(NoIntersectionException e){}
        assertNull(intersection1);
        try{
            line1.intersection(line1, intersection2);
            fail("NoIntersectionException expected but not thrown");
        }catch(NoIntersectionException e){}        
    }
    
    @Test
    public void testClosestPoint() throws WrongSizeException, 
        NotReadyException, LockedException, DecomposerException, 
        com.irurueta.algebra.NotAvailableException{
        
        //randomly choose 2 points to find their corresponding line
        Matrix m = Matrix.createWithUniformRandomValues(2, HOM_COORDS, 
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        SingularValueDecomposer decomposer = new SingularValueDecomposer(m);
        decomposer.decompose();
        
        while(decomposer.getRank() < 2){
            m = Matrix.createWithUniformRandomValues(2, HOM_COORDS, 
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            decomposer.setInputMatrix(m);
            decomposer.decompose();
        }
        
        Matrix V = decomposer.getV();
                
        Point2D point1 = new HomogeneousPoint2D(m.getElementAt(0, 0),
                m.getElementAt(0, 1),
                m.getElementAt(0, 2));
        
        Point2D point2 = new HomogeneousPoint2D(m.getElementAt(1, 0),
                m.getElementAt(1, 1),
                m.getElementAt(1, 2));
        
        Line2D line = new Line2D(point1, point2);
        
        
        //point1 and point2 belong to line, hence their distance is zero up to
        //machine precision
        assertEquals(line.signedDistance(point1), 0.0, 
                PRECISION_ERROR);
        assertEquals(line.signedDistance(point2), 0.0, 
                PRECISION_ERROR);        
        //because they belong to line, their closest point to line are 
        //themselves
        Point2D closestPoint1 = line.getClosestPoint(point1);
        Point2D closestPoint1b = line.getClosestPoint(point1, PRECISION_ERROR);
        Point2D closestPoint1c = Point2D.create();
        line.closestPoint(point1, closestPoint1c);
        Point2D closestPoint1d = Point2D.create();
        line.closestPoint(point1, closestPoint1d, ABSOLUTE_ERROR);
        
        assertTrue(closestPoint1.equals(point1, ABSOLUTE_ERROR));
        assertTrue(closestPoint1b.equals(point1, ABSOLUTE_ERROR));
        assertTrue(closestPoint1c.equals(point1, ABSOLUTE_ERROR));
        assertTrue(closestPoint1d.equals(point1, ABSOLUTE_ERROR));
        
        
        Point2D closestPoint2 = line.getClosestPoint(point2);
        Point2D closestPoint2b = line.getClosestPoint(point2, PRECISION_ERROR);
        Point2D closestPoint2c = Point2D.create();
        line.closestPoint(point2, closestPoint2c);
        Point2D closestPoint2d = Point2D.create();
        line.closestPoint(point2, closestPoint2d, ABSOLUTE_ERROR);
        
        assertTrue(closestPoint2.equals(point2, ABSOLUTE_ERROR));
        assertTrue(closestPoint2b.equals(point2, ABSOLUTE_ERROR));
        assertTrue(closestPoint2c.equals(point2, ABSOLUTE_ERROR));
        assertTrue(closestPoint2d.equals(point2, ABSOLUTE_ERROR));

        
        //use line director vector to find another point at desired signed 
        //distance
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double signedDistance = randomizer.nextDouble(MIN_RANDOM_DISTANCE, 
                MAX_RANDOM_DISTANCE);
        
        HomogeneousPoint2D point3 = new HomogeneousPoint2D();

        double normDirectorVector = Math.sqrt(line.getA() * line.getA() +
                line.getB() * line.getB());        
        point3.setInhomogeneousCoordinates(point1.getInhomX() + 
                signedDistance * line.getA() / normDirectorVector, 
                point1.getInhomY() + signedDistance * line.getB() / 
                normDirectorVector);
        
        assertEquals(line.signedDistance(point3), 
                signedDistance, PRECISION_ERROR);
        
        //because point3 goes in line's perpendicular direction from point1, its
        //closest point belonging to the line will be point1
        Point2D closestPoint = line.getClosestPoint(point3);
        Point2D closestPointB = line.getClosestPoint(point3, PRECISION_ERROR);
        Point2D closestPointC = Point2D.create();
        line.closestPoint(point3, closestPointC);
        Point2D closestPointD = Point2D.create();
        line.closestPoint(point3, closestPointD, ABSOLUTE_ERROR);
        
        assertTrue(closestPoint.equals(point1, ABSOLUTE_ERROR));
        assertTrue(closestPointB.equals(point1, ABSOLUTE_ERROR));
        assertTrue(closestPointC.equals(point1, ABSOLUTE_ERROR));
        assertTrue(closestPointD.equals(point1, ABSOLUTE_ERROR));
        
        try{
            line.getClosestPoint(point3, -PRECISION_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            line.closestPoint(point3, closestPointD, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
    }    
    
    @Test
    public void testDotProduct(){
        double[] array = new double[HOM_COORDS];
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        //a random line
        Line2D line1 = new Line2D(array);
        
        //oposed sign line
        Line2D line2 = new Line2D(-line1.getA(), -line1.getB(), -line1.getC());
        
        //another random line
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        Line2D line3 = new Line2D(array);
        
        //test for equal lines
        assertEquals(line1.dotProduct(line1), 1.0, ABSOLUTE_ERROR);
        //test for oposed sign lines
        assertEquals(line1.dotProduct(line2), -1.0, ABSOLUTE_ERROR);
        //test for 2 random lines
        assertEquals(line1.dotProduct(line3), line1.getA() * line3.getA() +
                line1.getB() * line3.getB() + line1.getC() * line3.getC(), 
                ABSOLUTE_ERROR);
    }
    
    @Test
    public void testEquals(){
        double[] array = new double[HOM_COORDS];
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        Line2D line1 = new Line2D(array);
        Line2D line2 = new Line2D(array);
        assertTrue(line1.equals(line2, ABSOLUTE_ERROR));
        assertTrue(line1.equals(line2));
        assertTrue(line1.equals((Object)line2));
        
        array[0] = line1.getA() + randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        array[1] = line1.getB();
        array[2] = line1.getC();
        
        line2 = new Line2D(array);
        assertFalse(line1.equals(line2, 0.0));
        
        array[0] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        array[1] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        array[2] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        line1 = new Line2D(array);
        array[0] *= 2.0;
        line2 = new Line2D(array);
        assertTrue(line1.equals(line2, 2.0));
        assertFalse(line1.equals(line2, 0.0));
        
        //Force IllegalArgumentException
        try{
            line1.equals(line1, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException but not thrown");
        }catch(IllegalArgumentException e){}
    }
    
    @Test
    public void testCreateCanonicalLineAtInfinity(){
        Line2D line = Line2D.createCanonicalLineAtInfinity();
        
        assertEquals(line.getA(), 0.0, 0.0);
        assertEquals(line.getB(), 0.0, 0.0);
        assertEquals(line.getC(), 1.0, 0.0);
        
        //create a point at infinity
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        Point2D point = new HomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 0.0);
        
        //check that point at infinity is locus of line
        assertTrue(line.isLocus(point));
    }
    
    @Test
    public void testSetAsCanonicalLineAtInfinity(){
        //create a point at infinity
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        Point2D point = new HomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 0.0);

        Line2D line = new Line2D(1.0, 1.0, 1.0);
        
        //initially line is not at infinity
        assertFalse(line.isLocus(point));
        
        //set line at infinity
        Line2D.setAsCanonicalLineAtInfinity(line);
        
        //check correctness
        assertEquals(line.getA(), 0.0, 0.0);
        assertEquals(line.getB(), 0.0, 0.0);
        assertEquals(line.getC(), 1.0, 0.0);
        
        //check that point at infinity is locus of line
        assertTrue(line.isLocus(point));        
    }
}