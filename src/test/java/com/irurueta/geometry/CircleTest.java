/**
 * @file
 * This file contains Unit Tests for
 * com.irurueta.geometry.Circle
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date September 14, 2012
 */
package com.irurueta.geometry;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.statistics.UniformRandomizer;
import java.util.Random;
import org.junit.After;
import org.junit.AfterClass;
import static org.junit.Assert.*;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

public class CircleTest {
    
    public static final double ABSOLUTE_ERROR = 1e-6;
    public static final double MIN_RANDOM_VALUE = -100.0;
    public static final double MAX_RANDOM_VALUE = 100.0;
    
    public static final double MIN_RANDOM_DEGREES = -180.0;
    public static final double MAX_RANDOM_DEGREES = 180.0;
    
    public static final int TIMES = 100;
    
    public CircleTest() { }
    
    @BeforeClass
    public static void setUpClass() { }
    
    @AfterClass
    public static void tearDownClass() { }
    
    @Before
    public void setUp() { }
    
    @After
    public void tearDown() { }
    
    @Test
    public void testConstructor() throws ColinearPointsException{
        //Test empty constructor
        Circle circle = new Circle();
        
        //check center is at origin and radius is 1.0
        assertTrue(circle.getCenter().equals(new InhomogeneousPoint2D(0.0, 0.0), 
                ABSOLUTE_ERROR));
        assertEquals(circle.getRadius(), 1.0, ABSOLUTE_ERROR);
        
        //Test constructor with center and radius
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        Point2D center = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        double radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE));
        
        circle = new Circle(center, radius);
        //check correctness
        assertTrue(circle.getCenter().equals(center, ABSOLUTE_ERROR));
        assertEquals(circle.getRadius(), radius, ABSOLUTE_ERROR);
        
        //Force IllegalArgumentException
        circle = null;
        try{
            circle = new Circle(center, -radius);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(circle); 
        
        //test constructor with three points
        
        //pick 3 points belonging to the circle locus
        Point2D point1, point2, point3;
        boolean areEqual;
        do{
            double angle = randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                    MAX_RANDOM_DEGREES) * Math.PI / 180.0;
            point1 = new HomogeneousPoint2D(
                    center.getInhomX() + radius * Math.cos(angle),
                    center.getInhomY() + radius * Math.sin(angle), 1.0);
            angle = randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                    MAX_RANDOM_DEGREES) * Math.PI / 180.0;
            point2 = new HomogeneousPoint2D(
                    center.getInhomX() + radius * Math.cos(angle),
                    center.getInhomY() + radius * Math.sin(angle), 1.0);
            angle = randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                    MAX_RANDOM_DEGREES) * Math.PI / 180.0;
            point3 = new HomogeneousPoint2D(
                    center.getInhomX() + radius * Math.cos(angle),
                    center.getInhomY() + radius * Math.sin(angle), 1.0);

            //ensure that all three points are different
            areEqual = point1.equals(point2, ABSOLUTE_ERROR) ||
                    point2.equals(point3, ABSOLUTE_ERROR) ||
                    point3.equals(point1, ABSOLUTE_ERROR);
        }while(areEqual);
        
        
        //compute circle
        Circle circle2 = new Circle(point1, point2, point3);
        
        //check that both circles are equal
        circle = new Circle(center, radius);
        assertEquals(circle.getCenter().distanceTo(circle2.getCenter()), 0.0, 
                ABSOLUTE_ERROR);
        assertEquals(circle.getRadius(), circle2.getRadius(), ABSOLUTE_ERROR);
        
        //Force ColinearPointsException
        circle = null;
        try{
            circle = new Circle(point1, point2, point2);
            fail("ColinearPointsException expected but not thrown");
        }catch(ColinearPointsException e){}
        assertNull(circle);
        
        //test from conic
        circle = new Circle(center, radius);
        Conic conic = circle.toConic();
        circle2 = new Circle(conic);
        assertEquals(circle.getCenter().distanceTo(circle2.getCenter()), 0.0, 
                ABSOLUTE_ERROR);
        assertEquals(circle.getRadius(), circle2.getRadius(), ABSOLUTE_ERROR);
        
        //Force IllegalArgumentException
        conic = new Conic();
        circle = null;
        try{
            circle = new Circle(conic);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(circle);
    }
    
    @Test
    public void testGetSetCenter(){
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        Point2D center = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        
        Circle circle = new Circle();
        //check center
        assertTrue(circle.getCenter().equals(new InhomogeneousPoint2D(0.0, 0.0), 
                ABSOLUTE_ERROR));
        
        //set center
        circle.setCenter(center);
        //check correctness
        assertTrue(circle.getCenter().equals(center, ABSOLUTE_ERROR));
        
        //Force NullPointerException
        try{
            circle.setCenter(null);
            fail("NullPointerException expected but not thrown");
        }catch(NullPointerException e){}
    }
    
    @Test
    public void testGetSetRadius(){
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE));
        
        Circle circle = new Circle();
        //check radius
        assertEquals(circle.getRadius(), 1.0, 0.0);
        
        //set radius
        circle.setRadius(radius);
        //check correctness
        assertEquals(circle.getRadius(), radius, 0.0);
        
        //Force IllegalArgumentException
        try{
            circle.setRadius(-radius);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
    }
    
    @Test
    public void testSetCenterAndRadius(){
        //Test constructor with center and radius
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        Point2D center = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        double radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE));
              
        Circle circle = new Circle();
        //check center
        assertTrue(circle.getCenter().equals(new InhomogeneousPoint2D(0.0, 0.0), 
                ABSOLUTE_ERROR));    
        //check radius
        assertEquals(circle.getRadius(), 1.0, 0.0);
        
        //set center and radius
        circle.setCenterAndRadius(center, radius);
        //check correctness
        assertTrue(circle.getCenter().equals(center, ABSOLUTE_ERROR));
        assertEquals(circle.getRadius(), radius, 0.0);
        
        //Force IllegalArgumentException
        try{
            circle.setCenterAndRadius(center, -radius);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        
        //Force NullPointerException
        try{
            circle.setCenterAndRadius(null, radius);
            fail("IllegalArgumentException expected but not thrown");
        }catch(NullPointerException e){}
    }
    
    @Test
    public void testSetParametersFromPoints() throws ColinearPointsException{
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        Point2D center = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        double radius = Math.abs(randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, 
                MAX_RANDOM_VALUE));
        
        Circle circle1 = new Circle(center, radius);
        
        //pick 3 points belonging to the circle locus
        Point2D point1, point2, point3;
        boolean areEqual;
        do{
            double angle = randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                    MAX_RANDOM_DEGREES) * Math.PI / 180.0;
            point1 = new HomogeneousPoint2D(
                    center.getInhomX() + radius * Math.cos(angle),
                    center.getInhomY() + radius * Math.sin(angle), 1.0);
            angle = randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                    MAX_RANDOM_DEGREES) * Math.PI / 180.0;
            point2 = new HomogeneousPoint2D(
                    center.getInhomX() + radius * Math.cos(angle),
                    center.getInhomY() + radius * Math.sin(angle), 1.0);
            angle = randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                    MAX_RANDOM_DEGREES) * Math.PI / 180.0;
            point3 = new HomogeneousPoint2D(
                    center.getInhomX() + radius * Math.cos(angle),
                    center.getInhomY() + radius * Math.sin(angle), 1.0);

            //ensure that all three points are different
            areEqual = point1.equals(point2, ABSOLUTE_ERROR) ||
                    point2.equals(point3, ABSOLUTE_ERROR) ||
                    point3.equals(point1, ABSOLUTE_ERROR);
        }while(areEqual);
        
        //create new circle and set parameters
        Circle circle2 = new Circle();
        
        circle2.setParametersFromPoints(point1, point2, point3);
        
        //check that both circles are equal
        assertEquals(circle1.getCenter().distanceTo(circle2.getCenter()), 0.0, 
                ABSOLUTE_ERROR);
        assertEquals(circle1.getRadius(), circle2.getRadius(), ABSOLUTE_ERROR);
        
        //Force ColinearPointsException
        try{
            circle2.setParametersFromPoints(point1, point2, point2);
            fail("ColinearPointsException expected but not thrown");
        }catch(ColinearPointsException e){}
    }
    
    @Test
    public void testArea(){
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        Point2D center = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        double radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE));

        Circle circle = new Circle(center, radius);
        
        double area = Math.PI * radius * radius;
        
        //Check correctness
        assertEquals(circle.getArea(), area, ABSOLUTE_ERROR);
        assertEquals(Circle.area(radius), area, ABSOLUTE_ERROR);
    }
    
    @Test
    public void testPerimeter(){
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        Point2D center = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        double radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE));

        Circle circle = new Circle(center, radius);
        
        double perimeter = 2.0 * Math.PI * radius;
        
        //Check correctness
        assertEquals(circle.getPerimeter(), perimeter, ABSOLUTE_ERROR);
        assertEquals(Circle.perimeter(radius), perimeter, ABSOLUTE_ERROR);        
    }
    
    @Test
    public void testCurvature() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        Point2D center = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        double radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE));

        Circle circle = new Circle(center, radius);
        
        assertEquals(Circle.curvature(radius), 1.0 / radius, ABSOLUTE_ERROR);
        assertEquals(circle.getCurvature(), 1.0 / radius, ABSOLUTE_ERROR);
    }
    
    @Test
    public void testIsInside(){
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        Point2D center = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        double radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE));
        double value = randomizer.nextDouble(0.2, 0.8);
        double value2 = 1.0 + value;
        double theta = randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                MAX_RANDOM_DEGREES) * Math.PI / 180.0;

        Circle circle = new Circle(center, radius);

        Point2D inside = new InhomogeneousPoint2D(
                center.getInhomX() + value * radius * Math.cos(theta),
                center.getInhomY() + value * radius * Math.sin(theta));
        Point2D outside = new InhomogeneousPoint2D(
                center.getInhomX() + value2 * radius * Math.cos(theta),
                center.getInhomY() + value2 * radius * Math.sin(theta));
        
        //check correctness
        assertTrue(circle.isInside(inside));
        assertTrue(circle.isInside(inside, ABSOLUTE_ERROR));        
        
        assertFalse(circle.isInside(outside));
        assertFalse(circle.isInside(outside, ABSOLUTE_ERROR));
        
        //test for a large positive threshold 
        assertTrue(circle.isInside(inside, radius));
        assertTrue(circle.isInside(outside, radius));
        
        assertFalse(circle.isInside(inside, -radius));
        assertFalse(circle.isInside(outside, -radius));        
    }
    
    @Test
    public void testSignedDistanceDistanceAndIsLocus(){
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        Point2D center = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        double radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE));
        double value = randomizer.nextDouble(0.2, 0.8);
        double value2 = 1.0 + value;
        double theta = randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                MAX_RANDOM_DEGREES) * Math.PI / 180.0;

        Circle circle = new Circle(center, radius);
        
        //center is not locus
        assertFalse(circle.isLocus(center));
        assertFalse(circle.isLocus(center, ABSOLUTE_ERROR));
        //but for a large enough threshold it might be
        assertTrue(circle.isLocus(center, 2.0 * radius));

        Point2D inside = new InhomogeneousPoint2D(
                center.getInhomX() + value * radius * Math.cos(theta),
                center.getInhomY() + value * radius * Math.sin(theta));
        Point2D outside = new InhomogeneousPoint2D(
                center.getInhomX() + value2 * radius * Math.cos(theta),
                center.getInhomY() + value2 * radius * Math.sin(theta));
        Point2D zero = new InhomogeneousPoint2D(
                center.getInhomX() + radius * Math.cos(theta),
                center.getInhomY() + radius * Math.sin(theta));
        
        //check correctness
        assertEquals(circle.getSignedDistance(inside), (value - 1.0) * radius, 
                ABSOLUTE_ERROR);
        assertEquals(Circle.signedDistance(circle, inside), 
                (value - 1.0) * radius, ABSOLUTE_ERROR);
        
        assertEquals(circle.getDistance(inside), 
                Math.abs((value - 1.0) * radius), ABSOLUTE_ERROR);
        assertEquals(Circle.distance(circle, inside), 
                Math.abs((value - 1.0) * radius), ABSOLUTE_ERROR);
                
        //for inside point signed distance is negative        
        assertTrue(circle.getSignedDistance(inside) <= 0.0);
        
        assertFalse(circle.isLocus(inside));
        assertFalse(circle.isLocus(inside, ABSOLUTE_ERROR));
        assertTrue(circle.isLocus(inside, radius)); //true for a large anough 
                                                    //threshold        
        
        assertEquals(circle.getSignedDistance(outside), (value2 - 1.0) * radius, 
                ABSOLUTE_ERROR);
        assertEquals(Circle.signedDistance(circle, outside), 
                (value2 - 1.0) * radius, ABSOLUTE_ERROR);
        
        assertEquals(circle.getDistance(outside), 
                Math.abs((value2 - 1.0) * radius), ABSOLUTE_ERROR);
        assertEquals(Circle.distance(circle, outside), 
                Math.abs((value2 - 1.0) * radius), ABSOLUTE_ERROR);
        
        
        //for outside point distance is positive
        assertTrue(circle.getSignedDistance(outside) >= 0.0);

        assertFalse(circle.isLocus(outside));
        assertFalse(circle.isLocus(outside, ABSOLUTE_ERROR));
        assertTrue(circle.isLocus(outside, radius)); //true for a large anough 
                                                    //threshold
        
        
        //for point at locus of circle, distance is zero
        assertEquals(circle.getSignedDistance(zero), 0.0, ABSOLUTE_ERROR);
        assertEquals(Circle.signedDistance(circle, zero), 0.0, ABSOLUTE_ERROR);
        
        //zero is locus
        assertTrue(circle.isLocus(zero));
        assertTrue(circle.isLocus(zero, radius));
        
        //Force IllegalArgumentExcepetion
        try{
            circle.isLocus(zero, -radius);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
    }
    
    @Test
    public void testClosestPointAndIsLocus() throws UndefinedPointException{
       UniformRandomizer randomizer = new UniformRandomizer(new Random());
        Point2D center = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        double radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE));
        double value = randomizer.nextDouble(0.2, 0.8);
        double value2 = 1.0 + value;
        double theta = randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                MAX_RANDOM_DEGREES) * Math.PI / 180.0;

        Circle circle = new Circle(center, radius);
        
        //center is not locus
        assertFalse(circle.isLocus(center));
        assertFalse(circle.isLocus(center, ABSOLUTE_ERROR));
        //but for a large enough threshold it might be
        assertTrue(circle.isLocus(center, 2.0 * radius));
        

        Point2D inside = new InhomogeneousPoint2D(
                center.getInhomX() + value * radius * Math.cos(theta),
                center.getInhomY() + value * radius * Math.sin(theta));
        Point2D outside = new InhomogeneousPoint2D(
                center.getInhomX() + value2 * radius * Math.cos(theta),
                center.getInhomY() + value2 * radius * Math.sin(theta));
        Point2D zero = new InhomogeneousPoint2D(
                center.getInhomX() + radius * Math.cos(theta),
                center.getInhomY() + radius * Math.sin(theta));     
        
        Point2D expectedPoint = new InhomogeneousPoint2D(zero);
        
        Point2D result = Point2D.create();
        
        //test for point inside (but far from center)
        assertTrue(circle.getClosestPoint(inside).equals(expectedPoint, 
                ABSOLUTE_ERROR));
        circle.closestPoint(inside, result);
        assertTrue(result.equals(expectedPoint, ABSOLUTE_ERROR));
        
        assertFalse(circle.isLocus(inside));
        assertFalse(circle.isLocus(inside, ABSOLUTE_ERROR));
        assertTrue(circle.isLocus(inside, radius)); //true for a large anough 
                                                    //threshold        
        
        
        //test for point outside of circle
        assertTrue(circle.getClosestPoint(outside).equals(expectedPoint, 
                ABSOLUTE_ERROR));
        circle.closestPoint(outside, result);
        assertTrue(result.equals(expectedPoint, ABSOLUTE_ERROR));
        
        assertFalse(circle.isLocus(outside));
        assertFalse(circle.isLocus(outside, ABSOLUTE_ERROR));
        assertTrue(circle.isLocus(outside, radius)); //true for a large anough 
                                                    //threshold
        
        
        //test for point in circle boundary
        assertTrue(circle.getClosestPoint(zero).equals(expectedPoint, 
                ABSOLUTE_ERROR));
        circle.closestPoint(zero, result);
        assertTrue(result.equals(expectedPoint, ABSOLUTE_ERROR));
        
        //zero is locus
        assertTrue(circle.isLocus(zero));
        assertTrue(circle.isLocus(zero, radius));
        
        
        //Force UndefinedPointException (by testing at center)
        try{
            circle.getClosestPoint(center);
            fail("UndefinedPointException expected but not thrown");
        }catch(UndefinedPointException e){}
        try{
            circle.closestPoint(center, result);
            fail("UndefinedPointException expected but not thrown");
        }catch(UndefinedPointException e){}
        
        //Force IllegalArgumentExcepetion
        try{
            circle.isLocus(zero, -radius);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}        
    }
    
    @Test
    public void testGetTangentLineAt() throws NotLocusException{
        for(int t = 0; t < TIMES; t++){
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            Point2D center = new HomogeneousPoint2D(randomizer.nextDouble(
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 1.0);
            double radius = Math.abs(randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, 
                    MAX_RANDOM_VALUE));
            double theta = randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                    MAX_RANDOM_DEGREES) * Math.PI / 180.0;
            double theta2; //angle corresponding to line slope
            if(theta > Math.PI / 2.0){
                theta2 = theta - Math.PI;
            }else if(theta < -Math.PI / 2.0){
                theta2 = theta + Math.PI;
            }else{
                theta2 = theta;
            }

            Circle circle = new Circle(center, radius);

            Point2D point = new HomogeneousPoint2D(
                    center.getInhomX() + radius * Math.cos(theta),
                    center.getInhomY() + radius * Math.sin(theta), 1.0); 
            point.normalize();

            assertTrue(circle.isLocus(point));

            //find tangent line at locus point
            Line2D line = circle.getTangentLineAt(point);

            //check that point is also at line's locus
            assertTrue(line.isLocus(point));

            //check that line angle is equal to theta
            double lineAngle = line.getAngle();
            double theta3 = theta2 - Math.PI / 2.0;
            if(theta3 < -Math.PI / 2.0){
                theta3 += Math.PI;
            }else if(theta3 > Math.PI / 2.0){
                theta3 -= Math.PI;
            }
            assertEquals(lineAngle * 180.0 / Math.PI, 
                    theta3 * 180.0 / Math.PI, ABSOLUTE_ERROR);
        }
    }
    
    @Test
    public void testToConic() throws WrongSizeException{
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        Point2D center = new HomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 1.0);
        double radius = Math.abs(randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, 
                MAX_RANDOM_VALUE));
        double theta = randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                MAX_RANDOM_DEGREES) * Math.PI / 180.0;
        center.normalize();

        Circle circle = new Circle(center, radius);
        Conic conic = circle.toConic();
        
        //center is not locus
        assertFalse(circle.isLocus(center));
        assertFalse(circle.isLocus(center, ABSOLUTE_ERROR));
        assertFalse(conic.isLocus(center));
        assertFalse(conic.isLocus(center, ABSOLUTE_ERROR));
        //but for a large enough threshold it might be
        assertTrue(circle.isLocus(center, 2.0 * radius));
        assertTrue(conic.isLocus(center, 2.0 * radius));


        Point2D locus = new HomogeneousPoint2D(
                center.getInhomX() + radius * Math.cos(theta),
                center.getInhomY() + radius * Math.sin(theta), 1.0); 
        locus.normalize();
        
        //test is locus
        assertTrue(circle.isLocus(locus, ABSOLUTE_ERROR));
        assertTrue(conic.isLocus(locus, ABSOLUTE_ERROR));
        
        //check correctness of estimated conic matrix (up to scale)
        Matrix m = new Matrix(Conic.BASECONIC_MATRIX_ROW_SIZE,
                Conic.BASECONIC_MATRIX_COLUMN_SIZE);
        m.setElementAt(0, 0, 1.0);
        m.setElementAt(1, 0, 0.0);
        m.setElementAt(2, 0, -center.getInhomX());
        
        m.setElementAt(0, 1, 0);
        m.setElementAt(1, 1, 1.0);
        m.setElementAt(2, 1, -center.getInhomY());
        
        m.setElementAt(0, 2, -center.getInhomX());
        m.setElementAt(1, 2, -center.getInhomY());
        m.setElementAt(2, 2, center.getInhomX() * center.getInhomX() +
                center.getInhomY() * center.getInhomY() - radius * radius);
        
        double norm = Utils.normF(m);
        m.multiplyByScalar(1.0 / norm);
        
        assertTrue(m.equals(conic.asMatrix(), ABSOLUTE_ERROR));
    }
    
    @Test
    public void testSetFromConic(){
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        Point2D center = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        double radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE));
        
        Circle circle = new Circle(center, radius);        
        
        //test from conic
        Conic conic = circle.toConic();
        Circle circle2 = new Circle();
        circle2.setFromConic(conic);
        
        //check correctness
        assertEquals(circle.getCenter().distanceTo(circle2.getCenter()), 0.0, 
                ABSOLUTE_ERROR);
        assertEquals(circle.getRadius(), circle2.getRadius(), ABSOLUTE_ERROR);
        
        //Force IllegalArgumentException
        conic = new Conic();
        try{
            circle2.setFromConic(conic);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
    }
}
